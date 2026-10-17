#include "gimbal.hpp"

#include <algorithm>
#include <array>
#include <cstring>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path, bool wait_for_rx)
{
  auto yaml = tools::load(config_path);
  uart_mode_ = yaml["use_can"] && !yaml["use_can"].as<bool>();

  if (uart_mode_) {
    auto serial_yaml = yaml["serial_imu"];
    uart44_mode_ = serial_yaml && serial_yaml["protocol"] &&
                   serial_yaml["protocol"].as<std::string>() == "uart44";
    uart_mode_value_ = parse_uart_mode(yaml);
    if (yaml["uart_bullet_speed"]) uart_bullet_speed_ = yaml["uart_bullet_speed"].as<double>();
    if (!std::isfinite(uart_bullet_speed_) || uart_bullet_speed_ < 0 || uart_bullet_speed_ > 100)
      throw std::invalid_argument("Invalid uart_bullet_speed");
    mode_ = uart_mode_value_;
    state_.bullet_speed = static_cast<float>(uart_bullet_speed_);
    if (!uart44_mode_) {
      if (!wait_for_rx)
        throw std::invalid_argument("UART latency test requires serial_imu.protocol: uart44");
      uart_transport_ = std::make_unique<CBoard>(config_path);
      tools::logger()->info("[Gimbal] Using configured CBoard UART protocol");
      return;
    }
    for (const char * key : {"frame_size", "frame_stride"})
      if (serial_yaml[key] && serial_yaml[key].as<size_t>() != 44)
        throw std::invalid_argument("Gimbal uart44 requires 44-byte size/stride");
    if (serial_yaml["float_endian"] && serial_yaml["float_endian"].as<std::string>() != "little")
      throw std::invalid_argument("Gimbal uart44 firmware uses little-endian floats");
    auto velocity_unit = serial_yaml["velocity_unit"]
      ? serial_yaml["velocity_unit"].as<std::string>() : "degrees";
    if (velocity_unit != "degrees" && velocity_unit != "radians")
      throw std::invalid_argument("UART velocity_unit must be degrees or radians");
    uart_velocity_scale_ = velocity_unit == "degrees" ? 3.14159265358979323846 / 180 : 1;
    uart_angle_scale_ = !serial_yaml["angle_unit"] || serial_yaml["angle_unit"].as<std::string>() == "degrees"
      ? 3.14159265358979323846 / 180 : 1;
    uart_rx_protocol_ = std::make_unique<SerialIMUProtocol>(serial_yaml, 0);
    uart_tx_protocol_ = std::make_unique<SerialCommandProtocol>(serial_yaml["tx"]);
    if (uart_tx_protocol_->frame_size() != 44)
      throw std::invalid_argument("UART44 Gimbal requires serial_imu.tx.protocol: sp44");
    tools::logger()->info("[Gimbal] UART: SP Euler RX/TX=44 bytes, original field order; acceleration ignored");
  }

  auto com_port = uart44_mode_ ? tools::read<std::string>(yaml["serial_imu"], "port")
                             : tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    auto baud = uart44_mode_ ? yaml["serial_imu"]["baudrate"] : yaml["gimbal_baudrate"];
    serial_.setBaudrate(baud ? baud.as<uint32_t>() : 921600);
    // 等待完整帧，避免默认零超时在正常帧间隙中快速累计错误并反复重连。
    auto timeout = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    throw;
  }

  auto startup_ms = yaml["imu_timeout_ms"] ? yaml["imu_timeout_ms"].as<int>() : 3000;
  if (startup_ms <= 0) throw std::invalid_argument("imu_timeout_ms must be positive");
  thread_ = std::thread(&Gimbal::read_thread, this);
  if (wait_for_rx) {
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> first;
    if (!queue_.pop_for(first, std::chrono::milliseconds(startup_ms))) {
      quit_ = true;
      thread_.join();
      serial_.close();
      throw std::runtime_error("[Gimbal] Timed out waiting for a valid 44-byte SP frame");
    }
    tools::logger()->info("[Gimbal] First q received.");
  }
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  uart_transport_.reset();
  serial_.close();
}

GimbalMode Gimbal::parse_uart_mode(const YAML::Node & yaml)
{
  auto value = yaml["uart_mode"] ? yaml["uart_mode"].as<std::string>() : "auto_aim";
  if (value == "idle") return GimbalMode::IDLE;
  if (value == "auto_aim") return GimbalMode::AUTO_AIM;
  if (value == "small_buff") return GimbalMode::SMALL_BUFF;
  if (value == "big_buff") return GimbalMode::BIG_BUFF;
  throw std::invalid_argument("uart_mode must be idle, auto_aim, small_buff or big_buff");
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

GimbalCommunicationStats Gimbal::communication_stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return communication_stats_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  if (uart_transport_) {
    // 使用同一条有界等待、过期检测和重连接收链路；保留原 MPC 四元数接口。
    auto q = uart_transport_->imu_at(t);
    auto ypr = tools::eulers(q, 2, 1, 0);
    std::lock_guard<std::mutex> lock(mutex_);
    state_.yaw = ypr[0];
    state_.pitch = ypr[1];
    return q;
  }
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  if (uart_mode_) {
    send(
      VisionToGimbal.mode != 0, VisionToGimbal.mode == 2, VisionToGimbal.yaw, VisionToGimbal.yaw_vel,
      VisionToGimbal.yaw_acc, VisionToGimbal.pitch, VisionToGimbal.pitch_vel, VisionToGimbal.pitch_acc);
    return;
  }
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = 0;  // 通信测试保留 CRC 字段，但不计算校验。
  tx_data_.tail = 0x0d;

  write_command();
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  if (uart_mode_) {
    if (uart44_mode_) {
      io::Command command{control, fire, yaw, pitch, 0};
      command.yaw_vel = yaw_vel;
      command.yaw_acc = yaw_acc;
      command.pitch_vel = pitch_vel;
      command.pitch_acc = pitch_acc;
      auto bytes = uart_tx_protocol_->encode(command);
      write_command(bytes.data(), bytes.size());
      return;
    }
    try {
      uart_transport_->send(io::Command{control, fire, yaw, pitch, 0});
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write UART command: {}", e.what());
    }
    return;
  }
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = 0;  // 通信测试保留 CRC 字段，但不计算校验。
  tx_data_.tail = 0x0d;

  write_command();
}

void Gimbal::write_command(const uint8_t * data, size_t size)
{
  if (!data) data = reinterpret_cast<const uint8_t *>(&tx_data_);
  auto started = std::chrono::steady_clock::now();
  size_t written = 0;
  try {
    written = serial_.write(data, size);
    if (written != size)
      tools::logger()->warn("[Gimbal] Short serial write: {}/{} bytes", written, size);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
  auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  std::lock_guard<std::mutex> lock(mutex_);
  auto & stats = communication_stats_;
  stats.tx_attempts++;
  stats.tx_bytes += written;
  if (written != size) {
    stats.tx_errors++;
    return;
  }
  stats.tx_frames++;
  double delta = elapsed - stats.tx_write_mean_ms;
  stats.tx_write_mean_ms += delta / stats.tx_frames;
  stats.tx_write_m2 += delta * (elapsed - stats.tx_write_mean_ms);
  stats.tx_write_max_ms = std::max(stats.tx_write_max_ms, elapsed);
}


void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  std::array<uint8_t, sizeof(GimbalToVision)> buffer{};
  size_t buffered = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      buffered = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    auto read_completed = std::chrono::steady_clock::now();
    try {
      auto count = serial_.read(buffer.data() + buffered, buffer.size() - buffered);
      read_completed = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        communication_stats_.rx_bytes += count;
      }
      buffered += count;  // 超时返回的半帧也保留到下次读取。
      if (count == 0) {
        // 没有上行数据/应答不是串口故障，不能因此中断发送。
        continue;
      }
      error_count = 0;  // 有字节到达说明传输正常；协议错误只统计，不关闭串口影响TX。
    } catch (const std::exception &) {
      buffered = 0;
      error_count++;
      continue;
    }

    // 逐字节重同步，支持奇数长度噪声、半帧和错误帧尾；不验证 CRC。
    size_t discarded = 0, bad_tails = 0;
    while (buffered >= 2) {
      if (buffer[0] == 'S' && buffer[1] == 'P' &&
          (buffered < buffer.size() || buffer.back() == 0x0d)) break;
      if (buffer[0] == 'S' && buffer[1] == 'P' && buffered == buffer.size()) bad_tails++;
      std::memmove(buffer.data(), buffer.data() + 1, --buffered);
      discarded++;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      communication_stats_.rx_discarded_bytes += discarded;
      communication_stats_.rx_bad_tails += bad_tails;
    }
    if (buffered < buffer.size()) continue;
    std::memcpy(&rx_data_, buffer.data(), sizeof(rx_data_));
    buffered = 0;
    auto t = std::chrono::steady_clock::now();

    error_count = 0;
    Eigen::Quaterniond q;
    UARTGimbalToVision uart_frame{};
    if (uart44_mode_) {
      std::memcpy(&uart_frame, buffer.data(), sizeof(uart_frame));
      if (!uart_rx_protocol_->orientation(buffer.data(), buffer.size(), q) ||
          !std::isfinite(uart_frame.pitch_vel) || !std::isfinite(uart_frame.yaw_vel)) continue;
    } else {
      q = Eigen::Quaterniond(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    }
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    auto & stats = communication_stats_;
    stats.rx_frames++;
    if (stats.last_rx != std::chrono::steady_clock::time_point{}) {
      double interval = std::chrono::duration<double, std::milli>(read_completed - stats.last_rx).count();
      double delta = interval - stats.rx_interval_mean_ms;
      stats.rx_interval_mean_ms += delta / (stats.rx_frames - 1);
      stats.rx_interval_m2 += delta * (interval - stats.rx_interval_mean_ms);
      stats.rx_interval_max_ms = std::max(stats.rx_interval_max_ms, interval);
    }
    stats.last_rx = read_completed;

    if (uart44_mode_) {
      state_.yaw = uart_frame.yaw * uart_angle_scale_;
      state_.pitch = uart_frame.pitch * uart_angle_scale_;
      state_.yaw_vel = uart_frame.yaw_vel * uart_velocity_scale_;
      state_.pitch_vel = uart_frame.pitch_vel * uart_velocity_scale_;
      state_.bullet_speed = uart_bullet_speed_;
      state_.bullet_count = 0;
      mode_ = uart_mode_value_;
    } else {
      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;

      switch (rx_data_.mode) {
        case 0:
          mode_ = GimbalMode::IDLE;
          break;
        case 1:
          mode_ = GimbalMode::AUTO_AIM;
          break;
        case 2:
          mode_ = GimbalMode::SMALL_BUFF;
          break;
        case 3:
          mode_ = GimbalMode::BIG_BUFF;
          break;
        default:
          mode_ = GimbalMode::IDLE;
          tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
          break;
      }
    }
    double parse_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - read_completed).count();
    stats.rx_parse_mean_ms += (parse_ms - stats.rx_parse_mean_ms) / stats.rx_frames;
    stats.rx_parse_max_ms = std::max(stats.rx_parse_max_ms, parse_ms);
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
