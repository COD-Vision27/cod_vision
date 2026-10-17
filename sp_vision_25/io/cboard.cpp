#include "cboard.hpp"

#include <algorithm>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: bullet_speed(0),
  mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  ft_angle(0),
  queue_(5000),
  can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))
// 注意: callback的运行会早于Cboard构造函数的完成
{
  tools::logger()->info("[Cboard] Waiting for q...");
  auto deadline = std::chrono::steady_clock::now() + imu_timeout_;
  if (!queue_.pop_for(data_ahead_, imu_timeout_) ||
      !queue_.pop_for(data_behind_, deadline - std::chrono::steady_clock::now()))
    throw std::runtime_error("[CBoard] Timed out waiting for valid IMU frames");
  tools::logger()->info("[Cboard] Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  auto deadline = std::chrono::steady_clock::now() + imu_wait_;
  while (data_behind_.timestamp < timestamp) {
    data_ahead_ = data_behind_;
    if (!queue_.pop_for(data_behind_, deadline - std::chrono::steady_clock::now()))
      throw std::runtime_error("[CBoard] IMU receive timeout");
    if (std::chrono::steady_clock::now() >= deadline && data_behind_.timestamp < timestamp)
      throw std::runtime_error("[CBoard] IMU interpolation timeout");
  }

  if (std::chrono::steady_clock::now() - data_behind_.timestamp > imu_max_age_ ||
      data_ahead_.timestamp - timestamp > imu_max_age_)
    throw std::runtime_error("[CBoard] IMU data is stale or requested timestamp is too old");

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  if (t_ab.count() <= 0) return q_b;
  auto k = std::clamp(t_ac / t_ab, 0.0, 1.0);
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) const
{
  try {
    // 保留原始浮点值，由 SocketCAN 根据 use_can 选择 CAN/UART 编码。
    can_.write(command);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    if (frame.can_dlc != 8) return;
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({Eigen::Quaterniond(w, x, y, z).normalized(), timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    if (frame.can_dlc < 6 || frame.data[2] >= MODES.size() ||
        frame.data[3] >= SHOOT_MODES.size()) return;
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
YAML::Node CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  for (int id : {quaternion_canid_, bullet_speed_canid_, send_canid_}) {
    if (id < 0 || id > CAN_SFF_MASK) throw std::invalid_argument("Invalid CBoard CAN ID");
  }
  if (quaternion_canid_ == bullet_speed_canid_ || quaternion_canid_ == send_canid_ ||
      bullet_speed_canid_ == send_canid_) throw std::invalid_argument("CBoard CAN IDs must differ");
  if (yaml["imu_timeout_ms"]) imu_timeout_ = std::chrono::milliseconds(yaml["imu_timeout_ms"].as<int>());
  if (yaml["imu_wait_ms"]) imu_wait_ = std::chrono::milliseconds(yaml["imu_wait_ms"].as<int>());
  if (yaml["imu_max_age_ms"]) imu_max_age_ = std::chrono::milliseconds(yaml["imu_max_age_ms"].as<int>());
  for (auto timeout : {imu_timeout_, imu_wait_, imu_max_age_})
    if (timeout.count() <= 0 || timeout.count() > 60000)
      throw std::invalid_argument("IMU timeouts must be in 1..60000 ms");

  return yaml;
}

}  // namespace io
