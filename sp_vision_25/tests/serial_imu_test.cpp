#include "io/cboard.hpp"
#include "io/serial_imu_protocol.hpp"
#include "io/serial_command_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

using namespace std::chrono_literals;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename F>
void must_throw(F action)
{
  bool thrown = false;
  try { action(); } catch (const std::exception &) { thrown = true; }
  require(thrown, "Expected an exception");
}

std::vector<uint8_t> packet(float roll = 0, float pitch = 0, float yaw = 90, bool big = false)
{
  std::vector<uint8_t> data(27, 0);
  data[0] = 1;
  data[26] = 13;
  const float values[]{pitch, roll, yaw};  // new_uart1 定义的线序，区别于 rpy 参数顺序。
  for (size_t i = 0; i < 3; ++i) {
    uint32_t bits;
    std::memcpy(&bits, &values[i], 4);
    for (size_t j = 0; j < 4; ++j)
      data[2 + 4 * i + j] = (bits >> (8 * (big ? 3 - j : j))) & 255;
  }
  return data;
}

void append(std::vector<uint8_t> & stream, const std::vector<uint8_t> & data, int copies = 1)
{
  while (copies-- > 0) stream.insert(stream.end(), data.begin(), data.end());
}

Eigen::Quaterniond quaternion(const can_frame & frame)
{
  require(frame.can_id == 0x100 && frame.can_dlc == 8, "CAN envelope mismatch");
  std::array<double, 4> q;
  for (size_t i = 0; i < 4; ++i)
    q[i] = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]) / 1e4;
  Eigen::Quaterniond result(q[3], q[0], q[1], q[2]);
  require(std::abs(result.squaredNorm() - 1) < 1e-2, "Not compatible with CBoard norm check");
  return result.normalized();
}

const io::Command UART_COMMAND{true, true, 3.141592653589793 / 6,
                              -3.141592653589793 / 12, 12.5};
const std::vector<uint8_t> UART_BYTES{
  0xff, 0, 0, 0xf0, 0x41, 0, 0, 0x70, 0xc1, 0, 0, 0x48, 0x41};

void command_tests()
{
  io::SerialCommandProtocol encoder(YAML::Load("angle_unit: degrees"));
  require(encoder.encode(UART_COMMAND) == UART_BYTES, "UART command does not match firmware offsets");
  auto stopped = UART_COMMAND;
  stopped.control = false;
  require(encoder.encode(stopped) == UART_BYTES, "control flag changed new_uart2 float layout");
  stopped.control = true;
  stopped.shoot = false;
  require(encoder.encode(stopped) == UART_BYTES, "shoot flag changed new_uart2 float layout");
  io::SerialCommandProtocol radians(YAML::Load("angle_unit: radians\nheader: 0xaa\ntail: []"));
  auto rad_command = UART_COMMAND;
  rad_command.pitch = -0.5;
  rad_command.yaw = 1.5;
  auto rad_bytes = radians.encode(rad_command);
  const std::vector<uint8_t> expected{
    0xaa, 0, 0, 0xc0, 0x3f, 0, 0, 0, 0xbf, 0, 0, 0x48, 0x41};
  require(rad_bytes == expected, "UART TX radians/header/tail configuration failed");
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(), 1e100}) {
    for (int field = 0; field < 3; ++field) {
      auto command = UART_COMMAND;
      if (field == 0) command.pitch = bad;
      if (field == 1) command.yaw = bad;
      if (field == 2) command.horizon_distance = bad;
      must_throw([&] { encoder.encode(command); });
    }
  }
  stopped.horizon_distance = -1;
  must_throw([&] { encoder.encode(stopped); });
  for (const char * bad : {"header: 256", "tail: [-1]", "tail: [0, 13]", "angle_unit: unknown"})
    must_throw([&] { io::SerialCommandProtocol invalid(YAML::Load(bad)); });
  std::cout << "UART TX: new_uart2 13-byte layout, full float precision, units, validation OK\n";
}

void drain(int fd)
{
  uint8_t data[256];
  while (::read(fd, data, sizeof(data)) > 0) {}
}

std::vector<uint8_t> read_bytes(int fd, size_t length)
{
  std::vector<uint8_t> result;
  auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (result.size() < length && std::chrono::steady_clock::now() < deadline) {
    uint8_t data[64];
    auto count = ::read(fd, data, std::min(sizeof(data), length - result.size()));
    if (count > 0) result.insert(result.end(), data, data + count);
    else {
      require(count < 0 && (errno == EAGAIN || errno == EINTR), "Reading UART TX test bytes failed");
      std::this_thread::sleep_for(1ms);
    }
  }
  require(result.size() == length, "UART TX test did not receive full packet");
  return result;
}

void protocol_tests()
{
  std::vector<uint8_t> good;
  append(good, packet(), 5);
  for (size_t chunk : {1u, 2u, 7u, 26u, 27u, 28u, 81u, 4096u}) {
    io::SerialIMUProtocol parser(YAML::Node(), 0x100);
    std::vector<can_frame> frames;
    for (size_t i = 0; i < good.size(); i += chunk) {
      auto result = parser.feed(good.data() + i, std::min(chunk, good.size() - i));
      frames.insert(frames.end(), result.begin(), result.end());
    }
    require(frames.size() == 5, "Split/coalesced frames lost");
    auto q = quaternion(frames[0]);
    require(std::abs(q.z() - std::sqrt(0.5)) < 1e-4 && std::abs(q.w() - std::sqrt(0.5)) < 1e-4,
            "90 degree yaw conversion mismatch");
  }
  io::SerialIMUProtocol parser(YAML::Node(), 0x100);
  auto single = packet();
  require(parser.feed(single.data(), single.size()).empty(), "Sync must require two frames");
  require(parser.feed(single.data(), single.size()).size() == 2, "Two-frame sync failed");
  parser.reset();
  require(parser.feed(single.data(), single.size()).empty(), "Reset must clear synchronization");

  std::vector<uint8_t> damaged;
  append(damaged, single, 2);
  auto truncated = single;
  truncated.erase(truncated.begin() + 8);
  append(damaged, truncated);
  append(damaged, packet(std::numeric_limits<float>::quiet_NaN()));
  append(damaged, packet(0, std::numeric_limits<float>::infinity()));
  append(damaged, packet(0, 0, 1e6));
  auto bad_tail = single;
  bad_tail[26] = 0;
  append(damaged, bad_tail);
  append(damaged, single, 3);
  parser.reset();
  require(parser.feed(damaged.data(), damaged.size()).size() == 5,
          "Corrupt frames accepted or synchronization did not recover");
  parser.reset();
  std::mt19937 random(42);
  std::vector<uint8_t> noise(100000);
  for (auto & value : noise) value = random() & 255;
  append(noise, single, 3);
  require(parser.feed(noise.data(), noise.size()).size() == 3, "Noise recovery failed");

  // 偏移 14..24 的未确认字段不参与姿态计算。
  std::vector<uint8_t> varied;
  for (uint8_t mode : {0, 1, 2, 255}) {
    auto frame = packet(25, -15, 75);
    frame[18] = mode;
    float speed = 23.5f;
    std::memcpy(frame.data() + 14, &speed, 4);
    append(varied, frame);
  }
  io::SerialIMUProtocol uart(YAML::Load("protocol: uart27\nangle_unit: degrees"), 0x100);
  auto varied_frames = uart.feed(varied.data(), varied.size());
  require(varied_frames.size() == 4, "Non-orientation fields broke UART framing");
  Eigen::Matrix3d expected_rotation =
    Eigen::AngleAxisd(75 * 3.141592653589793 / 180, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(-15 * 3.141592653589793 / 180, Eigen::Vector3d::UnitY()).toRotationMatrix() *
    Eigen::AngleAxisd(25 * 3.141592653589793 / 180, Eigen::Vector3d::UnitX()).toRotationMatrix();
  require((quaternion(varied_frames[0]).toRotationMatrix() - expected_rotation).norm() < 3e-4,
          "UART roll/pitch/yaw to quaternion conversion mismatch");

  // 独立字节样本，直接按截图构造：INS.Roll=15，INS.Pitch=-25，INS.Yaw=75。
  // 固件得到 Vision_Pitch=-15、Vision_Roll=25；PC 不应再取负或交换成旧 rpy 顺序。
  const std::vector<uint8_t> firmware_bytes{
    1, 0, 0, 0, 0x70, 0xc1, 0, 0, 0xc8, 0x41, 0, 0, 0x96, 0x42,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0d};
  std::vector<uint8_t> firmware_stream;
  append(firmware_stream, firmware_bytes, 2);
  uart.reset();
  auto fixture_frames = uart.feed(firmware_stream.data(), firmware_stream.size());
  require(fixture_frames.size() == 2 &&
            (quaternion(fixture_frames[0]).toRotationMatrix() - expected_rotation).norm() < 3e-4,
          "new_uart1 raw INS axis/sign fixture failed");
  auto old_tail = firmware_bytes;
  old_tail[25] = 0x0d;
  old_tail[26] = 0;
  std::vector<uint8_t> old_stream;
  append(old_stream, old_tail, 3);
  uart.reset();
  require(uart.feed(old_stream.data(), old_stream.size()).empty(), "Old UART tail was accepted");

  // 当前 Gimbal 固件的实际布局：26 字节有效帧，后跟 1 字节 00 对齐补充。
  auto uart26 = io::SerialIMUProtocol(
    YAML::Load("protocol: uart27\nframe_size: 26\nframe_stride: 27\n"
               "tail: [0, 13]\nangle_unit: degrees"),
    0x100);
  std::vector<uint8_t> padded26;
  for (int i = 0; i < 2; ++i) {
    // old_tail 的前26字节为 01 00 + 三个float + 保留区 + 00 0D，
    // 第27字节 00 是帧间补充，不属于有效帧。
    padded26.insert(padded26.end(), old_tail.begin(), old_tail.begin() + 26);
    padded26.push_back(0);
  }
  auto frames26 = uart26.feed(padded26.data(), padded26.size());
  require(frames26.size() == 2 &&
            (quaternion(frames26[0]).toRotationMatrix() - expected_rotation).norm() < 3e-4,
          "26-byte UART frame plus alignment padding failed");

  // 按 MiniPC_Transmit_Info 构造：零q区，pitch=-15/roll=25/yaw=75，角速度30/-60度每秒。
  std::vector<uint8_t> firmware44(44, 0);
  firmware44[0] = 'S'; firmware44[1] = 'P'; firmware44[43] = 0x0d;
  const std::vector<uint8_t> euler_and_velocity{
    0, 0, 0x70, 0xc1, 0, 0, 0xc8, 0x41, 0, 0, 0x96, 0x42,
    0, 0, 0xf0, 0x41, 0, 0, 0x70, 0xc2};
  std::copy(euler_and_velocity.begin(), euler_and_velocity.end(), firmware44.begin() + 19);
  std::vector<uint8_t> firmware44_stream;
  append(firmware44_stream, firmware44, 2);
  io::SerialIMUProtocol uart44(YAML::Load("protocol: uart44"), 0x100);
  Eigen::Quaterniond decoded44;
  require(uart44.orientation(firmware44.data(), firmware44.size(), decoded44) &&
          (decoded44.toRotationMatrix() - expected_rotation).norm() < 1e-10,
          "uart44 ignored the Euler fields or consumed the zero quaternion region");

  // 标准示例保持新图帧尾；sentry 对当前固件显式配置旧帧尾，不自动兼容二者。
  for (const auto & filename : {"configs/serial_imu.yaml", "configs/sentry.yaml", "configs/standard3.yaml"}) {
    auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / filename;
    auto yaml = YAML::LoadFile(path.string());
    io::SerialIMUProtocol configured(yaml["serial_imu"], 0x100);
    const bool sentry = std::string(filename) == "configs/sentry.yaml";
    const bool gimbal = std::string(filename) == "configs/standard3.yaml";
    const auto & accepted = gimbal ? firmware44_stream : (sentry ? old_stream : firmware_stream);
    const auto & rejected = (sentry || gimbal) ? firmware_stream : old_stream;
    auto result = configured.feed(accepted.data(), accepted.size());
    require(result.size() == (sentry ? 3 : 2) &&
              (quaternion(result[0]).toRotationMatrix() - expected_rotation).norm() < 3e-4,
            "Project YAML does not match its firmware frame markers/axes");
    configured.reset();
    require(configured.feed(rejected.data(), rejected.size()).empty(),
            "Project YAML silently accepted another firmware tail");
    io::SerialCommandProtocol configured_tx(yaml["serial_imu"]["tx"]);
    if (gimbal) {
      std::vector<uint8_t> expected_tx(44, 0);
      expected_tx[0] = 'S'; expected_tx[1] = 'P'; expected_tx[2] = 2;
      expected_tx[29] = 0xf0; expected_tx[30] = 0x41;  // yaw=30度 @27
      expected_tx[21] = 0x70; expected_tx[22] = 0xc1;  // pitch=-15度 @19
      expected_tx[43] = 0x0d;
      require(configured_tx.encode(UART_COMMAND) == expected_tx,
              "standard3 does not use the original SP44 Euler TX layout");
    } else {
      require(configured_tx.encode(UART_COMMAND) == UART_BYTES,
              "Project YAML does not match new_uart2 standard");
    }
  }

  io::SerialIMUProtocol rad_parser(YAML::Load("protocol: uart27\nangle_unit: radians"), 0x100);
  std::vector<uint8_t> rad_stream;
  append(rad_stream, packet(25 * 3.141592653589793 / 180, -15 * 3.141592653589793 / 180,
                            75 * 3.141592653589793 / 180), 2);
  auto rad_frames = rad_parser.feed(rad_stream.data(), rad_stream.size());
  require(rad_frames.size() == 2 &&
            std::abs(quaternion(rad_frames[0]).dot(quaternion(varied_frames[0]))) > 1 - 1e-8,
          "RX degrees/radians config did not produce equivalent orientations");

  auto config = YAML::Load("angle_unit: radians\nfloat_endian: big\nrpy_offsets: [10, 2, 6]\n"
                           "rpy_signs: [1, 1, -1]\nrpy_bias_deg: [0, 0, 10]\ninverse: true\n");
  io::SerialIMUProtocol mapped(config, 0x100);
  std::vector<uint8_t> mapped_stream;
  append(mapped_stream, packet(1.5707963268f, 0, 0, true), 2);
  auto q = quaternion(mapped.feed(mapped_stream.data(), mapped_stream.size()).at(0));
  Eigen::Quaterniond expected(Eigen::AngleAxisd(80 * 3.141592653589793 / 180, Eigen::Vector3d::UnitZ()));
  require(std::abs(q.dot(expected)) > 1 - 1e-8, "Configurable order/units/sign/bias/inverse failed");
  for (const char * bad : {"frame_size: 0", "rpy_offsets: [2, 6, 100]", "rpy_offsets: [2, 2, 10]",
                           "rpy_signs: [0, 1, 1]", "tail: []", "header: [256]",
                           "max_abs_angle_deg: .nan", "angle_unit: grad", "float_endian: unknown",
                           "protocol: unknown", "protocol: auto", "frame_size: 21"})
    must_throw([&] { io::SerialIMUProtocol invalid(YAML::Load(bad), 0x100); });
  must_throw([] { io::SerialIMUProtocol invalid(YAML::Node(), -1); });
  std::cout << "Protocol: split/coalesced frames, corruption, noise, config validation OK\n";
}

struct TempDir
{
  std::string path;
  TempDir()
  {
    char name[] = "/tmp/zimiao_imu_test_XXXXXX";
    auto result = mkdtemp(name);
    require(result != nullptr, "mkdtemp failed");
    path = result;
  }
  ~TempDir() { std::filesystem::remove_all(path); }
};

struct Pty
{
  int master;
  std::string slave;
  Pty()
  {
    master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    require(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0, "Creating PTY failed");
    slave = ptsname(master);
  }
  void close()
  {
    if (master >= 0) ::close(master);
    master = -1;
  }
  ~Pty() { close(); }
  bool send() const
  {
    std::vector<uint8_t> data;
    append(data, packet(), 2);
    return ::write(master, data.data(), data.size()) == static_cast<ssize_t>(data.size());
  }
};

YAML::Node serial_config(const std::string & port)
{
  auto yaml = YAML::Load("use_can: false\nquaternion_canid: 0x100\nbullet_speed_canid: 0x110\n"
                         "send_canid: 0xff\ncan_interface: deliberately-invalid-interface-name\n"
                         "imu_timeout_ms: 1000\nimu_wait_ms: 30\nimu_max_age_ms: 80\n"
                         "serial_imu: {timeout_ms: 100, reconnect_ms: 30}\n");
  yaml["serial_imu"]["port"] = port;
  return yaml;
}

bool pump_until(Pty & pty, const std::function<bool()> & done)
{
  auto deadline = std::chrono::steady_clock::now() + 1500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    pty.send();
    std::this_thread::sleep_for(5ms);
    if (done()) return true;
  }
  return false;
}

void transport_tests()
{
  TempDir dir;
  Pty first, second;
  auto link = dir.path + "/imu";
  std::filesystem::create_symlink(first.slave, link);
  std::atomic<int> count{0};
  std::atomic<bool> invalid{false};
  {
    io::SocketCAN socket(serial_config(link), [&](const can_frame & frame) {
      if (frame.can_id != 0x100 || frame.can_dlc != 8) invalid = true;
      ++count;
    });
    require(pump_until(first, [&] { return count >= 4; }), "Serial mode did not receive frames");
    // PTY 默认会回显；清除工作线程设置 raw 之前写入的测试数据回显。
    drain(first.master);
    socket.write(UART_COMMAND);
    require(read_bytes(first.master, UART_BYTES.size()) == UART_BYTES,
            "SocketCAN wrote CAN bytes instead of UART float command");
    can_frame raw_can{};
    must_throw([&] { socket.write(&raw_can); });
    int before = count;
    std::filesystem::remove(link);
    std::filesystem::create_symlink(second.slave, link);
    first.close();
    require(pump_until(second, [&] { return count > before + 2; }), "Disconnect/reconnect failed");
    drain(second.master);
    socket.write(UART_COMMAND);
    require(read_bytes(second.master, UART_BYTES.size()) == UART_BYTES, "UART TX failed after reconnect");
    // 连续垃圾数据也必须触发有效帧超时，重连后仍能恢复。
    auto deadline = std::chrono::steady_clock::now() + 160ms;
    std::array<uint8_t, 64> garbage{};
    while (std::chrono::steady_clock::now() < deadline) {
      auto written = ::write(second.master, garbage.data(), garbage.size());
      require(written == static_cast<ssize_t>(garbage.size()) ||
                (written < 0 && (errno == EAGAIN || errno == EIO)), "PTY write failed");
      std::this_thread::sleep_for(5ms);
    }
    before = count;
    require(pump_until(second, [&] { return count > before + 2; }), "Invalid-data timeout recovery failed");
  }
  require(!invalid, "Transport delivered malformed CAN envelope");
  int slave_fd = ::open(second.slave.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  require(slave_fd >= 0, "Serial exclusive lock was not released");
  ::close(slave_fd);
  auto yaml = serial_config(dir.path + "/missing");
  yaml["serial_imu"]["reconnect_ms"] = 5000;
  auto socket = std::make_unique<io::SocketCAN>(yaml, [](const can_frame &) {});
  std::this_thread::sleep_for(30ms);
  auto start = std::chrono::steady_clock::now();
  socket.reset();
  require(std::chrono::steady_clock::now() - start < 300ms, "Shutdown blocked on reconnect sleep");
  must_throw([] { io::SocketCAN socket(YAML::Load("{}"), [](const can_frame &) {}); });
  std::cout << "Transport: CAN checks skipped, UART TX, unplug/reconnect, timeout, shutdown OK\n";
}

void backpressure_test()
{
  Pty pty;
  auto yaml = serial_config(pty.slave);
  yaml["serial_imu"]["timeout_ms"] = 3000;
  yaml["serial_imu"]["tx"]["write_timeout_ms"] = 20;
  std::atomic<int> count{0};
  io::SocketCAN socket(yaml, [&](const can_frame &) { ++count; });
  require(pump_until(pty, [&] { return count >= 2; }), "Backpressure test UART not ready");
  drain(pty.master);
  bool failed = false;
  auto command = UART_COMMAND;
  command.shoot = false;
  for (int i = 0; i < 10000; ++i) {
    auto start = std::chrono::steady_clock::now();
    try { socket.write(command); } catch (const std::runtime_error &) { failed = true; }
    require(std::chrono::steady_clock::now() - start < 200ms, "UART write blocked indefinitely");
    if (failed) break;
  }
  require(failed, "Unread PTY did not trigger UART write backpressure");
  must_throw([&] { socket.write(command); });
  std::cout << "UART TX: bounded wait under backpressure, reject further writes before reconnect OK\n";
}

struct Feeder
{
  std::atomic<bool> stop{false}, paused{false};
  std::thread worker;
  explicit Feeder(Pty & pty) : worker([this, &pty] {
    while (!stop) {
      if (!paused) pty.send();
      std::this_thread::sleep_for(2ms);
    }
  }) {}
  ~Feeder() { stop = true; worker.join(); }
};

void cboard_tests()
{
  TempDir dir;
  Pty pty;
  auto config = dir.path + "/config.yaml";
  auto yaml = serial_config(pty.slave);
  { std::ofstream file(config); file << yaml; }
  Feeder feeder(pty);
  {
    io::CBoard board(config);
    auto q = board.imu_at(std::chrono::steady_clock::now());
    require(std::abs(q.z() - std::sqrt(0.5)) < 1e-4, "CBoard orientation mismatch");
    drain(pty.master);
    board.send(UART_COMMAND);
    require(read_bytes(pty.master, UART_BYTES.size()) == UART_BYTES,
            "CBoard::send did not preserve float UART values");
    feeder.paused = true;
    std::this_thread::sleep_for(30ms);
    auto start = std::chrono::steady_clock::now();
    must_throw([&] { board.imu_at(start); });
    require(std::chrono::steady_clock::now() - start < 200ms, "CBoard blocked after stream stopped");
    feeder.paused = false;
    std::this_thread::sleep_for(50ms);
    require(board.imu_at(std::chrono::steady_clock::now()).coeffs().allFinite(), "CBoard did not recover");
    auto old = std::chrono::steady_clock::now();
    board.imu_at(old);
    feeder.paused = true;
    std::this_thread::sleep_for(120ms);
    must_throw([&] { board.imu_at(old); });
  }
  yaml["serial_imu"]["port"] = dir.path + "/missing";
  yaml["imu_timeout_ms"] = 60;
  { std::ofstream file(config); file << yaml; }
  auto start = std::chrono::steady_clock::now();
  must_throw([&] { io::CBoard board(config); });
  require(std::chrono::steady_clock::now() - start < 300ms, "CBoard constructor blocked indefinitely");
  std::cout << "CBoard: interpolation, finite waits, stale rejection, recovery, startup failure OK\n";
}

}  // namespace

int main()
{
  try {
    protocol_tests();
    command_tests();
    transport_tests();
    backpressure_test();
    cboard_tests();
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
