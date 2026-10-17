#include "io/gimbal/gimbal.hpp"

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

struct MockGimbal
{
  const bool uart;
  int master = -1, slave = -1;
  char config_path[64] = "/tmp/gimbal_protocol_XXXXXX";
  std::atomic<bool> stop{false}, write_failed{false};
  std::atomic<int> phase{0};
  std::thread sender;

  explicit MockGimbal(bool uart_mode = false) : uart(uart_mode)
  {
    char port[128];
    require(openpty(&master, &slave, port, nullptr, nullptr) == 0, "openpty failed");
    termios settings{};
    require(tcgetattr(slave, &settings) == 0, "tcgetattr failed");
    cfmakeraw(&settings);
    require(tcsetattr(slave, TCSANOW, &settings) == 0, "tcsetattr failed");
    int config_fd = mkstemp(config_path);
    require(config_fd >= 0, "mkstemp failed");
    close(config_fd);
    std::ofstream config(config_path);
    config << "use_can: " << (uart ? "false" : "true") << "\ncom_port: " << port << '\n';
    if (uart) config << "serial_imu:\n  protocol: uart44\n  port: " << port
                     << "\n  angle_unit: degrees\n  velocity_unit: degrees\n"
                        "  tx:\n    protocol: sp44\n    angle_unit: degrees\n";
    config.close();
    require(bool(config), "Cannot save test config");
    sender = std::thread([this] {
      while (!stop) {
        io::GimbalToVision frame{};
        frame.mode = 1;
        frame.q[0] = 1;
        frame.yaw = 0.25f;
        frame.yaw_vel = -0.5f;
        frame.pitch = -0.125f;
        frame.pitch_vel = 0.75f;
        frame.bullet_speed = 22;
        int current_phase = phase;
        frame.bullet_count = 11 + current_phase;
        frame.crc16 = current_phase ? 0xbeef : 0;
        if (uart) {
          io::UARTGimbalToVision euler{};
          euler.pitch = -15; euler.roll = 25; euler.yaw = 75;
          euler.pitch_vel = 30; euler.yaw_vel = -60;
          std::memcpy(&frame, &euler, sizeof(frame));
        }
        if (current_phase == 2) frame.tail = 0;  // 必须拒绝错误帧尾。
        if (current_phase == 3) {
          const uint8_t noise[]{0xaa, 0x53, 0xaa};
          if (write(master, noise, sizeof(noise)) != sizeof(noise)) write_failed = true;
          const auto * bytes = reinterpret_cast<const uint8_t *>(&frame);
          if (write(master, bytes, 7) != 7) write_failed = true;
          std::this_thread::sleep_for(2ms);
          if (write(master, bytes + 7, sizeof(frame) - 7) != sizeof(frame) - 7)
            write_failed = true;
        } else if (write(master, &frame, sizeof(frame)) != sizeof(frame)) {
          write_failed = true;
        }
        std::this_thread::sleep_for(10ms);
      }
    });
  }

  ~MockGimbal()
  {
    stop = true;
    if (sender.joinable()) sender.join();
    close(master);
    close(slave);
    unlink(config_path);
  }

  template <size_t N = 44>
  std::array<uint8_t, N> command()
  {
    std::array<uint8_t, N> bytes{};
    size_t size = 0;
    auto deadline = std::chrono::steady_clock::now() + 1s;
    while (size < bytes.size() && std::chrono::steady_clock::now() < deadline) {
      pollfd fd{master, POLLIN, 0};
      if (poll(&fd, 1, 20) <= 0) continue;
      auto count = read(master, bytes.data() + size, bytes.size() - size);
      require(count > 0, "Cannot read Gimbal command");
      size += static_cast<size_t>(count);
    }
    require(size == bytes.size(), "Gimbal command length/timeout mismatch");
    return bytes;
  }
};

void uart_gimbal_test()
{
  constexpr double PI = 3.14159265358979323846;
  MockGimbal mock(true);
  io::Gimbal gimbal(mock.config_path);
  auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!gimbal.communication_stats().rx_frames && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  auto state = gimbal.state();
  require(gimbal.mode() == io::GimbalMode::AUTO_AIM && state.bullet_speed == 22,
          "UART reserved mode/speed bytes override configured defaults");
  require(std::abs(state.yaw - 75 * PI / 180) < 1e-6 &&
          std::abs(state.pitch + 15 * PI / 180) < 1e-6 &&
          std::abs(state.pitch_vel - 30 * PI / 180) < 1e-6 &&
          std::abs(state.yaw_vel + 60 * PI / 180) < 1e-6,
          "UART angle/velocity offsets or degrees-to-radians conversion failed");
  auto q = gimbal.q(std::chrono::steady_clock::now());
  Eigen::Quaterniond expected_q = Eigen::AngleAxisd(75 * PI / 180, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-15 * PI / 180, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(25 * PI / 180, Eigen::Vector3d::UnitX());
  require((q.toRotationMatrix() - expected_q.toRotationMatrix()).norm() < 1e-10,
          "Gimbal consumed zero q region instead of UART Euler angles");
  std::array<uint8_t, 44> expected{0x53, 0x50, 2};
  expected[21] = 0x70; expected[22] = 0xc1;  // pitch=-15 @19
  expected[29] = 0xf0; expected[30] = 0x41;  // yaw=30 @27
  expected[33] = 0xf0; expected[34] = 0xc1;  // pitch_vel=-30 @31
  expected[37] = 0x20; expected[38] = 0x41;  // yaw_vel=10 @35
  expected[43] = 0x0d;
  gimbal.send(true, true, PI / 6, PI / 18, PI / 9, -PI / 12, -PI / 6, -PI / 3);
  require(mock.command() == expected, "UART scalar TX changed original SP44 field order or encoded acc");
  io::VisionToGimbal command{};
  command.mode = 2; command.yaw = PI / 6; command.pitch = -PI / 12;
  command.yaw_vel = PI / 18; command.yaw_acc = 1234;
  command.pitch_vel = -PI / 6; command.pitch_acc = -5678;
  command.crc16 = 0xffff;
  gimbal.send(command);
  require(mock.command() == expected, "UART struct TX changed original SP44 field order or encoded acc");
  auto stats = gimbal.communication_stats();
  require(stats.tx_frames == 2 && stats.tx_bytes == 88 && !stats.tx_errors,
          "UART44 TX statistics mismatch");
  std::cout << "UART Gimbal: Euler44 RX/TX original offsets OK; acceleration ignored by both TX APIs\n";
}
}  // namespace

int main() try
{
  static_assert(sizeof(io::GimbalToVision) == 44);
  static_assert(sizeof(io::VisionToGimbal) == 44);
  MockGimbal mock;
  io::Gimbal gimbal(mock.config_path);
  auto deadline = std::chrono::steady_clock::now() + 1s;
  while (gimbal.state().bullet_count != 11 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  require(gimbal.mode() == io::GimbalMode::AUTO_AIM, "Zero CRC RX mode rejected");
  require(gimbal.state().bullet_count == 11, "Zero CRC RX frame rejected");
  auto q = gimbal.q(std::chrono::steady_clock::now());
  require(std::abs(q.w() - 1) < 1e-6 && q.vec().norm() < 1e-6, "RX quaternion changed");
  auto state = gimbal.state();
  require(state.yaw == 0.25f && state.yaw_vel == -0.5f && state.pitch == -0.125f &&
          state.pitch_vel == 0.75f && state.bullet_speed == 22, "RX payload changed");

  mock.phase = 1;
  deadline = std::chrono::steady_clock::now() + 1s;
  while (gimbal.state().bullet_count != 12 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  require(gimbal.state().bullet_count == 12, "RX still validates CRC");

  mock.phase = 2;
  std::this_thread::sleep_for(60ms);
  require(gimbal.state().bullet_count == 12, "RX accepted invalid tail");
  mock.phase = 3;
  deadline = std::chrono::steady_clock::now() + 1s;
  while (gimbal.state().bullet_count != 14 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  require(gimbal.state().bullet_count == 14, "RX failed to resync fragmented/noisy frames");

  // 独立样例：SP、mode=1、六个小端 float (1, -2, 3, -4, 5, -6)、CRC=0、尾0D。
  std::array<uint8_t, 44> expected{
    0x53, 0x50, 1, 0, 0, 0x80, 0x3f, 0, 0, 0, 0xc0, 0, 0, 0x40, 0x40,
    0, 0, 0x80, 0xc0, 0, 0, 0xa0, 0x40, 0, 0, 0xc0, 0xc0};
  expected[43] = 0x0d;
  gimbal.send(true, false, 1, -2, 3, -4, 5, -6);
  require(mock.command() == expected, "Scalar send payload/zero CRC mismatch");
  io::VisionToGimbal command{};
  command.mode = 1;
  command.yaw = 1;
  command.yaw_vel = -2;
  command.yaw_acc = 3;
  command.pitch = -4;
  command.pitch_vel = 5;
  command.pitch_acc = -6;
  command.crc16 = 0xffff;
  command.tail = 0;  // 调用方传入的尾字节也必须被协议固定值覆盖。
  gimbal.send(command);
  require(mock.command() == expected, "Struct send did not force CRC to zero");
  require(!mock.write_failed, "Mock RX write failed");
  auto stats = gimbal.communication_stats();
  require(stats.rx_frames > 1 && stats.rx_bytes >= stats.rx_frames * 44,
          "RX communication counters mismatch");
  require(stats.rx_bad_tails > 0 && stats.rx_discarded_bytes > 0,
          "RX rejection counters missing");
  require(stats.tx_attempts == 2 && stats.tx_frames == 2 && stats.tx_bytes == 88 &&
          stats.tx_errors == 0, "TX communication counters mismatch");
  uart_gimbal_test();
  std::cout << "Gimbal PTY: 44-byte RX/tail/resync OK; both 44-byte TX APIs send CRC=0, tail=0D\n";
  return 0;
}
catch (const std::exception & e)
{
  std::cerr << "Gimbal protocol test failed: " << e.what() << '\n';
  return 1;
}
