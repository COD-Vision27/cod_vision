#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "io/cboard.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;  // 保留字段，接收时不校验。
  uint8_t tail = 0x0d;
};

static_assert(sizeof(GimbalToVision) == 44);

// 用户电控 MiniPC_Transmit_Info 的 UART 布局；q 区域为保留零字节，不能当四元数。
struct __attribute__((packed)) UARTGimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t reserved_mode = 0;
  uint8_t reserved_q[16]{};
  float pitch, roll, yaw, pitch_vel, yaw_vel;
  uint16_t reserved_count = 0, crc16 = 0;
  uint8_t tail = 0x0d;
};
static_assert(sizeof(UARTGimbalToVision) == 44);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint8_t reserved[14]{};  // 原SP轨迹接口保留区；UART发送由编码器转成原44字节欧拉角顺序，忽略acc。
  uint16_t crc16 = 0;  // 通信测试固定发送 0。
  uint8_t tail = 0x0d;
};

static_assert(sizeof(VisionToGimbal) == 44);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

struct GimbalCommunicationStats
{
  uint64_t rx_bytes = 0, rx_frames = 0, rx_discarded_bytes = 0, rx_bad_tails = 0;
  uint64_t tx_attempts = 0, tx_frames = 0, tx_bytes = 0, tx_errors = 0;
  double rx_interval_mean_ms = 0, rx_interval_m2 = 0, rx_interval_max_ms = 0;
  double rx_parse_mean_ms = 0, rx_parse_max_ms = 0;
  double tx_write_mean_ms = 0, tx_write_m2 = 0, tx_write_max_ms = 0;
  std::chrono::steady_clock::time_point last_rx{};
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path, bool wait_for_rx = true);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  GimbalCommunicationStats communication_stats() const;
  size_t tx_frame_size() const
  { return uart_tx_protocol_ ? uart_tx_protocol_->frame_size() : (uart_mode_ ? 13 : sizeof(VisionToGimbal)); }
  bool is_uart() const { return uart_mode_; }
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;
  std::unique_ptr<CBoard> uart_transport_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  GimbalCommunicationStats communication_stats_{};
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};
  bool uart_mode_ = false;
  bool uart44_mode_ = false;
  double uart_angle_scale_ = 1, uart_velocity_scale_ = 1;
  std::unique_ptr<SerialIMUProtocol> uart_rx_protocol_;
  std::unique_ptr<SerialCommandProtocol> uart_tx_protocol_;
  GimbalMode uart_mode_value_ = GimbalMode::AUTO_AIM;
  double uart_bullet_speed_ = 22;

  void read_thread();
  void write_command(const uint8_t * data = nullptr, size_t size = sizeof(VisionToGimbal));
  void reconnect();
  static GimbalMode parse_uart_mode(const YAML::Node & yaml);
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
