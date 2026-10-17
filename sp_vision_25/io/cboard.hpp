#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  int quaternion_canid_, bullet_speed_canid_, send_canid_;
  std::chrono::milliseconds imu_timeout_{3000}, imu_wait_{100}, imu_max_age_{200};
  tools::ThreadSafeQueue<IMUData, true> queue_;  // 满时丢最旧数据，避免长时间积压旧姿态。
  IMUData data_ahead_;
  IMUData data_behind_;
  SocketCAN can_;  // 最后构造、最先析构，回调依赖的成员必须已初始化且仍然存活。

  void callback(const can_frame & frame);

  YAML::Node read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP
