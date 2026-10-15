#pragma once
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <tasks/auto_aim/planner/planner.hpp>

namespace sp_sim {
// Bevy (right, up, back) -> SP/ROS (forward, left, up).
inline Eigen::Matrix3d align() {
  Eigen::Matrix3d m; m << 0,0,-1, -1,0,0, 0,1,0; return m;
}
inline Eigen::Matrix3d optical_to_gimbal() {
  Eigen::Matrix3d r; r << 0,0,1, -1,0,0, 0,-1,0; return r;
}
inline Eigen::Vector3d camera_offset(const Eigen::Quaterniond &shot,
                                    const Eigen::Vector3d &view,
                                    const Eigen::Vector3d &muzzle) {
  Eigen::Matrix3d basis = shot.normalized().toRotationMatrix() *
    Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return align() * basis.transpose() * (view-muzzle);
}
// Bevy projectile forward is local +Y; a horizontal muzzle therefore has Rx(-pi/2).
inline double muzzle_pitch(double sp_pitch) { return -sp_pitch - M_PI/2; }
inline std::chrono::steady_clock::time_point steady_from_ros(int64_t stamp_ns) {
  const auto sys = std::chrono::system_clock::now().time_since_epoch();
  const auto delta = std::chrono::nanoseconds(stamp_ns) -
    std::chrono::duration_cast<std::chrono::nanoseconds>(sys);
  return std::chrono::steady_clock::now() + delta;
}
inline bool finite_plan(const auto_aim::Plan &p) {
  return std::isfinite(p.yaw) && std::isfinite(p.pitch) && std::isfinite(p.yaw_vel) &&
    std::isfinite(p.pitch_vel) && std::isfinite(p.yaw_acc) && std::isfinite(p.pitch_acc);
}
}
