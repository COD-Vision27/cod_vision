#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim {
// Bounds on the Planner's reference error, not a replacement for actuator feedback.
// Dimensions match Solver's existing armor geometry. Only an inscribed ellipse
// of the central fraction is used; range and obliquity can only shrink the window.
inline bool within_fire_window(
  double yaw_error, double pitch_error, double threshold, double yaw_scale,
  double armor_fraction, double range, double facing_cosine, bool big_armor)
{
  if (!std::isfinite(yaw_error) || !std::isfinite(pitch_error)) return false;
  if (armor_fraction == 0) return std::hypot(yaw_error, pitch_error) < threshold;
  if (!std::isfinite(range) || range <= 0 || !std::isfinite(facing_cosine) || facing_cosine <= 0)
    return false;
  const double width = big_armor ? .230 : .135;
  const double yaw_limit = std::min(threshold*yaw_scale,
    std::atan2(.5*width*armor_fraction*std::min(1., facing_cosine), range));
  const double pitch_limit = std::min(threshold, std::atan2(.5*.056*armor_fraction, range));
  if (yaw_limit <= 0 || pitch_limit <= 0) return false;
  return std::hypot(yaw_error/yaw_limit, pitch_error/pitch_limit) < 1.;
}
}  // namespace auto_aim
