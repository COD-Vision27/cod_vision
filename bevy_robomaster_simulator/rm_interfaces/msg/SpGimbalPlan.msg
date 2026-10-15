# SP Planner trajectory, radians; yaw positive left, pitch negative up.
# Absolute orientation in ROS map axes, around the muzzle origin.
std_msgs/Header header
bool control
bool fire
float64 yaw
float64 yaw_vel
float64 yaw_acc
float64 pitch
float64 pitch_vel
float64 pitch_acc
float64 max_yaw_acc
float64 max_pitch_acc
# Watchdog measured since receipt in simulator time, not a ROS wall-clock deadline.
float64 valid_for
