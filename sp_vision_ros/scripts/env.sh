#!/usr/bin/env bash
# Source this file; ROS system installations take precedence over the local runtime.
SP_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# rosidl's CMake file(STRINGS) path parser splits non-ASCII paths. Build through
# an ASCII symlink; source files stay in the user's original workspace.
if [[ "$SP_SIM_DIR" =~ [^a-zA-Z0-9_./-] ]]; then
  SP_ASCII_ROOT="$HOME/.cache/sp-vision-sim/workspace"
  mkdir -p "$(dirname "$SP_ASCII_ROOT")"
  if [[ ! -e "$SP_ASCII_ROOT" ]]; then ln -s "$(dirname "$SP_SIM_DIR")" "$SP_ASCII_ROOT"; fi
  SP_SIM_DIR="$SP_ASCII_ROOT/sp_vision_ros"
fi
SP_SYSTEM_ROS=""
if [[ "${SP_USE_LOCAL_ROS:-0}" != 1 ]]; then
  for SP_DISTRO_CANDIDATE in ${SP_ROS_DISTRO:-${ROS_DISTRO:-humble jazzy}}; do
    if [[ -f "/opt/ros/$SP_DISTRO_CANDIDATE/setup.bash" ]]; then
      SP_SYSTEM_ROS="/opt/ros/$SP_DISTRO_CANDIDATE"
      break
    fi
  done
fi
if [[ -n "$SP_SYSTEM_ROS" ]]; then
  source "$SP_SYSTEM_ROS/setup.bash"
  export CMAKE_PREFIX_PATH="$SP_SYSTEM_ROS:${CMAKE_PREFIX_PATH:-}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
elif [[ -f "$SP_SIM_DIR/.runtime/setup.bash" ]]; then
  export PATH="$SP_SIM_DIR/.runtime/bin:$PATH"
  export LD_LIBRARY_PATH="$SP_SIM_DIR/.runtime/lib:${LD_LIBRARY_PATH:-}"
  export AMENT_PREFIX_PATH="$SP_SIM_DIR/.runtime"
  source "$SP_SIM_DIR/.runtime/setup.bash"
  export CMAKE_PREFIX_PATH="$SP_SIM_DIR/.runtime:${CMAKE_PREFIX_PATH:-}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
else
  echo "ROS2 unavailable. Install Humble/Jazzy, or create the local RoboStack runtime described in README.md." >&2
  return 1
fi
export PKG_CONFIG_PATH="$SP_SIM_DIR/.runtime/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
for SP_ROS_PACKAGE in rm_interfaces sp_vision_ros; do
  if [[ -f "$SP_SIM_DIR/install/share/$SP_ROS_PACKAGE/local_setup.bash" ]]; then
    source "$SP_SIM_DIR/install/share/$SP_ROS_PACKAGE/local_setup.bash"
  fi
done
export CMAKE_PREFIX_PATH="$SP_SIM_DIR/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$SP_SIM_DIR/install/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"
# Needed by the system OpenVINO installed with SP Vision.
export LD_LIBRARY_PATH="/opt/intel/openvino_2024.6.0/runtime/lib/intel64:$LD_LIBRARY_PATH"
