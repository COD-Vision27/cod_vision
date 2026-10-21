#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$root/sp_vision_ros" -B "$root/sp_vision_ros/build-ros-ascii" \
  -DSP_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR="${OpenVINO_DIR:-/opt/intel/openvino_2024.6.0/runtime/cmake}"
cmake --build "$root/sp_vision_ros/build-ros-ascii" -j"${SP_BUILD_JOBS:-8}"
ctest --test-dir "$root/sp_vision_ros/build-ros-ascii" --output-on-failure
cmake -S "$root/sp_vision_25/tests/tuning" -B "$root/sp_vision_25/build-tuning" \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR="${OpenVINO_DIR:-/opt/intel/openvino_2024.6.0/runtime/cmake}"
cmake --build "$root/sp_vision_25/build-tuning" -j"${SP_BUILD_JOBS:-8}"
ctest --test-dir "$root/sp_vision_25/build-tuning" --output-on-failure
