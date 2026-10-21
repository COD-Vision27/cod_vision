#!/usr/bin/env bash
set -eo pipefail
if [[ -f "$(dirname "$0")/../SIMULATOR_INTEGRATION_PAUSED" ]]; then
  echo "仿真接入已按用户要求回滚。请先用 tools/simulator_baseline.py 在远端验证原版，勿运行旧接入二进制。" >&2
  exit 1
fi
source "$(dirname "$0")/env.sh"
SP_BUILD_JOBS="${SP_BUILD_JOBS:-2}"
cd "$SP_SIM_DIR"
cmake -S interfaces/rm_interfaces -B build-interfaces-ascii \
  -DCMAKE_INSTALL_PREFIX="$SP_SIM_DIR/install" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DPython3_EXECUTABLE="$(command -v python3)" \
  -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++
cmake --build build-interfaces-ascii -j"$SP_BUILD_JOBS"
cmake --install build-interfaces-ascii
source install/share/rm_interfaces/local_setup.bash
cmake -S . -B build-ros-ascii -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$SP_SIM_DIR/install" -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DPython3_EXECUTABLE="$(command -v python3)" \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
  -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build-ros-ascii -j"$SP_BUILD_JOBS"
ctest --test-dir build-ros-ascii --output-on-failure
cmake --install build-ros-ascii
