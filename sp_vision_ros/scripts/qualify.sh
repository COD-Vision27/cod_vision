#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
cd "$SP_SIM_DIR/.."
# Test the supplied configs exactly; regenerate geometry only when explicitly requested.
bash sp_vision_ros/scripts/build.sh
python3 sp_vision_ros/tests/test_geometry.py
if [[ -f sp_vision_25/docs/tuning-source-manifest.json ]]; then
  python3 sp_vision_25/tools/check_tuned_sources.py
else
  python3 sp_vision_ros/scripts/check_sp_unchanged.py
fi
python3 sp_vision_ros/tests/ros_smoke.py
(cd bevy_robomaster_simulator && cargo test --no-default-features --features ros2 --bin daedalus -j"${SP_BUILD_JOBS:-8}")
python3 tools/sim_accuracy.py "$@"
