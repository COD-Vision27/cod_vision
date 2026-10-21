#!/usr/bin/env bash
# Default simulation uses same-host IPC; ROS adapter is optional (build_ros.sh).
set -euo pipefail
exec bash "$(dirname "$0")/../../sp_vision_sim/build.sh" "$@"
