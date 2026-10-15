#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
cd "$SP_SIM_DIR"
exec ./install/lib/sp_vision_ros/sp_sim_auto_aim --ros-args \
  -p "config:=${SP_AIM_CONFIG:-$SP_SIM_DIR/configs/standard3.yaml}" "$@"
