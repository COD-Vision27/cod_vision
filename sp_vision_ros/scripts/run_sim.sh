#!/usr/bin/env bash
set -eo pipefail
if [[ -f "$(dirname "$0")/../SIMULATOR_INTEGRATION_PAUSED" ]]; then
  echo "仿真接入已按用户要求回滚。请先用 tools/simulator_baseline.py 在远端验证原版，勿运行旧接入二进制。" >&2
  exit 1
fi
source "$(dirname "$0")/env.sh"
SP_SIM_REPO="$SP_SIM_DIR/../bevy_robomaster_simulator"
export BEVY_ASSET_ROOT="$SP_SIM_REPO"
export RUST_LOG="${RUST_LOG:-info}"
export LD_LIBRARY_PATH="$SP_SIM_REPO/target/debug/deps:$(rustc --print target-libdir):$LD_LIBRARY_PATH"
SP_RUN_DIR="$SP_SIM_DIR/artifacts/sim-runtime"
mkdir -p "$SP_RUN_DIR"
if [[ ! -e "$SP_RUN_DIR/assets" ]]; then ln -s "$SP_SIM_REPO/assets" "$SP_RUN_DIR/assets"; fi
cp "${SP_SIM_CONFIG:-$SP_SIM_DIR/configs/simulator.toml}" "$SP_RUN_DIR/config.toml"
cd "$SP_RUN_DIR"
export DAEDALUS_AUTO_AIM="${DAEDALUS_AUTO_AIM:-1}"
export DAEDALUS_METRICS_PATH="${DAEDALUS_METRICS_PATH:-$SP_SIM_DIR/artifacts/simulator_metrics.jsonl}"
exec "$SP_SIM_REPO/target/debug/daedalus" "$@"
