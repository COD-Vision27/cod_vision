#!/usr/bin/env bash
# Start both ROS endpoints in one SSH worker job so the workspace lock is shared.
set -eo pipefail
SP_PAIR_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ -z "${DISPLAY:-}" ]]; then
  command -v xvfb-run >/dev/null || { echo "No DISPLAY or xvfb-run available" >&2; exit 1; }
  exec xvfb-run -a bash "$0" "$@"
fi
bash "$SP_PAIR_DIR/run_sim.sh" "$@" &
SP_PAIR_SIM_PID=$!
SP_PAIR_AIM_PID=""
cleanup() {
  for SP_PAIR_PID in "$SP_PAIR_SIM_PID" "${SP_PAIR_AIM_PID:-}"; do
    [[ -n "$SP_PAIR_PID" ]] || continue
    if kill -0 "$SP_PAIR_PID" 2>/dev/null; then kill -INT "$SP_PAIR_PID" 2>/dev/null || true; fi
  done
  wait "$SP_PAIR_SIM_PID" 2>/dev/null || true
  if [[ -n "$SP_PAIR_AIM_PID" ]]; then wait "$SP_PAIR_AIM_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Load assets and let robots settle before starting the real vision estimator.
sleep "${SP_PAIR_WARMUP_SECONDS:-5}"
if ! kill -0 "$SP_PAIR_SIM_PID" 2>/dev/null; then wait "$SP_PAIR_SIM_PID"; exit 0; fi
bash "$SP_PAIR_DIR/run_aim.sh" &
SP_PAIR_AIM_PID=$!
while kill -0 "$SP_PAIR_SIM_PID" 2>/dev/null; do
  if ! kill -0 "$SP_PAIR_AIM_PID" 2>/dev/null; then
    SP_PAIR_STATUS=0
    wait "$SP_PAIR_AIM_PID" || SP_PAIR_STATUS=$?
    echo "Auto-aim exited before the simulator (status $SP_PAIR_STATUS)" >&2
    if [[ "$SP_PAIR_STATUS" == 0 ]]; then SP_PAIR_STATUS=1; fi
    exit "$SP_PAIR_STATUS"
  fi
  sleep 0.5
done
wait "$SP_PAIR_SIM_PID"
