#!/usr/bin/env bash
# Run identical, ground-truth-controlled Gazebo trials for three odometry modes.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
MODE=${1:-all}
if [[ $# -gt 0 ]]; then shift; fi
REPEATS=3
OUTPUT_DIR=
BUILD=true

usage() {
  cat <<'EOF'
Usage: run_ekf_benchmark.sh [all|baseline|ekf_overlay|ekf_independent]
                            [--repeats N] [--output-dir ABSOLUTE_PATH] [--skip-build]

The script builds the Team2 package, creates a test-only world with independent
Gazebo ground truth, starts and stops one headless simulation per trial, drives
an identical 0.55 m square route, and writes CSV/JSON results plus a summary.
The trial alone publishes /cmd_vel; do not run another simulation in domain 76.

The default is 3 repetitions of every mode. A result with fewer than 3 valid
repetitions per mode is preliminary and will not select a winning mode.
Results default to REPO/tmp/ekf_benchmark/TIMESTAMP/ (ignored by Git).
EOF
}

case "$MODE" in
  all|baseline|ekf_overlay|ekf_independent) ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repeats)
      if [[ $# -lt 2 || ! "$2" =~ ^[1-9][0-9]*$ ]]; then usage >&2; exit 2; fi
      REPEATS=$2; shift 2 ;;
    --output-dir)
      if [[ $# -lt 2 || "$2" != /* ]]; then usage >&2; exit 2; fi
      OUTPUT_DIR=$2; shift 2 ;;
    --skip-build) BUILD=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

OUTPUT_DIR=${OUTPUT_DIR:-$REPO_DIR/tmp/ekf_benchmark/$(date +%Y%m%d_%H%M%S)}
if [[ -e "$OUTPUT_DIR" ]]; then
  printf 'Output directory already exists; choose a new --output-dir: %s\n' "$OUTPUT_DIR" >&2
  exit 2
fi
mkdir -p -- "$OUTPUT_DIR"

# A running Gazebo master could absorb a spawn/command and invalidate the
# comparison.  Refuse rather than disturb the existing user's simulation.
if (echo > /dev/tcp/127.0.0.1/11376) >/dev/null 2>&1; then
  printf 'Gazebo master 11376 is already in use. Stop the other domain-76 simulation.\n' >&2
  exit 2
fi

if [[ "$BUILD" == true ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  (cd "$WORKSPACE_ROOT" && colcon build --packages-select smart_vision_amr --symlink-install)
fi

# A non-root local installation is accepted when robot_localization is absent
# from /opt/ros/humble. This machine's experiment build uses these paths.
LOCAL_EKF_ROOT=$HOME/.local/opt/robot_localization/extracted
if [[ -z "${SMART_VISION_EXTRA_ROS_PREFIX:-}" \
      && -d "$LOCAL_EKF_ROOT/opt/ros/humble/share/robot_localization" ]]; then
  export SMART_VISION_EXTRA_ROS_PREFIX=$LOCAL_EKF_ROOT/opt/ros/humble
  export SMART_VISION_EXTRA_LIBRARY_DIR=$LOCAL_EKF_ROOT/usr/lib/x86_64-linux-gnu
fi
# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
if [[ "$MODE" != baseline ]] && ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  printf 'robot_localization is not installed; EKF trials cannot run.\n' >&2
  exit 2
fi

WORLD=$OUTPUT_DIR/ekf_benchmark.world
python3 "$SCRIPT_DIR/make_ekf_benchmark_world.py" \
  "$REPO_DIR/ros2_ws/src/smart_vision_amr/worlds/simulation_room.world" "$WORLD"

LAUNCH_PID=
cleanup_launch() {
  if [[ -z "$LAUNCH_PID" ]]; then return; fi
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  local deadline=$((SECONDS + 20))
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      break
    fi
    sleep .2
  done
  wait "$LAUNCH_PID" 2>/dev/null || true
  LAUNCH_PID=
  # Gazebo Classic needs a moment to release its master port between trials.
  sleep 2
}
trap cleanup_launch EXIT INT TERM

if [[ "$MODE" == all ]]; then
  MODES=(baseline ekf_overlay ekf_independent)
else
  MODES=("$MODE")
fi
FAILURES=0
for trial_mode in "${MODES[@]}"; do
  for ((trial=1; trial<=REPEATS; trial++)); do
    prefix=$OUTPUT_DIR/${trial_mode}_trial${trial}
    printf '\n=== %s trial %d/%d ===\n' "$trial_mode" "$trial" "$REPEATS"
    setsid ros2 launch smart_vision_amr simulation.launch.py \
      "world:=$WORLD" "localization_mode:=$trial_mode" \
      gui:=false use_rviz:=false use_motion_mux:=false \
      >"$prefix.launch.log" 2>&1 &
    LAUNCH_PID=$!
    sleep 5
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      printf 'Simulation launch exited early. See %s.launch.log\n' "$prefix" >&2
      FAILURES=$((FAILURES + 1))
      cleanup_launch
      continue
    fi
    if ! python3 "$SCRIPT_DIR/ekf_benchmark_trial.py" \
        --mode "$trial_mode" --output-prefix "$prefix" --wall-timeout 150 \
        >"$prefix.trial.log" 2>&1; then
      printf 'Trial failed. See %s.trial.log and .json\n' "$prefix" >&2
      FAILURES=$((FAILURES + 1))
    fi
    cleanup_launch
  done
done

if [[ "$MODE" == all ]]; then
  python3 "$SCRIPT_DIR/compare_ekf_benchmarks.py" "$OUTPUT_DIR" \
    --minimum-trials "$REPEATS" || FAILURES=$((FAILURES + 1))
fi
printf 'EKF benchmark results: %s\n' "$OUTPUT_DIR"
exit "$((FAILURES > 0))"
