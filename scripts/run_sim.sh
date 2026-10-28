#!/usr/bin/env bash
# Launch only the Team2 workspace overlay, with isolated simulation settings.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
MAP_PREFIX=
NAV_PARAMS_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/simulation.yaml
NODES_PARAMS_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/simulation_nodes.yaml

usage() {
  cat <<'EOF'
Usage: run_sim.sh [--workspace WORKSPACE_ROOT] [--map-prefix /absolute/path/map] [LAUNCH_ARGUMENT:=VALUE ...]

Launch smart_vision_amr/simulation.launch.py from the Team2 workspace.
Isolation: ROS_DOMAIN_ID=76, ROS_LOCALHOST_ONLY=1,
          GAZEBO_MASTER_URI=http://127.0.0.1:11376, TURTLEBOT3_MODEL=waffle_pi.
Inherited ROS and Gazebo isolation values are replaced. Launch arguments pass through.
Navigation and team-node parameters use the source simulation YAML files by default.
Custom nav_params_file:=PATH and nodes_params_file:=PATH must also be selected in
run_sim_controller.sh with --nav-params-file PATH and --nodes-params-file PATH.

Fresh manual SLAM: run_sim.sh slam:=true
Ctrl+C during SLAM: save maps/slam_DATE_TIME_PID/map.yaml + map.pgm, then stop.
If saving fails, SLAM stays running; Ctrl+C retries the save.
Custom save path: run_sim.sh --map-prefix /absolute/path/my_map slam:=true
Existing controller: run_sim_controller.sh (in another terminal)
Saved-map navigation: run_sim.sh map:=/absolute/path/map.yaml
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --workspace|--map-prefix)
      if [[ $# -lt 2 || -z "$2" ]]; then usage >&2; exit 2; fi
      if [[ "$1" == --workspace ]]; then WORKSPACE_ROOT=$2; else MAP_PREFIX=$2; fi
      shift 2
      ;;
    --) shift; break ;;
    *) break ;;
  esac
done

SLAM_ENABLED=false
LAUNCH_ARGUMENTS=()
for argument in "$@"; do
  case "$argument" in
    nav_params_file:=*) NAV_PARAMS_FILE=${argument#nav_params_file:=}; continue ;;
    nodes_params_file:=*) NODES_PARAMS_FILE=${argument#nodes_params_file:=}; continue ;;
  esac
  LAUNCH_ARGUMENTS+=("$argument")
  if [[ "$argument" == slam:=* ]]; then
    value=${argument#slam:=}
    case "${value,,}" in
      true|1) SLAM_ENABLED=true ;;
      *) SLAM_ENABLED=false ;;
    esac
  fi
done
for variable in NAV_PARAMS_FILE NODES_PARAMS_FILE; do
  value=${!variable}
  if [[ ! -f "$value" ]]; then
    printf 'Parameter file does not exist: %s\n' "$value" >&2
    exit 2
  fi
  printf -v "$variable" '%s' "$(realpath -- "$value")"
done
LAUNCH_ARGUMENTS=("nav_params_file:=$NAV_PARAMS_FILE" "nodes_params_file:=$NODES_PARAMS_FILE"
                  "${LAUNCH_ARGUMENTS[@]}")

# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
printf '+ ros2 launch smart_vision_amr simulation.launch.py'
printf ' %q' "${LAUNCH_ARGUMENTS[@]}"
printf '\n'
if [[ "$SLAM_ENABLED" != true ]]; then
  exec ros2 launch smart_vision_amr simulation.launch.py "${LAUNCH_ARGUMENTS[@]}"
fi

WORKSPACE_ROOT=${SMART_VISION_SIM_WORKSPACE:-$WORKSPACE_ROOT}
MAP_PREFIX=${MAP_PREFIX:-$WORKSPACE_ROOT/maps/slam_$(date +%Y%m%d_%H%M%S)_$$/map}
if [[ "$MAP_PREFIX" != /* || "$(basename -- "$MAP_PREFIX")" == *.* ]]; then
  printf 'Use an absolute map prefix without .yaml or .pgm: %s\n' "$MAP_PREFIX" >&2
  exit 2
fi
for suffix in yaml pgm png; do
  if [[ -e "$MAP_PREFIX.$suffix" ]]; then
    printf 'Map already exists: %s.%s; choose a new --map-prefix.\n' "$MAP_PREFIX" "$suffix" >&2
    exit 1
  fi
done

LAUNCH_PID=
SAVE_ATTEMPT=0

stop_launch() {
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  local deadline=$((SECONDS + 20))
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      break
    fi
    sleep 0.2
  done
  deadline=$((SECONDS + 5))
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      break
    fi
    sleep 0.2
  done
  wait "$LAUNCH_PID" 2>/dev/null || true
}

save_and_stop() {
  # Keep both Gazebo's clock and slam_toolbox alive while map_saver subscribes.
  # A no-op handler absorbs repeated Ctrl+C without making children ignore TERM.
  trap ':' INT TERM
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    printf 'SLAM has already stopped; its live map cannot be saved.\n' >&2
    exit 1
  fi
  local output_prefix=$MAP_PREFIX saver_pid save_status=0
  SAVE_ATTEMPT=$((SAVE_ATTEMPT + 1))
  # A failed saver can leave a partial image. Keep it and retry to a new prefix.
  while [[ -e "$output_prefix.yaml" || -e "$output_prefix.pgm" || -e "$output_prefix.png" ]]; do
    output_prefix="${MAP_PREFIX}_retry${SAVE_ATTEMPT}"
    SAVE_ATTEMPT=$((SAVE_ATTEMPT + 1))
  done
  printf '\nSLAM 지도 자동 저장 중: %s.yaml / .pgm\n' "$output_prefix"
  setsid timeout --signal=TERM --kill-after=3s 30s \
    "$SCRIPT_DIR/save_sim_map.sh" --workspace "$WORKSPACE_ROOT" "$output_prefix" &
  saver_pid=$!
  # wait can return early when another terminal signal reaches this wrapper.
  while true; do
    if wait "$saver_pid"; then save_status=0; break; else save_status=$?; fi
    if ! kill -0 "$saver_pid" 2>/dev/null; then break; fi
  done
  if [[ "$save_status" == 0 && -s "$output_prefix.yaml" && -s "$output_prefix.pgm" ]]; then
    printf '지도 자동 저장 완료: %s.yaml\nSLAM과 Gazebo를 종료합니다.\n' "$output_prefix"
    stop_launch
    exit 0
  fi
  printf '지도 저장 실패: SLAM을 유지합니다. Ctrl+C로 다시 저장하세요.\n' >&2
  trap save_and_stop INT TERM
}

# A separate process group prevents terminal Ctrl+C from killing SLAM first.
setsid ros2 launch smart_vision_amr simulation.launch.py "${LAUNCH_ARGUMENTS[@]}" &
LAUNCH_PID=$!
trap save_and_stop INT TERM
printf 'Ctrl+C: 지도 자동 저장 후 종료\n자동 저장 경로: %s.yaml\n' "$MAP_PREFIX"
launch_status=0
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  if wait "$LAUNCH_PID"; then launch_status=0; else launch_status=$?; fi
done
printf 'SLAM 실행이 종료되었습니다. 자동 저장은 run_sim.sh의 Ctrl+C/종료 요청으로 수행됩니다.\n' >&2
exit "$launch_status"
