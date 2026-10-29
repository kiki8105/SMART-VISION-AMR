#!/usr/bin/env bash
# Keep SLAM alive while Ctrl+C saves its live /map, then stop the launch group.
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MAP_PREFIX=
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: ./run_slam.sh [--map-prefix /absolute/new_map/map] [gui:=false] [LAUNCH_ARGUMENT:=VALUE ...]\nFresh SLAM each run. Ctrl+C saves map.yaml + map.pgm before shutdown. Failed saves leave SLAM running; Ctrl+C retries.\n'; exit 0
fi
if [[ "${1:-}" == --map-prefix ]]; then
  if [[ $# -lt 2 || -z "$2" ]]; then printf '%s\n' '--map-prefix needs a path.' >&2; exit 2; fi
  MAP_PREFIX=$2; shift 2
fi
source "$LAB_DIR/tools/lab_env.bash"
MAP_PREFIX=${MAP_PREFIX:-$LAB_DIR/maps/slam_$(date +%Y%m%d_%H%M%S)_$$/map}
if [[ "$MAP_PREFIX" != /* || "$(basename -- "$MAP_PREFIX")" == *.* ]]; then
  printf 'Use an absolute new map prefix without extension: %s\n' "$MAP_PREFIX" >&2; exit 2
fi
for suffix in yaml pgm png; do
  if [[ -e "$MAP_PREFIX.$suffix" ]]; then printf 'Map exists; choose a new prefix: %s.%s\n' "$MAP_PREFIX" "$suffix" >&2; exit 1; fi
done
LAUNCH_PID=
SAVE_ATTEMPT=0
stop_launch() {
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  local deadline=$((SECONDS + 20))
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    if (( SECONDS >= deadline )); then kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true; break; fi
    sleep 0.2
  done
  deadline=$((SECONDS + 5))
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    if (( SECONDS >= deadline )); then kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true; break; fi
    sleep 0.2
  done
  wait "$LAUNCH_PID" 2>/dev/null || true
}
save_and_stop() {
  trap ':' INT TERM
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then printf 'SLAM stopped before saving.\n' >&2; exit 1; fi
  local output_prefix=$MAP_PREFIX saver_pid save_status=0
  SAVE_ATTEMPT=$((SAVE_ATTEMPT + 1))
  while [[ -e "$output_prefix.yaml" || -e "$output_prefix.pgm" || -e "$output_prefix.png" ]]; do
    output_prefix="${MAP_PREFIX}_retry${SAVE_ATTEMPT}"; SAVE_ATTEMPT=$((SAVE_ATTEMPT + 1))
  done
  printf '\nSLAM 지도 자동 저장 중: %s.yaml / .pgm\n' "$output_prefix"
  setsid timeout --signal=TERM --kill-after=3s 30s \
    /usr/bin/python3 "$LAB_DIR/tools/save_map.py" "$output_prefix" &
  saver_pid=$!
  while true; do
    if wait "$saver_pid"; then save_status=0; break; else save_status=$?; fi
    if ! kill -0 "$saver_pid" 2>/dev/null; then break; fi
  done
  if [[ "$save_status" == 0 && -s "$output_prefix.yaml" && -s "$output_prefix.pgm" ]]; then
    printf '지도 자동 저장 완료: %s.yaml\nSLAM과 Gazebo를 종료합니다.\n' "$output_prefix"
    stop_launch; exit 0
  fi
  printf '지도 저장 실패: SLAM을 유지합니다. Ctrl+C로 다시 저장하세요.\n' >&2
  trap save_and_stop INT TERM
}
setsid ros2 launch smart_vision_amr aruco_lab.launch.py mode:=slam use_motion_mux:=false \
  "world:=$LAB_DIR/worlds/maze.world" \
  "slam_params_file:=$LAB_DIR/ros2_ws/src/smart_vision_amr/config/slam_simulation.yaml" "$@" &
LAUNCH_PID=$!
trap save_and_stop INT TERM
printf 'Ctrl+C: 지도 자동 저장 후 종료\n자동 저장 경로: %s.yaml\n' "$MAP_PREFIX"
launch_status=0
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  if wait "$LAUNCH_PID"; then launch_status=0; else launch_status=$?; fi
done
exit "$launch_status"
