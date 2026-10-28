#!/usr/bin/env bash
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: ./run_world.sh [gui:=false] [use_rviz:=true] [LAUNCH_ARGUMENT:=VALUE ...]\nStarts the maze, Waffle Pi and sensors only; no SLAM or navigation.\n'; exit 0
fi
source "$LAB_DIR/tools/lab_env.bash"
exec ros2 launch smart_vision_amr aruco_lab.launch.py mode:=world \
  "world:=$LAB_DIR/worlds/maze.world" "$@"
