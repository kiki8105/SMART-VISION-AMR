#!/usr/bin/env bash
# Start the web UI after run_navigation.sh use_motion_mux:=true.
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: ./run_web.sh [WEB_SERVER_OPTIONS ...]\nFirst start ./run_navigation.sh use_motion_mux:=true in another terminal.\nDefault HTTP port: 8790. Gazebo and the web UI must use the same ROS domain.\n'
  exit 0
fi
source "$LAB_DIR/tools/lab_env.bash"
mkdir -p -- "$LAB_DIR/state"
exec ros2 run smart_vision_amr web_mission_server \
  --host 0.0.0.0 --port "${SMART_VISION_WEB_PORT:-8790}" --use-sim-time \
  --markers-file "$LAB_DIR/ros2_ws/src/smart_vision_amr/config/aruco_mission_markers.yaml" \
  --map-file "$LAB_DIR/maps/maze_aruco_20261026/map.yaml" \
  --config-file "$LAB_DIR/state/web.json" "$@"
