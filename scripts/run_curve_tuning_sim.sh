#!/usr/bin/env bash
# ArUco maze, saved SLAM map, and separate editable corner-tuning defaults.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
exec "$SCRIPT_DIR/run_sim_aruco.sh" slam:=false \
  map:="$WORKSPACE_ROOT/maps/maze_aruco_20261026/map.yaml" \
  nav_params_file:="$CONFIG_DIR/curve_tuning_simulation.yaml" \
  nodes_params_file:="$CONFIG_DIR/curve_tuning_simulation_nodes.yaml" "$@"
