#!/usr/bin/env bash
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: ./run_navigation.sh [gui:=false] [map:=/absolute/map.yaml] [LAUNCH_ARGUMENT:=VALUE ...]\nStarts maze, robot, measured map, AMCL, DWB and Team2 A*. Uses writable E tuning YAMLs.\nStop run_world.sh/run_slam.sh before starting this script.\n'; exit 0
fi
source "$LAB_DIR/tools/lab_env.bash"
CONFIG_DIR="$LAB_DIR/ros2_ws/src/smart_vision_amr/config"
exec ros2 launch smart_vision_amr aruco_lab.launch.py mode:=navigation \
  "world:=$LAB_DIR/worlds/maze.world" "map:=$LAB_DIR/maps/maze_aruco_20261026/map.yaml" \
  "nav_params_file:=$CONFIG_DIR/curve_tuning_simulation.yaml" \
  "nodes_params_file:=$CONFIG_DIR/curve_tuning_simulation_nodes.yaml" \
  initial_x:=-3.000799585626671 initial_y:=-2.0010944835652884 initial_yaw:=0.0008724678696151052 \
  "memory_path:=$LAB_DIR/state/navigation_memory.json" "$@"
