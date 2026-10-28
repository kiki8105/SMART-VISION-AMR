#!/usr/bin/env bash
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: ./run_controller.sh [CONTROLLER_ARGUMENTS ...]\nUses installed turtlebot3-controller, simulation time, lab domain, Team2 A* endpoints and E source YAMLs.\n'; exit 0
fi
if [[ -x /usr/bin/turtlebot3-controller ]]; then
  CONTROLLER=/usr/bin/turtlebot3-controller
else
  CONTROLLER=$(command -v turtlebot3-controller || true)
fi
if [[ -z "$CONTROLLER" ]]; then
  printf 'Install the bundled controller DEB first:\n  ./install_controller.sh\n' >&2; exit 1
fi
source "$LAB_DIR/tools/lab_env.bash"
export TURTLEBOT3_CONTROLLER_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"
mkdir -p -- "$LAB_DIR/state"
CONFIG_DIR="$LAB_DIR/ros2_ws/src/smart_vision_amr/config"
exec "$CONTROLLER" "$@" --domain-id "$ROS_DOMAIN_ID" --robot-namespace / --use-sim-time \
  --settings-file "$LAB_DIR/state/controller_simulation.ini" --camera-topic /camera/image_raw \
  --parameter-config "$CONFIG_DIR/curve_tuning_simulation.yaml" \
  --parameter-config "$CONFIG_DIR/curve_tuning_simulation_nodes.yaml" \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  --ros-args -r /goal_pose:=/smart_vision_amr/goal -r /plan:=/smart_vision_amr/path
