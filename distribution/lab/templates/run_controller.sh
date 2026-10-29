#!/usr/bin/env bash
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: ./run_controller.sh [CONTROLLER_ARGUMENTS ...]\nUses the bundled controller source, simulation time, lab domain and editable tuning YAMLs.\n'; exit 0
fi
CONTROLLER=$LAB_DIR/tools/controller/cmd_vel_controller.py
if [[ ! -f "$CONTROLLER" ]]; then
  printf 'Bundled controller source is missing: %s\n' "$CONTROLLER" >&2; exit 1
fi
source "$LAB_DIR/tools/lab_env.bash"
export TURTLEBOT3_CONTROLLER_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"
mkdir -p -- "$LAB_DIR/state"
CONFIG_DIR="$LAB_DIR/ros2_ws/src/smart_vision_amr/config"
exec /usr/bin/python3 "$CONTROLLER" "$@" --domain-id "$ROS_DOMAIN_ID" --robot-namespace / --use-sim-time \
  --settings-file "$LAB_DIR/state/controller_simulation.ini" --camera-topic /camera/image_raw \
  --parameter-config "$CONFIG_DIR/curve_tuning_simulation.yaml" \
  --parameter-config "$CONFIG_DIR/curve_tuning_simulation_nodes.yaml" \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  --ros-args -r /goal_pose:=/smart_vision_amr/goal -r /plan:=/smart_vision_amr/path
