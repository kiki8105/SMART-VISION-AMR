#!/usr/bin/env bash
# Real-robot tuning GUI: saves accepted edits back into the launch's source YAML.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
PARAMS_FILE=${TURTLEBOT3_TUNING_PARAMS_FILE:-$REPO_DIR/ros2_ws/src/smart_vision_amr/config/curve_tuning_waffle_pi.yaml}
NODES_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/nodes.yaml
DOMAIN_ID=${TURTLEBOT3_TUNING_DOMAIN_ID:-16}
CONTROLLER=${SMART_VISION_CONTROLLER_SCRIPT:-$REPO_DIR/tools/cmd_vel_controller.py}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: run_hardware_tuning_controller.sh [CONTROLLER_ARGUMENTS ...]
Real-robot domain 16, /camera/image_raw, real time, Team2 A* goal/path endpoints.
Default-save targets are the real-robot tuning YAML and hardware nodes.yaml.
TURTLEBOT3_TUNING_DOMAIN_ID and TURTLEBOT3_TUNING_PARAMS_FILE override defaults.
EOF
  exit 0
fi
if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ || "$DOMAIN_ID" -gt 232 ]]; then
  printf 'TURTLEBOT3_TUNING_DOMAIN_ID must be 0..232.\n' >&2
  exit 2
fi
for path in "$CONTROLLER" "$PARAMS_FILE" "$NODES_FILE" "$WORKSPACE_ROOT/install/local_setup.bash"; do
  if [[ ! -f "$path" ]]; then printf 'Required file is missing: %s\n' "$path" >&2; exit 2; fi
done
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="$DOMAIN_ID" ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
source "$SCRIPT_DIR/dds_env.bash" ""
exec /usr/bin/python3 "$CONTROLLER" "$@" \
  --domain-id "$DOMAIN_ID" --robot-namespace / --no-use-sim-time \
  --settings-file "$HOME/.config/turtlebot3/cmd_vel_controller_hardware_tuning.ini" \
  --parameter-config "$(realpath -- "$PARAMS_FILE")" --parameter-config "$(realpath -- "$NODES_FILE")" \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  --ros-args -r /goal_pose:=/smart_vision_amr/goal -r /plan:=/smart_vision_amr/path
