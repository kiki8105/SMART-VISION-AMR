#!/usr/bin/env bash
# Initial pose and manual setup for the physical classroom profile.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
CONTROLLER=${SMART_VISION_CONTROLLER_SCRIPT:-$REPO_DIR/tools/cmd_vel_controller.py}
CAMERA_TRANSPORT=${SMART_VISION_CLASSROOM_CAMERA_TRANSPORT:-compressed}
case "$CAMERA_TRANSPORT" in
  raw) CAMERA_SUFFIX= ;;
  compressed) CAMERA_SUFFIX=/compressed ;;
  *) printf 'SMART_VISION_CLASSROOM_CAMERA_TRANSPORT must be raw or compressed.\n' >&2; exit 2 ;;
esac
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: run_classroom_controller_16.sh [CONTROLLER_ARGUMENTS ...]
Open the bundled PyQt controller for the physical classroom robot (domain 16).
SMART_VISION_CONTROLLER_SCRIPT overrides the bundled controller source.
Rear camera defaults to /camera/image_raw/compressed for the EKF bringup.
Set SMART_VISION_CLASSROOM_CAMERA_TRANSPORT=raw for a raw Image publisher.
EOF
  exit 0
fi
if [[ ! -f "$CONTROLLER" ]]; then
  printf 'Controller script does not exist: %s\n' "$CONTROLLER" >&2
  exit 1
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
exec /usr/bin/python3 "$CONTROLLER" \
  --domain-id 16 --robot-namespace / --no-use-sim-time \
  --camera-topic "/camera/image_raw$CAMERA_SUFFIX" \
  --settings-file "$HOME/.config/turtlebot3/cmd_vel_controller_classroom_16.ini" \
  --parameter-config "$CONFIG_DIR/curve_tuning_waffle_pi.yaml" \
  --parameter-config "$CONFIG_DIR/classroom_hardware_nodes.yaml" \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  "$@" --ros-args -r /goal_pose:=/smart_vision_amr/goal -r /plan:=/smart_vision_amr/path
