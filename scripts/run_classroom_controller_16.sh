#!/usr/bin/env bash
# Initial pose and manual setup for the physical classroom profile.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
exec /usr/bin/python3 /home/kiki/turtlebot3/tools/cmd_vel_controller.py \
  --domain-id 16 --robot-namespace / --no-use-sim-time \
  --camera-topic /camera/image_raw \
  --settings-file "$HOME/.config/turtlebot3/cmd_vel_controller_classroom_16.ini" \
  --parameter-config "$CONFIG_DIR/curve_tuning_waffle_pi.yaml" \
  --parameter-config "$CONFIG_DIR/classroom_hardware_nodes.yaml" \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  "$@" --ros-args -r /goal_pose:=/smart_vision_amr/goal -r /plan:=/smart_vision_amr/path
