#!/usr/bin/env bash
# Use only while the robot is physically parked at classroom Home with its
# rear camera facing marker 0. This updates AMCL; it does not move the robot.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0

exec python3 -m smart_vision_amr.classroom_home_init \
  --map "$REPO_DIR/ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml" \
  --markers "$REPO_DIR/ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers.yaml" "$@"
