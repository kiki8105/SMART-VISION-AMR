#!/usr/bin/env bash
# PC-side physical classroom map, localization, A*, and Nav2. No Gazebo.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
MAP_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml
export TURTLEBOT3_TUNING_DOMAIN_ID=16
exec "$SCRIPT_DIR/run_hardware_tuning.sh" "$MAP_FILE" \
  nodes_file:="$CONFIG_DIR/classroom_hardware_nodes.yaml" \
  use_composition:=false \
  lifecycle_bond_timeout:=15.0 \
  use_motion_mux:=false \
  memory_enabled:=false "$@"
