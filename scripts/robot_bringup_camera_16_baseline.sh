#!/usr/bin/env bash
# Explicit rollback to the stock TurtleBot3 odometry profile on the robot.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export TURTLEBOT3_LOCALIZATION_MODE=baseline
exec "$SCRIPT_DIR/robot_bringup_camera_16.sh" "$@"
