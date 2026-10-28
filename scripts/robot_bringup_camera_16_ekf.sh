#!/usr/bin/env bash
# Robot-side independent wheel/IMU EKF profile. Stop this before using baseline.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export TURTLEBOT3_LOCALIZATION_MODE=ekf_independent
# Keep camera traffic bounded on the real robot's Wi-Fi while protecting
# time-sensitive wheel, IMU, and LiDAR streams. Operators may override this.
export CAMERA_BACKEND=${CAMERA_BACKEND:-rate_limited}
exec "$SCRIPT_DIR/robot_bringup_camera_16.sh" "$@"
