#!/usr/bin/env bash
# Robot-side independent wheel/IMU EKF profile. Stop this before using baseline.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export TURTLEBOT3_LOCALIZATION_MODE=ekf_independent
# Trial gate, not a manufacturer cutoff: after a low-voltage trip, reset
# requires 0.2 V above this limit. Override only after measuring the pack.
export SMART_VISION_MIN_BATTERY_VOLTAGE=${SMART_VISION_MIN_BATTERY_VOLTAGE:-11.2}
# Keep camera traffic bounded on the real robot's Wi-Fi while protecting
# time-sensitive wheel, IMU, and LiDAR streams. Operators may override this.
export CAMERA_BACKEND=${CAMERA_BACKEND:-rate_limited}
exec "$SCRIPT_DIR/robot_bringup_camera_16.sh" "$@"
