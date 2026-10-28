#!/usr/bin/env bash
# PC-side companion to robot_bringup_camera_16_ekf.sh.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "$SCRIPT_DIR/run_classroom_navigation_16.sh" "$@" localization_mode:=ekf_independent
