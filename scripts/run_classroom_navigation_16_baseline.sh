#!/usr/bin/env bash
# PC-side rollback companion to robot_bringup_camera_16_baseline.sh.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "$SCRIPT_DIR/run_classroom_navigation_16.sh" "$@" localization_mode:=baseline
