#!/usr/bin/env bash
# ArUco maze, saved SLAM map, and separate editable corner-tuning defaults.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# Reuse the packaged saved map and repository marker path. This also keeps
# custom map/world arguments working without a ~/.gazebo/models installation.
exec "$SCRIPT_DIR/run_web_mission_sim.sh" "$@"
