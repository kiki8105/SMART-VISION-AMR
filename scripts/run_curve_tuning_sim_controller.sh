#!/usr/bin/env bash
# Default-save targets match run_curve_tuning_sim.sh exactly.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
exec "$SCRIPT_DIR/run_sim_controller.sh" \
  --nav-params-file "$CONFIG_DIR/curve_tuning_simulation.yaml" \
  --nodes-params-file "$CONFIG_DIR/curve_tuning_simulation_nodes.yaml" "$@"
