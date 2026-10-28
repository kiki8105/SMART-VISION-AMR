#!/usr/bin/env bash
# Saved-map maze navigation for the web mission, without a separate RViz window.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=${SMART_VISION_WEB_WORKSPACE:-$(cd -- "$REPO_DIR/../.." && pwd)}
DOMAIN_ID=${SMART_VISION_WEB_DOMAIN_ID:-76}
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
WORLD=$REPO_DIR/ros2_ws/src/smart_vision_amr/worlds/simulation_maze_aruco.world
LAUNCH_ARGUMENTS=()

usage() {
  cat <<'EOF'
Usage: run_web_mission_sim.sh [--domain-id N] [--workspace PATH] [LAUNCH_ARGUMENT:=VALUE ...]
Start the four-marker maze, saved measured map, AMCL, A*, and Nav2 controller.
Default: domain76, localhost ROS, Gazebo port11376, Gazebo GUI on, RViz off.
The Gazebo port is 11300 + the selected domain. No build runs here.

Examples:
  scripts/run_web_mission_sim.sh
  SMART_VISION_WEB_DOMAIN_ID=82 scripts/run_web_mission_sim.sh gui:=false

Use the same domain with scripts/run_web_mission.sh in a second terminal.
SMART_VISION_WEB_WORKSPACE overrides the workspace root.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --domain-id|--workspace)
      if [[ $# -lt 2 || -z "$2" ]]; then
        printf 'Missing value for %s\n' "$1" >&2
        exit 2
      fi
      if [[ "$1" == --domain-id ]]; then DOMAIN_ID=$2; else WORKSPACE_ROOT=$2; fi
      shift 2
      ;;
    --) shift; LAUNCH_ARGUMENTS+=("$@"); break ;;
    *) LAUNCH_ARGUMENTS+=("$1"); shift ;;
  esac
done
if [[ ! "$DOMAIN_ID" =~ ^[0-9]{1,3}$ ]] || (( 10#$DOMAIN_ID > 232 )); then
  printf 'ROS domain must be an integer from 0 to 232.\n' >&2
  exit 2
fi
DOMAIN_ID=$((10#$DOMAIN_ID))

# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
WORKSPACE_ROOT=$SMART_VISION_SIM_WORKSPACE
MAP_FILE=$WORKSPACE_ROOT/maps/maze_aruco_20261026/map.yaml
for required in "$WORLD" "$MAP_FILE" "$CONFIG_DIR/curve_tuning_simulation.yaml" \
                "$CONFIG_DIR/curve_tuning_simulation_nodes.yaml"; do
  if [[ ! -f "$required" ]]; then
    printf 'Simulation input is missing: %s\n' "$required" >&2
    exit 2
  fi
done
for marker_id in 0 1 2 3; do
  if [[ ! -f "$REPO_DIR/gazebo_models/smart_vision_aruco_4x4_50_id_$marker_id/model.sdf" ]]; then
    printf 'Marker model %s is missing. Run scripts/generate_aruco_models.py --ids 0 1 2 3 once.\n' \
      "$marker_id" >&2
    exit 2
  fi
done

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI="http://127.0.0.1:$((11300 + DOMAIN_ID))"
export GAZEBO_MODEL_PATH="$REPO_DIR/gazebo_models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
printf 'Web mission simulator: domain=%s localhost_only=%s Gazebo=%s\n' \
  "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$GAZEBO_MASTER_URI"
exec ros2 launch smart_vision_amr simulation.launch.py \
  world:="$WORLD" map:="$MAP_FILE" \
  nav_params_file:="$CONFIG_DIR/curve_tuning_simulation.yaml" \
  nodes_params_file:="$CONFIG_DIR/curve_tuning_simulation_nodes.yaml" \
  slam:=false gui:=true use_rviz:=false memory_enabled:=false \
  x_pose:=-3.0 y_pose:=-2.0 yaw:=0.0 \
  initial_x:=-3.000799585626671 initial_y:=-2.0010944835652884 \
  initial_yaw:=0.0008724678696151052 \
  "${LAUNCH_ARGUMENTS[@]}"
