#!/usr/bin/env bash
# Connect the bundled TurtleBot3 GUI to this workspace's isolated simulation.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
CONTROLLER_SCRIPT=${SMART_VISION_CONTROLLER_SCRIPT:-$REPO_DIR/tools/cmd_vel_controller.py}
SETTINGS_FILE=${HOME}/.config/turtlebot3/cmd_vel_controller_simulation.ini
CAMERA_TOPIC=/camera/image_raw
NAV_PARAMS_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/simulation.yaml
NODES_PARAMS_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/simulation_nodes.yaml

usage() {
  cat <<'EOF'
Usage: run_sim_controller.sh [OPTIONS] [-- ROS_ARGUMENTS ...]

Open the repository's bundled PyQt controller against Gazebo or SLAM.
  --workspace PATH        Built Team2 workspace
  --controller-script PATH  Alternate cmd_vel_controller.py
  --settings-file PATH    Separate simulation preferences INI
  --camera-topic TOPIC    Camera image topic (default: /camera/image_raw)
  --nav-params-file PATH  Nav2 simulation defaults YAML (default: source config/simulation.yaml)
  --nodes-params-file PATH  Team-node defaults YAML (default: source config/simulation_nodes.yaml)
  -h, --help              Show this help

ROS_DOMAIN_ID=76, localhost only, robot namespace '/', and simulation time are
selected explicitly. Hardware controller preferences use a separate INI file.
Parameter saves update the selected simulation YAML files for future launches.
When run_sim.sh uses custom nav_params_file:=PATH or nodes_params_file:=PATH,
pass the same paths here with the matching options above.
The map's Goal button sends the selected point to the team's A* planner;
the displayed plan and Stop/cancel use the team's path and cancellation service.
Start run_sim.sh slam:=true for manual SLAM, or run_sim.sh map:=MAP.yaml
for navigation. The Nav2 FollowWaypoints button requires a separate action server.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --workspace|--controller-script|--settings-file|--camera-topic|--nav-params-file|--nodes-params-file)
      if [[ $# -lt 2 || -z "$2" ]]; then usage >&2; exit 2; fi
      case "$1" in
        --workspace) WORKSPACE_ROOT=$2 ;;
        --controller-script) CONTROLLER_SCRIPT=$2 ;;
        --settings-file) SETTINGS_FILE=$2 ;;
        --camera-topic) CAMERA_TOPIC=$2 ;;
        --nav-params-file) NAV_PARAMS_FILE=$2 ;;
        --nodes-params-file) NODES_PARAMS_FILE=$2 ;;
      esac
      shift 2
      ;;
    --) shift; break ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f "$CONTROLLER_SCRIPT" ]]; then
  printf 'Controller script does not exist: %s\n' "$CONTROLLER_SCRIPT" >&2
  exit 1
fi
for variable in NAV_PARAMS_FILE NODES_PARAMS_FILE; do
  value=${!variable}
  if [[ ! -f "$value" ]]; then
    printf 'Parameter file does not exist: %s\n' "$value" >&2
    exit 2
  fi
  printf -v "$variable" '%s' "$(realpath -- "$value")"
done

# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
printf '+ controller: %s\nSettings: %s\nNavigation defaults: %s\nTeam-node defaults: %s\n' \
  "$CONTROLLER_SCRIPT" "$SETTINGS_FILE" "$NAV_PARAMS_FILE" "$NODES_PARAMS_FILE"
exec /usr/bin/python3 "$CONTROLLER_SCRIPT" "$@" \
  --domain-id "$ROS_DOMAIN_ID" --robot-namespace / --use-sim-time \
  --settings-file "$SETTINGS_FILE" --camera-topic "$CAMERA_TOPIC" \
  --parameter-config "$NAV_PARAMS_FILE" --parameter-config "$NODES_PARAMS_FILE" \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  --ros-args -r /goal_pose:=/smart_vision_amr/goal \
  -r /plan:=/smart_vision_amr/path
