#!/usr/bin/env bash
# Run the web/ROS bridge against an already-running simulation or real robot.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=${SMART_VISION_WEB_WORKSPACE:-$(cd -- "$REPO_DIR/../.." && pwd)}
DOMAIN_ID=${SMART_VISION_WEB_DOMAIN_ID:-76}
HTTP_HOST=${SMART_VISION_WEB_HOST:-0.0.0.0}
HTTP_PORT=${SMART_VISION_WEB_PORT:-8790}
MARKERS_FILE=${SMART_VISION_WEB_MARKERS_FILE:-$REPO_DIR/ros2_ws/src/smart_vision_amr/config/aruco_mission_markers.yaml}
USE_SIM_TIME=true
SERVER_ARGUMENTS=()

usage() {
  cat <<'EOF'
Usage: run_web_mission.sh [OPTIONS] [-- SERVER_OPTIONS ...]
Start the web mission server; Gazebo or robot bringup/navigation runs separately.

  --domain-id N             ROS domain (default: 76)
  --host ADDRESS            HTTP bind address (default: 0.0.0.0 for phone access)
  --port N                  HTTP port (default: 8790)
  --markers-file PATH       Marker observation poses in the active map frame
  --workspace PATH          Built Team2 workspace root
  --use-sim-time            Use Gazebo /clock (default)
  --no-use-sim-time         Use real time; ROS network discovery enabled
  -h, --help                Show this help

Examples:
  scripts/run_web_mission.sh
  scripts/run_web_mission.sh --domain-id 82 --port 8792
  scripts/run_web_mission.sh --domain-id 16 --no-use-sim-time --markers-file /path/real_markers.yaml

SMART_VISION_WEB_DOMAIN_ID/HOST/PORT/WORKSPACE/MARKERS_FILE override defaults.
SMART_VISION_WEB_LOCALHOST_ONLY=0 or 1 overrides ROS discovery scope.
The server prints the browser address and pairing code. No build runs here.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --domain-id|--host|--port|--markers-file|--workspace)
      if [[ $# -lt 2 || -z "$2" ]]; then
        printf 'Missing value for %s\n' "$1" >&2
        exit 2
      fi
      case "$1" in
        --domain-id) DOMAIN_ID=$2 ;;
        --host) HTTP_HOST=$2 ;;
        --port) HTTP_PORT=$2 ;;
        --markers-file) MARKERS_FILE=$2 ;;
        --workspace) WORKSPACE_ROOT=$2 ;;
      esac
      shift 2
      ;;
    --use-sim-time) USE_SIM_TIME=true; shift ;;
    --no-use-sim-time) USE_SIM_TIME=false; shift ;;
    --) shift; SERVER_ARGUMENTS+=("$@"); break ;;
    *) SERVER_ARGUMENTS+=("$1"); shift ;;
  esac
done

if [[ ! "$DOMAIN_ID" =~ ^[0-9]{1,3}$ ]] || (( 10#$DOMAIN_ID > 232 )); then
  printf 'ROS domain must be an integer from 0 to 232.\n' >&2
  exit 2
fi
if [[ ! "$HTTP_PORT" =~ ^[0-9]{1,5}$ ]] || (( 10#$HTTP_PORT < 1 || 10#$HTTP_PORT > 65535 )); then
  printf 'HTTP port must be an integer from 1 to 65535.\n' >&2
  exit 2
fi
DOMAIN_ID=$((10#$DOMAIN_ID))
HTTP_PORT=$((10#$HTTP_PORT))
if [[ ! -f "$MARKERS_FILE" ]]; then
  printf 'Marker configuration does not exist: %s\n' "$MARKERS_FILE" >&2
  exit 2
fi
MARKERS_FILE=$(realpath -- "$MARKERS_FILE")

# sim_env rebuilds overlay paths and selects its own default domain. Restore the
# requested web mission domain afterwards, before any node is started.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
WORKSPACE_ROOT=$SMART_VISION_SIM_WORKSPACE
export ROS_DOMAIN_ID="$DOMAIN_ID"
if [[ "$USE_SIM_TIME" == true ]]; then
  export ROS_LOCALHOST_ONLY=${SMART_VISION_WEB_LOCALHOST_ONLY:-1}
  CLOCK_ARGUMENT=--use-sim-time
else
  export ROS_LOCALHOST_ONLY=${SMART_VISION_WEB_LOCALHOST_ONLY:-0}
  CLOCK_ARGUMENT=--no-use-sim-time
fi
if [[ "$ROS_LOCALHOST_ONLY" != 0 && "$ROS_LOCALHOST_ONLY" != 1 ]]; then
  printf 'SMART_VISION_WEB_LOCALHOST_ONLY must be 0 or 1.\n' >&2
  exit 2
fi

PACKAGE_PREFIX=$(ros2 pkg prefix smart_vision_amr)
if [[ ! -x "$PACKAGE_PREFIX/lib/smart_vision_amr/web_mission_server" ]]; then
  printf 'The new web_mission_server executable is not built yet. Run once:\n' >&2
  printf '  cd %q\n' "$WORKSPACE_ROOT" >&2
  printf '  colcon build --packages-select smart_vision_amr --symlink-install\n' >&2
  printf '  source install/local_setup.bash\n' >&2
  printf 'Then run this script again.\n' >&2
  exit 1
fi
printf 'Web mission: domain=%s localhost_only=%s use_sim_time=%s bind=%s:%s\n' \
  "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$USE_SIM_TIME" "$HTTP_HOST" "$HTTP_PORT"
exec ros2 run smart_vision_amr web_mission_server \
  --host "$HTTP_HOST" --port "$HTTP_PORT" --markers-file "$MARKERS_FILE" \
  "$CLOCK_ARGUMENT" "${SERVER_ARGUMENTS[@]}"
