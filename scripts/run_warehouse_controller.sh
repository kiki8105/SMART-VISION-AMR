#!/usr/bin/env bash
# PyQt manual input -> existing warehouse web lease -> motion_mux, never /cmd_vel.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE=$(cd -- "$REPO/../.." && pwd)
CONTROLLER=${SMART_VISION_CONTROLLER_SCRIPT:-$REPO/tools/cmd_vel_controller.py}
WEB_PORT=${SMART_VISION_WAREHOUSE_PORT:-8793}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: scripts/run_warehouse_controller.sh [CONTROLLER_OPTIONS ...]
Start the PyQt controller for the running warehouse (domain 178, web 8793).
Manual input goes through the web's existing priority/lease handover.
Arrow keys: hold to drive; release to hand control back. Space/Escape: ESTOP.
Start run_warehouse_sim.sh and run_warehouse_web.sh first.
This wrapper does not start another web server or publish directly to /cmd_vel.
EOF
  exit 0
fi
if [[ ! "$WEB_PORT" =~ ^[0-9]{1,5}$ ]] || ((10#$WEB_PORT < 1 || 10#$WEB_PORT > 65535)); then
  printf 'Invalid warehouse web port.\n' >&2; exit 2
fi
if [[ ! -f "$CONTROLLER" ]]; then printf 'Controller not found: %s\n' "$CONTROLLER" >&2; exit 2; fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/smart-vision-warehouse-controller-178.lock"
if ! flock -n 9; then printf 'Warehouse controller is already open.\n' >&2; exit 2; fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 TURTLEBOT3_MODEL=waffle_pi
source "$SCRIPT_DIR/dds_env.bash" ""
/usr/bin/python3 - "$WEB_PORT" <<'PY'
import http.client, json, sys
client = http.client.HTTPConnection('127.0.0.1', int(sys.argv[1]), timeout=2.)
try:
    client.request('GET', '/api/state')
    response = client.getresponse()
    data = json.loads(response.read())
    connection = data.get('connection', {})
    valid = (response.status == 200
             and response.getheader('Server', '').startswith('SmartVisionMission/1.0')
             and connection.get('domain_id') == 178
             and connection.get('use_sim_time') is True
             and connection.get('access_mode') == 'domain'
             and isinstance(data.get('manual'), dict)
             and isinstance(data.get('safety'), dict))
    if not valid:
        raise ValueError('Expected the domain-178 warehouse simulation web server')
except (OSError, ValueError) as exc:
    raise SystemExit('물류 웹 연결을 확인하세요: '+str(exc)+'\n터미널 2에서 ./scripts/run_warehouse_web.sh를 먼저 실행하세요.')
finally:
    client.close()
PY

BRIDGE_PID=
GUI_PID=
cleanup() {
  trap '' INT TERM HUP
  trap - EXIT
  for pid in "$GUI_PID" "$BRIDGE_PID"; do
    if [[ -n "$pid" ]]; then kill -TERM "$pid" 2>/dev/null || true; fi
  done
  for pid in "$GUI_PID" "$BRIDGE_PID"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'Warehouse controller: domain 178 / web %s / manual through existing web lease.\n' "$WEB_PORT"
/usr/bin/python3 "$REPO/tools/controller_manual_bridge.py" --domain-id 178 --web-port "$WEB_PORT" &
BRIDGE_PID=$!
/usr/bin/python3 "$CONTROLLER" \
  --domain-id 178 --robot-namespace / --use-sim-time --manual-via-web \
  --settings-file "$HOME/.config/turtlebot3/cmd_vel_controller_warehouse_178.ini" \
  --camera-topic /camera/image_raw/compressed --navigation-profile team-astar \
  --navigation-cancel-service /smart_vision_amr/cancel_navigation \
  "$@" --ros-args -r /goal_pose:=/smart_vision_amr/goal -r /plan:=/smart_vision_amr/path &
GUI_PID=$!
while kill -0 "$GUI_PID" 2>/dev/null; do
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    printf 'Manual bridge stopped; closing this controller.\n' >&2; exit 1
  fi
  sleep .3
done
status=0
wait "$GUI_PID" || status=$?
exit "$status"
