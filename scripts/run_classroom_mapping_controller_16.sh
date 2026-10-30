#!/usr/bin/env bash
# Physical PyQt manual control via the local web server's motion lease.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: run_classroom_mapping_controller_16.sh
Start the physical classroom controller with manual drive through the local
web server (port 8790) and motion_mux. Start the robot bringup first. A missing
local domain-16 web server is started here and stopped when the controller exits.
This controller never publishes directly to /cmd_vel.
EOF
  exit 0
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
if [[ "${SMART_VISION_ESTIMATION_LOCATION:-robot}" != robot ]]; then
  printf 'Mapping controller requires robot-side estimation/motion_mux. Set SMART_VISION_ESTIMATION_LOCATION=robot.\n' >&2
  exit 2
fi
export SMART_VISION_ESTIMATION_LOCATION=robot
# Match the UDP DDS profile used by the physical bringup and web bridge.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/dds_env.bash" ""
WEB_PORT=${SMART_VISION_WEB_PORT:-8790}
if [[ ! "$WEB_PORT" =~ ^[0-9]{1,5}$ ]] || (( 10#$WEB_PORT < 1 || 10#$WEB_PORT > 65535 )); then
  printf 'SMART_VISION_WEB_PORT must be an integer from 1 to 65535.\n' >&2
  exit 2
fi
WEB_PORT=$((10#$WEB_PORT))

# A listener on another interface still owns the port. Never start a second
# server or send manual commands to a different application/domain.
check_web() {
  /usr/bin/python3 - "$WEB_PORT" <<'PY'
import http.client
import json
import socket
import sys

port = int(sys.argv[1])
try:
    conn = socket.create_connection(('127.0.0.1', port), timeout=.35)
except ConnectionRefusedError:
    conn = None
except OSError:
    sys.exit(4)
if conn is None:
    probe = socket.socket()
    try:
        probe.bind(('0.0.0.0', port))
    except OSError:
        sys.exit(4)
    finally:
        probe.close()
    sys.exit(3)  # Port is free: this wrapper may start the web server.
conn.close()
try:
    client = http.client.HTTPConnection('127.0.0.1', port, timeout=.7)
    client.request('GET', '/api/state')
    response = client.getresponse()
    payload = json.loads(response.read())
    valid = (response.status == 200
             and response.getheader('Server', '').startswith('SmartVisionMission/1.0')
             and payload.get('connection', {}).get('domain_id') == 16
             and payload.get('connection', {}).get('use_sim_time') is False
             and payload.get('connection', {}).get('access_mode') == 'domain'
             and isinstance(payload.get('manual'), dict)
             and isinstance(payload.get('safety'), dict))
    client.close()
except (OSError, ValueError, KeyError, TypeError):
    valid = False
sys.exit(0 if valid else 4)
PY
}

web_pid=
bridge_pid=
cleanup() {
  if [[ -n "$bridge_pid" ]]; then
    kill -TERM "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
  if [[ -n "$web_pid" ]]; then
    kill -TERM -- "-$web_pid" 2>/dev/null || true
    wait "$web_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if check_web; then
  printf 'Existing local domain-16 classroom web server reused.\n'
else
  result=$?
  if (( result != 3 )); then
    printf 'Port %s is occupied by an unknown or wrong-domain web server; controller not started.\n' "$WEB_PORT" >&2
    exit 1
  fi
  web_log=$(mktemp "/tmp/smart_vision_controller_web_16.XXXXXX.log")
  setsid "$SCRIPT_DIR/run_classroom_web_16.sh" >"$web_log" 2>&1 &
  web_pid=$!
  web_ready=false
  for _ in {1..100}; do
    if check_web; then
      web_ready=true
      break
    else
      # Binding the port can precede the first complete /api/state response.
      # A temporary invalid response during startup is not an ownership error
      # as long as the web process we just started is still alive.
      if ! kill -0 "$web_pid" 2>/dev/null; then
        break
      fi
    fi
    sleep .2
  done
  if [[ "$web_ready" != true ]]; then
    printf 'Domain-16 web server did not start safely; log: %s\n' "$web_log" >&2
    tail -n 15 "$web_log" >&2 || true
    exit 1
  fi
  printf 'Local domain-16 classroom web server started (log: %s).\n' "$web_log"
fi
/usr/bin/python3 "$REPO_DIR/tools/controller_manual_bridge.py" --domain-id 16 \
  --web-port "$WEB_PORT" &
bridge_pid=$!
"$SCRIPT_DIR/run_classroom_controller_16.sh" --manual-via-web "$@"
