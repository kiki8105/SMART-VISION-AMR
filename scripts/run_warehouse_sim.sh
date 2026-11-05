#!/usr/bin/env bash
# Warehouse geometry, physical tuning and moving models; operator owns all goals.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE=$(cd -- "$REPO/../.." && pwd)
PACKAGE=$REPO/ros2_ws/src/smart_vision_amr
LIFETIME=3600
OUTPUT=
MOVING=true

usage() {
  cat <<'EOF'
Usage: scripts/run_warehouse_sim.sh [--lifetime 10..3600] [--output NEW_DIRECTORY]
                                    [--static-obstacles]

Start the 10-sector logistics warehouse on ROS domain 178 (localhost only),
Gazebo master http://127.0.0.1:11578. Physical 0.22 m tuning, independent EKF,
rear camera and in-session obstacle memory are retained. AMCL starts without
an initial pose: the web operator sets the registered warehouse Home.
Moving workers/carts are enabled; --static-obstacles leaves them stationary.
No goal or mission is automatically dispatched. Start run_warehouse_web.sh
and run_warehouse_view.sh in separate terminals. Ctrl+C stops only this run.
EOF
}

while (($#)); do
  case "$1" in
    --lifetime|--output)
      if (($# < 2)); then usage >&2; exit 2; fi
      if [[ "$1" == --lifetime ]]; then LIFETIME=$2; else OUTPUT=$2; fi
      shift 2;;
    --static-obstacles) MOVING=false; shift;;
    -h|--help) usage; exit 0;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2;;
  esac
done
if [[ ! "$LIFETIME" =~ ^[0-9]+$ ]] || ((LIFETIME < 10 || LIFETIME > 3600)); then
  printf 'Lifetime must be 10 to 3600 seconds.\n' >&2; exit 2
fi
# A duplicate wrapper must fail before any moving-model controller starts.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/smart-vision-warehouse-178.lock"
if ! flock -n 9; then
  printf 'Warehouse domain 178 is already owned by another run.\n' >&2; exit 2
fi
/usr/bin/python3 - <<'PY'
import socket
try:
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 11578))
except OSError as exc:
    raise SystemExit('Warehouse master 11578 is already in use: '+str(exc))
PY
OUTPUT=${OUTPUT:-/tmp/warehouse-demo-$(date +%Y%m%d-%H%M%S)-$$}
if [[ -e "$OUTPUT" ]]; then
  printf 'Choose a new output directory: %s already exists.\n' "$OUTPUT" >&2; exit 2
fi
# Check, rather than silently regenerate or rewrite the checked-in assets.
if ! ASSET_CHECK=$(/usr/bin/python3 "$SCRIPT_DIR/generate_warehouse_demo.py" --check 2>&1); then
  printf '%s\n' "$ASSET_CHECK" >&2; exit 1
fi
printf 'Warehouse assets checked: 10 sectors, Home, matching occupancy map and actor routes.\n'
HOME_DATA=$(/usr/bin/python3 - "$PACKAGE/config/aruco_warehouse_markers.yaml" <<'PY'
import math, sys, yaml
data = yaml.safe_load(open(sys.argv[1]))
home = next(marker for marker in data['markers'] if marker['id'] == 0)
values = [float(home[key]) for key in ('x', 'y', 'yaw')]
if not all(math.isfinite(value) for value in values):
    raise SystemExit('Warehouse Home must have finite x, y and yaw')
print(*values)
PY
)
read -r HOME_X HOME_Y HOME_YAW <<< "$HOME_DATA"

# The wrapper needs the same overlay for the actor process. The simulator
# runner also rebuilds its own environment independently.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11578
export FASTRTPS_DEFAULT_PROFILES_FILE="$PACKAGE/config/dds_udp_localhost.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"

SIM_PID=
ACTOR_PID=
cleanup() {
  trap '' INT TERM HUP
  trap - EXIT
  local pid tick alive
  for pid in "$ACTOR_PID" "$SIM_PID"; do
    if [[ -n "$pid" ]]; then kill -INT -- "-$pid" 2>/dev/null || true; fi
  done
  for ((tick=0; tick<60; tick++)); do
    alive=false
    for pid in "$ACTOR_PID" "$SIM_PID"; do
      if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 0.1
  done
  for pid in "$ACTOR_PID" "$SIM_PID"; do
    if [[ -n "$pid" ]]; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

printf 'Warehouse: 10 sectors / domain 178 / master 11578 / moving models %s\n' "$MOVING"
printf 'Logs: %s\n' "$OUTPUT"
setsid "$SCRIPT_DIR/run_classroom_fault_sim.sh" \
  --domain 178 --master-port 11578 \
  --mode ekf_independent --physical-footprint --hardware-profile sensor_rates \
  --memory-session --cold-start --no-verifier --lifetime "$LIFETIME" \
  --world "$PACKAGE/worlds/warehouse_logistics.world" \
  --map "$PACKAGE/maps/warehouse_logistics.yaml" \
  --initial-x "$HOME_X" --initial-y "$HOME_Y" --initial-yaw "$HOME_YAW" \
  --nav-params-source "$PACKAGE/config/classroom_demo_operator_20261104.yaml" \
  --nodes-params-source "$PACKAGE/config/classroom_hardware_nodes.yaml" \
  --output "$OUTPUT" &
SIM_PID=$!

if [[ "$MOVING" == true ]]; then
  # The runner writes this only after its own master preflight and launch.
  # A directory alone is not evidence that this run owns a Gazebo launch.
  for ((tick=0; tick<1200; tick++)); do
    if [[ -s "$OUTPUT/owned_launch.pid" ]]; then break; fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then wait "$SIM_PID"; exit $?; fi
    sleep 0.1
  done
  if [[ ! -s "$OUTPUT/owned_launch.pid" ]]; then
    printf 'Simulator did not start its owned launch.\n' >&2; exit 1
  fi
  read -r OWNED_LAUNCH_PID <"$OUTPUT/owned_launch.pid"
  if [[ ! "$OWNED_LAUNCH_PID" =~ ^[0-9]+$ ]] ||
      ! kill -0 "$SIM_PID" 2>/dev/null || ! kill -0 "$OWNED_LAUNCH_PID" 2>/dev/null; then
    printf 'Owned simulator launch is no longer alive.\n' >&2; exit 1
  fi
  setsid /usr/bin/python3 "$SCRIPT_DIR/warehouse_dynamic_obstacles.py" \
    --domain 178 --routes "$PACKAGE/config/warehouse_dynamic_routes.yaml" \
    --output "$OUTPUT/dynamic_obstacles.json" \
    >"$OUTPUT/dynamic_obstacles.log" 2>&1 &
  ACTOR_PID=$!
fi

while kill -0 "$SIM_PID" 2>/dev/null; do
  if [[ -n "$ACTOR_PID" ]] && ! kill -0 "$ACTOR_PID" 2>/dev/null; then
    status=0
    wait "$ACTOR_PID" || status=$?
    printf 'Moving obstacle process exited (status %s); see %s/dynamic_obstacles.log\n' "$status" "$OUTPUT" >&2
    exit 1
  fi
  sleep 1
done
status=0
wait "$SIM_PID" || status=$?
exit "$status"
