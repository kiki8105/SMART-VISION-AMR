#!/usr/bin/env bash
# Disposable, loopback-only Gazebo trial. Never touches the physical domain 16.
set -eo pipefail

REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE=$(cd -- "$REPO/../.." && pwd)
DOMAIN=176
MASTER_PORT=11576
MODE=baseline
PROFILE=clean
OBSTACLE=none
GOAL_X=-0.25
GOAL_Y=-1.98
GOAL_YAW=-1.40
INITIAL_X=-0.188431
INITIAL_Y=-0.220497
INITIAL_YAW=-0.027514
TIMEOUT=150
OBSTACLE_HOLD_TIMEOUT=15
OUTPUT=
WORLD=$REPO/ros2_ws/src/smart_vision_amr/worlds/classroom_20261030_replay.world
MAP=$REPO/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml
NAV_PARAMS_SOURCE=$REPO/ros2_ws/src/smart_vision_amr/config/curve_tuning_waffle_pi.yaml
NODES_PARAMS_SOURCE=$REPO/ros2_ws/src/smart_vision_amr/config/classroom_hardware_nodes.yaml
STRESS=false
ALLOW_SAFE_HOLD=false
MISSION_ROUTE=false
ROUTE_PREFIX=
SECTOR1_CANDIDATE=false
SECTOR1_ORIGINAL=false
HOME_OBSTACLE=false
ROUTE_OBSTACLE=false
SINGLE_GOAL=()
NO_VERIFIER=false
LIFETIME=240
RECORD_INCIDENTS=false
EXPECT_SUSTAINED_OUTAGE_HOLD=false
TF_SOURCE_BEST_EFFORT=false
LASER_FILTER_FOR_CHANGE_MEMORY=false
MEMORY_SESSION=false
LASER_FILTER_PREFIX=${SMART_VISION_LASER_FILTER_PREFIX:-}
GAZEBO_SEED=4242
MEASURE_SCAN_FILTER=false

usage() {
  cat <<'EOF'
Usage: run_classroom_fault_sim.sh [--mode baseline|ekf_independent]
       [--profile clean|jitter|outages|mixed] [--obstacle none|moving]
       [--world WORLD] [--stress] [--map MAP] [--goal-x X] [--goal-y Y]
       [--goal-yaw RADIANS] [--timeout SECONDS] [--allow-safe-hold]
       [--initial-x X] [--initial-y Y] [--initial-yaw RADIANS]
       [--obstacle-hold-timeout SECONDS]  # route verifier: default 15; 0 disables early hold verdict
       [--mission-route]  # Sector 1 → 2 → 3 → Home, timeout per leg
       [--route-prefix 1|2|3]  # first N sectors, no Home
       [--home-obstacle]  # inject an unmapped box on the active Home route
       [--route-obstacle]  # with --route-prefix, inject on its last sector
       [--sector1-candidate]  # isolated experimental observation pose
       [--sector1-original]  # registered Sector 1 as an isolated goal
       [--single-goal NAME X Y YAW]  # isolated observation-pose candidate
       [--no-verifier --lifetime SECONDS]  # keep isolated Gazebo running for external web test
       [--record-incidents]  # enable high-CPU flight recorder diagnostics
       [--tf-source-best-effort]  # sim-only raw TF QoS A/B; forwarded /tf stays reliable
       [--laser-filter-for-change-memory]  # weak speckle filter on A* change_memory input only
       [--memory-session]  # remember confirmed cells only while this navigation process runs
       [--measure-scan-filter]  # equal raw/filtered scan observer in both A/B trials
       [--laser-filter-prefix PATH]  # optional extracted ROS Humble laser_filters prefix
       [--gazebo-seed INTEGER]  # fixed 4242 by default for comparable A/B scans
       [--expect-sustained-outage-hold]  # profile outages: safety latch, no auto-resume
       [--nav-params-source YAML]  # Nav2 source profile, default classroom YAML
       [--master-port PORT]  # dedicated localhost Gazebo master, default 11576
       [--output DIRECTORY]

Creates a copied SDF/world/map and runs Gazebo on localhost ROS domain 176.
Independent EKF mode also runs the physical-style sensor guard and a latched
motion mux. An explicit operator reset is required after healthy inputs;
the runner never clears that latch merely to make a trial move.
The copied Waffle Pi sensor outputs pass through a deterministic fault relay;
ROS message contents and original stamps are preserved. The trial writes
fixture_manifest.json, sensor_faults.jsonl, acceptance.json and launch logs.
Use a fresh --output path for each run. Normal robot domain 16 is never used.
EOF
}

while (($#)); do
  case "$1" in
    --stress) STRESS=true; shift;;
    --allow-safe-hold) ALLOW_SAFE_HOLD=true; shift;;
    --mission-route) MISSION_ROUTE=true; shift;;
    --route-prefix)
      if (($# < 2)); then usage >&2; exit 2; fi
      MISSION_ROUTE=true
      ROUTE_PREFIX=$2
      shift 2;;
    --home-obstacle) HOME_OBSTACLE=true; shift;;
    --route-obstacle) ROUTE_OBSTACLE=true; shift;;
    --no-verifier) NO_VERIFIER=true; shift;;
    --record-incidents) RECORD_INCIDENTS=true; shift;;
    --tf-source-best-effort) TF_SOURCE_BEST_EFFORT=true; shift;;
    --laser-filter-for-change-memory) LASER_FILTER_FOR_CHANGE_MEMORY=true; shift;;
    --memory-session) MEMORY_SESSION=true; shift;;
    --measure-scan-filter) MEASURE_SCAN_FILTER=true; shift;;
    --expect-sustained-outage-hold) EXPECT_SUSTAINED_OUTAGE_HOLD=true; shift;;
    --sector1-candidate) MISSION_ROUTE=true; SECTOR1_CANDIDATE=true; shift;;
    --sector1-original) MISSION_ROUTE=true; SECTOR1_ORIGINAL=true; shift;;
    --single-goal)
      if (($# < 5)); then usage >&2; exit 2; fi
      MISSION_ROUTE=true
      SINGLE_GOAL=("$2" "$3" "$4" "$5")
      shift 5;;
    --mode|--profile|--obstacle|--world|--map|--goal-x|--goal-y|--goal-yaw|--initial-x|--initial-y|--initial-yaw|--timeout|--obstacle-hold-timeout|--output|--nav-params-source|--master-port|--lifetime|--laser-filter-prefix|--gazebo-seed)
      if (($# < 2)); then usage >&2; exit 2; fi
      case "$1" in
        --mode) MODE=$2;; --profile) PROFILE=$2;; --obstacle) OBSTACLE=$2;;
        --world) WORLD=$2;; --map) MAP=$2;; --goal-x) GOAL_X=$2;;
        --goal-y) GOAL_Y=$2;; --goal-yaw) GOAL_YAW=$2;;
        --initial-x) INITIAL_X=$2;; --initial-y) INITIAL_Y=$2;;
        --initial-yaw) INITIAL_YAW=$2;;
        --timeout) TIMEOUT=$2;; --output) OUTPUT=$2;;
        --obstacle-hold-timeout) OBSTACLE_HOLD_TIMEOUT=$2;;
        --nav-params-source) NAV_PARAMS_SOURCE=$2;;
        --master-port) MASTER_PORT=$2;;
        --lifetime) LIFETIME=$2;;
        --laser-filter-prefix) LASER_FILTER_PREFIX=$2;;
        --gazebo-seed) GAZEBO_SEED=$2;;
      esac
      shift 2;;
    -h|--help) usage; exit 0;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2;;
  esac
done
case "$MODE" in baseline|ekf_independent) ;; *) usage >&2; exit 2;; esac
case "$PROFILE" in clean|jitter|outages|mixed) ;; *) usage >&2; exit 2;; esac
case "$OBSTACLE" in none|moving) ;; *) usage >&2; exit 2;; esac
if [[ ! "$LIFETIME" =~ ^[0-9]+$ ]] || ((LIFETIME < 10 || LIFETIME > 3600)); then
  printf 'Gazebo lifetime must be 10 to 3600 seconds.\n' >&2
  exit 2
fi
if [[ ! "$OBSTACLE_HOLD_TIMEOUT" =~ ^[0-9]+$ ]] ||
   ((OBSTACLE_HOLD_TIMEOUT > 3600)); then
  printf '%s\n' '--obstacle-hold-timeout must be an integer from 0 to 3600 seconds.' >&2
  exit 2
fi
if [[ ! "$MASTER_PORT" =~ ^[0-9]+$ ]] || ((MASTER_PORT < 1024 || MASTER_PORT > 65535)); then
  printf 'Gazebo master port must be an integer from 1024 to 65535.\n' >&2
  exit 2
fi
if [[ ! "$GAZEBO_SEED" =~ ^[0-9]+$ ]] || ((GAZEBO_SEED > 2147483647)); then
  printf 'Gazebo seed must be an integer from 0 to 2147483647.\n' >&2
  exit 2
fi
MASTER=http://127.0.0.1:$MASTER_PORT
if (( ${#SINGLE_GOAL[@]} > 0 )) && [[ "$SECTOR1_CANDIDATE" == true || "$SECTOR1_ORIGINAL" == true ]]; then
  printf '%s\n' '--single-goal cannot be combined with Sector 1 shortcuts.' >&2
  exit 2
fi
if [[ "$SECTOR1_CANDIDATE" == true && "$SECTOR1_ORIGINAL" == true ]]; then
  printf '%s\n' '--sector1-candidate and --sector1-original are mutually exclusive.' >&2
  exit 2
fi
if [[ -n "$ROUTE_PREFIX" ]]; then
  if [[ ! "$ROUTE_PREFIX" =~ ^[123]$ || "$SECTOR1_CANDIDATE" == true ||
        "$SECTOR1_ORIGINAL" == true || ${#SINGLE_GOAL[@]} -gt 0 ]]; then
    printf '%s\n' '--route-prefix accepts 1, 2, or 3 and cannot be combined with single-goal shortcuts.' >&2
    exit 2
  fi
fi
if [[ "$HOME_OBSTACLE" == true && ( "$MISSION_ROUTE" != true || "$SECTOR1_CANDIDATE" == true || "$SECTOR1_ORIGINAL" == true || ${#SINGLE_GOAL[@]} -gt 0 ) ]]; then
  printf '%s\n' '--home-obstacle requires the full four-stop --mission-route.' >&2
  exit 2
fi
if [[ "$HOME_OBSTACLE" == true && -n "$ROUTE_PREFIX" ]]; then
  printf '%s\n' '--home-obstacle requires the full four-stop route.' >&2
  exit 2
fi
if [[ "$ROUTE_OBSTACLE" == true && ( -z "$ROUTE_PREFIX" || "$HOME_OBSTACLE" == true ) ]]; then
  printf '%s\n' '--route-obstacle requires --route-prefix 1, 2, or 3.' >&2
  exit 2
fi
if [[ "$NO_VERIFIER" == true && ( "$MISSION_ROUTE" == true || "$HOME_OBSTACLE" == true ||
      "$ROUTE_OBSTACLE" == true || "$OBSTACLE" != none || "$PROFILE" != clean ) ]]; then
  printf '%s\n' '--no-verifier is only for a web-controlled clean replay.' >&2
  exit 2
fi
if [[ "$EXPECT_SUSTAINED_OUTAGE_HOLD" == true &&
      ( "$PROFILE" != outages || "$OBSTACLE" != none || "$STRESS" == true ||
        "$ALLOW_SAFE_HOLD" == true || "$MISSION_ROUTE" == true || "$NO_VERIFIER" == true ) ]]; then
  printf '%s\n' '--expect-sustained-outage-hold requires the single-goal outages profile.' >&2
  exit 2
fi
if [[ "$MISSION_ROUTE" == true && ( "$OBSTACLE" != none || "$PROFILE" != clean || "$ALLOW_SAFE_HOLD" == true ) ]]; then
  printf 'The four-stop route requires --profile clean --obstacle none without --allow-safe-hold.\n' >&2
  exit 2
fi
if [[ "$STRESS" == true ]]; then
  WORLD=$REPO/ros2_ws/src/smart_vision_amr/worlds/classroom_20261030_stress.world
fi
if [[ "$TF_SOURCE_BEST_EFFORT" == true && "$MODE" != baseline ]]; then
  printf '%s\n' '--tf-source-best-effort only applies to baseline Gazebo TF relay.' >&2
  exit 2
fi
if [[ ! -s "$WORLD" || ! -s "$MAP" || ! -s "$NAV_PARAMS_SOURCE" || ! -s "$NODES_PARAMS_SOURCE" ]]; then
  printf 'World, map, or classroom hardware-like YAML missing.\n' >&2
  exit 2
fi
MODEL=$WORKSPACE/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf
if [[ ! -s "$MODEL" || ! -f "$WORKSPACE/install/local_setup.bash" ]]; then
  printf 'Build the Team2 simulation workspace first with scripts/setup_sim.sh.\n' >&2
  exit 2
fi
OUTPUT=${OUTPUT:-$REPO/benchmarks/classroom_fault_$(date +%Y%m%d_%H%M%S)_$$}
if [[ -e "$OUTPUT" ]]; then
  printf 'Refusing to overwrite trial output: %s\n' "$OUTPUT" >&2
  exit 2
fi
mkdir -p -- "$OUTPUT"
OUTPUT=$(cd -- "$OUTPUT" && pwd)
FIXTURE=$OUTPUT/fixture
/usr/bin/python3 "$REPO/scripts/prepare_classroom_fault_fixture.py" \
  --source-model "$MODEL" --world "$WORLD" --map "$MAP" \
  --mode "$MODE" --output "$FIXTURE"
NAV_PARAMS=$OUTPUT/classroom_nav_sim_time.yaml
NODES_PARAMS=$OUTPUT/classroom_nodes_sim.yaml
/usr/bin/python3 "$REPO/scripts/build_classroom_replay_nav_profile.py" \
  --source "$NAV_PARAMS_SOURCE" --nodes-source "$NODES_PARAMS_SOURCE" \
  --nodes-destination "$NODES_PARAMS" "$NAV_PARAMS"
/usr/bin/python3 - "$OUTPUT/runtime_manifest.json" "$MODE" "$NAV_PARAMS" "$NODES_PARAMS" \
  "$REPO/ros2_ws/src/smart_vision_amr/config/ekf_hardware.yaml" "$NAV_PARAMS_SOURCE" \
  "$NODES_PARAMS_SOURCE" \
  "$REPO/ros2_ws/src/smart_vision_amr/config/change_memory_speckle.yaml" \
  "$MASTER" "$RECORD_INCIDENTS" "$LASER_FILTER_FOR_CHANGE_MEMORY" "$GAZEBO_SEED" \
  "$MEASURE_SCAN_FILTER" "$MEMORY_SESSION" \
  "$INITIAL_X" "$INITIAL_Y" "$INITIAL_YAW" <<'PY'
import hashlib, json, sys
from pathlib import Path
(_, manifest_path, mode, nav_generated, nodes_generated, ekf_source,
 nav_source, nodes_source, filter_source, master, record_incidents,
 laser_filter, gazebo_seed, measure_scan_filter, memory_session,
 initial_x, initial_y, initial_yaw) = sys.argv
files = [Path(path) for path in (nav_generated, nodes_generated, ekf_source,
                                nav_source, nodes_source, filter_source)]
payload = {'domain': 176, 'localhost_only': True, 'mode': mode,
           'gazebo_master_uri': master,
           'config_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in files},
           'nav_profile_source': str(Path(nav_source).resolve()),
           'nodes_profile_source': str(Path(nodes_source).resolve()),
           'nav_profile_generated': str(Path(nav_generated).resolve()),
           'nodes_profile_generated': str(Path(nodes_generated).resolve()),
           'sim_footprint_overrides': {
               'robot_radius_m': 0.24,
               'nav2_inflation_radius_m': 0.26,
               'astar_static_inflation_cells': 5,
               'astar_preferred_clearance_m': 0.25,
               'follower_recovery_radius_m': 0.24,
           },
           'record_incidents': record_incidents == 'true',
           'laser_filter_for_change_memory': laser_filter == 'true',
           'laser_filter_input_topic': '/scan',
           'laser_filter_output_topic': '/scan_filtered_for_change',
           'laser_filter_consumer': 'change_memory only; raw /scan retained by Nav2, AMCL and motion_mux',
           'scan_pair_observer': measure_scan_filter == 'true',
           'memory_session': memory_session == 'true',
           'initial_pose': {'x': float(initial_x), 'y': float(initial_y),
                            'yaw': float(initial_yaw)},
           'gazebo_seed': int(gazebo_seed),
           'simulated_battery': ({'topic': '/battery_state', 'voltage_v': 12.0,
                                  'percentage': 0.75, 'rate_hz': 2.0,
                                  'guard_minimum_voltage_v': 10.8}
                                 if mode == 'ekf_independent' else None),
           'nav_profile_note': 'Physical Nav2 tuning copied with simulated time and Gazebo collision radius/inflation only.',
           'nodes_profile_note': 'Physical A*/follower tuning copied with Gazebo collision radius and 5-cell static inflation only.',
           'sensor_fault_scope': ('scan, Gazebo odom, Gazebo odom->base TF' if mode == 'baseline'
                                  else 'scan only; independent EKF publishes /odom and odom->base TF directly')}
Path(manifest_path).write_text(json.dumps(payload, indent=2)+'\n')
PY

# Do not source sim_env.bash: its domain 76 might have a separate running test.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH \
  FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.sh ]]; then source /usr/share/gazebo/setup.sh; fi
# shellcheck disable=SC1091
source "$WORKSPACE/install/local_setup.bash"
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1 GAZEBO_MASTER_URI=$MASTER
if [[ "$LASER_FILTER_FOR_CHANGE_MEMORY" == true ]]; then
  if [[ -n "$LASER_FILTER_PREFIX" ]]; then
    if [[ ! -f "$LASER_FILTER_PREFIX/share/laser_filters/package.xml" ]]; then
      printf 'Invalid laser_filters prefix: %s\n' "$LASER_FILTER_PREFIX" >&2
      exit 2
    fi
    export AMENT_PREFIX_PATH="$LASER_FILTER_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$LASER_FILTER_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  if ! FILTER_PACKAGE_PREFIX=$(ros2 pkg prefix laser_filters); then
    printf 'laser_filters is required for --laser-filter-for-change-memory.\n' >&2
    exit 2
  fi
  /usr/bin/python3 - "$OUTPUT/runtime_manifest.json" "$FILTER_PACKAGE_PREFIX" <<'PY'
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
data = json.loads(path.read_text())
data['laser_filter_package_prefix'] = sys.argv[2]
path.write_text(json.dumps(data, indent=2) + '\n')
PY
fi
installed_args=$(ros2 launch smart_vision_amr simulation.launch.py --show-args)
if [[ "$installed_args" != *robot_model_file* || "$installed_args" != *laser_filter_for_change_memory* ]]; then
  printf 'Installed simulation launch is stale; rebuild smart_vision_amr with colcon.\n' >&2
  exit 2
fi
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$REPO/gazebo_models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export ROS_LOG_DIR=$OUTPUT/ros_logs
mkdir -p -- "$ROS_LOG_DIR"

# This is a captured *offline* replay of 33 real scan cells, not a synthetic
# claim that Gazebo reproduced those exact returns. It guards the regression
# which previously blocked the current (32,53) footprint diagonally.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  "$REPO/ros2_ws/src/smart_vision_amr/test/test_live_obstacle_circular_inflation.py" \
  >"$OUTPUT/captured_33_hit_replay.log" 2>&1

# If the dedicated Gazebo master is already owned, do not join another trial.
/usr/bin/python3 - "$MASTER_PORT" <<'PY'
import socket, sys
with socket.socket() as sock:
    sock.bind(('127.0.0.1', int(sys.argv[1])))
PY

LAUNCH_PID=
RELAY_PID=
CONDITIONER_PID=
BATTERY_PID=
PAIR_PID=
VERIFY_PID=
cleanup() {
  trap '' INT TERM HUP
  trap - EXIT
  # Every PID below is the leader of a setsid group started by this isolated
  # runner. A stuck Nav2 lifecycle client can outlive ros2 launch and ignore
  # INT/TERM, so finish by reaping only these owned process groups.
  local pid tick alive
  local -a groups=()
  for pid in "$VERIFY_PID" "$LAUNCH_PID" "$RELAY_PID" "$CONDITIONER_PID" "$BATTERY_PID" "$PAIR_PID"; do
    if [[ -n "$pid" ]]; then groups+=("$pid"); fi
  done
  for pid in "${groups[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  for ((tick=0; tick<20; tick++)); do
    alive=false
    for pid in "${groups[@]}"; do
      if kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 0.1
  done
  for pid in "${groups[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for ((tick=0; tick<20; tick++)); do
    alive=false
    for pid in "${groups[@]}"; do
      if kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 0.1
  done
  for pid in "${groups[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  for pid in "${groups[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP

EKF_ARGS=()
if [[ "$MODE" == ekf_independent ]]; then
  /usr/bin/python3 - "$REPO/ros2_ws/src/smart_vision_amr/config/ekf_hardware.yaml" \
    "$OUTPUT/ekf_hardware_like_sim.yaml" <<'PY'
import sys, yaml
from pathlib import Path
config = yaml.safe_load(Path(sys.argv[1]).read_text())
params = config['ekf_hardware']['ros__parameters']
assert params['imu0_twist_rejection_threshold'] == 20.0
# The conditioner runs as its own node in this fixture. The same physical
# wheel/IMU fusion axes, covariance gate and 20-Hz EKF frequency are retained.
config = {'ekf_independent': {'ros__parameters': params}}
Path(sys.argv[2]).write_text(yaml.safe_dump(config, sort_keys=False))
PY
  EKF_ARGS+=("ekf_params_file:=$OUTPUT/ekf_hardware_like_sim.yaml"
             "ekf_wheel_params_file:=$REPO/ros2_ws/src/smart_vision_amr/config/ekf_hardware.yaml"
             "use_ekf_sensor_guard:=true"
             "minimum_battery_voltage:=10.8")
  setsid /usr/bin/python3 "$REPO/scripts/classroom_sim_battery.py" \
    --voltage 12.0 --rate 2.0 --ros-args -p use_sim_time:=true \
    >"$OUTPUT/sim_battery.log" 2>&1 &
  BATTERY_PID=$!
  setsid ros2 run smart_vision_amr imu_ekf_conditioner --ros-args \
    --params-file "$REPO/ros2_ws/src/smart_vision_amr/config/ekf_hardware.yaml" \
    -p use_sim_time:=true >"$OUTPUT/conditioner.log" 2>&1 &
  CONDITIONER_PID=$!
fi

TF_SOURCE_ARGS=()
if [[ "$TF_SOURCE_BEST_EFFORT" == true ]]; then
  TF_SOURCE_ARGS+=(--tf-source-best-effort)
fi
if [[ "$MODE" == ekf_independent ]]; then
  TF_SOURCE_ARGS+=(--scan-only)
fi
setsid /usr/bin/python3 "$REPO/scripts/classroom_fault_relay.py" \
  --domain "$DOMAIN" --output "$OUTPUT/sensor_faults.jsonl" \
  "${TF_SOURCE_ARGS[@]}" \
  >"$OUTPUT/fault_relay.log" 2>&1 &
RELAY_PID=$!

if [[ "$MEASURE_SCAN_FILTER" == true ]]; then
  setsid /usr/bin/python3 "$REPO/scripts/measure_scan_filter.py" \
    --output "$OUTPUT/scan_filter_pair.json" >"$OUTPUT/scan_filter_pair.log" 2>&1 &
  PAIR_PID=$!
fi

setsid ros2 launch smart_vision_amr simulation.launch.py \
  "world:=$FIXTURE/classroom_fault.world" "map:=$FIXTURE/classroom_fault.yaml" \
  "robot_model_file:=$FIXTURE/waffle_pi_fault.sdf" \
  "nav_params_file:=$NAV_PARAMS" "nodes_params_file:=$NODES_PARAMS" \
  "x_pose:=$INITIAL_X" "y_pose:=$INITIAL_Y" "yaw:=$INITIAL_YAW" \
  "initial_x:=$INITIAL_X" "initial_y:=$INITIAL_Y" "initial_yaw:=$INITIAL_YAW" \
  "localization_mode:=$MODE" "camera_layout:=classroom_dual" \
  "gazebo_seed:=$GAZEBO_SEED" \
  "laser_filter_for_change_memory:=$LASER_FILTER_FOR_CHANGE_MEMORY" \
  "memory_enabled:=$MEMORY_SESSION" "memory_persist_to_disk:=false" \
  "gui:=false" "use_rviz:=false" "record_incidents:=$RECORD_INCIDENTS" \
  "${EKF_ARGS[@]}" >"$OUTPUT/gazebo_launch.log" 2>&1 &
LAUNCH_PID=$!

if [[ "$NO_VERIFIER" == true ]]; then
  printf 'Headless Gazebo readying in isolated ROS domain %s for up to %ss; external web test owns goals.\n' "$DOMAIN" "$LIFETIME"
  end_at=$((SECONDS + LIFETIME))
  while ((SECONDS < end_at)); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      printf 'Gazebo launch exited before the web-controlled trial completed.\n' >&2
      exit 1
    fi
    sleep 1
  done
  exit 0
fi

VERIFY_ARGS=()
if [[ "$ALLOW_SAFE_HOLD" == true ]]; then VERIFY_ARGS+=(--allow-safe-hold); fi
if [[ "$STRESS" == true ]]; then VERIFY_ARGS+=(--stress); fi
if [[ "$EXPECT_SUSTAINED_OUTAGE_HOLD" == true ]]; then
  VERIFY_ARGS+=(--expect-sustained-outage-hold)
fi
set +e
if [[ "$MISSION_ROUTE" == true ]]; then
  ROUTE_ARGS=()
  if [[ "$SECTOR1_CANDIDATE" == true ]]; then
    ROUTE_ARGS+=(--single-goal Sector1Candidate 1.20 0.55 3.114078)
  fi
  if [[ "$SECTOR1_ORIGINAL" == true ]]; then
    ROUTE_ARGS+=(--single-goal Sector1Registered 1.475000 0.575000 3.114078)
  fi
  if (( ${#SINGLE_GOAL[@]} > 0 )); then
    ROUTE_ARGS+=(--single-goal "${SINGLE_GOAL[@]}")
  fi
  if [[ -n "$ROUTE_PREFIX" ]]; then ROUTE_ARGS+=(--route-prefix "$ROUTE_PREFIX"); fi
  if [[ "$HOME_OBSTACLE" == true ]]; then ROUTE_ARGS+=(--home-obstacle); fi
  if [[ "$ROUTE_OBSTACLE" == true ]]; then ROUTE_ARGS+=(--route-obstacle); fi
  setsid /usr/bin/python3 "$REPO/scripts/verify_classroom_route.py" \
    --domain "$DOMAIN" --output "$OUTPUT/acceptance.json" \
    --leg-timeout "$TIMEOUT" --obstacle-hold-timeout "$OBSTACLE_HOLD_TIMEOUT" \
    "${ROUTE_ARGS[@]}" >"$OUTPUT/acceptance.log" 2>&1 &
else
  setsid /usr/bin/python3 "$REPO/scripts/verify_classroom_faults.py" \
    --domain "$DOMAIN" --output "$OUTPUT/acceptance.json" \
    --profile "$PROFILE" --obstacle "$OBSTACLE" \
    --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --goal-yaw "$GOAL_YAW" \
    --timeout "$TIMEOUT" "${VERIFY_ARGS[@]}" >"$OUTPUT/acceptance.log" 2>&1 &
fi
VERIFY_PID=$!
wait "$VERIFY_PID"
STATUS=$?
if [[ -s "$OUTPUT/acceptance.json" && -s "$OUTPUT/sensor_faults.jsonl" ]]; then
  /usr/bin/python3 "$REPO/scripts/assess_classroom_fault_delivery.py" \
    --profile "$PROFILE" --mode "$MODE" \
    --relay "$OUTPUT/sensor_faults.jsonl" --acceptance "$OUTPUT/acceptance.json" \
    >"$OUTPUT/fault_delivery.log" 2>&1
  if (($? != 0)); then STATUS=1; fi
else
  STATUS=1
fi
set -e
cat "$OUTPUT/acceptance.log"
if [[ -s "$OUTPUT/fault_delivery.log" ]]; then cat "$OUTPUT/fault_delivery.log"; fi
printf 'Trial evidence: %s\n' "$OUTPUT"
exit "$STATUS"
