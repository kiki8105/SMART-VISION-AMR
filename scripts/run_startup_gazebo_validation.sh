#!/usr/bin/env bash
# Build a disposable snapshot and run the finite, loopback-only acceptance trial.
set -eo pipefail
REPO=${SMART_VISION_VALIDATION_REPO:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}
if [[ "${SMART_VISION_VALIDATION_SNAPSHOT:-0}" != 1 ]]; then
  RUNNER=$(mktemp /tmp/smart_vision_startup_runner.XXXXXX.sh)
  cp -- "${BASH_SOURCE[0]}" "$RUNNER"
  SMART_VISION_VALIDATION_REPO="$REPO" SMART_VISION_VALIDATION_SNAPSHOT=1 exec bash "$RUNNER" "$@"
fi
MODE=${1:-baseline}
SCENARIO=${2:-suite}
AMCL_TF_PROBE=${SMART_VISION_AMCL_TF_BEST_EFFORT_PROBE:-0}
case "$AMCL_TF_PROBE" in 0|1) ;; *) exit 2 ;; esac
TF_DIAGNOSTICS=${SMART_VISION_TF_DIAGNOSTICS:-$AMCL_TF_PROBE}
case "$TF_DIAGNOSTICS" in 0|1) ;; *) exit 2 ;; esac
case "$MODE" in baseline|ekf_independent) ;; *) exit 2 ;; esac
case "$SCENARIO" in startup|suite|home_initialization|navigation|precision|local_detour|barrier) ;; *) exit 2 ;; esac
EVIDENCE=${3:-$REPO/benchmarks/startup_20261030/evidence/${MODE}_${SCENARIO}_$(date +%H%M%S)}
if [[ -e "$EVIDENCE" ]]; then
  printf 'Refusing to overwrite evidence: %s\n' "$EVIDENCE" >&2
  exit 2
fi
# Check ports without probing either the physical robot or any production web.
/usr/bin/python3 - <<'PY'
import socket
for port in (8796, 11586):
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', port))
PY
mkdir -p -- "$EVIDENCE"
EVIDENCE=$(cd -- "$EVIDENCE" && pwd)
WORK=$(mktemp -d /tmp/smart_vision_startup186.XXXXXX)
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11586
export TURTLEBOT3_MODEL=waffle_pi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH \
  FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
VENDOR=${SMART_VISION_ROBOT_LOCALIZATION_EXTRACTED:-$HOME/.local/opt/robot_localization/extracted}
if [[ "$MODE" == ekf_independent ]] && ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  PREFIX=$VENDOR/opt/ros/humble
  if [[ ! -f "$PREFIX/share/ament_index/resource_index/packages/robot_localization" ]]; then
    printf 'Independent EKF requires robot_localization under /opt/ros/humble, or an extracted package at %s. Set SMART_VISION_ROBOT_LOCALIZATION_EXTRACTED to its extraction root.\n' "$VENDOR" >&2
    exit 2
  fi
  export AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH"
  export CMAKE_PREFIX_PATH="$PREFIX:$CMAKE_PREFIX_PATH"
  export LD_LIBRARY_PATH="$PREFIX/lib:$VENDOR/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$REPO/gazebo_models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export ROS_LOG_DIR="$WORK/ros_logs"
mkdir -p -- "$WORK/src"
cp -- "$REPO/scripts/verify_startup_gazebo.py" "$WORK/verify_startup_gazebo.py"
cp -a -- "$REPO/ros2_ws/src/smart_vision_amr" "$WORK/src/"
TEAM_SRC=$(cd -- "$REPO/.." && pwd)
for dependency in turtlebot3/turtlebot3_description turtlebot3/turtlebot3_navigation2 turtlebot3_simulations/turtlebot3_gazebo; do
  if [[ ! -f "$TEAM_SRC/$dependency/package.xml" ]]; then
    printf 'Pinned simulation source is missing: %s. Prepare the workspace with scripts/setup_sim.sh --no-build before running this disposable validation.\n' "$TEAM_SRC/$dependency" >&2
    exit 2
  fi
done
cp -a -- "$TEAM_SRC/turtlebot3/turtlebot3_description" "$WORK/src/"
cp -a -- "$TEAM_SRC/turtlebot3/turtlebot3_navigation2" "$WORK/src/"
cp -a -- "$TEAM_SRC/turtlebot3_simulations/turtlebot3_gazebo" "$WORK/src/"
/usr/bin/python3 - "$WORK" "$REPO" "$EVIDENCE" "$MODE" "$SCENARIO" <<'PY'
from pathlib import Path
import hashlib, json, os, subprocess, sys, xml.etree.ElementTree as ET
work, repo, evidence = map(Path, sys.argv[1:4])
model = work/'src/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
text = model.read_text()
assert text.count('~/out:=scan') == 1
model.write_text(text.replace('~/out:=scan', '~/out:=/startup_validation/raw_scan'))
world = work/'src/smart_vision_amr/worlds/simulation_room.world'
tree = ET.parse(world)
root = tree.getroot().find('world')
state_plugin = ET.SubElement(root, 'plugin', name='startup_validation_ground_truth', filename='libgazebo_ros_state.so')
ET.SubElement(ET.SubElement(state_plugin, 'ros'), 'namespace').text = '/gazebo'
ET.SubElement(state_plugin, 'update_rate').text = '20.0'
if sys.argv[5] == 'precision':
    model_tree = ET.parse(model)
    camera = model_tree.getroot().find('.//sensor[@type="camera"]')
    ET.SubElement(camera, 'pose').text = '-0.25 0 0.157 0 0 3.141592653589793'
    model_tree.write(model, encoding='unicode')
    board = ET.SubElement(root, 'include')
    ET.SubElement(board, 'uri').text = 'model://smart_vision_aruco_4x4_50_id_1'
    ET.SubElement(board, 'name').text = 'startup_validation_rear_wall_id1'
    ET.SubElement(board, 'pose').text = '-4.992 -2.0 0.25 0 0 3.141592653589793'
if sys.argv[5] == 'barrier':
    # Both the copied occupancy map and actual Gazebo geometry gain these
    # known walls. The whole blocking object's extent is then within LiDAR
    # range, so an unseen far end cannot be mistaken for a global escape.
    from PIL import Image
    map_path = work/'src/smart_vision_amr/maps/simulation_room.pgm'
    image = Image.open(map_path).convert('L')
    pixels = image.load()
    for index, center_y in enumerate((-3.1, -.9)):
        wall = ET.SubElement(root, 'model', name=f'startup_validation_corridor_wall_{index}')
        ET.SubElement(wall, 'static').text = 'true'
        ET.SubElement(wall, 'pose').text = f'0 {center_y} .5 0 0 0'
        link = ET.SubElement(wall, 'link', name='body')
        for kind in ('collision', 'visual'):
            body = ET.SubElement(link, kind, name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(body, 'geometry'), 'box'), 'size').text = '10 .1 1'
        for row in range(image.height):
            y = -4.1+(image.height-row-.5)*.05
            if abs(y-center_y) <= .05+1e-9:
                for col in range(image.width):
                    x = -5.1+(col+.5)*.05
                    if abs(x) <= 5.:
                        pixels[col, row] = 0
    image.save(map_path)
tree.write(world, encoding='unicode')
copied_launch = work/'src/smart_vision_amr/launch/simulation.launch.py'
qos_probe = os.environ.get('SMART_VISION_AMCL_TF_BEST_EFFORT_PROBE') == '1'
if qos_probe:
    # Only this AMCL process receives the wildcard file. Its hidden TF-listener
    # node inherits process-global arguments; /amcl-scoped overrides miss it.
    qos_file = work/'amcl_tf_best_effort_probe.yaml'
    qos_file.write_text('/**:\n  ros__parameters:\n    qos_overrides./tf.subscription.reliability: best_effort\n    qos_overrides./tf.subscription.depth: 20\n')
    launch_text = copied_launch.read_text()
    before = "parameters=[config, sim, {'set_initial_pose':"
    assert launch_text.count(before) == 1
    copied_launch.write_text(launch_text.replace(before,
        f"parameters=[config, sim, {str(qos_file)!r}, {{'set_initial_pose':"))
files = sorted(path for path in (repo/'ros2_ws/src/smart_vision_amr').rglob('*')
               if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc')
manifest = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files}
payload = {'source_commit': subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
           'source_sha256': manifest, 'snapshot_workspace': str(work), 'mode': sys.argv[4],
           'verifier_sha256': hashlib.sha256((work/'verify_startup_gazebo.py').read_bytes()).hexdigest(),
           'domain': 186, 'localhost_only': 1, 'gazebo_master_uri': 'http://127.0.0.1:11586',
           'web_url': 'http://127.0.0.1:8796',
           'simulation_set_initial_pose': sys.argv[5] != 'home_initialization',
           'copied_fixture_sha256': {str(path.relative_to(work/'src')): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in (world, model, copied_launch, work/'src/smart_vision_amr/maps/simulation_room.pgm')},
           'test_fixture_changes': ['Copied TB3 laser output remapped to /startup_validation/raw_scan; untouched messages transparently relayed to /scan',
                                   'Gazebo state plugin added to copied world to observe actual model ground truth',
                                   'Test-only reachable room marker poses; no marker detection or navigation result fabricated']}
if sys.argv[5] == 'precision':
    payload['test_fixture_changes'] += [
        'Copied camera sensor faces rear from known base pose (-0.174,0,0.25,yaw=pi), upright image',
        'Actual textured ID1 board mounted on west wall facing the rear camera; initial body yaw deliberately +0.20 rad']
if sys.argv[5] == 'barrier':
    payload['test_fixture_changes'] += [
        'Only copied world and copied occupancy PGM gain matching known full-length corridor walls at y=-3.1 and y=-0.9, size 10x0.1x1 m; original source PGM is unchanged',
        'Actual dynamic 0.25x2.2x1 m barrier at (-1.5,-2) joins the two known corridor walls; its full span is within real LiDAR range']
if sys.argv[5] == 'home_initialization':
    payload['test_fixture_changes'] += [
        'Production set_initial_pose launch argument is false: AMCL boots without a prior/map-to-odom transform; only the confirmed Home HTTP actions supply legitimate registered-Home priors, with ESTOP preserved until explicit reset']
if qos_probe:
    payload['test_fixture_changes'] += ['A/B diagnostic ONLY: copied AMCL Node alone receives a process-specific /** QoS file selecting dynamic /tf BEST_EFFORT depth20; static TF and all other nodes unchanged; actual AMCL-participant hidden reader is checked before motion']
    payload['amcl_tf_probe_parameters'] = qos_file.read_text()
    payload['tf_observer_sha256'] = hashlib.sha256((repo/'scripts/gazebo_tf_ingress_probe.cpp').read_bytes()).hexdigest()
elif os.environ.get('SMART_VISION_TF_DIAGNOSTICS') == '1':
    payload['tf_observer_sha256'] = hashlib.sha256((repo/'scripts/gazebo_tf_ingress_probe.cpp').read_bytes()).hexdigest()
(evidence/'source_manifest.json').write_text(json.dumps(payload, indent=2)+'\n')
markers = 'markers:\n  - {id: 0, name: Home, x: -3.0, y: -2.0, yaw: 0.0, camera: rear}\n'
markers += ('  - {id: 1, name: Dynamic obstacle goal, x: 1.0, y: -2.0, yaw: 0.0}\n'
            if sys.argv[5] in ('local_detour', 'barrier') else
            '  - {id: 1, name: Rear wall goal, x: -3.0, y: -2.0, yaw: 0.20, camera: rear}\n'
            if sys.argv[5] == 'precision' else
            '  - {id: 1, name: Trial goal, x: -1.0, y: -2.0, yaw: 0.0}\n')
(evidence/'markers_fixture.yaml').write_text(markers)
PY
cd -- "$WORK"
colcon build --symlink-install --base-paths "$WORK/src" \
  --event-handlers console_direct+ >"$EVIDENCE/build.log" 2>&1
source "$WORK/install/local_setup.bash"
if [[ "$TF_DIAGNOSTICS" == 1 ]]; then
  cp -- "$REPO/scripts/gazebo_tf_ingress_probe.cpp" "$WORK/gazebo_tf_ingress_probe.cpp"
  mkdir -p "$WORK/tf_ingress_probe"
  cat > "$WORK/tf_ingress_probe/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(smart_vision_tf_probe)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
add_executable(gazebo_tf_ingress_probe ../gazebo_tf_ingress_probe.cpp)
ament_target_dependencies(gazebo_tf_ingress_probe rclcpp sensor_msgs tf2_msgs)
CMAKE
  cmake -S "$WORK/tf_ingress_probe" -B "$WORK/tf_ingress_probe/build" >"$WORK/tf_ingress_build.log" 2>&1
  cmake --build "$WORK/tf_ingress_probe/build" -j 2 >>"$WORK/tf_ingress_build.log" 2>&1
fi
LAUNCH_PID=
WEB_PID=
TF_OBSERVER_PID=
cleanup() {
  trap - EXIT INT TERM
  for owned in "$WEB_PID" "$LAUNCH_PID" "$TF_OBSERVER_PID"; do
    if [[ -n "$owned" ]] && kill -0 "$owned" 2>/dev/null; then
      kill -INT -- "-$owned" 2>/dev/null || true
    fi
  done
  deadline=$((SECONDS+15))
  while ((SECONDS < deadline)); do
    alive=false
    for owned in "$WEB_PID" "$LAUNCH_PID" "$TF_OBSERVER_PID"; do
      if [[ -n "$owned" ]] && kill -0 "$owned" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep .2
  done
  for owned in "$WEB_PID" "$LAUNCH_PID" "$TF_OBSERVER_PID"; do
    if [[ -n "$owned" ]]; then
      kill -TERM -- "-$owned" 2>/dev/null || true
    fi
  done
  deadline=$((SECONDS+5))
  while ((SECONDS < deadline)); do
    alive=false
    for owned in "$WEB_PID" "$LAUNCH_PID" "$TF_OBSERVER_PID"; do
      if [[ -n "$owned" ]] && kill -0 "$owned" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep .2
  done
  for owned in "$WEB_PID" "$LAUNCH_PID" "$TF_OBSERVER_PID"; do
    if [[ -n "$owned" ]]; then
      kill -KILL -- "-$owned" 2>/dev/null || true
      wait "$owned" 2>/dev/null || true
    fi
  done
  # Full ROS logs stay in the disposable workspace; repository evidence keeps
  # bounded process logs plus compact measurement/assertion summaries.
  for log in build stack web; do
    if [[ -f "$EVIDENCE/$log.log" ]]; then
      cp -- "$EVIDENCE/$log.log" "$WORK/$log.full.log"
      tail -n 180 -- "$WORK/$log.full.log" >"$EVIDENCE/$log.log"
    fi
  done
}
trap cleanup EXIT INT TERM
VERIFIER_EXTRA=(--require-nav2-tf-best-effort)
if [[ "$AMCL_TF_PROBE" == 1 ]]; then
  VERIFIER_EXTRA=(--require-amcl-tf-best-effort)
fi
if [[ "$TF_DIAGNOSTICS" == 1 ]]; then
  setsid "$WORK/tf_ingress_probe/build/gazebo_tf_ingress_probe" 150 >"$WORK/tf_ingress.jsonl" &
  TF_OBSERVER_PID=$!
fi
SPAWN_YAW=0.0
SET_INITIAL_POSE=true
if [[ "$SCENARIO" == home_initialization ]]; then SET_INITIAL_POSE=false; fi
WEB_EXTRA=()
if [[ "$SCENARIO" == precision ]]; then
  SPAWN_YAW=0.20
  WEB_EXTRA=(--require-marker-wall-alignment --marker-center-tolerance 0.05
             --marker-rear-camera-pose -0.174 0 3.141592653589793 --camera-rotation 0)
fi
setsid ros2 launch smart_vision_amr simulation.launch.py \
  gui:=false use_rviz:=false record_incidents:=false use_motion_mux:=false \
  localization_mode:="$MODE" camera_update_rate:=4.0 \
  set_initial_pose:="$SET_INITIAL_POSE" \
  memory_path:="$WORK/simulation_memory.json" \
  x_pose:=-3.0 y_pose:=-2.0 initial_x:=-3.0 initial_y:=-2.0 yaw:="$SPAWN_YAW" initial_yaw:="$SPAWN_YAW" \
  >"$EVIDENCE/stack.log" 2>&1 &
LAUNCH_PID=$!
setsid /usr/bin/python3 -m smart_vision_amr.web_mission_server \
  --host 127.0.0.1 --port 8796 --domain-id 186 --access-mode domain \
  --use-sim-time --require-localization-health --with-motion-mux \
  --config-file "$WORK/web_mission.json" \
  --markers-file "$EVIDENCE/markers_fixture.yaml" \
  --map-file "$WORK/src/smart_vision_amr/maps/simulation_room.yaml" \
  "${WEB_EXTRA[@]}" \
  >"$EVIDENCE/web.log" 2>&1 &
WEB_PID=$!
printf 'Disposable workspace %s; own launch=%s web=%s\n' "$WORK" "$LAUNCH_PID" "$WEB_PID"
/usr/bin/python3 - <<'PY'
import time, urllib.request
until = time.monotonic()+30
while time.monotonic() < until:
    try:
        urllib.request.urlopen('http://127.0.0.1:8796/api/state', timeout=1).read()
        break
    except Exception:
        time.sleep(.2)
else:
    raise SystemExit('Test-only web did not start')
PY
VERIFIER_STATUS=0
PYTHONUNBUFFERED=1 /usr/bin/python3 "$WORK/verify_startup_gazebo.py" \
  --scenario "$SCENARIO" --simulation-launch-pid "$LAUNCH_PID" \
  "${VERIFIER_EXTRA[@]}" \
  --raw-output "$WORK/full_result.json" --output "$EVIDENCE/result.json" \
  >"$EVIDENCE/verifier.log" 2>&1 || VERIFIER_STATUS=$?
cat -- "$EVIDENCE/verifier.log"
exit "$VERIFIER_STATUS"
