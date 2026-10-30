#!/usr/bin/env bash
# PC-side real-robot navigation; run TurtleBot3 bringup on the robot separately.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
PARAMS_FILE=${TURTLEBOT3_TUNING_PARAMS_FILE:-$REPO_DIR/ros2_ws/src/smart_vision_amr/config/curve_tuning_waffle_pi.yaml}
DOMAIN_ID=${TURTLEBOT3_TUNING_DOMAIN_ID:-16}
if [[ $# == 0 || "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: run_hardware_tuning.sh /absolute/path/real_robot_map.yaml [LAUNCH_ARGUMENT:=VALUE ...]
PC-side AMCL/Nav2/team A* with the editable curve_tuning_waffle_pi.yaml preset.
Default domain: 16; real time; RViz disabled. Robot bringup runs separately.
TURTLEBOT3_TUNING_DOMAIN_ID and TURTLEBOT3_TUNING_PARAMS_FILE override defaults.
Use the same overrides with run_hardware_tuning_controller.sh.
EOF
  if [[ $# == 0 ]]; then exit 2; else exit 0; fi
fi
MAP_FILE=$(realpath -- "$1")
shift
if [[ ! -f "$MAP_FILE" || ! -f "$PARAMS_FILE" || ! -f "$WORKSPACE_ROOT/install/local_setup.bash" ]]; then
  printf 'Map, tuning YAML, or built Team2 workspace is missing.\n' >&2
  exit 2
fi
if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ || "$DOMAIN_ID" -gt 232 ]]; then
  printf 'TURTLEBOT3_TUNING_DOMAIN_ID must be 0..232.\n' >&2
  exit 2
fi
# One physical navigation stack per ROS domain: a second launch would create
# duplicate map_server/AMCL/lifecycle-manager names and break their bonds.
LOCK_DIR=${XDG_RUNTIME_DIR:-$HOME/.cache}/smart-vision-amr
mkdir -p -- "$LOCK_DIR"
chmod 700 -- "$LOCK_DIR"
exec 9>"$LOCK_DIR/hardware-navigation-${DOMAIN_ID}.lock"
if ! flock -n 9; then
  printf 'Navigation is already running in ROS domain %s. Stop that launch before starting another.\n' "$DOMAIN_ID" >&2
  exit 1
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="$DOMAIN_ID" ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
source "$SCRIPT_DIR/dds_env.bash" ""
# An EKF run needs robot_localization on the PC. Reuse the non-root Humble
# package used by our benchmark when the host apt installation is absent.
needs_ekf=false
estimation_location=pc
localization_mode=baseline
for arg in "$@"; do
  case "$arg" in
    localization_mode:=*) localization_mode=${arg#localization_mode:=} ;;
    estimation_location:=*) estimation_location=${arg#estimation_location:=} ;;
  esac
done
if [[ "$localization_mode" == ekf_independent && "$estimation_location" == pc ]]; then needs_ekf=true; fi
if [[ "$needs_ekf" == true ]] && ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  local_ekf_root=$HOME/.local/opt/robot_localization/extracted
  local_ekf_prefix=$local_ekf_root/opt/ros/humble
  local_ekf_lib=$local_ekf_root/usr/lib/x86_64-linux-gnu
  if [[ -d "$local_ekf_prefix/share/ament_index/resource_index/packages"
        && -d "$local_ekf_lib" ]]; then
    export AMENT_PREFIX_PATH="$local_ekf_prefix:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$local_ekf_prefix:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$local_ekf_lib:$local_ekf_prefix/lib:$LD_LIBRARY_PATH"
  fi
fi
if [[ "$needs_ekf" == true ]] && ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  printf 'Independent EKF requires ros-humble-robot-localization on the PC.\n' >&2
  exit 2
fi
if [[ "$needs_ekf" == true || "$estimation_location" == robot ]] && ! timeout 12s /usr/bin/python3 \
    "$SCRIPT_DIR/check_ekf_robot_preflight.py" --timeout 8 \
    --estimation-location "$estimation_location" --localization-mode "$localization_mode"; then
  if [[ "$estimation_location" == robot ]]; then
    printf 'Start the robot-local runtime first (scripts/setup_robot_runtime.sh builds it).\n' >&2
    printf 'For previous PC deployment, set SMART_VISION_ESTIMATION_LOCATION=pc on robot, navigation AND web.\n' >&2
  else
    printf 'Robot must run wheel-only EKF bringup before PC estimation starts.\n' >&2
  fi
  exit 2
fi
# Also catch a Nav2 stack started directly, outside this script/lock. Query the
# live graph rather than the ROS daemon cache, which can retain old node names.
if ! EXISTING_NODES=$(timeout 10s ros2 node list --no-daemon --spin-time 2); then
  printf 'Could not inspect ROS domain %s; refusing to risk a duplicate navigation launch.\n' "$DOMAIN_ID" >&2
  exit 1
fi
if printf '%s\n' "$EXISTING_NODES" | grep -Eq '^/(map_server|amcl|lifecycle_manager_localization)$'; then
  printf 'A map server, AMCL, or localization lifecycle manager already exists in ROS domain %s. Stop the old navigation launch first.\n' "$DOMAIN_ID" >&2
  exit 1
fi
# A lingering Cartographer or other /map publisher can overwrite Nav2's saved
# map for AMCL, the A* planner, and the web UI. Check the live DDS graph rather
# than the ros2 daemon's cache, before starting another map_server.
if ! timeout 10s /usr/bin/python3 "$SCRIPT_DIR/check_map_publishers.py" --domain-id "$DOMAIN_ID"; then
  printf 'Refusing to start hardware navigation until ROS domain %s has one planned map source.\n' "$DOMAIN_ID" >&2
  exit 1
fi
exec ros2 launch smart_vision_amr hardware.launch.py \
  map:="$MAP_FILE" params_file:="$(realpath -- "$PARAMS_FILE")" use_rviz:=false "$@"
