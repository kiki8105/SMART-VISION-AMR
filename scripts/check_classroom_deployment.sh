#!/usr/bin/env bash
# Read-only checks. Sourcing happens only in this child shell; no clocks/settings change.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
ROLE=pc
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
ROBOT_WS=${TURTLEBOT3_WORKSPACE:-$HOME/turtlebot3_ws}
LIVE=false
usage() {
  cat <<'EOF'
Usage: check_classroom_deployment.sh [--pc|--robot] [--workspace PATH] [--live]
Check the selected Humble overlay and physical ROS domain 16 without launching
nodes, publishing, changing system clocks, or connecting by SSH. --robot defaults
to ~/smart_vision_runtime_ws. --live (PC only) also observes robot publishers and
sensor timestamps for 8s after terminal 1 is running. It sends no commands.
EOF
}
WORKSPACE_GIVEN=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --pc) ROLE=pc; shift ;;
    --robot) ROLE=robot; shift ;;
    --live) LIVE=true; shift ;;
    --workspace)
      if [[ $# -lt 2 || -z "$2" ]]; then usage >&2; exit 2; fi
      WORKSPACE_ROOT=$2; WORKSPACE_GIVEN=true; shift 2 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done
if [[ "$ROLE" == robot ]] && ! "$WORKSPACE_GIVEN"; then
  WORKSPACE_ROOT=${SMART_VISION_RUNTIME_WORKSPACE:-$HOME/smart_vision_runtime_ws}
fi
if [[ "$ROLE" == robot ]] && "$LIVE"; then
  printf -- '--live is a PC observation of the already-running robot; use --pc.\n' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$WORKSPACE_ROOT/install/local_setup.bash" ]]; then
  printf 'ROS Humble or built overlay is missing: %s/install/local_setup.bash\n' "$WORKSPACE_ROOT" >&2
  exit 2
fi
WORKSPACE_ROOT=$(cd -- "$WORKSPACE_ROOT" && pwd)
printf 'Role: %s\nSelected overlay: %s/install/local_setup.bash\n' "$ROLE" "$WORKSPACE_ROOT"
printf 'Inherited ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s (check uses 16 / 0 in this shell)\n' \
  "${ROS_DOMAIN_ID:-unset}" "${ROS_LOCALHOST_ONLY:-unset}"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [[ "$ROLE" == robot ]]; then
  if [[ ! -f "$ROBOT_WS/install/local_setup.bash" ]]; then
    printf 'Missing stock TurtleBot3 overlay: %s/install/local_setup.bash\n' "$ROBOT_WS" >&2; exit 2
  fi
  source "$ROBOT_WS/install/local_setup.bash"
fi
source "$WORKSPACE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=waffle_pi
source "$SCRIPT_DIR/dds_env.bash" ""
PREFIX=$(ros2 pkg prefix smart_vision_amr)
case "$PREFIX" in
  "$WORKSPACE_ROOT/install"|"$WORKSPACE_ROOT/install/"*) ;;
  *) printf 'Wrong smart_vision_amr overlay: %s\n' "$PREFIX" >&2; exit 2 ;;
esac
if [[ "$ROLE" == pc ]]; then
  PACKAGES=(nav2_bringup turtlebot3_navigation2 robot_localization compressed_image_transport)
  EXECUTABLES=(web_mission_server path_follower astar_planner command_stamper)
else
  PACKAGES=(turtlebot3_bringup robot_localization)
  EXECUTABLES=(motion_mux localization_health wheel_joint_odometry imu_ekf_conditioner ekf_sensor_guard)
fi
for package in "${PACKAGES[@]}"; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    printf 'Missing ROS dependency: %s\n' "$package" >&2; exit 2
  fi
done
for executable in "${EXECUTABLES[@]}"; do
  if [[ ! -x "$PREFIX/lib/smart_vision_amr/$executable" ]]; then
    printf 'Missing built executable: %s\n' "$executable" >&2; exit 2
  fi
done
printf 'Package: %s\nROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s\nDDS profile: %s\n' \
  "$PREFIX" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "${FASTDDS_DEFAULT_PROFILES_FILE:-middleware default}"
printf 'Local UTC clock: '; date -u '+%Y-%m-%d %H:%M:%S UTC'
if command -v timedatectl >/dev/null; then
  timedatectl show -p NTPSynchronized -p TimeUSec 2>/dev/null || true
fi
printf 'Compare PC/robot UTC clocks. Unsynchronized clocks can reject command/sensor timestamps.\n'
if "$LIVE"; then
  timeout 12s /usr/bin/python3 "$SCRIPT_DIR/check_ekf_robot_preflight.py" \
    --timeout 8 --estimation-location robot --localization-mode ekf_independent
else
  printf 'Static checks passed. After terminal 1, PC --live checks discovery and clock-stamped sensors.\n'
fi
