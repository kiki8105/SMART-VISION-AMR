#!/usr/bin/env bash
# Build the physical-robot PC package; never starts ROS nodes or fetches Gazebo assets.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
INSTALL_DEPS=true
BUILD=true
ROOT_GIVEN=false
usage() {
  cat <<'EOF'
Usage: setup_pc_runtime.sh [--no-install] [--no-build] [WORKSPACE_ROOT]
Ubuntu 22.04 / ROS Humble must already be installed. Installs only the physical
PC navigation/web dependencies and builds this clone's smart_vision_amr package.
No Gazebo, simulator downloads, robot connection, or .bashrc changes.
--no-install  Dependencies are already installed; skip apt.
--no-build    Install/check dependencies only.
EOF
}
for argument in "$@"; do
  case "$argument" in
    -h|--help) usage; exit 0 ;;
    --no-install) INSTALL_DEPS=false ;;
    --no-build) BUILD=false ;;
    -*) printf 'Unknown option: %s\n' "$argument" >&2; exit 2 ;;
    *) if "$ROOT_GIVEN"; then usage >&2; exit 2; fi
       WORKSPACE_ROOT=$argument; ROOT_GIVEN=true ;;
  esac
done
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'Install ROS Humble first: /opt/ros/humble/setup.bash is missing.\n' >&2
  exit 2
fi
if "$INSTALL_DEPS"; then
  sudo apt update
  sudo apt install -y build-essential python3-colcon-common-extensions python3-setuptools \
    ros-humble-nav2-bringup ros-humble-turtlebot3-navigation2 ros-humble-turtlebot3-description \
    ros-humble-robot-localization ros-humble-compressed-image-transport \
    ros-humble-diagnostic-msgs python3-opencv python3-numpy python3-yaml python3-pil
fi
WORKSPACE_ROOT=$(realpath -m -- "$WORKSPACE_ROOT")
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
# Reuse dependencies built in this selected workspace, such as the official
# TurtleBot3 packages. local_setup avoids importing unrelated underlay chains;
# a new clone without an install directory still uses the apt packages above.
if [[ -f "$WORKSPACE_ROOT/install/local_setup.bash" ]]; then
  source "$WORKSPACE_ROOT/install/local_setup.bash"
fi
for package in nav2_bringup turtlebot3_navigation2 robot_localization compressed_image_transport; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    printf 'Missing ROS package %s in Humble and selected workspace %s; rerun without --no-install.\n' \
      "$package" "$WORKSPACE_ROOT" >&2
    exit 2
  fi
done
/usr/bin/python3 -c 'import cv2, numpy, yaml; from PIL import Image; assert hasattr(cv2, "aruco"), "OpenCV ArUco is required"'
mkdir -p -- "$WORKSPACE_ROOT"
WORKSPACE_ROOT=$(cd -- "$WORKSPACE_ROOT" && pwd)
if "$BUILD"; then
  cd -- "$WORKSPACE_ROOT"
  colcon build --packages-select smart_vision_amr --symlink-install \
    --base-paths "$REPO_DIR/ros2_ws/src/smart_vision_amr" --event-handlers console_direct+
fi
printf '\nPhysical PC workspace prepared: %s; no ROS node started.\n' "$WORKSPACE_ROOT"
printf 'Read-only check: %q --workspace %q\n' "$SCRIPT_DIR/check_classroom_deployment.sh" "$WORKSPACE_ROOT"
