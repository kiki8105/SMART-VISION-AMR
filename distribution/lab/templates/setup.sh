#!/usr/bin/env bash
# Build only the source packages shipped inside this extracted lab.
set -eo pipefail
LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
INSTALL_DEPS=false
CHECK_ONLY=false
for arg in "$@"; do
  case "$arg" in
    --install-deps) INSTALL_DEPS=true ;;
    --check-only|--no-build) CHECK_ONLY=true ;;
    -h|--help)
      printf 'Usage: ./setup.sh [--install-deps] [--check-only]\nUbuntu 22.04 and ROS Humble are required. Builds ./ros2_ws; no git clone.\n'
      exit 0 ;;
    *) printf 'Unknown option: %s\n' "$arg" >&2; exit 2 ;;
  esac
done
APT_PACKAGES=(build-essential cmake python3-colcon-common-extensions gazebo
  ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup ros-humble-xacro
  ros-humble-slam-toolbox ros-humble-robot-localization
  ros-humble-compressed-image-transport
  ros-humble-robot-state-publisher ros-humble-rviz2
  python3-pyqt5 python3-numpy python3-yaml python3-pil python3-opencv)
if "$INSTALL_DEPS"; then
  sudo apt-get update
  sudo apt-get install "${APT_PACKAGES[@]}"
fi
missing=false
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS Humble is missing: /opt/ros/humble/setup.bash\n' >&2
  missing=true
else
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
    PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH
  source /opt/ros/humble/setup.bash
  for tool in c++ cmake colcon gazebo ros2 setsid timeout; do
    if ! command -v "$tool" >/dev/null; then printf 'Missing executable: %s\n' "$tool" >&2; missing=true; fi
  done
  for package in ament_cmake ament_package gazebo_ros gazebo_plugins nav2_bringup \
    xacro robot_state_publisher rviz2 slam_toolbox robot_localization compressed_image_transport; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
      printf 'Missing ROS package: %s\n' "$package" >&2; missing=true
    fi
  done
  if ! /usr/bin/python3 -c 'from PyQt5 import QtWidgets; import numpy, yaml, cv2; from PIL import Image; assert hasattr(cv2, "aruco"), "OpenCV ArUco is missing"'; then
    missing=true
  fi
fi
if "$missing"; then
  printf '\nUbuntu 22.04 + configured ROS Humble apt sources are required.\nInstall missing dependencies explicitly with:\n  ./setup.sh --install-deps\nOr run:\n  sudo apt-get update\n  sudo apt-get install' >&2
  printf ' %s' "${APT_PACKAGES[@]}" >&2
  printf '\nNo packages are installed without --install-deps.\n' >&2
  exit 1
fi
if "$CHECK_ONLY"; then printf 'Dependency checks passed; build skipped.\n'; exit 0; fi
BUILD_ROOT_FILE="$LAB_DIR/ros2_ws/.lab_build_root"
if [[ -f "$BUILD_ROOT_FILE" && "$(cat -- "$BUILD_ROOT_FILE")" != "$LAB_DIR/ros2_ws" ]]; then
  printf 'Bundle moved: clearing generated build/install/log paths before rebuilding.\n'
  rm -rf -- "$LAB_DIR/ros2_ws/build" "$LAB_DIR/ros2_ws/install" "$LAB_DIR/ros2_ws/log"
fi
printf '%s\n' "$LAB_DIR/ros2_ws" > "$BUILD_ROOT_FILE"
cd -- "$LAB_DIR/ros2_ws"
colcon build --symlink-install --base-paths "$LAB_DIR/ros2_ws/src" \
  --packages-select turtlebot3_description turtlebot3_navigation2 turtlebot3_gazebo smart_vision_amr \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
printf '\nLab built: %s\nRun ./run_slam.sh or ./run_navigation.sh, then ./run_controller.sh in another terminal.\n' "$LAB_DIR"
