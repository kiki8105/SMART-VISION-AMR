#!/usr/bin/env bash
# Provision the Team2 simulation workspace without sourcing another workspace.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
BUILD=true
ROOT_GIVEN=false

usage() {
  cat <<'EOF'
Usage: setup_sim.sh [--no-build] [WORKSPACE_ROOT]

Fetch pinned official TurtleBot3 Humble simulation assets and build Team2/src.
The default workspace is the directory containing this repository's src folder.
--no-build only prepares and checks dependencies.
EOF
}

for arg in "$@"; do
  case "$arg" in
    -h|--help) usage; exit 0 ;;
    --no-build) BUILD=false ;;
    -*) printf 'Unknown option: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
    *)
      if "$ROOT_GIVEN"; then usage >&2; exit 2; fi
      WORKSPACE_ROOT=$arg
      ROOT_GIVEN=true
      ;;
  esac
done

run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

APT_PACKAGES=(git build-essential cmake python3-colcon-common-extensions gazebo
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup
  ros-humble-xacro ros-humble-slam-toolbox ros-humble-robot-localization
  ros-humble-compressed-image-transport
  ros-humble-teleop-twist-keyboard python3-opencv python3-pyqt5 python3-numpy
  python3-yaml python3-pil)

missing_dependencies() {
  printf '\nRequired dependencies are missing. On Ubuntu 22.04 with ROS Humble apt sources, run:\n' >&2
  printf '  sudo apt update\n  sudo apt install' >&2
  printf ' %s' "${APT_PACKAGES[@]}" >&2
  printf '\nThen rerun this script. No packages were installed automatically.\n' >&2
  exit 1
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS Humble was not found at /opt/ros/humble.\n' >&2
  missing_dependencies
fi
# Discard paths inherited from hardware, UAV, or other ROS workspaces.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH
run source /opt/ros/humble/setup.bash

MISSING=false
for tool in git cmake colcon gazebo ros2; do
  if ! command -v "$tool" >/dev/null; then
    printf 'Missing executable: %s\n' "$tool" >&2
    MISSING=true
  fi
done
for package in ament_cmake ament_package gazebo_ros gazebo_plugins nav2_bringup \
  xacro robot_state_publisher rviz2 slam_toolbox robot_localization compressed_image_transport; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    printf 'Missing ROS package: %s\n' "$package" >&2
    MISSING=true
  fi
done
if ! /usr/bin/python3 -c 'from PyQt5 import QtWidgets; import cv2, numpy, yaml; from PIL import Image; assert hasattr(cv2, "aruco"), "OpenCV ArUco is required"'; then
  printf 'Missing Python dependencies for the bundled controller, ArUco, or SLAM verification.\n' >&2
  MISSING=true
fi
if "$MISSING"; then missing_dependencies; fi

run mkdir -p -- "$WORKSPACE_ROOT/src"
WORKSPACE_ROOT=$(cd -- "$WORKSPACE_ROOT" && pwd)

fetch_dependency() {
  local name=$1 url=$2 revision=$3
  shift 3
  local destination="$WORKSPACE_ROOT/src/$name"
  local origin top
  if [[ ! -e "$destination" ]]; then
    run git clone --filter=blob:none --sparse --branch humble "$url" "$destination"
  fi
  if [[ ! -d "$destination/.git" ]]; then
    printf 'Refusing to replace non-repository path: %s\n' "$destination" >&2
    exit 1
  fi
  top=$(git -C "$destination" rev-parse --show-toplevel)
  if [[ "$top" != "$destination" ]]; then
    printf 'Unexpected repository root at %s\n' "$destination" >&2
    exit 1
  fi
  origin=$(git -C "$destination" remote get-url origin)
  if [[ "${origin%.git}" != "${url%.git}" ]]; then
    printf 'Unexpected origin at %s: %s\n' "$destination" "$origin" >&2
    exit 1
  fi
  if [[ -n $(git -C "$destination" status --porcelain) ]]; then
    printf 'Dependency has local changes; left untouched: %s\n' "$destination" >&2
    printf 'Inspect and commit or stash those changes before retrying.\n' >&2
    exit 1
  fi
  if ! git -C "$destination" cat-file -e "$revision^{commit}" 2>/dev/null; then
    run git -C "$destination" fetch origin "$revision"
  fi
  # Git 2.34 (Ubuntu 22.04) sets cone mode through init, not through set.
  run git -C "$destination" sparse-checkout init --cone
  run git -C "$destination" sparse-checkout set -- "$@"
  run git -C "$destination" checkout --detach "$revision"
  printf '%s pinned at %s\n' "$name" "$(git -C "$destination" rev-parse HEAD)"
}

# Humble branch commits fetched from the official ROBOTIS-GIT repositories.
fetch_dependency turtlebot3 https://github.com/ROBOTIS-GIT/turtlebot3.git \
  90a68bd2e3c61c12966779da89d8eeaec82730e9 \
  turtlebot3_description turtlebot3_navigation2
fetch_dependency turtlebot3_simulations https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
  a35a56c8b04877dc89772b598084d8ce648a9023 turtlebot3_gazebo

run cd -- "$WORKSPACE_ROOT"
run colcon list --base-paths "$WORKSPACE_ROOT/src"
if "$BUILD"; then
  run colcon build --symlink-install --base-paths "$WORKSPACE_ROOT/src" \
    --event-handlers console_direct+
  printf '\nBuilt simulation workspace: %s\n' "$WORKSPACE_ROOT"
  printf 'Start it with: %q --workspace %q\n' "$SCRIPT_DIR/run_sim.sh" "$WORKSPACE_ROOT"
else
  printf '\nDependencies ready. Build skipped (--no-build).\n'
fi
