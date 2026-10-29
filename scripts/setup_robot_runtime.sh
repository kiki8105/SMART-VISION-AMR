#!/usr/bin/env bash
# Run on the Pi. Installs/builds only the robot-side runtime; starts no processes.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
RUNTIME_WS=${SMART_VISION_RUNTIME_WORKSPACE:-$HOME/smart_vision_runtime_ws}
ROBOT_WS=${TURTLEBOT3_WORKSPACE:-$HOME/turtlebot3_ws}
INSTALL_DEPS=true
BUILD=true

usage() {
  cat <<'EOF'
Usage: setup_robot_runtime.sh [--no-install] [--no-build] [RUNTIME_WORKSPACE]

Run on an Ubuntu 22.04 / ROS Humble TurtleBot3. Keeps turtlebot3_ws intact,
links this repository's ROS package into ~/smart_vision_runtime_ws and builds
only smart_vision_amr. Installs runtime dependencies, not Gazebo/Nav2. Does not
start bringup, cameras, navigation, or change .bashrc.

--no-install   Skip apt; dependencies must already be installed.
--no-build     Prepare source workspace only (and install dependencies unless skipped).
TURTLEBOT3_WORKSPACE overrides the stock workspace (default ~/turtlebot3_ws).
EOF
}

WORKSPACE_GIVEN=false
for argument in "$@"; do
  case "$argument" in
    -h|--help) usage; exit 0 ;;
    --no-install) INSTALL_DEPS=false ;;
    --no-build) BUILD=false ;;
    -*) printf 'Unknown option: %s\n' "$argument" >&2; exit 2 ;;
    *)
      if "$WORKSPACE_GIVEN"; then usage >&2; exit 2; fi
      RUNTIME_WS=$argument
      WORKSPACE_GIVEN=true
      ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'Install ROS Humble first: /opt/ros/humble/setup.bash is missing.\n' >&2
  exit 2
fi
if [[ ! -f "$ROBOT_WS/install/local_setup.bash" ]]; then
  printf 'Stock robot workspace missing: %s/install/local_setup.bash\n' "$ROBOT_WS" >&2
  printf 'Set TURTLEBOT3_WORKSPACE to the existing TurtleBot3 workspace.\n' >&2
  exit 2
fi
if [[ ! -f "$REPO_DIR/ros2_ws/src/smart_vision_amr/setup.py" ]]; then
  printf 'Repository ROS package missing. Run this script from a complete clone.\n' >&2
  exit 2
fi

if "$INSTALL_DEPS"; then
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions python3-setuptools \
    python3-yaml python3-numpy python3-opencv v4l-utils \
    ros-humble-launch-ros ros-humble-rclpy ros-humble-robot-localization \
    ros-humble-tf2-ros ros-humble-geometry-msgs ros-humble-nav-msgs \
    ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs \
    ros-humble-diagnostic-msgs
fi

# A Pi may have unrelated overlays in .bashrc. Build over the vendor workspace
# explicitly instead of inheriting desktop navigation/simulation packages.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/local_setup.bash"
ros2 pkg prefix turtlebot3_bringup >/dev/null
mkdir -p -- "$RUNTIME_WS/src"
RUNTIME_WS=$(cd -- "$RUNTIME_WS" && pwd)
PACKAGE_LINK="$RUNTIME_WS/src/smart_vision_amr"
PACKAGE_SOURCE=$(cd -- "$REPO_DIR/ros2_ws/src/smart_vision_amr" && pwd)
if [[ -e "$PACKAGE_LINK" || -L "$PACKAGE_LINK" ]]; then
  if [[ $(readlink -f -- "$PACKAGE_LINK") != "$PACKAGE_SOURCE" ]]; then
    printf 'Refusing to replace existing package: %s\n' "$PACKAGE_LINK" >&2
    exit 2
  fi
else
  ln -s -- "$PACKAGE_SOURCE" "$PACKAGE_LINK"
fi
if "$BUILD"; then
  cd -- "$RUNTIME_WS"
  colcon build --packages-select smart_vision_amr --symlink-install \
    --base-paths "$RUNTIME_WS/src" --event-handlers console_direct+
fi
printf '\nRuntime prepared at %s; no robot process was started.\n' "$RUNTIME_WS"
printf 'After stopping the old bringup, use the supplied robot_bringup_camera_16.sh wrapper.\n'
printf 'Runtime overlay to source: %s/install/local_setup.bash\n' "$RUNTIME_WS"
