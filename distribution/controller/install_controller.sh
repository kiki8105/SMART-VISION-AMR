#!/usr/bin/env bash
# Install a locally built controller DEB. ROS Humble apt sources must exist.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CHECK_ONLY=false
if [[ "${1:-}" == --check ]]; then CHECK_ONLY=true; shift; fi
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: install_controller.sh [--check] [PATH_TO_CONTROLLER_DEB]\nRequires Ubuntu 22.04 and configured ROS 2 Humble apt sources.\n'
  exit 0
fi
if (( $# > 1 )); then printf 'Expected at most one DEB path.\n' >&2; exit 2; fi
CONTROLLER_DEB=${1:-$SCRIPT_DIR/../releases/turtlebot3-controller_1.1.0_all.deb}
if [[ ! -f "$CONTROLLER_DEB" ]]; then
  printf 'Controller DEB not found: %s\nBuild it with: python3 distribution/controller/build_deb.py\n' "$CONTROLLER_DEB" >&2
  exit 1
fi
CONTROLLER_DEB=$(realpath -- "$CONTROLLER_DEB")
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  printf 'Supported OS: Ubuntu 22.04. Current OS: %s\n' "$PRETTY_NAME" >&2
  exit 1
fi
if [[ $(dpkg-deb -f "$CONTROLLER_DEB" Package) != turtlebot3-controller ]]; then
  printf 'Not a turtlebot3-controller package: %s\n' "$CONTROLLER_DEB" >&2
  exit 1
fi
if "$CHECK_ONLY"; then
  dpkg-deb -f "$CONTROLLER_DEB" Package Version Architecture Depends
  printf 'Package is valid. Installation requires ROS Humble apt sources, Internet and sudo.\n'
  exit 0
fi
sudo apt-get update
sudo apt-get install -y "$CONTROLLER_DEB"
turtlebot3-controller --check
printf 'Installed. Open TurtleBot3 Controller from the application menu.\n'
