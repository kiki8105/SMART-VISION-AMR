#!/usr/bin/env bash
# Source this helper for ROS CLI commands in the same isolated simulation domain.
# It intentionally does not enable nounset: ROS Humble setup files use unset vars.
# shellcheck disable=SC1091

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'Source this helper: source scripts/sim_env.bash [WORKSPACE_ROOT]\n' >&2
  exit 2
fi

_smart_vision_sim_env() {
  local script_dir repo_dir workspace_root package prefix
  script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
  repo_dir=$(cd -- "$script_dir/.." && pwd)
  workspace_root=${1:-$(cd -- "$repo_dir/../.." && pwd)}
  if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$workspace_root/install/local_setup.bash" ]]; then
    printf 'Simulation workspace is not built. Run:\n  %q %q\n' \
      "$script_dir/setup_sim.sh" "$workspace_root" >&2
    return 1
  fi
  workspace_root=$(cd -- "$workspace_root" && pwd)
  # Keep display/system settings, but rebuild ROS/Gazebo paths from known roots.
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
    PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH
  printf '+ source /opt/ros/humble/setup.bash\n'
  source /opt/ros/humble/setup.bash || return
  if [[ -f /usr/share/gazebo/setup.sh ]]; then
    printf '+ source /usr/share/gazebo/setup.sh\n'
    source /usr/share/gazebo/setup.sh || return
  fi
  # local_setup avoids replaying other workspaces captured by setup.bash.
  printf '+ source %q\n' "$workspace_root/install/local_setup.bash"
  source "$workspace_root/install/local_setup.bash" || return

  # Replace inherited hardware settings, including the user's Humble domain64 alias.
  export ROS_DOMAIN_ID=76
  export ROS_LOCALHOST_ONLY=1
  export GAZEBO_MASTER_URI=http://127.0.0.1:11376
  export TURTLEBOT3_MODEL=waffle_pi
  export SMART_VISION_SIM_WORKSPACE="$workspace_root"

  for package in smart_vision_amr turtlebot3_gazebo turtlebot3_description turtlebot3_navigation2; do
    prefix=$(ros2 pkg prefix "$package") || return
    case "$prefix" in
      "$workspace_root/install"|"$workspace_root/install/"*) ;;
      *) printf '%s resolved outside Team2: %s\n' "$package" "$prefix" >&2; return 1 ;;
    esac
  done
  printf 'Workspace: %s\nROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s\nGAZEBO_MASTER_URI=%s\n' \
    "$workspace_root" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$GAZEBO_MASTER_URI"
}

_smart_vision_sim_env "$@"
