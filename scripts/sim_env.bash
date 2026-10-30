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
  # Marker meshes and textures are versioned with the project; no per-user
  # ~/.gazebo/models registration is needed after cloning the repository.
  export GAZEBO_MODEL_PATH="$repo_dir/gazebo_models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

  # Optional user-space ROS package for local experiments. Normal installations
  # already live under /opt/ros/humble and need neither override.
  if [[ -n "${SMART_VISION_EXTRA_ROS_PREFIX:-}" ]]; then
    if [[ ! -d "$SMART_VISION_EXTRA_ROS_PREFIX/share/ament_index/resource_index/packages" ]]; then
      printf 'Invalid extra ROS prefix: %s\n' "$SMART_VISION_EXTRA_ROS_PREFIX" >&2
      return 1
    fi
    export AMENT_PREFIX_PATH="$SMART_VISION_EXTRA_ROS_PREFIX:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$SMART_VISION_EXTRA_ROS_PREFIX:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$SMART_VISION_EXTRA_ROS_PREFIX/lib:$LD_LIBRARY_PATH"
  fi
  if [[ -n "${SMART_VISION_EXTRA_LIBRARY_DIR:-}" ]]; then
    if [[ ! -d "$SMART_VISION_EXTRA_LIBRARY_DIR" ]]; then
      printf 'Invalid extra library directory: %s\n' "$SMART_VISION_EXTRA_LIBRARY_DIR" >&2
      return 1
    fi
    export LD_LIBRARY_PATH="$SMART_VISION_EXTRA_LIBRARY_DIR:$LD_LIBRARY_PATH"
  fi

  # Replace inherited hardware settings, including the user's Humble domain64 alias.
  export ROS_DOMAIN_ID=76
  export ROS_LOCALHOST_ONLY=1
  export GAZEBO_MASTER_URI=http://127.0.0.1:11376
  export TURTLEBOT3_MODEL=waffle_pi
  export SMART_VISION_SIM_WORKSPACE="$workspace_root"
  # Pass the empty optional profile explicitly. A sourced script otherwise
  # inherits this function's workspace argument as its own $1.
  source "$script_dir/dds_env.bash" "" || return

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
