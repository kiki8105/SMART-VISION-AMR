# Source this file after locating the extracted bundle. ROS setup files need unset variables.
_turtlebot3_lab_env() {
  local helper_dir lab_dir domain port package prefix
  helper_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
  lab_dir=$(cd -- "$helper_dir/.." && pwd)
  if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$lab_dir/ros2_ws/install/local_setup.bash" ]]; then
    printf 'Build the extracted lab first:\n  cd %q\n  ./setup.sh\n' "$lab_dir" >&2
    return 1
  fi
  domain=${TURTLEBOT3_LAB_DOMAIN_ID:-76}
  if [[ ! "$domain" =~ ^[0-9]+$ ]] || (( 10#$domain > 232 )); then
    printf 'TURTLEBOT3_LAB_DOMAIN_ID must be an integer from 0 to 232.\n' >&2; return 2
  fi
  domain=$((10#$domain))
  port=${TURTLEBOT3_LAB_GAZEBO_PORT:-$((11300 + domain))}
  if [[ ! "$port" =~ ^[0-9]+$ ]] || (( 10#$port < 1024 || 10#$port > 65535 )); then
    printf 'TURTLEBOT3_LAB_GAZEBO_PORT must be an integer from 1024 to 65535.\n' >&2; return 2
  fi
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
    PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH
  source /opt/ros/humble/setup.bash || return
  if [[ -f /usr/share/gazebo/setup.sh ]]; then source /usr/share/gazebo/setup.sh || return; fi
  source "$lab_dir/ros2_ws/install/local_setup.bash" || return
  export TURTLEBOT3_LAB_ROOT="$lab_dir"
  export ROS_DOMAIN_ID="$domain"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
  export GAZEBO_MASTER_URI="${TURTLEBOT3_LAB_GAZEBO_MASTER_URI:-http://127.0.0.1:$port}"
  export TURTLEBOT3_MODEL=waffle_pi
  # All assets resolve directly from the bundle; no ~/.gazebo model registration.
  export GAZEBO_MODEL_PATH="$lab_dir/assets/models:$lab_dir/ros2_ws/src/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
  export GAZEBO_MODEL_DATABASE_URI=
  for package in smart_vision_amr turtlebot3_gazebo turtlebot3_description turtlebot3_navigation2; do
    prefix=$(ros2 pkg prefix "$package") || return
    case "$prefix" in
      "$lab_dir/ros2_ws/install"|"$lab_dir/ros2_ws/install/"*) ;;
      *) printf 'Package resolves outside this bundle: %s -> %s\nRerun ./setup.sh after moving the bundle.\n' "$package" "$prefix" >&2; return 1 ;;
    esac
  done
  printf 'Lab: %s\nROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s GAZEBO_MASTER_URI=%s\n' \
    "$lab_dir" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$GAZEBO_MASTER_URI"
}
_turtlebot3_lab_env "$@"
