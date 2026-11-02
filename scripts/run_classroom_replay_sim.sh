#!/usr/bin/env bash
# Isolated Gazebo replay of the 2026-10-30 classroom occupancy map.
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/.." && pwd)
world=$repo_dir/ros2_ws/src/smart_vision_amr/worlds/classroom_20261030_replay.world
stress_world=$repo_dir/ros2_ws/src/smart_vision_amr/worlds/classroom_20261030_stress.world
map=$repo_dir/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml
generator=$repo_dir/ros2_ws/src/smart_vision_amr/scripts/generate_classroom_replay_world.py
hardware_nav=$repo_dir/ros2_ws/src/smart_vision_amr/config/curve_tuning_waffle_pi.yaml
hardware_nodes=$repo_dir/ros2_ws/src/smart_vision_amr/config/classroom_hardware_nodes.yaml

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: scripts/run_classroom_replay_sim.sh [--stress] [--ideal-profile] [LAUNCH_ARGUMENT:=VALUE ...]

Gazebo Classic starts at the registered classroom Home with the October 30
map, map-derived physical walls/partitions, and front/rear cameras. The
default uses real classroom navigation/node tuning with simulated time.
--stress adds one unmapped 20 cm crate and keeps the saved map unchanged.
--ideal-profile restores the old simulation-only navigation/node tuning.
Launch arguments such as gui:=false pass through to run_sim.sh.
The helper isolates ROS_DOMAIN_ID=76; it never connects to the live robot's
domain 16. See docs/classroom_replay_simulation.md for measurement limits.
EOF
  exit 0
fi

ideal_profile=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --stress) world=$stress_world; shift ;;
    --ideal-profile) ideal_profile=true; shift ;;
    --) shift; break ;;
    *) break ;;
  esac
done

if [[ ! -s "$world" || ! -s "$map" ]]; then
  printf 'Classroom replay map/world is missing. Restore the repository assets.\n' >&2
  exit 1
fi
if ! /usr/bin/python3 "$generator" --check; then
  printf 'Classroom replay world no longer matches the map. Rebuild with:\n' >&2
  printf '  /usr/bin/python3 %q\n' "$generator" >&2
  exit 1
fi
for marker_id in 0 1 2 3; do
  if [[ ! -s "$repo_dir/gazebo_models/smart_vision_aruco_4x4_50_id_$marker_id/meshes/marker.obj" ]]; then
    printf 'Missing ArUco marker model %s\n' "$marker_id" >&2
    exit 1
  fi
done

profile_args=()
if [[ "$ideal_profile" == false ]]; then
  if [[ ! -s "$hardware_nav" || ! -s "$hardware_nodes" ]]; then
    printf 'Missing classroom hardware tuning profile.\n' >&2
    exit 1
  fi
  profile_dir=$(mktemp -d -t smart-vision-classroom-replay-XXXXXXXX)
  trap 'rm -rf -- "$profile_dir"' EXIT
  profile=$profile_dir/classroom_nav_sim_time.yaml
  /usr/bin/python3 "$script_dir/build_classroom_replay_nav_profile.py" "$profile"
  profile_args=("nav_params_file:=$profile" "nodes_params_file:=$hardware_nodes")
fi

"$script_dir/run_sim.sh" "${profile_args[@]}" \
  "world:=$world" "map:=$map" \
  x_pose:=-0.188431 y_pose:=-0.220497 yaw:=-0.027514 \
  initial_x:=-0.188431 initial_y:=-0.220497 initial_yaw:=-0.027514 \
  set_initial_pose:=true camera_layout:=classroom_dual \
  use_rviz:=false "$@"
