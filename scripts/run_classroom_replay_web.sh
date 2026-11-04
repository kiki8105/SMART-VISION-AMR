#!/usr/bin/env bash
# Web view for the isolated classroom Gazebo replay; never joins robot domain 16.
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/.." && pwd)
map_file=$repo_dir/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml
raw_map_file=$repo_dir/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_raw.yaml
markers_file=$repo_dir/ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers_20261030.yaml
floorplan_file=$repo_dir/ros2_ws/src/smart_vision_amr/config/classroom_floorplan_20261030.yaml
http_port=${SMART_VISION_CLASSROOM_SIM_PORT:-8791}
sim_domain=${SMART_VISION_CLASSROOM_SIM_DOMAIN:-76}
front_camera_topic=${SMART_VISION_CLASSROOM_SIM_FRONT_CAMERA_TOPIC:-}
rotation_radius=${SMART_VISION_CLASSROOM_SIM_ROTATION_RADIUS:-0.24}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: scripts/run_classroom_replay_web.sh [WEB_SERVER_OPTIONS ...]

Run after scripts/run_classroom_replay_sim.sh in another terminal.
Connects only to the configured local simulation ROS domain (default 76),
serves the matching classroom map,
markers, rear camera, live obstacle pixels, and active detour route.
Default browser URL: http://127.0.0.1:8791/
Set SMART_VISION_CLASSROOM_SIM_PORT to change the HTTP port.
Set SMART_VISION_CLASSROOM_SIM_DOMAIN=176 for run_classroom_fault_sim.sh.
Set SMART_VISION_CLASSROOM_SIM_FRONT_CAMERA_TOPIC to enable an optional front feed.
Set SMART_VISION_CLASSROOM_SIM_ROTATION_RADIUS=0.22 only for a matching --physical-footprint simulator.
EOF
  exit 0
fi

for required in "$map_file" "$raw_map_file" "$markers_file" "$floorplan_file"; do
  if [[ ! -s "$required" ]]; then
    printf 'Missing classroom replay input: %s\n' "$required" >&2
    exit 1
  fi
done

if [[ ! "$sim_domain" =~ ^[0-9]{1,3}$ ]] || (( 10#$sim_domain > 232 || 10#$sim_domain == 16 )); then
  printf 'Simulation ROS domain must be 0-232 except physical domain 16.\n' >&2
  exit 2
fi

# Keep each isolated ROS domain in its own browser mission/home configuration.
settings_file=${SMART_VISION_CLASSROOM_SIM_CONFIG:-$HOME/.config/smart-vision-amr/classroom-sim-${sim_domain}-20261030.json}

exec "$script_dir/run_web_mission.sh" \
  --domain-id "$sim_domain" --port "$http_port" --markers-file "$markers_file" \
  -- --map-file "$map_file" --raw-map-file "$raw_map_file" --floorplan-file "$floorplan_file" \
  --config-file "$settings_file" --access-mode domain \
  --require-localization-health --marker-confirmation-mode recognized \
  --camera-topic /camera/image_raw/compressed --camera-rotation 0 \
  --front-camera-topic "$front_camera_topic" \
  --rotation-clearance-radius "$rotation_radius" \
  --front-camera-rotation 0 "$@"
