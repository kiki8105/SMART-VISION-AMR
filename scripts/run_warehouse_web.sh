#!/usr/bin/env bash
# Web-controlled logistics simulation; never joins the physical robot domain.
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/.." && pwd)
map_file=$repo_dir/ros2_ws/src/smart_vision_amr/maps/warehouse_logistics.yaml
markers_file=$repo_dir/ros2_ws/src/smart_vision_amr/config/aruco_warehouse_markers.yaml
floorplan_file=$repo_dir/ros2_ws/src/smart_vision_amr/config/warehouse_floorplan.yaml
sim_domain=${SMART_VISION_WAREHOUSE_DOMAIN:-178}
http_port=${SMART_VISION_WAREHOUSE_PORT:-8793}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: scripts/run_warehouse_web.sh [WEB_SERVER_OPTIONS ...]
Run after scripts/run_warehouse_sim.sh in another terminal.
Default URL: http://127.0.0.1:8793/ (inventory: /inventory).
Select Home 0 or Sector 1..10 using this warehouse's marker coordinates.
Rear camera only, upright image, 0.22 m chassis; initial pose is set in the web.
SMART_VISION_WAREHOUSE_DOMAIN/PORT/CONFIG override isolated defaults.
EOF
  exit 0
fi

if [[ ! "$sim_domain" =~ ^[0-9]{1,3}$ ]] || ((10#$sim_domain > 232 || 10#$sim_domain == 16)); then
  printf '%s\n' 'Warehouse simulation ROS domain must be 0..232 except physical domain 16.' >&2
  exit 2
fi
for required in "$map_file" "$markers_file" "$floorplan_file"; do
  if [[ ! -s "$required" ]]; then
    printf 'Missing warehouse input: %s\n' "$required" >&2
    exit 1
  fi
done
settings_file=${SMART_VISION_WAREHOUSE_CONFIG:-$HOME/.config/smart-vision-amr/warehouse-sim-${sim_domain}.json}

exec "$script_dir/run_web_mission.sh" \
  --domain-id "$sim_domain" --port "$http_port" --markers-file "$markers_file" \
  -- --map-file "$map_file" --raw-map-file "$map_file" --floorplan-file "$floorplan_file" \
  --config-file "$settings_file" --access-mode domain \
  --require-localization-health --marker-confirmation-mode recognized \
  --observed-obstacles-topic /smart_vision_amr/display_obstacles \
  --camera-topic /camera/image_raw/compressed --camera-rotation 0 \
  --front-camera-topic '' --rotation-clearance-radius 0.22 \
  --sector-dwell-seconds 5 "$@"
