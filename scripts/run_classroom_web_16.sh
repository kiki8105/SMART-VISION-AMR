#!/usr/bin/env bash
# Physical classroom website and saved-map preview; no simulator or mission auto-start.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
MARKERS=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers_20261030.yaml
MAP_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml
RAW_MAP_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_raw.yaml
FLOORPLAN_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/classroom_floorplan_20261030.yaml
# A new map frame must not silently reuse Home coordinates from the 2026-10-27 map.
# Inventory can be migrated separately; the original files remain untouched.
SETTINGS_FILE=${SMART_VISION_CLASSROOM_CONFIG:-$HOME/.config/smart-vision-amr/classroom-16-20261030.json}
CAMERA_TRANSPORT=${SMART_VISION_CLASSROOM_CAMERA_TRANSPORT:-compressed}
FRONT_CAMERA_TOPIC=${SMART_VISION_CLASSROOM_FRONT_CAMERA_TOPIC:-}
MINIMUM_BATTERY_VOLTAGE=${SMART_VISION_MIN_BATTERY_VOLTAGE:-10.8}
MUX_OPTIONS=()
for argument in "$@"; do
  case "$argument" in
    --map-file|--map-file=*|--raw-map-file|--raw-map-file=*|--floorplan-file|--floorplan-file=*|--markers-file|--markers-file=*)
      printf '교실 웹 지도·마커는 2026-10-30 프로필에 고정되어 있습니다. 옵션 덮어쓰기를 거부했습니다: %s\n' "$argument" >&2
      printf '다른 지도는 run_web_mission.sh에 맞는 지도와 마커를 명시해 실행하세요.\n' >&2
      exit 2
      ;;
  esac
done
case "${SMART_VISION_ESTIMATION_LOCATION:-robot}" in
  pc) MUX_OPTIONS+=(--with-motion-mux) ;;
  robot) ;;
  *) printf 'SMART_VISION_ESTIMATION_LOCATION must be pc or robot.\n' >&2; exit 2 ;;
esac
case "$CAMERA_TRANSPORT" in
  raw) CAMERA_SUFFIX= ;;
  compressed) CAMERA_SUFFIX=/compressed ;;
  *) printf 'SMART_VISION_CLASSROOM_CAMERA_TRANSPORT must be raw or compressed.\n' >&2; exit 2 ;;
esac
printf '[교실 웹] 사용 지도: %s\n[교실 웹] 사용 마커: %s\n' "$MAP_FILE" "$MARKERS" >&2

# Seed this map's own home/route once. Subsequent launches preserve user changes.
/usr/bin/python3 - "$SETTINGS_FILE" "$MARKERS" <<'PY'
import json
import os
from pathlib import Path
import sys
import tempfile
import yaml

path = Path(sys.argv[1]).expanduser()
if not path.exists():
    markers = yaml.safe_load(Path(sys.argv[2]).read_text())['markers']
    home = next(item for item in markers if item['id'] == 0)
    data = {'domains': {'16': {'route': [1, 2, 3], 'use_sim_time': False,
            'homepoint': {key: home[key] for key in ('x', 'y', 'yaw')}}},
            'last_domain_id': 16}
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.classroom-16-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
PY

exec "$SCRIPT_DIR/run_web_mission.sh" \
  --domain-id 16 --no-use-sim-time --markers-file "$MARKERS" \
  -- --map-file "$MAP_FILE" --raw-map-file "$RAW_MAP_FILE" --floorplan-file "$FLOORPLAN_FILE" \
  --config-file "$SETTINGS_FILE" --access-mode domain --require-localization-health --require-battery-health --minimum-battery-voltage "$MINIMUM_BATTERY_VOLTAGE" --marker-confirmation-mode recognized "${MUX_OPTIONS[@]}" \
  --camera-topic "/camera/image_raw$CAMERA_SUFFIX" --camera-rotation 180 \
  --front-camera-topic "$FRONT_CAMERA_TOPIC" \
  --rotation-clearance-radius 0.22 \
  --front-camera-rotation 0 "$@"
