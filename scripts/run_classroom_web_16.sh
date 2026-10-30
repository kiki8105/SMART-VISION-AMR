#!/usr/bin/env bash
# Physical classroom website and saved-map preview; no simulator or mission auto-start.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
MARKERS=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers.yaml
MAP_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml
FLOORPLAN_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/config/classroom_floorplan.yaml
SETTINGS_FILE=${SMART_VISION_CLASSROOM_CONFIG:-$HOME/.config/smart-vision-amr/classroom-16.json}
CAMERA_TRANSPORT=${SMART_VISION_CLASSROOM_CAMERA_TRANSPORT:-compressed}
MINIMUM_BATTERY_VOLTAGE=${SMART_VISION_MIN_BATTERY_VOLTAGE:-11.2}
MUX_OPTIONS=()
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
  -- --map-file "$MAP_FILE" --floorplan-file "$FLOORPLAN_FILE" \
  --config-file "$SETTINGS_FILE" --access-mode domain --require-localization-health --require-battery-health --minimum-battery-voltage "$MINIMUM_BATTERY_VOLTAGE" --require-marker-wall-alignment "${MUX_OPTIONS[@]}" \
  --camera-topic "/camera/image_raw$CAMERA_SUFFIX" --camera-rotation 180 \
  --front-camera-topic "/camera/front/image_raw$CAMERA_SUFFIX" \
  --front-camera-rotation 0 "$@"
