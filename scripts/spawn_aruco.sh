#!/usr/bin/env bash
# Insert a generated marker into the running isolated Team2 Gazebo world.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=$(cd -- "$REPO_DIR/../.." && pwd)
MARKER_ID=0
DICTIONARY=DICT_4X4_50
X=-2.0
Y=-2.0
Z=0.20
YAW=0
ENTITY=

usage() {
  cat <<'EOF'
Usage: spawn_aruco.sh [--id 0] [--dictionary DICT_4X4_50]
                      [--x -2] [--y -2] [--z 0.20] [--yaw 0]
                      [--entity UNIQUE_NAME] [--workspace PATH]
Distances are metres. Yaw is radians. Printed front faces local -X.
Start run_sim.sh first. Default placement faces the robot's default (-3,-2) start.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --id|--dictionary|--x|--y|--z|--yaw|--entity|--workspace)
      if [[ $# -lt 2 || -z "$2" ]]; then usage >&2; exit 2; fi
      case "$1" in
        --id) MARKER_ID=$2 ;;
        --dictionary) DICTIONARY=$2 ;;
        --x) X=$2 ;;
        --y) Y=$2 ;;
        --z) Z=$2 ;;
        --yaw) YAW=$2 ;;
        --entity) ENTITY=$2 ;;
        --workspace) WORKSPACE_ROOT=$2 ;;
      esac
      shift 2
      ;;
    *) usage >&2; exit 2 ;;
  esac
done

if [[ ! "$MARKER_ID" =~ ^[0-9]+$ || ! "$DICTIONARY" =~ ^DICT_[A-Za-z0-9_]+$ ]]; then
  printf 'Invalid marker ID or dictionary name.\n' >&2
  exit 2
fi
MODEL_NAME="smart_vision_aruco_${DICTIONARY#DICT_}_id_${MARKER_ID}"
MODEL_NAME=${MODEL_NAME,,}
MODEL_FILE="$REPO_DIR/gazebo_models/$MODEL_NAME/model.sdf"
if [[ ! -f "$MODEL_FILE" ]]; then
  printf 'Generate the marker first:\n/usr/bin/python3 %q --dictionary %q --ids %q\n' \
    "$SCRIPT_DIR/generate_aruco_models.py" "$DICTIONARY" "$MARKER_ID" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
export GAZEBO_MODEL_PATH="$REPO_DIR/gazebo_models:${GAZEBO_MODEL_PATH:-}"
exec ros2 run gazebo_ros spawn_entity.py -entity "${ENTITY:-$MODEL_NAME}" \
  -file "$MODEL_FILE" -x "$X" -y "$Y" -z "$Z" -Y "$YAW" -timeout 20
