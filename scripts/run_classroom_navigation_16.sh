#!/usr/bin/env bash
# PC-side physical classroom map, localization, A*, and Nav2. No Gazebo.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
CONFIG_DIR=$REPO_DIR/ros2_ws/src/smart_vision_amr/config
MAP_FILE=$REPO_DIR/ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml
export TURTLEBOT3_TUNING_DOMAIN_ID=16
LOCALIZATION_MODE=ekf_independent
ESTIMATION_LOCATION=${SMART_VISION_ESTIMATION_LOCATION:-robot}
LAUNCH_ARGUMENTS=()
for argument in "$@"; do
  case "$argument" in
    map:=*)
      printf '교실 실차 지도는 2026-10-30 지도에 고정되어 있습니다: %s\n' "$MAP_FILE" >&2
      printf 'map:= 덮어쓰기를 거부했습니다 (%s). 다른 지도는 run_hardware_tuning.sh를 사용하세요.\n' "$argument" >&2
      exit 2
      ;;
    estimation_location:=pc|estimation_location:=robot) ESTIMATION_LOCATION=${argument#estimation_location:=} ;;
    estimation_location:=*) printf 'estimation_location must be pc or robot.\n' >&2; exit 2 ;;
    localization_mode:=baseline|localization_mode:=ekf_independent)
      LOCALIZATION_MODE=${argument#localization_mode:=}
      ;;
    localization_mode:=*)
      printf 'localization_mode must be baseline or ekf_independent.\n' >&2
      exit 2
      ;;
    *) LAUNCH_ARGUMENTS+=("$argument") ;;
  esac
done
printf '[교실 실차] 사용 지도: %s\n' "$MAP_FILE" >&2
exec "$SCRIPT_DIR/run_hardware_tuning.sh" "$MAP_FILE" \
  nodes_file:="$CONFIG_DIR/classroom_hardware_nodes.yaml" \
  use_composition:=false \
  lifecycle_bond_timeout:=15.0 \
  use_motion_mux:=false \
  memory_enabled:=false "${LAUNCH_ARGUMENTS[@]}" \
  estimation_location:="$ESTIMATION_LOCATION" \
  localization_mode:="$LOCALIZATION_MODE"
