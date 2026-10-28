#!/usr/bin/env bash
# Save the live SLAM map using the same isolated environment as run_sim.sh.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "$SCRIPT_DIR/../../.." && pwd)
if [[ "${1:-}" == --workspace ]]; then
  if [[ $# -lt 3 || -z "$2" ]]; then
    printf 'Usage: save_sim_map.sh [--workspace WORKSPACE_ROOT] /absolute/path/map_prefix\n' >&2
    exit 2
  fi
  WORKSPACE_ROOT=$2
  shift 2
fi
if [[ $# != 1 || "$1" == -h || "$1" == --help ]]; then
  printf 'Usage: save_sim_map.sh [--workspace WORKSPACE_ROOT] /absolute/path/map_prefix\nCreates map_prefix.yaml and map_prefix.pgm without replacing existing files.\n'
  if [[ $# == 1 && ( "$1" == -h || "$1" == --help ) ]]; then exit 0; fi
  exit 2
fi
MAP_PREFIX=$1
if [[ "$MAP_PREFIX" != /* || "$(basename -- "$MAP_PREFIX")" == *.* ]]; then
  printf 'Use an absolute output prefix, without .yaml or .pgm.\n' >&2
  exit 2
fi
for suffix in yaml pgm png; do
  if [[ -e "$MAP_PREFIX.$suffix" ]]; then
    printf 'Map already exists: %s.%s; choose a new prefix.\n' "$MAP_PREFIX" "$suffix" >&2
    exit 1
  fi
done
# shellcheck disable=SC1091
source "$SCRIPT_DIR/sim_env.bash" "$WORKSPACE_ROOT"
mkdir -p -- "$(dirname -- "$MAP_PREFIX")"
# Trinary unknown pixels are 205/255; .196 keeps them unknown when reloaded.
exec ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" --fmt pgm --free 0.196 --occ 0.65 \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=15.0
