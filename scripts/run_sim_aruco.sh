#!/usr/bin/env bash
# Start the four-marker maze in fresh SLAM mode; run_sim.sh saves on Ctrl+C.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORLD="$REPO_DIR/ros2_ws/src/smart_vision_amr/worlds/simulation_maze_aruco.world"

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: run_sim_aruco.sh [LAUNCH_ARGUMENT:=VALUE ...]
Default: four-marker maze, fresh SLAM, Gazebo GUI, no RViz.
The four marker models are included; no marker-generation step is required.
Ctrl+C: save the new map under Team2_workspace/maps, then stop.
Controller in another terminal: scripts/run_sim_controller.sh
After mapping: run_sim_aruco.sh slam:=false map:=/absolute/path/new_map.yaml
EOF
  exit 0
fi
if [[ ! -f "$WORLD" ]]; then
  printf 'Create the maze first:\n/usr/bin/python3 %q --ids 0 1 2 3\n/usr/bin/python3 %q\n' \
    "$SCRIPT_DIR/generate_aruco_models.py" "$SCRIPT_DIR/generate_aruco_maze.py" >&2
  exit 1
fi
for marker_id in 0 1 2 3; do
  model="smart_vision_aruco_4x4_50_id_$marker_id"
  if [[ ! -f "$REPO_DIR/gazebo_models/$model/model.sdf" ]]; then
    printf 'Bundled marker model is missing: %s\nRestore this repository file before launching.\n' \
      "$REPO_DIR/gazebo_models/$model/model.sdf" >&2
    exit 1
  fi
done
exec "$SCRIPT_DIR/run_sim.sh" world:="$WORLD" slam:=true use_rviz:=false "$@"
