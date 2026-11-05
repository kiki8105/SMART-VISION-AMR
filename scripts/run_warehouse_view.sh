#!/usr/bin/env bash
# Connect one GUI to the existing warehouse server, without starting a world.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd -- "$SCRIPT_DIR/.." && pwd)
WORKSPACE=$(cd -- "$REPO/../.." && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: scripts/run_warehouse_view.sh\nWait up to 60 seconds for the warehouse at localhost:11578, then open the GUI.\n'
  exit 0
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
  PYTHONPATH LD_LIBRARY_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "$WORKSPACE/install/local_setup.bash"
export ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11578
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$REPO/gazebo_models:$WORKSPACE/src/turtlebot3_simulations/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}"
/usr/bin/python3 - <<'PY'
import socket
import time

deadline = time.monotonic()+60.
announced = False
try:
    while True:
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise SystemExit(
                '물류 가제보 서버에 60초 동안 연결하지 못했습니다.\n'
                '터미널 1에서 ./scripts/run_warehouse_sim.sh가 계속 실행 중인지 확인하세요.\n'
                '터미널 1의 Logs: 경로 아래 gazebo_launch.log와 dynamic_obstacles.log를 확인하세요.')
        try:
            with socket.create_connection(('127.0.0.1', 11578), timeout=min(1., remaining)):
                break
        except OSError:
            if not announced:
                print('물류 가제보 서버 기동 대기 중 · 최대 60초. 터미널 1을 실행해 두세요.', flush=True)
                announced = True
            time.sleep(min(.5, max(0., deadline-time.monotonic())))
except KeyboardInterrupt:
    raise SystemExit(130)
print('가제보 서버 연결 확인 · 화면을 엽니다. 장면 로딩은 잠시 걸릴 수 있습니다.', flush=True)
PY
exec gzclient
