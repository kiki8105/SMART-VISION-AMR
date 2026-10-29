# Team2 Gazebo 주행 검증

Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11용입니다. Gazebo Waffle Pi가 `/scan`, `/odom`, TF를 제공하고 기존 코드가 `/smart_vision_amr/goal` → A* → `/smart_vision_amr/path` → Nav2 `/follow_path` → `/cmd_vel` 순서로 주행합니다. AMCL과 controller만 실행하므로 목표는 RViz의 전용 2D Goal Pose 또는 위 goal 토픽으로 지정합니다.

## 미로 코너 튜닝과 기본값 저장

터미널 1은 저장한 ArUco 미로 지도와 별도 튜닝 YAML을 사용합니다.

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_curve_tuning_sim.sh
```

터미널 2의 조종기는 같은 튜닝 YAML을 기본값 저장 대상으로 연결합니다.
이 구성에서는 지도·Goal·파라미터 기능을 사용하고, 수동 주행은 웹 조이스틱으로 합니다.

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_curve_tuning_sim_controller.sh
```

`값 적용`으로 노드가 수락한 값을 `기본값 저장`하면 해당 원본 YAML이 갱신되고
다음 실행에도 유지됩니다. 저장 전 백업을 만들며 다른 값과 주석은 보존합니다.
화살표 메뉴에서 연결된 저장 대상을 바꾸거나 전체 YAML을 별도 내보낼 수 있습니다.
설정 파일은 `config/curve_tuning_simulation.yaml`과
`config/curve_tuning_simulation_nodes.yaml`입니다. 시뮬레이션과 실차의 노드 기본값은
각각 `simulation_nodes.yaml`과 `nodes.yaml`로 분리합니다.

`FollowPath.forward_prune_distance`는 로컬 컨트롤러에 전달할 전방 경로 범위,
`FollowPath.prune_distance`는 경로 시작점 검색·정리에 사용하는 거리입니다.
현재 미로 튜닝 프로필은 각각 0.75 m와 0.25 m, 전진 속도 상한은 0.12 m/s입니다.
두 경로 범위는 함께 조정하며, DWB가 구성 시 읽으므로 값 수락·기본값 저장 후
Nav2 리셋 또는 프로세스 재실행으로 반영합니다. 조종기 툴팁과 상태줄에도 안내합니다.

진행 제한은 20초 동안 0.15 m이며, 제자리 회전만으로는 위치 진행으로 인정되지 않습니다.
실패로 임무가 끝난 뒤에는 설정을 바꾸고 새 Goal을 전송해야 합니다.
`drive_enabled`, `use_sim_time`, 지도 경로 등 운용·실행 인자 값은 기본값 저장에서 제외합니다.

실차 파인튜닝 원본은 `config/curve_tuning_waffle_pi.yaml`입니다. PC에서
`scripts/run_hardware_tuning.sh /실차로/매핑한/map.yaml`과
`scripts/run_hardware_tuning_controller.sh`를 별도 터미널에서 실행합니다.
실차 기본 Domain은 16이며, 로봇의 bringup은 별도로 실행합니다. 두 스크립트에서 동일한
`TURTLEBOT3_TUNING_DOMAIN_ID`, `TURTLEBOT3_TUNING_PARAMS_FILE`로 대상을 바꿀 수 있습니다.
실차 프로필은 실물 차량에서 검증한 값이 아니므로 시뮬레이션 결과와 구분합니다.

2026-10-26 미로 비교 시험에서는 기본 0.22 m/s와 속도만 낮춘 0.12 m/s 모두
첫 코너에서 진행 제한으로 실패했습니다. A* 팽창 9 cells는 경로가 없었고,
8 cells는 경로는 생성했지만 진행 제한으로 실패했습니다. 속도 0.12 m/s,
A* 팽창 6 cells를 유지하면서 두 pruning 거리만 0.25/0.75 m로 바꾼 시험 E는
22.717 m 주행 후 `SUCCEEDED`, 도착 오차 0.161 m(허용 0.20 m)를 기록했습니다.
시뮬레이션 튜닝 파일은 이 시험 E와 YAML 값이 일치합니다. 실차 파일은 기존
전체 Nav2 설정에 속도 상한 두 항목과 pruning 거리 두 항목만 옮긴 시작용 템플릿입니다.
한 차례의 전체 경로 성공이며, 다른 시작점·지도·실차에서의 검증은 별도입니다.
증거는 `/home/kiki/Team2_workspace/log/curve_tuning_20261026/all_trials.json`,
`E_supplement.json`, `E_comparison.png`에 있습니다.

## 작업 위치와 빌드

```text
~/Team2_workspace/
├── src/
│   ├── SMART-VISION-AMR/           # 팀 코드와 시뮬레이션 설정
│   │   └── ros2_ws/src/smart_vision_amr/
│   ├── turtlebot3/                # 공식 모델/내비게이션 리소스
│   └── turtlebot3_simulations/    # 공식 Gazebo 리소스
├── build/
├── install/
└── log/
```

빌드 루트는 `Team2_workspace`입니다. 저장소 안의 `ros2_ws`에서 중복 빌드할 필요가 없습니다. 기존 `~/turtlebot3`, UAV 등 다른 워크스페이스를 source하지 않습니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/setup_sim.sh
```

필요한 시스템 패키지를 검사하고 공식 ROBOTIS Humble 저장소에서 필요한 패키지만 가져와 빌드합니다. 시스템 의존성이 없으면 필요한 apt 명령을 출력합니다. 의존성 저장소에 수정 사항이 있으면 덮어쓰지 않습니다. 다른 경로에서는 `./scripts/setup_sim.sh /절대경로/워크스페이스`를 사용합니다.

공식 리소스 고정 커밋:

- `ROBOTIS-GIT/turtlebot3`: `90a68bd2e3c61c12966779da89d8eeaec82730e9`
- `ROBOTIS-GIT/turtlebot3_simulations`: `a35a56c8b04877dc89772b598084d8ce648a9023`

## 실행과 목표 지정

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh
```

Gazebo와 RViz가 열립니다. 로봇은 `(-3, -2, yaw=0)`에서 시작하고 AMCL 초기 위치도 자동 설정됩니다. 지도와 로봇 위치가 나타난 뒤 RViz의 **2D Goal Pose**를 클릭하고 도착 방향으로 드래그합니다. 발행 토픽은 `/smart_vision_amr/goal`입니다. 첫 목표는 `(-1, -2)`, 중앙 장애물 우회 목표는 `(3, 2)`가 적합합니다.

GUI 없는 실행:

```bash
./scripts/run_sim.sh gui:=false use_rviz:=false
```

다른 터미널에서 ROS 명령을 사용할 때는 환경을 먼저 불러옵니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source scripts/sim_env.bash
ros2 topic pub --once /smart_vision_amr/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -1.0, y: -2.0}, orientation: {w: 1.0}}}'
ros2 topic echo /smart_vision_amr/navigation_status
```

주행 취소:

```bash
source scripts/sim_env.bash
ros2 service call /smart_vision_amr/cancel_navigation std_srvs/srv/Trigger '{}'
```

전체 종료는 실행 터미널에서 `Ctrl+C`입니다. 재현 가능한 새 테스트는 종료 후 다시 실행해 같은 시작 위치에서 진행합니다.

## 자동 주행 검증

터미널 1에서 시뮬레이션을 실행한 상태로 터미널 2에서 실행합니다. 가상 로봇에 목표를 보내 실제로 이동시킵니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/check_sim.sh
```

검사 항목: 시계 진행, 지도·LiDAR·odom 수신, `map → base_footprint` TF, controller 활성화, FollowPath action 준비, A* 경로 생성, 0이 아닌 속도 명령, 0.3 m 이상 이동, `SUCCEEDED` 상태, 목표 오차 0.35 m 이내. 성공 시 JSON 결과와 종료 코드 0을 출력합니다.

중앙 장애물 우회는 시뮬레이션을 새로 실행한 뒤:

```bash
./scripts/check_sim.sh --goal-x 3.0 --goal-y 2.0 --timeout 240
```

준비 제한 시간은 기본 120초, 주행 제한 시간은 기본 150초이며 실제 시간 기준입니다. `--ready-timeout`, `--timeout`으로 조정합니다. 목표는 지도 안의 충분히 넓은 자유 공간을 지정합니다.

## 포함된 PyQt 조종기로 SLAM을 처음부터 다시 하기

실행 중인 시뮬레이션을 `Ctrl+C`로 종료한 후 터미널 1에서 시작합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh slam:=true use_motion_mux:=false
```

매 실행마다 새 SLAM 지도와 pose graph를 만듭니다. 이 모드는 Gazebo, 로봇 TF, `slam_toolbox`, RViz를 실행합니다. 수동 주행 중 속도 명령이 겹치지 않도록 AMCL·고정 지도 서버·A*·Nav2 controller는 실행하지 않습니다.

SLAM 터미널에서 **Ctrl+C로 종료하면 지도를 먼저 자동 저장한 뒤 Gazebo와 SLAM을 종료합니다.**
기본 경로는 `~/Team2_workspace/maps/slam_날짜_시간_실행번호/map.yaml` 및
`map.pgm`이며 시작할 때와 저장 완료 시 터미널에 실제 경로를 출력합니다. 이전 지도는 덮어쓰지 않습니다.
저장 실패 시 SLAM을 유지하고, 다시 Ctrl+C를 누르면 저장을 재시도합니다.
저장 중 Ctrl+C를 여러 번 눌러도 현재 저장이 완료될 때까지 기다립니다.
이 자동 저장은 AMCL 주행용 YAML·PGM 지도이며 SLAM pose graph 저장은 별도입니다.

자동 저장 위치를 직접 지정하려면 확장자를 제외한 절대경로를 사용합니다.

```bash
./scripts/run_sim.sh --map-prefix ~/Team2_workspace/maps/team2_slam slam:=true use_motion_mux:=false
```

기존 출력 파일이 있으면 실행 전에 알려 주므로 새 경로를 지정합니다. 강제 종료나 SLAM 프로세스 자체의 오류 종료는
정상적인 종료 요청과 달리 저장 시점에 `/map`이 사라질 수 있으므로 자동 저장을 보장할 수 없습니다.

터미널 2에서 저장소에 포함된 **`tools/cmd_vel_controller.py`**를 연결합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim_controller.sh
```

조종기는 Domain **76**, 로봇 namespace **/**, 시뮬레이션 시각으로 연결됩니다. 실기용 저장 설정과 분리된 설정 파일을 사용합니다. 지도에서 로봇 위치를 확인한 뒤 방향 버튼을 누르고 있는 동안 주행하고, 놓으면 정지합니다. 벽·선반·중앙 장애물 주변의 열린 통로를 천천히 돌아 지도 빈 영역을 채웁니다. 수동 조종에는 장애물 회피 기능이 없습니다.

조종기의 파라미터 저장은 소스의 `ros2_ws/src/smart_vision_amr/config/simulation.yaml`과
`simulation_nodes.yaml`에 반영되며, `run_sim.sh`는 다음 실행에도 이 파일들을 직접 읽습니다.
별도 YAML을 쓰는 경우 실행기의 `nav_params_file:=/절대경로/nav.yaml`,
`nodes_params_file:=/절대경로/nodes.yaml`과 조종기의 `--nav-params-file /절대경로/nav.yaml`,
`--nodes-params-file /절대경로/nodes.yaml`에 같은 파일을 지정하세요.
지도 파일, AMCL 시작 위치, 시뮬레이션 시각, 기억 기능의 활성화·저장 경로는 launch 인자로 관리합니다.

방 외곽만 돌면 목표 주변에 미확인 셀이 남을 수 있습니다. 기본 우회 목표 `(3, 2)`를 검증하려면 `(2.2, 2.8) → (2.2, 1.2) → (3.8, 1.2) → (3.8, 2.8)`의 열린 공간도 돌아 여러 방향에서 관측합니다. 목표 셀뿐 아니라 주변도 자유 공간이어야 합니다. A*는 미확인·장애물 영역을 0.30 m 팽창시키므로, 빈 셀을 선택해도 주변 미확인 영역 때문에 경로가 거부될 수 있습니다.
시작점 주변도 관측해야 합니다. 기본 시작점으로 돌아온 뒤 `(-2, -2)`까지 전진하고 `(-3, -2)`로 돌아오면 시작점 주변을 반대쪽에서도 스캔할 수 있습니다. 자동 검증 경로에는 이 왕복이 포함됩니다.

조종기와 지도·파라미터 패널 모듈은 모두 저장소의 `tools/`에 포함되어 있습니다.
별도 파일 복사나 patch 적용 없이 실행하며, 기존 `~/turtlebot3/install/setup.bash`를
추가로 source할 필요는 없습니다. 다른 조종기를 쓰려는 경우에만
`SMART_VISION_CONTROLLER_SCRIPT=/절대경로/cmd_vel_controller.py` 또는
`--controller-script /절대경로/cmd_vel_controller.py`로 바꿉니다.

PyQt 수동 조종은 `/cmd_vel`을 직접 발행합니다. 수동 SLAM 실습에서는
위 예시처럼 `use_motion_mux:=false`를 지정합니다. 저장 지도 주행은 기본 mux를
유지하고 `./scripts/run_sim_controller.sh`로 PyQt의 지도·Goal·파라미터 기능을
사용합니다. 조종기는 화면을 열거나 정지 상태일 때 속도 발행자를 만들지 않습니다.
웹 mux가 `/cmd_vel`을 담당하면 PyQt 방향 버튼은 잠기고 웹 조이스틱을 안내합니다.
이때 수동 조작은
[웹 임무 안내](web_mission.md)의 웹 조이스틱을 사용하고 PyQt 방향 버튼과
동시에 사용하지 않습니다.

종료 전 중간 지도를 따로 저장하려면 주행을 멈춘 상태에서 터미널 3으로 아래 명령을 실행합니다.
출력 경로는 확장자를 제외한 절대경로이며 기존 파일은 덮어쓰지 않습니다. Ctrl+C 자동 저장도 그대로 작동합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/save_sim_map.sh ~/Team2_workspace/maps/my_slam
```

`my_slam.yaml`과 `my_slam.pgm`이 생성됩니다. 이 파일은 AMCL 주행용 지도이며 SLAM 세션을 이어 만드는 pose graph 저장과는 별개입니다.

저장 스크립트는 `free_thresh=0.196`을 사용해 미확인 영역을 재로딩해도 유지합니다. [Humble의 trinary 지도 저장·로드 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)은 미확인 픽셀을 205로 저장하므로 이 값이 자유 공간으로 바뀌지 않는 임계값을 적용했습니다.

자동 저장·종료 순서와 저장 실패 후 재시도는 Gazebo를 띄우지 않고 검증할 수 있습니다.

```bash
python3 scripts/test_run_sim_autosave.py -v
```

### 새 지도에서 A*/Nav2 검증

조종기와 SLAM 실행을 종료한 뒤 새 시뮬레이션을 저장 지도와 함께 실행합니다. 기본 월드의 시작 위치를 그대로 사용한 경우:

```bash
./scripts/run_sim.sh map:="$HOME/Team2_workspace/maps/my_slam.yaml"
```

SLAM에서 사용한 시작 좌표계가 다르면 Gazebo의 실제 spawn 좌표 `x_pose/y_pose/yaw`와 지도 안의 초기 위치 `initial_x/initial_y/initial_yaw`를 각각 지정합니다. 이 PC의 기본 월드에서는 SLAM 시작 위치가 지도에서도 약 `(-3, -2, 0)`임을 TF로 확인했습니다. 임의로 `(0, 0)`을 초기 위치로 지정하지 마세요. RViz **2D Pose Estimate**로 위치를 보정할 수도 있습니다.

다른 터미널에서 새 지도 기반 주행을 검사합니다.

```bash
./scripts/check_sim.sh --goal-x -1 --goal-y -2
# 새 실행에서 중앙 장애물 우회
./scripts/check_sim.sh --goal-x 3 --goal-y 2 --timeout 240
```

조종기를 다시 실행하면 지도·로봇·계획 경로를 볼 수 있습니다.
지도에서 단일 Goal을 선택해 보내면 기존 A* 목표로 연결됩니다.
**이동 취소/정지**는 FollowPath를 취소합니다. 기본 mux가 실행 중인 저장 지도
주행에서는 PyQt 방향 버튼을 사용하지 않습니다. 이 구성의 주행 인터페이스는
단일 목표이며 웨이포인트 action 서버는 제공하지 않습니다.

### SLAM + 기존 조종기 자동 검증

기본 월드에서 `./scripts/run_sim.sh slam:=true use_motion_mux:=false`를 새로 실행한 직후,
GUI 조종기를 닫은 상태로 아래 명령을 실행합니다.

```bash
./scripts/check_slam.sh
```

실제 조종기 클래스의 버튼 유지 주행·정지 기능으로 방 외곽과 목표 주변 사각형을 돌며 odom 이동, 정지 후 이동량, SLAM 지도 확장, TF, 지도 저장을 확인합니다. 저장 지도 재로딩 시 전체 셀 일치, 목표 주변 0.4 m 범위의 자유 공간, 실제 A*의 장애물 팽창을 적용한 `(3, 2)` 우회 경로 생성까지 통과해야 성공입니다. 수 분이 걸리며 결과와 지도는 `Team2_workspace/log/slam_validation/` 아래에 저장됩니다. 이 검사는 기본 월드와 시작 위치 전용입니다. 자동 검증 중 별도 조종기나 속도 발행기를 실행하지 마세요. 저장한 지도 YAML을 위 `map:=` 인자로 지정해 주행 검증을 이어갑니다.

## 팀원 코드 연결

팀원의 변경을 이 체크아웃에 반영하고 `./scripts/setup_sim.sh`로 재빌드한 뒤 시뮬레이션을 재시작합니다. 기존 goal/path/action/velocity 인터페이스를 유지하면 별도 어댑터가 필요 없습니다. 다른 노드로 바꾸면 `launch/simulation.launch.py`의 해당 실행 부분을 교체합니다. 속도 명령을 직접 발행하는 제어기는 기존 Nav2와 동시에 명령을 보내지 않도록 구성합니다.

주요 파일은 `ros2_ws/src/smart_vision_amr` 아래에 있습니다.

| 파일 | 용도 |
|---|---|
| `launch/simulation.launch.py` | Gazebo, 모델, TF, AMCL, controller, 기존 노드, RViz 실행 |
| `config/simulation.yaml` | 위치 추정, DWB controller, local costmap |
| `config/slam_simulation.yaml` | 새 SLAM 지도 작성, LiDAR·TF·지도 갱신 설정 |
| `config/nodes.yaml` | 기존 A*, FollowPath, 장애물 관측 설정 |
| `worlds/simulation_room.world` | 벽 4개, 중앙 장애물, 선반 2개 |
| `maps/simulation_room.yaml`, `.pgm` | 위 월드와 동일한 0.05 m 격자 지도 |
| `rviz/simulation.rviz` | 지도, 로봇, LiDAR, 경로, 목표 도구 |
| `scripts/generate_sim_world.py` | 같은 형상 정의로 월드·지도 동시 생성 |

장애물 배치를 바꿀 때는 생성 스크립트의 `BOXES`를 수정하고:

```bash
python3 ros2_ws/src/smart_vision_amr/scripts/generate_sim_world.py
python3 ros2_ws/src/smart_vision_amr/scripts/generate_sim_world.py --check
./scripts/setup_sim.sh
```

`classroom_map`은 실물 교실 지도이며 이 월드와 형태가 다릅니다. 교실을 검증하려면 교실과 일치하는 월드와 지도를 함께 제공해야 합니다.

실행 인자:

```bash
./scripts/run_sim.sh x_pose:=-3.0 y_pose:=-2.0 yaw:=0.0
./scripts/run_sim.sh memory_enabled:=true
./scripts/run_sim.sh world:=/절대경로/custom.world map:=/절대경로/custom.yaml
```

시뮬레이션 메모리는 기본 비활성이며 활성화하면 `~/.ros/smart_vision_amr/simulation_memory.json`을 사용합니다. 실기 메모리 경로와 분리되어 있습니다. Gazebo LiDAR를 받을 수 있도록 관측 노드에 sensor data QoS를 적용했습니다.

## 확인 명령과 문제 해결

```bash
source scripts/sim_env.bash
ros2 topic echo /clock --once
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /smart_vision_amr/observed_obstacles --once
ros2 topic echo /smart_vision_amr/path --once
```

- 시계가 멈추면 Gazebo 일시 정지 여부를 확인합니다.
- TF가 없으면 `/scan`, `/odom`, AMCL 로그를 확인합니다. AMCL 초기 위치는 실제 스폰 위치에 대응하는 저장 지도 좌표여야 합니다.
- 경로가 비면 시작·목표가 벽 또는 A*의 0.30 m 팽창 영역에 들어갔는지 확인합니다.
- 속도 명령은 있는데 움직이지 않으면 모델 스폰 성공 및 시계 진행을 확인합니다.
- 렌더링 문제가 있으면 GUI를 끄고 주행을 검사할 수 있습니다. LiDAR/카메라 렌더링을 위해 그래픽 드라이버와 DISPLAY가 필요할 수 있습니다.
- RViz의 첫 지도 수신 때 `active samplers ... same texture image unit` 로그가 1회 나와도 지도와 Global Status가 정상이면 주행 검증을 계속할 수 있습니다. [동일한 RViz 공식 이슈](https://github.com/ros2/rviz/issues/463)가 있습니다.
- 같은 포트의 Gazebo를 여러 개 실행하지 않습니다. 이전 실행을 종료한 뒤 재시작합니다.

run/check 환경은 `ROS_DOMAIN_ID=76`, `ROS_LOCALHOST_ONLY=1`, `GAZEBO_MASTER_URI=http://127.0.0.1:11376`을 적용합니다. 실기의 domain 16 및 PC 별칭의 domain 64와 분리되어 있으며, 두 터미널 모두 제공 스크립트를 사용해야 서로 통신합니다.

## 이 PC에서 확인한 결과 (2026-10-26)

| 검사 | 결과 |
|---|---|
| colcon 빌드 | 4개 패키지 성공 |
| rosdep | 전체 시스템 의존성 충족 |
| 직선 목표 `(-1, -2)` | 41 poses, odom 변위 1.78 m, 도착 오차 0.21 m, SUCCEEDED |
| 장애물 우회 목표 `(3, 2)` | Gazebo/RViz 동시 실행, 약 57초, 도착 오차 0.16 m, SUCCEEDED |
| 주행 취소 | CANCELED, 속도 명령 0, 이후 3초 odom 변위 약 0.0002 m |
| 화면 | Gazebo 월드와 RViz 지도·로봇·LiDAR·경로 표시 확인 |

실행 로그는 `/home/kiki/Team2_workspace/log/verification_straight.log`, `verification_detour.log`, `verification_cancel.log`에 있습니다. 주행 검증용 설정은 실제 장비의 동작 성능을 보증하지 않으며 팀원 코드 변경 후에는 위 자동 검증을 다시 실행합니다.

### 기존 tools 조종기 + 새 SLAM 지도 검증 (2026-10-26)

| 검사 | 결과 |
|---|---|
| 기존 조종기 버튼으로 SLAM 주행 | 약 37.46 m, 외곽·목표 사각형·시작점 왕복 13개 경유 지점 통과 |
| 지도 생성 | 기존 지도 없이 시작, 확인 영역 2.93 → 75.97 m², 201×161 cells, 0.05 m/cell |
| 버튼 해제 정지 | 각 구간 모두 통과, 정지 후 관측 변위 최대 약 0.000079 m |
| 조종기 지도·TF·파라미터 | 실시간 표시 확인 |
| 지도 저장·다시 읽기 | live SLAM 셀과 100% 일치, 미확인 셀 1,974개 보존 |
| 저장 지도에서 A* 우회 경로 | `(-3, -2)` → `(3, 2)`, 장애물 팽창 6 cells, 222 poses, 약 11.48 m |
| 최종 저장 지도에서 조종기 Goal 우회 주행 | `(3, 2)`, SUCCEEDED, 약 54.7초, 도착 오차 약 0.220 m |
| 초기 저장 지도에서 조종기 Goal 직선 주행 | `(-1, -2)`, SUCCEEDED, 오차 약 0.194 m |
| 조종기 주행 취소·재출발 | 이동 중 수동 입력 차단, CANCELED, 3초 정지 변위 약 0.000099 m, 새 Goal로 재출발 SUCCEEDED |
| 조종기 우회 도착 | `(3.8, 2.8)`, 오차 약 0.169 m, 약 52초 |
| 조종기 카메라 | Gazebo `/camera/image_raw` 640×480 실제 수신 |
| 조종기 회귀 테스트 | 86개 통과 |
| 경로 취소·목표 수신 순서 회귀 테스트 | 11개 통과 |

최종 지도는 `/home/kiki/Team2_workspace/log/slam_validation/2026-10-26/complete/map.yaml`입니다. 같은 폴더의 `map.json`에 시작점 추가 스캔을 포함한 결과, 이전 실행 증거의 경로·SHA, 전체 셀 일치, 미확인 영역 보존, 저장 지도 A* 경로 생성 결과를 기록했습니다. 현재 기본 자동 검증 경로에는 추가 스캔 구간이 모두 포함돼 있습니다.

```bash
./scripts/run_sim.sh map:=/home/kiki/Team2_workspace/log/slam_validation/2026-10-26/complete/map.yaml
# 다른 터미널
./scripts/check_sim.sh --via-controller --goal-x 3 --goal-y 2 --timeout 240
```

조종기 취소·재출발·카메라 검증은 상위 폴더의 `controller_runtime.json`, `controller_runtime.png`에 있습니다. 초기 시도에서 목표 주변과 시작점의 미확인 영역 때문에 실패한 기록도 보존했습니다. 최종 SLAM 로그는 `log/verification_slam_complete.log`, 최종 우회 주행 로그는 `log/verification_slam_complete_detour.log`, 초기 직선 주행 로그는 `log/verification_slam_straight.log`입니다. 최종 우회 결과의 JSON과 조종기 화면은 `complete/navigation.json`, `complete/controller_detour.png`에 있습니다.
