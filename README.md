# SMART-VISION-AMR — TurtleBot3 Waffle Pi A* 주행

Ubuntu 22.04 / ROS 2 Humble용입니다. 실기는 Raspberry Pi의 TurtleBot3 bringup과
Linux PC의 Nav2를 사용합니다. 첨부받은 `classroom_map.yaml` 및 `.pgm`을 포함합니다.
Gazebo Classic 11에서 동일한 A* → Nav2 FollowPath 코드를 검증하는 환경도 제공합니다.
실제 카메라의 ArUco 표식을 확인하며 사용자가 지정한 순서로 주행하는 웹 관제를 제공합니다.

## 웹 재고 요청과 배송 관제

운용자 0번은 `/`에서 부족 요청을 확인하고 적재 완료 후 배송을 출발시킵니다.
재고 관리자 1~99번은 별도 `/inventory`에서 위치·품목·수량을 요청합니다.
운행 중 요청은 접수 순서대로 추가하고, 모두 처리하면 고정 Homepoint로 복귀해
다음 적재를 기다립니다. 자세한 절차는 [재고 배송 안내](docs/inventory_delivery.md)를 참고하세요.

### 수동 ArUco 실습

PC와 휴대폰 브라우저에서 지도·카메라를 보며 마커 방문 순서를 편집하고
주행을 시작·일시정지하고, 홈으로 복귀할 수 있습니다. 임무 취소도 남은
경유지를 취소한 뒤 저장된 Homepoint로 복귀합니다. 예제 `2 → 3` 외에 등록한 마커를
원하는 순서로 추가·삭제·이동하거나 반복해서 방문할 수 있습니다.

```bash
# 터미널 1: ArUco 미로 + 저장 지도 + A* / Nav2
./scripts/run_web_mission_sim.sh

# 터미널 2: 웹 서버 (PC / 휴대폰 공용)
./scripts/run_web_mission.sh
```

PC 주소는 `http://localhost:8790`, 휴대폰 주소는 같은 Wi-Fi에서
`http://PC의_IP:8790`입니다. 강의실 실차 웹은 별도 접속 코드 대신
ROS Domain ID `16`으로 접속합니다. 새 노드가 추가되었으므로 최초 한 번 빌드가 필요합니다.
설치·빌드·순서 저장·실차 연결은 [웹 임무 안내](docs/web_mission.md)를 참고하세요.

경로가 막히면 목표를 유지하고 정지 상태에서 기본 2초 간격으로 재계획합니다.
일시정지와 비상정지는 자동 재계획으로 해제하지 않습니다.

## Gazebo에서 먼저 검증하기

현재 PC의 워크스페이스는 `/home/kiki/Team2_workspace`입니다.

```bash
cd /home/kiki/Team2_workspace/src/SMART-VISION-AMR
./scripts/setup_sim.sh  # 최초 준비 / 코드 수정 후 재빌드
./scripts/run_sim.sh   # Gazebo + RViz + 위치 추정 + 기존 주행 노드
```

다른 터미널에서 `./scripts/check_sim.sh`를 실행하면 경로 생성, 속도 명령, 실제 이동, 목표 도착을 자동 확인합니다.
새 SLAM 지도부터 만들려면 `./scripts/run_sim.sh slam:=true`를 실행하고,
다른 터미널에서 `./scripts/run_sim_controller.sh`로 기존 `~/turtlebot3/tools/` 조종기를 연결합니다.
SLAM 터미널에서 `Ctrl+C`를 누르면 `Team2_workspace/maps/slam_날짜_시간_실행번호/map.yaml`과
`map.pgm`을 자동 저장한 뒤 종료합니다. 저장 실패 시 SLAM을 유지하며 `Ctrl+C`로 재시도합니다.
`./scripts/check_slam.sh`는 조종기 주행·정지·지도 작성·저장을 자동 검증합니다.
실행 스크립트가 시뮬레이션용 ROS domain 76과 localhost 통신을 설정합니다.
[상세 실행·검증·팀원 연동 방법](docs/GAZEBO.md)을 참고하세요.

오도메트리 정확도는 기존 Gazebo 방식, 기존 `/odom`에 EKF를 추가한 진단 방식,
휠 조인트와 IMU 각속도를 중복 없이 융합한 EKF 방식으로 비교할 수 있습니다.
[세 모드의 구조](docs/ekf_modes.md)와 [반복 시험 방법](docs/ekf_benchmark.md)을 참고하세요.
[2026-10-28 비교 결과](docs/ekf_results_2026-10-28.md)에 따라 기본 실행 모드는
`baseline`이며 실차 설정은 별도로 유지합니다.

ArUco ID 0~3이 배치된 미로에서 새로 매핑하려면
`./scripts/run_sim_aruco.sh`와 다른 터미널의 `./scripts/run_sim_controller.sh`를
실행합니다. 미로 실행은 기본적으로 SLAM 모드이며 Ctrl+C로 지도를 자동 저장합니다.
[마커 규격·위치·생성 방법](docs/ARUCO.md)을 참고하세요.

## 실행 명령 한눈에 보기

최초 설치와 빌드를 마친 후, **Pi 터미널**과 **Linux PC 터미널**에서 각각 실행합니다.

```bash
# Raspberry Pi: 로봇 센서와 모터
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash  # 해당 워크스페이스에 설치한 경우
export TURTLEBOT3_MODEL=waffle_pi LDS_MODEL=LDS-03 ROS_DOMAIN_ID=16
ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=false
```

```bash
# Linux PC: 지도, Nav2, A*, 가벼운 RViz
source /opt/ros/humble/setup.bash
source ~/SMART-VISION-AMR/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi ROS_DOMAIN_ID=16
ros2 launch smart_vision_amr hardware.launch.py
```

Pi에서 이미 bringup이 실행 중이면 PC 명령만 실행합니다. 설치 위치가 다르면
`source` 경로를 바꾸고, `ROS_DOMAIN_ID`는 두 장비에서 기존에 맞춰 둔 값을
그대로 사용하세요. RViz 화면에서 **2D Pose Estimate**로 시작 위치를 잡고
**2D Goal Pose**로 도착지를 지정합니다.

## 구성

`/smart_vision_amr/goal` → A* (`/smart_vision_amr/path`) → Nav2 `/follow_path`
→ Nav2 controller `/cmd_vel` → TurtleBot3. `/scan` 관측은 A*의 동적 장애물
회피에 사용하고, Nav2 local costmap이 근거리 충돌 검사를 담당합니다.
장애물 기억은 기본적으로 꺼져 있으며 `memory_enabled:=true`로 켤 수 있습니다.

## 1. 로봇 Raspberry Pi

이미 하드웨어 bringup과 PC 간 ROS 토픽 공유가 된 환경에서 실행합니다.
Pi의 `turtlebot3_bringup`과 LDS-03용 `coin_d4_driver`가 설치돼 있어야 합니다.
없다면 [ROBOTIS의 Ubuntu 22.04/Humble SBC 설치 안내](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)에
따라 Pi에 TurtleBot3 `humble` 브랜치와 `coin_d4_driver`를 설치합니다.
PC와 Pi는 같은 네트워크 및 같은 `ROS_DOMAIN_ID`를 사용하세요.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash  # 이 경로에 ROBOTIS 패키지를 빌드한 경우
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-03
export ROS_DOMAIN_ID=16  # PC에도 같은 값을 설정; 이미 다른 값을 쓰고 있다면 그 값 유지
ros2 pkg prefix turtlebot3_bringup
ros2 pkg prefix coin_d4_driver
ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=false
```

Pi에서 `/scan`, `/odom`, `/tf`, `/cmd_vel`이 존재해야 합니다. LDS-03 드라이버는
설치된 ROBOTIS TurtleBot3 bringup의 `LDS_MODEL=LDS-03` 지원을 사용합니다.
기본 장치 경로는 OpenCR `/dev/ttyACM0`, LDS-03 `/dev/ttyUSB0`입니다.
장치 연결이 다르면 포트와 권한을 확인하세요. ROS 패키지가 이미 시스템에
설치되어 `~/turtlebot3_ws`가 없다면 위 `source` 줄은 생략합니다.

## 2. Linux PC 설치 및 실행

PC와 Pi에서 같은 `ROS_DOMAIN_ID`를 사용합니다. PC에 ROS 2 Humble, Nav2,
TurtleBot3 navigation 패키지가 설치되어 있어야 합니다.

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3-navigation2 ros-humble-rviz2 \
  python3-colcon-common-extensions python3-rosdep
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=16  # Pi와 동일한 값
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch smart_vision_amr hardware.launch.py
```

이 launch는 가벼운 RViz 화면도 함께 엽니다. 지도, 로봇 위치 축, A* 경로만
표시하며 화면 갱신을 10 FPS로 제한합니다. LaserScan 점군, 로봇 3D 모델,
Nav2 costmap 화면은 표시하지 않습니다. GUI가 필요 없는 진단 실행은
`ros2 launch smart_vision_amr hardware.launch.py use_rviz:=false`입니다.

기본 Nav2 설정은 설치된 `turtlebot3_navigation2/param/humble/waffle_pi.yaml`을
사용합니다. 지도는 저장소에 포함된 교실 지도입니다. 실제 지도와 로봇 위치가
맞아야 합니다. 다른 지도를 사용하려면 `map:=/절대경로/map.yaml`을 지정합니다.

## 3. 주행 전 확인

로봇을 넓고 비어 있는 곳에 놓고 다음을 확인합니다.

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 action list | grep follow_path
ros2 lifecycle get /controller_server
```

자동으로 열린 RViz에서 실제 위치에 맞춰 **2D Pose Estimate**를 지정한 뒤
`ros2 run tf2_ros tf2_echo map base_footprint`가 연속적으로 출력되는지
확인합니다. 지도와 위치 추정이 맞지 않으면 목표를 보내지 마세요.

## 4. 첫 목표

RViz 상단의 **2D Goal Pose**를 선택해 지도 안의 가깝고 비어 있는 지점을
클릭하고 방향으로 드래그합니다. 이 버튼은 `/smart_vision_amr/goal`로
직접 발행하도록 설정되어 있어 A* 경로가 사용됩니다. 지도 위의 시작 위치는
**2D Pose Estimate**로 지정합니다. PC의 launch를 실행해도 Pi에서 로봇
bringup은 별도로 실행되어 있어야 합니다.

클릭 대신 터미널로 목표를 보낼 수도 있습니다. 아래 좌표는 실제 지도에서
확인한 자유 공간 좌표로 바꿔야 합니다.

```bash
ros2 topic pub --once /smart_vision_amr/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
ros2 topic echo /smart_vision_amr/navigation_status
ros2 topic echo /cmd_vel
```

중단은 `ros2 service call /smart_vision_amr/cancel_navigation std_srvs/srv/Trigger '{}'`입니다.
첫 주행은 바퀴를 띄워 방향을 확인하고, 이후 넓은 공간에서 저속으로 수행하세요.
프로젝트는 실제 장비에서 아직 검증되지 않았습니다.

## 문제를 좁히는 순서

1. `/map`이 없으면 map server와 지도 경로를 확인합니다.
2. `map -> base_footprint` TF가 없으면 AMCL 초기 위치 및 Pi/PC 시계 동기화를 확인합니다.
3. `/smart_vision_amr/path`가 비면 A* 로그의 시작/목표 셀과 지도 장애물 판정을 확인합니다.
4. FollowPath가 실패하면 controller 로그, local costmap, `/scan`을 확인합니다.
5. `/cmd_vel`이 나오는데 바퀴가 돌지 않으면 Pi의 OpenCR/모터 쪽을 확인합니다.

`config/nodes.yaml`의 `obstacle_inflation_cells`는 0.05 m 해상도 기준
6셀(0.30 m)입니다. 실제 통로 폭과 로봇 외형을 확인한 뒤 조정하세요.
Nav2의 controller/local costmap은 설치된 TurtleBot3 Humble 설정을 따릅니다.

## 출처

A* 격자 탐색, 장애물 관측/기억 및 FollowPath 연결은
[`resilient-amr-astar`](https://github.com/waterhyunsu/resilient-amr-astar)의
구현을 참고해 실기 전용 패키지로 구성했습니다.
