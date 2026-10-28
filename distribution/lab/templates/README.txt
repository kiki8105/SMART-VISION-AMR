TurtleBot3 ArUco 미로 실습 배포본 (Ubuntu 22.04 / ROS 2 Humble)
================================================================

이 폴더는 다른 사용자 이름과 다른 경로에 압축을 풀어 사용할 수 있습니다.
전체 폴더를 쓰기 가능한 곳에 푸세요. 경로를 옮긴 뒤에는 ./setup.sh로 다시 빌드하세요.
ROS 2 Humble은 /opt/ros/humble에 설치되어 있어야 합니다. Gazebo Classic 11을 사용합니다.
TurtleBot3 모델은 waffle_pi입니다. 실물 로봇을 연결하지 않은 실습 환경을 기준으로 합니다.

1. 설치 (최초 한 번)
-------------------
압축을 푼 turtlebot3-aruco-lab 폴더에서 터미널을 여세요.

  cd /압축을/푼/경로/turtlebot3-aruco-lab
  ./install_controller.sh --check
  ./install_controller.sh
  ./setup.sh

setup.sh는 이 폴더의 ros2_ws만 빌드합니다. git clone은 실행하지 않습니다.
필요한 ROS/Gazebo 패키지가 없으면 목록과 설치 명령을 출력합니다.
Ubuntu 22.04와 ROS Humble apt 저장소가 준비된 PC에서 사용자가 설치를 요청하려면:

  ./setup.sh --install-deps

의존성 확인만 수행하려면 ./setup.sh --check-only 를 실행하세요.
install_controller.sh는 Ubuntu 22.04 amd64/arm64를 확인하고 공식 ROS 저장소 설정 패키지의
SHA-256을 검증한 뒤 앱 실행 의존성을 apt로 설치합니다. --check는 설치하지 않고 확인만 합니다.
실제 설치에는 인터넷과 sudo 권한이 필요합니다.
인터넷 없이 빌드하려면 아래의 시스템 의존성이 이미 설치되어 있어야 합니다.
ROS Humble/Gazebo/Qt 등의 시스템 패키지 자체는 이 ZIP에 포함되지 않습니다.

2. 저장된 실제 SLAM 지도로 주행
-----------------------------
터미널 A:

  cd /압축을/푼/경로/turtlebot3-aruco-lab
  ./run_navigation.sh

터미널 B (같은 폴더):

  ./run_controller.sh

GUI의 지도 Goal 기능은 /smart_vision_amr/goal로 목표를 보내며,
/smart_vision_amr/path에 표시되는 팀 A* 경로를 /follow_path로 실행합니다.
Stop/취소는 /smart_vision_amr/cancel_navigation 서비스를 사용합니다.
FollowWaypoints는 이 실습의 A* 실행 경로가 아닙니다. 별도의 FollowWaypoints 서버는 시작하지 않습니다.
기본 카메라 토픽은 /camera/image_raw, 지도 좌표계는 map입니다.

기본 지도를 다시 생성하거나 덮어쓰지 않고 maps/maze_aruco_20261026/map.yaml을 읽습니다.
이 지도는 2026-10-26 Gazebo LaserScan을 수동 주행하며 slam_toolbox로 얻은 측정 지도입니다.
0.05 m/cell, origin [-5.03, -4.14, 0]이고 가상 벽 도형에서 합성한 점유 지도가 아닙니다.
캡처 당시 map->odom과 초기 map 좌표는 mapping_info.json에 보존되어 있습니다.
로봇 Gazebo 시작 좌표 (-3, -2, 0)와 지도 초기화 좌표는 조금 다릅니다.
run_navigation.sh는 캡처한 초기 map pose
(-3.000799585626671, -2.0010944835652884, 0.0008724678696151052)를 AMCL에 전달합니다.
SLAM pose graph는 배포본에 포함하지 않았습니다. 저장 지도 주행에는 YAML/PGM이면 충분합니다.

3. 처음부터 SLAM 실습과 Ctrl+C 자동 저장
----------------------------------------
먼저 실행 중인 run_navigation.sh 또는 run_world.sh를 Ctrl+C로 종료하세요.
이 스크립트들은 각각 Gazebo와 로봇을 시작하므로 동시에 실행하지 마세요.

터미널 A:

  ./run_slam.sh

터미널 B:

  ./run_controller.sh

GUI에서 수동 조종하며 미로를 조사하세요. 이 모드에는 A* 주행 노드를 시작하지 않습니다.
터미널 A에서 Ctrl+C를 누르면 SLAM과 Gazebo 시계를 살려 둔 채 /map을 먼저 저장하고 종료합니다.
결과는 maps/slam_날짜_시각_PID/map.yaml 및 map.pgm입니다.
저장이 실패하면 SLAM을 유지합니다. Ctrl+C를 다시 누르면 새 이름으로 저장을 재시도합니다.
프로세스를 강제 종료하거나 창을 강제로 닫으면 자동 저장을 보장할 수 없습니다.
원하는 새 저장 위치를 정하려면 (파일 확장자 없이, 기존 파일이 없는 절대 경로):

  ./run_slam.sh --map-prefix "$PWD/maps/my_survey/map"

새 지도는 기본 측정 지도를 덮어쓰지 않습니다. 직접 새 지도를 선택하려면:

  ./run_navigation.sh map:="$PWD/maps/my_survey/map.yaml"

새 SLAM의 초기 map 좌표를 변경했으면 initial_x:=값 initial_y:=값 initial_yaw:=라디안 을 함께 전달하세요.
지도 저장 시 기록하는 map_mapping_info.json의 robot_map_pose_at_save는 저장 당시의 최종 위치이며,
새 Gazebo 시작 위치의 AMCL 초기 좌표로 그대로 쓰면 안 됩니다.
run_slam.sh는 매번 빈 SLAM 그래프로 시작하며 기존 PGM/YAML을 SLAM 초기 그래프로 읽지 않습니다.

4. 월드만 열기, 화면 없이 실행
-----------------------------
SLAM/지도 주행을 시작하지 않고 카메라·라이다와 미로만 확인하려면 터미널 A:

  ./run_world.sh

터미널 B:

  ./run_controller.sh

Gazebo 창 없이 실행하려면 각 실행 스크립트에 gui:=false 를 붙이세요.
RViz를 추가하려면 use_rviz:=true 를 붙이세요.
각 스크립트의 --help에서 실행 인자를 확인할 수 있습니다.

5. ArUco 4개와 지도 관찰 좌표
----------------------------
사전: OpenCV DICT_4X4_50, ID: 0, 1, 2, 3
검출/pose 추정 markerLength: 0.20 m (검은 정사각형의 한 변, 흰 여백 제외)
흰 여백은 각 변 0.04 m, 전체 보드는 0.28 m입니다. 네 보드의 인쇄면은 -X 방향입니다.
마커는 Gazebo의 실제 텍스처/메시 모델입니다. ID 역할은 start/waypoint_1/waypoint_2/goal 라벨입니다.
이 배포본에는 실시간 ArUco 검출 노드나 ID 기반 자동 목표 전환 노드를 포함하지 않습니다.
GUI에서 카메라를 보고 검출기를 추가하는 실습 또는 수동 목표 주행에 사용하세요.

모델 중심 (Gazebo world x, y, z, yaw; 미터/라디안):
  ID 0 start       : (-2.0621, -2.0, 0.22, 0)
  ID 1 waypoint_1  : ( 0.2379,  3.0, 0.22, 0)
  ID 2 waypoint_2  : ( 2.5379, -2.8, 0.22, 0)
  ID 3 goal        : ( 4.9900, -2.5, 0.22, 0)

카메라로 정면 관찰하는 로봇 위치 (Gazebo world x, y, yaw):
  ID 0: (-3.0, -2.0, 0)
  ID 1: (-1.0,  3.0, 0)
  ID 2: ( 1.4, -2.8, 0)
  ID 3: ( 3.9, -2.5, 0)

배포된 측정 지도에서 같은 관찰 지점 (map x, y, yaw):
  ID 0: (-3.0007995856, -2.0010944836, 0.0008724679)
  ID 1: (-1.0051626856,  3.0006485490, 0.0008724679)
  ID 2: ( 1.3998967139, -2.7972553209, 0.0008724679)
  ID 3: ( 3.8996340221, -2.4950742657, 0.0008724679)

직선으로 마커를 관통하는 목표를 주지 말고 위 관찰 위치를 목표로 선택하세요.
미로 경유 힌트: (-3,-2) -> (-3,3) -> (-1,3) -> (-1,-3)
               -> (1.4,-3) -> (1.6,3) -> (3.7,3) -> (3.9,-2.5)
모든 위치와 모델 설정 원본은 assets/aruco_maze_layout.json 및 각 marker.json에 있습니다.

6. E 곡선 주행 파라미터 저장
---------------------------
run_navigation.sh와 run_controller.sh는 다음 동일한 쓰기 가능한 소스 YAML을 사용합니다.

  ros2_ws/src/smart_vision_amr/config/curve_tuning_simulation.yaml
  ros2_ws/src/smart_vision_amr/config/curve_tuning_simulation_nodes.yaml

주요 기본값: max_vel_x/max_speed_xy=0.12 m/s, prune_distance=0.25 m,
forward_prune_distance=0.75 m, obstacle_inflation_cells=6 (0.30 m).
GUI의 기본값 저장이 이 파일들을 갱신하므로 다음 실행에도 반영됩니다.
DWB pruning 값 등 configure 시 읽는 항목은 변경 후 주행 스크립트를 종료/재실행하세요.
controller의 개인 설정은 이 폴더의 state/controller_simulation.ini에 새로 생성됩니다.
다른 PC의 개인 설정, ROS 로그, 인증 정보는 배포하지 않습니다.

7. 도메인과 모델 가져오기
-----------------------
기본값은 ROS_DOMAIN_ID=76, ROS_LOCALHOST_ONLY=1, Gazebo master=127.0.0.1:11376입니다.
세 실행 터미널에 같은 환경값을 적용하세요. 도메인을 바꾸려면:

  export TURTLEBOT3_LAB_DOMAIN_ID=77
  ./run_navigation.sh

다른 터미널에서도 export TURTLEBOT3_LAB_DOMAIN_ID=77 후 ./run_controller.sh를 실행하세요.
Gazebo 기본 포트는 11300 + 도메인 ID입니다. 필요하면 TURTLEBOT3_LAB_GAZEBO_PORT로 지정합니다.
다른 PC와 ROS 통신을 허용하려면 각 터미널에서 export ROS_LOCALHOST_ONLY=0 을 사용하세요.
원격 Gazebo master가 필요한 경우 TURTLEBOT3_LAB_GAZEBO_MASTER_URI로 URI를 지정할 수 있습니다.

실행 스크립트는 assets/models와 포함된 TurtleBot3 models를 GAZEBO_MODEL_PATH에 직접 추가합니다.
~/.gazebo/models 심볼릭 링크를 만들 필요가 없습니다.
다른 Gazebo 프로젝트로 모델만 가져오려면 다음처럼 경로를 추가하고 worlds/maze.world를 여세요.

  export GAZEBO_MODEL_PATH="$PWD/assets/models:$PWD/ros2_ws/src/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

8. 포함 파일, 출처, 재배포 빌드
------------------------------
worlds/maze.world                    : 미로 월드, 네 마커 포함 (생성 원본과 동일한 바이트)
assets/models/                       : ID 0~3의 SDF/OBJ/material/PNG/marker.json
maps/maze_aruco_20261026/              : map.yaml, map.pgm, map_preview.png, mapping_info.json
ros2_ws/src/smart_vision_amr/          : 팀 ROS 소스 전체 및 실습 전용 aruco_lab.launch.py
ros2_ws/src/turtlebot3_description/    : ROBOTIS 소스 2.3.7
ros2_ws/src/turtlebot3_navigation2/    : ROBOTIS 소스 2.3.7
ros2_ws/src/turtlebot3_gazebo/         : ROBOTIS 소스 2.3.8
deb/                                 : 설치용 turtlebot3-controller DEB
install_controller.sh / deps/         : 수신 PC용 앱 설치 스크립트와 공식 ROS 저장소 설정 DEB
licenses/                            : 포함된 ROBOTIS Apache 2.0 라이선스
source_manifest.json / SHA256SUMS     : 고정 커밋, 배포 파일 SHA-256

고정 소스 커밋:
  ROBOTIS-GIT/turtlebot3             90a68bd2e3c61c12966779da89d8eeaec82730e9
  ROBOTIS-GIT/turtlebot3_simulations a35a56c8b04877dc89772b598084d8ce648a9023
smart_vision_amr의 package.xml/setup.py는 MIT를 선언합니다.
원본 저장소에는 별도 MIT LICENSE 본문이 없어 이를 새로 작성하지 않았습니다.
현재 실습 월드/마커/지도 소스는 source_manifest.json에서 SHA-256으로 추적합니다.

개발용 원본의 distribution/lab/build_lab_bundle.py가 로컬 파일에서 이 폴더를 재생성합니다.
네트워크 clone을 수행하지 않으며 .git/build/install/log/__pycache__/개인 설정을 포함하지 않습니다.
현재 추출된 폴더의 무결성 확인:

  sha256sum -c SHA256SUMS

빌드 후 생성되는 ros2_ws/build, install, log와 state 폴더는 실행 PC 전용입니다.
이 폴더를 재배포할 때는 개발용 빌드 도구로 새 배포본을 생성하세요.
