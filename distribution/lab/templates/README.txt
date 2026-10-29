TurtleBot3 ArUco 미로 실습 배포본
==============================
Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 / Waffle Pi용입니다.
다른 사용자 이름과 폴더에서도 실행할 수 있습니다. ROS Humble과 apt 저장소는
먼저 준비되어 있어야 합니다. 시스템 패키지 자체는 ZIP에 포함하지 않습니다.

1. 처음 설치
------------
압축을 쓰기 가능한 곳에 풀고 해당 폴더에서 실행하세요.

  cd /압축을/푼/경로/turtlebot3-aruco-lab
  ./setup.sh --install-deps

이 폴더에 포함된 ROS 소스만 빌드합니다. 추가 git clone은 하지 않습니다.
설치된 의존성만 검사하려면 ./setup.sh --check-only 를 사용하세요.
폴더를 옮겼으면 ./setup.sh 로 다시 빌드하세요.
조종기 소스가 포함되어 있어 DEB 설치는 필수가 아닙니다.
--deb 옵션으로 생성한 ZIP에 DEB가 있으면 ./install_controller.sh 로
앱 메뉴 설치도 가능합니다. 설치 시 인터넷과 sudo 권한이 필요합니다.

2. 저장 지도에서 주행
--------------------
터미널 1:

  ./run_navigation.sh

터미널 2:

  ./run_controller.sh

조종기 지도에서 목표를 지정합니다. 목표는 /smart_vision_amr/goal,
A* 경로는 /smart_vision_amr/path, 취소는 /smart_vision_amr/cancel_navigation
서비스를 사용합니다. 카메라는 /camera/image_raw입니다.
지도 주행 중에는 motion_mux가 속도 명령을 관리하므로 PyQt의 수동 조종
버튼은 잠깁니다. 수동 개입은 아래 웹 조이스틱을 사용하세요. PyQt의 지도,
목표 지정, 파라미터 기능은 계속 사용할 수 있습니다.
종료하려면 각 터미널에서 Ctrl+C를 누르세요.

기본 지도는 maps/maze_aruco_20261026/map.yaml입니다. 2026-10-26
Gazebo의 실제 LaserScan을 slam_toolbox로 매핑한 지도이며,
0.05 m/cell, origin [-5.03, -4.14, 0]입니다.
Gazebo 시작 위치는 (-3, -2, 0), 저장 지도에서 같은 위치는
(-3.0007995856, -2.0010944836, 0.0008724679)입니다.
측정 정보는 같은 폴더의 mapping_info.json에 있습니다.

3. 웹 조종기와 ArUco 임무
------------------------
기존 실행 터미널을 먼저 Ctrl+C로 종료합니다.

터미널 1:

  ./run_navigation.sh use_motion_mux:=true

터미널 2:

  ./run_web.sh

PC 브라우저에서 http://127.0.0.1:8790/ 을 엽니다.
같은 Wi-Fi의 휴대폰은 http://실행PC의IP:8790/ 에 접속합니다.
터미널에 출력된 접속 코드를 사용합니다. 재고 요청은 /inventory 페이지입니다.
ArUco 검출과 임무 서버 소스도 포함되어 있습니다.

4. 새 지도 만들기
-----------------
기존 navigation/world 실행을 종료한 뒤 시작합니다.

터미널 1:

  ./run_slam.sh

터미널 2:

  ./run_controller.sh

수동으로 이동하며 매핑합니다. 터미널 1에서 Ctrl+C를 누르면
maps/slam_날짜_시각_PID/map.yaml 및 map.pgm을 저장하고 종료합니다.
저장 실패 시 SLAM을 계속 유지하며 Ctrl+C로 다시 시도할 수 있습니다.
새 지도로 주행하려면 실제 저장된 경로를 지정하세요.

  ./run_navigation.sh map:="$PWD/maps/slam_날짜_시각_PID/map.yaml"

새 SLAM의 초기 좌표가 달라졌다면 initial_x:=값 initial_y:=값
initial_yaw:=라디안도 함께 지정합니다. 저장 당시 최종 위치를
새 시뮬레이션의 시작 위치로 그대로 넣으면 안 됩니다.

5. 기타 실행
------------
월드와 로봇만: ./run_world.sh
Gazebo 화면 없이: ./run_navigation.sh gui:=false
RViz 추가: ./run_navigation.sh use_rviz:=true
독립 EKF 시험: ./run_navigation.sh localization_mode:=ekf_independent

기본 도메인은 76, ROS_LOCALHOST_ONLY=1, Gazebo 포트는 11376입니다.
바꾸려면 모든 실행 터미널에 같은 값을 지정하세요.

  export TURTLEBOT3_LAB_DOMAIN_ID=77

6. 마커와 파라미터
------------------
사전은 DICT_4X4_50, ID는 0~3, 마커 검은 영역의 한 변은 0.20 m입니다.
흰 여백을 포함한 보드는 0.28 m입니다. 모델·배치 정보는 assets/에 있습니다.
지도에서 마커를 정면으로 보는 위치 (x, y, yaw):

  0: (-3.0008, -2.0011, 0.00087)
  1: (-1.0052,  3.0006, 0.00087)
  2: ( 1.3999, -2.7973, 0.00087)
  3: ( 3.8996, -2.4951, 0.00087)

조종기의 파라미터 저장 대상은 다음 로컬 파일입니다.
  ros2_ws/src/smart_vision_amr/config/curve_tuning_simulation.yaml
  ros2_ws/src/smart_vision_amr/config/curve_tuning_simulation_nodes.yaml
개인 설정은 state/에 생성됩니다. 다른 PC의 설정·인증정보는 포함하지 않습니다.

7. 무결성과 출처
----------------
  sha256sum -c SHA256SUMS

source_manifest.json에 소스 커밋과 지도·월드 SHA-256이 있습니다.
ROBOTIS 고정 소스:
  turtlebot3:             90a68bd2e3c61c12966779da89d8eeaec82730e9
  turtlebot3_simulations: a35a56c8b04877dc89772b598084d8ce648a9023
ROBOTIS Apache 2.0 라이선스는 licenses/에 있습니다.
팀 ROS 패키지는 package.xml/setup.py에서 MIT를 선언합니다.
팀 원본 저장소: https://github.com/kiki8105/SMART-VISION-AMR
배포본 재생성은 원본의 distribution/lab/build_lab_bundle.py --zip 을 사용합니다.
