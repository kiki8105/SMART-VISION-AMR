# 팀원 실행·검증 인계 (2026-10-29)

이 저장소의 ROS 2 Humble 패키지는 `ros2_ws/src/smart_vision_amr`에 있다. 저장소를
`<작업공간>/src/SMART-VISION-AMR`에 두고 `<작업공간>`에서 빌드한다. Gazebo는
기본 ROS 도메인 76과 로컬 통신을, 강의실 실차는 도메인 16과 네트워크 통신을
사용한다. 강의실 실차의 기존 이름 실행 스크립트는 독립 바퀴·IMU EKF를 기본으로 켠다.
웹 서버는 PC에서 실행하며, 브라우저는 PC의 HTTP 포트 8790에 접속한다.
브라우저에 ROS를 설치할 필요는 없다.

## 팀원 PC에서 처음 준비

Ubuntu 22.04, ROS 2 Humble, Gazebo Classic을 준비한 다음:

```bash
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
./scripts/setup_sim.sh
```

`setup_sim.sh`는 부족한 시스템 패키지를 알려주고 공식 TurtleBot3 Humble
시뮬레이션 소스를 고정된 리비전으로 가져와 Team2 작업공간을 빌드한다. 카메라·웹
코드를 바꾼 뒤에는 다음을 실행한다.

```bash
cd ~/Team2_workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_vision_amr --symlink-install
source install/local_setup.bash
```

## Gazebo에서 먼저 확인

각각 다른 PC 터미널에서 실행한다. 이 기본 시뮬레이션은 저장소의
`simulation_room.world`와 `simulation_room.yaml`을 사용한다.

```bash
# 터미널 1: Gazebo, 저장 지도, AMCL, A*, Nav2
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh
```

```bash
# 터미널 2: 저장소에 포함된 PyQt 지도·Goal·파라미터 화면
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim_controller.sh
```

PyQt 조종기와 지도·파라미터 패널은 저장소의 `tools/`에 포함되어 있다.
실행 스크립트가 해당 파일을 자동으로 찾으므로 별도 조종기 소스를 복사하지 않는다.
명시적으로 다른 조종기를 쓰려는 경우에만
`SMART_VISION_CONTROLLER_SCRIPT=/절대경로/cmd_vel_controller.py`로 지정한다.
저장 지도 주행에서는 PyQt의 지도·Goal·파라미터 기능을 사용한다.
PyQt는 화면만 열었을 때 속도 발행자를 만들지 않아 기본 mux와 충돌하지 않는다.
웹 mux가 `/cmd_vel`을 담당하면 PyQt 방향 버튼은 잠기며 웹 조이스틱을 안내한다.
수동 조작은 웹 조이스틱을 사용하거나, 아래 수동 SLAM처럼 mux를 끄고
`./scripts/run_sim_controller.sh`를 추가 인자 없이 실행한다.
로봇과 지도·경로·속도 토픽의 자동 검사는 다른 터미널에서
`./scripts/check_sim.sh`로 실행한다. 새 지도를 만들 때는 터미널 1에
`./scripts/run_sim.sh slam:=true use_motion_mux:=false`를 사용한다. 그 터미널에서 `Ctrl+C`를 누르면
`~/Team2_workspace/maps/slam_날짜_시간_실행번호/map.yaml`과 `.pgm`을 저장한다.

ArUco 미로 새 SLAM은 `./scripts/run_sim_aruco.sh use_motion_mux:=false`로 시작한다.
마커 0~3의 모델·재질·이미지는 `gazebo_models/`에 포함되어 자동으로 로드된다.
저장 지도 기반 미로 웹 실습은 `./scripts/run_web_mission_sim.sh`와
`./scripts/run_web_mission.sh`를 별도 터미널에서 실행한다.
해당 지도도 `ros2_ws/src/smart_vision_amr/maps/maze_aruco_20261026.yaml` 및
`.pgm`으로 포함되어 있으므로 첫 실행 전 재매핑이나 별도 지도 복사가 필요 없다.

## 강의실 실차 도메인 16

실차 실행 전에 강의실 지도와 실제 가벽 배치가 같은지 확인한다. 강의실
스크립트가 쓰는 저장 지도는
`ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml`이다.
저장 지도 주행 시 SLAM/Cartographer는 종료한다. `/map`의 발행자는 Nav2
`/map_server` 하나여야 한다. 로봇이 홈에 실제로 있는지, 후방 카메라가
0번 마커를 보는지도 위치 초기화 전에 확인한다.

로봇(`robot@192.168.0.187`)에는 브링업·카메라 스크립트와 EKF launch 파일을 복사한다.
주소가 바뀌면 현재 로봇 주소로 바꾼다. 카메라 장치 경로와 포맷은 현장 장치에
따라 확인한다.

```bash
# PC에서 복사
cd ~/Team2_workspace/src/SMART-VISION-AMR
scp scripts/robot_bringup_camera_16.sh scripts/robot_bringup_camera_16_baseline.sh \
  scripts/ros_camera_compressed.py \
  ros2_ws/src/smart_vision_amr/launch/hardware_robot_ekf.launch.py \
  robot@192.168.0.187:~/

# 로봇 SSH 터미널에서. 기존 bringup/카메라가 있으면 먼저 해당 터미널에서 종료.
chmod +x ~/robot_bringup_camera_16.sh ~/robot_bringup_camera_16_baseline.sh
ENABLE_FRONT_CAMERA=false REAR_CAMERA_SIZE=320x240 \
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ~/robot_bringup_camera_16.sh
```

위 명령은 후방 단독 EKF 구성이다. 기본 카메라 백엔드 `rate_limited`는
압축 토픽에 최대 5 FPS, 320×240, JPEG 품질 60으로 발행하며 **원본 Image
토픽은 발행하지 않는다**. 후방은 고정 장치 경로를, 전방은 `camera0` 장치를
사용한다. 포함된 PyQt 조종기의 `run_classroom_controller_16.sh`도 압축 영상을
기본으로 구독한다. 원본 토픽이 필요한 별도 구성에서는 `CAMERA_BACKEND=v4l2`를
명시한다. 이 백엔드는 `/camera/image_raw`와 `/camera/image_raw/compressed`를
함께 발행한다. 스크립트는 로봇의 `~/turtlebot3_ws`
설치본과 `LDS_MODEL=LDS-03` 기본값을 사용한다. 두 카메라 동시 기동은 아래
제한 사항을 확인하고 전송 지연 원인을 조사할 때만 시도한다.

PC에서 다른 터미널로 독립 EKF·Nav2·A*를 시작한다. 이 명령은 Gazebo를 켜지
않는다. 위 Pi 스크립트와 PC 스크립트를 **함께** 실행해야 한다. 기존 오도메트리
구성으로 돌아가려면 두 장치의 실행 터미널을 종료하고 각각
`~/robot_bringup_camera_16_baseline.sh`와
`./scripts/run_classroom_navigation_16_baseline.sh`를 실행한다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

EKF 실행은 센서 상태를 확인할 때까지 비상정지로 시작한다. `/wheel/odom`,
`/imu/ekf`, `/scan`, `/odom`과 지도 위치가 정상인지 확인한 뒤 웹에서 **비상정지 해제**,
**임무 시작/재개**를 순서대로 누른다. 자세한 사전 검사와 복귀 절차는
[실차 EKF 실행 안내](hardware_ekf.md)에 있다.

웹을 직접 실행하려면 별도 PC 터미널에서 다음을 사용한다. 이미 사용자 서비스가
포트 8790을 사용 중이면 그 서비스를 먼저 중지한다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

강의실 웹은 압축 영상 토픽을 기본으로 구독한다. 원본 토픽을 내는 별도 카메라
구성에서만 `SMART_VISION_CLASSROOM_CAMERA_TRANSPORT=raw`로 바꾼다. 상시
웹 서비스는 `./scripts/install_classroom_web_service.sh --start`로 등록하고
`systemctl --user status smart-vision-classroom-web.service`로 상태를 확인한다.

이 PC는 `http://127.0.0.1:8790/`, 같은 Wi-Fi의 휴대폰·다른
PC는 `http://PC의_IP:8790/`에 접속한다. 재고 화면 경로는 `/inventory`다.
웹 서비스는 로봇 브링업이나 Nav2를 대신 시작하지 않는다.

AMCL 초기 위치는 물리적 홈과 방향을 확인한 뒤 웹의 **자세 초기화** 또는
`./scripts/set_classroom_home_pose_16.sh`로 넣는다. 웹 지도에 차량이 보인다는
것만으로 최신 `map→base_footprint` TF가 확정된 것은 아니다. 임무 전에는
`/scan`, `/odom`, `/map`, `map→base_footprint`와 카메라 수신을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
ros2 topic info /map -v
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info /camera/image_raw/compressed -v
ros2 topic info /camera/front/image_raw/compressed -v
```

## 검증 범위와 현재 제한

- 시뮬레이션과 웹 기능에 대한 패키지 테스트는 이전 실행에서 544개 통과했다.
  팀원은 `source /opt/ros/humble/setup.bash`, 작업공간의
  `install/local_setup.bash`를 적용한 뒤
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q ros2_ws/src/smart_vision_amr/test`
  로 재검증할 수 있다. 카메라 보조 노드는
  `python3 -m unittest discover -s scripts -p test_ros_camera_compressed.py`
  로 별도 검사한다.
- 2026-10-28 후방 카메라만 기존 `v4l2` 압축 전송(320×240)으로 켰을 때는
  PC 웹에서 12회 연속 최신 프레임이 관측됐다. 짧은 ICMP 검사 평균은 약
  103ms였다. 이는 후방 영상 수신의 제한된 확인 결과다.
- 전·후방 두 카메라를 동시에 켠 시험은 `v4l2` 방식과 새 5 FPS 제한 방식
  모두에서 Wi-Fi 지연이 수 초까지 늘고 `/scan`, `/odom`, 영상 수신이
  오래되는 현상이 있었다. 두 카메라 동시 운용, 전방 ArUco 검증,
  전체 배송 주행은 아직 **실차에서 검증되지 않았다**. 원인과 전송량을
  다시 측정하기 전에는 자동 배송을 시작하지 않는다.
- 위 원격 시험이 끝난 뒤 시험을 위해 시작한 로봇 프로세스는 종료했다.
  현재 실행 여부는 각 장치에서 다시 확인해야 한다. 셸 테스트 통과와
  실제 주행 안정성은 서로 다른 검증이다.

실차 세부 절차는 [강의실 시작](classroom_navigation_startup.md),
[재고 배송](inventory_delivery.md), [웹 서비스](classroom_web_service.md)를 참고한다.
