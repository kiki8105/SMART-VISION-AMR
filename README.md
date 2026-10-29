# SMART-VISION-AMR

TurtleBot3 Waffle Pi가 저장 지도에서 A* 경로를 만들고 Nav2로 이동하는 프로젝트입니다.
Gazebo에서 먼저 시험하고, 실제 로봇과 웹 배송 관제로 이어서 사용할 수 있습니다.

경로를 따라가지 못하면 목표를 유지하고 실패한 접근 방향을 잠시 피해서 재계획합니다.
장애물 때문에 기본 3회 실패하면 웹에 대기 안내를 표시하고, 제거가 2초간 확인되면 재개합니다.
후진·재진입 조건과 조절값은 [실패 접근 재계획 안내](docs/FAILURE_REPLANNING.md)에 있습니다.

## 처음 설치하기 (PC)

아래 명령은 **Ubuntu 22.04와 ROS 2 Humble이 설치된 PC**에서 실행합니다.
예시 작업공간은 `~/Team2_workspace`입니다. 저장소를 `src` 안에 두어야
제공된 스크립트가 작업공간을 올바르게 찾습니다.

**1. 필요한 프로그램 설치**

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions gazebo \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup \
  ros-humble-xacro ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-compressed-image-transport \
  ros-humble-teleop-twist-keyboard \
  python3-opencv python3-pyqt5 python3-numpy python3-yaml python3-pil
```

**2. GitHub에서 코드 받기**

```bash
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
```

**3. TurtleBot3 시뮬레이션 파일 준비 및 빌드**

```bash
./scripts/setup_sim.sh
```

이 스크립트가 필요한 ROBOTIS 저장소를 가져오고 `~/Team2_workspace`에서
`colcon build`를 실행합니다. 처음에는 다운로드와 빌드에 시간이 걸립니다.

## Gazebo에서 주행 시험하기

**터미널 1 — 시뮬레이터 실행**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh
```

Gazebo와 RViz가 열리고, 저장소에 포함된 예제 지도·A*·Nav2가 실행됩니다.

**터미널 2 — 자동 주행 시험**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/check_sim.sh
```

`check_sim.sh`는 **실제로 시뮬레이션 로봇을 움직여** 경로 생성부터 도착까지
검사합니다. 종료할 때는 각 실행 터미널에서 `Ctrl+C`를 누르세요.
스크립트가 시뮬레이션용 `ROS_DOMAIN_ID=76`을 설정하므로 별도 `export`는 필요 없습니다.
Gazebo 기본 위치 추정은 기존 오도메트리(`baseline`)를 사용합니다.
시뮬레이션은 로컬 통신으로 격리합니다. 실행 스크립트의 DDS 프로필 기본값과
이전 설정으로 복귀하는 방법은 [운용 보완 안내](docs/UPGRADE_20261029.md)에 있습니다.

카메라·지도·파라미터를 PyQt 조종기로 보려면 별도 터미널에서 실행합니다.
조종기 소스는 저장소의 `tools/`에 포함되어 있습니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim_controller.sh
```

자동주행의 속도 중재기(`motion_mux`)가 실행 중이면 수동 조작은 웹 조이스틱을
사용합니다. 아래 수동 SLAM에서는 PyQt 조종기로 직접 운전할 수 있습니다.

## ArUco 미로와 웹 조종기

미로 월드·마커 4개·저장 지도·웹 파일이 모두 포함되어 있습니다.
설치·빌드 후 각각 다른 터미널에서 실행하세요.

```bash
# 터미널 1: Gazebo 미로 + 저장 지도 + 주행 노드
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_web_mission_sim.sh
```

```bash
# 터미널 2: 웹 관제·재고 요청 사이트
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_web_mission.sh
```

PC는 `http://localhost:8790/`, 같은 Wi-Fi의 휴대폰은 `http://PC의_IP:8790/`으로
접속합니다. 재고 요청 화면은 주소 뒤에 `/inventory`를 붙입니다.

## 새 지도 만들기

`SLAM` 모드에서는 경로 계획을 끄고 직접 조종하며 지도를 만듭니다.

**터미널 1 — Gazebo와 SLAM**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh slam:=true use_motion_mux:=false
```

**터미널 2 — 키보드로 조종**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source scripts/sim_env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

키보드 조종 터미널을 활성화해 로봇을 움직입니다. 매핑을 마친 뒤
**터미널 1에서 `Ctrl+C`**를 누르면
`~/Team2_workspace/maps/slam_날짜_시간_실행번호/map.yaml`과 `map.pgm`이 저장됩니다.
저장한 지도로 다시 주행할 때는 실제 출력된 경로를 넣으세요.

```bash
./scripts/run_sim.sh map:=/home/사용자명/Team2_workspace/maps/slam_날짜_시간_실행번호/map.yaml
```

매핑 조종 방법과 ArUco 미로 준비는 [Gazebo 실습 안내](docs/GAZEBO.md)에 있습니다.
키보드 대신 같은 터미널에서 `./scripts/run_sim_controller.sh`로 PyQt 조종기를
사용할 수도 있습니다.

## 실제 로봇·웹 관제

실제 로봇은 Raspberry Pi에서 센서·모터 bringup·독립 EKF·정지 감시·속도 중재를,
PC에서 AMCL·경로 계획·웹을 실행합니다. 두 장치 모두 같은 Wi-Fi와 `ROS_DOMAIN_ID=16`을 사용합니다.
시뮬레이션의 76번 도메인과 혼용하지 마세요.
실차의 기본 실행 스크립트는 **독립 바퀴·IMU EKF**를 켭니다.
처음에는 로봇에서 저장소를 받고 `./scripts/setup_robot_runtime.sh`로 로봇용 런타임을
빌드하세요. 자세한 명령은 아래 실차 시작 안내에 있습니다.
이전 PC EKF 구성으로 복귀할 때는 양쪽에서 `SMART_VISION_ESTIMATION_LOCATION=pc`를 사용합니다.
EKF 자체를 끄는 기존 오도메트리 구성은 양쪽의 `_baseline.sh`로 선택합니다.

- [팀원용 설치·실행 순서](docs/collaboration_handoff.md)
- [실차 터미널별 실행 순서](docs/classroom_navigation_startup.md)
- [재고 요청·배송 웹 사용법](docs/inventory_delivery.md)
- [ArUco 마커 준비](docs/ARUCO.md)
- [실차 기본 EKF 실행과 기존 구성으로 복귀](docs/hardware_ekf.md)
- [조종기 DEB 설치와 실습 ZIP 생성](distribution/README.md)
- [주행·화면·안전·기록 보완 내용](docs/UPGRADE_20261029.md)
- [튜닝 전후 동일 경로 반복 시험](docs/NAVIGATION_BENCHMARK.md)

진행 흐름은 `목표 → A* 경로 → Nav2 FollowPath → /cmd_vel → 로봇`입니다.

## 출처

A* 격자 탐색, 장애물 관측/기억 및 FollowPath 연결은
[`resilient-amr-astar`](https://github.com/waterhyunsu/resilient-amr-astar)를 참고했습니다.
