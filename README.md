# SMART-VISION-AMR

TurtleBot3 Waffle Pi가 저장 지도에서 A* 경로를 만들고 Nav2로 이동하는 프로젝트입니다.
Gazebo에서 먼저 시험하고, 실제 로봇과 웹 배송 관제로 이어서 사용할 수 있습니다.

## 처음 설치하기 (PC)

아래 명령은 **Ubuntu 22.04와 ROS 2 Humble이 설치된 PC**에서 실행합니다.
예시 작업공간은 `~/Team2_workspace`입니다. 저장소를 `src` 안에 두어야
제공된 스크립트가 작업공간을 올바르게 찾습니다.

**1. 필요한 프로그램 설치**

```bash
sudo apt update
sudo apt install -y git cmake python3-colcon-common-extensions gazebo \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup \
  ros-humble-xacro ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-teleop-twist-keyboard \
  python3-pyqt5 python3-numpy python3-yaml python3-pil
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
ArUco 웹 미로는 **별도로 저장한 미로 지도**가 필요하므로 처음 설치 직후의 실행 예제와는 구분합니다.

## 실제 로봇·웹 관제

실제 로봇은 Raspberry Pi에서 센서·모터 bringup을, PC에서 위치 추정·경로 계획을
실행합니다. 두 장치 모두 같은 Wi-Fi와 `ROS_DOMAIN_ID=16`을 사용합니다.
시뮬레이션의 76번 도메인과 혼용하지 마세요.

- [팀원용 설치·실행 순서](docs/collaboration_handoff.md)
- [실차 터미널별 실행 순서](docs/classroom_navigation_startup.md)
- [재고 요청·배송 웹 사용법](docs/inventory_delivery.md)
- [ArUco 마커 준비](docs/ARUCO.md)
- [독립 바퀴·IMU EKF 설정과 원래 구성으로 복귀](docs/hardware_ekf.md)

진행 흐름은 `목표 → A* 경로 → Nav2 FollowPath → /cmd_vel → 로봇`입니다.

## 출처

A* 격자 탐색, 장애물 관측/기억 및 FollowPath 연결은
[`resilient-amr-astar`](https://github.com/waterhyunsu/resilient-amr-astar)를 참고했습니다.
