# 팀원용 최종 설치·실행 매뉴얼

Ubuntu 22.04 / ROS 2 Humble 기준입니다. 명령어 앞의 `$`나 터미널 프롬프트는 복사하지 않습니다.
PC 사용자 이름이 달라도 `~`를 쓰므로 명령어를 그대로 사용할 수 있습니다.

| 구분 | 실행 위치 | 역할 |
|---|---|---|
| 터미널 1 | PC에서 SSH → 로봇 | 브링업·카메라·독립 EKF·최종 속도 제어 |
| 터미널 2 | 로컬 PC | 저장 지도·AMCL·A*·Nav2 |
| 터미널 3 | 로컬 PC | 웹 운용·재고 사이트 |
| 터미널 4 (선택) | 로컬 PC | PyQt 조종기·지도·파라미터 |

실차는 **ROS_DOMAIN_ID=16**, 시뮬레이션은 **76**입니다. 제공 스크립트가 설정합니다.
한 로봇의 브링업·주행·웹은 한 운용자가 실행하고, 다른 팀원은 그 PC의 웹에 접속합니다.
휴대폰에는 ROS를 설치할 필요가 없습니다.

## 1. PC 처음 설치 — 한 번만

ROS 2 Humble이 설치된 PC에서 필요한 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions \
  gazebo ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup ros-humble-xacro ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-compressed-image-transport \
  ros-humble-teleop-twist-keyboard python3-opencv python3-pyqt5 \
  python3-numpy python3-yaml python3-pil
```

저장소를 받고 빌드합니다. 이미 같은 폴더에 클론했다면 다음 절의 업데이트 절차를 사용하세요.

```bash
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
./scripts/setup_sim.sh
```

`setup_sim.sh`는 공식 TurtleBot3 시뮬레이션 소스를 준비하고 작업공간을 빌드합니다.
Gazebo·로봇을 자동 실행하지 않습니다. 완료 후 아래 로봇 준비로 넘어갑니다.

## 2. 로봇 처음 설치 — 한 번만

PC에서 로봇에 접속합니다.

```bash
ssh robot@192.168.0.187
```

비밀번호 요청에는 안내받은 **로봇 계정 비밀번호**를 입력합니다. 입력 문자는 화면에 표시되지 않습니다.
아래 명령은 SSH로 접속한 `robot@robot` 터미널에서 실행합니다.
로봇에는 ROS Humble과 기존 `~/turtlebot3_ws`가 설치되어 있어야 합니다.

```bash
sudo apt update
sudo apt install -y git
cd ~
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd ~/SMART-VISION-AMR
./scripts/setup_robot_runtime.sh
```

이 스크립트가 로봇용 의존성을 설치하고 `~/smart_vision_runtime_ws`를 빌드합니다.
기존 `~/turtlebot3_ws`는 유지합니다. 로봇에서 Gazebo·Nav2를 실행할 필요는 없습니다.

매번 `~/turtlebot16`으로 켜려면 **로봇 터미널에서** 아래 파일을 한 번 만듭니다.

```bash
cat > ~/turtlebot16 <<'SH'
#!/usr/bin/env bash
exec env ENABLE_FRONT_CAMERA="${ENABLE_FRONT_CAMERA:-true}" \
  "$HOME/SMART-VISION-AMR/scripts/robot_bringup_camera_16_ekf.sh" "$@"
SH
chmod +x ~/turtlebot16
```

이 파일은 저장소의 최신 브링업 스크립트를 호출합니다. 예전 이름의 실행 파일이 이미 있다면
위 내용으로 맞춥니다. 실제 실행은 아래 터미널 1 순서에서 합니다.

카메라는 저장된 USB `by-id` 경로와 전방 `camera0`를 사용합니다. 다른 카메라를 연결했다면
[카메라·로봇 런타임 설정](hardware_ekf.md)을 확인하세요.
LiDAR는 로봇의 기존 `LDS_MODEL` 설정을 사용하며 미설정 기본값은 `LDS-03`입니다.
실물이 `LDS-02`라면 실행할 때 `LDS_MODEL=LDS-02 ~/turtlebot16`을 사용합니다.

## 3. 코드 업데이트 — 팀원이 변경 사항을 받는 경우

주행을 종료한 상태에서 수행합니다. 새로 클론했다면 반복할 필요가 없습니다.

**로컬 PC:**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
git pull --ff-only
cd ~/Team2_workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_vision_amr
```

**SSH 접속한 로봇:**

```bash
cd ~/SMART-VISION-AMR
git pull --ff-only
./scripts/setup_robot_runtime.sh --no-install
```

의존성 추가 안내가 있는 업데이트라면 로봇의 `--no-install`을 빼고 실행합니다.
실행 중인 프로세스는 `git pull`만으로 바뀌지 않으므로, 아래 순서대로 다시 켭니다.

## 4. 실차 매일 실행 — 터미널별 순서

현재 강의실 가벽·마커 배치가 포함된 저장 지도와 같은 상태에서 사용합니다.
지도는 `ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml`입니다.
이 과정은 새 SLAM이 아니라 **저장 지도에서 주행**하는 과정입니다.

### 터미널 1 — SSH 접속 후 로봇 실행

```bash
ssh robot@192.168.0.187
```

로그인 후, 같은 터미널에서:

```bash
~/turtlebot16
```

브링업·전후방 카메라·바퀴/IMU 독립 EKF·센서 감시·최종 motion mux가 실행됩니다.
이 터미널은 켜 둡니다. 시작 직후 로봇을 정지시켜 IMU 초기 측정을 마칩니다.

`~/turtlebot16`을 만들지 않았다면 대신 아래 한 줄을 사용합니다. **두 명령을 동시에 실행하지 않습니다.**

```bash
ENABLE_FRONT_CAMERA=true ~/SMART-VISION-AMR/scripts/robot_bringup_camera_16_ekf.sh
```

### 터미널 2 — 로컬 PC에서 주행 실행

SSH 터미널과 별개로 PC 터미널을 새로 엽니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

저장 지도·AMCL·A*·Nav2가 실행됩니다. Gazebo와 RViz는 켜지지 않습니다.
PC에서 EKF나 최종 mux를 중복 실행하지 않으며, 로봇 런타임 준비가 안 됐으면 이유를 출력하고 중단합니다.
폴더 이동은 `cd`로 하고, 위의 `./scripts/run_classroom_navigation_16.sh`를 별도 명령으로 실행합니다.

### 터미널 3 — 로컬 PC에서 웹 실행

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

브라우저에서:

- 로봇 운용: <http://127.0.0.1:8790/>
- 재고 요청: <http://127.0.0.1:8790/inventory>

다른 PC·휴대폰은 운용 PC와 같은 Wi-Fi에 연결합니다. **운용 PC**에서 다음으로 IP를 확인합니다.

```bash
hostname -I
```

예를 들어 PC의 Wi-Fi 주소가 `192.168.0.110`이면 다음 주소를 공유합니다.

- 로봇 운용: <http://192.168.0.110:8790/>
- 재고 요청: <http://192.168.0.110:8790/inventory>

PC 주소가 바뀌면 링크도 바뀝니다. 로봇 SSH 주소인 `192.168.0.187`과 웹 서버 PC 주소를 구분합니다.
웹의 도메인은 **16**으로 맞춥니다. 상시 웹 서비스가 이미 8790을 사용 중이면 터미널 3은
추가 실행하지 않고 기존 웹에 접속합니다. 웹만 켠 상태에서는 브링업·주행이 자동으로 시작되지 않습니다.

### 터미널 4 — 선택: PyQt 조종기

지도·차량 위치·파라미터를 데스크톱 창으로 보고 싶을 때만 실행합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_controller_16.sh
```

운용 중 수동 개입은 웹 조이스틱을 사용합니다. 임무보다 조이스틱이 우선하며,
비상정지를 해제하거나 자세를 초기화한 것만으로 임무를 자동 시작하지 않습니다.

## 5. 웹에서 출발하기

1. 지도·로봇 전압·전후방 영상과 위치 준비 상태를 확인합니다.
2. 실제 등록 홈에 정지해 있고 후방 카메라가 0번 마커를 볼 때만 **자세 초기화**를 사용합니다.
   다른 위치라면 실제 위치·방향을 지정합니다.
3. 비상정지가 잠겼으면 웹 안내에 따라 명시적으로 해제하고 위치 준비 상태를 다시 확인합니다.
4. 재고 관리자가 `/inventory`에서 Sector 요청을 등록합니다.
5. 로봇 운용자가 적재를 확인하고 임무를 출발시킵니다.

Home은 `(-0.11, -0.79)`, 차체 전방 yaw `0`이며 후방 카메라가 0번을 봅니다.
Sector 1~3은 전방 카메라로 표식을 확인하므로 전방 영상이 필요합니다.
등록 위치는 [마커 설정](../ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers.yaml)에서 확인합니다.
로봇을 들어 옮겼거나 전원을 다시 켰으면 이전 위치 추정을 그대로 사용하지 않습니다.

## 6. 화면에 입력 대기·TF 오류가 계속 뜨는 경우

처음 켜서 AMCL이 위치를 잡기 전에는 준비 대기가 나타날 수 있습니다.
현재 입력 확인은 scan/odom 0.75초, footprint 1.5초, local costmap 2초 이내 수신을 요구합니다.
입력 문제가 생기면 목표를 유지하고 정지·취소 확인 후 재계획합니다.
입력 회복이 수동 정지나 비상정지를 해제하지는 않습니다.

별도 **PC 점검 터미널**에서:

```bash
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
source ~/Team2_workspace/src/SMART-VISION-AMR/scripts/dds_env.bash
```

아래 명령은 **하나씩** 실행합니다. 연속 출력은 `Ctrl+C`로 끝내고 다음 명령으로 넘어갑니다.

```bash
ros2 topic info /map -v
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /local_costmap/costmap
ros2 topic hz /local_costmap/published_footprint
ros2 topic echo /smart_vision_amr/navigation_status
ros2 run tf2_ros tf2_echo map base_footprint
```

`/map`은 지도 서버 하나, `/odom`은 EKF 하나, `/cmd_vel`은 로봇 mux 하나가 발행하는 구성입니다.
같은 도메인의 SLAM/Cartographer나 다른 PC의 중복 주행 런치를 종료한 뒤 다시 시작합니다.
포트·장치 점유 오류도 기존 실행 터미널을 찾아 `Ctrl+C`로 종료합니다.
[시작 문제 해결](classroom_navigation_startup.md)과 [이번 통합 내용](TEAM_NAVIGATION_INTEGRATION.md)을 참고하세요.

## 7. Gazebo로 먼저 시험하는 경우 — 실차와 별도

로봇 SSH 접속 없이 PC에서 실행합니다. 도메인 76과 로컬 ROS 통신을 사용합니다.

**터미널 1 — ArUco 미로·저장 지도·주행:**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_web_mission_sim.sh
```

**터미널 2 — 시뮬레이션용 웹:**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_web_mission.sh
```

PC 브라우저 <http://127.0.0.1:8790/>에 접속하고 도메인 **76**을 사용합니다.
실차용 웹이 같은 PC의 8790을 사용 중이면 기존 웹을 정상 종료하고 실행합니다.
이번 통합의 오프라인 회귀와 빌드 통과는 새 실차·Gazebo 완주 검증을 대신하지 않습니다.

## 8. 종료하기

움직이는 중이면 웹에서 먼저 정지시킵니다. 이어서 터미널 3(웹), 2(주행), 1(로봇) 순서로
각각 `Ctrl+C`를 누릅니다. SSH 자체를 끝내려면 브링업 종료 후 `exit`를 입력합니다.

## 마지막 요약 — 설치 완료 후 매번 이 순서

**터미널 1 (PC → 로봇 SSH):**

```bash
ssh robot@192.168.0.187
```

로그인 완료 후:

```bash
~/turtlebot16
```

**터미널 2 (로컬 PC):**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

**터미널 3 (로컬 PC):**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

**브라우저:** <http://127.0.0.1:8790/> → 도메인 16 → 실제 위치 확인·자세 초기화 → 적재 확인 → 임무 출발.
