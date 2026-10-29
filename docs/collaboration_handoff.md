# 팀원 설치·실행 인계

Ubuntu 22.04 / ROS 2 Humble 기준입니다. **PC는 `<작업공간>/src/SMART-VISION-AMR`에 클론**, 실차 로봇은 기존 `~/turtlebot3_ws`를 유지하고 별도 runtime을 설치합니다. Gazebo는 기본 도메인 76·로컬 통신, 강의실 실차는 도메인 16·Wi-Fi 통신입니다. 휴대폰·브라우저에는 ROS 설치가 필요 없습니다.

## 1. 팀원 PC 처음 준비

ROS 2 Humble을 설치한 PC에서:

```bash
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
./scripts/setup_sim.sh
```

부족한 시스템 패키지는 스크립트가 설치 명령으로 안내합니다. 공식 TurtleBot3 Humble 소스와 저장소 패키지를 빌드합니다. 이미 클론한 PC는 저장소에서 `git pull --ff-only` 후 다음처럼 다시 빌드합니다.

```bash
cd ~/Team2_workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_vision_amr --symlink-install
```

## 2. Gazebo에서 확인

**터미널 1: Gazebo·지도·AMCL·A*·Nav2**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh
```

**터미널 2: 포함된 PyQt 조종기**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim_controller.sh
```

PyQt 소스와 지도·파라미터 패널은 `tools/`에 포함돼 있습니다. 지도·Goal·파라미터를 확인할 수 있습니다. 최종 `/cmd_vel`은 mux가 단독 발행하며 PyQt와 웹은 같은 비상정지 서비스를 사용합니다. 웹 조이스틱으로 수동 제어할 수 있습니다.

새 SLAM이 필요하면 기존 시뮬레이션을 종료하고 터미널 1에서 다음을 실행합니다. 이 수동 매핑 모드에서는 직접 조종하려고 mux를 끕니다.

```bash
./scripts/run_sim.sh slam:=true use_motion_mux:=false
```

다른 터미널에서 `./scripts/run_sim_controller.sh`로 조종합니다. SLAM 터미널의 `Ctrl+C`는 `~/Team2_workspace/maps/slam_날짜_시간_실행번호/`에 지도를 저장합니다. ArUco 미로 매핑은 `./scripts/run_sim_aruco.sh use_motion_mux:=false`입니다.

저장 지도 기반 미로 웹 실습:

```bash
# 터미널 1
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_web_mission_sim.sh
```

```bash
# 터미널 2
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_web_mission.sh
```

미로 월드·지도·ArUco 모델은 저장소에 포함돼 있습니다. 웹은 `http://127.0.0.1:8790/`, 재고 화면은 `/inventory`입니다.

## 3. 실차 로봇 처음 준비

로봇의 ROS Humble과 `~/turtlebot3_ws` 설치가 끝난 상태에서 시작합니다. PC에서 SSH로 접속하세요. 주소와 계정은 실제 로봇에 맞춥니다.

```bash
ssh robot@192.168.0.187
```

접속한 **로봇 터미널**에서:

```bash
cd ~
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd ~/SMART-VISION-AMR
./scripts/setup_robot_runtime.sh
```

이미 클론했다면 `git pull --ff-only` 후 설치 스크립트를 다시 실행합니다. `~/smart_vision_runtime_ws`에 로봇용 ROS 패키지를 빌드합니다. 기존 TurtleBot3 파일을 덮어쓰지 않으며 로봇을 자동 출발시키지 않습니다. 예전처럼 브링업 셸 파일만 복사해서는 새 robot runtime이 설치되지 않습니다.

## 4. 실차 터미널별 실행

현장 가벽 배치가 `ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml`과 같은지 확인합니다. 저장 지도 주행에서는 SLAM/Cartographer를 종료합니다.

**터미널 1 — 로봇 SSH: Bringup·카메라·독립 EKF·안전 제어**

```bash
cd ~/SMART-VISION-AMR
ENABLE_FRONT_CAMERA=true ./scripts/robot_bringup_camera_16.sh
```

**터미널 2 — PC: 지도·AMCL·A*·Nav2**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

**터미널 3 — PC: 웹**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

상시 웹 서비스가 이미 포트 8790을 사용하면 중복 실행하지 않습니다. PC는 `http://127.0.0.1:8790/`, 같은 Wi-Fi의 다른 PC·휴대폰은 `http://PC의_IP:8790/`에 접속합니다. 웹은 브링업·Nav2를 대신 시작하지 않습니다.

새 기본 배치는 **로봇**에서 EKF·센서 감시·위치 신뢰도·최종 `motion_mux`를 실행합니다. **PC**에서는 지도·AMCL·주행 계획과 웹을 실행하며 mux를 중복 생성하지 않습니다. PC의 자동 명령에는 PC 전달 경계 시각·순서가 붙고, 로봇에서 지연·중복 명령을 거부합니다. 원본 `Twist`에는 시각 필드가 없으므로 제어기 내부의 최초 생성 시각까지 증명하는 것은 아닙니다.

기본 카메라는 후방 단독 320×240 JPEG 최대 5 FPS로 압축 전송합니다. 고정 USB `by-id` 장치가 다르면 `CAMERA_DEVICE`로 지정합니다. 기본 LiDAR가 `LDS-03`과 다른 로봇은 실제 모델을 `LDS_MODEL`로 지정합니다. 전·후방 영상 동시 운용은 Wi-Fi 지연이 늘었던 이력이 있어 현장 대역폭을 측정한 뒤 활성화합니다.

## 5. 출발·위치 초기화

1. 부팅 후 정지 상태를 유지하여 IMU 바이어스 측정을 마칩니다.
2. `/scan`, `/wheel/odom`, `/imu/ekf`, `/odom`과 로봇 TF가 안정적이면 웹에서 비상정지를 명시적으로 해제합니다.
3. 실제 등록 홈에 있고 후방 카메라가 0번 마커를 보면 자세 초기화를 실행합니다. 다른 위치라면 실제 위치를 지도에 지정합니다.
4. 위치 준비 상태와 적재를 확인하고 임무를 시작합니다.

초기화 중에는 자동주행이 차단되지만, 로컬 센서가 신선하면 명시적인 저속 위치 확인 회전은 가능합니다. 운용 중 위치 신뢰도가 크게 떨어지면 비상정지를 잠급니다. 초기 위치 지정이나 센서 회복만으로 자동 출발하지 않습니다.

확인이 필요하면 PC 새 터미널에서 한 명령씩 실행합니다. 연속 출력은 `Ctrl+C`로 마칩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0

ros2 topic info /map -v
ros2 topic info /odom -v
ros2 topic info /cmd_vel -v
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /smart_vision_amr/localization_health --once
ros2 run tf2_ros tf2_echo map base_footprint
```

`/map`은 지도 서버 하나, `/odom`은 EKF 하나, `/cmd_vel`은 로봇 mux 하나가 발행해야 합니다. 재부팅·손으로 위치 이동 이후에는 초기 위치를 다시 확인합니다.

## 6. 비교 시험과 복귀

기존 오도메트리로 돌아가려면 두 장치 실행을 종료한 뒤 로봇의 `./scripts/robot_bringup_camera_16_baseline.sh`, PC의 `./scripts/run_classroom_navigation_16_baseline.sh`를 함께 실행합니다. 최종 mux와 신뢰도 감시는 로봇에 유지합니다.

로봇 runtime 도입 전의 PC 배치까지 되돌리는 경우는 **로봇 브링업·PC 주행·PC 웹 모두** `SMART_VISION_ESTIMATION_LOCATION=pc`로 실행합니다. 어느 배치든 최종 mux는 하나만 켭니다. 상세한 복귀 예시는 [실차 EKF 안내](hardware_ekf.md)를 따릅니다.

반복 주행·기록은 [주행 성능 비교](NAVIGATION_BENCHMARK.md)를 사용합니다. `--execute` 없이 관찰만 할 수 있으며 설정 파일 해시와 CSV/JSON 결과가 남습니다. 파라미터 변경·과거 설정 복원과 사고 기록도 이번 보완에 포함됩니다.

## 검증 범위

코드·단위시험·격리 Gazebo 시험은 실차 완주 성능을 보장하지 않습니다. 특히 전·후방 두 영상 동시 전송에서 `/scan`, `/odom`이 지연된 사례가 있었고, 새 robot runtime의 실제 주행 정확도는 별도 측정이 필요합니다. 반복 시험 없이 EKF가 baseline보다 정확하다고 단정하지 않습니다.

- [실차 설치·EKF·복귀](hardware_ekf.md)
- [강의실 시작 문제 해결](classroom_navigation_startup.md)
- [재고 배송](inventory_delivery.md)
- [웹 상시 서비스](classroom_web_service.md)
- [반복 주행 성능 비교](NAVIGATION_BENCHMARK.md)
