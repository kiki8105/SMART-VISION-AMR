# 실차 EKF 설치·실행·복귀

도메인 16 실차는 **로봇에서 바퀴·IMU EKF와 최종 속도 제어**, **PC에서 지도·AMCL·A*·Nav2·웹**을 실행합니다. 웹이나 영상 처리가 지연돼도 로봇의 명령 타임아웃 감시는 별도 프로세스에서 동작합니다. 실차 주행 정확도 개선은 아직 측정 전입니다.

| 위치 | 실행 내용 |
|---|---|
| 로봇 | Bringup, 카메라, 바퀴 적분, IMU 바이어스 보정, EKF, 센서·위치 신뢰도 감시, 최종 `motion_mux` |
| PC | 저장 지도, AMCL, A*·Nav2, 명령 시각 부여, 사고 기록, 웹 |
| TF 소유자 | EKF가 `odom→base_footprint`, AMCL이 `map→odom` |

EKF 입력은 바퀴의 **전진속도·측면속도 0 제약**과 IMU의 **z축 각속도**입니다. 바퀴 회전각과 IMU 방향각을 함께 중복 융합하지 않습니다. ROBOTIS 기존 오도메트리는 `/wheel/odom_raw`로 분리하며 EKF 입력으로 사용하지 않습니다.

## 1. 로봇에 처음 설치하기

Ubuntu 22.04, ROS 2 Humble, 기존 `~/turtlebot3_ws`가 설치된 로봇 기준입니다. PC에서 SSH로 접속합니다. 주소/계정은 실제 장치에 맞게 바꾸세요.

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

이미 클론했다면 `git clone` 대신 해당 폴더에서 `git pull --ff-only`를 실행합니다. 설치 스크립트는 `~/smart_vision_runtime_ws`에 로봇용 패키지만 빌드하며 기존 TurtleBot3 작업공간은 유지합니다. 설치만 하고 로봇이나 카메라를 켜지는 않습니다.

## 2. 터미널별 실행

전환 전 기존 Bringup·카메라·Nav2를 각 실행 터미널에서 `Ctrl+C`로 종료합니다. 저장 지도 주행에서는 SLAM/Cartographer도 종료합니다.

**터미널 1 — 로봇 SSH: Bringup·카메라·EKF·제어**

```bash
cd ~/SMART-VISION-AMR
ENABLE_FRONT_CAMERA=true ./scripts/robot_bringup_camera_16.sh
```

강의실 Home과 Sector 1~3은 모두 후방 카메라로 표식을 정면 확인하고 차체 길이 방향을 벽과 90°로 정렬합니다. 위 명령은 운용 보조용 전방 카메라도 활성화합니다. 전방 영상은 도착 조건이 아니며 스크립트 자체의 기본 전방 설정은 꺼짐입니다. 기본 후방 장치는 고정 `by-id` 경로의 웹캠이며 320×240 JPEG, 최대 5 FPS로 전송합니다. 다른 웹캠은 `CAMERA_DEVICE=/dev/v4l/by-id/실제장치`로 지정합니다. LiDAR 종류가 기본 `LDS-03`과 다르면 `LDS_MODEL=LDS-02 ./scripts/robot_bringup_camera_16.sh`처럼 실제 모델을 지정합니다. 새 로봇은 카메라 장치와 LiDAR 모델을 먼저 확인하세요.

**터미널 2 — PC: 저장 지도·AMCL·A*·Nav2**

PC 설치·빌드는 [README](../README.md)를 먼저 따릅니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

**터미널 3 — PC: 웹**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

이미 상시 웹 서비스가 켜졌다면 터미널 3을 중복 실행하지 않습니다. PC는 `http://127.0.0.1:8790/`, 같은 Wi-Fi의 다른 기기는 `http://PC의_IP:8790/`에 접속합니다. 기본 구성에서 PC의 Nav2·웹은 `/cmd_vel`을 직접 발행하지 않으며 로봇의 `motion_mux`가 유일한 최종 발행자입니다.

## 3. 출발 전 초기화

1. 로봇을 최소 2초 정지시켜 IMU 바이어스 측정을 마칩니다. `/imu/ekf`가 나와야 합니다.
2. 웹에서 **비상정지 해제**를 누릅니다. 로컬 센서·EKF 출력이 안정적이어야 해제됩니다. 자동 출발하지 않습니다.
3. 실제 등록 홈 위치·방향이 맞으면 **자세 초기화**를 실행합니다. 홈이 아니면 지도에서 실제 위치를 지정합니다.
4. 위치 신뢰도가 준비된 뒤 물품 적재를 확인하고 **임무 시작/재개**를 누릅니다.

부팅 직후 지도·AMCL이 없을 때는 `초기화 중` 경고로 표시하며 자동주행을 차단합니다. 최신 LiDAR·odom·odom TF가 있으면 명시적인 저속 위치 확인 회전은 허용합니다. 이미 위치를 잡은 뒤 신뢰도를 잃으면 비상정지를 잠급니다. 다시 위치를 잡아야 한다면 **초기 위치 지정 → 로컬 센서 안정 확인 → 비상정지 해제 → 위치 확인 회전** 순서로 진행합니다. 초기 위치 지정만으로 비상정지가 풀리지 않습니다.

비상정지 때문에 자세 초기화 버튼을 누를 수 없다면 **비상정지 해제 창 → 실제 등록 홈 위치·방향 확인란 → 등록 홈 초기 위치만 입력**을 사용합니다. 최신 센서·정지 상태·저장 지도·등록 홈에서의 LiDAR 일치 검사를 통과해야 하며, 이 버튼은 회전·목표 전송·비상정지 해제를 하지 않습니다. 센서가 안정된 뒤 같은 창에서 해제하고 자세 초기화를 실행하세요. 실제 홈이 아니라면 지도 조종기에서 현재 위치를 지정해야 합니다.

## 확인 명령

PC의 별도 터미널에서 아래 명령을 **한 개씩** 실행합니다. 계속 출력되는 명령은 `Ctrl+C`로 마칩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0

ros2 topic info /odom -v
ros2 topic hz /imu/ekf
ros2 topic hz /wheel/odom
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /smart_vision_amr/localization_health --once
```

`/odom` 발행자는 EKF 하나, `/cmd_vel` 발행자는 로봇 mux 하나여야 합니다. PC 사전 검사는 실제 DDS 발행자와 메시지 시각을 확인합니다. 로봇 실행을 다시 켰다면 PC Nav2도 재시작하고 실제 위치에 맞게 초기화합니다. 두 장치의 시계가 어긋나면 정상 센서도 오래된 데이터로 판정하므로 시간 동기화를 확인합니다.

## 오차 누적과 설정

바퀴·IMU만으로 절대 위치를 복원할 수 없습니다. AMCL이 저장 지도·LiDAR를 대조해 지도 기준 누적 오차를 보정합니다. `ekf_sensor_guard`는 입력 세 흐름뿐 아니라 **실제 `/odom` 출력과 `odom→base` TF, 수치·공분산**도 감시합니다. `localization_health`는 AMCL 불확실성·odom 대비 위치 점프·측정 시각 TF로 투영한 LiDAR와 지도 일치도를 감시합니다. 정지 중 AMCL 메시지가 뜸한 것은 정상으로 취급합니다.

- EKF·전처리·감시 설정: `ros2_ws/src/smart_vision_amr/config/ekf_hardware.yaml`
- 위치 신뢰도 설정: `ros2_ws/src/smart_vision_amr/config/localization_health.yaml`
- 실제 바퀴 치수는 로봇에 설치된 ROBOTIS YAML에서 읽습니다. 반복된 `/**` 블록도 보존하며 모호한 설정은 거부합니다.
- 바퀴 치수·센서 오차 후보 추출: `python3 scripts/calibrate_wheel_imu_reference.py --help`
- 동일 경로 반복 기록: [주행 비교 도구](NAVIGATION_BENCHMARK.md)

교정한 치수를 시험할 때는 robot runtime launch의 `wheel_radius:=0.033`과 `wheel_separation:=0.287` 인자로 명시적으로 덮어쓸 수 있습니다. 숫자는 예시이며 실측값으로 바꾸세요. 비워두면 설치된 ROBOTIS 치수를 사용합니다. 기존 vendor YAML을 수정할 필요는 없습니다.

현재 공분산·임계값은 초기 설정입니다. 실차 측정을 완료한 값으로 간주하면 안 됩니다. 필터·감시·최종 mux 노드가 종료되면 로봇 runtime 전체를 종료합니다. PC 명령이 끊기면 로봇 mux가 오래된 명령을 버리고 정지합니다.

## 기존 오도메트리로 복귀

Pi·PC 실행을 종료한 뒤 양쪽에서 baseline을 선택합니다. 로봇의 별도 최종 mux·신뢰도 감시는 유지하며 EKF 융합만 끕니다.

```bash
# 로봇 터미널
cd ~/SMART-VISION-AMR
ENABLE_FRONT_CAMERA=true CAMERA_BACKEND=rate_limited ./scripts/robot_bringup_camera_16_baseline.sh
```

```bash
# PC 터미널
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16_baseline.sh
```

로봇 runtime 설치 전의 **PC 처리 구조**까지 되돌리는 경우에는 Pi·PC 양쪽의 해당 명령 앞에 `SMART_VISION_ESTIMATION_LOCATION=pc`를 붙입니다. PC 웹에도 같은 변수를 붙여야 웹이 mux를 소유합니다. 예를 들어 Pi는 `SMART_VISION_ESTIMATION_LOCATION=pc ./scripts/robot_bringup_camera_16_baseline.sh`, PC는 같은 변수로 navigation/web baseline을 실행합니다. 다른 배치 방식의 mux를 동시에 켜지 않습니다.
