# 강의실 도메인 16 시작 순서

새 팀원은 [실차 설치·실행 안내](hardware_ekf.md)의 로봇 `setup_robot_runtime.sh`를 먼저 실행합니다. 기본 구조는 로봇이 EKF·안전 감시·최종 속도 mux, PC가 지도·AMCL·A*·Nav2·웹을 담당합니다.
실행 스크립트는 기본 LAN UDP 프로필을 적용합니다. 다른 RMW나 사용자의 DDS 프로필은 유지합니다. 기존 기본 통신 설정으로 비교할 때는 양쪽 실행 명령 앞에 `SMART_VISION_DDS_TRANSPORT=default`를 붙입니다. [통신 설정 설명](UPGRADE_20261029.md)을 참고하세요.

## 터미널 1 — 로봇

```bash
ssh robot@192.168.0.187
cd ~/SMART-VISION-AMR
ENABLE_FRONT_CAMERA=true ./scripts/robot_bringup_camera_16.sh
```

주소와 LiDAR·카메라 장치는 실제 로봇에 맞춥니다. 시작 후 2초 이상 정지해 IMU 바이어스를 측정합니다.

## 터미널 2 — PC 주행

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

이 명령은 Gazebo를 켜지 않습니다. 저장 지도는 `ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml`이며 현장 가벽 배치가 이 지도와 같아야 합니다. Nav2를 별도 프로세스 방식(`use_composition:=false`)으로 실행하고 로봇 runtime의 mux에 명령을 보냅니다. PC에서 EKF·mux를 중복 생성하지 않습니다.

## 터미널 3 — PC 웹

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

상시 서비스가 포트 8790을 이미 사용한다면 이 터미널을 중복 실행하지 않습니다. 웹에서 비상정지를 명시적으로 해제하고, 실제 홈에 있을 때 자세 초기화, 위치 준비 확인, 적재 확인, 임무 시작을 순서대로 진행합니다. 자세 초기화만으로 임무를 자동 출발시키지 않습니다.

## 처음마다 map/TF 오류가 나타날 때

Bringup 직후에는 아직 PC 지도·AMCL이 없어 초기화 중인 것이 정상입니다. PC 실행 후에도 계속 오류라면 순서대로 확인합니다.

1. 같은 도메인에서 SLAM/Cartographer가 `/map`을 함께 발행하는지 확인합니다.
2. 로봇 `/scan`, `/odom`이 갱신되는지 확인합니다.
3. 실제 위치·방향으로 초기 위치를 지정했는지 확인합니다.
4. 위치 신뢰도 이유와 `map→base_footprint` TF를 확인합니다.

PC의 새 터미널:

```bash
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0

ros2 topic info /map -v
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /smart_vision_amr/localization_health --once
ros2 run tf2_ros tf2_echo map base_footprint
```

위 명령은 한 개씩 실행하며 연속 출력은 `Ctrl+C`로 끝냅니다. 저장 지도 주행의 `/map` 발행자는 `/map_server` 하나여야 합니다. 시작 스크립트는 기존 지도 발행자를 발견하면 이름을 표시하고 거부합니다. 다른 PC도 같은 도메인을 쓰면 지도 충돌이 생길 수 있습니다.

SLAM 실행 터미널을 못 찾겠다면 해당 장치에서:

```bash
ps -eo pid,args | grep -E '[c]artographer_occupancy_grid_node|[c]artographer_node|[s]lam_toolbox'
```

필요 없는 SLAM을 실행한 터미널에서 `Ctrl+C`로 종료하고 PC 주행을 다시 시작합니다. 실제 지도 충돌을 경고만 숨겨서 해결하지 않습니다.

홈이 물리적으로 확인된 경우 PC의 `./scripts/set_classroom_home_pose_16.sh`도 초기화에 사용할 수 있습니다. 저장 지도 일치, 단일 지도 발행자, 최신 센서, 정지 상태와 홈 주변 스캔 일치도를 검사합니다. 로봇을 손으로 옮겼거나 재부팅했다면 이전 TF만 믿지 말고 다시 초기화합니다.

## 시작 도중 Nav2 lifecycle이 중단된 경우

기본 강의실 프로필은 bond 연결에 15초를 허용합니다. 그래도 지도 서버·AMCL 시작이 실패했다면 로그를 확인하고 PC 주행 launch를 종료 후 재시작하는 것이 우선입니다. 로봇·지도·센서가 정상이고 정지 중일 때만 아래 수명주기 재시도를 사용할 수 있습니다.

```bash
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 3}'
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
```

두 응답의 `success: true`를 확인한 뒤 실제 위치를 다시 초기화합니다. 위치 미확정 때문에 해제가 거절되면, 실제 등록 홈에 있을 때만 웹 해제 창에서 **홈 초기 위치만 지정**하고 센서 안정 후 다시 해제하세요. 이 버튼은 회전·임무 시작·비상정지 해제를 하지 않습니다. 홈이 아니면 조종기에서 실제 위치를 지정합니다. 초기 위치가 불확실한 상태에서 홈 좌표를 자동 확정하지 않습니다. [위치 복구 절차](hardware_ekf.md)를 확인하세요. baseline/이전 PC 배치로의 복귀 명령은 [실차 EKF 안내](hardware_ekf.md#기존-오도메트리로-복귀)에 있습니다.
