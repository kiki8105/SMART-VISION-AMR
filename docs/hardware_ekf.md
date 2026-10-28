# 실차 독립 EKF 적용과 복귀

도메인 16 실차에서만 선택하는 오도메트리 모드입니다. 기존 Gazebo와 실차 기본 실행은 그대로 유지합니다. **Pi와 PC를 같은 모드로 실행**해야 합니다. 두 모드의 bringup 또는 Nav2를 동시에 켜면 `/odom`과 `odom→base_footprint`가 중복되므로, 전환할 때는 기존 터미널을 `Ctrl+C`로 종료하고 새 모드를 시작합니다.

| 모드 | Pi의 바퀴·IMU 처리 | PC의 `/odom`·TF 소유자 |
| --- | --- | --- |
| 기존 `baseline` | ROBOTIS 기본 오도메트리 (`use_imu=true`, `publish_tf=true`) | Pi의 TurtleBot3 노드 |
| `ekf_independent` | 원본 바퀴 각도는 `/joint_states`로 전달. TurtleBot3가 만든 바퀴 전용 오도메트리는 `/wheel/odom_raw`로 분리하고 TF 발행은 끔 | PC의 `robot_localization` EKF만 `/odom`과 `odom→base_footprint` 발행 |

독립 모드에서 PC의 `wheel_joint_odometry`는 `/joint_states`의 좌·우 바퀴 각도로 `/wheel/odom`을 만듭니다. EKF는 여기서 **전진속도와 차동구동의 측면속도 0 제약**만 취하고, IMU에서는 **z축 각속도**만 취합니다. 바퀴 회전각과 IMU 방향각을 다시 넣지 않아 같은 회전 정보를 중복 융합하지 않습니다. Pi의 기존 `/odom`은 EKF 입력으로 사용하지 않습니다.

## 실행

Pi에는 저장소의 `hardware_robot_ekf.launch.py`를 **`robot_bringup_camera_16.sh`와 같은 폴더**에 복사합니다. 기존 스크립트도 새 버전으로 교체하고, `robot_bringup_camera_16_ekf.sh`와 `ros_camera_compressed.py`를 같은 곳에 둡니다. EKF launch는 Pi에 **이미 설치된** Waffle Pi YAML에서 실행 시 바퀴 전용 임시 설정을 만들므로, 로봇마다 다른 OpenCR·바퀴 설정을 저장소 값으로 덮어쓰지 않습니다. EKF용 Pi 래퍼는 기본적으로 카메라를 5 FPS JPEG로 제한해 Wi-Fi의 센서 데이터 전송 지연을 줄이며, 필요하면 `CAMERA_BACKEND` 등 기존 환경변수로 바꿀 수 있습니다. PC에는 `robot_localization`과 빌드된 `smart_vision_amr` 패키지가 필요합니다.

```bash
# PC에서 실행: Pi에 프로필 파일만 복사 (Pi 주소가 바뀌었다면 수정)
cd /home/kiki/Team2_workspace/src/SMART-VISION-AMR
scp scripts/robot_bringup_camera_16.sh scripts/robot_bringup_camera_16_ekf.sh \
  scripts/ros_camera_compressed.py \
  ros2_ws/src/smart_vision_amr/launch/hardware_robot_ekf.launch.py \
  robot@192.168.0.187:/home/robot/
ssh robot@192.168.0.187 \
  'chmod +x /home/robot/robot_bringup_camera_16.sh /home/robot/robot_bringup_camera_16_ekf.sh'

# PC에서 새 노드와 launch 파일 빌드
cd /home/kiki/Team2_workspace
colcon build --packages-select smart_vision_amr --symlink-install
```

```bash
# Pi 터미널: 기존 bringup을 먼저 Ctrl+C로 종료
~/robot_bringup_camera_16_ekf.sh

# PC 터미널: 기존 Nav2를 먼저 Ctrl+C로 종료
cd /home/kiki/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16_ekf.sh
```

실차 EKF는 반드시 위의 `./scripts/run_classroom_navigation_16_ekf.sh`로 시작합니다. `ros2 launch smart_vision_amr hardware.launch.py localization_mode:=ekf_independent`를 직접 실행하면 Pi 모드·중복 발행자·센서 타임스탬프 사전 검사를 건너뛰므로 실차 운용에는 지원하지 않습니다.

웹 관제와 조종의 `motion_mux`도 실행되어 있어야 합니다. 기존 웹 서비스가 꺼져 있다면 별도 PC 터미널에서 `./scripts/run_classroom_web_16.sh`를 시작합니다. EKF 모드를 켜면 안전 감시가 **시작 직후 비상정지를 걸어 둡니다.** 로봇을 정지 상태로 유지하고 센서 흐름과 AMCL 위치를 확인한 뒤, `/wheel/odom`, `/imu/ekf`, `/scan`이 2초 이상 연속 정상일 때 웹에서 **비상정지 해제**를 누릅니다. 그 후 **임무 시작/재개**를 별도로 누릅니다. 센서가 정상이라는 이유만으로 자동 출발하지 않습니다.

PC 스크립트는 시작할 때 ROS daemon 캐시 대신 실제 DDS 발행자를 조회합니다. `/wheel/odom_raw`, `/joint_states`, `/imu`, `/scan` 발행자가 **각각 정확히 한 개**이고, 아직 `/odom` 발행자가 없어야 EKF를 켭니다. 세 센서의 **실제 메시지**도 각각 받아 타임스탬프가 PC 시각보다 1초 넘게 오래되거나 0.25초 넘게 미래가 아닌지 확인합니다. 이 검사는 서로 다른 모드의 Pi·PC 실행과 시각 불일치를 막습니다. 출발 전에 `/imu/ekf`, `/wheel/odom`, `/odom`과 두 TF(`odom→base_footprint`, `map→odom`)의 갱신을 확인합니다. EKF를 켠 경우 `/odom` 발행자는 **하나**여야 합니다. AMCL의 초기 위치는 실제 로봇 위치와 지도의 홈이 일치할 때만 입력합니다. 이 확인이 끝나기 전에는 임무를 시작하지 않습니다.

Pi와 PC의 시스템 시각도 맞아야 합니다. 이번 모드의 바퀴·IMU 타임스탬프 검사는 PC 시각과 비교하므로 두 장치의 시계가 어긋나면 새 데이터도 오래되거나 미래의 데이터로 판단될 수 있습니다. 두 장치에서 `date -u`를 비교하고 시각 동기화를 확인합니다.

Pi를 재부팅하거나 bringup을 다시 켠 경우, PC의 EKF/Nav2도 종료하고 다시 시작합니다. 바퀴 각도의 적분 기준과 `odom`의 연속성이 재부팅 전후로 달라질 수 있기 때문입니다. 재시작 뒤 실제 로봇 위치에서 AMCL 초기 위치도 다시 설정합니다.

```bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
ros2 topic info /odom -v
ros2 topic hz /joint_states
ros2 topic hz /imu
ros2 topic hz /imu/ekf
ros2 topic hz /wheel/odom
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

## 오차 누적의 한계와 대응

바퀴와 IMU만으로는 **절대 위치를 복원할 수 없습니다.** 바퀴 미끄럼과 IMU 바이어스가 생기면 EKF의 `odom` 기준 위치도 시간이 지나며 누적 오차가 납니다. 지도와 LiDAR를 비교하는 AMCL이 별도의 `map→odom`을 갱신하여 **지도 기준 위치**를 보정합니다. 따라서 지도·LiDAR·AMCL이 끊기면 자동으로 정확한 위치가 유지된다고 보장할 수 없습니다.

이번 모드에는 누적 오차를 *줄이기 위한* 장치를 넣었습니다. 출발 전 바퀴가 정지한 상태에서 IMU 각속도 25개 이상을 2초 이상 모아 바이어스를 추정하고, 이후에도 바퀴가 정지했다고 판단될 때만 천천히 바이어스를 갱신합니다. 비정상적으로 큰 각속도, 시간상 오래된 바퀴 측정값, 바퀴 각도의 갑작스러운 점프는 입력에서 제외합니다. EKF는 센서 타임아웃과 이상치 거부 설정을 사용합니다. 실제 출발 전에는 `/imu/ekf`가 발행되도록 로봇을 최소 2초 이상 정지시켜 두고, 지도 기준 위치가 맞는지 확인합니다.

EKF 모드 전용 `ekf_sensor_guard`는 `/wheel/odom`, `/imu/ekf`, `/scan`의 **도착 시각과 메시지 시각**을 확인합니다. 시작 직후에도 정지 상태를 잠그며, 정상 입력이 2초간 이어지고 운용자가 웹에서 명시적으로 해제해야 주행이 가능합니다. 시작 후 하나라도 1초 이상 끊기면 웹 조종보다 우선하는 비상정지를 다시 겁니다. 센서가 회복되어도 자동 해제·출발하지 않습니다. 세 흐름이 다시 2초 이상 안정적일 때 웹에서 **비상정지 해제**를 누르고 임무를 별도로 재개합니다. 안전 감시는 웹의 해제 요청을 직접 확인하므로 웹이나 조종 노드의 재시작만으로 정지 잠금이 풀리지 않습니다. 감시 노드의 비상정지 메시지는 웹이 소유한 `motion_mux`로 전달됩니다. 이 감시는 센서 단절에 대한 정지 장치이지, 살아 있지만 잘못된 센서 측정이나 바퀴 미끄럼으로 생기는 위치 오차를 제거하는 장치는 아닙니다.

EKF, 바퀴 오도메트리, IMU 보정기 또는 안전 감시 노드가 비정상 종료되면 실차 EKF용 PC launch 전체를 종료하도록 구성합니다. 원인을 확인하고 새로 실행한 뒤에도 시작 단계의 비상정지 해제 절차를 다시 밟습니다.

## 즉시 기존 구성으로 복귀

Pi와 PC의 EKF 실행 터미널을 각각 `Ctrl+C`로 종료한 다음, **기존 스크립트**를 실행합니다. 설치된 ROBOTIS YAML을 변경하지 않았으므로 파일 복원 작업은 없습니다.

```bash
# Pi 터미널
~/robot_bringup_camera_16.sh

# PC 터미널
cd /home/kiki/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_navigation_16.sh
```

EKF 안전 감시가 웹 `motion_mux`에 비상정지를 잠가 둔 상태일 수 있습니다. EKF 감시 노드가 종료된 뒤 기본 모드의 센서·TF가 정상인지 확인하고, 웹에서 **비상정지 해제**와 **임무 재개**를 명시적으로 실행합니다. 안전 감시가 아직 실행 중일 때는 해제하지 않습니다.

Gazebo에서 독립 EKF가 동작한 것은 확인했지만 이 실차 모드의 주행 정확도는 아직 측정하지 않았습니다. 동일한 실차 경로에서 기본 모드와 EKF 모드의 도착 오차, 추정 위치 끊김, TF 지연을 기록한 후 성능을 판단해야 합니다.
