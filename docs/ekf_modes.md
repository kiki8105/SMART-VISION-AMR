# Gazebo 오도메트리·EKF 비교

실차 설정을 바꾸기 전에 Gazebo에서 세 가지 위치 추정 방식을 같은 경로로 비교합니다. 이 실험은 시뮬레이션 전용입니다. 기본 실행은 기존과 동일한 `baseline`입니다.

| `localization_mode` | `/odom`·`odom→base_footprint` 소유자 | 추가 입력 | 의미 |
| --- | --- | --- | --- |
| `baseline` | Gazebo diff-drive | 없음 | 기존 주행 성능 그대로 측정 |
| `ekf_overlay` | Gazebo diff-drive | 기존 `/odom` + `/imu` → `/odometry/filtered` | EKF를 추가했을 때 추정 오차만 비교. EKF는 주행에 개입하지 않음 |
| `ekf_independent` | `robot_localization` EKF | 휠 각도에서 만든 `/wheel/odom`의 전진속도·측면 0 제약 + `/imu`의 z축 각속도 | 독립 센서 융합으로 실제 주행까지 평가 |

`ekf_overlay`는 일부러 기존 오도메트리를 다시 EKF에 넣는 실험입니다. 두 측정값의 상관관계 때문에 오차가 작아 보이더라도 독립 센서 융합의 효과라고 해석할 수 없습니다. 기존 Gazebo 모델은 `/odom`과 해당 TF를 이미 발행하므로, 이 모드의 EKF는 TF를 발행하지 않습니다.

`ekf_independent`는 원본 TurtleBot3 SDF를 수정하지 않습니다. 실행할 때 만든 임시 SDF에서 Gazebo의 오도메트리 TF를 끄고 원시 토픽을 `/gazebo/odom_raw`로 이동합니다. 휠 조인트 각도를 적분하는 노드는 `/wheel/odom`만 발행하고 TF는 발행하지 않습니다. EKF만 `/odom`과 `odom→base_footprint`를 발행하며, AMCL은 기존처럼 `map→odom`을 담당합니다. 휠 yaw·yaw rate와 Gazebo IMU의 절대 yaw는 EKF에 넣지 않고, gyro z 각속도만 적분해 회전 추정을 만듭니다.

ROS Humble PC에 `robot_localization`을 설치하고 Team2 워크스페이스를 다시 빌드합니다.

```bash
sudo apt install ros-humble-robot-localization
cd /home/kiki/Team2_workspace
colcon build --packages-select smart_vision_amr --symlink-install
```

같은 지도와 경로에서 모드를 한 번에 하나씩 실행합니다. 실행 사이에는 Gazebo·ROS 노드를 모두 종료합니다. 실차에서는 독립 EKF가 기본 실행이며 복귀 방법은 [실차 EKF 안내](hardware_ekf.md)를 참고하세요.

```bash
./src/SMART-VISION-AMR/scripts/run_sim.sh localization_mode:=baseline
./src/SMART-VISION-AMR/scripts/run_sim.sh localization_mode:=ekf_overlay
./src/SMART-VISION-AMR/scripts/run_sim.sh localization_mode:=ekf_independent
```

각 실행에서 `/odom`, `/tf`, `/imu`, `/wheel/odom`, `/odometry/filtered`의 발행자를 확인합니다. `ekf_overlay`에서 `/odometry/filtered`만 새로 생기고, `ekf_independent`에서 `/odom`과 `odom→base_footprint`의 발행자가 각각 하나인지 확인해야 합니다. [반복 위치 추정 시험](ekf_benchmark.md)은 실제 Gazebo 위치 대비 오차와 TF 공백을 측정합니다. 경로 오차, 도착률, 회복 횟수와 소요 시간은 같은 A*/Nav2 임무를 다시 실행해 별도로 비교해야 합니다. 현재 Gazebo 기본 오도메트리는 실제 하드웨어보다 정확할 수 있으므로, 시뮬레이션 우승 모드를 실차에 그대로 적용하지 않습니다. 실차에서는 OpenCR의 기본 `use_imu: true`와 `publish_tf: true`가 독립 EKF와 동시에 켜지지 않도록 별도 실행 모드로 분리합니다.
