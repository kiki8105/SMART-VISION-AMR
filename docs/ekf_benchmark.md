# Waffle Pi EKF 비교 시험

이 시험은 같은 Gazebo 월드·시작 위치·속도 명령에서 세 가지 로컬 위치 추정 방식을 비교합니다. 실제 로봇 설정을 바꾸지 않습니다.

| 모드 | 입력과 출력 | 주행에 사용되는 위치 |
|---|---|---|
| `baseline` | 기존 Gazebo `/odom`, 기존 `odom→base_footprint` TF | 기존 값 |
| `ekf_overlay` | 기존 `/odom`과 `/imu`를 EKF에 추가 입력, 후보 `/odometry/filtered` 발행 | **기존 값 유지**. 후보는 진단용으로만 측정 |
| `ekf_independent` | `/joint_states`에서 적분한 독립 `/wheel/odom` 속도와 `/imu`의 yaw rate를 EKF에 입력 | EKF가 유일한 `/odom` 및 `odom→base_footprint` TF 발행자 |

세 번째 모드는 기존 Gazebo 오도메트리를 다시 EKF에 넣지 않습니다. 한 측정의 위치·속도·방위를 여러 입력처럼 세는 중복을 줄이기 위해, 바퀴 입력은 종방향·측방향 속도만 사용하고 회전속도는 IMU의 z축 자이로에서 사용합니다. Gazebo에서 실제 방위와 지나치게 비슷할 수 있는 IMU 절대 yaw는 사용하지 않습니다. AMCL은 계속 `map→odom`을 담당합니다.

먼저 팀 워크스페이스와 Gazebo Classic이 준비돼 있어야 합니다. `robot_localization`이 `/opt/ros/humble`에 없으면 `sudo apt install ros-humble-robot-localization`을 설치합니다. 이 PC는 `$HOME/.local/opt/robot_localization/extracted`의 사용자 설치도 자동으로 찾습니다. 이미 도메인 76에서 실행 중인 시뮬레이션은 종료하세요. 다음 한 명령은 패키지를 빌드한 뒤 세 모드를 **각 3회** 순차 시험합니다.

```bash
cd /home/kiki/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_ekf_benchmark.sh all --repeats 3
```

짧은 기능 점검은 `./scripts/run_ekf_benchmark.sh baseline --repeats 1`처럼 실행할 수 있지만, 한 번의 결과로 모드를 채택하지 않습니다. 실행 결과는 `tmp/ekf_benchmark/<시각>/`에 남습니다. 각 시험의 `.csv`는 정합된 실제·추정 위치와 오차, `.json`은 위치·방위 RMSE, 95백분위 오차, TF 공백·중복, 경로 진행률, 전체 실제 이동거리, 표본 수신율·비율, 헤더 지연, 벽시계 시간과 실시간 비율을 포함합니다. `summary.md`와 `summary.json`은 세 모드의 중위값과 유효 시험 수를 비교합니다. 완료 표시만으로 성공으로 세지 않고 최소 100표본·5 Hz·40% coverage·끝부분 신선도도 요구합니다. 실패한 시험은 오차 0으로 처리하지 않습니다.

시험 전용 월드에는 Gazebo의 `libgazebo_ros_state.so`를 넣습니다. 기록기는 `/gazebo/get_entity_state` 응답에 포함된 **시뮬레이션 시각이 찍힌 실제 위치**를 받습니다. 이는 `/odom`, `/imu`, AMCL 지도를 참조하지 않습니다. Gazebo 서비스 시각은 10 Hz `/clock` 수신보다 최대 한 주기 앞설 수 있으므로 시각이 최대 0.12초 앞선 값도 수용하지만, 오차 계산에는 반드시 정확한 stamp를 사용합니다. `/odom`의 시각이 실제 위치 응답 시각을 양쪽에서 감싸는 두 메시지 사이에 있을 때만 위치·방위를 보간하며, 외삽은 하지 않습니다. 보간 시각 간격도 CSV와 JSON에 남기고 너무 벌어지면 해당 시험을 유효하다고 보지 않습니다. 시작 시 오도메트리 좌표계의 임의 원점만 실제 위치에 정렬하고 그다음 발생하는 누적 오차를 잽니다. 가로·세로 0.55 m 정사각형의 각 구간 종료는 Gazebo 실제 위치로 판단하므로, 이 결과는 **로컬 위치 추정 성능**을 평가합니다. 표의 `route_completion_fraction`은 이 시험 경로를 완주했는지 뜻하며, 팀 A* 계획이나 Nav2 임무 성공을 뜻하지 않습니다. `/tf`의 DDS 발행자 목록만으로 실제 TF 메시지 소유자를 단정할 수 없으므로 `/odom` 발행자 수, TF 프레임의 중복 stamp·발행률·공백을 함께 확인하지만, 임의의 제3자 TF 충돌을 완전히 증명하지는 못합니다.

A* 임무 성능은 별도로 확인합니다. 첫 터미널에서 `./scripts/run_sim.sh localization_mode:=baseline gui:=false use_rviz:=false`, 두 번째 터미널에서 `./scripts/check_sim.sh --goal-x -1 --goal-y -2`를 실행합니다. 두 프로세스를 종료한 다음 첫 터미널의 `localization_mode`를 `ekf_independent`로 바꾸고 같은 목표를 다시 시험합니다. `ekf_overlay`는 주행 오도메트리와 TF를 바꾸지 않으므로 이 모드의 A* 성공은 EKF 후보의 주행 효과가 아닙니다.

`summary.json`의 추천은 **시뮬레이션 후보**일 뿐입니다. 기존 Gazebo 오도메트리는 물리 엔진의 실제 위치와 지나치게 비슷할 수 있어, 실차 바퀴 미끄럼·바닥 편차를 대변하지 않습니다. 실차 적용 전에는 단일 TF 발행, `/odom`·`/imu` 신선도, 안전한 짧은 경로의 반복 완주와 외부 기준 위치 오차를 별도로 검증해야 합니다. 어느 단계에서든 TF 중복·수신 공백·미완주가 나오면 기본 모드를 유지합니다.

[2026-10-28 실측 결과와 채택 이유](ekf_results_2026-10-28.md)를 보관했습니다.
