# 실차 실패를 반영한 Gazebo 검증

Gazebo 완주는 실차 완주를 보장하지 않습니다. 교실의 벽·ArUco만 복제하지 않고,
실차에서 기록된 입력 속도와 시작 절차를 맞춘 뒤 지연과 구동 오차를 나눠 시험합니다.
운용 명령은 [팀원 퀵스타트](TEAM_QUICKSTART.md), 정책은
[주행 정책](NAVIGATION_POLICY.md)을 따릅니다.

## 기존 시험에서 빠졌던 조건

| 실차 기록 / 확인 사항 | 기존 Gazebo의 차이 | 보강할 시험 |
|---|---|---|
| 약 300초 동안 scan 3,075개, IMU 6,099개, joint 6,098개 | 최신 복사 모델 scan 5Hz·IMU 200Hz·joint 30Hz로 서로 다름 | 복사 모델에서 10/20/20Hz 생산 |
| 첫 시작에 AMCL 초기 위치·map→base TF가 없을 수 있음 | 실제 spawn과 같은 초기 위치를 미리 제공 | 초기 위치 없이 시작→웹 Home 초기 위치 지정→pre-arm |
| EKF 갱신 초과 로그에 약 0.10~0.34초 지연 | 독립 EKF 시험의 기존 relay는 scan-only | 실제 EKF 출력 odom·동적 TF의 전달 지연·유실 주입 |
| 실차 선회 오버슛·경로 이탈 | 동일한 두 휠과 빠른 구동 응답 | 휠 접지 반경 비대칭과 느린 구동 응답을 각각 분리 |
| 배터리·OpenCR·USB·모터 통신 단절 | 모의 배터리와 정상 드라이버 | 시뮬레이션 범위 밖. 실차 별도 확인 필요 |

센서 주기 근거는
[`opencr_soak_steady_1e76932.json`](../benchmarks/physical_sector2_20261028/opencr_soak_steady_1e76932.json),
EKF 지연 근거는
[원본 로그에서 추출한 갱신 초과 기록](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/recorded_ekf_overruns.json)입니다.
출력 전달 지연은 EKF 자체의 CPU 과부하 재현과 구분합니다. 휠 비대칭·가속 응답의
스트레스 계수도 실차에서 측정·보정한 값으로 표현하지 않습니다.
후방 영상의 해상도·주기는 맞추지만 실제 렌즈 오염·초점·조명·모션 블러나 가벽 재질의
반사 특성까지 재현한 것은 아닙니다. 시뮬레이션 표식 확인이 실차 카메라 검출 성공을
증명하지 않으며, 해당 구간은 실차 원본 영상과 함께 따로 확인해야 합니다.

## 판정 방법

시험은 domain 176·localhost에서 격리합니다. 실차 domain 16을 사용하지 않습니다.
기억은 켜고 scan filter는 끕니다. 실차 주행 YAML·22cm 차체·기존 충돌 검사는 유지합니다.
아래 `--memory-session`은 같은 시험 프로세스 안의 여러 경로에서 기억을 유지합니다.
프로세스를 재시작한 뒤 과거 디스크 기억을 복원하는 시험과는 구분하며,
실차의 기억 설정을 끄지 않습니다.

| 구분 | 확인할 근거 |
|---|---|
| 시작 | 초기 위치 없는 상태 확인, 실제 웹 초기 위치 응답, 정지 확인, 비상정지 해제, 실제 pre-arm |
| 임무 | 재고 시험(Stage 25·19)은 요청·적재 확인부터, 운용자 경로 시험은 경로 설정부터 출발. 이후 표식 확인·5초 대기·다음 섹터·Home 복귀 |
| 장애물 | 원래 경로를 막는 실제 상자, 실제 우회, 최소 2cm 차체 여유, 제거 전 통과 |
| 지연 | 원본 stamp 유지, scan·odom·TF 각각 주입 확인, 지연 중 상태·회복 후 같은 목표 확인 |
| 도착 | 웹 완료 상태와 독립 Gazebo 실제 위치·방향·정지 확인. 실제 위치 허용 25cm 유지 |
| 후진 | BackUp 요청 수와 실제 음수 속도·실제 이동 거리를 따로 기록. 요청만으로 성공 처리하지 않음 |

충돌 없이 멈췄지만 임무를 끝내지 못했다면 **안전 정지 / 임무 미완료**입니다.
장애물을 치운 뒤에만 완주했다면 **장애물 우회 성공**으로 기록하지 않습니다.
입력 부재·충돌 검사를 지워 완주율을 올리는 방식은 사용하지 않습니다.

## 이번에 재현한 소프트웨어 문제

LiDAR 가드의 일시 정지 이력이 새 국소 경로 수신까지 막아, 계획기는 새 경로를
소유하지만 추종기는 이전 경로에서 멈추는 인계 결함을 수정했습니다. 실제 실행 실패
진단이 남아 있으면 차단을 유지하고, 가드로 진단이 폐기된 경우에는 새 경로를 보관하되
이전 액션의 실제 종료와 기존 정지·선회 검증 이후에 실행합니다.
[원본 94→93점 경로 순서와 수정 검증](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/independent_wheel_guard_handoff_audit.json)을
보존했습니다. 최종 Python 2,948개 검사와 C++ 검사를 통과했으며, 같은 소스의
실제 웹 임무 폐루프 결과는 아래에서 별도로 판정합니다.

같은 장애물 장면에서도 경로가 반복 취소되는 실행을 보존했습니다. 첫 구간의 통과
증명을 유지하지 못한 경우와, 최신 raw scan의 획득 시각이 TF보다 34~121ms 앞서
증명을 취소한 경우를 구분했습니다. 실차의 원인으로 확정한 것은 아니며,
시뮬레이션에서 실제 발생한 실패입니다.

수정본은 동일한 **원본 stamp**의 raw·투영 관측 중 획득 TF가 준비된 가장 새 쌍을
사용합니다. 원본 시각과 양쪽 최초 수신 시각 모두 250ms 제한을 유지합니다.
증명이 아직 준비되지 않았으면 같은 목표·국소 우회 단계를 유지하고 입력을 기다립니다.
최신 실제 장애물·기억·정적 지도 충돌과 최종 raw 근접 정지는 별도로 계속 검사합니다.
과거 시각을 현재 시각으로 바꾸거나, 충돌 검사를 지워서 통과시키지 않습니다.
[원본 실패와 수정 경계 검토](../benchmarks/driving_parameter_tuning_20261104/planner_start_cancel_audit_20261105/coherent_scan_fix_review.json)를
함께 보존했습니다. 이 수정 이후의 폐루프 결과는 아래에서 별도로 표시합니다.

냉기동 검사 도구의 AMCL 파라미터 조회 응답이 유실된 실행도 실패로 보존합니다.
조회는 기존 75초 예산 안에서 최대 3회만 재시도하며, 초기 위치 지정이나 출발 명령은
재시도하지 않습니다. 각 조회 결과를 기록하고, 초기 위치 사전 설정이 없었다는 실제
`false` 응답을 받아야 다음 단계로 갑니다.
[조회 전송 실패 분석](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/independent_cold_parameter_transport_audit.json)에서
Nav2 lifecycle 중단과 단순 읽기 응답 유실을 구분합니다.

## 현재 결과를 읽는 법

최종 공통 장면에는 raw scan이 43ms로 최신인데 투영 관측의 원본 시각은 545ms
오래된 표본도 있었습니다. 투영 상태는 `VALID`였지만 일부 전달 표본이 332~402ms로,
250ms 구간 증명 조건을 넘었습니다. 이때는 잘못된 빔이나 TF 예외가 아니라 일치하는
신선한 입력 쌍을 기다렸습니다. 표본 기록만으로 모든 중간 tick의 연속 실패를 단정하지
않습니다. 이후에는 기억·실측 셀의 출발점 팽창 차단이 남았습니다.
[최종 실행 독립 감사](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/independent_final_handoff_matrix_audit.json)에
두 원인과 경로 인계 결과를 분리합니다. 조건을 완화해 시험을 통과시키지 않았습니다.

기존 센서 주기·초기 위치 사전 제공 조건에서의 완주와 보강 조건의 결과를 별도 표로
남깁니다. 아래 이력의 이전 PASS는 최신 관측 쌍 수정본의 PASS를 뜻하지 않습니다. 시험 전후 소스 해시와 복사 모델·실제 사용 YAML을 함께 보존합니다.
브라우저 화면의 시각적 클릭 검증과 실제 백엔드 API·ROS·물리 시뮬레이션 검증도 구분합니다.

### 최종 주행 소스의 새 실행

아래는 경로 인계 수정 후 **같은 주행 런타임**으로 새로 실행한 결과입니다.
단위 검사 통과와 폐루프 완주를 구분합니다. 각 실행의 소스 전후 해시는 동일합니다.
재고 시험과 운용자 경로 시험은 별도 절차입니다.

| 시험 | 전체 판정 | 확인된 결과 / 남은 문제 |
|---|---|---|
| 재고 Sector 3 이동 장애물, 10/20/20Hz | **실패** | 상자 4/4회 이동, 제거 전 우회 못함. 제거 후 섹터·Home 복귀·정지 통과 |
| 같은 장면, 바퀴 비대칭 | **실패** | 상자 2/4회 이동, 제거 전 우회 못함. 제거 후 Home 복귀·정지 통과 |
| 같은 세션 재고 Sector 2 왕복 두 번 | **통과** | 실제 pre-arm 두 번·정지·재초기화·명시적 해제. 실제 Home 오차 15.25/16.00cm |
| 악조건 월드 1→2→3→Home | **통과** | 212.689초. 각 표식·5초 대기·최종 정지. 실제 Home 오차 11.41cm |
| 악조건 월드 1→3→2→Home | **중도 정지 / 실패** | Sector 3 접근에서 입력 일치 대기와 출발점 NoPath가 반복. 약 6분 이상 정체 후 운용자 일시정지, Home 미완료 |
| EKF 실제 출력 전달 지연·유실 첫 실행 | **주행 전 실패** | 데이터는 정상이나 DDS 발행자 이름 미발견 상태로 pre-arm graph 검사가 조기 종결. 지연 주입 미실행 |
| 느린 실제 휠 응답, 이동 상자 | **실패** | 실제 상자 여유 1.77cm로 2cm QA 미달. 상자 제거 후 Home. 취소 대기 중 관성 이동도 별도 확인 |
| 발행자 확인 대기 수정 후 EKF 전달 지연 재시험 | **통과** | 134.809초. 출력 지연 최대 약 0.361초·유실 scan/odom/TF 7/16/16건. 같은 목표 재개·실제 Home 오차 15.17cm |

재계획 횟수 증가는 관측·위치 변화가 대기를 깨우는 현상과 함께 기록합니다. 같은 입력을
무조건 반복했다고 단정하지 않습니다. DDS 발행자 검사에는 기존 4초 예산 안에서
실제 EKF·릴레이 이름이 준비될 때까지 기다리는 **읽기 전용 검사 수정**을 적용했습니다.
알 수 없는 발행자·다른 발행자·중복 발행자는 계속 거부하며 주행 기준은 바꾸지 않았습니다.
원본 시작 실패는 보존하고 수정 후 전달 지연 시험은 별도 실행으로 기록합니다.
[최종 8개 실행 인덱스](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/final_matrix.json)와
[통합 경로 독립 감사](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/independent_final_integrated_matrix_audit.json)를
함께 보존합니다. 이 작은 표본에서 실차 성공률을 추정하지 않습니다.

느린 구동 응답 실행의 원래 정지 검사는 `WAIT_REPLAN` 구간만 보아 통과했습니다.
하지만 실제 `WAIT_CANCEL / WAIT_STOP` 진입부터 정상 Spin 전까지 약 **14.42cm**
이동했으며 기존 속도·감속도 기반 허용치 약 **11.55cm**를 초과했습니다.
0속도 명령 뒤 첫 truth 표본부터만 계산해도 11.87cm의 관성 이동이 남았습니다.
이는 구동 지연 민감도 시험 결과이며 실차 제동 거리를 측정한 값은 아닙니다.
원래 판정을 덮어쓰지 않고 취소 대기 구간을 포함한 추가 정지 검토를 별도 보존합니다.
이제 같은 정지 기준이 현재 목표·경로에 속한 취소 대기에도 적용됩니다. 반복 대기나
종료 응답으로 최초 정지 시각을 새로 잡지 않으며, 이전 목표·경로·후진 episode의
늦은 메시지로 정상 기동 면제를 주지 않습니다. 소유권이 불명확하면 미확인으로 남깁니다.
[8개 원본 로그의 추가 정지 재평가](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/cancel_stop_reassessment.json)에서
느린 구동 실행의 원래 정지 PASS가 FAIL로 바뀌었고, 전체 통과 3개는 유지됐습니다.
이는 같은 로그의 검사 범위를 보완한 재평가이며 새 폐루프 실행은 아닙니다.

### 수정 과정의 이전 실행

| 2026-11-05 실제 폐루프 시험 | 결과 |
|---|---|
| 초기 위치 없음·10/20/20Hz·재고 Sector 3 이동 장애물→Home, 수정 전 | **실패**. 초기화·Home·약 11.8cm 실제 후진은 성공했지만 제거 전 우회 미확인, 정지 중 실제 회전 0.1243rad로 QA 초과 |
| 같은 조건·같은 상자 장면, 기억 기하·이동 선회 후보 수정 후 | **통과·119.170초**. 상자 4/4회 이동·제거 전 우회·최소 실제 여유 9.66cm, Home 실제 오차 21.8cm·최종 정지 통과 |
| EKF 출력 전달 지연 시험의 첫 실행 | **시작 실패**. lifecycle get_state 응답 시간 초과→bringup 중단. 지도·AMCL이 활성화되지 않아 실제 지연 주입 전 종료. `TRANSIENT_NOT_STARTED`로 구분 |
| 실제 EKF 출력 scan·odom·TF 전달 지연·유실, 1→2→Home | **통과**. 최대 약 0.364초 입력 지연, 유실 7/16/16건. 안전 정지 후 같은 목표로 자동 재개·Home 정지 확인 |
| 초기 위치 없음·10/20/20Hz·악조건 월드 1→2→3→Home | **통과**. 실제 pre-arm·후방 표식·5초 대기·각 도착·고정 상자 여유·최종 정지 확인 |
| 느린 휠 응답 0.15m/s², 재고 Sector 3 이동 장애물→Home | **실패**. 임무·상자 4회 이동·제거 전 우회·Home은 완료했으나 정지거리·정지각 QA 초과 |
| 휠 비대칭 시험의 첫 실행 | **장애물 시험 무효 / 전체 실패**. 주행·Home·정지 통과, 상자 위치 사전 검증 거부로 실제 장애물 주입 없음 |
| 공통 상자 장면의 첫 센서 주기 대조군 | **장애물 시험 무효 / 전체 실패**. 주행·Home·정지 통과, 최종 주입 시점 선정 범위 이탈. 무장애물 완주를 우회 합격으로 처리하지 않음 |
| 공통 장면·최종 주입 검사 수정 후 | **실패**. 실제 상자 4회 이동했으나 경로 취소가 반복되어 Home 미완료 |
| 첫 구간 증명 소유권 수정 후 | **실패**. Home 완료, 상자 1/4회 이동·제거 전 우회 미확인. 획득 시각 TF 미래 예외가 남음 |
| 원본 관측 쌍 수정 후 첫 실행 | **주행 전 실패**. AMCL은 활성화됐으나 읽기 전용 파라미터 응답 전달 실패로 초기화 검증 시작 못함 |
| 원본 관측 쌍 수정 후 공통 장면 | **실패**. 실제 후진 두 번·Home 완료, 제거 전 우회 미확인. 이후 새 실행은 위 표에 분리 |

[수정 전 원본](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/cold_rates_before_memory_fix/summary.json)과
[수정 후 원본](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/cold_rates_after_memory_turn_fix/summary.json)을
따로 보존했습니다. 수정 후에도 실제 도착 기준 25cm·상자 여유 2cm·센서/제어권 검사는
유지했습니다. 실제 최종 spawn SDF도 보존해 IMU20Hz·LiDAR10Hz·후방5Hz·차체22cm를 확인합니다.

첫 시험의 실제 source stamp 주기는 약 9.98/19.93/20Hz, wall 수신은 약
8.62/17.27/17.33Hz였습니다. Gazebo 시간이 실시간보다 느려 원본 stamp 기준 주기와
wall 전달 지연은 같은 수치가 아닙니다. 원본 로그의 EKF CPU 지연을 완전히 재현했다고
표현하지 않습니다. **수정본 실차 재시험은 아직 하지 않았습니다.**

## 아직 해결하지 않은 좁은 경로

공통 상자 장면에서는 5cm 격자가 실제로 여유 있는 연속 경로를 표현하지 못한 경우도
재현했습니다. 당시 지도상 차체 여유는 약 2.48cm였지만 주변 후보 셀이 모두 막혔습니다.
기존 경로를 그대로 허용하면 일부 구간 여유가 0.46mm뿐이어서, 그 경로를 해제하지 않았습니다.
원본 장애물 사각형·22cm 차체를 유지한 1cm 후보 검색은 전체 경로를 찾았으나,
짧은 재합류 경로의 예비 구현도 전처리부터 기존 60ms 예산을 넘었습니다.
**이 예비 구현은 실행 코드에 채택하지 않았고, 해당 정체가 해결됐다고 기록하지 않습니다.**
[기하 대안·시간 측정·재현 도구](../benchmarks/driving_parameter_tuning_20261104/fine_geometry_feasibility_20261105/README.md)를
보존했습니다. 당시 AMCL과 Gazebo 실제 위치의 약 5.4cm 차이도 있어, 지도상 경로 발견만으로
실차의 좁은 구간 통과를 보장할 수 없습니다.

다른 정체에서는 실제 LiDAR 반환점이 차체보다 약 4cm 밖에 있었지만, 관측점을 포함하는
5cm 셀 전체를 장애물로 표현하면 차체 모델과 약 4mm 겹쳤습니다. 이는 없는 장애물이라고
단정할 수 없으며, 기존 장애물 셀을 그대로 둔 채 검색 격자만 세분화해도 해결되지 않습니다.
[원본 scan·표본 TF·셀 기하 대조](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/independent_final_common_recorded_audit.json)는
기록 간 시각 차이와 TF 표본 한계까지 포함합니다. 이 장면을 해결된 항목으로 표시하지 않습니다.

활성 목표의 장애물·입력 대기 전이에도 incident 기록을 남기도록 보완했습니다.
이는 제한된 주기의 전후 기록이며 rosbag 전체 기록은 아닙니다. 실차에서 새 관측점의
정확한 원본 scan·획득 시각 TF까지 대조해야 할 때는 아래 기록을 별도 PC 터미널에서
함께 실행하고 시험 후 `Ctrl+C`로 마칩니다. 기록 자체는 주행 명령을 보내지 않습니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
source ./scripts/dds_env.bash ""
mkdir -p ~/smart_vision_logs
ros2 bag record -o "$HOME/smart_vision_logs/drive_$(date +%Y%m%d-%H%M%S)" \
  /scan /odom /imu /tf /tf_static \
  /smart_vision_amr/path /smart_vision_amr/planner_status \
  /smart_vision_amr/navigation_status /smart_vision_amr/motion_status \
  /smart_vision_amr/observed_obstacles /smart_vision_amr/remembered_cells
```

## 재현 명령

PC 설치와 [Gazebo 의존성 설치](GAZEBO.md)를 먼저 완료합니다. 아래 자동 시험은
Gazebo·주행·웹을 직접 켜고 종료하므로 다른 시뮬레이션·8792 웹을 중복 실행하지 않습니다.
로봇 SSH나 `turtlebot16`은 실행하지 않습니다.

**PC 터미널 1 · 초기 위치 없는 재고·장애물 시험**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
python3 scripts/run_classroom_staged_validation.py \
  --run 25_web_inventory_sector3_moving_obstacle \
  --physical-footprint --memory-session --cold-start \
  --hardware-profile sensor_rates \
  --nav-params-source ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml \
  --nodes-params-source ros2_ws/src/smart_vision_amr/config/classroom_hardware_nodes.yaml \
  --obstacle-scenario benchmarks/driving_parameter_tuning_20261104/scenarios/baseline_sector3_box.json \
  --leg-timeout 240 --obstacle-hold-timeout 0 \
  --output "/tmp/classroom-physical-transfer-$(date +%Y%m%d-%H%M%S)"
```

**브라우저 · 시험 관찰**

시험 중 [운용 웹](http://127.0.0.1:8792/)과
[재고 웹](http://127.0.0.1:8792/inventory)을 엽니다. 자동 시험이 재고 요청부터
초기 위치·출발까지 조작하므로 관찰 중에는 추가 임무·초기 위치·수동 명령을 넣지 않습니다.
이 자동 시험의 ROS 도메인은 **176**입니다. 실차 웹 **8790**과 구분합니다.

같은 명령에서 시험·프로필을 바꿔 원인을 분리합니다. 매번 새로운 output 경로를 씁니다.
**이동 장애물 시험이 아닌 경우 `--obstacle-scenario …` 줄을 삭제합니다.**

| 바꿀 항목 | 값 |
|---|---|
| 1→2→3→Home, 악조건 월드 | `--run 17_web_hardware_value_parity` |
| 1→3→2→Home, 악조건 월드 | `--run 12_web_integrated_stress_permuted` |
| 동일 세션 재고 Sector 2 왕복 두 번 | `--run 19_web_inventory_sector2_same_session_reentry` |
| 실제 EKF 출력 전달 지연·유실 | `--run 16_web_transient_resume --ekf-delivery-faults` |
| 휠 접지 반경 비대칭 | `--hardware-profile wheel_asymmetry` |
| 느린 실제 휠 응답 | `--hardware-profile actuator_lag` |

물리 오차만 비교할 때는 `nominal` 대신 같은 센서 주기를 쓰는 `sensor_rates`를 대조군으로
사용합니다. `nominal`→`sensor_rates` 비교는 센서 주기 효과를 따로 확인합니다.

비대칭 프로필은 좌·우 접지 반경을 32.67/33.33mm로, 느린 응답은 휠 가속 한계를
0.15m/s²로 바꿉니다. 두 프로필 모두 센서 생산은 10/20/20Hz이고, 명목 휠·EKF 보정값은
그대로입니다. 이 수치는 측정된 실차 계수가 아닌 민감도 시험값입니다.

이전 소스의 `actuator_lag` 폐루프에서는 임무·Home 복귀와 상자 4회 이동·제거 전 우회를
완료했지만, **전체 판정은 실패**입니다. 정지 명령 뒤 실제 이동 7.0cm와 회전
0.2059/0.2389rad가 기존 정지 QA를 초과했습니다. 임무 완료를 전체 안전 검증 통과로
바꾸지 않았으며 QA 기준도 완화하지 않았습니다.
[시험 결과](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/cold_actuator_lag/summary.json)와
[정지 구간 분석](../benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/independent_actuator_stop_dynamics_audit.json)을 보존했습니다.

해당 시험의 DWB 가속도는 이미 선형 0.20m/s²·각 0.80rad/s², 감속도 크기는
0.40m/s²·1.60rad/s²였습니다. 실제 시뮬레이션 명령 경로는 Nav2→motion_mux→Gazebo
구동기이며, `velocity_smoother`는 YAML 설정만 있고 실행 노드는 없습니다.
mux의 가속 제한도 0.20/0.80이지만, 이 스트레스 모델의 **각 바퀴 제동 능력은
0.15m/s²**입니다. 따라서 같은 가속도 값을 다시 적용하는 것으로 해결됐다고 볼 수 없습니다.

좌·우 바퀴가 함께 감속하면 이동과 회전의 제동이 결합됩니다. 실제 측정 속도로 각 바퀴의
정지거리를 계산하면 최종 명령이 0이 된 뒤에도 약 0.15/0.21rad의 회전이 예상되어,
선형·각 감속을 독립적으로 계산한 약 0.03rad보다 큽니다. 세 실패 구간은 이동 중 선회·제동이며,
순수 제자리 회전 0.5rad/s 예외가 원인이라는 근거는 없었습니다. 명령 전달 시간과 실제 구동
응답까지 맞춰야 하므로, 다음 실차 검증에서는 여러 직진·선회 속도에서 좌·우 휠 응답과
정지거리·정지각을 측정해 제어기의 동역학 예측을 보정해야 합니다. 이 미보정 민감도 시험의
실패를 실차 계수나 실차 검증 완료로 해석하지 않습니다.
