# 교실 주행 고장 주입 시뮬레이션

`scripts/run_classroom_fault_sim.sh`는 실차 도메인 16을 사용하지 않습니다. ROS 도메인 176, 로컬 DDS, 별도 Gazebo 마스터에서 2026-10-30 교실 지도·월드의 복사본을 실행합니다. 센서 메시지는 Gazebo가 실제로 생성하며, 중계 노드는 원본 내용과 시각을 바꾸지 않고 전달 시각이나 유실만 재현합니다. 시험 뒤 Gazebo와 중계 노드를 종료합니다.

아래 **ROS 직접 목표 진단**을 화면 없이 실행하면서 수동 관제하려면 웹을 별도 터미널에서 실행할 수 있습니다. PC·휴대폰에서 [시뮬레이션 관제](http://192.168.0.110:8791/)를 열어 로봇 위치, 전·후방 카메라, 정리된 지도/원본 지도와 계획 경로를 확인합니다. 접속 IP는 실행 PC의 `hostname -I` 값으로 바꿉니다. 이 웹은 시험 전용 도메인 176·포트 8791을 쓰므로 실차 웹 8790과 분리됩니다. **웹이 임무를 소유하는 전체 시스템 단계**에는 이 8791 웹을 동시에 실행하지 마세요. [단계별 검증 안내](classroom_staged_validation.md)는 임시 웹 8792를 단계마다 실행·종료하고 8791에는 읽기 전용 관제 프록시만 허용합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
SMART_VISION_CLASSROOM_SIM_DOMAIN=176 \
SMART_VISION_CLASSROOM_SIM_PORT=8791 \
./scripts/run_classroom_replay_web.sh
```

그 다음 터미널에서 아래 시험 명령을 실행합니다. 시험 스크립트가 종료되면 Gazebo도 정리되어 웹의 경로·카메라가 사라집니다. 아래 `--mission-route` 검증기는 ROS 목적지를 직접 보내므로 웹 배송 접수·임무 버튼의 동작까지 검증하는 시험은 아닙니다. 합격 판정은 Gazebo 실측 주행, 후방 기대 ArUco 인식, 안전 간격, 홈 복귀 장애물 반응을 기준으로 합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_fault_sim.sh \
  --mode baseline --profile clean --obstacle none --timeout 180 \
  --output /tmp/classroom-clean-1
```

섹터 1→2→3→홈을 한 번의 임무로 확인할 때는 `--mission-route`를 사용합니다. `--timeout`은 각 구간 제한 시간입니다. 홈 복귀 중 지도에 없는 장애물을 놓고 실제 우회·간격·제거 후 복귀까지 확인하려면 `--home-obstacle`을 함께 사용합니다. 이 옵션은 장애물이 살아 있는 동안 실제 차량이 우회 지점을 지나야 통과하며, 시간이 끝났다는 이유만으로 장애물을 제거해 성공시키지 않습니다.

```bash
./scripts/run_classroom_fault_sim.sh --mission-route --mode baseline \
  --profile clean --timeout 180 --output /tmp/classroom-four-stop-1

./scripts/run_classroom_fault_sim.sh --mission-route --home-obstacle \
  --mode baseline --profile clean --timeout 180 \
  --output /tmp/classroom-home-obstacle-1

# 관측 좌표 한 곳을 전체 임무 전에 단독 검증
./scripts/run_classroom_fault_sim.sh \
  --single-goal Sector2Candidate 1.45 -2.65 1.543282 \
  --mode baseline --profile clean --timeout 120 \
  --output /tmp/classroom-sector2-candidate-1
```

다른 시험은 위 명령을 끝낸 다음 각각 **새로운 `--output` 디렉터리**로 실행합니다. Gazebo를 동시에 여러 개 켜면 CPU 부하가 지연 측정을 왜곡합니다.

시뮬레이션 기본값은 CPU 부하를 줄이기 위해 incident recorder를 끕니다. 실차 런치의 현재 기본값은 recorder 켜짐이므로 지연 비교 때는 `--record-incidents`를 붙인 시뮬레이션도 따로 실행하고 `runtime_manifest.json`의 `record_incidents`를 확인합니다. 끄면 사고 직전 상세 기록을 잃습니다. 순간적인 TF·센서 지연 중에는 안전하게 정지하고 **동일 목표를 유지한 채** 수신 복구·재계획·도착해야 합격이며, 비상정지로 목표가 취소되면 불합격입니다.

```bash
# 0~230ms 지터, 약 1초 센서 단절, 센서 메시지 유실
./scripts/run_classroom_fault_sim.sh --profile mixed --timeout 180 \
  --output /tmp/classroom-mixed-1

# 긴 scan/odom/TF 단절: 목표를 유지하되 안전 정지·수동 재개 필요 상태를 별도 판정
./scripts/run_classroom_fault_sim.sh --profile outages \
  --expect-sustained-outage-hold --timeout 90 \
  --output /tmp/classroom-outage-hold-1

# 경로 앞에 지도에 없는 장애물 생성 → 옆으로 이동 → 제거
./scripts/run_classroom_fault_sim.sh --profile clean --obstacle moving \
  --timeout 180 --output /tmp/classroom-moving-1

# 별도 가혹 월드: 원본 교실 구조 위에 좁은 통로와 장애물을 추가
./scripts/run_classroom_fault_sim.sh --stress --allow-safe-hold \
  --timeout 180 --output /tmp/classroom-stress-1

```

`--mode ekf_independent`는 실차형 휠 적분기·EKF 센서 감시·시작 비상정지를 켭니다. 직접 ROS 목표를 보내는 위 검증기는 시작 비상정지를 웹에서 해제하지 않으므로 이 모드의 전체 시스템 합격 시험으로 쓰지 않습니다. 웹의 명시적 안전 해제, ArUco 중앙 확인, 섹터별 5초 대기와 Home 복귀까지 검사하려면 다음 명령을 사용합니다. 단계마다 임시 웹 8792가 열리고 시험이 끝나면 닫힙니다.

```bash
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 03_web_sector3_clean,06_web_sector3_moving_obstacle,12_web_integrated_clean,12_web_integrated_home_obstacle \
  --output /tmp/classroom-web-full-$(date +%Y%m%d-%H%M%S)
```

위 ROS 직접 목표 진단의 `acceptance.json`은 실제 Gazebo 위치로 목표 도착, 주행 중 안전 정지, 장애물 최소 차체 여유, 경로 재계획 횟수, AMCL 위치 오차를 따로 기록합니다. `sensor_faults.jsonl`은 센서별 전달·지연·유실 수를 기록합니다. 실제 fault가 적용되지 않으면 `fault_delivery`가 불합격으로 바뀝니다. `fixture/fixture_manifest.json`과 `runtime_manifest.json`에 지도·월드·YAML·모델 해시가 남습니다. `--allow-safe-hold`는 의도적으로 통행 불가능한 가혹 월드에서만 사용하며 목표 도달 성공과 구별됩니다. 웹 단계는 별도의 웹 임무 `acceptance.json`과 단계별 `staged_report.json`에서 안전 해제, 마커 중앙 확인, 5초 정지, Home 도착을 판정합니다.

기준 시험 결과: 초기 단독 실행은 약 55초에 `execution_no_progress`로 실패했고, 같은 조건의 180초 제한 재시험은 68.7초에 Sector 3 목표 4.4cm 이내 도착, 물리적 정지를 확인했습니다. 재시험에서도 글로벌 계획 3회, 국소 우회 3회, 빈 경로 발행 2회가 발생해 경로 안정성 개선이 남았습니다. 100개 근접 시각 AMCL/Gazebo 표본의 위치 오차는 중앙 3.9cm, 최대 6.6cm였습니다. 측정 자료는 `/tmp/classroom_fault_clean_diagnostic_20261030/`에 있습니다. mixed 센서 지연·단절 시험은 기능상 목표 도착과 안전 정지 후 회복을 확인했으나, 다른 Gazebo GUI의 CPU 부하가 있었으므로 지연 성능의 기준 수치로 사용하지 않습니다. 기록은 `/tmp/classroom_fault_mixed_20261030/`에 있습니다.

2026-10-30 네 구간 시험은 **아직 불합격**입니다. 섹터 1 후보 위치 `(1.20, 0.55)`가 한 차례 단독 시험에서 29.3초에 도착했으나, 이어 실행한 전체 임무는 섹터 1에서 160초 대기 후 시간 초과했습니다. 지도 경계와 일치하는 라이다 반사가 중복된 동적 장애물로 팽창했고, 일시적으로 `map→base` 위치 변환도 끊겼습니다. 실제 지도 밖 새 장애물과 지도 경계 반사를 안전하게 구분하는 수정과 전체 임무 재시험이 끝날 때까지 Home 장애물 시험·실차 운용 합격으로 간주하지 않습니다. 자료는 `/tmp/classroom_four_stop_fast_20261030/`에 있습니다. moving·stress·독립 EKF 결과도 **아직 합격으로 판정하지 않았습니다**.

2026-11-02 첫 네 구간·홈 장애물 시험도 **불합격**입니다. 섹터 1은 Gazebo 실측 오차 4.1cm로 통과했으나, 섹터 2는 목표 근처에서 저장 지도 벽에 가까운 라이다 반환이 새 장애물로 재팽창해 경로가 사라졌고 방향 정렬을 마치지 못했습니다. 위치 변환 지연으로 안전 정지한 뒤 종료됐으며, 홈 복귀 장애물 투입까지 도달하지 않았습니다. 근거는 `/tmp/classroom_four_stop_home_obstacle_20261102_a/acceptance.json`과 같은 폴더의 `gazebo_launch.log`, `web_observations.jsonl`입니다. 섹터 2의 더 안쪽 관측점 `(1.45, -2.65, 1.543282)` 단독 시험은 55.8초에 실측 위치 오차 3.0cm·방향 오차 0.226rad, 후방 ArUco 2번 중앙 오차 2%로 통과했습니다. 근거는 `/tmp/classroom_sector2_candidate_20261102_a/acceptance.json`입니다. 이 결과는 **네 구간 연속 주행이나 홈 장애물 우회 합격을 뜻하지 않습니다.**

2026-11-02 두 번째 연속 시험도 **불합격**입니다. 섹터 1·2는 실측 도착과 후방 표식 검출을 통과했으나 섹터 3은 목표 좌표에 도달해도 3번 표식이 카메라 시야 밖이었습니다. `(-0.25, -1.98, -1.40)` 단독 후보는 기록 노드를 켠 시험에서 TF 수신 공백이 안전 정지로 이어져 실패했고, 기록 노드를 끈 동일 후보는 69.7초에 목표에 도착해 위치 오차 4.8cm·방향 오차 0.207rad, 후방 3번 마커 검출로 통과했습니다. 기록 노드가 약 50% CPU를 사용한 표본이 있으나 다른 병렬 부하도 달라 단독 원인으로 단정하지 않습니다. 전체 경로, 홈 복귀 중 장애물 우회, 실차 동등성은 아직 미검증입니다. 기록은 `/tmp/classroom_four_stop_home_obstacle_20261102_b/`, `/tmp/classroom_sector3_candidate_20261102_{a,b}/`에 있으며 이 경로는 시험 PC의 임시 자료입니다.

이 시험은 실차의 Wi-Fi, 배터리, OpenCR, 모터, USB 웹캠 드라이버 장애를 재현하지 않습니다. `baseline`은 Gazebo 원래 odom을 사용하고 `ekf_independent`는 Gazebo 바퀴/IMU 입력으로 물리 EKF 설정을 적용하므로, 실차의 센서 품질·주파수와 완전히 같은 조건은 아닙니다. 지도에 없는 장애물의 **빨간 셀**도 HTTP 상태와 SVG 작성 규칙의 자동 검사 자료는 있지만, 실제 브라우저 DOM에서 정리된 지도·원본 지도 양쪽에 표시되고 제거 뒤 사라지는 시각적 검증은 브라우저 접근 정책 때문에 아직 하지 못했습니다. 실차 주행 허가는 별도 센서·전원 확인과 현장 시험으로 결정해야 합니다.
