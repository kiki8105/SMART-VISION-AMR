# 동일 장애물 장면으로 단일 인자 비교

기존 adaptive actor는 주행 경로에 따라 상자 좌표와 이동 방향을 다시 선택합니다. 이전 gain 2.2 시험의 stage 25 통과는 보존하되, 기준 시험과 상자 장면이 달랐으므로 **gain 변경의 인과적 개선 증거로 사용하지 않습니다.**

새 `--obstacle-scenario` 옵션은 같은 상자 위치·방향·4회 이동 좌표를 고정합니다.
고정 장면 비교에서는 복구 실패가 재현됐으며, 아직 최적 파라미터를 선정하지 않았습니다.

## 고정 입력

[baseline_sector3_box.json](scenarios/baseline_sector3_box.json)

| 항목 | 값 |
|---|---|
| 상자 중심 | `(0.5239140050447512, -0.599706838892526)` m |
| 기준 접선 | `(0.6954214275160555, -0.7186021417665909)` |
| 이동 방향 | `-1` |
| 상자 크기 | `0.25 × 0.25 × 0.5` m |
| 이동 | 직교 방향으로 2.5cm씩 4회, 총 10cm |
| 입력 SHA-256 | `a57b9ab553ab04c02e4af9559531d22d3d3d0208b65ab98b5d92258cc28ec75e` |

최종 상자 중심은 `(0.45205379086809205, -0.6692489816441315)` m입니다. 모든 시험에서 동일한 JSON 바이트를 사용합니다.

## 기존 검사 유지

- 웹이 실제 주행 목표를 소유합니다. actor는 자체 Gazebo 상자만 생성·이동·삭제합니다.
- navigation 상태, 현재 goal/path, LiDAR 관측, memory 및 Gazebo truth를 기다리는 기존 준비 절차는 유지됩니다. 준비되지 않은 순간을 입력 기하 실패로 분류하지 않습니다.
- 신선한 현재 경로가 상자와 차체 기준 1cm 이상 겹쳐 실제로 막히는지 확인합니다. 전방 배치·거리 0.6~1.0m, 목적지 거리, 정적 벽 거리 및 기존 관측과의 겹침 검사도 유지합니다.
- 최초 위치와 이후 4회 위치 각각에서 기존 실제 크기·A* 팽창·현재 관측·기억을 사용해 우회 가능성을 증명합니다.
- 생성 직전 신선한 truth/route/관측·memory 변화·서비스 준비 상태를 다시 확인합니다. 이후 이동마다 실제 로봇과 상자의 전체 이동 sweep 여유를 확인하고 필요하면 이동을 보류합니다.
- 경로 반응, 살아 있는 상자 우회, 4회 이동 완료, 차체 최소 2cm 여유, 빨간 관측 셀과 제거 후 해제 QA를 동일하게 요구합니다.
- 고정 상자가 후보 경로를 막지 않거나 우회 증명이 실패하면 **`INVALID_INPUT` / `passed=false`**로 기록합니다. 상자를 옮겨 놓거나 반대 이동 방향을 탐색해 시험을 통과시키지 않습니다.

복사한 원본 JSON, SHA-256, 5개 상자 위치는 `staged_report.json`과 `web_obstacle_injection.json`에 보존합니다. derived stage verdict에서도 요청한 입력과 actor 입력의 SHA 일치를 요구합니다. 생성 시점은 기존 준비·실제 출발 검사에 따르고, 상자 이동 시점도 경로 반응과 여유 조건에 따릅니다. **두 시험의 절대 시각이나 센서 관측이 동일하다는 뜻은 아닙니다.**

## 실행 예시

Gazebo 및 ROS domain 176이 비어 있을 때 저장소 루트에서 순서대로 실행합니다. 실제 로봇 domain 16에는 연결하지 않습니다. 두 시험에서 Nav2 YAML, memory, 차체, 감속 여유 및 actor JSON을 동일하게 유지합니다.

기준 설정:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_staged_validation.py \
  --run 25_web_inventory_sector3_moving_obstacle \
  --physical-footprint --memory-session \
  --leg-timeout 240 --obstacle-hold-timeout 0 \
  --nav-params-source ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml \
  --nodes-params-source ros2_ws/src/smart_vision_amr/config/classroom_hardware_nodes.yaml \
  --obstacle-scenario benchmarks/driving_parameter_tuning_20261104/scenarios/baseline_sector3_box.json \
  --output /tmp/classroom-fixed-baseline
```

A* 비용 계수만 `1.5→2.2`로 변경:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_staged_validation.py \
  --run 25_web_inventory_sector3_moving_obstacle \
  --physical-footprint --memory-session \
  --leg-timeout 240 --obstacle-hold-timeout 0 \
  --nav-params-source ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml \
  --nodes-params-source benchmarks/driving_parameter_tuning_20261104/profiles/nodes_gain_22.yaml \
  --obstacle-scenario benchmarks/driving_parameter_tuning_20261104/scenarios/baseline_sector3_box.json \
  --output /tmp/classroom-fixed-gain22
```

출력 폴더를 덮어쓰지 않습니다. 이미 존재하면 새 경로를 사용하세요. 별도 margin .25 또는 .35 후보와 비교할 때도 **선택한 한 인자만** 바꾸고 같은 actor JSON을 사용합니다. 조합 프로필은 각 단일 인자 결과를 본 다음 시험합니다.

## 현재 증거 범위

| 검사 | 결과 | 범위 |
|---|---|---|
| injector/staged 관련 회귀 | 90 PASS | 기본 adaptive 유지, 고정 좌표·side·5위치 증명, 미차단 거부, 옵션 전달, 입력 hash 검증 |
| 녹화 기준 경로 + 고정 상자 | 오프라인 기하 증명 | 반경 .22m / 5셀, 5위치 우회 `[82,82,83,84,84]`셀, 기존 경로와 상자 겹침 약 9.68cm |
| 고정 장면 기준 폐루프 | 실패 | 아래 공통 코드 비교표 참고 |
| 고정 장면 gain 2.2 폐루프 | 최초 입력 무효 / 재시험 실패 | 무효 입력은 주행 성능 실패와 구분 |

## 후진 우선순위 수정 전 공통 코드 비교

[원본 요약·설정 및 소스 해시](fixed_scene_before_reverse_priority.json)를 보존했습니다.
한 인자 비교에서 inflation은 local/global costmap의 같은 총 반경을 함께 변경했습니다.
모든 실행이 pre-arm과 Sector 3→Home을 마쳤지만, 살아 있는 상자를 우회하는 전체 조건은 통과하지 못했습니다.
상자를 제거한 뒤 Home에 도착한 사실만으로 장애물 시험을 합격 처리하지 않습니다.

| 설정 | 입력 | 상자 이동 | 제거 전 우회 | 최소 실제 여유 | Home | 전체 판정 |
|---|---|---:|---|---:|---|---|
| 기준 inflation .22 / gain 1.5 | 유효 | 1/4 | 실패 | 6.44cm | 완료 | 실패 |
| inflation만 .30 | 유효 | 4/4 | 실패 | 5.99cm | 완료 | 실패 |
| gain만 2.2 · 최초 | `INVALID_INPUT` | 미설치 | 평가 불가 | — | 완료 | 입력 무효 |
| gain만 2.2 · 재시험 | 유효 | 3/4 | 실패 | 7.23cm | 완료 | 실패 |
| inflation만 .40 | `INVALID_INPUT` | 미설치 | 평가 불가 | — | 완료 | 입력 무효 |

무효 원인은 `FIXED_SCENE_TOO_CLOSE_TO_OBSERVED_CELL`입니다. 고정 장면의 배치 검사와
실시간 관측이 맞지 않아 상자를 설치하지 않았으며, 장애물 주행 성능의 근거로 사용하지 않습니다.
유효 실행의 raw-scan guard 정지 표본은 모두 0초였습니다. 기준·.30 실행은 기존 정지 회전량
한도도 초과했습니다. 회전량 판정을 완화하지 않았습니다.

.30 실행은 후방과 전체 후진 궤적이 실제 크기 기준으로 비어 있어도,
가까운 벽과의 거리가 매 순간 증가해야 한다는 휴리스틱 및 가상 깊이 reserve 때문에
후진을 선택하지 못했습니다. [기록 기반 감사](recovery_audit/README.md)는 실제 관측과
합성 직선 주행 이력을 구분합니다. 실제 breadcrumb는 녹화되지 않아 이 감사 자체가
실제 후진 성공을 증명하지는 않습니다. 후속 국소 우회→후진→전체 재계획 수정 결과는
다른 소스 해시의 별도 실행으로 비교해야 합니다.
