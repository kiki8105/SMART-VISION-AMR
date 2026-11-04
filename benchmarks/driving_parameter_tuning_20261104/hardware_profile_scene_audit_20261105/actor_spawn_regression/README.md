# 고정 상자 spawn 경계 회귀

이 자료는 **과거 실패 입력에 대한 harness 판정 회귀**입니다. 이후 실행 중인 Gazebo 시험의
통과 결과나 실차 검증으로 해석하지 않습니다. 원래 실패는
[원본 요약](../../physical_transfer_20261105/cold_sensor_common_invalid_spawn/summary.json)과
[actor 기록](../../physical_transfer_20261105/cold_sensor_common_invalid_spawn/25_web_inventory_sector3_moving_obstacle/web_obstacle_injection.json)에 보존돼 있습니다.

## 실제 실패 원인

상자는 `(0.44, -0.60)`에 고정됐고, spawn 전에 이동·회전하거나 재선정되지 않았습니다.
처음 선정한 경로는 원래 25cm 상자를 22cm 차체가 지날 때 최소 여유가 **−15.6365cm**여서
실제 경로를 막았습니다. preflight 동안 경로 발행은 1회뿐이며 stamp는 `9:320000000`입니다.
검색에서 기록한 원본 path SHA는 `b98614957975fa4e2ceaa4767d233621ae86332408dfcdf85e17f29ce4609cb4`입니다.
타임라인 좌표는 반올림돼 저장되므로, 이 좌표를 재해시해 원본 전체 정밀도 SHA와 같다고 주장하지 않습니다.

| 실제 truth 수신 wall 시각 | 차량 위치 | 상자 중심까지 거리 | 기존 선정 조건 | 새 spawn 기하 판정 |
|---|---|---:|---|---|
| 1793812904.5892 | (−0.10724, −0.31660) | 0.616269m | 허용 | 허용 |
| 1793812904.8512 | (−0.08884, −0.33850) | 0.589961m | 거부 | 허용 |

차량은 여전히 상자보다 tangent 방향 약 0.56m 앞에 있었지만, 최종 확인 사이에
처음 **선정용** `.60m` 하한을 넘었습니다. 기존 harness는 이를
`FIXED_SCENE_PASSED_OR_ROUTE_CHANGED_BEFORE_SPAWN`으로 합쳐 기록했습니다.
검색 계산 자체는 0.296초였으며, 검색 시작/결과 소비의 4.52초 차이에는 실제 Home 출발
10cm를 기다린 시간이 포함됩니다. 경로가 바뀌어서 실패했다고 단정할 근거는 없습니다.

## 수정한 범위와 유지한 기준

초기 후보 선정의 **0.60–1.00m 범위는 그대로**입니다. 선정 후 spawn 직전에는 기존
`movement_step_readiness`를 정지된 상자(start=end)에 재사용해 다음을 확인합니다.

- 같은 목표 stamp, 실제 `MOVING` 상태와 기존 경로·관측 신선도
- 상자가 여전히 앞에 있음(along ≤ −0.25m), 현재 경로를 1cm 초과로 실제 방해함
- 정확한 25cm 상자 AABB와 선택된 22cm 원형 차체 사이 여유가
  `2cm + 0.22m/s × (0.20s 반응시간 + truth 나이)` 이상임
- 기존 truth 나이 0.20초 상한, static/observed/remembered 우회 증명

비용이 들 수 있는 memory 우회 재검증 뒤에도 service dispatch 바로 앞에서 기존
`route_readiness`를 현재 시각으로 다시 확인합니다. 같은 고정 위치·이동 방향을 유지하며,
이후 네 상자 이동의 swept-body 검사와 실제 2cm QA도 변경하지 않습니다. 로봇의
주행 정책·파라미터·충돌 검사는 수정 범위가 아닙니다.

`fixture.json`의 before/after truth 위치에 대해 허용 가능한 truth 나이 0/50/200ms를
각각 재생합니다. 기존 actor는 해당 함수 호출 순간의 정확한 truth 나이를 기록하지 않았으므로
이는 **신선한 입력을 가정한 순수 판정 비교**이며, 과거 spawn이 실제 실행됐다는 뜻이 아닙니다.
몸체 겹침, 반응 여유 부족, 이미 뒤에 있는 상자, 오래된 truth, 더 이상 방해하지 않는 경로,
다른 목표, `MOVING` 아님, 긴 검색 중 만료된 경로 lease 등 8개 반례를 함께 거부합니다.

## ROS 없이 재생

이 폴더 전체를 다른 위치에 복사해도 표준 Python만으로 실행됩니다.

```bash
python3 replay.py
```

재생기는 `injector_before.py`/`injector_after.py`의 **정확한 보존 소스**에서 순수 함수만
AST로 읽어 실행합니다. ROS·웹·Gazebo 프로세스를 시작하거나 명령을 발행하지 않습니다.
보존된 소스·패치 SHA를 먼저 확인하며, 결과는 `report.json`에 있습니다.
수정 당시 신규·기존 injector/full-stack 회귀 **64개가 통과**했습니다.

| 보존 대상 | SHA256 |
|---|---|
| 수정 전 actor script | `66e046248ee8931054c33dafbf2d1775681cdeeba631d41cfad9849f8c78f001` |
| 수정 후 actor script | `2ccc7928a94469f6331d379b9a86b19c9b82d6e043b67ed686a56170f8f5b126` |
| 적용 패치 | `f32e16b7ddbaf61f1e2f07ab483e41160fcbea62fc03d659c07941c1828ceb6a` |

후속 실제 폐루프 결과는 별도 시험 기록을 따릅니다. 이 회귀 자료에는 후속 시험 PASS를 기입하지 않습니다.
