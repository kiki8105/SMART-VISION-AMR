# 실행 실패 기억을 경로 차단에서 비용으로 교체 — 2026-11-05

실제 통합 시험은 Sector 1→2→3 뒤 Home 복귀 도중 중단됐으며 **불합격**입니다.
실제 물체가 없는 시점에도 이전 controller abort의 방향성 금지 간선 276개가 통로를 막았습니다.
한때 센서가 빈 공간을 확인해 비용으로 처리했던 간선이, 나중 물리 장애물 복구 단계에서는
다시 hard로 적용된 것이 원인 중 하나입니다. 원본 입력과 시각은
[실제 실패 감사](../recovery_replacement_validation/clean_home_failure_audit/README.md)에 있습니다.

최종 구현은 분류 결과를 캐시하는 보완안 대신 **실행 실패는 항상 제한된 방향성 비용**이라는
단일 정책으로 교체했습니다. 이 폴더명은 최초 조사 이름을 보존한 것이며 현재 코드에
분류 캐시·entry별 허가·과거 scan 재분류 출발 조건은 없습니다.

| 동일 실제 입력 | 실행 실패 hard 간선 | 방향성 비용 간선 | 전체 경로 |
|---|---:|---:|---:|
| 당시 hard 정책 | 276 | 740 | 없음 |
| 현재 cost-only adapter | 0 | 1,016, 비용 각각 2.0 | 33셀 |
| 현재 adapter + 실제 전폭 장애물 대조 | 0 | 동일 | 없음 |

정적 지도·unknown·실측/기억 장애물·차체 팽창·인증된 첫 간선은 계속 실제 충돌 조건입니다.
controller가 진행하지 못했다는 사실은 별도 점유 셀을 만들지 않습니다.
등록된 물리 복구는 국소 우회→정지·후진→전체 계획을 계속 소유하며, 그 동안 새 controller
재시도 횟수는 차감하지 않습니다. 그 외에는 수락한 실제 terminal event ID로 3회까지
관리하고, 같은 ID의 재수신이나 기억 TTL 만료로 예산을 돌려주지 않습니다.
과거 시도 구간 근처에 있는 측면 물체나 오래된 진단 TF가 별도 출발 정책을 만들지도 않습니다.

A*의 별도 `_turn_ready` 선회 거부도 제거했습니다. planner는 충돌 없는 경로를 생성하고,
실제 제자리 선회는 follower의 취소 종결·200ms 실제 정지 확인과 Nav2 Spin·현재 footprint/
costmap·raw guard가 담당합니다. 이 기록은 선회나 후진의 실제 성공을 주장하지 않습니다.

## 재현

```bash
python3 benchmarks/driving_parameter_tuning_20261104/controller_classification_persistence_20261105/replay_adapter_policy.py \
  --output /tmp/execution-cost-policy-replay.json
```

ROS 실행 없이 실제 현재 adapter 함수의 AST를 그대로 추출해 기록 지도·실패 항목에 적용합니다.
기록된 276개 금지 간선은 기존 순수 helper의 역사적 대조로만 사용합니다. 추가 두 retry ID와
물리 복구 phase는 단위 대조 입력이며 원래 실제 시험에서 발생했다고 표시하지 않습니다.
입력·실행 코드의 해시는 `results.json`에 보존합니다. 마지막 미사용 hold-probe 분기 정리 뒤의 최종 소스·동일 adapter 함수 해시는 `results_after_dead_probe_cleanup.json`에 따로 보존했습니다. `astar_planner_before.py`와
`failure_memory_before.py`는 수정 직전 원본이며, 순수 FailureMemory의 역사적 hard API는
현재 adapter가 호출 결과를 비용으로만 소비하므로 운용 정책의 결정권을 갖지 않습니다.

관련 검증은 `test_execution_failure_cost_policy.py`와 기존 failure/obstacle/remaining-route/
독립 복구 phase/pure-memory 검사를 합쳐 **192개 통과**했습니다. 전체 웹 폐루프의 최종
판정과 실차 반복 주행은 이 오프라인 경로·단위 검사 결과와 별개입니다.

생산 코드에서 호출자가 없던 `hold_probe=True` 전체 경로와 두 보조 함수, 사용되지 않는
이전 경로 복사본도 제거했습니다. 현재 WAIT 재개 증거와 초기 목표의 정상 A*·후진 검증은
이 정리에서 변경하지 않았습니다. 실제 source 상태의 전체 폐루프 결과를 기다리는 기록입니다.
