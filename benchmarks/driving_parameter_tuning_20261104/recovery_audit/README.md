# `NO_SAFE_EGRESS` 후진 거절 감사

**실제 후진 성공 시험이 아닙니다.** 기록된 기하와 수정 전 selector를 사용한 오프라인 대조입니다.
실제 breadcrumb 목록은 저장되지 않아 완전히 직선인 13개 이력을 가정했습니다.

대상은 `/tmp/classroom-tuning-fixed-repaired-infl30-20261104`의 stage25입니다.
상자 4회 이동 뒤 `(33,87)`에서 정지했고 첫 `NO_SAFE_EGRESS`는 wall epoch
`1793799587.539447`입니다. 직전 진단 pose는 `(0.006046, -0.889081, -1.521321)`입니다.

## 확인된 두 거절 조건

| 항목 | 결과 |
|---|---|
| 재구성 관측/기억 | 9 / 15셀; 보수 팽창 346 / 261셀로 실제 로그와 일치 |
| 실제 raw static·unknown·관측·기억 셀 대비 시작 거리 | 24.7064cm |
| 후진 0~30cm 전체 직선 거리 하한 | **24.5088cm > 기존 요구 23.5cm** |
| 최근접 셀 거리의 초기 감소 | 약 2.7cm 후진 시점까지 최대 **1.476mm 감소** |
| 수정 전 selector, 완벽한 직선 이력 | 후보 없음; 목표 도달 가능성 callback 0회 |
| 기존 `allow_parallel_static_egress=True` 대조 | 동일하게 후보 없음; 중간 단조 조건은 그대로 유지됨 |

직선 거리 하한은 1mm 간격으로 측정한 최소 24.5588cm에서, 거리 함수의 1-Lipschitz 성질을
이용해 미측정 구간 0.5mm를 추가로 뺀 값입니다. 차체·handover 여유·기존 sample 여유는 줄이지 않았습니다.

| 후진 거리 | 전체 inflation 밖 | 전체 가상 reserve 적용 시 목표 경로 | 실측·기억 hard만 적용 시 목표 경로 |
|---:|---|---|---|
| 10cm | 아니오 | 없음 | 없음 |
| 15cm | 예 | 없음 | 있음 |
| 20cm | 예 | 없음 | 있음 |
| 25cm | 예 | 없음 | 있음 |
| 30cm | 예 | 없음 | 있음 |

따라서 단조 거리 조건만 없애도 그대로 해결된다고 볼 수 없습니다.
명목 직선의 물리 셀 간격은 충분하지만, 초기 1.476mm 감소가 먼저 거절하고
전체 가상 reserve는 후진 끝점에서 목표로 가는 경로도 막습니다.
실측·기억을 유지한 경로 가능성은 확인했으나 실제 차량의 이력·주행 가능성 검증과는 구분합니다.

## 기록과 한계

- [fixture.json](fixture.json): 실제 관측·기억·reserve 셀, 시작 pose, 경로, 수신 시각차, 상자 좌표·크기, 지도·코드 해시
- [report.json](report.json): 재현 결과와 끝점별 판정
- `reverse_recovery_before_neutral.py`: 변경 전 selector 원본
- `replay_reverse_gate.py`: 고정 원본과 fixture를 읽는 독립 오프라인 실행기

관측/기억은 독립 observer의 직전 표본이며 planner의 원자적 dump가 아닙니다.
실제 breadcrumb·anchor·콜백 순서, 각 reverse 호출의 정확한 blocker 집합과 selector 내부 최초 거절 지점은 없습니다.
뒤쪽 ±30° 라이다 최소 약 1.20m와 정지 상태 Guard `CLEAR`만으로 후진 중 차체 전체를 검증할 수 없습니다.
전체 raw beam·장착 TF·획득 시각 pose와 실제 이력은 향후 실주행 검증에 필요합니다.
상자 bounds는 소유 actor의 네 번째 성공한 위치 변경과 25cm 형상으로 계산했으며 별도 전체 ModelStates dump는 없습니다.

## 재현

ROS·가제보·모터를 실행하지 않는 일반 Python 도구입니다. 기존 결과를 덮어쓰지 않습니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
python3 benchmarks/driving_parameter_tuning_20261104/recovery_audit/replay_reverse_gate.py \
  --output /tmp/amr-reverse-gate-replay.json
```

향후 복구 개선도 22cm 차체·23.5cm 전체 sweep 요구·유효한 실제 이력·현재 제어권·
전체 inflation 밖의 끝점·동일 목표 경로 증명을 유지해야 합니다.
가상 reserve를 비용으로 바꾸는 경우에도 실측·기억·정적 장애물은 hard로 유지합니다.
