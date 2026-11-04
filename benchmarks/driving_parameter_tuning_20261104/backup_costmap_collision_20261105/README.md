# BackUp의 Collision Ahead 원인 — 2026-11-05

실제 전체 웹 v2 시험에서 후진 요청은 전달되고 Nav2 BackUp가 시작됐지만, 음수 속도를 내기 전에 충돌 검사로 거부됐습니다. Sector 3 임무와 홈 복귀는 완료했으나 장애물 우회 시험 전체는 불합격입니다.

| 검사 | 결과 |
|---|---|
| 설치된 Nav2 C++ 검사기 + 실제 5cm costmap | 7~10mm 후진 지점에서 LETHAL 254 |
| 같은 점유 사각형 전체를 보존하며 2.5cm로 분할 | 같은 지점 거부 |
| 식별된 원본 측정점만 2.5cm에 다시 표시; 다른 점유·unknown 사각형 모두 유지 | 전체 0~107mm 구간 254 없음 |

| 입력·경계 | 보존된 근거 |
|---|---|
| 실제 BackUp 요청 | 0.107881m, 0.1m/s, 10Hz; 첫 음수 명령 전에 중단 |
| 전체 costmap | source 16.660s, 충돌 경고 218ms 전 수신; 60×60, odom, 5cm |
| 원본 측정점 | scan 15.594s의 66번 beam; source 시각 odom TF 보간 |
| footprint·현재 TF | footprint 16.794s, 현재 odom TF 16.859s; 16각형·반경 22cm |
| 실제 거부 경계 | 현재 253은 허용, 7~10mm 후진은 254로 거부 |
| 격자 해상도 대조 | 기존 점유 사각형 유지 대조와 원본 점 재표시 대조를 분리 |

- 실제 Nav2 검사기는 253을 허용하고 254 이상을 거부합니다. 현재 위치는 253이며 실제 속도 0.1m/s·10Hz의 첫 10mm 예측 위치는 254입니다.
- 충돌 셀은 odom `(34,31)`, 사각형 `[0.50,0.55] × [-0.35,-0.30]m`입니다. 7mm 후진 시 선택된 22cm 원과 약 0.92mm 겹칩니다. 따라서 기존 셀을 유지하면서 검사를 끄거나 통과시켜서는 안 됩니다.
- 원본 scan 시각 15.594초, beam 66, 거리 0.283277m의 끝점 `(0.542818,-0.349054)`가 그 셀 안에 있습니다. 같은 점은 2.5cm 격자에서도 보존됩니다.
- 동일 객체를 표현한 global map 셀보다 local odom 셀이 차량 쪽으로 약 2.5cm 더 뻗습니다. 격자 좌표계 차이와 셀 영역의 보수성이 planner 통과·Nav2 거부의 차이를 설명합니다.
- 다음에 기록된 scan 두 개에는 같은 셀의 끝점이 없지만 costmap에는 남아 있습니다. 중간 scan은 모두 녹화되지 않았으므로 최초 점이 유일한 원인이라거나 유지 기간만의 문제라고 확정하지 않습니다.

2.5cm의 실제 raw 재표시는 별도 전체 웹 시험으로 검증해야 합니다. 이 원인 분리 시험은 최적 파라미터나 실차 합격을 의미하지 않습니다. 2.5cm 실시간 시험에서도 다른 이유로 후진 후보가 거부되어, 현재 공식값으로 승격하지 않았습니다.

## 재현

ROS Humble/Nav2와 C++ 컴파일러가 설치된 Ubuntu에서 저장소 루트 기준:

```bash
python3 benchmarks/driving_parameter_tuning_20261104/backup_costmap_collision_20261105/replay.py \
  --output /tmp/backup-costmap-replay.json
```

ROS 노드·Gazebo·모터 명령은 실행하지 않습니다. 설치된 `libnav2_costmap_2d_core.so`의 실제 footprint checker에 저장 입력을 넣습니다. 결과에는 입력·C++ 코드·라이브러리 해시를 기록하며 기존 결과 파일은 덮어쓰지 않습니다.

`fixture.json.gz`에 충돌 218ms 전 전체 costmap, 후방 확인 당시 footprint, source scan과 TF를 보존했습니다. footprint와 현재 pose는 기록 도구의 소수점 5자리 반올림 한계가 있으며, soft cost 복원은 양자화됩니다. lethal/unknown 분류는 보존합니다. `frame_projection_audit.json`은 실제 상자·global/local 셀의 독립 좌표 대조입니다.
