# 실제 점유 사각형을 유지한 짧은 metric connector

이번 구현은 원본 5cm 지도 셀을 작게 만들지 않는다. 원래 occupied/unknown·현재 관측·확인된 기억의 **전체 사각형**을 유지하고, 후보 차량 중심만 1cm 간격으로 둔다. 반경22cm와 기존 설정의 clearance margin을 검사한다. 숨은 +2cm hard 여유는 없다. 사각형 병합은 정확한 합집합을 보존하는 계산 최적화이다.

기존 국소 계획에서 coarse start가 막혔거나 경로가 없으면 같은 60ms/1500확장 예산 안에서 짧은 connector와 원래 forward suffix 전체를 검사한다. 기존 suffix의 점 순서와 좌표를 보존한다. barely-free suffix보다 clearance 이득과 connector 길이를 함께 가격으로 평가한 suffix를 선호한다. 완성 경로는 실제 시작점·anchor·전체 tail을 정확한 segment–사각형 거리로 다시 검증하고, 현재 목표/지도/관측/기억/authority 및 paired 원본 raw scan도 발행 직전에 재확인한다.

후진은 실제 rear sweep/이력·정지 증명이 있을 때만 요청한다. 후진 불가이면 같은 episode의 남은 국소 예산을 한 번만 사용하여 현재 방향의 좌우45~135° anchor와 실제 첫15cm tangent가 옆 방향인 경로를 찾는다. 성공한 비어 있지 않은 경로는 기존 follower의 stop→owned Spin→FollowPath를 사용한다. 임의 회전 요청이나 새 복구 프로토콜은 추가하지 않았다.

새 경로의 관측/기억 검증도 현재 원형 차체와 실제 remaining polyline을 검사한다. crossed 5cm 후보 셀 전체를 다시 footprint로 삼아 취소하지 않는다. 새 실제 관측 사각형, 현재 raw body/sweep, unknown, 지도 외부, 수동/비상정지와 source freshness는 그대로 hard 조건이다. time/expansion budget 초과 결과는 게시하지 않는다. 반복 global 억제는 **현재 실제22cm 원형 자체가 원본 점유 사각형과 교차하는 경우만** 대상으로 하고 매 cache hit마다 fresh 현재 원형 교차를 다시 확인한다. 그 밖의 NoPath는 기존 장애물 확인 계수/복구 phase를 거친다. raw-only endpoint 거부·입력대기·예산 소진은 cache하지 않는다. yaw5° 변화와 실제 공간 odom trail 변화도 재검토한다.

## 입력과 증거 범위

- 양성 입력: [common_coherent_v2_geometry_tracking_inputs.json](../physical_transfer_20261105/common_coherent_v2_geometry_tracking_inputs.json). pose는 `(0.179834,−1.155174,−0.114983)`, 관측11셀·기억23셀이다. 독립 DDS observer snapshot이므로 exact 내부 callback snapshot으로 주장하지 않는다.
- 두 번째 양성 입력: [stress_first_no_path.json](stress_first_no_path.json). 기존 `final_stress_permuted`의 최초 NoPath 원자료를 변경 없이 보존한다. pose `(0.358839,−1.183793)`에서 가장 가까운 실제 지도 점유 square는 **(38,76)의 corner**이다. 정확 중심거리는224.070mm,22cm 차체 여유는 **+4.070mm**이다. 관측/기억 observer는 planner보다115ms 이른 자료이며 이 한계도 파일에 남겼다. [독립 재평가](../recovery_runtime_fixes_20261105/stress_corner_geometry_reassessment.json)도 같은 결과를 확인했다.
- synthetic fresh raw/odom/TF를 제공한 실제 planner callback 회귀에서 common 입력은 원래 goal을 보존한 전체 route를 생성하고 같은 관측·기억 승격에도 취소되지 않았다. 새로운 실제 body hit, raw wall/unknown, stale source·authority 및 paired input 부재는 거부했다.
- 독립 연결 회귀는 실제 rear selector→BackUp 요청0→side Path→cancel ACK/terminal/새 정지창→Spin→FollowPath까지 검사한다. **이것은 실제 Gazebo 또는 실차 완주 결과가 아니다.**

초기 해석에서 벽의 top edge `y=−1.40`이 x전체로 이어진다고 잘못 가정해 “3.793mm 겹침”으로 보고했다. **그 해석은 철회한다.** 원본 square는 x=.30에서 끝나므로 corner까지의 2차원 거리를 써야 한다. 기존 Lipschitz 샘플 간 reserve2.5mm는 실제1.923mm의 안전한 첫 fine-edge 여유를 거부했다. 최종 구현은 endpoint가 body-clear인 경계 edge에 한해 정확한 전체 segment–사각형 거리로 재증명한다. 반경을 낮추거나 샘플 간 미검사 구간을 허용하지 않는다. 이 입력의 전체 connector+tail도 unit에서 양성으로 검증했다. 이것이 실제2cm QA 여유나 폐루프 성공을 의미하지는 않는다. 진짜 overlap 음성은 벽 위 x=.275의 별도 fixture로 검증한다.

동결 구현 SHA256:

| 파일 | SHA256 |
|---|---|
| `astar_planner.py` | `78845bb22e11dfd4c29673696d6c8afce9ca66236c36b30caf6d910ce0f572ce` |
| `metric_connector.py` | `1396a00041b5fbcbaa646bc0de61ca5f5875be559c4c801ad92a92f184dfdda5` |

기능/소유권 unit은 명시적인 test-only100ms 예산을 사용한다. 생산 설정은60ms로 유지한다. 이전 중간 구현의 수동 isolate 측정은 helper20.8ms/전체 adapter39.9ms·341확장이었지만, host scheduling에 따라40ms 후진 잔여 예산에서 timeout도 발생했다. 이를 최종 구현의 생산 성능 합격이나 p95로 보고하지 않는다. 최종 소스에서는 source cache P1 및 stress corner 정확 기하를 추가 수정했다. 최종 동결 소스의 폐루프 결과는 별도 actual Gazebo 보고서로 판단해야 한다.

## 가벼운 pure replay

Python3, NumPy, Pillow, PyYAML이 필요하다. ROS import/init, DDS, Gazebo 또는 주행 명령은 없다. 아래 도구는 입력 decode를 마친 in-memory geometry에서 **pure helper**의 준비·검색·전체 출력 검증 시간을 잰다. adapter의 추가 fresh TF/원본 raw endpoint 검증·Path 작성 시간은 포함하지 않으므로 전체 callback60ms 합격으로 해석하면 안 된다. 이미 존재하는 출력 파일은 덮어쓰지 않는다.

```bash
python3 benchmarks/driving_parameter_tuning_20261104/metric_connector_recorded_20261105/replay_connector.py \
  --case common --output /tmp/metric-common-replay.json
python3 benchmarks/driving_parameter_tuning_20261104/metric_connector_recorded_20261105/replay_connector.py \
  --case stress --output /tmp/metric-stress-replay.json
```

원래 [fine geometry prototype](../fine_geometry_feasibility_20261105/README.md)의 미채택·60ms 초과 결과와 이전 실패 기록은 삭제하지 않았다. 새 helper는 SciPy/EDT·worker 없이 작은 지역의 분석적 거리장과 공유 suffix 거리 계산을 사용한다.
