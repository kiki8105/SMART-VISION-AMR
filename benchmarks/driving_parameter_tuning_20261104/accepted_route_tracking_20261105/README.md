# 실행이 확인된 우회 경로 추종 감사 — 2026-11-05

전체 시험은 두 경우 모두 장애물 시험 실패·HOME 복귀 성공이다. 아래는 그 결과를 성공으로 바꾸는 판정이 아니라, 실행 소유권이 확인된 경로와 실제 궤적의 원인 분석이다.

| 항목 | v2 기준 | local costmap 2.5cm 단일변경 |
|---|---:|---:|
| 첫 우회 진입 heading error(기존15cm lookahead) | −47.5° | −75.5° |
| 최종 우회 실제 횡오차 | 7.99cm | 14.54cm |
| 같은 시점 TF합성 추정 횡오차 | 7.16cm | 11.20cm |
| 계획선 투영 위치의 상자 여유 | 11.86cm | 18.59cm |
| 실제 차체와 상자 여유 | 6.22cm | 4.06cm |
| 실행 path stamp(ns) | 15280000000 | 16340000000 |

`tracking_report.json`은 exact path_stamp가 붙은 MOVING ACK 이후만 사용한다. 초기 Spin은 제외하고, 새 ACK·빈 경로·terminal/wait 상태에서 구간을 끊었다. 첫40초에 equivalent alias ACK는 없었다. 상자 위치는 실제 spawn/move의 절대 wall time으로 정렬했으며 이동이 유예된 목표 위치를 실제 위치로 취급하지 않았다. 차체22cm 원과25cm 정사각 상자를 사용했다. 원래 장애물 발견 전 경로는 새 상자를 관통하므로 교체 경로의 수치와 구분한다.

TF 합성은 최신 수신 map→odom과 odom→base_footprint로 추정한 map 자세이다. AMCL 메시지나 컨트롤러 내부의 동시 자세를 직접 기록한 값은 아니다. 최종 witness의 TF 수신 age는 기준38/39ms, 후보39/1ms이다. 경로 좌표는 recorder가0.1mm로 반올림했다. 전체 경로의 최소 상자 여유는1mm 샘플이므로 연속 하한은0.5mm를 빼야 한다. 개별 witness의 원→사각형 거리 및 투영 거리는 해석식이다.

수정 전 follower는 최초 FollowPath를 보낸 뒤 goal generation 전체를 정렬 완료로 취급한다. 따라서 같은 목적지의 큰 우회 prefix 변경이 기존0.55rad 조건을 넘더라도 Spin 없이 전진한다. 실제 두 기록에서도 최초 Spin만1회씩 있었고 우회 진입 후 추가 Spin은 없었다. DWB 세부 critic별 선택 원인은 이 자료만으로 확정하지 않는다.

`actual_entry_fixture.json`은 실제 두 경로·수락 ACK·자세·입력 hash를 보존한다. `path_follower_before_material_replan_alignment.py`는 이 수정 직전 코드의 원본이다. 회귀 A/B에서 같은 실제 입력을 두 구현에 넣되, 정지 증거는 별도 테스트용 ROS 메시지로 제공하며 실차 정지가 기록됐다고 주장하지 않는다.

## 새 시험의 실행 경로 오차 분석

새 시험이 생성한 stage 폴더를 지정할 수 있습니다. 기본 관찰 범위는 원래 감사와 같은
처음40초입니다. 필요하면 `--window 90`처럼 범위를 명시합니다. 결과가 이미 있으면 덮어쓰지 않습니다.

```bash
python3 benchmarks/driving_parameter_tuning_20261104/accepted_route_tracking_20261105/analyze_owned_tracking.py \
  --trial new=/tmp/시험폴더/25_web_inventory_sector3_moving_obstacle \
  --output /tmp/new-owned-tracking.json
```

Spin 상태를 FollowPath 수락으로 간주하지 않습니다. 동등 경로 alias를 포함한 새 자료는
실제 액션에 전달된 기하와 최대3cm 차이가 날 수 있으므로 별도 확인해야 합니다.
원래 두 실행은 관찰 범위에 alias ACK가0회였고, CLI 정리 후에도 기존 수치·귀속 결과가 모두 일치했습니다.

독립 재계산에서는 수락 구간의 truth69개·heading12개를 확인했다. 두 시험의 기록된 follower 해시는 위 원본과 일치한다. `accepted_routes_fixture.json`에 실제 경로8개와 ACK·중단 이벤트를 보존했으므로 다른 PC에서도 ROS·Gazebo나 원래 `/tmp` 시험 파일 없이 산술 검증할 수 있다.

```bash
python3 benchmarks/driving_parameter_tuning_20261104/accepted_route_tracking_20261105/verify_recorded_witnesses.py
```

원본 시험 파일도 남아 있다면 `--verify-original-inputs`를 붙여 전체 입력 해시까지 확인할 수 있다. 검증 결과는 `independent_verification.json`에 보존했다. 이 검증은 기록된 자세로 횡오차·상자 간격·15cm lookahead 방향을 다시 계산하는 절차이며 새로운 컨트롤러 실행이나 주행 합격 판정이 아니다. 향후 equivalent-route alias가 발생하는 자료는 alias가 가리키는 실제 실행 기하를 따로 증명해야 하며, 최근 발행 경로만으로 실행 경로를 대체하지 않는다.
