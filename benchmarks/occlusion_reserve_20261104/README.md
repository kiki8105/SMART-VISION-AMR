# 22cm 모델의 가상 장애물 깊이 과대 확장 재현

**실측 장애물·차체·A* 팽창을 줄이지 않고**, 가상 깊이 예약의 신뢰도를 구분한다.
먼저 25→20cm 튜닝으로 첫 저장 입력의 경로를 복구했지만, 다음 실제 스캔에서는 다시
막혔다. 현재 구현은 보수적인 hard 예약 검색 실패 후 **가상 셀만 soft 비용으로 재시도**한다.
아래 결과는 저장 입력의 경로·기하 회귀다. 변경 후 22cm 폐루프 통합·장애물 시험은 **PENDING**이다.

## 저장된 실패

`classroom-inflation-20261104-022/25_web_inventory_sector3_moving_obstacle`에서
상자 설치 직전 80셀 우회 가능성이 확인됐지만, 설치 후 A*가 경로를 만들지 못했다.
첫 실패에는 관측 셀 25개, 기억 셀 0개, 실패 간선 0개였고 LiDAR guard는 `CLEAR`였다.
상자 제거 뒤 임무의 Home 복귀는 통과했으나, **상자가 있는 동안 우회는 실패**했다.
이후 통과 결과로 이 실패를 덮어쓰지 않는다.

원인은 관측 hit 자체가 아니라, 기존 경로 방향으로 숨은 깊이 25cm를 예약한
가상 셀과 정적 지도의 결합이었다. 같은 지도·관측에 예약을 빼면 경로가 존재했고,
실제 상자의 보이지 않던 뒷면과도 충돌하지 않았다.

## 고정 입력과 변경 범위

| 항목 | 값 |
| --- | --- |
| 원형 차체 반경 | 22cm, 변경 없음 |
| 지도 | 5cm / 86열×140행 / origin (-1.65, -5.25) |
| A* 정적 팽창 | 5셀=25cm, 변경 없음 |
| Nav2 비용 팽창 | 당시 22cm, 이번 오프라인 A* 재생에서는 사용하지 않음 |
| 중앙 선호 | 45cm / gain 1.5, 차단 반경이 아닌 비용 |
| 곡선화 | 간격 5cm / shortcut 80cm / 2회 |
| 실제 상자 | 25×25cm, 중심 (0.523914, -0.599707) |
| 실제 상자 범위 | x [0.398914, 0.648914], y [-0.724707, -0.474707] |
| 조정한 설정 | `local_detour_occlusion_extension: 0.25 → 0.20` |

국소 복구와 전체 A*가 같은 설정으로 같은 예약 셀을 계산하는지 회귀로 확인한다.
원본 관측, 기억 확정·해제, raw 지도, 전체 차체 팽창, 후진·액션 인계 조건은 바꾸지 않는다.

## 민감도

간격은 **경로의 모든 연속 선분과 실제 상자 사각형 사이의 정확한 최소 거리에서
원형 차체 반경 22cm를 뺀 값**이다. 격자 중심점만 비교한 값이 아니다.

| 예약 깊이 | 가상 추가 셀 | A* 경로 셀 수 | 곡선화 후 실제 상자 간격 |
| ---: | ---: | ---: | ---: |
| 0cm — 원인 분리용 | 0 | 75 | 9.41cm |
| 5cm | 8 | 76 | 9.64cm |
| 10cm | 8 | 76 | 9.64cm |
| 15cm | 14 | 76 | 12.63cm |
| **20cm — 1차 튜닝** | **20** | **76** | **13.88cm** |
| 25cm — 실패 설정 | 22 | 경로 없음 | 해당 없음 |

1차 튜닝 경로의 곡선화 전 간격은 15.07cm다. 예약을 완전히 끄는 대신 20cm를 남겼고,
새 실제 hit가 기존 우회 통로에 들어오면 계속 hard 장애물로 차단하는 회귀도 유지한다.

## 후속 스캔 실패와 confidence fallback

`classroom-inflation-fixed-20261104-022`의 새 스캔 **관측 26개 / 기억 0개**에서
20cm 예약도 `NoPath`를 만들었다. 이후 관측 18개 / 기억 12개에서도 실패가 이어져
시험을 중도 종료했다. 첫 고정 입력에 맞춘 튜닝만으로 일반적인 해결을 주장하지 않는다.

현재 구현은 다음 순서다.

1. 실제 관측·기억·정적/unknown과 가상 예약을 모두 hard로 보수 검색한다.
2. 실패하면 **가상 예약만** 유한 비용으로 바꾸고, 동일 목표·원래 경로의 국소 복구를 먼저 시도한다. 실제 관측·기억의 전체 차체 팽창은 그대로 hard다.
3. 국소 복구가 실패하면 전체 A*도 같은 신뢰도 구분으로 재탐색한다. 전체 fallback은 신선한 scan·odom·TF·자율 제어권, 현재 목표·지도·원래 경로 상관을 재확인하며 일반 free start에서만 허용한다. start-egress 예외는 넓히지 않는다.

국소 두 단계는 **동일 attempt 1회 / 전체 60ms / 1,500확장 / 합류 후보 10개**를
공유한다. 보수 단계는 시간·확장의 60%, 후보의 50%를 먼저 배정하며, soft 단계는
실제로 남은 예산만 쓴다. TF 확인·비용 구성 시간도 같은 기한에 포함한다.
새 국소 시도권을 만들거나 원래 suffix를 바꾸지 않는다. 전체 재계획에도 실제 hard
마스크로 곡선화·연속 구간을 검사하고 Nav2 외곽 검사·raw guard를 유지한다.

| 저장 입력 | 국소 soft 결과 | 확장 / 합류 후보 | 실제 상자 최소 간격 | 전체 A* 호출 |
| --- | --- | ---: | ---: | ---: |
| 첫 시험 25cm 예약: 관측25 / 기억0 | 성공·attempt 1회 | 255 / 10 | 11.53cm | 0 |
| 후속 20cm 예약: 관측26 / 기억0 | 성공·attempt 1회 | 225 / 10 | 15.07cm | 0 |
| 후속 20cm 예약: 관측18 / 기억12 | 성공·attempt 1회 | 189 / 10 | 18.03cm | 0 |

전체 A* fallback도 세 입력에서 실제 장애물·기억을 보존한 채 같은 상자 간격의 경로를
생성했다. 실제 관측 또는 기억으로 통로 전체를 막은 반례는 국소·전체 모두 거부한다.
센서·TF·제어권이 오래됐거나 reference가 다르면 soft fallback을 승인하지 않는다.
`occlusion_constraint_mode=SOFT_VIRTUAL`, `occlusion_fallback`,
`occlusion_primary_reason`으로 가상 예약을 완화한 원인을 상태·로그에 남긴다.
soft 예약은 미관측 공간이 실제로 비었다는 확인을 뜻하지 않는다.

수치는 [confidence_fallback_replay.json](confidence_fallback_replay.json)에 있다.
후속 입력은 [관측26 fixture](../../ros2_ws/src/smart_vision_amr/test/fixtures/occlusion_reserve_second_first_20261104.json)와
[기억12 fixture](../../ros2_ws/src/smart_vision_amr/test/fixtures/occlusion_reserve_second_memory_20261104.json)에 보존했다.

## 움직인 상자와 동일 호출의 출발 인증 재사용

후속 22/30/40cm Nav2 팽창 비교에서는 첫 국소 우회는 생성됐지만, 상자가 우회 쪽으로
세 번 움직인 뒤 출발 셀이 관측 팽창에 걸렸다. 보수 A*가 만든 **first-edge 인증 1개**가
있어도 당시 soft fallback은 인증이 있는 출발점을 제외해 대기했다. 이후 기억 팽창도
출발점을 막았고, 상자 제거 뒤 재개했다. 이 비교의 장애물 우회 실패를 보존한다.

22cm 비교의 `1793796120.478…` 입력은 관측29 / 기억9이며 시작은 (33,91)이다.
그 호출의 fresh 연속 자세에서 인증된 간선은 **(33,91)→(32,91)** 하나뿐이다.
같은 escape **{(33,91)}**과 금지 간선 **23개**를 유지한 채 가상 예약만 soft로 바꾸면
75셀 경로가 생긴다. 세 번째 이동 후 실제 box 범위는
x [0.345019,0.595019], y [-0.776863,-0.526863]이고, 원형 차체 22cm의
연속 경로와 상자 간격은 **10.00cm**다.

현재 최소 변경은 같은 `plan_to` 호출의 기존 인증만 재사용한다.
fallback 직전과 최종 발행 직전에 fresh pose로 원래 인증 함수를 다시 실행하고,
**원인증과의 교집합만 허용**한다. 신규 간선·escape 셀은 추가하지 않는다.
검증에는 원래 보수 mask·반경·margin을 그대로 쓰며, 기존 first-edge 확인·재진입/대각
금지·인증 경로의 곡선화 제한도 유지한다. 관측/기억 snapshot이 바뀌거나, 같은 셀 안에서
자세가 이동해 인증이 무효가 되거나, 이후 기억 팽창이 출발점을 막으면 발행하지 않는다.
옛 호출의 인증을 캐시해 다시 쓰지 않는다.

| 회귀 | 결과 |
| --- | --- |
| 관측29/기억9, 원인증·escape·금지 간선 그대로 재사용 | 75셀 경로 / 실제 box 간격10.00cm |
| primary 검색 뒤 같은 셀 안에서 불안전한 자세로 이동 | 경로 거부 |
| soft 검색 뒤 발행 직전 pose·scan·제어권·관측·기억 변경 | 비어 있는 정지 경로만 발행 |
| 이후 기억14가 출발점을 막음 | 이전 인증 재사용 거부 |

원자료는 [인증 출발 snapshot](../../ros2_ws/src/smart_vision_amr/test/fixtures/occlusion_reserve_certified_start_20261104.json),
production adapter 재생 결과는 [certified_start_reuse_replay.json](certified_start_reuse_replay.json),
회귀는 [test_certified_occlusion_fallback.py](../../ros2_ws/src/smart_vision_amr/test/test_certified_occlusion_fallback.py)에 있다.
이 수정 후 팽창별 폐루프 재비교는 **PENDING**이다.

## 닫힌 구간

핵심 관측은 셀 **(40,91)**, 지도 좌표 **(0.375,-0.675)**다. 이 한 관측만 있어도
25cm 예약에서는 `NoPath`가 재현되며, 전체 25개 관측에서 이 셀만 빼면 경로가 생긴다.
원인을 분리하기 위한 삭제 시험이며, 실제 운용에서 관측 셀을 버리는 수정은 하지 않았다.

25cm 예약은 이 hit를 원래 경로 접선 방향으로 늘려 **(0.575,-0.875)**까지 점유한다.
이는 실제 상자 아래 끝보다 약 15cm 더 내려간다. 차체 팽창 이후에는 유일 연결부인
**(41,82)~(44,82), 지도 y=-1.125**를 차단한다.
기록된 최신 TF로 근사한 관측 ray는 -30.19°, 경로 접선은 -45.38°여서,
이 예약은 센서 ray 뒤쪽보다 약 15.2° 더 남쪽으로 확장됐다.
이 각도 비교는 획득 시각의 원본 beam 복원이 아닌 최신 TF 기반 근사다.

## 재현

고정 입력은 [관측·지도·원래 경로 fixture](../../ros2_ws/src/smart_vision_amr/test/fixtures/occlusion_reserve_inflation022_20261104.json),
수치는 [reserve_sensitivity.json](reserve_sensitivity.json)에 있다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/ros2_ws/src/smart_vision_amr:$PWD/ros2_ws/src/smart_vision_amr/test:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q \
  ros2_ws/src/smart_vision_amr/test/test_occlusion_reserve_incident.py
```

ROS 노드·Gazebo·실제 로봇을 실행하지 않는다. 이 회귀의 통과는 해당 입력의 허위 폐쇄가
해소되고 실제 상자와 겹치지 않는다는 뜻이며, 새 환경의 미관측 물체까지 보장하지 않는다.
