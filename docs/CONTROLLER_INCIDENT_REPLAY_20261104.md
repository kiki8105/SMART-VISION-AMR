# 2026-11-04 갑작스러운 선회: 컨트롤러 재현과 수정

직진 경로를 따라가다가 제자리에서 좌우로 돌던 실차 입력을 실제 ROS Humble DWB 컨트롤러에 다시 넣어 원인을 재현했다. 로봇은 연결하지 않았고, ROS 도메인 186·loopback 통신·별도 속도 출력 토픽으로 격리했다.

## 확인된 원인

- 사건: `incident_20261104T084140_566449Z_ecb28e47`, 원본 ROS 시각 `1793781696.5`.
- A* 경로는 U자 통로를 따라 아래로 진행한 뒤 돌아 들어가는 경로였다.
- DWB의 `forward_prune_distance=0.75`는 **경로를 따라 75 cm**가 아니라 **차량과의 직선거리 반경**이다. 접힌 U자 경로의 맞은편 최종 목표까지 반경 안에 들어가 로컬 목표로 사용됐다.
- 그 결과 정상 직진 후보가 433개나 유효했는데도 `GoalDist`·`GoalAlign` 점수는 직진할수록 나빠졌다. 컨트롤러가 `v=0`, `w=-0.2727 rad/s`를 선택했다. 이 재현에서는 충돌 검사 거부가 선회의 직접 원인이 아니었다.
- 반경은 A/B 양쪽 모두 요청한 **20 cm**를 사용했다. 따라서 17 cm 설정 하나만으로 이 현상을 설명할 수 없다.

## 적용한 수정

`smart_vision_amr_dwb::RouteTrackingDWB`는 기존 DWB를 상속하고 다음 세 부분을 보완한다.

1. **로컬 채점 목표:** 현재 경로 진행점부터 경로를 따라 55 cm까지만 DWB critic에 제공한다. 맞은편 미래 경로가 가까워도 현재 통로를 먼저 따라간다. 실제 목적지와 FollowPath 완료 조건은 유지한다.
2. **선회 속도:** 물리 로봇의 motion mux와 같은 선속도 상한을 후보 평가 전에 적용한다. 직선 최대 0.22 m/s, 선회 진입 0.15 m/s, 큰 선회 약 0.10 m/s다. 이미 충돌 검사를 마친 궤적의 선속도만 나중에 바꾸는 경우를 줄인다.
3. **속도 제한:** `/speed_limit`는 선속도 후보만 제한한다. 커브 감속 때문에 각속도 상한도 같이 줄어드는 기존 DWB 동작을 피한다. 장애물·회전·진행 방향 critic은 그대로 실행한다.

`/smart_vision_amr/controller_diagnostics`에는 1초 간격으로 원본/변환/제한 경로의 점 개수, 경로 진행 거리, 로컬 목표, 속도 상한을 발행한다. 전체 DWB 후보 점수는 아래 진단 시험에서만 수집한다.

## 실제 DWB 실행 결과

| 시험 | 결과 | 관측 |
|---|---|---|
| 실차 기록·기존 DWB | 현상 재현 | 선속도 0, 각속도 -0.2727 rad/s |
| 같은 기록·수정 DWB | 전진 복원 | 선속도 0.1042 m/s, 각속도 -0.0641 rad/s |
| 직선 | 통과 | 선속도 0.1853 m/s, 각속도 0 |
| 직각 코너 | 통과 | 선속도 0.0926 m/s, 각속도 0.3205 rad/s; 선회 상한 준수 |
| 정면 장애물 | 통과 | 충돌 후보 거부; 선택 후보는 충돌 critic 통과 |
| 15 mm 측면 간격 | 통과 | 중심 셀 검사에서 허용한 충돌 후보 2개를 차체 경계 검사로 거부; 전진 후보 269개 유지 |
| 실제 최종 목적지에 정지 | 통과 | FollowPath `SUCCEEDED`, 속도 출력 0 |
| 경로/속도 함수 회귀 | 통과 | C++ 9건, 재현 입력·critic 증거 처리 Python 5건 |

이 표는 **기록된 입력을 고정한 컨트롤러 시험**이다. 실차 당시 costmap은 저장되지 않아 LiDAR에서 다시 구성했다. 코너 시험도 합성 scan을 쓴 정지 입력 시험이다. Gazebo 폐루프 완주나 실차 임무 성공을 이 표로 대신하지 않는다.

작은 재현 입력과 결과는 [`benchmarks/controller_replay_20261104`](../benchmarks/controller_replay_20261104)에 포함했다. 재현 입력에는 scan·odom·TF·경로·속도 제한만 들어 있다.

## 차체 전체 충돌 검사 보완

`BaseObstacle`는 예측 궤적의 중심 셀을 검사하므로 중심은 빈 공간이어도 차체 가장자리가 장애물 셀과 겹치는 후보를 놓칠 수 있다. 두 운용 YAML에 Humble 기본 `ObstacleFootprint` critic을 추가했다. 이 critic은 같은 예측 궤적의 차체 경계도 검사한다. 아래 기록 당시에는 추가 반경 없이 `robot_radius=0.20`, `inflation_radius=0.20`, `footprint_padding=0.0`을 사용했고, 실제 발행된 16각형 꼭짓점의 최대 반경도 0.20 m였다. **현재 운용 반경은 사용자 요청에 따라 22cm**이며, 과거 20cm 재생에는 보존한 설정을 사용한다.

15 mm 측면 간격 합성 입력에서 중심 셀 검사는 충돌 후보를 찾지 못했지만, 차체 경계 검사는 2개를 거부하면서 유효 전진 후보 269개를 유지했다. scale 0.02와 0.5 모두 기존 회귀를 통과했다. 이후 통합 시험의 느린 진행을 captured costmap으로 A/B 재현한 결과, 0.5의 soft 비용이 정상 전진 후보를 밀어내는 것을 확인해 최종적으로 0.02를 채택했다. 두 설정에서 차체 충돌로 거부된 후보 집합은 동일했다. 결과는 [`full_footprint_results.json`](../benchmarks/controller_replay_20261104/full_footprint_results.json)에 있다. 이 입력 시험은 움직이는 장애물 주변에서 실제 추종 오차까지 포함한 2 cm 여유 확보의 통과 판정을 대신하지 않는다.

captured costmap A/B의 재현 입력·점수표·제약은 [차체 경계 비용 A/B](../benchmarks/controller_footprint_soft_cost_ab_20261104/README.md)에 별도로 정리했다. 실제 기록 ROS 69 s에서 같은 정상 전진 후보는 scale 0.5일 때 6.275점의 추가 비용 때문에 탈락했지만, 0.02일 때는 선택됐다. ROS 71 s에서도 전진 명령이 0.0116 m/s에서 0.1042 m/s로 복원됐다. 이것은 고정 입력 비교이며 실차·Gazebo 폐루프 성공 판정은 별도로 남긴다.

## 다른 PC에서 재현

ROS Humble·Nav2가 설치된 PC에서 저장소의 런타임을 빌드한 뒤 실행한다. 로봇 브링업은 필요 없다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/setup_pc_runtime.sh --no-install

source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/setup.bash

python3 scripts/run_controller_regression.py \
  --nav-params benchmarks/controller_replay_20261104/nav_profile_20cm.yaml \
  --incident benchmarks/controller_replay_20261104/recorded_frozen_snapshot.jsonl.gz \
  --output "/tmp/controller-regression-$(date +%Y%m%d-%H%M%S)"
```

시험은 `results.json`, 각 시험의 `controller.log`, `summary.json`, `observations.jsonl`을 남긴다. `all_passed=true`는 위 컨트롤러 회귀의 통과를 뜻한다. 실제 왕복 주행은 별도로 확인해야 한다.
