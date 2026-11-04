# 장애물 없는 Home 복귀 실패: 추종·위치·격자·채점 분리

공식 설정은 바꾸지 않았다. 실제 `12_web_integrated_clean`은 Sector 1·2·3 확인 후 Home 복귀 중 정지해 중단됐으며 **전체 실패**다. 아래는 그 실행의 실제 기록 분석과 domain **186**·localhost의 실제 Humble DWB 고정 입력 비교다. 제어 명령은 `/replay/controller_cmd_vel`에만 발행했다. 실차나 Gazebo는 시작하지 않았다.

## 실제 실행 근거

실행 소유권은 Home 목표의 `MOVING` ACK와 `path_stamp=143680000000`으로 확인했다. 계획선은 문제 구간에서 `x=0.675m`이고, world 벽은 `x=1.05m`부터다. 반경22cm 차체가 계획선 위에 있으면 여유는15.5cm다.

| 실제 기록 | 값 |
|---|---:|
| 최대 실제 횡오차 | 9.351cm |
| 같은 시점 TF 합성 추정 횡오차 | 5.985cm |
| truth↔TF 합성 추정 위치 차이 | 3.748cm |
| 실제 차체와 world 벽 최소 여유 | 6.149cm·비접촉 |
| 전체 계획 경로의 world 여유 | 1mm 샘플 최소14.553cm; 연속 하한14.503cm |
| 첫 `No valid trajectories` / ABORT | wall1793806695.884 / 1793806696.682 |

직선으로 넘어오는 동안 실제 횡오차는 관측 t182의3.42cm→t184의5.05cm→t186의7.81cm→t188의9.25cm로 증가했다. 단순히 계획선이 벽을 관통한 사례가 아니다. TF 합성은 최신 수신 변환으로 계산한 추정 자세이며 컨트롤러 내부의 동시 자세를 직접 기록한 값은 아니다. world 접촉 거리와 추정 거리도 구분한다.

`horizon_audit.json`의10개 실제 controller 진단에서 anchor는0.356~0.432m로0.85m 검색 상한에 닿지 않았다. local goal은 현재 경로 투영점보다 약55cm 앞이고 원래 경로 위였다. 이 구간에서 horizon 포화나 다른 가지 선택을 원인으로 판단하지 않았다.

첫 정지 직전 full costmap의 실제 lethal100 셀 `(34,31)`은 당시 odom 자세의22cm 원과 셀 사각형 기준으로 약5.24mm 겹친다. 따라서 최종 현재 footprint의 costmap 충돌 거부 자체와, 그 전에 왜 경로 쪽으로 충분히 보정하지 않았는지는 별개로 분석해야 한다. 이 기록만으로 실제 벽 접촉이라고 해석하지 않는다.

## 실제 DWB 단일 인자 비교

동일 기록의 full costmap·TF·odom·경로·속도 상한을 고정했다. 기준 대비 `PathDist.scale 32→64` 또는 `PathAlign.scale 32→64`만 각각 변경했다. 두 값을 동시에64로 바꾸는 시험은 하지 않았다.

| 기록 ROS 시각 | 기준 v / ω | PathDist64 v / ω | PathAlign64 v / ω | 예측 끝점 경로오차: 기준→후보 |
|---|---|---|---|---|
| 159.30s | .1737 / .0385 | .1158 / .1410 | .1158 / .1410 | 4.79→1.68cm |
| 161.34s | .2084 / 0 | .2084 / 0 | .2084 / 0 | 7.42→7.42cm |
| 163.44s | .1853 / .0385 | .1274 / .1410 | .1274 / .1410 | 5.00→2.90cm |

단위는 v=m/s, ω=rad/s이다. 끝점 오차는 선택된2초 예측 궤적의 끝점을 실제 기록 경로에 투영한 기하값으로, 실제 이동 결과가 아니다. 각 프레임의 유효 후보 수432/398/426과 모든798개 후보의 유효·거부 판정/거부 critic은 세 설정에서 동일했다. 경로 가중치가 충돌 거부를 완화한 비교가 아니다.

159.30s 기준은 경로 오차1셀을 감수하는 빠른 후보를8.451점으로 선택했고, 경로 오차0셀의 보정 후보는8.651점으로0.2점 높아 탈락했다. 후자는 `GoalAlign`·`GoalDist`의 전진 비용이 더 크기 때문이다. `PathDist` 또는 `PathAlign` 한 항만 올리면 이 순위가 바뀐다. 같은 프레임의 기준 재생 명령은 당시 기록 명령 `.173684/.038462`와 일치했다. 다른 두 프레임은 직전 기록 명령과 완전히 같지 않으므로 원래 내부 상태까지 복제했다고 주장하지 않는다.

**두 후보의 폐루프 우열·반복 신뢰도는 미검증**이다. 161.34s에서는 한 항을 두 배로 올려도 결과가 같아 이 값만으로 모든 이탈을 해결한다고 주장하지 않는다. 공식 YAML은 유지했다.

## 재현과 자료

Humble·기본 Nav2와 이 저장소의 C++ 플러그인을 빌드한 PC에서 실행한다. 원래 trial과 같은 `Team2_workspace/install`을 사용하며, 잘못된 repo 내부 install로 초기화한 실패 시도6개는 `invalid_initialization_attempts.json`에 유효 DWB 시험과 구분했다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/setup.bash
python3 benchmarks/driving_parameter_tuning_20261104/clean_home_failure_audit/tracking_ab/run_frozen_screen.py \
  --output "/tmp/clean-home-tracking-ab-$(date +%Y%m%d-%H%M%S)"
```

- `captured_inputs.jsonl.gz`·`capture_manifest.json`: 당시 비행기록기의 원본 입력과 source/receipt 시각.
- `snapshots/`: 프레임별 입력을 중복 없이 보존. 각 비교의 입력 SHA가 같다.
- `paired_results.json`: winner·critic별 점수·끝점 오차·직전 실제 명령.
- `candidate_identity_checks.json`: 6개 paired 비교의 전체 후보 판정 일치 확인.
- `t*/observations.jsonl.gz`: 모든 후보의 점수를 포함한 전체 재생 관측. 작은 `summary.json`과 원본 후보 배열을 중복 저장하지 않았다.
- `owned_home_tracking_audit.json.gz`·`owned_home_tracking_compact.json`: 실제208개 소유권 확인 표본과 요약.
- `manifest.json`: 입력·진단 도구·실제 플러그인 SHA 및 재생 범위.

`StaticLayer`로 기록 OccupancyGrid를 넣으므로 lethal100/unknown은 유지되지만 중간 soft 비용은 재양자화된다. 누적 Oscillation critic 상태·이전 pruning 이력·이후 실제 움직임은 복제하지 않는다. 모든 비교에서 evaluation을 켜고 short-circuit을 꺼 전체 후보 점수를 수집했다. frozen 입력은 현재 시각으로 다시 발행된다. 실제 지연·AMCL 갱신·OpenCR·바닥 미끄러짐을 재현한 실차 검증이 아니다.
