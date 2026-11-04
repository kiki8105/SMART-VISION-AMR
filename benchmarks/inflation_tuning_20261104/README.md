# Nav2 inflation 비교

최종 실차 설정을 복사한 `profiles/nav_inflation_022.yaml`, `030.yaml`, `040.yaml`은
두 costmap의 `inflation_radius`만 각각 0.22/0.30/0.40m로 다릅니다.
차체 반경 0.22m, A* 정적 팽창 5셀, 속도·기억·복구·안전 조건은 같습니다.
원본 설정과 이전 시험 기록은 변경하지 않습니다. `profile_manifest.json`에 해시를 저장했습니다.

시험 담당자는 각 프로필을 `run_classroom_staged_validation.py`의
`--nav-params-source`로 전달하고, 같은 stage25 전체 웹·재고·pre-arm·이동 장애물·Home 시험을
`--physical-footprint --memory-session` 조건에서 반복합니다.
각 값 1회는 해당 장면 비교이며 반복 신뢰도 검증을 대신하지 않습니다.

기존 안전 timeline 관찰기가 guard 코드·근거·반경·예측 시간·최소 여유·
scan/TF 나이와 실제 가속 제한 상태를 함께 저장합니다. 별도 ROS 관찰기는 필요 없습니다.
세 시험 종료 후 각 실제 stage 디렉터리를 넣습니다.

```bash
python3 benchmarks/inflation_tuning_20261104/summarize_inflation_trials.py \
  --trial 022=/tmp/inflation-022/25_web_inventory_sector3_moving_obstacle \
  --trial 030=/tmp/inflation-030/25_web_inventory_sector3_moving_obstacle \
  --trial 040=/tmp/inflation-040/25_web_inventory_sector3_moving_obstacle \
  --output benchmarks/inflation_tuning_20261104/results.json
```

결과에는 완주·전체 임무 시간·guard 코드/정지 구간·목표/경로/국소/전체 재계획 횟수·
실제 출력의 큰 선회 반전·후진·상자 최소 여유를 기록합니다.

`counter_aggregation_version: 2`부터 전체 탐색·국소 시도 횟수는 **노드 수명 동안 누적되는
카운터의 임무 구간 증가량**입니다. 임무 직전 마지막 표본을 기준으로 삼으며 목표가 Home으로
바뀌거나 같은 상태가 반복되어도 다시 합산하지 않습니다. 전체 탐색에는 최초 탐색과 실패한
탐색도 포함되므로 전부 ‘재계획’ 횟수라고 읽으면 안 됩니다. 기준 표본이 없으면 첫 누적값을
제외한 관측 증가량을 하한으로 표시합니다. 카운터 감소는 재시작과 지연된 과거 상태를
구분할 프로세스 ID가 없으므로 횟수를 `null`로 두고 한계를 기록합니다. 구간 경계는 상태 수신 시각이며 내부 계산
시작 시각을 정밀하게 측정한 값은 아닙니다.

이전 목표별 최대값 합산은 누적치를 중복 계산했습니다. 2026-11-05 반복 clean은
전체 탐색 10→**4**, 재고·이동 장애물 시험은 전체 탐색 9→**5**, 국소 시도 6→**3**으로
정정됩니다. 원본 결과 JSON은 당시 집계 기록으로 보존하고 새 파일명으로만 결과를 만듭니다.
두 집계 CLI는 기존 출력 파일 덮어쓰기를 거부합니다. 다음 검사는 ROS·원본 `/tmp` 없이
보존된 실제 전이 표본과 기준값/목표 변경/초기화 반례를 검증합니다.

```bash
python3 -m unittest discover \
  -s benchmarks/inflation_tuning_20261104 -p test_counter_aggregation.py -v
```

**전체 합격은 `overall_passed`와 `overall_reasons`로 확인합니다.** 이 값은 각 stage의
상위 폴더 `staged_report.json`에서 해당 stage 판정을 읽습니다. `runner_passed`와
Home 완주가 참이어도 장애물 우회·이동·제거 검증이 실패하면 전체 시험은 실패입니다.
전체 판정 파일이나 대응 stage가 없으면 `null`과 사유를 표시하며 완주로 합격을 추정하지 않습니다.
추종 오차는 교체 경로의 발행과 실제 FollowPath 수락 시점이 timeline에서 고유하게
연결되지 않아 산출하지 않습니다. 새 경로를 아직 실행 중인 경로로 오인한 RMS를 보고하지 않습니다.
guard 시간은 관찰 표본 구간을 합한 값이며 표본이 없는 긴 공백을 추정하지 않습니다.

`navigation_status`는 임무 시작~종료 안의 상태별 표본 수와 표본 체류 시간을 기록합니다.
`WAIT_REPLAN`·`CONTROLLER_HOLD`를 따로 표시하며, 각 표본부터 다음 표본까지 최대
0.5초만 합합니다. 첫 표본 전·마지막 표본 후 시간은 추정하지 않고,
`unmeasured_seconds`로 집계되지 않은 시간을 표시합니다. 상태 메시지가 변경될 때만
발행되는 구간은 실제 체류 시간보다 작게 나올 수 있으므로 전체 대기 시간으로 읽으면 안 됩니다.

`actual_command_state`는 실제 `/cmd_vel` 표본을 같은 방식으로 정지·제자리 회전·이동으로
분리합니다. 선속도 절댓값 0.005m/s 이하, 각속도 0.01rad/s 이하일 때 정지 표본으로 셉니다.
출력 정지는 장애물 guard 정지와 같지 않습니다. `navigation_status`, `actual_command_state`,
`guard`를 함께 비교해야 재계획 대기·제어기 대기·라이다 충돌 방지를 구분할 수 있습니다.
