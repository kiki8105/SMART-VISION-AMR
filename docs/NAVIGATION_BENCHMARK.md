# 같은 경로로 주행 성능 비교하기

이 도구는 이미 켜진 A*/Nav2 시스템의 경로, 차량 TF, 최종 속도 명령을 기록합니다. 기본 동작은 **관찰만** 합니다. 목표 전송은 `--execute`를 넣었을 때만 시작합니다.

## 1. 준비

평소처럼 Gazebo 또는 실차 브링업, 위치 추정, 주행 시스템을 켭니다. 지도에서 초기 위치를 확인합니다. 비교할 때 지도, 출발 위치, 장애물 배치, 경로점은 같게 유지합니다.

새 터미널에서:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/local_setup.bash
```

## 2. 관찰만 하기

시뮬레이션을 평소 조종기/웹으로 주행시키면서 180초 기록합니다. 이 명령은 목표나 정지 명령을 보내지 않습니다.

```bash
python3 scripts/benchmark_navigation.py \
  --domain 76 --use-sim-time --mode ekf_independent \
  --duration 180 \
  --config ros2_ws/src/smart_vision_amr/config/ekf_hardware.yaml \
  --output benchmarks/navigation_observe
```

실차 관찰은 `--domain 16`으로 바꾸고 `--use-sim-time`을 뺍니다.

## 3. 정해진 경로 반복 실행하기

다음 좌표는 **형식 예시**입니다. 현재 지도의 빈 공간 좌표로 바꾸세요. 다른 웹 임무를 종료한 상태에서 사용합니다. 같은 좌표를 두 번 순회합니다.

실차는 운용자가 먼저 **비상정지 해제**를 완료하고 위치 준비 상태를 확인해야 합니다. 도구가 비상정지를 해제하지 않습니다. 다른 조종기가 mux 소유자로 남았다면 웹의 정지·해제 절차로 제어를 `hold`에 두고 임무를 끝낸 뒤 실행합니다. 허용된 실행에서만 고유 순서 번호의 `auto` 요청을 보내고, mux의 소유권·모드 확인 응답을 받은 뒤 목표를 전송합니다. 수동 조종 중이거나 다른 임무가 활성 상태면 제어를 빼앗지 않고 종료합니다.

```bash
python3 scripts/benchmark_navigation.py \
  --domain 76 --use-sim-time --mode ekf_independent \
  --execute --repeat 2 \
  --waypoint '1.0,0.5,0.0' \
  --waypoint '1.0,1.5,1.57' \
  --waypoint '0.0,0.5,3.14' \
  --duration 600 --goal-timeout 90 \
  --config ros2_ws/src/smart_vision_amr/config/simulation.yaml \
  --output benchmarks/navigation_trial_01
```

각 점은 `x,y,yaw`이며 단위는 m, m, rad입니다. `yaw`를 생략하면 0입니다. 실차 실행은 `--domain 16 --allow-hardware`를 쓰고 `--use-sim-time`을 뺍니다. 실행 시험 중 Ctrl+C/제한시간 초과 시 해당 경로를 취소하고 공통 비상정지를 요청합니다. 비상정지 해제는 운용 UI에서 상태 확인 후 수행합니다.

## 결과 읽기

출력 폴더에 다음 파일이 생깁니다.

| 파일 | 내용 |
|---|---|
| `report.json` | 완주율, 소요 시간, 추정 이동거리, 경로 이탈, 정지 비율, 재계획/막힘 횟수, 설정 SHA-256 |
| `samples.csv` | 10 Hz 지도 위치, 현재 경로와의 거리, 목표 거리, 명령 속도, 상태 |
| `commands.csv` | 최종 `/cmd_vel` 수신 기록 |
| `events.json` | 경로 변경, 정지/회복 상태 전환, TF 위치 점프 |

`blocked_transitions`는 막힘·실패 상태로 전환한 횟수입니다. `replans`는 같은 목표에서 경로 좌표열이 변경된 횟수이며 경로 시작점 갱신도 포함합니다. `zero_command_fraction`은 최신 명령이 있었던 위치 샘플 중 전진·회전 속도가 모두 거의 0인 비율입니다. 명령을 못 받은 구간은 정지로 간주하지 않습니다.

`--config`는 지정 파일의 해시를 기록하며 그 파일을 차량에 적용하지 않습니다. 실제 실행한 설정 파일을 지정하세요. `cleanup_requests`에서 종료 시 취소/비상정지 서비스 응답도 확인할 수 있습니다. 실행 시험은 위치 신뢰도 감시가 `autonomous_allowed=true`로 준비된 뒤 목표를 보냅니다.

**이 결과는 TF로 추정한 경로 추종 성능입니다. 실제 절대 위치 오차를 증명하지 않습니다.** EKF의 실제 정확도를 비교하려면 외부 측정/기준점 등 독립 위치 기준이 필요합니다. 가속도·jerk도 명령 수신 간격으로 계산한 값이며 차체의 실제 가속도 측정값이 아닙니다. 바퀴·IMU 교정 후보 추출은 `scripts/calibrate_wheel_imu_reference.py --help`에서 확인합니다.

한 설정으로 적어도 세 번 시험하고 `baseline`, `ekf_independent` 및 튜닝 버전별 완주율, 경로 이탈 p95, 막힘 횟수와 시간을 비교하세요. 여기에는 실차 시험 결과가 미리 들어 있지 않습니다.
