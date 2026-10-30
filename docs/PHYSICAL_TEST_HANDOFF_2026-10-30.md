# 2026-10-30 실차 시험 인수인계

**실차 검증은 미통과이며, 구동기 통신 장애로 중단했습니다.** 사용자가 차체 정지를
확인했습니다. 병진 경유점·배송 임무는 실행하지 않았고, AMCL 위치 추정도 수렴하지 않았습니다.
시뮬레이션 통과 결과와 이번 실차 결과를 구분해야 합니다.

| 비교 조건 | 실제 확인 결과 |
|---|---|
| 기본 Fast DDS | 같은 Pi의 안전 메시지가 mux에서 약 1~3.7초 늦게 처리됐습니다. 같은 메시지는 PC 관측기에 즉시 도착한 사례가 있어 PC·로봇 시계 차이만으로 설명되지 않습니다. |
| Fast DDS `ASYNCHRONOUS` | 제어 수신 지연 최대 10.179초가 남았고, 위치 추정 시작 RPC도 완료되지 않았습니다. 해결로 채택하지 않았습니다. |
| 선택 시험 Cyclone DDS | 이번 시험에서 지도·서비스·정지 해제가 진행됐고 일반 위치 확인으로 실제 약 188.8° 회전했습니다. 제어 지연 거부 2회·최대 약 1.28초도 관측돼 통신 문제가 모두 해결됐다고 볼 수 없습니다. |

회전 중 `DynamixelSDKWrapper: no status packet` 오류가 10회 반복된 뒤
`turtlebot3_ros`가 `stack smashing detected`, 종료 코드 `-6`으로 종료됐습니다.
원시 바퀴·IMU·배터리 입력이 끊겼고 센서 감시가 비상정지를 잠갔습니다.
마지막 전압은 약 **10.97V**였으나 배터리가 장애 원인이라는 증거는 없습니다.
AMCL 위치 표준편차는 약 **0.485m**, 라이다·지도 일치율은 **0.3875**로 완료 기준에 못 미쳤습니다.

원시 입력 소실 뒤에도 EKF가 새 시각의 `/odom`에 이전 각속도 약 `0.196rad/s`를
예측값으로 발행한 현상이 있습니다. 새 `/odom` 시각만으로 센서와 정지를 검증하면 안 됩니다.
원시 입력 소실 시 기존 센서 감시는 정지를 유지합니다. 추가로, 포함된 OpenCR·라이다 등
필수 프로세스가 예상 밖에 종료되면 전체 로봇 runtime을 종료하도록 수정했습니다.
관련 회귀 테스트 264개, 패키지 빌드, 실제 로컬 모의 프로세스 종료 시험을 통과했고 Pi에도 반영했습니다.
수정 뒤 실차 하드웨어를 다시 구동해 시험하지는 않았습니다.

근거는 [실차 기록 디렉터리](../benchmarks/physical_startup_20261030/),
[기본값 기록](../benchmarks/physical_startup_20261030/baseline.json),
[비동기 비교](../benchmarks/physical_startup_20261030/async_result.json),
[Cyclone 결과](../benchmarks/physical_startup_20261030/cyclone_result.json),
[회전 중 상태](../benchmarks/physical_startup_20261030/cyclone_localization_attempt1.json)에 있습니다.
PC에 보관한 로봇 로그는 `~/smart_vision_logs/robot_bringup_cyclone_timing_20261030.log`입니다.
수정 전 Pi 모듈 백업은 로봇의 `~/smart_vision_backups/20261030_handoff/`에 있습니다.
PC 수정 전 기준은 Git 커밋 `81c785f`이며, 이번 변경은 별도 커밋으로 보존합니다.

## 재시험 전제

- 충전된 전원과 OpenCR USB·Dynamixel 연결을 확인하고, `no status packet` 반복이나
  드라이버 종료가 없어야 합니다. 전압 수치 하나로 원인을 확정하지 않습니다.
- 원시 바퀴·IMU·배터리 수신을 확인하고, 프로세스 종료 보완 패치를 적용한 실차 재시험을 진행합니다.
- 이전 로봇 브링업과 PC 주행 실행을 종료한 뒤 최신 코드를 양쪽에 적용합니다.
  이번 시험에서 시작한 로봇 실행과 PC 주행 런치는 모두 종료된 것을 확인했습니다.
- 비상정지를 강제로 해제하거나 실제 위치와 다른 Home 초기 위치를 입력하지 않습니다.
  센서·제어권·정지 조건이 모두 통과한 뒤 운용자가 다음 동작을 선택합니다.
- 이번 시험은 제자리 회전만 했으므로 위치를 홈으로 확정하지 않습니다. 충전·점검 뒤
  실제 등록 홈에 놓고 후방 카메라가 0번 마커를 보도록 방향을 다시 맞춥니다.

Cyclone은 이번 비교 시험의 **선택 설정**입니다. PC와 Pi에는 공식 ROS 패키지를 설치했지만,
프로젝트 기본 RMW·QoS·의존성 설치 정책은 바꾸지 않았습니다. 아래 명령은 전제 조건을
충족한 뒤 재시험할 때 사용하며, 세 실행 모두 같은 RMW와 도메인 16을 사용합니다.

## 세 터미널 재시험 절차

**터미널 1 — PC에서 로봇으로 SSH 접속한 뒤 실행**

```bash
ssh robot@192.168.0.187
cd ~/SMART-VISION-AMR
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
SMART_VISION_TIMING_DIAGNOSTICS=true \
./scripts/robot_bringup_camera_16_ekf.sh
```

원시 센서와 구동기 로그가 정상인지 확인합니다. 후방 카메라는 필수이며 전방 카메라는
기본으로 끕니다. 드라이버 오류가 재발하면 재설정·회전·임무를 진행하지 않고 로그를 보존합니다.

**터미널 2 — SSH가 아닌 로컬 PC에서 실행**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
./scripts/check_classroom_deployment.sh --live && \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
./scripts/run_classroom_navigation_16.sh
```

점검 실패 시 주행 노드는 시작하지 않습니다. 지도·AMCL·A*·Nav2 준비와 센서 상태를 확인합니다.

**터미널 3 — 로컬 PC에서 웹 실행**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./scripts/run_classroom_web_16.sh
```

이미 같은 PC에서 최신 웹이 실행 중이면 중복 실행하지 않습니다. 인수인계 시점에는
`http://192.168.0.110:8790/` 웹을 진단용으로 남겼습니다. 운용 PC에서는
`http://127.0.0.1:8790/`을 사용합니다. `192.168.0.187`은 로봇 SSH 주소이며,
웹 주소에는 웹을 실행한 PC의 IP를 사용합니다.

시작만으로 위치 초기화나 임무가 실행되지는 않습니다. 먼저 준비 상태와 정지 응답을 확인하고,
일반 위치 확인·실제 초기 위치 입력·등록 Home 초기화 중 실제 위치에 맞는 절차를 선택합니다.
회전 시험이 성공해도 위치 수렴과 실제 정지를 별도로 확인한 뒤 병진 임무를 검증해야 합니다.
