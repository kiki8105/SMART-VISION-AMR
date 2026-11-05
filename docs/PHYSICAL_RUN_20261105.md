# 11월 5일 변경사항과 실차 실행

**11월 5일 후속 실차에서 반복 정지 개선을 적용했고 사용자가 “잘된다”고 확인했습니다.**
검사 중복 제거·명령 전달 분리·C++ 기하 계산 가속을 포함한 코드입니다.
[성능 결과보고서](DRIVING_PERFORMANCE_REPORT_20261105.md)와
[11월 4일 FTA](PHYSICAL_FTA_REPORT_20261104.md)에 근거를 정리했습니다.
전체 반복 임무·장애물 시험의 합격을 의미하지 않습니다. 아래 이전 중단 기록도 보존합니다.

## 변경사항

- 5cm 장애물 셀을 유지하면서 차량 중심 후보만 1cm로 검색하는 짧은 연결 경로.
- 국소 우회 → 검증 후진 → 전체 재계획. 뒤가 막히면 후진하지 않고, 확인된 옆쪽
  연결 경로가 있을 때 기존 정지·선회·추종 절차로 진입.
- 기존 경로 재합류 검사에서 실제 꺾인 점을 보존하고, 같은 실제 차체 교차의 반복 탐색 억제.
- LiDAR 투영의 최신 입력 우선 처리와 clock·TF 분리. 원본 시각·최초 수신은 유지.
- 가까운 경로의 횡오차·방향 오차 비용 추가, 출발 가속·선회 전 감속 보완.
- 측정 odom의 잔류 속도까지 포함한 제동 검사, MUX 중복 계산·수신 병목 보완.
- 계산 중 정지 우선권과 첫 출발·재개 요청 보존, 나중에 승인된 요청의 우선권 유지.

| 현재 설정 | 값 |
|---|---|
| 도메인 / 지도 | 16 / 2026-10-30 교실 지도 |
| 차체 반경 | 22cm |
| 직선 / 선회 / 후진 상한 | 0.22 / 0.10~0.15 / 0.10m/s |
| 출발 선가속 | 0.15m/s² |
| A* 정적 팽창 / Nav2 팽창 | 총 25cm / 총 22cm. 반경에 다시 더하지 않음 |
| 장애물 기억 / scan filter | 켬 / 끔 |
| 카메라 | 후방만 사용 |

바퀴 제동 가정 0.15m/s²는 느린 Gazebo 응답을 위한 값이며 실차 측정값은 아닙니다.
설정은 `classroom_demo_operator_20261104.yaml`과 `motion_mux_hardware.yaml`에 있습니다.

## 시험 결과

| 검사 | 결과 |
|---|---|
| MUX·서비스·제동 집중 검사 | 135개 통과 |
| 최종 DDS 전달 검사 | 23개 재검증 통과 |
| 최종 제어 인계 독립 감사 | 요청 인계 13조건·정지/시각/기한 15조건 통과 |
| 최종 전체 Python 검사 | **사용자 요청으로 중도 중단: 1,478개 통과·1개 실패**, 나머지 미실행 |
| 실패 항목 | metric connector 기능 fixture의 100ms 시간 예산 초과. 중단 후 재검사·기준 완화 없음 |
| 수신 병목 수정 전 이동 상자 시험 | 실패·운용자 중지. 실제 후진 확인, 제거 전 우회·섹터·Home 미완료 |
| 최종 수정 후 Gazebo / 실차 시험 | 미실행 |
| 후속 반복 정지 개선 실차 | 수정본 적용 후 사용자 주행 개선 확인. 전체 반복 임무·장애물 합격 판정은 아님 |
| 후속 코드 공개 전 회귀 검사 | 제동·raw scan 기하·native 계산·MUX 스케줄링 181개 통과 |

[원본 기록과 소스 해시](../benchmarks/driving_parameter_tuning_20261104/recovery_runtime_fixes_20261105/README.md),
[중단한 전체 검사](../benchmarks/driving_parameter_tuning_20261104/recovery_runtime_fixes_20261105/checks_user_interrupted.json)를
보존했습니다. 현재 설정을 실차 최적값 또는 전체 검증 완료로 표시하지 않습니다.

## 실행 전: PC 코드 갱신

PC·로봇의 기존 주행·웹·브링업을 먼저 종료합니다. **PC와 로봇을 모두 갱신·재빌드**해야
합니다. 아래는 이미 설치한 환경 기준이며, 새 PC·로봇은 [클론·첫 설치](TEAM_QUICKSTART.md)를 먼저 완료합니다.

로컬 PC 터미널에서:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
git pull --ff-only origin main &&
./scripts/setup_pc_runtime.sh --no-install
```

의존성 누락 안내가 나오면 `./scripts/setup_pc_runtime.sh`로 설치합니다.

## 터미널 1: 로봇 갱신·브링업

`192.168.0.187`은 기존 로봇 주소입니다. 변경됐다면 로봇에서 `hostname -I`로 확인합니다.

**11월 5일 실제 SSH에서 `~/turtlebot16` 내용을 읽어 확인한 실행 경로입니다.**
이전 인수인계의 `*-fresh-20261102` 폴더도 남아 있지만 현재 스크립트는 아래 경로를 사용합니다.

| 용도 | 실제 사용 경로 |
|---|---|
| 로봇 소스 | `/home/robot/SMART-VISION-AMR` |
| 로봇 런타임 | `/home/robot/smart_vision_runtime_ws_live` |

**기존 `~/turtlebot16`은 수정·재생성하지 않습니다.** 이 스크립트가 후방 카메라와
런타임 경로를 이미 지정합니다. 실행은 다음 두 단계입니다.

```bash
ssh robot@192.168.0.187
```

접속 후 로봇 터미널에서:

```bash
cd ~
./turtlebot16
```

기존 소스의 미커밋 수정은 백업·stash로 보존한 뒤 GitHub 코드로 갱신했습니다.
당시 실행 경로의 런타임 재빌드를 완료했고, 설치된 모듈이 당시 최신 소스 파일을 읽는지
확인했습니다. 기존 `turtlebot16`의 내용과 SHA256은 변경하지 않았습니다.
보존 위치는 로봇의 `~/smart_vision_backups/before-github-4912835-20261105-092802`입니다.
실행 파일의 원본 SHA256은
`2ca2e70df0acad9eade5e99fcbd74f4f85d9ecb0dcfb545eea18bb3e674e393c`입니다.
코드 갱신·빌드와 실제 브링업·주행 시험은 구분합니다. 위 경로 확인 작업 자체는 주행 시험이 아닙니다.

GitHub의 후속 개선을 받으려면 브링업 종료 후 로봇에서도 갱신·재빌드합니다.
기존 `turtlebot16`은 그대로 사용합니다.

```bash
cd ~/SMART-VISION-AMR &&
git pull --ff-only origin main &&
./scripts/setup_robot_runtime.sh --no-install "$HOME/smart_vision_runtime_ws_live"
```

컴파일러·Python 개발 헤더 누락 시 `--no-install`을 빼고 실행합니다. PC도 아래 명령으로
갱신·빌드한 뒤 세 터미널을 시작합니다.

다른 로봇은 기존 파일이 참조하는 경로를 먼저 확인합니다. 위 경로가 없는 장치에서
파일을 임의로 재생성하지 않습니다. 새 설치는 [팀원 퀵스타트](TEAM_QUICKSTART.md)를 따릅니다.

## 터미널 2: 로컬 PC 최신 코드·빌드·주행 런치

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR &&
git pull --ff-only origin main &&
./scripts/setup_pc_runtime.sh --no-install &&
./scripts/check_classroom_deployment.sh --live &&
TURTLEBOT3_TUNING_PARAMS_FILE="$PWD/ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml" \
  ./scripts/run_classroom_navigation_16.sh
```

코드 수신·빌드·점검이 성공하면 주행 런치가 실행됩니다.
`nav_classroom_16`을 추가로 켜지 않습니다. 실패하면 다음 명령은 실행되지 않습니다.

## 터미널 3: 로컬 PC 웹

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

- 해당 PC: `http://127.0.0.1:8790/` / `http://127.0.0.1:8790/inventory`
- 다른 기기: **웹 PC의 Wi-Fi IP**를 사용합니다. PC IP가 `192.168.0.110`이면
  `http://192.168.0.110:8790/` / `http://192.168.0.110:8790/inventory`입니다.
- IP는 웹 PC에서 `hostname -I`로 확인합니다. Gazebo용 `8792`와 구분합니다.

## 웹 운용 순서

1. 실제 홈에 정지하고 후방 카메라가 0번 마커를 보는지 확인합니다.
2. 운용 웹에서 도메인 **16**, 지도·scan·odom·정지 응답을 확인합니다.
3. 비상정지를 유지한 채 **초기 위치 지정 → 등록 홈에서 시작**을 체크·적용합니다.
   홈이 아니라면 실제 X·Y·차체 전방 yaw를 입력합니다. 이미 유효한 AMCL·TF가 있고
   재배치하지 않았다면 반복 입력하지 않습니다.
4. **AMCL·TF 확인됨**과 출발 준비 상태를 확인하고 비상정지를 직접 해제합니다.
5. 재고 웹에서 물품·섹터 요청 → 운용 웹에서 요청 확인 → 실제 적재 또는 빈 차 시험
   확인 → 적재 확인 → 임무 시작.
6. 섹터 표식 확인·5초 대기·다음 섹터·Home 복귀와 최종 정지를 확인합니다.

실제 센서·TF가 없을 때 초기 위치만 입력하거나 비상정지만 해제해 해결하지 않습니다.
종료는 웹에서 정지한 뒤 터미널 **3 → 2 → 1**에서 `Ctrl+C` 순서입니다.
