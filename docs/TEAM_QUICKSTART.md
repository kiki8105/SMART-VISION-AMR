# 팀원용 실차 설치·실행

Ubuntu 22.04와 ROS 2 Humble이 설치된 PC 기준입니다. 실차 도메인은 **16**, 지도는
**2026-10-30 교실 지도**입니다. 로봇 브링업·주행·웹은 각각 **한 번만** 실행합니다.
PC 계정이 달라도 `~`는 각자의 홈 폴더입니다. 명령 앞에 프롬프트나 `$`를 붙이지 마세요.
새 PC와 로봇은 아래 첫 설치를 완료한 후 세 터미널을 켜세요. 이미 설치했다면
[매일 실행](#매일-터미널-123)로 이동합니다. 더 자세한 명령·검증 결과는
[운용 참고 문서](TEAM_OPERATIONS_REFERENCE.md)에 있습니다.
기존 사용자는 [PC와 로봇 코드를 모두 갱신](#업데이트종료)해야 합니다.

**차체 크기 설정:** 이 문서의 실차 실행은 사용자가 제자리 회전에서 확인한 최대
돌출 반경 **0.17 m**를 사용합니다. A*·경로 추종·Nav2 설정이 이 값을 공유하며,
5 cm 지도에서 A* 정적 장애물 팽창은 4셀, Nav2 비용 팽창은 0.18 m입니다.
기본 Gazebo Waffle Pi 모델의 외곽은 약 0.237 m이므로 시뮬레이션 스크립트는
별도 임시 설정으로 반경 **0.24 m**, A* 5셀, Nav2 비용 팽창 0.26 m를 사용합니다.
시뮬레이션 설정은 실차 YAML을 덮어쓰지 않으며, 속도 등 나머지 주행 설정은
실차 프로필에서 가져옵니다. 두 모델의 크기가 다른 이유와 검증 범위는
[Home 복귀 정지 분석](PHYSICAL_HOME_STALL_2026-11-04.md)에 적었습니다.

## 처음 한 번: PC 설치

ROS Humble은 먼저 설치되어 있어야 합니다. 기본 개발 도구와 저장소를 준비합니다.
새 PC는 바로 아래 설치 명령을 실행합니다. 기존 작업공간을 교체해야 한다면
[기존 PC 작업공간 교체](TEAM_OPERATIONS_REFERENCE.md)를 먼저 읽으세요.

같은 PC 터미널에서 새 작업공간을 만들고 클론·설치합니다.

```bash
sudo apt update
sudo apt install -y git build-essential python3-colcon-common-extensions
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
./scripts/setup_pc_runtime.sh
./scripts/check_classroom_deployment.sh
```

`setup_pc_runtime.sh`는 실차 PC에 필요한 ROS/Python 의존성을 설치하고 이 저장소의
`smart_vision_amr`만 빌드합니다. Gazebo 설치나 시뮬레이션 소스 다운로드는 필요 없습니다.
의존성이 모두 설치되어 있다면 `./scripts/setup_pc_runtime.sh --no-install`로 빌드합니다.
설치·점검 스크립트는 로봇이나 주행 노드를 시작하지 않습니다.
전체 작업공간을 지운 뒤 Gazebo도 다시 쓰려면 [Gazebo 설치 안내](GAZEBO.md)에
따라 `./scripts/setup_sim.sh`를 별도로 실행합니다. 실차만 사용할 때는 필요 없습니다.

## 처음 한 번: 로봇 설치

이미 최신 `~/turtlebot16`과 로봇 런타임이 준비되어 있으면 [매일 실행](#매일-터미널-123)으로 이동합니다.
새 로봇은 ROS Humble과 기존 `~/turtlebot3_ws`가 있어야 합니다.
로봇 본체 터미널이나 공유기 화면에서 `hostname -I`로 **현재 로봇 IP**를 확인합니다.
PC 터미널에서 아래 명령을 실행하고 확인한 IP와 로봇 계정 비밀번호를 입력합니다.
예전에 사용한 `192.168.0.187`은 고정 주소가 아닙니다.

```bash
read -rp '로봇 IP: ' ROBOT_IP
ssh robot@"$ROBOT_IP"
```

새 로봇이라면 **SSH 접속한 로봇 터미널에서** 바로 아래 명령을 실행합니다.
기존 코드를 처음부터 교체한다면 먼저 [기존 로봇 코드 교체](TEAM_OPERATIONS_REFERENCE.md)를
읽으세요.

```bash
sudo apt update
sudo apt install -y git
cd ~
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd ~/SMART-VISION-AMR
./scripts/setup_robot_runtime.sh
./scripts/check_classroom_deployment.sh --robot
```

로봇용 스크립트는 `~/smart_vision_runtime_ws`에 런타임을 빌드합니다.
기존 `~/turtlebot3_ws`는 유지하며 Gazebo·Nav2는 설치하지 않습니다.
`~/turtlebot16`이 없거나 예전 실행 내용을 사용한다면 로봇에서 한 번 만듭니다.

```bash
cat > ~/turtlebot16 <<'SH'
#!/usr/bin/env bash
exec env ENABLE_FRONT_CAMERA="${ENABLE_FRONT_CAMERA:-false}" \
  "$HOME/SMART-VISION-AMR/scripts/robot_bringup_camera_16_ekf.sh" "$@"
SH
chmod +x ~/turtlebot16
```

이 명령은 실행 파일만 준비합니다. 실제 브링업은 아래 매일 실행 절에서 합니다.
기본 LiDAR는 `LDS-03`이며 실물이 `LDS-02`면 `LDS_MODEL=LDS-02 ~/turtlebot16`을 사용합니다.
카메라 경로가 다른 로봇은 [로봇 런타임 설정](hardware_ekf.md)을 확인하세요.

## 매일: 터미널 1·2·3

처음이라면 [PC 설치](#처음-한-번-pc-설치)와 [로봇 설치](#처음-한-번-로봇-설치)를
끝낸 다음 진행합니다. **아래 순서대로 새 터미널 세 개**를 열어 실행하고 그대로
켜 둡니다. 터미널 2의 3분 검사가 실패하면 주행 런치는 시작되지 않습니다.

**터미널 1 · PC에서 로봇에 SSH 접속, 브링업**

```bash
read -rp '로봇 IP: ' ROBOT_IP
ssh -t robot@"$ROBOT_IP" '$HOME/turtlebot16'
```

**터미널 2 · 로컬 PC에서 3분 센서 검사 후 주행**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
source ./scripts/dds_env.bash ""
/usr/bin/python3 scripts/soak_robot_opencr_16.py --duration 180 --json-out "$HOME/smart_vision_logs/opencr_soak_after_reboot.json" &&
./scripts/check_classroom_deployment.sh --live &&
./scripts/run_classroom_navigation_16.sh
```

**터미널 3 · 로컬 PC에서 웹**

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh
```

웹을 실행한 PC의 브라우저에서는 `http://127.0.0.1:8790/`(로봇 운용),
`http://127.0.0.1:8790/inventory`(재고 요청)을 엽니다. 다른 기기에서는
`127.0.0.1` 대신 **웹 PC의 Wi-Fi IP**를 사용합니다. 이후 화면에서
[초기 위치를 확인하고 출발](#웹-초기-위치와-출발)합니다.
실차 웹은 기본적으로 **후방 카메라 영상만 표시·구독**합니다.

## 웹: 초기 위치와 출발

1. 로봇이 실제 등록 홈에 정지하고 **후방 카메라가 0번 마커**를 보는지 확인합니다.
   홈이 아니면 등록 홈 좌표를 사용하지 않습니다.
2. 웹 상단 **출발 준비 → 막힌 항목·해결 방법**에서 실제 `/map`, 최신 `/scan`·`/odom`,
   로봇 정지 응답과 AMCL 상태를 확인합니다. 비상정지 상태가 아니라면 웹의
   **비상정지**를 누르고 로봇 측 `motion_mux`의 속도 0 확인 응답을 기다립니다.
3. 현재 `AMCL · TF 확인됨`이 유지되면 좌표를 다시 입력할 필요가 없습니다.
   위치가 없거나 로봇을 재배치했다면 **비상정지 상태에서** 초기 위치를 지정합니다.
   실제 홈이라면 **초기 위치 지정 → 등록 홈에서 시작** 확인란을 체크하고 적용합니다.
   홈이 아니라면 **X·Y·차체 전방 yaw(°) 직접 입력**으로 실제 위치를 적용합니다.
   좌표 적용은 비상정지를 해제하거나 로봇을 움직이지 않습니다.
4. **AMCL · TF 확인됨**과 위치 신뢰도 회복을 확인한 다음, 비상정지를 **직접 해제**합니다.
   확인란·적용 버튼이 회색이면 [차단 원인](TEAM_OPERATIONS_REFERENCE.md)을 점검합니다.
   센서·지도가 끊겼을 때 좌표만 다시 입력해도 해결되지 않습니다.
   라이다·저장 지도 대조율은 가벽 이동이나 임시 장애물의 영향을 받는 **참고값**입니다.
   대조율이 0%까지 낮아져도 그 수치만으로 출발·수동 초기 위치 지정을 막지 않습니다.
   설정 지도와 실제 `/map`의 일치, 최신 라이다·오도메트리·TF, AMCL 위치와 비상정지는
   별도로 확인합니다. 낮은 대조율이 정확한 위치를 증명하지는 않으므로 지도 위 차량
   위치·방향을 직접 확인하세요.
   테스트베드에서는 AMCL 위치 표준편차가 0.35~0.50m일 때 다른 검사에 문제가
   없고 2초간 안정적이면 **위치 주의·속도 50% 제한**으로 출발할 수 있습니다.
   0.50m 초과는 초기 위치 획득을 계속 기다립니다.
5. 재고 요청자가 `/inventory`에서 섹터를 접수하고, 운용자가 적재를 확인한 뒤
   임무를 시작합니다. 초기 위치 지정·비상정지 해제만으로 자동 출발하지 않습니다.

로봇을 들어 옮기거나 전원을 다시 켰다면 이전 AMCL 추정을 그대로 사용하지 마세요.
웹의 `HOME_REACHED`는 허용 오차 내 도착 표시이며 실제 등록 좌표에 놓였다는 증거가
아닙니다. 상세한 재출발·자세 초기화 절차는 [운용 참고 문서](TEAM_OPERATIONS_REFERENCE.md)에 있습니다.

## 막힐 때

| 화면 상태 | 먼저 확인할 것 |
|---|---|
| 홈 확인란이 회색 | 웹 연결·등록 홈 좌표를 확인하고, 진행 중인 임무·조이스틱·위치 확인을 종료 |
| 확인란은 되지만 적용 버튼이 회색 | 실제 정지·비상정지·로봇 `motion_mux` 정지 응답, 도메인 16, 일치하는 `/map`, 최신 `/scan`·`/odom`, 활성 AMCL |
| `최신 map→base TF 대기` | AMCL 초기 위치 적용 여부와 AMCL·센서·PC/로봇 시각 |
| `설정 지도와 다른 /map` | 10/27 지도나 SLAM 중복 실행. 10/30 주행·웹을 한 번씩 실행 |
| 위치·방향 추정 오차 큼 | 실제 위치·지도·LiDAR 일치와 새 AMCL 추정 확인. 비상정지 기준 완화 금지 |

웹의 **출발 준비**에 나온 실제 차단 이유를 확인하세요. 읽기 전용 ROS·웹 진단 명령은
[운용 참고 문서](TEAM_OPERATIONS_REFERENCE.md)에 있습니다.

## 업데이트·종료

**로컬 PC**, 주행을 종료한 상태에서:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
git pull --ff-only
./scripts/setup_pc_runtime.sh --no-install
```

**SSH 접속한 로봇**, 브링업을 종료한 상태에서:

```bash
cd ~/SMART-VISION-AMR
git pull --ff-only
./scripts/setup_robot_runtime.sh --no-install
```

PC 주행·웹과 로봇 브링업을 종료한 상태에서 각각 갱신합니다. 의존성 추가 안내가
있으면 `--no-install`을 빼고 설치합니다. 다시 실행해야 새 코드가 적용됩니다.
종료할 때는 웹에서 정지한 뒤 터미널 **3 → 2 → 1** 순서로 `Ctrl+C`를 누릅니다.
상시 웹 서비스·새 SLAM 지도 적용 절차는 [운용 참고 문서](TEAM_OPERATIONS_REFERENCE.md)에 있습니다.
