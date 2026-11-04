# 2026-11-04 교실 실차 주행 검토

근거: [공유 작업 세션](https://chatgpt.com/s/cx_6aeab69c7c8081918c0ebc56e1a86740),
현장 `navigation-reset-intent.log`와 `operator-two-roundtrips-review.json`.
초기 출발 실패부터 Sector 2 왕복까지의 세션을 대상으로 한다. 새 후진 소스의
실차 주행 기록은 없다.

| 순서 | 실차에서 확인된 결과 |
|---|---|
| 초기 기동 | LiDAR·odom은 들어왔지만 AMCL map→base TF가 오래되어 위치가 고정됐다. `tf2` 구성 버전을 맞춘 뒤 정지 상태의 짧은 관측에서는 TF 시각이 회복됐다. |
| 초기 A*·저속 시험 | 경로는 생성됐으나 TF 오류로 바퀴가 움직이지 않은 시험이 있었다. 재시험의 위치 검증 단절도 주행을 중단시켰다. 이 단계는 임무 성공으로 볼 수 없다. |
| 첫 웹 Sector 2 시험 | 배송과 후방 마커 확인은 관찰됐다. 복귀 중 선회 여유 부족으로 멈춰 사용자가 로봇을 조금 이동시킨 뒤 재개했고, 이후 배터리 전원 중단으로 왕복 완료를 확인하지 못했다. |
| 배터리 교체 뒤 왕복 | Sector 2 배송 2건과 Home 복귀 2회가 기록됐다. 아래 표의 경로 실패·시각 불일치·최종 위치 정합 문제가 남았다. |
| 이후 소스 설치 | 10~30cm 후진 후보와 출발점 보정이 로봇에 복사됐으나, 재기동한 실차 후진 또는 소화기 장애물 주행 기록은 없다. |

| 관찰 | 결과와 해석 |
|---|---|
| Sector 2 배송 및 복귀 | 배송 2건과 Home 복귀 2회가 임무 기록에 남았다. 첫 복귀는 좁은 선회 공간에서 정지해 수동 위치 조정 뒤 이어졌다. |
| A* 경로 없음 | 17회. 그중 `static_start=True`가 8회, 라이다 장애물 관측 `observed_raw=0`인데도 경로가 없던 경우가 3회다. 모든 17회에서 `remembered_raw=0`이므로 이 기록의 경로 없음은 지속 기억 자체 때문은 아니다. |
| 동적 장애물 | 관측 장애물 팽창으로 한 번에 약 300셀 이상 막힌 시점이 있었다. 배치·제거 시각이 없는 관측값을 특정 소화기로 단정할 수 없다. |
| 주행 시각 정합 | footprint 취득 시각이 마지막 odom 이력보다 약 49~100ms 앞선 경고가 5회 있었다. 이때 FollowPath 취소가 발생했다. |
| 후진 복구 | 기록된 주행에는 후진 요청·실행 로그가 없다. 10~30cm 후진 후보와 시각·정지·차체 여유 검사를 추가한 소스는 왕복 시험 **뒤** 배포됐고 재시작·실주행 검증은 아직 없다. 따라서 후진 구동 실패로 확정할 수 없다. |
| 종료 후 위치 | 임무는 `HOME_REACHED`이나 최종 상태의 지도 위치는 등록 Home과 약 0.82m/151° 다르고 위치 신뢰도도 낮다. 완료 기록과 최종 위치 정합 검증은 구별해야 한다. |

## 이번 조정

교실 실차 프로필의 `robot_radius`는 이미 0.22m다. 5cm 지도에서 정적·기억
장애물은 5셀(축 방향 0.25m, 사각 팽창)로 막고, 후진은 실제 모델을 고려해
최소 0.24m 차체 반경과 별도의 여유를 요구한다. 저장 지도에서 문제의 시작점
일부는 장애물 셀 면까지 약 0.225m밖에 남지 않는다. 0.22m 원만으로 그 셀을
열면 A* 경로는 생기지만 회전 차체·부착물·측위 오차까지 통과를 보장할 수 없다.

그래서 하드 충돌 한계는 유지하면서, 국소 우회의 **추가 라이다 관측 여유**를
0.02→0m, A*의 소프트 선호 여유를 0.45→0.35m, Nav2 국소 비용 영향 범위를
1.0→0.55m로 줄였다. 라이다 관측에는 여전히 0.22m 차체 반경 + 0.02m 물리 여유와
지도 셀 안에서의 위치 불확실도가 반영된다. 정적 5셀 팽창과 후진 약 0.275m
요구 여유는 변경하지 않았다. 실차에서 실제 사용한 별도 데모 Nav2 설정에도
같은 국소 비용 영향 범위를 백업 후 적용했다. 로봇 주행 프로세스는 정지 상태로
유지했으며, 다음 실행부터 이 값들이 적용된다. 실차 통과 성능은 아직 검증되지 않았다.

후진이 시작되지 않는 첫 조건은 `REVERSE_REJECT[사유]`로 최대 5초에 한 번
기록하도록 했다. 경로 없음이 반복될 때 이 코드와 당시 `No path` 행,
`/smart_vision_amr/planner_status`, 후방 `/scan`, odom, FollowPath 결과를
같이 보아야 원인을 특정할 수 있다. 예를 들어 `TRAIL_UNVERIFIED`는 연속 전진
기록 부재, `NOT_STOPPED`는 정지 확인 미완료, `NO_SAFE_EGRESS`는 물리 여유 또는
후진 지점에서 목표로 이어지는 경로 부재를 뜻한다. 로그만으로 구분되지 않는
`NO_SAFE_EGRESS` 내부 조건은 후보 검사 계측을 더해야 한다.

AMCL 위치 표준편차 **0.446m**는 기존 0.35m 경고 기준을 넘어, 초기 획득의
2초 연속 양호 판정을 계속 막았다. 테스트베드에서는 위치 경고만 남고 다른
센서·지도·TF·라이다 일치도가 정상일 때 **0.50m까지 초기 획득**을 허용한다.
0.35m 이상이면 경고를 유지하고 motion mux가 자율주행 속도 명령을 절반으로
제한한다. 0.80m 이상은 기존처럼 중대 오차이며 출발할 수 없다. 라이다 회전
위치 확인의 AMCL 오차 한계도 위치 0.50m·방향 0.40rad로 맞췄다. 이 완화는
실차의 실제 위치 정확도가 개선됐다는 뜻이 아니며, 차량 반경 0.22m보다 큰
불확실성에서 좁은 통로 통과를 보증하지 않는다.

## 이번 검증

교실 하드웨어 노드 설정을 그대로 읽는 격리 Gazebo 도메인 176의 전체 웹·주행
시험 `06_web_sector3_moving_obstacle`을 통과했다. 비상정지 해제와 pre-arm
검사 후 웹에서 Sector 3 임무를 시작했고, 주행 중 장애물을 추가하자 라이다에
나타났으며 경로가 바뀌었다. 장애물 제거 후 관측 표시가 사라졌고, Sector 3의
후방 마커 확인·5초 대기와 Home 복귀까지 수행했다. 결과는
`/tmp/smart_vision_web_obstacle_tune_20261104/06_web_sector3_moving_obstacle/acceptance.json`에
남겼다. 최종 상태 `HOME_REACHED`, Home 거리 0.047m, 안전 위반 0건이다.
시험은 시뮬레이션이며 새 후진 진입의 실차 성공을 입증하지 않는다.

실차 데모와 같은 선속도 상한 0.24m/s를 쓰되 회전 상한을 0.2에서 0.5rad/s로
조정한 [검증 프로필](../ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml)로
`25_web_inventory_sector3_moving_obstacle`도 통과했다. 재고 웹 요청 →
웹 초기 위치 지정·비상정지 해제·pre-arm → 적재 확인·출발 → 주행 중 이동 장애물
우회 → Sector 3 후방 마커 확인·5초 대기 → 자동 Home 복귀를 한 번에 시험했다.
AMCL 출발 기준 조정 뒤 같은 전체 시험을 다시 통과했으며 최종 증거는
`/tmp/smart_vision_20261104_inventory_amcl_final/staged_report.json`이다.
최종 상태 `HOME_REACHED`, Home 거리 0.058m, 장애물 활성 중 우회와 제거 후
라이다 표시 해제 확인, 안전 위반 0건이다. 이 시험은 격리 Gazebo 도메인 176에서
수행돼 실제 로봇을 움직이지 않았다. 후진이 필요한 배치는 이번 합격 경로에
없었으므로 후진 기동의 실차 합격으로 해석하지 않는다.

장애물 기억을 켠 `24_web_memory_sector1_to_sector2_removal` 전체 시험도
`/tmp/smart_vision_20261104_memory_stage24_retry/staged_report.json`에서
합격했다. Sector 1에서 발견한 상자를 기억한 채 Sector 2 첫 경로를 만들었고,
그 경로는 상자와 0.033m 떨어졌다. 상자를 치우자 빈 공간 관측으로 기억이
해제되고 새 경로가 나왔다. Sector 1→2→3과 Home 복귀를 완료했고 최종 Home
거리 0.043m, 안전 위반 0건이었다. 이 역시 실차 기억 기능 검증은 아니다.

다른 PC에서 같은 시험을 재실행하려면 ROS Humble·Gazebo·Nav2를 준비하고 저장소
루트에서 아래 명령을 쓴다. 시험은 실제 로봇 도메인 16과 분리된 176을 사용한다.

```bash
python3 scripts/run_classroom_staged_validation.py \
  --run 25_web_inventory_sector3_moving_obstacle \
  --nav-params-source ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml \
  --output /tmp/classroom_web_inventory_validation \
  --leg-timeout 240 --obstacle-hold-timeout 0
```

실차 웹 기본값은 후방 카메라만 구독·표시한다. 시뮬레이션의 두 카메라 표시는
Gazebo 마커 시험용 설정이다. 2026-11-04 이 문서를 마지막으로 갱신할 당시
로봇 `192.168.0.187`에는 SSH가 연결되지 않아 새 프로필과 최신 웹·주행 소스의
실차 반영 및 재시험은 아직 확인되지 않았다. 아래 데모 실행 명령은 **배포 완료 후**
사용해야 한다.

## 다음 실차 확인

로봇의 실제 위치와 방향을 먼저 확인한 뒤 한 PC·한 로봇 스택으로 시작한다.
첫 시험은 열린 통로에서 Sector 2 왕복을 반복하고, 다음에는 위치가 기록된
이동 장애물을 놓는다. 경로 없음 → 국소 우회 또는 후진 요청 → 후진 거리 →
재계획 → 재출발의 시각과 실제 이동을 함께 기록한다. 통로의 실측 폭과
돌출 웹캠·브래킷 치수가 확인되기 전에는 정적 지도/후진의 물리 여유를 더 줄이지 않는다.

## 현재 실차 데모 실행

2026-11-04 설치 상태에서 **PC IP는 `192.168.0.110`**, **로봇 IP는
`192.168.0.187`**이다. ROS 도메인은 **16**이다. 이 데모는 네 프로세스를
모두 로봇에서 실행하며, PC와 휴대전화는 로봇의 웹 서버에 접속한다. PC용
3터미널 퀵스타트와 혼용하거나 브링업·SLAM·주행 런치를 중복 실행하지 않는다.
로봇 IP가 바뀌면 로봇에서 `hostname -I`로 확인해 아래 주소를 바꾼다.

### 실차 코드 반영 · 최초 1회

로봇의 기존 브링업·카메라·주행·웹 터미널을 모두 종료한 뒤 **PC**에서 실행한다.
이 단계는 현재 PC 작업본을 로봇으로 복사하며, 로봇의 기존 데모 Nav2 설정은
시간이 붙은 백업으로 남긴다. `rsync`나 빌드가 실패하면 아래 주행 터미널로
넘어가지 말고 실패 이유를 확인한다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
rsync -az --exclude '.git/' --exclude 'build/' --exclude 'install/' \
  --exclude 'log/' --exclude '__pycache__/' --exclude '*.egg-info/' ./ \
  robot@192.168.0.187:/home/robot/SMART-VISION-AMR-fresh-20261102/
ssh robot@192.168.0.187 'cd /home/robot/SMART-VISION-AMR-fresh-20261102 && SMART_VISION_RUNTIME_WORKSPACE=/home/robot/smart_vision_runtime_ws_fresh_20261102 ./scripts/setup_robot_runtime.sh --no-install'
ssh robot@192.168.0.187 'cp -a /home/robot/smart_vision_demo_20261104_domain16/nav2_demo_operator.yaml /home/robot/smart_vision_demo_20261104_domain16/nav2_demo_operator.yaml.bak-$(date +%Y%m%d-%H%M%S) && cp /home/robot/SMART-VISION-AMR-fresh-20261102/ros2_ws/src/smart_vision_amr/config/classroom_demo_operator_20261104.yaml /home/robot/smart_vision_demo_20261104_domain16/nav2_demo_operator.yaml && sha256sum /home/robot/smart_vision_demo_20261104_domain16/nav2_demo_operator.yaml'
```

마지막 SHA-256 값은
`be7c858b4050d1e6f660dbab1d20b075a1566b75cbbd0bbe72aef7b252d0ff51`이어야
한다. 이 프로필은 가제보에서 확인된 0.24m/s 직선 상한과 0.5rad/s 회전 상한을
담고 있다. 실제 로봇에서 그 값을 시험해 합격한 것은 아니다.

각각 **PC의 새 터미널**에서 다음을 순서대로 실행하고 터미널을 유지한다.
로봇 계정의 SSH 비밀번호는 프롬프트에 입력한다.
터미널 1~3의 `smart_vision_demo_20261104_domain16` 스크립트는 이 로봇에
이미 만들어 둔 현장용 파일이며 GitHub 클론만으로 생성되지 않는다. 파일이
없다면 [팀원 퀵스타트](TEAM_QUICKSTART.md)의 표준 PC·로봇 절차를 사용한다.

**터미널 1 · 로봇 브링업**

```bash
ssh -t robot@192.168.0.187 '/home/robot/smart_vision_demo_20261104_domain16/hardware.sh'
```

**터미널 2 · 후방 카메라**

```bash
ssh -t robot@192.168.0.187 '/home/robot/smart_vision_demo_20261104_domain16/camera.sh'
```

**터미널 3 · 저장 지도·AMCL·주행**

```bash
ssh -t robot@192.168.0.187 '/home/robot/smart_vision_demo_20261104_domain16/navigation.sh'
```

**터미널 4 · 웹 서버**

```bash
ssh -t robot@192.168.0.187 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp SMART_VISION_WEB_WORKSPACE=/home/robot/smart_vision_runtime_ws_fresh_20261102 SMART_VISION_WEB_LOCALHOST_ONLY=1 SMART_VISION_CLASSROOM_FRONT_CAMERA_TOPIC= /home/robot/SMART-VISION-AMR-fresh-20261102/scripts/run_classroom_web_16.sh'
```

터미널 4는 저장소에 포함된 스크립트를 직접 실행해 전방 토픽을 비우고 후방 영상만
표시한다. 기존 현장 웹이 다른 `SMART_VISION_CLASSROOM_CONFIG` 파일을 썼다면
재고 기록을 이어 쓰도록 같은 경로를 터미널 4의 환경변수로 전달해야 한다.

브라우저 주소는 로봇 IP 기준 [운용 웹](http://192.168.0.187:8790/)과
[재고 요청 웹](http://192.168.0.187:8790/inventory)이다. 이 데모에서
`192.168.0.110:8790`은 접속 주소가 아니다. 로봇의 현재 등록 Home은
**x=-0.188431, y=-0.220497, yaw=-0.027514rad**이며 예전
**(-0.11, -0.79)은 다른 지도 기준의 오래된 좌표**다. 로봇이 실제로 등록
Home에 정지해 후방 카메라로 0번 마커를 바라보는 경우에만 등록 홈 초기 위치
단축 입력을 사용한다. 홈 근처이기만 하거나 로봇을 들어 옮긴 뒤라면 실제
위치와 방향을 다시 확인한다.

### 웹 운용 순서

1. 운용 웹의 **출발 준비 → 막힌 항목·해결 방법**에서 지도·최신 `/scan`·
   `/odom`·AMCL/TF·정지 제어 상태를 확인한다. 필요하면 **비상정지**를 누르고
   실제 로봇의 정지를 확인한다. 센서나 통신이 끊겼다면 위치만 다시 입력해도
   복구되지 않는다.
2. 로봇이 **정확한 등록 Home**에 정지하고 후방 카메라가 0번 ArUco를 볼 때만
   **초기 위치 지정 → 등록 홈에서 시작** 확인란을 체크하고 **등록 홈으로 초기
   위치 지정**을 누른다. 그 외에는 **홈이 아니면 X·Y·방향 직접 입력**에서
   실제 지도 좌표와 **차체 전방** yaw(°)를 넣고 정지 확인란을 체크한 다음
   **AMCL 초기 위치 적용**을 누른다. 후방 카메라가 보는 방향은 차체 전방의
   180° 반대다. AMCL·TF가 이미 정상이고 로봇을 옮기지 않았다면 재입력할
   필요가 없다.
3. **출발 준비**에서 비상정지를 제외한 센서·지도·위치 차단 항목이 사라지고
   위치 추정이 안정되면 **비상정지 해제 → 확인·해제**를 누른다. 초기 위치
   지정이나 비상정지 해제만으로
   자동 출발하지 않는다.
4. 재고 관리자 웹 `/inventory`에서 사용자 번호 1~99와 ROS Domain ID **16**으로
   연결한다. **부족한 위치**, **필요한 물품**, **필요 수량**을 입력하고
   **재고 부족 요청 보내기**를 누른다. 운용 웹에서 요청과 목적지를 확인한
   뒤 실제 적재 또는 빈 차 시험 준비를 마치고 **접수된 요청의 물품을 로봇에
   적재했습니다.** 확인란을 체크해 **적재 완료 · 출발**을 누른다.
5. 지도에서 로봇·현재 경로·목적지를 관찰한다. 섹터에서 해당 ArUco를 확인하면
   인수 대기 후 다음 접수 목적지로 가며, 모든 요청을 처리하면 저장된 Home으로
   자동 복귀한다. **홈 도착**이나 `HOME_REACHED` 표시는 허용 오차 내 임무
   상태이므로 실제 등록 좌표·방향은 따로 확인한다.

**일시정지**는 현재 위치에서 멈추고 임무를 보존하며 **배송 재개**로 이어간다.
**홈 복귀**와 **임무 취소 · 복귀**는 미완료 요청을 접수 상태로 되돌리고 Home
이동을 요청한다. **비상정지**는 즉시 정지시키며 해제해도 자동으로 임무를
재개하지 않는다. 재고 관리자 웹의 **전체 배송 기록 초기화 · 홈 복귀**는
완료·취소 기록 삭제를 포함하므로 일반 복귀 버튼으로 사용하지 않는다.

현재 `navigation.sh`에는 `memory_enabled:=false`가 지정되어 있다. 장애물
지속 기억을 검증한 프로필로 간주하지 않는다. 종료할 때는 웹에서 임무를
정지시키고 로봇의 정지를 확인한 뒤 터미널 **4 → 3 → 2 → 1** 순서로
`Ctrl+C`를 누른다.
