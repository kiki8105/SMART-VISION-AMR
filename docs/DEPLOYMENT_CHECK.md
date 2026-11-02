# 실차 배포 점검

정상 구성은 로봇의 센서·EKF·안전 감시·motion mux와 PC의 지도·AMCL·경로 계획·웹입니다.
실차는 도메인 16과 네트워크 DDS를 사용합니다. Gazebo 도메인 76은 별개입니다.

## 실행 전 읽기 전용 점검

**PC**, 클론과 `setup_pc_runtime.sh` 빌드 후:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/check_classroom_deployment.sh
```

**로봇**, `setup_robot_runtime.sh` 빌드 후 SSH 접속한 터미널에서:

```bash
cd ~/SMART-VISION-AMR
./scripts/check_classroom_deployment.sh --robot
```

점검은 자식 셸에서 Humble과 선택 작업공간만 불러오고, `smart_vision_amr`가 그
작업공간에서 선택되는지, 필요한 ROS 패키지와 실행 파일이 있는지 확인합니다.
상속받은 도메인을 표시한 뒤 점검에만 `ROS_DOMAIN_ID=16`, `ROS_LOCALHOST_ONLY=0`을 사용합니다.
UTC 시각과 시스템의 시간 동기화 상태도 읽습니다. 시스템 시계·SSH·서비스 설정을 바꾸지 않습니다.
정적 점검은 DDS 노드를 만들지 않고 로봇으로 연결하지 않습니다.

터미널 1에서 로봇을 시작한 뒤 **PC**에서 다음으로 센서·발행자까지 확인합니다.

```bash
./scripts/check_classroom_deployment.sh --live
```

약 8초 동안 DDS 그래프와 센서 메시지를 관찰하며 명령·자세·서비스 요청을 보내지 않습니다.
로봇의 독립 EKF·mux·위치 감시 발행자와 신선한 joint/IMU/scan/odom 입력을 요구합니다.
센서 시각과 PC 시각의 차이가 크면 시계 차이·센서 지연·통신 지연을 구분해 점검해야 합니다.
PC/로봇의 `date -u`와 동기화 상태를 비교하세요. 이 도구는 시계를 자동 수정하지 않습니다.
정적 통과만으로 로봇 연결이나 센서 수신이 확인되지는 않습니다.

다른 경로에 빌드했다면 `--workspace /절대/작업공간`을 사용합니다.
로봇의 기존 제조사 작업공간 경로는 `TURTLEBOT3_WORKSPACE`로 지정합니다.

위치 감시의 `dynamic_tf_best_effort: true`는 동적 `/tf`의 구독 방식만 변경합니다.
최신 원본 시각·변환 유효성 검사는 그대로 적용하며 `/tf_static`의 신뢰성·보존 설정은
바꾸지 않습니다. `false`로 되돌리면 설치된 Humble의 기본 동적 TF 구독을 사용합니다.
이 설정은 노드 생성 시 적용되므로 로봇 런타임을 다시 빌드하고 재시작해야 합니다.
상태의 `metrics.dynamic_tf_*` 진단값으로 수신 자체가 끊긴 경우와 수신 후 버퍼·변환이 지연된
경우를 구분합니다. 수신 카운터가 증가한다는 이유만으로 오래된 변환을 허용하지 않습니다.

PC의 Nav2도 동적 TF 구독에 BEST_EFFORT·깊이 20을 사용합니다. AMCL 내부의 별도
리스너와 Nav2의 다른 동적 TF 리스너에 적용되며 `/tf_static`은 변경하지 않습니다.
실차 실행 시 `./scripts/run_classroom_navigation_16.sh nav2_dynamic_tf_best_effort:=false`로
이 덮어쓰기를 생략할 수 있습니다. QoS 변경은 실행 중 파라미터 조절이 아니라 재시작이 필요합니다.
이 설정만으로 패킷 손실이나 시계 차이를 해결한 것으로 간주하지 않습니다.

### 시계 차이가 표시될 때

PC와 SSH 접속한 로봇에서 각각 확인합니다.

```bash
date -u
timedatectl status
```

자동 시간 동기화가 꺼져 있다면 **주행·브링업을 종료한 뒤**, 해당 장치에서 켭니다.

```bash
sudo timedatectl set-ntp true
timedatectl status
```

`System clock synchronized: yes`와 두 장치의 UTC를 확인한 뒤 브링업·주행·웹을
다시 실행하고 `check_classroom_deployment.sh --live`를 재검사합니다.
동기화 서버에 접근하지 못하는 환경은 `set-ntp true`만으로 해결되지 않습니다.
오래된 센서 시각을 현재 시각으로 바꾸는 방식으로 우회하지 않습니다.

## 제어 영수증과 멈춤 확인

`/smart_vision_amr/motion_status`의 `source_id`와 `control_seq`는 마지막으로 수락한
현재 제어 소스·순번입니다. `last_control`은 마지막 제어 패킷의 수락 여부·이유·출처이며,
`last_control_rejection`은 이후 정상 패킷이 와도 마지막 거부를 보존합니다.
`control_received/accepted/rejected`와 자율 명령의 `auto_*` 카운터로 유실·거부를 구분합니다.
`control_by_origin`과 `auto_by_origin`에는 출처별 수신·수락·거부 횟수도 있습니다.
`origin`은 외부 `topic`, 내부 적용 `service`, 직접 코어 호출 `core`, 기존 Twist 입력 `legacy`입니다.
JSON 파싱 실패와 비활성화된 topic reset도 영수증에 기록하며 서비스 DDS loopback은 중복 집계하지 않습니다.

`stop_acknowledged`는 독점 출력 권한이 있는 mux가 최근 실제로 **0 속도를 발행했다**는 확인입니다.
물리적 정지를 측정한 값은 아닙니다. 위치 보정은 별도로 신선한 odom의 정지를 확인해야 합니다.
`localization`은 선속도 0인 전용 회전 임대이며, 신선한 `allow_localization` 건강 상태와
회전 상한·dead-man을 적용합니다. 조이스틱 임대가 우선하고 비상정지는 모든 모드를 차단합니다.

## 섹터 표식과 벽 방향

현재 실차 강의실 설정은
`ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers_20261030.yaml`을
사용합니다. 좌표는 벽 표식 위치가 아닌 **로봇의 관측 목표**이며 `yaw`는 차체
전방 방향입니다.

| 위치 | x (m) | y (m) | 목표 yaw (rad) |
| --- | ---: | ---: | ---: |
| Home | -0.188431 | -0.220497 | -0.027514 |
| Sector 1 | 1.200000 | 0.575000 | 3.114078 |
| Sector 2 | 1.450000 | -2.650000 | 1.543282 |
| Sector 3 | -0.250000 | -1.980000 | -1.400000 |

Sector 1~3은 후방 카메라에서 해당 ID를 확인합니다. 현재 기본 `recognized` 모드는
표식이 보이면 영상 가로 중앙 ±15% 폭을 목표로 최대 3초·약 0.30rad 이내에서
저속 회전합니다. 새 영상 3개 이상과 약 0.6초의 안정성, 실제 정지를 확인해
도착을 승인합니다. 중앙 보정 한도 안에 들어오지 못하면 임무를 일시정지합니다.
표식 ID 하나만으로 도착하지 않으며, 이 모드는 **실제 벽과의 정확한 90° 정렬을
측정하거나 보증하지 않습니다.** 벽 법선 LiDAR 검사는 `--require-marker-wall-alignment`
선택 옵션으로만 켭니다. 전방 영상은 운용 보조용입니다.

Home 복귀는 등록 위치·방향과 실제 정지를 확인합니다. 초기 위치를 Home으로
지정할 때에는 로봇이 실제 그 자리에 있고 후방 카메라가 0번을 보는지 확인하며,
LiDAR·저장 지도 일치도도 대조합니다. 초기 위치 지정만으로 회전하거나 자동
출발하지 않습니다.

카메라 장착 좌표의 기본값은 차체 기준 전방 `(0, 0, 0)`, 후방 `(0, 0, π)`입니다.
이는 카메라 축을 차체 원점에 두는 **XY=0 좌표 관례**이며 실측 위치 보정값이 아닙니다.
알려진 전후방 장착 방향만 사용하고 영상 시야각·렌즈 보정값을 만들어 내지 않습니다.
선택형 벽 법선 검사는 평면에서 계산하지만, 벽 후보를 고르는 축의 원점은
실제 장착 위치의 영향을 받습니다.
장착 위치를 측정했다면 차체 기준 X는 전방, Y는 왼쪽의 미터값으로 지정합니다.
후방이 차체 뒤를 정면으로 보는 장착 방향은 yaw=π이며 다른 장착 각도도 실측값을 사용합니다.

실측값을 `REAR_CAMERA_X`, `REAR_CAMERA_Y`, `REAR_CAMERA_YAW` 환경변수에 지정한 뒤
PC의 웹 터미널에서 다음처럼 전달합니다. 값이 비어 있으면 시작하지 않습니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_classroom_web_16.sh --marker-rear-camera-pose \
  "${REAR_CAMERA_X:?실측 X(m)를 지정하세요}" \
  "${REAR_CAMERA_Y:?실측 Y(m)를 지정하세요}" \
  "${REAR_CAMERA_YAW:?실측 yaw(rad)를 지정하세요}"
```

전방 좌표를 측정해 지정할 때는 같은 형식의 `--marker-front-camera-pose X Y YAW`를 사용합니다.
클래스룸 래퍼의 추가 인자는 웹 서버에 그대로 전달됩니다. 일반 실행은 세 터미널 안내를 따릅니다.

## 검증 범위

소프트웨어 회귀 검사는 패킷 검증·소유권·정지·우선순위·임대 만료·웹 셸 진입을 검증합니다.
셸 진입 테스트는 가짜 ROS 실행 파일과 임시 작업공간을 사용하며 로봇에 접속하지 않습니다.
ROS 통신이 필요한 회귀는 localhost 전용의 격리된 테스트 도메인을 사용합니다.
빌드·오프라인 회귀 통과는 현재 로봇의 센서·모터·카메라 또는 실제 강의실 완주를 검증한 결과가 아닙니다.
실제 장비 검증은 운용자가 위 세 터미널 순서와 웹 준비 상태를 확인한 후 수행합니다.
