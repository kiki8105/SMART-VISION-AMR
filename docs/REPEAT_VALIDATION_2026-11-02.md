# 교실 임무 반복 검증 기록 (2026-11-02)

이 문서는 같은 지도와 주행 설정으로 **Sector 2 왕복 재출발**, 섹터 통합 임무,
지도에 없는 장애물 우회를 반복 시험한 결과를 기록한다. 실차 결과와 Gazebo 결과는
구분한다. 자동 시험은 PC의 독립 Gazebo master, `ROS_DOMAIN_ID=176`,
`ROS_LOCALHOST_ONLY=1`에서 실행하며 실차 도메인 16에 명령을 보내지 않는다.

## 실차에서 보고된 현상

팀원 보고에 따르면 10/30 지도 YAML을 적용한 뒤 웹 재고 요청으로
**Sector 2→Home 왕복을 1회 완료**했다. 영상은 팀원이 보유하며 이 저장소에는
그때의 ROS 원본 로그가 없다. 두 번째 출발은 비상정지, 위치·방향 불확실도,
초기 위치 검증 대기로 차단됐다. 하루 동안 10/27 지도 YAML과 10/30 좌표를
혼용했던 이력도 보고됐다. 실차 반복 주행이나 장애물 회피 성공으로 확대 해석하지 않는다.
현재 기본 지도는 `classroom_20261030_cleaned.yaml`의 SHA-256
`16a419cd5a9c499efb9e5af2f77b6b54c100d3df67c3ec6486b238ca968433b9`와
연결된 PGM의 SHA-256
`e4e25d36d84854eb327982984ba0ee2ede53ee3563b6b31177624a42035ad761`이다.

## 확인한 결함과 수정

- 웹은 AMCL `/set_initial_pose` 서비스로 초기 위치를 보냈지만, 위치 감시 노드는
  `/initialpose` 토픽만 초기 위치 변경 신호로 들었다. 새 AMCL 추정·새 TF·센서·정지를
  확인한 뒤 위치 감시 노드가 재획득을 승인하는 전용 응답을 추가했다. 승인은
  비상정지를 해제하지 않는다.
- 같은 요청의 서비스 응답이 유실돼도 중복 재초기화 없이 같은 승인을 다시 전달한다.
  센서나 TF가 실제로 오래되면 승인을 거부하고 출발도 막는다.
- PC 주행·웹 실행 스크립트가 10/30 지도 경로를 명시하고, 실수로 과거 지도 경로를
  명령행에서 덮어쓰는 것을 거부한다. 실제 `/map` 내용·단일 발행자도 출발 전 확인한다.
- 같은 세션의 첫 배송 뒤 등록 Home 좌표를 실제 정차 좌표로 간주해 재초기화하면
  라이다·지도 일치율이 48%까지 낮아져 안전하게 거부된 사례가 있었다. `HOME_REACHED`는
  등록 Home의 좌표·방향과 완전히 일치한다는 증거가 아니다. 실제 정차 위치를 확인해
  수동 초기화하고 비상정지는 별도로 해제해야 한다.
- Home으로 돌아온 후 Nav2 `SUCCEEDED`와 임무 완료 사이에 최종 방향 오차가
  남을 수 있어, 제한된 저속 Home 방향 보정과 실패 시 명시적 일시정지를 추가했다.
  이 보정 분기는 단위 시험을 통과했지만 전체 Gazebo 왕복에서 실제 작동한
  결과는 아직 없다.
- 독립 EKF Gazebo 시험에는 시각이 찍힌 12.0V 모의 배터리를 공급하고 로봇 감시·웹
  양쪽에 실차 시험 기준인 10.8V를 적용한다. 모의 전압은 OpenCR과 실물 배터리의
  부하·접촉 불량을 재현하지 않는다.
- 악조건 반복 시험에서 Sector 2의 후방 표식은 중앙에 보였지만 실제 차체 방향이
  엄격 기준에서 벗어났다. 현재 관측 목표 X를 1.450m에서 1.525m로 조정했다.
  변경 후 악조건 전체 왕복 3회가 엄격 판정을 통과했다. 세 번째 회차에는 긴
  재계획 대기가 있어 이를 응답성 해결 근거로 삼지 않는다. 순서 변경은 별도 2회, Home 이동 장애물은 독립 2회 엄격 통과했다.
  실차 반복 주행은 미검증이다.
- Home 부근 상자 관측이 시작 셀의 팽창 경계에 걸려 경로 생성이 거부된 사례가
  있었다. 연속 위치와 첫 이동 구간의 차체 여유를 확인하는 제한된 출발 복구를
  추가했다. 첫 합격 회차에서는 이 분기가 미발동했고, 두 번째 합격 회차에서는
  실제 4회 작동하며 매번 새 A* 경로를 만들었다. 실차 안전성은 별도 검증이다.

저장 벽보다 5–10cm 앞의 긴 물체는 벽 반사 오차와 구별되지 않아 A*의 관측
장애물 레이어에서 빠질 수 있다. Nav2에는 원본 `/scan`이 계속 전달되므로
주행은 멈출 수 있지만, 자동 우회가 막히면 실제 장애물을 확인하고 치운 뒤
경로 재생성을 확인해야 한다. 모든 비식별 장애물을 탐지한다는 뜻은 아니다.

## 출발 전 검사

웹 임무 검증기는 출발 전에 격리 도메인, 설정 지도와 실제 `/map` 일치·단일 발행,
최신 `map→base` TF, `/scan`·`/odom` 수신과 헤더 시각, AMCL 위치·방향 공분산과
라이다·지도 일치도, 카메라, 배터리 10.8V, A*·취소 서비스, 차량 정지 응답과
운용자의 명시적 비상정지 해제를 **개별 gate**로 기록한다. 하나라도 실패하면
목표를 보내지 않는다. 재출발 시험에서는 출발마다 **20/20 gate**를 기록했다.
Gazebo에서는 물리 OpenCR 직렬 통신·모터 배선·실물 배터리
상태를 검사할 수 없으므로 이는 실차 출발 전 점검이 별도로 필요하다.
또한 시뮬 웹 검증기는 전방·후방 카메라를 모두 gate로 확인하지만, 실차
`~/turtlebot16` 기본값은 전방 카메라를 끄고 모든 등록 표식은 후방을 사용한다.
실차 웹은 현재 경로의 표식에 필요한 카메라를 검사하므로 두 pre-arm의 카메라
조건은 완전히 같지 않다.

## 시험 기록

| 시험 | 결과 | 증거·범위 |
|---|---|---|
| 기본 교실 1→2→3→Home | **PASS 1회** | `/tmp/classroom-repeat-20261102-clean-1`; 배터리 gate 추가 전 결과 |
| 같은 웹·ROS 프로세스 Sector 2→Home 재출발 | **PASS 2회** | `/tmp/classroom-repeat-20261102-reentry-4`; 출발마다 pre-arm 20/20, 명시적 비상정지·실제 위치 기반 재초기화, 안전 위반 0 |
| 관리자 재고 API 요청→Sector 2→Home 재출발 | **PASS 2회** | `/tmp/classroom-repeat-20261102-inventory-2`; 두 요청 모두 배송 완료, 출발마다 pre-arm 20/20, 마커·5초 대기·Home, 안전 위반 0 |
| 악조건 교실 1→2→3→Home | **PASS 1회** | `/tmp/classroom-repeat-20261102-stress-clean-1`; 출발 20/20, 세 마커·대기·Home, 안전 위반 0. 당시 Sector 2 X=1.450m |
| 악조건 교실 1→2→3→Home 반복 | **엄격 판정 FAIL** | `/tmp/classroom-repeat-20261102-stress-clean-2`; 웹 임무는 완료됐지만 Gazebo 실제 방향 오차가 Sector 2 약 0.260rad, Home 약 0.358rad로 각각 0.25rad 기준 초과. 당시 Sector 2 X=1.450m |
| Sector 2 X=1.525m 변경 뒤 악조건 재시험 | **FAIL, Home 안전 정지** | `/tmp/classroom-repeat-20261102-stress-clean-sector2-1525-1`; 20/20 출발 gate와 세 섹터 방문 로그 후, Home 경로가 상자 관측 근처에서 생성되지 않아 제한 시간 내 완주하지 못함. 강제 진행 없음 |
| 출발 복구·Sector 2 X=1.525m 적용 뒤 악조건 1→2→3→Home | **엄격 PASS 2회** | `/tmp/classroom-repeat-20261102-stress-clean-sector2-1525-egress-1`, `/tmp/classroom-repeat-20261102-stress-clean-sector2-1525-egress-2`; 매 출발 20/20, 세 섹터 후방 ArUco·전방 영상·5초 대기·Home, 고정 상자 11/11·안전 위반 0. 1회차 Sector 2/Home 실제 방향 오차 0.090/0.079rad, 차체 최소 여유 0.0658m. 2회차 0.029/0.068rad, 최소 여유 0.0694m. 2회차에 연속 위치 출발 복구 4회 실제 작동·매번 새 A* 경로 생성, 후진 없음, 최장 `WAIT_REPLAN` 2.03초. 실행 중 소스 해시 불변 |
| 악조건 1→2→3→Home 세 번째 반복 | **엄격 PASS, 지연 미해결** | `/tmp/classroom-repeat-20261102-stress-clean-sector2-1525-egress-3`; 출발 20/20, 세 섹터 ArUco·5초 대기·Home, 고정 상자 11/11·안전 위반 0, 차체-상자 최소 여유 0.079m. Sector 3 진입 전 `controller_aborted` 뒤 약 120초 재계획 대기 후 재개했고, Home에서도 경로 소실·재생성이 여러 번 반복됨. 대기 길이는 실패 접근 기억의 만료 시간과 일치하지만 원인 확정에는 로그 재생이 필요함. 최종 Gazebo 실제 Sector 2/Home 방향 오차 0.208/0.099rad. 완주 판정과 빠른 복구 성능은 별개 |
| 악조건 1→3→2→Home 순서 변경 | **엄격 PASS 2회** | `/tmp/classroom-repeat-20261102-route-aware-permuted-1`, `-2`; 각 pre-arm 20/20, 1→3→2→Home 순서·후방 ArUco·5초 대기·Gazebo 실제 위치/방향, 고정 상자 라이다 관측→안전 경로 재계획→실제 우회, 안전 위반 0. 최소 차체 여유 0.0699/0.0873m. 이전에 고정 1→2→3 순서를 가정한 판정기를 경로 순서별로 수정한 뒤 새로 실행한 결과 |
| 악조건 Home 복귀 중 비식별 이동 장애물 | **엄격 PASS 2회** | `/tmp/classroom-repeat-20261102-cached-injector-moving-1`, `/tmp/classroom-repeat-20261102-cached-injector-moving-2`; 독립 두 회차 모두 pre-arm 20/20, 웹 1→2→3→Home, 실제 25cm 상자 생성·4단계 이동·라이다 관측·안전 경로 변경·실제 우회·제거, 관제 빨간 장애물 표시·소멸, 안전 위반 0. 이동 상자 최소 차체 여유 0.124/0.178m. 실차는 미검증 |
| 초기 실패 사례 | **FAIL, 수정 근거** | `reentry-2`: 실제 Home과 등록 Home 차이를 무시한 재초기화로 지도 일치율 48%; `inventory-1`: Home 방향 오차로 `RETURNING_HOME` 대기; 초반 관측기 발행자 발견 순서 결함은 재시험 전에 수정 |
| 실차 Sector 2 왕복 | **팀원 보고 1회 완료** | 영상은 팀원 보유, 원본 ROS 로그 없음. 장애물 없음. 두 번째 출발은 공분산·비상정지로 차단 |
| 실차 반복·장애물 | **미검증** | 현장 지도·전원·통신·위치 확인 후 별도 검증 필요 |
| 실차 설정값 시뮬 비교 1→2→3→Home | **엄격 PASS 1회** | `/tmp/classroom-repeat-20261102-final-hardware-value-parity-1`; pre-arm 20/20, 웹 1→2→3→Home·마커·5초 대기, 안전 위반 0, 체크인된 Nav2·교실 노드·EKF 파일의 생성 시뮬 설정값과 해시·시계 항목 제외 값 9/9 일치. Sector 3의 일시적 progress abort와 Home 재계획 대기 후 자동 복구. 고정 상자 최소 차체 여유 0.0511m. 실차 현재 파라미터·전원·통신/구동은 검증 범위 밖 |
| 선택 `laser_filters` A/B 악조건 1→2→3→Home | **판정 유보** | 필터 OFF/ON 각 1회 웹 임무·pre-arm 20/20·Home·안전 위반 0. 원본 OFF 단계는 당시 Gazebo 실측 연결 누락으로 FAIL, 저장 타임라인의 최신 보수 판정기로는 고정 상자 우회 양쪽 PASS. ON에서 단일 근거리 빔 210/210 제거, 도착 지연 p95 6.39ms; 성능 이점 미입증. [A/B 상세](LASER_FILTER_AB.md) |

앞선 `/tmp/classroom-repeat-20261102-reviewed-injector-moving-3`은 후보 탐색 지연 동안 로봇이 접근하고 이후 후보가 최신 관측 셀에 막혀 `spawned=false`로 끝난 시험 장치 FAIL이었다. 물체가 투입되지 않아 그 회차를 우회 성공 또는 주행 실패로 계산하지 않는다. 위 최신 회차의 성공이 이전 기록을 덮어쓰지 않는다.

위 `/tmp` 경로는 시험한 PC의 로컬 증거이며 Git clone에 포함되지 않는다.
`staged_report.json`과 단계별 `acceptance.json`, `safety_assessment.json`을 함께
확인한다. 웹 완료 표시만으로는 엄격한 실제 위치·방향·정지 합격을 판정하지 않는다.

## 반복 출발 시 초기 위치

`HOME_REACHED`는 Home 허용 범위 안에서 정지했다는 뜻이다. 첫 성공 주행 후
로봇이 실제 등록 Home 위치·방향과 조금 달랐는데 등록 좌표로 AMCL을 다시
초기화하자 LiDAR·지도 일치율이 48%로 내려가 재출발이 거부됐다. 위치 신뢰도가
정상이고 비상정지가 해제된 상태라면 매 왕복마다 AMCL을 다시 초기화할 필요가
없다. 경고가 뜨면 실제 정지를 확인하고, 등록 Home에 **실제로 놓였을 때만** 웹의
등록 Home 초기화 버튼을 사용한다. 그 외에는 지도에서 확인한 실제 X·Y·차체 전방
yaw를 입력한다. 새 AMCL·TF·센서·감시 재획득 완료 후 비상정지를 **별도로** 해제한다.
Gazebo에서 이 절차가 통과했어도 실차의 위치 표준편차 0.446m 상승이 자동으로
해결됐다는 뜻은 아니다.

## 재현과 실차 전환

Ubuntu 22.04·ROS Humble 설치 PC에서 다음을 **처음 한 번** 실행한다.
`setup_sim.sh`는 누락된 apt 패키지를 자동 설치하지 않으므로 먼저 설치한다.

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions gazebo \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup \
  ros-humble-xacro ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-compressed-image-transport ros-humble-teleop-twist-keyboard \
  python3-opencv python3-pyqt5 python3-numpy python3-yaml python3-pil python3-pytest
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
./scripts/setup_sim.sh
python3 scripts/run_classroom_staged_validation.py --list
```

각 시험은 **새 결과 폴더**에서 순차 실행한다. runner가 격리 도메인 176과 Gazebo
master를 설정·종료한다. 같은 PC에서 시험 둘을 동시에 실행하지 않는다. 아래는
재출발과 순서 변경 시험의 실행 예시다. 전체 결과가 `PASS`인지는 JSON으로 확인한다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
OUT="$HOME/classroom-repeat-$(date +%Y%m%d-%H%M%S)"
python3 scripts/run_classroom_staged_validation.py \
  --run 18_web_sector2_same_session_reentry,19_web_inventory_sector2_same_session_reentry,12_web_integrated_stress_clean,12_web_integrated_stress_permuted \
  --output "$OUT"
python3 -m json.tool "$OUT/staged_report.json" | less
```

한 단계가 실패하면 runner가 다음 단계를 시작하지 않는다. 원인을 확인·수정한 뒤
**새 출력 폴더**로 다시 실행한다. 이전 실패를 감추려고 안전 기준을 낮추지 않는다.
고정·이동 장애물과 실차 설정값 비교도 해당 단계에서 별도로 실행하고 결과를 기록한다.
실차에서는 [팀원용 세 터미널 매뉴얼](TEAM_QUICKSTART.md)의 10/30 지도 확인,
로봇/PC 시계·센서 수신, 실제 Home 초기 위치 지정, 위치 신뢰도 확인,
명시적 비상정지 해제를 거친 뒤에만 재고 요청을 출발시킨다.
