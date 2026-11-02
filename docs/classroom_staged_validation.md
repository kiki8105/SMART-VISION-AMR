# 교실 주행 단계별 검증

이 문서는 **시뮬레이션으로 확인할 수 있는 것**과 **실차에서만 확인할 수 있는 것**을 나눈다. 모든 자동 시험은 Gazebo 화면 없이 ROS 도메인 176, 로컬 DDS, 전용 Gazebo 마스터를 사용한다. 실차 도메인 16과 실차 웹 포트 8790은 사용하지 않는다. 다른 Gazebo 시험이 끝난 뒤 하나씩 실행한다. `web_` 단계는 임무 접수 웹→A*·추종기·제어 mux→ArUco 확인·5초 대기→Home 복귀까지 실제 웹이 소유한 임무를 검사한다. 그 밖의 ROS 목표 직접 발행 단계는 주행 계층의 원인 진단이며, 웹 배송 임무의 합격 증거로 쓰지 않는다.

## 새 Ubuntu PC: 클론·설치·빌드

Ubuntu 22.04와 ROS 2 Humble이 설치된 PC에서 아래 명령을 순서대로 실행한다. 저장소는 `~/Team2_workspace/src/SMART-VISION-AMR`에 둔다. `setup_sim.sh`는 공식 TurtleBot3 Gazebo 의존 소스를 고정 버전으로 가져와 빌드하며, 빠진 apt 패키지가 있으면 설치 명령을 출력하고 종료한다. 특히 단계별 하네스의 시작 전 회귀 검사에는 `python3-pytest`가 필요하다.

```bash
mkdir -p ~/Team2_workspace/src
cd ~/Team2_workspace/src
git clone https://github.com/kiki8105/SMART-VISION-AMR.git
cd SMART-VISION-AMR
./scripts/setup_sim.sh
```

기존 클론을 업데이트할 때에는 임의로 로컬 변경을 버리지 말고 `git status`를 확인한 다음 최신 커밋을 가져온다. `setup_sim.sh`의 빌드가 통과하면 아래 두 터미널을 연다. **실차용 `run_classroom_navigation_16.sh`나 로봇 SSH 브링업은 이 시뮬레이션에 필요하지 않다.**

## 터미널 1: 읽기 전용 웹 관제

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
/usr/bin/python3 scripts/run_classroom_readonly_monitor.py
```

PC/휴대폰에서 `http://<시험 PC의 hostname -I 주소>:8791/`을 연다. 같은 PC라면 `http://127.0.0.1:8791/`도 된다. 이 프록시는 ROS 노드나 명령 발행자가 없고 임무·조종 요청을 차단한다. 전체 시스템 `web_` 단계가 진행될 때만 해당 단계의 단일 임무 웹(로컬 8792)을 읽기 전용으로 보여주며, 구간 사이에는 대기 화면을 표시한다. **8792는 단계마다 자동으로 열리고 종료되므로 상시 접속 주소가 아니다.** 도메인 입력 창이 나오면 **176**을 넣는다. 별도 `run_classroom_replay_web.sh`를 8791에 실행하면 두 번째 임무 소유자가 생겨 `web_` 시험이 시작을 거부한다.

주행 계층 진단 단계의 자동 검증기는 ROS 목표를 직접 발행하므로 이 프록시에 웹 임무 상태가 나타나지 않을 수 있다. 관제 화면에서 지도·경로·카메라가 보이는 것과 **웹 배송 기능 합격**은 구별한다. 시험이 끝나면 Gazebo 토픽이 끊기므로 경로·영상이 사라지는 것은 정상이다.

## 터미널 2: 순서대로 시험

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
/usr/bin/python3 scripts/run_classroom_staged_validation.py --list

# 각 단계의 결과는 새 디렉터리에 저장한다.
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 01_sector_1 --output /tmp/classroom-stage-01-a

/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 02_sector_2 --output /tmp/classroom-stage-02-a
```

`--run all`은 과거 원인 진단 단계까지 포함하고 첫 불합격에서 멈춘다. 새 PC의 전체 시스템 수용 시험 명령으로 사용하지 않는다. 각 구간 제한 시간 기본값은 180초이며, `--continue-on-fail`은 후속 진단용일 뿐 앞 단계 불합격을 합격으로 바꾸지 않는다.

먼저 짧은 핵심 순서로 확인할 수 있다. 각 단계는 새 Gazebo를 띄웠다가 완전히 종료한 뒤 다음 단계로 간다. 이 결과만으로 나머지 단계를 합격으로 간주하지 않는다.

```bash
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 01_sector_1,03_sector_3,05_sectors_1_2_3,08_browser_client_disconnect,12_integrated_clean,12_integrated_home_obstacle \
  --output /tmp/classroom-smoke-$(date +%Y%m%d-%H%M%S)
```

위 핵심 순서는 주로 주행 계층 진단이다. 실차와 같은 임무 제어 흐름의 합부를 보려면 웹 소유 단계도 별도로 실행한다. 이 단계들은 독립 EKF와 실차형 센서 감시·시작 비상정지, 사고 기록기를 켜고, 웹에서 안전 정지를 명시적으로 해제한 뒤 임무를 시작한다. 자동 해제나 안전 조건 우회는 허용하지 않는다. `--run all`은 진단 단계까지 포함해 첫 실패에서 중단되므로, 필요한 웹 단계는 개별 선택할 수 있다.

```bash
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 03_web_sector3_clean,06_web_sector3_moving_obstacle,12_web_integrated_clean,12_web_integrated_home_obstacle \
  --output /tmp/classroom-web-full-$(date +%Y%m%d-%H%M%S)
```

팀원이 **현재 전체 시스템 시험을 재현**하려면 아래 명령을 터미널 2에서 **하나씩** 실행한다. 첫 번째는 지도에 없는 고정 20cm 상자를 감지해 Sector 1→2 경로를 새로 만들고 1→2→3→Home을 완주하는지 검사한다. 이 단계는 2026-11-02 v6에서 한 차례 PASS했다. 두 번째는 Home 복귀 중 별도 이동 장애물을 넣고 제거하는 시험이며, v1은 엄격한 경로·제동 게이트 FAIL, v2는 사용자 요청으로 중도 정지했으므로 **아직 PASS 기록이 없다.** 세 번째는 실차 Nav2·A*·EKF 원본 YAML 값 비교 및 같은 전체 경로의 추가 시험으로, **아직 실행하지 않았다.** 모두 `ekf_independent`·센서 guard·사고 기록기를 켜고 웹 비상정지를 명시적으로 해제한다. **동시에 실행하면 안 된다.** 각 출력 루트의 `staged_report.json`에서 결과를 확인하고, `acceptance.json`, `safety_assessment.json`, `stress_crate_assessment.json`을 대조한다. `passed: true`가 없으면 성공으로 해석하지 않는다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 12_web_integrated_stress_clean --leg-timeout 600 --obstacle-hold-timeout 300 \
  --output /tmp/classroom-stress-clean-$(date +%Y%m%d-%H%M%S)

/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 12_web_integrated_stress_home_obstacle --leg-timeout 600 --obstacle-hold-timeout 300 \
  --output /tmp/classroom-stress-obstacle-$(date +%Y%m%d-%H%M%S)

# 아래 Stage 17은 후속 검증용이며 현재 결과는 NOT RUN이다.
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 17_web_hardware_value_parity --leg-timeout 600 --obstacle-hold-timeout 300 \
  --output /tmp/classroom-physical-values-$(date +%Y%m%d-%H%M%S)
```

한 번의 Gazebo 합격은 재현 환경의 소프트웨어 동작 증거이며, 다른 PC에서 같은 결과가 자동 보장되거나 실차의 OpenCR·배터리·무선망·카메라 처리량까지 동일하다는 뜻은 아니다. 마지막 단계의 값 비교는 저장소 원본 YAML 및 실행 manifest SHA에 한정된다. Gazebo 카메라는 실차 카메라보다 해상도·fps가 높고, 실차의 robot-local 노드 배치와 Nav2 velocity smoother는 시뮬레이션과 다르다.

개별 단계를 실행할 때 `--run 03_sector_3`처럼 `--list`의 ID를 사용한다. 쉼표로 묶으면 지정한 순서대로 순차 실행한다. 예: `--run 04_sectors_1_2,05_sectors_1_2_3`. 사용 중인 Gazebo 마스터가 있으면 시작을 거부한다.

섹터 단독 시험을 마친 뒤 장애물 없는 통합 경로를 먼저 검사하고, 새 결과 디렉터리에서 Home 복귀 중 장애물 시험을 이어간다. 구간별 600초는 무한 대기가 아닌 시험 종료·증거 보존을 위한 상한이다. `--obstacle-hold-timeout 300`은 정지 상태의 재계획을 기본 15초보다 길게 관찰하려는 시험 설정이다. 안전 차단은 그대로 적용되며, 복구·도착 여부는 별도 증거로 판정한다.

```bash
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 12_integrated_clean --leg-timeout 600 --obstacle-hold-timeout 300 \
  --output /tmp/classroom-integrated-clean-$(date +%Y%m%d-%H%M%S)

/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 12_integrated_home_obstacle --leg-timeout 600 --obstacle-hold-timeout 300 \
  --output /tmp/classroom-integrated-obstacle-$(date +%Y%m%d-%H%M%S)
```

| 순서 | ID | 시험 목적 | 요구되는 결과 |
| --- | --- | --- | --- |
| 1–3 | `01_sector_1`, `02_sector_2`, `03_sector_3` | Home에서 섹터별 단독 목적지 | 실측 도착·정렬, 해당 후방 마커 인식, 실제 경로·속도 명령 |
| 3w | `03_web_sector3_clean` | 전체 시스템에서 웹이 소유한 Sector 3→Home 단독 임무 | 실제 후방 마커 중앙 확인·5초 정지·Home 실측 도착과 안전 해제 기록 |
| 4–5 | `04_sectors_1_2`, `05_sectors_1_2_3` | 목적지를 1→2, 1→2→3으로 누적 | 순서대로 각 구간 도착, 중간 목표가 다음 경로를 막지 않음 |
| 6 | `06_single_unmapped_obstacle` | Sector 3 단독 주행 중 이동 장애물 생성·이동·제거 | 새 장애물 관측, 경로 반응, 차체 간격, 제거 뒤 관측 해제, 도착 |
| 6w | `06_web_sector3_moving_obstacle` | 전체 시스템 Sector 3→Home 중 이동 장애물 | 웹 임무 유지·실측 우회·장애물 셀 표시/소거와 차체 여유 |
| 7 | `07_cumulative_unmapped_obstacle` | 누적 주행 중 지도에 없는 장애물 | 장애물 존재 중 실제 우회, 경로 반응, 제거 뒤 주행 완료 |
| 8 | `08_browser_client_disconnect` | 시험 전용 웹 8792에서 Sector 3 임무를 접수한 뒤 접수 클라이언트를 끊음 | 25초 동안 HTTP 접속 없이 경로·실측 이동이 유지되고, 별도 관측 클라이언트가 진행 상태 확인 |
| 9 | `09_wan_absent` | **외부 인터넷/WAN 없음** | 새 user+network namespace의 루프백만으로 주행 완료. 호스트 Wi-Fi는 변경하지 않음 |
| 10 | `10_sensor_dds_outage` | scan·odom·TF 각각 약 3초 단절 | 모든 단절 주입 확인, 비상정지 latch·0속도·실측 정지·목표 유지. **자동 재출발 불합격**, 운용자 재설정 대기 |
| 11 | `11_map_world_mismatch` | 저장 지도보다 더 좁은 통로와 새 상자 | 안전 도착 또는 물리적 정지. `SAFE_HOLD`는 배송 성공과 구분 |
| 12a | `12_integrated_clean` | 새 장애물 주입 없이 1→2→3→Home | 네 구간 모두 실제 경로·속도·Gazebo 실측 도착·후방 마커 확인; 장애물 주입 플래그가 없어야 함 |
| 12b | `12_integrated_home_obstacle` | 1→2→3→Home 중 Home 경로에 새 장애물 | 전 구간 마커·도착, 주입 중 물리적 우회·간격, 제거 후 빨간 관측 셀 해제 |
| 12w | `12_web_integrated_clean`, `12_web_integrated_home_obstacle` | 전체 시스템의 웹 소유 1→2→3→Home 및 Home 중 이동 장애물 | 섹터별 마커 중앙 확인·5초 정지·다음 경로·Home 도착, 장애물 주입 시 우회·표시/소거 |
| 12s | `12_web_integrated_stress_clean`, `12_web_integrated_stress_home_obstacle` | 악조건 월드의 고정 20cm 비식별 상자, 그리고 Home 중 별도 이동 장애물 | 라이다 관측→상자와 차체 여유를 둔 새 경로→Gazebo 실측 통과→전 섹터·Home, 이동 장애물의 표시·우회·제거 |
| 13 | `13_web_sector_dwell_1_2` | 웹에서 1→2 임무 접수 | 각 섹터의 카메라 확인→정지 5초→다음 목표, 2번 뒤 5초→Home. 각 도착 시 Gazebo 실측과 대조 |
| 14 | `14_web_sector_dwell_1_2_3` | 13단계 합격 후 웹에서 1→2→3 임무 접수 | 세 섹터마다 5초 정지, 마지막 3번 뒤 Home. 목표별 경로와 Gazebo 실측 대조 |
| 15 | `15_integrated_with_recorder` | ROS 직접 목표 주행에 사고 기록기를 켜고 12단계 반복 | 기록기 부하에서 주행 계층을 재검사하고 CPU·TF 지연 비교; 웹 임무·독립 EKF 합격과 구별 |
| 16 | `16_web_transient_resume` | **웹이 소유한** 1→2→Home 배송 임무 중 수백 ms 전달 지연과 짧은 입력 단절을 주입 | 원래 웹 임무·목표·배송 순서 유지, 입력 무효 동안 0속도와 제한된 실측 이동, 내부 FollowPath 취소·재발행 뒤 자동 재개, 각 5초 대기·Home 도착 |
| 17 | `17_web_hardware_value_parity` | 실차 원본 YAML 값 사용 여부를 비교하며 악조건 월드의 전체 웹 임무 재실행 | Nav2 원본 값과 생성본이 `use_sim_time` 외 완전히 같고 nodes/EKF 원본 SHA가 실행 manifest와 일치하며, 1→2→3→Home·고정 상자 우회·안전 게이트 모두 합격 |

`08_browser_client_disconnect`는 주행 스택과 웹 서버를 계속 켜둔 채, 웹에서 임무를 시작한 원래 HTTP 클라이언트만 버린다. 시험 웹은 `127.0.0.1:8792`, 독립 설정 파일을 사용한다. 25초 뒤 **다른 읽기 전용 클라이언트**가 임무 상태를 확인한다. 브라우저 접속이 실제로 끊어진 상태의 ROS 실측 이동을 요구하며, 이는 서버 프로세스나 로봇↔PC DDS 연결 단절 시험은 아니다.

`12_integrated_clean`을 먼저 실행해 네 구간 통합 주행 자체를 확인하고, 그다음 `12_integrated_home_obstacle`에서 Home 복귀 중 새 장애물의 영향을 분리해 검사한다. 두 단계는 각각 새 Gazebo 월드와 별도 결과 디렉터리를 사용한다. 전체 경로 검증에는 45초 진단 제한을 쓰지 않으며, 구간별 한도는 `--leg-timeout`으로 지정한다. 제한에 걸린 주행은 합격이 아닌 미완료로 기록한다.

`13_web_sector_dwell_1_2`와 `14_web_sector_dwell_1_2_3`도 별도 시험 웹 8792에서 실행한다. 먼저 두 섹터에서 5초 대기를 확인한 뒤 세 섹터로 늘린다. 웹 서버의 설정·카메라 확인 이벤트뿐 아니라, **대기 시작부터 끝까지** 독립 Gazebo 실측과 `/cmd_vel`의 연속 표본으로 정지를 확인한다. 실측 이동 2.5cm·방향 변화 0.06rad 이하, 0이 아닌 명령 없음, 긴 표본 공백 없음이 요구된다. 다음 `/smart_vision_amr/goal` 발행이 대기 종료 이벤트 **뒤**이고 다음 섹터 또는 Home 좌표와 일치해야 한다. 마지막 Home도 Gazebo 실측과 대조한다. `15_integrated_with_recorder`는 ROS 직접 목표 주행에 기록기를 더한 **주행 계층 진단**이다. Gazebo와 기록기 부하가 실차와 같지는 않으므로 합격 기록도 분리한다.

`06_web_sector3_moving_obstacle`와 `12_web_integrated_home_obstacle`는 시험 소유 장애물이 살아 있는 동안과 제거된 뒤에 관측된 지도 셀을 대조한다. 시험은 웹 HTTP 상태의 표시 가능 셀과 각 지도 모드의 SVG 작성 규칙을 검사한다. **실제 브라우저 DOM에서 정리된 지도와 원본 지도에 빨간 픽셀이 보이는지까지는 브라우저 접근 정책 때문에 확인하지 못했다.** 두 화면의 시각적 합격은 사용자가 8791 읽기 전용 관제에서 직접 확인해야 한다. HTTP·JS 검사는 그 대체 증거가 아니며 실차 UI 합격으로 확대하지 않는다.

`09_wan_absent`는 Gazebo/ROS/검증기를 **같은** `unshare -Urn` 네트워크 공간 안에 띄운다. `network.txt`에 기본 라우트가 없고 루프백이 활성화된 증거를 남긴다. 이는 인터넷 없이 로컬 ROS가 가능한지를 시험할 뿐, 서로 다른 두 컴퓨터 사이 Wi-Fi가 끊긴 상황은 재현하지 않는다. `10_sensor_dds_outage`는 로봇·PC 사이 센서 전달 단절의 소프트웨어 대용 시험이며 실제 Wi-Fi 패킷/제어 링크 장애를 완전히 재현하지 않는다.

`16_web_transient_resume`는 외부 ROS 목적지를 직접 보내는 시험과 다르다. 웹에서 1→2 배송 임무를 접수해 첫 섹터로 실제 이동한 뒤, 센서 원본 시각을 고치지 않는 fault relay에 **0.35초 전달 지연**과 이어지는 **0.8초 입력 공백**을 요청한다. 타임라인은 웹 발행 목표가 지연 전에 **한 번** 나왔는지, scan·odom·TF 각각 실제 지연/손실이 있었는지, 입력 무효 시 0.25초 안에 0속도 명령을 내고 실측 이동 5cm·회전 0.12rad 이내로 멈추는지 확인한다. 내부 Nav2 FollowPath가 취소·실패 후 재발행되는 것은 허용하되, **웹 임무 취소·비상 latch·중복 배송은 불합격**이다. 같은 목표로 자동 재개해 Sector 1·2의 각 5초 대기와 Home 도착까지 확인해야 합격이다. 실제로 입력 무효·내부 취소/재발행이 관측되지 않으면 이 시나리오를 검증했다고 보지 않는다. `10_sensor_dds_outage`의 장시간 단절은 복구 자동 재출발이 아니라 명시적 재설정 대기를 요구한다.

### 재계획 중 움직임의 안전 게이트

각 단계는 별도 관측 노드가 `/smart_vision_amr/planner_status`, `/smart_vision_amr/navigation_status`, `/smart_vision_amr/path`, `/cmd_vel`, `/scan`, `/gazebo/model_states`를 수신 시각으로 기록한다. `safety_timeline.jsonl`과 `safety_assessment.json`을 확인한다.

- `PLANNING` 동안 직전 경로가 살아 있고 정면 라이다가 신선하고 여유 있으면 움직임을 즉시 오류로 보지 않는다. 다만 정면 여유만으로 전체 경로 안전을 증명하지는 못한다.
- `WAIT_REPLAN`은 경로가 무효화된 상태다. **0.25초 이내 0속도 명령**, 실제 차체 **5cm 이내 정지거리**, **0.12rad 이내 추가 회전**을 기본 게이트로 확인한다. 반응 초반의 순간 속도나 관성을 자동 불합격으로 처리하지 않는다. Nav2의 별도 `RECOVERING` 후진은 기록해 별도로 판단한다.
- 정면 ±12° 라이다 최단거리와 각도, SDF상의 라이다→차체 앞끝 **0.132m**, 실제 추가 전진거리를 이용해 추정 차체 여유 **2cm 이상**인지 본다. ±30°의 최단 빔을 정면 차체 간격으로 단정하지 않는다. 측정 범위 밖 차체 측면·높이·동적 장애물은 이 게이트만으로 보장할 수 없다.
- 실제 경로 토픽, Gazebo 실측 위치, 라이다, 속도 중 하나라도 누락되면 `PASS`를 내지 않는다.

정적 Gazebo 월드임이 확인된 단계에서는 별도의 형상 증거도 만든다. 월드 충돌 박스와 Gazebo 실측 차체 궤적을 대조하며, Waffle Pi의 모든 수평 충돌 형상을 감싸는 반경 **0.24m**를 쓴다. 전 구간에서 충돌 박스까지 **5cm 이상** 남아야 한다. 단일 스캔의 12~18cm 근접 값이 앞뒤 각 0.26초 이내 스캔에서는 35cm 이상이고, 해당 빔이 정적 월드에서는 50cm 이상 떨어진 곳을 가리키며, 보수적인 차체 충돌 여유도 충분할 때에만 **정적 시뮬레이션 센서 이상치**로 분류한다. 연속 근접 값, 관측 누락, 형상상 가까운 벽은 계속 불합격이다. 동적 장애물이 주입되는 6·7·12b·15단계와 실차에는 이 예외를 적용하지 않는다. 큰 추가 회전도 해당 정적 시험에서는 실제 회전 궤적 전체의 충돌 여유가 입증될 때만 형상 근거와 함께 판정한다. 형상 게이트 합격은 실차 안전 인증이 아니다.

2026-11-02 Sector 3 단독 초회 시험은 목적지·마커는 통과했으나 안전 게이트에서 보류됐다. 세 차례의 단발 근접 빔은 **0.131m@+2°, 0.120m@+11°, 0.124m@+9°**였고, 같은 각도의 정적 월드 예상 거리는 각각 약 **1.04m, 1.39m, 1.41m**였다. 앞뒤 스캔은 0.88~1.31m로 복귀했다. 보수적 원형 차체의 실제 월드 충돌 여유 최솟값은 세 구간에서 **0.151m, 0.136m, 0.090m**였다. 초회 결과를 안전 합격으로 소급하지 않는다.

새 형상 검증으로 다시 실행한 `03_sector_3` 기록(`/tmp/svam_staged_20261102_stage03_recheck/03_sector_3/`)은 **당시 주행 기능 불합격**이었다. 안전 타임라인에서는 위반이 없었으나, t≈12/14/16초에 0.12~0.14m 단발 빔이 반복돼 발행된 경로가 세 차례 바로 무효화됐다. 이후 follower가 `execution_oscillation`으로 실패 접근을 기억했고, A*가 대체 경로를 찾지 못했다. 종료 시 목표까지 Gazebo 실측 약 **1.00m**가 남았다. 센서 전달은 clean 프로필에서 scan 258/258, odom·TF 각 1513/1513이었고 안전 차단은 없었다. 아래 재시험은 단독 구간의 후속 합격이며, 누적·장애물·통합 주행의 합격을 대신하지 않는다.

이후 단발 근접 빔을 다음 스캔에서 다시 확인하는 guard를 적용한 **새 프로세스**로 Stage 3을 120초 한도로 재시험했다. `/tmp/svam_staged_20261102_stage03_baseline_extended/03_sector_3/`는 약 82.18초에 **PASS**했으며, Gazebo 실측 목표 위치 오차 4.98cm·헤딩 오차 0.179rad, 후방 3번 마커 인식, 별도 안전 타임라인 합격을 기록했다. 처음 FollowPath 입력 결손 후 약 2.3초 만에 같은 목표로 다시 계획했고, 후반 국소 우회 뒤 원경로에 재합류했다. 앞선 동일 목표 실패가 있었으므로 이것은 **한 번의 성공 증거**이며 반복 성공률이나 실차 통과를 뜻하지 않는다. 40~45초짜리 진단 시험은 목표 도착 전 종료되므로 합격률에 합산하지 않는다.

원시 TF가 한 번 약 2.77초 정체한 동안 `/odom`·`/scan`은 계속 도착하고, 지연된 TF가 버스트로 들어와 위치 안전 latch가 걸린 기록도 있다(`/tmp/svam_staged_20261102_stage03_tf_stamp/`). TF QoS 비교 A/B는 다른 전체 pytest/build가 A 시뮬과 겹쳐 **부하 통제가 안 됐다**. 현재 비교 파일(`/tmp/svam_staged_20261102_tf_qos_comparison.json`)은 참고 계측이며 QoS 변경의 성능 판정으로 쓰지 않는다. 관제 웹은 한 번의 Gazebo 시계 역행 뒤 지도·TF·센서를 다시 구독하도록 수정했다. 동일 웹 PID의 두 번째 시뮬 epoch에서 지도와 차량 위치가 약 3초, 새 목표와 경로가 약 5초 안에 복원되는 것을 확인했다. 이 관제 확인은 주행 완료 판정과 별개다.

2026-11-02 선행 시험에서는 `WAIT_REPLAN` 직후 **7/54/65ms**까지 명령이 남고 약 **105ms**에 0이 됐으며, Gazebo 실측은 약 **0.33초·3.05cm** 뒤 멈췄다. 당시 ±30° 최소 거리는 **0.134m**였지만 빔 각도가 없어 차체 여유가 안전했다고 확정할 수 없다. 이 수치는 새 안전 게이트의 사전 관측 자료이며 실차 안전 합격 기록이 아니다.

각 단계의 `acceptance.json`은 원래 주행 검증 결과이고, `staged_report.json`은 순서·웹/통신 범위·추가 안전 게이트까지 확인한 결과다. `runner_exit_code=0`만으로 합격시키지 않는다. 기존 결과 폴더를 다시 평가할 때는 다음 명령을 쓴다. 타임라인을 수집하지 않은 과거 시험은 안전 증거 누락으로 불합격이다.

```bash
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --assess-existing all --output /tmp/기존-단계별-결과-루트
```

### 통신 단절은 세 가지로 나누어 판정

1. **브라우저/휴대폰 접속만 끊김:** 웹에서 임무가 접수된 뒤 브라우저를 닫아도 ROS 노드와 웹 서버는 계속 살아 있어야 한다. `08_browser_client_disconnect`는 다른 포트의 시험 웹에서 임무를 직접 접수한 뒤 25초 동안 원래 클라이언트를 끊고 Gazebo 실측 이동과 임무 상태를 확인한다. ROS 직접 goal 시험을 이 항목의 증거로 쓰지 않는다.
2. **외부 인터넷만 끊김:** `09_wan_absent`가 유효하다. 전부 한 PC의 루프백에서 실행된 결과이므로 분산 실차 무선망 검증으로 확대 해석하지 않는다.
3. **로봇↔PC Wi-Fi/DDS 링크 끊김:** 원격 경로/제어 데이터가 끊긴 채 무작정 주행을 계속하면 안 된다. 실차에서는 로봇 쪽 제어 노드가 입력 지연을 감지해 정지해야 한다. `10_sensor_dds_outage`는 센서 경로의 모의 단절까지만 확인한다. 실제 링크 시험은 현장에서 사용자가 주변을 통제하고 저속으로 별도 실시한다.

### 실차 연결성: 사용자가 직접 하는 최종 단계

가제보 전 단계가 합격해도 배터리 전압, OpenCR/Dynamixel, USB 카메라, Wi-Fi, 실측 마커 위치는 가제보가 보장하지 못한다. 실차로 넘어갈 때에는 [팀 실행 안내](TEAM_QUICKSTART.md)의 터미널별 빌드·실행 순서를 따른다. 실차 실행은 이 자동 하네스에 포함되지 않는다. 사용자가 로봇 주변을 확인하고 직접 명령을 입력한다.

1. 로봇과 PC에서 같은 Git 커밋을 체크아웃·빌드하고, 로봇에서는 `./scripts/check_classroom_deployment.sh --robot`, PC에서는 `./scripts/check_classroom_deployment.sh --pc --live`로 필수 패키지·센서·시계를 확인한다.
2. 로봇 브링업/카메라, PC 주행 런치, PC 웹을 **각각 한 번만** 실행한다. `/map`·`/odom`·`/cmd_vel`의 발행자가 각각 하나인지 확인한다. Home에서 실제 자세 초기화와 후방 0번 마커를 확인한다.
3. 짧고 열린 Sector 1만 먼저 주행한다. 계획 경로 표시, 실제 이동, 후방 마커 인식, 정지 후 표식 확인을 기록한다. 이어 Sector 2, Sector 3, Home 순서로 늘린다.
4. 지도에 없는 장애물은 사람이 즉시 정지시킬 수 있는 조건에서만 추가한다. 라이다 관측→지도 빨간 셀→국소 우회/전체 재계획 또는 안전 대기→제거 뒤 셀 소거를 확인한다.
5. 실차에서 브라우저만 닫는 시험(A)과 외부 인터넷 단절(B)은 로봇 제어 Wi-Fi 링크를 유지한 채 한다. 로봇↔PC 링크 단절(C)은 별도 저속 안전 시험이며 정지 확인이 합격 조건이다.

실차 시험 결과는 Gazebo 결과와 별도 기록한다. 특히 지연·위치 신뢰도·재계획 시간·최소 거리·마커 검출률을 같은 양식으로 비교하고, 실차에서 실패하면 시뮬레이션 합격을 실차 합격으로 바꾸지 않는다.
