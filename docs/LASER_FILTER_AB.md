# 라이다 단일 반사 필터 A/B 시험 (선택 기능)

ROS 2 Humble `laser_filters`의 약한 Speckle 필터를 **`change_memory` 입력에만** 적용합니다. 기본값은 `false`입니다. AMCL, Nav2 costmap과 로봇 `motion_mux`의 근거리 안전정지는 원본 `/scan`을 계속 사용합니다. `change_memory`가 내보내는 A* 관측 장애물과 웹 빨간 장애물 표시에는 필터가 영향을 줄 수 있습니다. 한 빔으로만 보이는 실제 작은 물체도 제거될 수 있으므로 좁은 장애물·이동 물체 시험 없이 실차 기본값으로 채택하지 않습니다.

## 현재 A/B 결과 (2026-11-02)

격리 도메인 176, 악조건 교실 월드, 동일 지도·코드·Gazebo seed 4242와 웹 1→2→3→Home 임무에서 필터 토글만 바꾼 **각 1회** 결과입니다. 고정 20cm 상자는 저장 지도에 없으며, 두 회차 모두 pre-arm 20/20, 세 섹터 ArUco·5초 대기, Home, 안전 위반 0, 라이다 관측 뒤 A* 재계획 우회를 확인했습니다.

| 항목 | 필터 OFF (기본값) | 필터 ON |
|---|---:|---:|
| 웹 임무 시간 | 319.67초 | 335.01초 |
| 고정 상자 차체 최소 여유 (최신 보수 판정기) | 0.0688m | 0.0551m |
| 원본 `staged_report.json` | FAIL: 당시 엄격 판정기의 Gazebo 실측 연결 누락 | PASS |
| 저장 타임라인의 최신 보수 판정기 오프라인 재평가 | 고정 상자 우회 PASS | 고정 상자 우회 PASS |
| 원본/필터 스캔 동일 시각 짝 | 해당 없음 (필터 비실행) | 1545/1545 |
| 근거리 고립 단일 빔 제거 | 해당 없음 | 210/210 |
| 필터 영상 추가 도착 지연 p95 | 해당 없음 | 6.39ms |

필터 OFF 회차의 **원본 전체 단계 결과는 FAIL로 보존**합니다. 당시 Gazebo 실제 위치 샘플의 일시적 빈 구간(최대 0.548초)을 엄격 판정기가 증거 누락으로 처리했습니다. 저장 타임라인을 다시 검사한 최신 보수 판정기는 상자 위치 샘플 2개만 양옆의 최신 동일 위치 기록으로 보간했고, 로봇 이동 구간은 시간 간격과 속도 상한을 포함해 보수적으로 검사한 뒤 고정 상자 우회를 PASS로 판정했습니다. 이 오프라인 재판정은 원본 결과를 덮어쓰지 않으며, 갱신된 판정기로 새 A/B를 반복해야 전체 단계 재현성이 확정됩니다.

필터 ON에서 단일 빔 210개를 제거했지만 **주행 시간이나 충돌 여유 개선은 입증되지 않았습니다.** 오히려 이 한 쌍에서는 시간이 15.34초 길고 최소 여유가 작았습니다. 이는 실행 간 ROS·Gazebo 스케줄 차이도 포함하므로 필터가 원인이라고 단정할 수 없습니다. 6.39ms는 두 토픽의 관측자 도착 차이이며 순수 필터 계산 시간이 아닙니다. 이동 장애물 우회는 필터 기본 OFF로 별도 Gazebo 독립 회차에서 2회 엄격 통과했습니다. 이동 장애물에서 필터 ON/OFF를 직접 비교한 시험은 아직 없습니다.

## 팀원 PC에서 재현

필터를 켠 시험 PC에만 선택 패키지를 설치합니다. 기본 실차 실행에는 필요하지 않습니다.

```bash
sudo apt update
sudo apt install ros-humble-laser-filters
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/setup_sim.sh
STAMP=$(date +%Y%m%d-%H%M%S)
RAW="$HOME/laser-ab-raw-$STAMP"
FILTERED="$HOME/laser-ab-filtered-$STAMP"
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 12_web_integrated_stress_clean --measure-scan-filter --output "$RAW"
/usr/bin/python3 scripts/run_classroom_staged_validation.py \
  --run 12_web_integrated_stress_clean --measure-scan-filter \
  --laser-filter-for-change-memory --output "$FILTERED"
```

**두 시험은 순차 실행**하고 원본 코드·지도·설정은 중간에 수정하지 않습니다. Runner는 기존 출력 폴더를 덮어쓰지 않습니다. 각 폴더의 `staged_report.json`, 단계별 `acceptance.json`, `safety_assessment.json`, `stress_crate_assessment.json`, `runtime_manifest.json`, `scan_filter_pair.json`을 대조합니다. 원본 단계의 PASS뿐 아니라 pre-arm, 마커·대기·Home, 고정 상자 라이다 관측→안전 경로→실제 통과, 차체 여유, 소스 해시 불변을 확인합니다. 필터 OFF의 `scan_filter_pair.json`은 원본 스캔 수만 유효하며 `filtered_frames=0`, `same_stamp_paired=0`, 도착 지연 `null`이 정상입니다. 필터 ON일 때만 짝·제거 빔·도착 지연을 해석합니다.

시험이 **실행 중일 때** 도메인 176의 별도 터미널에서 연결을 확인할 수 있습니다. 필터 ON일 때 `/change_memory scan_topic`은 `/scan_filtered_for_change`이고, OFF일 때는 `/scan`입니다. `/scan`은 양쪽에서 계속 발행됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/setup.bash
export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
ros2 param get /change_memory scan_topic
ros2 topic info /scan_filtered_for_change -v
ros2 topic info /scan -v
```

필터를 설치한 PC에서 **별도 실차 시험**을 원하면 기존 주행 스크립트에 다음 인자를 추가합니다. 패키지가 없으면 ON으로 실행하지 마세요. 반복 Gazebo 시험과 실제 좁은 물체 시험에서 이점과 누락 위험을 확인하기 전까지 기본 OFF를 유지합니다.

```bash
./scripts/run_classroom_navigation_16.sh laser_filter_for_change_memory:=true
```
