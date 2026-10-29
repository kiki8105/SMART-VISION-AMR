# 팀원 주행 개선 선택 통합 (2026-10-29)

기준은 이 저장소의 `b49850e`와 팀원 저장소
[waterhyunsu/SMART-VISION-AMR](https://github.com/waterhyunsu/SMART-VISION-AMR/tree/988cc1a95aea5cc3d879f3b4675c85583f4bcd5d)입니다.
`f33a2ce`, `4126113`, `988cc1a`의 필요한 로직을 현재 구조에 맞게 통합했습니다.
전체 파일이나 실행 스택을 교체하지 않았습니다.

## 반영 내용

- **벽 옆 실제 장애물 구별:** 지도에서 해당 레이저 방향의 첫 점유 셀까지 거리를 계산합니다.
  측정 거리와 차이가 `mapped_wall_range_tolerance_m` 이내일 때만 지도 벽의 반사로 제외합니다.
  벽과 가깝다는 이유로 장애물을 다시 제외하던 planner 필터도 제거했습니다.
  미확인 셀은 경로 계산에서 계속 막지만, 알려진 벽이라는 근거로 사용하지 않습니다.
- **탈출 시 차체 여유 검사:** 장애물의 팽창 영역을 벗어나는 예외 경로에도 로봇 반경,
  추가 여유와 격자 간 이동 여유를 적용합니다. 원본 장애물·미확인 영역·지도 경계를
  차체가 침범하는 탈출은 허용하지 않습니다.
- **Nav2 입력 확인:** 출발 전과 주행 중에 라이다·오도메트리·local costmap·footprint의
  수신 시각, 프레임 및 수치·형상 일관성을 확인합니다. footprint의 헤더가 오래됐어도
  실제 수신과 차체 형상이 최신 odometry와 일치하는 경우는 허용합니다.
  입력 문제는 경로 실패 기억에 넣지 않으며, 기존 취소 확인 절차와 목표 유지로 처리합니다.

원형 footprint는 새로 수신된 위치·반경·형상으로 확인합니다. 비대칭 footprint의 오래된
헤더를 허용하려면 먼저 최신 헤더로 확인한 차체 기준 형상이 필요합니다.
잘못된 다각형이나 NaN 값을 단순히 중심점이 맞는다는 이유로 통과시키지 않습니다.

## 유지한 동작

독립 EKF, 로봇 측 motion mux 소유권, 웹·조이스틱 우선권, ArUco 도착 확인,
방향별 실패 접근 기억, 3회 실패 후 장애물 대기, 2초간 실제 빈 공간 확인 후 재개,
레이저 **측정 시각** TF·짧은 대기 큐·visibility 발행을 유지합니다.
기존 속도·가속도·도착 허용 오차 및 인플레이션 튜닝값은 변경하지 않았습니다.

지속 장애물 기억의 5회·2초 확정/해제는 원래 있던 기능입니다.
기억된 셀이 가려져 있으면 물체를 치워도 자동 해제 근거가 쌓이지 않을 수 있습니다.
이번 통합이 모든 실차 막힘을 해결했다는 의미는 아닙니다.

## 설정과 재실행

`config/nodes.yaml`, `config/classroom_hardware_nodes.yaml`,
`config/curve_tuning_simulation_nodes.yaml`의 `/change_memory` 아래:

```yaml
mapped_wall_range_tolerance_m: 0.05
```

단위는 미터이며 지도 해상도와 독립적입니다. 5cm 이내로 벽과 붙어 있는 물체는
벽 반사와 구별하기 어렵고, 지도·위치 오차가 크면 벽을 실제 장애물로 판단할 수 있습니다.
예전 `mapped_wall_return_tolerance_cells`는 호환성을 위해 남겨두었으며
분류가 끝난 실시간 장애물을 planner에서 무시하는 용도로 사용하지 않습니다.

업데이트는 PC에서 실행합니다. 로봇이 정지한 상태에서 기존 주행 런치를 종료한 뒤:

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
git pull --ff-only
cd ~/Team2_workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_vision_amr
source install/setup.bash
```

그 다음 기존 시뮬레이션 또는 실차 실행 스크립트를 다시 사용합니다.
별도의 로봇·웹 실행 명령으로 바뀌지 않았습니다.
기존 실행 중인 프로세스는 소스 변경만으로 갱신되지 않습니다.

## 검증 범위

회귀 시험은 벽 반사와 가까운 실물의 구별, 기억 확정·해제, 측정 시각 TF·visibility,
차체 겹침과 지도 경계, 기존 실패 기억·HOLD 재개, 입력 지연·손상과 취소 중 경로 재수신을
확인합니다. 단위 시험과 빌드는 실차 주행 성공을 대신하지 않습니다.
이번 통합 작업에서는 실차 도메인 16의 노드나 웹 서버를 재시작하지 않습니다.

통합 시점 검증 결과: 오프라인 회귀 **1,055개 통과**, 실제 ROS 런타임을 만드는
시험 **80개 제외**, `colcon build --packages-select smart_vision_amr` 통과.
오프라인 전체 회귀에서는 실제 ROS context/Node 초기화를 차단했습니다.

노드를 만들지 않는 이번 변경의 핵심 회귀만 재실행하려면 저장소 루트에서:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/smart_vision_amr:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  ros2_ws/src/smart_vision_amr/test/test_wall_range_classification.py \
  ros2_ws/src/smart_vision_amr/test/test_dynamic_escape_footprint_safety.py \
  ros2_ws/src/smart_vision_amr/test/test_navigation_input_preflight.py
```

테스트 실행에 `python3-pytest`가 필요합니다.
기준 버전으로 되돌릴 때는 이 통합 커밋을 `git revert`하고 다시 빌드합니다.
기존 설정·지도·배송 기록을 삭제하거나 저장소를 강제 초기화할 필요는 없습니다.
