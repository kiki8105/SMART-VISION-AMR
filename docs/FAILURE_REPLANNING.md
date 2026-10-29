# 실패 접근을 기억하는 재계획

A*가 길을 만들었어도 차체가 실제로 진행하지 못하면 같은 길을 반복할 수 있습니다.
이번 보완은 **목표를 유지하면서 실패 지점과 진입 방향을 잠시 기억**하고 다른 접근을 찾습니다.
SLAM 지도나 저장된 PGM에 장애물을 그려 넣는 방식이 아닙니다.

## 동작 순서

1. 현재 라이다 장애물이 남은 경로를 가로막으면 기존 경로를 취소하고 재계획합니다.
   이미 지나온 구간의 장애물은 현재 경로를 취소하지 않습니다.
2. 센서·TF·위치 추정·실제 자동 속도 명령이 정상인데 6초간 위치나 헤딩 진척이
   없다면 실제 주행 실패로 분류합니다. 정상적인 제자리 헤딩 정렬은 진척으로 인정합니다.
3. Nav2 동작의 종료를 확인한 뒤 실패 지점 전방 약 0.20m와 시도한 경로 방향을 기록합니다.
   같은 구간을 같은 방향으로 통과하는 A* 간선을 120초간 제한합니다.
   후진이나 다른 방향 접근은 별도의 충돌 검사를 통과하면 사용할 수 있습니다.
4. A*와 경로 스무딩 모두 이 제한을 지킵니다. 다른 경로가 있어도 처음 선회에 필요한
   공간이 부족하면 최근 실제 전진 궤적을 따라 짧은 후진을 검토합니다.
5. 후진은 기존 조건을 유지합니다. 최근의 연속된 직선 이동, 실제 차체 여유,
   새 A* 출발점, 최신 후방 라이다, Nav2 취소 완료가 모두 확인돼야 합니다.
   기본 거리는 0.10~0.30m, 속도는 0.04m/s입니다.
6. 새 경로를 발행한 것만으로 회복 완료로 처리하지 않습니다. 실제 오도메트리와 지도 좌표의
   이동이 일치하고 새 경로로 0.35m 이상 진행해야 재진입을 확인합니다.
   다른 경로를 통과했더라도 원래 실패 접근의 기억은 만료까지 남겨 둡니다.

막힌 통로를 무조건 뚫거나 후진 횟수를 무한히 늘리지 않습니다. 안전한 대안이 없으면
목표를 유지하고 정지 상태에서 주기적으로 다시 확인합니다. 라이다·오도메트리·TF 단절,
수동 개입, 취소 응답 유실, 서버 연결 실패는 공간의 실패로 기록하지 않습니다.

## 세 번 시도해도 진행할 수 없는 장애물

경로를 막는 실제 라이다 장애물 때문에 계획 또는 주행이 세 번 실패하면 `OBSTACLE_HOLD`로
전환해 이동을 멈추고 목표를 유지합니다. 개별 라이다 갱신이나 경로 발행 횟수를 세는 것이
아닙니다. 근처에서 조금 움직이거나 새 경로에 잠깐 진입한 것만으로 장애물 시도 횟수를
초기화하지 않습니다. 장애물 제거 확인·목표 완료·다른 목표 지정 때 초기화합니다.

웹에는 다음 문구를 표시합니다.

> 장애물이 있어 경로 주행을 잠시 멈춥니다. 장애물을 치우면 경로를 재개합니다.

기존 장애물 셀을 정상 라이다 광선이 통과하는 것이 **연속 2초** 확인되면 해당 장애물과
연결된 실패 기억만 해제하고 같은 목표로 재계획합니다. 데이터의 TTL 만료·센서 단절·
가려진 영역·지도 변경만으로 제거를 판단하지 않습니다. 수동 정지와 비상정지 중에는
자동 재개하지 않습니다. 수동 조이스틱이 우선하며 자동 운용 상태로 돌아온 뒤에도
정상 센서로 제거 확인 시간을 다시 채워야 합니다.

이는 주변의 실제 미등록 장애물을 기다리는 기능입니다. 원래 지도에 들어 있는 벽이나
지도 밖/벽 안의 잘못된 목표는 장애물을 치워서 해결할 수 있는 경우로 취급하지 않습니다.

## 파일과 파라미터

- `smart_vision_amr/execution_progress.py`: 진척·반복 선회 판정
- `smart_vision_amr/path_follower.py`: 실제 자동 명령 증거 및 Nav2 종료 확인
- `smart_vision_amr/failure_memory.py`: 임시 방향별 실패 기억
- `smart_vision_amr/astar.py`: 방향별 금지 간선과 추가 비용
- `smart_vision_amr/astar_planner.py`: 목표 유지·재계획·후진 연결·재진입 확인

설정은 `ros2_ws/src/smart_vision_amr/config/`의 `simulation_nodes.yaml`과
`classroom_hardware_nodes.yaml`에 있습니다. 기존 속도·인플레이션 튜닝값은 유지했습니다.

| 노드 | 파라미터 | 기본값 | 의미 |
|---|---|---:|---|
| astar_planner | failure_memory_enabled | true | 실패 접근 기억 사용 |
| astar_planner | obstacle_hold_enabled | true | 재시도 제한 후 장애물 제거 대기 |
| astar_planner | obstacle_retry_limit | 3 | 장애물로 진행하지 못한 시도의 상한 |
| astar_planner | obstacle_clear_duration | 2.0 | 라이다가 장애물 제거를 연속 확인하는 시간, 초 |
| astar_planner | failure_memory_ttl | 120.0 | 기억 유지 시간, 초 |
| astar_planner | failure_memory_radius | 0.25 | 실패 구간의 중심선 반경, m |
| astar_planner | failure_memory_heading_tolerance | 1.0472 | 같은 진입 방향 범위, rad(60도) |
| astar_planner | failure_memory_penalty | 2.0 | 제한 구간 주변의 추가 A* 비용 |
| astar_planner | rejoin_progress_distance | 0.35 | 재진입 확인에 필요한 경로 진척, m |
| astar_planner | recovery_turn_clearance_margin | 0.03 | 차체 회전 반경에 더하는 여유, m |
| path_follower | execution_progress_window | 6.0 | 유효 증거로 정체를 판단하는 시간, 초 |
| path_follower | execution_min_progress | 0.05 | 위 시간 동안 필요한 최소 이동, m |
| path_follower | execution_heading_progress | 0.15 | 정상 정렬로 인정하는 헤딩 개선, rad |

기억은 현재 프로세스 안의 임시 상태입니다. 지도 프레임·크기·원점·해상도와 목표 좌표별로
구분합니다. 같은 목표를 새로 누르거나 라이다 지도가 갱신되는 것만으로 지워지지 않습니다.
프로세스를 종료하거나 유지 시간이 만료되면 사라집니다.

## 실행과 상태 확인

기존 실행 스크립트와 웹 임무 화면을 그대로 사용합니다. 별도 노드를 추가로 켤 필요는 없습니다.
새 코드는 실행 중인 프로세스를 다시 시작해야 반영됩니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
cd ~/Team2_workspace
colcon build --packages-select smart_vision_amr --symlink-install
```

가상환경은 각 터미널에서 다음처럼 실행합니다.

```bash
# 터미널 1: 기존 실차와 분리된 Gazebo 도메인 76
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/run_sim.sh
```

```bash
# 터미널 2: 시험용 목표를 전송하고 실제 도착 확인
cd ~/Team2_workspace/src/SMART-VISION-AMR
./scripts/check_sim.sh
```

```bash
# 터미널 3: Gazebo의 실패 접근·재계획 상태 관찰
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1
ros2 topic echo /smart_vision_amr/failure_memory_status
```

웹 운용 화면에는 실패 접근 회피, 대체 경로 대기, 후진, 재진입, 센서 회복 대기를 구분해
표시합니다. 현재 임무와 일치하는 최신 상태만 보여주며 비상정지·조이스틱이 항상 우선합니다.
진단 기록기는 실패 이벤트와 기억 상태도 함께 보관합니다.
`/smart_vision_amr/observed_visibility`는 측정 시각의 지도 좌표에서 확인된 영역을 발행합니다.
값 `-1`은 이번에 확인하지 못한 셀, `0`은 정상 광선이 통과한 셀, `100`은 새 장애물 반사점입니다.
이 토픽은 저장 지도를 덮어쓰지 않습니다.

저속 이동이나 짧은 좌우 정렬이 AMCL의 내부 이동 갱신 기준보다 작을 때는
`localization_health`가 `/request_nomotion_update`로 실제 라이다 필터 갱신을 요청합니다.
기본 간격은 2초이며 요청을 보냈거나 응답을 받았다는 이유로 위치 수신 시각·공분산을
갱신하지 않습니다. 실제 새 AMCL 메시지가 오지 않으면 기존 4초 이동 감시와 정지 잠금은
그대로 작동합니다.

## 기존 방식으로 돌아가기

두 설정을 모두 `false`로 바꾸고 주행 노드를 다시 실행하면 새 감지·실패 기억을 끕니다.

```yaml
/astar_planner:
  ros__parameters:
    failure_memory_enabled: false
    obstacle_hold_enabled: false
/path_follower:
  ros__parameters:
    execution_failure_enabled: false
```

수정 전 전체 코드 기준점은 Git 커밋 `cdc3f6f`입니다. 다른 팀원의 작업을 덮어쓰는
`git reset --hard` 대신 설정으로 기능을 끄거나 해당 커밋을 별도 작업 폴더에서 비교하세요.
이 문서의 소스 보완은 기존 1.2.0 DEB/ZIP 바이너리에 자동 반영되지 않습니다.
