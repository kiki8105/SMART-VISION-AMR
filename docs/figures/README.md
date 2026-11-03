# PPT용 한 장 비교 다이어그램

- [PNG 1920×1080](navigation_system_comparison.png): 투명 배경. PowerPoint에 바로 삽입.
- [SVG](navigation_system_comparison.svg): 투명 배경 벡터 파일. 확대해도 선명함.
- [생성 스크립트](make_navigation_comparison.py): 문구와 배치 수정 후 재생성.

위 그림의 **기본 A* 탐색**은 격자 경로 계산 단계만 뜻합니다. A* 자체가
속도 명령·장애물 회피를 모두 수행한다는 뜻이 아닙니다. **기본 TurtleBot3 운용**은
[ROBOTIS의 Navigation2 실습](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)에서
RViz 목표 지정·AMCL·Nav2를 쓰는 예시입니다. 기본 Nav2도 설정에 따라 장애물을
회피할 수 있습니다.

팀 구성은 [실차 실행 흐름](../TEAM_QUICKSTART.md#주행-흐름)과
[국소 우회](../LOCAL_DETOUR.md), [장애물 기억](../live_obstacle_avoidance.md)을
간추린 것입니다. 현재 경로 추종은 **Nav2 FollowPath**이며 Pure Pursuit라고
표시하지 않았습니다. 후방 ArUco는 섹터 도착 확인에 사용합니다.

그림의 **장애물 셀 기억** 블록은 실차 기본 설정
`memory_enabled:=true memory_persist_to_disk:=false`를 나타냅니다. 확정된
장애물 셀은 현재 주행 프로세스 안에서 섹터 간 유지되고, 유효한 빈 공간을
재관측하면 해제됩니다. 프로세스를 재시작하면 비워지며 원본 SLAM 지도는
바뀌지 않습니다. 확정 전 실시간 LiDAR 관측도 경로 판단에 즉시 반영됩니다.

경로가 막히면 차체 여유가 있는 **국소 A* 우회**로 기존 경로 앞쪽에 합류합니다.
우회할 수 없으면 전체 재계획을 시도하고, 직전 궤적과 후방 공간이 안전할 때만
짧게 후진해 다시 계획합니다. 안전 경로가 없으면 목표를 유지하고 정지합니다.
도식의 **실패 접근 기억**은 정체 지점·진입 방향을 최대 120초간 피하는 별도
기능이며, 장애물 셀 기억과 혼동하지 않아야 합니다.

재생성하려면 Ubuntu에서 Matplotlib와 Noto Sans CJK 글꼴을 설치한 뒤 저장소
루트에서 실행합니다.

```bash
python3 docs/figures/make_navigation_comparison.py
```
