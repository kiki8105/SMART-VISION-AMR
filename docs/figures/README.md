# PPT용 한 장 비교 다이어그램

- [PNG 1920×1080](navigation_system_comparison.png): PowerPoint에 바로 삽입.
- [SVG](navigation_system_comparison.svg): 확대해도 선명한 벡터 파일.
- [생성 스크립트](make_navigation_comparison.py): 문구와 배치 수정 후 재생성.

위 그림의 **A* 단독 계획**은 격자 경로 계산 단계만 뜻합니다. A* 자체가
속도 명령·장애물 회피를 모두 수행한다는 뜻이 아닙니다. **기본 TurtleBot3 운용**은
[ROBOTIS의 Navigation2 실습](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)에서
RViz 목표 지정·AMCL·Nav2를 쓰는 예시입니다. 기본 Nav2도 설정에 따라 장애물을
회피할 수 있습니다.

팀 구성은 [실차 실행 흐름](../TEAM_QUICKSTART.md#주행-흐름)과
[국소 우회](../LOCAL_DETOUR.md), [장애물 기억](../live_obstacle_avoidance.md)을
간추린 것입니다. 현재 경로 추종은 **Nav2 FollowPath**이며 Pure Pursuit라고
표시하지 않았습니다. 후방 ArUco는 섹터 도착 확인에 사용합니다.

재생성하려면 Ubuntu에서 Matplotlib와 Noto Sans CJK 글꼴을 설치한 뒤 저장소
루트에서 실행합니다.

```bash
python3 docs/figures/make_navigation_comparison.py
```
