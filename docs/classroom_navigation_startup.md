# 실차 강의실 주행 노드 시작 방식

`scripts/run_classroom_navigation_16.sh`는 Nav2를 `use_composition:=false`로
실행한다. 지도 서버, AMCL, 제어기와 수명주기 관리자가 각각 별도 프로세스로
시작되며, 웹 서비스의 속도 mux를 그대로 사용한다.

2026-10-27 실제 PC에서 기존 공유 컨테이너 방식은 제어기 구성 도중
`Failed to change state for node: controller_server`로 시작을 중단했다.
제어기는 inactive, 이후 주행 노드는 unconfigured에 남아 경로를 따라갈 수 없었다.
이 프로필은 구성 동작을 프로세스별로 분리해 다시 시작하고 상태를 확인하기 위한
설정이다. 이 변경만으로 초기 위치나 Homepoint를 자동 확정하지는 않는다.

저장 지도 주행 시에는 로봇 Bringup과 카메라만 켜고 **SLAM/Cartographer는
종료**한다. SLAM 지도와 Nav2의 저장 지도가 둘 다 `/map`에 발행되면 AMCL,
A* 계획, 웹이 서로 다른 지도를 받을 수 있다. 강의실 주행 스크립트는 시작 전에
ROS 도메인 16의 지도 발행자를 확인하고 기존 발행자가 있으면 이름을 표시하며
기동을 거절한다. 이미 Nav2를 켠 뒤 SLAM이 켜졌다면 SLAM을 실행한 터미널에서
Ctrl+C로 종료하고 Nav2를 다시 시작한다. 웹의 `설정 지도와 다른 /map`은 실제
지도 충돌을 뜻하므로 무시하고 출발하지 않는다.

SLAM 터미널을 찾지 못하면 로봇 PC와 다른 ROS 도메인 16 PC에서 다음 명령으로
프로세스를 찾는다. 해당 SLAM 작업이 더 필요하지 않음을 확인한 뒤 그 터미널에서
Ctrl+C로 종료한다. 터미널이 없다면 출력된 PID에 `kill PID`를 실행한다.

```bash
ps -eo pid,args | grep -E '[c]artographer_occupancy_grid_node|[c]artographer_node|[s]lam_toolbox'
```

지도 발행자를 확인하는 명령은 다음과 같다. 저장 지도 주행 중에는 `/map_server`
하나만 Publisher로 나타나야 한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
ros2 topic info /map -v
```

일반 `hardware.launch.py`의 기본값은 기존과 같은 `use_composition:=true`다.
필요하면 실행 인자로 두 방식 중 하나를 선택할 수 있다. 강의실 스크립트 뒤에
`use_composition:=true`를 붙이면 해당 실행만 공유 컨테이너 방식으로 재정의한다.
Humble 하위 런치의 Python 조건식에 맞게 전달값을 `True`/`False`로 정규화하므로
명령줄에서는 소문자 `true`/`false`도 사용할 수 있다.

Humble의 현재 lifecycle manager에는 `service_timeout`이라는 설정이 없다.
확인되지 않은 제한 시간을 추가하거나 heartbeat 감시를 끄는 변경은 하지 않았다.

2026-10-28 실차 시작 직후 지도 서버의 bond가 4초 안에 연결되지 않아
`lifecycle_manager_localization`이 기동을 중단한 사례가 있었다. 지도와 LiDAR가
정상 수신되고 로봇이 정지해 있다면, PC에서 다음 순서로 수명주기를 재시도한다.

```bash
source /opt/ros/humble/setup.bash
source /home/kiki/Team2_workspace/install/setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 3}'
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
```

두 응답의 `success: true`를 확인한 뒤, 실차가 등록된 홈에 정지하고 **후방
카메라가 0번 마커를 볼 때만** `scripts/set_classroom_home_pose_16.sh`로 AMCL
초기 위치를 입력한다. 이 명령은 저장 지도와 수신 지도 일치, 지도 발행자 수,
LiDAR/오도메트리 최신성, 정지 상태, 현재 스캔과 등록 홈 주변 지도의 일치도를
확인하고 `map→base` TF 생성까지 기다린다. 일치도가 낮다면 실제 위치 또는
지도와 현장 배치의 차이를 먼저 확인한다. 물리적 홈 위치를 직접 확인했을 때만
`--force-verified-home`을 명시할 수 있으며, 이 옵션도 지도 충돌과 센서 오류는
우회하지 않는다. 재시작마다 AMCL은 초기 위치가 필요하다. 로봇이 다른 곳에
있을 수 있어 홈 좌표를 자동으로 확정하지 않는다.
