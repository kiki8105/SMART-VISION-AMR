# SMART-VISION-AMR — TurtleBot3 Waffle Pi 실기 A* 주행

Ubuntu 22.04 / ROS 2 Humble용 첫 버전입니다. Raspberry Pi의 TurtleBot3 bringup과
Linux PC의 Nav2를 사용합니다. 첨부받은 `classroom_map.yaml` 및 `.pgm`을 포함합니다.
Gazebo, 카메라, ArUco 기능은 포함하지 않습니다.

## 구성

`/smart_vision_amr/goal` → A* (`/smart_vision_amr/path`) → Nav2 `/follow_path`
→ Nav2 controller `/cmd_vel` → TurtleBot3. `/scan` 관측은 A*의 동적 장애물
회피에 사용하고, Nav2 local costmap이 근거리 충돌 검사를 담당합니다.
장애물 기억은 기본적으로 꺼져 있으며 `memory_enabled:=true`로 켤 수 있습니다.

## 1. 로봇 Raspberry Pi

이미 하드웨어 bringup과 PC 간 ROS 토픽 공유가 된 환경에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-03
ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=false
```

Pi에서 `/scan`, `/odom`, `/tf`, `/cmd_vel`이 존재해야 합니다. LDS-03 드라이버는
설치된 ROBOTIS TurtleBot3 bringup의 `LDS_MODEL=LDS-03` 지원을 사용합니다.

## 2. Linux PC 설치 및 실행

PC와 Pi에서 같은 `ROS_DOMAIN_ID`를 사용합니다. PC에 ROS 2 Humble, Nav2,
TurtleBot3 navigation 패키지가 설치되어 있어야 합니다.

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3-navigation2 python3-colcon-common-extensions python3-rosdep
git clone https://github.com/waterhyunsu/SMART-VISION-AMR.git
cd SMART-VISION-AMR/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch smart_vision_amr hardware.launch.py
```

기본 Nav2 설정은 설치된 `turtlebot3_navigation2/param/humble/waffle_pi.yaml`을
사용합니다. 지도는 저장소에 포함된 교실 지도입니다. 실제 지도와 로봇 위치가
맞아야 합니다. 다른 지도를 사용하려면 `map:=/절대경로/map.yaml`을 지정합니다.

## 3. 주행 전 확인

로봇을 넓고 비어 있는 곳에 놓고 다음을 확인합니다.

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 action list | grep follow_path
ros2 lifecycle get /controller_server
```

RViz에서 실제 위치에 맞춰 **2D Pose Estimate**를 지정한 뒤
`ros2 run tf2_ros tf2_echo map base_footprint`가 연속적으로 출력되는지
확인합니다. 지도와 위치 추정이 맞지 않으면 목표를 보내지 마세요.

## 4. 첫 목표

지도 안의 가깝고 비어 있는 지점을 먼저 선택합니다. 좌표는 예시가 아니며
실제 로봇 위치와 지도에서 확인한 값으로 바꿔야 합니다.

```bash
ros2 topic pub --once /smart_vision_amr/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
ros2 topic echo /smart_vision_amr/navigation_status
ros2 topic echo /cmd_vel
```

중단은 `ros2 service call /smart_vision_amr/cancel_navigation std_srvs/srv/Trigger '{}'`입니다.
첫 주행은 바퀴를 띄워 방향을 확인하고, 이후 넓은 공간에서 저속으로 수행하세요.
프로젝트는 실제 장비에서 아직 검증되지 않았습니다.

## 문제를 좁히는 순서

1. `/map`이 없으면 map server와 지도 경로를 확인합니다.
2. `map -> base_footprint` TF가 없으면 AMCL 초기 위치 및 Pi/PC 시계 동기화를 확인합니다.
3. `/smart_vision_amr/path`가 비면 A* 로그의 시작/목표 셀과 지도 장애물 판정을 확인합니다.
4. FollowPath가 실패하면 controller 로그, local costmap, `/scan`을 확인합니다.
5. `/cmd_vel`이 나오는데 바퀴가 돌지 않으면 Pi의 OpenCR/모터 쪽을 확인합니다.

`config/nodes.yaml`의 `obstacle_inflation_cells`는 0.05 m 해상도 기준
6셀(0.30 m)입니다. 실제 통로 폭과 로봇 외형을 확인한 뒤 조정하세요.
Nav2의 controller/local costmap은 설치된 TurtleBot3 Humble 설정을 따릅니다.

## 출처

A* 격자 탐색, 장애물 관측/기억 및 FollowPath 연결은
[`resilient-amr-astar`](https://github.com/waterhyunsu/resilient-amr-astar)의
구현을 참고해 실기 전용 패키지로 구성했습니다.
