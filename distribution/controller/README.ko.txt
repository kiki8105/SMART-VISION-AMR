TurtleBot3 Controller 1.1.0
==========================

지원 환경: Ubuntu 22.04 (Jammy), ROS 2 Humble 실행 라이브러리.
순수 Python 패키지이며 amd64/arm64 대상의 native 라이브러리는 apt가 설치합니다.
Ubuntu 24.04 및 Windows용 패키지가 아닙니다.

설치
----
ROS apt 저장소가 준비되어 있으면:
  sudo apt install ./turtlebot3-controller_1.1.0_all.deb

저장소의 distribution/controller/install_controller.sh는 ROS Humble apt 저장소가
준비된 PC에서 DEB와 필요한 실행 의존성을 설치합니다. 인터넷이 필요합니다.
조종기 PC에 TurtleBot3 소스, colcon 빌드, Gazebo는 필요하지 않습니다.

실행
----
앱 메뉴에서 '터틀봇3 조종기' 또는 'TurtleBot3 Controller'를 실행합니다.
Domain을 로봇과 맞추고 저장하면 연결을 다시 시작합니다. 일반 로봇은 실제 시간,
시뮬레이션은 /clock을 사용합니다. 자동 주행 프로필은 연결된 Team A*를 탐색합니다.
카메라 토픽과 차량 namespace는 필요할 때 변경할 수 있습니다.

기본 조작: 화살표 키/방향 버튼을 누르는 동안 이동, 놓으면 정지, Space 비상정지.
로봇 bringup과 카메라/지도/주행 서버는 연결 대상 PC 또는 로봇에서 실행 중이어야 합니다.
서로 통신 가능한 네트워크와 같은 Domain이 필요합니다.

파라미터 저장
-------------
'값 적용'으로 수락된 값을 '기본값 저장'하면 선택한 실제 실행 YAML을 수정하고
백업을 생성합니다. 다른 PC의 YAML을 네트워크로 자동 수정하지는 않습니다.
실습 실행 스크립트는 로컬 YAML을 자동으로 연결합니다.
재구성이 필요한 DWB 값은 Nav2 리셋/재실행 안내를 확인하세요.

명령행
------
  turtlebot3-controller --check
  turtlebot3-controller --help
  turtlebot3-controller --domain-id 16 --no-use-sim-time
  turtlebot3-controller --domain-id 76 --use-sim-time --navigation-profile team-astar

설정: ~/.config/turtlebot3/
로그: ~/.local/state/turtlebot3-controller/controller.log
제거: sudo apt remove turtlebot3-controller

소스: https://github.com/kiki8105/SMART-VISION-AMR
이 빌드에는 배포자의 개인 설정, 계정 인증정보, 사용자 로그를 포함하지 않습니다.
