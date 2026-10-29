# Gazebo Classic용 ArUco 마커

이 단계는 카메라로 볼 수 있는 ArUco 표지판을 생성하고 Gazebo에 배치합니다.
기존 A*·Nav2 주행 코드는 그대로 사용합니다. 마커를 배치하는 것만으로
마커 검출이나 정지·도킹 같은 제어 동작이 추가되지는 않습니다.

## 1. 포함된 모델과 새 모델 생성

마커 ID `0~3`의 SDF·메시·재질·PNG와 미로 월드가 저장소에 포함되어 있습니다.
제공 실행 스크립트는 `gazebo_models/`를 Gazebo 모델 경로로 지정하므로
처음 클론한 PC에서도 모델 생성이나 `~/.gazebo/models` 등록 없이 실행합니다.
같은 미로를 바로 주행하려면 [웹 임무 안내](web_mission.md)를 사용합니다.

다른 ID나 크기의 모델을 생성할 때만 OpenCV ArUco 지원을 설치하고 생성기를 실행합니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
sudo apt install -y python3-opencv
/usr/bin/python3 scripts/generate_aruco_models.py --ids 0 1 2 3
```

기본 마커 규격은 다음과 같습니다.

| 항목 | 기본값 |
|---|---|
| 사전 | `DICT_4X4_50` |
| 배치할 ID | `0, 1, 2, 3` |
| 검은 테두리를 포함한 마커 한 변 | `0.20 m` |
| 한쪽 흰 여백 | `0.04 m` |
| 바깥 흰 여백을 포함한 보드 한 변 | `0.28 m` |

여러 ID를 한 번에 만들 수도 있습니다. `DICT_4X4_50`의 ID 범위는 `0~49`입니다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
/usr/bin/python3 scripts/generate_aruco_models.py \
  --ids 0 1 2 \
  --marker-size 0.20
```

기본 모델은 아래 폴더에 생성하고 `~/.gazebo/models`의 심볼릭 링크로
등록합니다. 모델 안에는 `model.config`, `model.sdf`, 표면 메시, 재질 파일,
마커 PNG가 들어갑니다. `colcon build`는 필요하지 않습니다.

같은 결과로 재실행할 수 있지만 기존 모델과 결과가 다르면 덮어쓰지 않고
중단합니다. 크기·여백을 바꿔 시험할 때는 다른 `--output-dir`를 지정하고
`--no-register`로 별도 생성합니다. 제공 모델 ID `0~3`으로 실습할 때는
생성기를 실행하지 않고 아래 미로 실행부터 진행할 수 있습니다.

```text
~/Team2_workspace/src/SMART-VISION-AMR/gazebo_models/smart_vision_aruco_4x4_50_id_0
~/.gazebo/models/smart_vision_aruco_4x4_50_id_0
```

[온라인 마커 생성기](https://chev.me/arucogen/)에서 직접 만들 경우에도 사전
`4x4_50`, ID `0`, 크기 `200 mm`로 선택하면 같은 규격입니다.

## 2. 미로 월드 매핑과 마커 4개 확인

`simulation_maze_aruco.world`는 번갈아 위·아래로 돌아가야 하는 칸막이와
작은 돌출 벽이 있는 별도 미로 월드입니다. ID `0~3`은 시작 구역, 두 경유
구역, 도착 구역의 벽면에 배치했습니다. 로봇 시작 위치는 `(-3, -2)`,
시작 방향은 `+X`입니다.

터미널 1:

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_sim_aruco.sh use_motion_mux:=false
```

이 스크립트는 저장소의 마커 모델 4개와 미로 월드를 확인한 뒤
**기본적으로 `slam:=true`로 실행**합니다. PyQt 수동 조종을 위해 mux를 끕니다.
예전에 저장한 지도를 사용하지 않으므로, 컨트롤러로 직접 주행하면서 새 지도를 만듭니다.
이미 포함된 저장 지도로 실행하려면 별도 매핑 없이 `scripts/run_web_mission_sim.sh`를
사용합니다. 기본 지도는 `ros2_ws/src/smart_vision_amr/maps/maze_aruco_20261026.yaml`입니다.

터미널 2:

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_sim_controller.sh
```

매핑을 마친 뒤 **터미널 1에서 `Ctrl+C`**를 누르면 지도 저장 후 종료합니다.
저장 파일은 아래 형식으로 생성됩니다. 터미널에 출력된 실제 경로를
다음 저장 지도 주행에서 사용합니다.

```text
~/Team2_workspace/maps/slam_날짜_시간_실행번호/map.yaml
~/Team2_workspace/maps/slam_날짜_시간_실행번호/map.pgm
```

저장 후 같은 미로에서 A* 주행을 시험하려면 아래처럼 새 지도를 지정합니다.
`map:=` 뒤는 실제 저장된 경로로 바꿉니다.

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_sim_aruco.sh \
  slam:=false \
  map:="$HOME/Team2_workspace/maps/실제_저장_폴더/map.yaml"
```

마커 위치와 카메라로 정면 관측하기 좋은 로봇 위치는 서로 다릅니다.
**주행 목표는 마커가 붙은 벽 좌표 대신 아래 관측 위치를 사용합니다.**

| ID / 역할 | 마커 중심 `(x, y, z)` m | 관측 위치 `(x, y)` m | 관측 방향 yaw rad |
|---|---|---|---|
| `0` / 시작 구역, 첫 칸막이 서쪽 면 | `(-2.062, -2.00, 0.22)` | `(-3.00, -2.00)` | `0` |
| `1` / 경유 구역 1, 둘째 칸막이 서쪽 면 | `(0.238, 3.00, 0.22)` | `(-1.00, 3.00)` | `0` |
| `2` / 경유 구역 2, 셋째 칸막이 서쪽 면 | `(2.538, -2.80, 0.22)` | `(1.40, -2.80)` | `0` |
| `3` / 도착 구역, 동쪽 벽 | `(4.990, -2.50, 0.22)` | `(3.90, -2.50)` | `0` |

월드 형상을 5cm 격자로 바꿔 장애물 팽창 6셀을 적용했을 때 관측 위치
4개가 모두 통과 가능한 셀이며, 순서대로 A* 경로가 생성됨을 확인했습니다.
실제 SLAM 지도에서는 관측 결과에 따라 벽의 두께와 자유 공간이 달라질
수 있으므로 새로 저장한 지도에서도 목표와 경로를 확인합니다.
표의 yaw는 **마커를 보는 카메라 방향**입니다. 현재 경로 추종이 목표에서
이 방향까지 맞춰 주는지는 별개이므로 필요하면 도착 후 수동으로 회전합니다.

이 월드는 마커의 배치만 제공합니다. ID 순서로 자동 방문하거나 마커를
검출해 다음 목표를 고르는 임무는 이후 검출 노드와 연결할 때 추가합니다.

## 3. 기존 월드에 마커를 직접 배치

터미널 1에서 기존 시뮬레이션을 시작합니다. 저장 지도에서 주행하려면 평소
사용하던 `map:=/절대경로/map.yaml` 인자를 함께 지정합니다.

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_sim.sh
```

다른 터미널에서 아래 명령으로 로봇 전방에 마커를 놓을 수 있습니다.
이 스크립트도 시뮬레이션의 ROS domain 76과 Gazebo 포트 11376을 사용합니다.

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/spawn_aruco.sh \
  --id 0 --dictionary DICT_4X4_50 \
  --x -2.0 --y -2.0 --z 0.20 --yaw 0
```

`x/y/z` 단위는 m, `yaw`는 rad입니다. 위 값은 기본 로봇 위치 `(-3, -2)`에서
`+X` 방향으로 바라볼 때 전방 1 m에 해당합니다. `z`는 보드 중심 높이입니다.
인쇄면은 모델의 `-X` 쪽을 향하므로 `yaw=0`에서 `-X` 쪽의 카메라에 보이고,
반대쪽은 흰색입니다. 다른 위치에서 바라볼 때는 `--yaw`를 바꿉니다.

마우스로 놓으려면 Gazebo 왼쪽의 **Insert** 탭에서
`smart_vision_aruco_4x4_50_id_0` 모델을 선택하고 월드에 놓습니다.
목록에 바로 보이지 않으면 모델 생성 후 Gazebo를 다시 시작합니다.
마커 정면을 로봇 카메라 쪽으로 향하게 배치하고, 로봇·벽·기존 물체와
겹치지 않도록 합니다.

카메라 영상과 마커 확인은 [웹 임무 안내](web_mission.md)의 웹 화면에서 진행할 수 있습니다.
PyQt로 수동 SLAM을 진행할 때는 터미널 1을 `slam:=true use_motion_mux:=false`로
시작하고 터미널 2에서 포함된 조종기를 사용합니다.

```bash
~/Team2_workspace/src/SMART-VISION-AMR/scripts/run_sim_controller.sh
```

모델을 실행 중인 월드에 추가한 배치는 Gazebo를 끄면 사라집니다.
배치를 계속 사용하려면 Gazebo의 **File → Save World As**로 별도 `.world`를
저장하고 다음 실행 때 `world:=/절대경로/저장한.world`로 불러옵니다.

## 4. 이후 카메라 검출과 연결할 때

다음 단계의 검출 노드는 `/camera/image_raw`를 받아 OpenCV ArUco 검출기로
ID와 네 모서리를 확인하게 됩니다. 검출기의 사전도 반드시
`DICT_4X4_50`으로 맞춰야 합니다. 같은 ID라도 사전이 다르면 다른 마커입니다.

거리·자세 추정을 추가할 때 사용하는 `marker_length`는 **흰 여백을 제외한
검은 정사각형의 한 변 `0.20 m`**입니다. 보드 전체 크기 `0.28 m`를 넣으면
추정 거리에 비례 오차가 생깁니다. 이때는 `/camera/camera_info`의 카메라
보정 정보도 필요합니다.

마커가 잘 안 보이면 정면 방향, 화면에서 차지하는 크기, 조명, 가림 여부를
확인합니다. 뒷면의 반전 무늬나 먼 거리에서 몇 픽셀로 축소된 무늬는 정상
마커처럼 검출되지 않을 수 있습니다.

## 참고

- [Gazebo Classic: 모델의 visual·collision 및 메시 구성](https://classic.gazebosim.org/tutorials?cat=build_robot&tut=build_model)
- [OpenCV: ArUco 사전, 마커 생성과 검출](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

OpenCV 공식 최신 문서의 생성 API는 `generateImageMarker()`이며, 현재 PC의
OpenCV 4.5.4에서는 `drawMarker()`를 사용합니다. 제공 생성기가 버전에 맞게
처리하므로 문서의 최신 API를 그대로 복사할 필요는 없습니다.
