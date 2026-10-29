# 배포 파일 만들기

저장소를 `~/Team2_workspace/src/SMART-VISION-AMR`에 클론하고
루트의 `./scripts/setup_sim.sh`를 실행한 환경을 기준으로 합니다.

바로 받을 배포본은 [조종기 DEB](releases/turtlebot3-controller_1.1.0_all.deb)와
[미로 실습 ZIP](releases/turtlebot3-aruco-lab_1.1.0.zip)입니다.
각 파일 옆의 `.sha256`으로 다운로드한 파일을 확인할 수 있습니다.

## 조종기 앱 DEB

저장소 루트에서 실행합니다. `tools/`의 현재 소스와 앱 아이콘을 묶습니다.

```bash
python3 distribution/controller/build_deb.py
./distribution/controller/install_controller.sh --check
./distribution/controller/install_controller.sh
```

`distribution/releases/turtlebot3-controller_1.1.0_all.deb`와 SHA-256이 생성됩니다.
받는 PC는 Ubuntu 22.04와 ROS Humble apt 저장소가 필요합니다.
설치 후 앱 메뉴의 **TurtleBot3 Controller**를 실행하고 로봇 도메인을 맞춥니다.
이 DEB는 조종기 앱이며 로봇 bringup·Nav2·웹 서버를 자동 시작하지 않습니다.

## 미로 실습 ZIP

```bash
python3 distribution/lab/build_lab_bundle.py --zip
```

저장소에 포함된 미로·SLAM 지도·마커·조종기 소스와 `setup_sim.sh`가 준비한
고정 ROBOTIS 소스를 묶습니다. `distribution/staging/turtlebot3-aruco-lab.zip`과
SHA-256이 생성됩니다. 기존 출력물을 갱신할 때는 `--replace`를 추가하세요.

DEB도 넣으려면 다음처럼 실행합니다.

```bash
python3 distribution/lab/build_lab_bundle.py --zip --replace \
  --deb distribution/releases/turtlebot3-controller_1.1.0_all.deb
```

받는 사람은 압축을 풀고 해당 폴더에서 순서대로 실행합니다.

```bash
./setup.sh --install-deps
./run_navigation.sh
```

다른 터미널에서 `./run_controller.sh`를 실행합니다. 조종기 소스가 함께
들어 있어 DEB 설치는 선택입니다. 웹 실행은 배포본의 `README.txt`를 따르세요.
시스템 ROS/Gazebo 패키지는 ZIP에 포함하지 않으며 설치에 인터넷이 필요합니다.

생성된 staging 폴더는 Git에서 제외합니다. 검증한 배포본은 `releases/`에
버전을 붙여 보관합니다. 개인 설정·로그·빌드 결과는 배포본에 복사하지 않습니다.
