# 강의실 웹사이트 상시 실행

`systemd` 사용자 서비스로 웹사이트만 실행한다. 터미널을 닫아도 계속 실행하고, 비정상 종료하면 5초 후 다시 실행한다. 1분 동안 5회 이상 시작에 실패하면 재시도를 멈추므로 로그에서 원인을 확인한 뒤 다시 시작한다.

강의실 실행 스크립트의 기본 접속 방식은 **ROS Domain ID**다. 다른 기기의 운용자는
Domain ID `16`, 재고 관리자는 사용자 번호 `1~99`와 Domain ID `16`을 입력한다.
별도의 접속 코드나 관리자 코드를 터미널에서 확인하지 않는다. Domain ID를 아는 같은
네트워크 사용자는 운용자 접속도 할 수 있으므로 신뢰하는 실습 네트워크에서 사용한다.
요청만 보내는 관리자 세션과 운용자 세션의 API 권한 구분은 유지한다.

로봇 브링업, 카메라, SLAM, 위치 추정, 주행 노드는 이 서비스가 실행하지 않는다. 웹사이트가 열려 있어도 로봇 운용은 각 실행 스크립트로 별도 시작한다. 저장된 지도는 로봇 연결 전에도 웹에서 볼 수 있다.

## 설치

Team2 워크스페이스에서 `smart_vision_amr` 패키지를 한 번 빌드한 다음, 일반 사용자 `kiki`로 실행한다. 설치 명령 앞에 `sudo`를 붙이지 않는다.

```bash
/home/kiki/Team2_workspace/src/SMART-VISION-AMR/scripts/install_classroom_web_service.sh
```

설치 경로는 현재 사용자 설정 디렉터리의 `systemd/user/smart-vision-classroom-web.service`다. 저장소의 실제 절대 경로를 서비스에 기록한다. 위 명령은 자동 실행을 등록하지만 현재 웹사이트를 시작하거나 다시 시작하지 않는다.

기존 터미널에서 웹사이트를 실행 중이면 그 웹사이트를 먼저 `Ctrl+C`로 종료한다. 같은 포트 8790을 두 프로세스가 동시에 사용할 수 없다. 그 후 서비스를 시작한다.

```bash
systemctl --user start smart-vision-classroom-web.service
```

기존 웹사이트가 꺼져 있다면 설치와 시작을 한 번에 할 수도 있다.

```bash
/home/kiki/Team2_workspace/src/SMART-VISION-AMR/scripts/install_classroom_web_service.sh --start
```

등록 없이 생성될 서비스 파일만 검토하려면 `--dry-run`을 사용한다.

```bash
/home/kiki/Team2_workspace/src/SMART-VISION-AMR/scripts/install_classroom_web_service.sh --dry-run
```

## 로그인 전부터 부팅 시 실행

기본 사용자 서비스는 사용자 로그인 시 시작한다. PC를 켰을 때 로그인 전부터 실행하고 로그아웃 후에도 유지하려면 한 번 설정한다.

```bash
loginctl enable-linger kiki
loginctl show-user kiki -p Linger
```

`Linger=yes`이면 적용된 것이다. PC 전원이 켜져 있어야 웹사이트에 접속할 수 있다. 같은 공유기/Wi-Fi에 연결한 기기에서 PC의 IP 주소와 포트 8790을 사용한다.

- 이 PC: [운용자 화면](http://127.0.0.1:8790/), [재고 요청 화면](http://127.0.0.1:8790/inventory)
- 같은 네트워크: `http://PC의_IP:8790/`, `http://PC의_IP:8790/inventory`

사용자 서비스의 `After=network-online.target`은 시작 순서 힌트다. 시스템 전체의 네트워크 준비 완료를 보장하는 설정은 아니다. 서버는 모든 인터페이스에 바인딩하므로 네트워크가 나중에 연결되어도 같은 포트로 접속할 수 있다.

## 상태 확인 및 관리

```bash
# 상태와 최근 로그
systemctl --user status smart-vision-classroom-web.service
journalctl --user -u smart-vision-classroom-web.service -n 60 --no-pager

# 로그 계속 보기: Ctrl+C는 로그 보기만 종료한다.
journalctl --user -u smart-vision-classroom-web.service -f

# 웹 코드/설정 수정 후 반영
systemctl --user restart smart-vision-classroom-web.service

# 중지 / 시작
systemctl --user stop smart-vision-classroom-web.service
systemctl --user start smart-vision-classroom-web.service

# 반복 실패 원인을 해결한 후 재시도
systemctl --user reset-failed smart-vision-classroom-web.service
systemctl --user start smart-vision-classroom-web.service

# 자동 실행 해제 및 현재 서비스 중지
systemctl --user disable --now smart-vision-classroom-web.service
```

서비스를 중지하면 해당 서비스에 속한 웹 프로세스와 자식 프로세스만 종료한다. 다른 터미널의 브링업, SLAM, 조종기 및 주행 노드는 종료하지 않는다.

설치 이후 저장소를 다른 경로로 옮겼다면 새 저장소 경로에서 설치 스크립트를 다시 실행하고 서비스를 다시 시작한다. 서비스는 `.bashrc`를 읽지 않으며 `run_classroom_web_16.sh`가 필요한 ROS 환경과 강의실 설정을 로드한다.
