# 관측→기억 승격 시 충돌 기하 불일치

실제 cold-start·센서 10/20/20 Hz 시험의 장애물 우회는 **실패**했다. 상자 4회 이동과 최종 Home 복귀는 확인됐지만 상자 제거 전 통과하지 못했다. 이 기록은 수정 후 통과 결과로 바꾸지 않았다.

`fixture.json`은 t≈52.37초의 관측 13셀·기억 15셀, 직전 실제 발행 경로 67점, 지도·설정·원자료 SHA, 실제 상자 크기/좌표, 원본 NoPath 진단을 보존한다. 첫 기억 확정별 실제 위치와 TF 합성 위치도 포함한다. `/tmp`의 원자료 없이 클론된 저장소의 지도와 이 파일만으로 재생할 수 있다.

| 동일 입력의 기하 재생 | 기존 기억 square5 | 관측·기억 공통 footprint22cm |
|---|---:|---:|
| 기억 팽창 셀 | 242 | 215 |
| 보수 관측 팽창 셀 | 239 | 239 |
| 가상 reserve만 추가한 셀 | 36 | 37 |
| 시작이 기억 팽창 안 | 예 | 아니오 |
| 보수 primary 경로 | 없음 | 없음 |
| 기존 soft-reserve fallback | 실제 기억 시작 gate로 금지 | 67셀 경로 |

새 경로는 실제 상자 대비 차체 여유 하한 **10.975cm**, 정적 지도·unknown·관측·기억 원본 사각형 전체 대비 원형 차체 여유 하한 **5.40cm**다. 전체 선분을 2mm 간격으로 검사하고 거리 함수의 Lipschitz 상한에 따라 1mm를 공제했다. 정적 지도 5셀 팽창, 22cm 반경, 기존 margin, 원본 관측/기억 셀은 그대로다.

기억 확정은 지속성만 바꾼다. 실시간 관측과 기억에 같은 `inflate_cells_footprint` 경로를 적용해 확정 순간 충돌 영역이 사각형으로 커지는 현상을 없앴다. 가상 reserve의 보수 검색→soft fallback 및 실제 센서·정지·후진 검증 정책은 유지한다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
python3 benchmarks/driving_parameter_tuning_20261104/memory_footprint_promotion_20261105/replay.py \
  --output /tmp/memory-promotion-replay.json
```

ROS Python 메시지/라이브러리를 가져오지만 Node·DDS·Gazebo 프로세스를 시작하거나 주행 명령을 발행하지 않는다. `report.json`은 오프라인 기하 재생 결과이며 폐루프 주행 성공 증거가 아니다. 같은 callback의 원시 scan 전체는 녹화되지 않아 이 재생으로 새로운 실행 인증을 만들었다고 주장하지 않는다.
