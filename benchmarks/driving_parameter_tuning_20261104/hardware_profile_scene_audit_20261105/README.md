# 하드웨어 스트레스 두 프로필의 공통 상자 입력

`wheel_asymmetry` 첫 시험은 차량·HOME 안전 판정은 통과했지만, 상자를 생성하기 전에 `INVALID_INPUT / FIXED_SCENE_TOO_CLOSE_TO_OBSERVED_CELL`로 종료됐다. 장애물 주행 성능을 시험한 결과로 분류하지 않는다. 기존 실패 기록과 원래 고정 장면은 그대로 보존한다.

최신 `sensor_rates` 성공 시험과 이 시험의 최초 후보 검색 입력을 같은 선택 함수로 재생했다. 두 시험의 원래 경로 SHA는 같고, 관측은 각각 5셀·기억은 0셀이었다. 차이는 하단 관측 한 셀이다.

| 최초 입력 | 원래 상자와 가장 가까운 관측 셀 | 셀 중심 거리 | 원래 선택 결과 | 새 공통 장면 선택 결과 |
|---|---|---:|---|---|
| sensor_rates | (46,99) = (0.675,−0.275)m | 35.81cm | VALID | VALID |
| wheel_asymmetry | (46,98) = (0.675,−0.325)m | 31.35cm | INVALID_INPUT | VALID |

기존 35cm 관측 근접 기준은 변경하지 않았다. 추가 셀은 저장 지도상 unknown이며 기존 정적 팽창에 포함된다. 실제 월드에도 중심 (1.075,−0.1)m, 크기 0.65×0.50m인 `inferred_center_object`가 있다. 관측 셀은 그 왼쪽 면보다 7.5cm 서쪽이다. 초기 추정 위치가 truth보다 약 4.3cm 서쪽인 편향과 5cm 래스터 양자화가 있어, 이 셀을 유령 장애물이라고 삭제할 근거는 없다.

승인된 새 장면은 원래 상자를 왼쪽으로 약 8.39cm 옮긴 **(0.44,−0.60)m**이며 원래 접선·이동 방향은 유지한다.

```bash
--obstacle-scenario benchmarks/driving_parameter_tuning_20261104/scenarios/hardware_profiles_common_box.json
```

장면 SHA256: `b54c29018768664d3d9789bf4287efa5b8bb13e637987a2a19edd1fcadf7040e`

| 새 공통 장면의 오프라인 확인 | 결과 |
|---|---:|
| 반경 / A* 정적 팽창 | 22cm / 5셀, 변경 없음 |
| 최초 경로의 차체 영역과 상자 겹침 | 15.64cm: 기존 경로를 실제로 막음 |
| 최초 HOME 중심과 상자 중심 거리 | 73.40cm |
| 가장 가까운 관측 셀 중심 거리 | sensor_rates 40.11cm / wheel_asymmetry 36.17cm |
| 초기 위치와 4회 이동 위치의 경로 탐색 | 양쪽 모두 83 / 84 / 85 / 85 / 86셀 |
| 초기 HOME 로봇을 고정한 4회 상자 이동 중 최소 차체 간격 | 31.95cm; 해당 QA 요구 6.40cm |
| 명시된 월드 충돌 상자들과 전체 이동 상자의 최소 간격 | 8.86cm |

이 표는 **기록된 최초 입력의 사전 검사**다. 이동 중 로봇의 미래 위치는 재현하지 않았으므로 실제 상자 생성·4회 이동·우회 완주 PASS를 뜻하지 않는다. 실제 시험에서 fresh truth, 현재 경로 소유권과 전방성, 최신 관측·기억을 반영한 통행로, 2cm 차체 여유 및 4회 이동 QA를 다시 증명해야 한다. 두 프로필을 비교할 때에는 이 같은 장면 파일을 사용한다.

## 재현

저장소 루트에서 실행한다. ROS 메시지 Python 모듈만 읽으며 노드 생성·ROS 통신·가제보 실행·주행 명령은 하지 않는다.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 benchmarks/driving_parameter_tuning_20261104/hardware_profile_scene_audit_20261105/replay.py \
  --output /tmp/hardware-profile-scene-audit.json
```

`fixture.json`에 원본 두 시험의 후보 검색 이벤트, 관측·기억, 경로, 추정/실제 위치와 source SHA를 보존했다. `report.json`에 선택 함수 결과와 간격을 기록했다. 재생은 함께 제공되는 fixture·장면 JSON·저장소 지도와 선택 함수만 사용하며 `/tmp` 원시험 파일을 다시 읽지 않는다. 지도 YAML·PGM 및 장면 SHA를 확인한다.

범위: 경로 좌표는 로그의 소수점 4자리(평면 최대 약 0.071mm 양자화)이고 정확한 원경로 SHA가 두 시험에서 같다. 관측은 후보 검색 직전의 개수가 일치하는 독립 구독 샘플이며 개별 빔 원점까지 녹화되지는 않았다. 월드 간격은 원본 월드의 명시된 inline collision box만 포함하며 include/mesh 전체를 확장한 충돌 검사로 주장하지 않는다. 기존 35cm 근접 기준·실제 차체 크기·온라인 QA는 완화하지 않았다.
