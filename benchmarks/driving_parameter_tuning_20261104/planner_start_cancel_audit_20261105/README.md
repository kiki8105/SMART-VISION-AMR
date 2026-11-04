# 공통 상자 시험: 출발 첫 간선 인증 소유권 감사

실제 시험 `/tmp/classroom-transfer-cold_sensor_common_validated-20261105/25_web_inventory_sector3_moving_obstacle`은 상자 생성·4회 이동은 완료했지만 우회를 완료하지 못해 **FAIL**이다. 실제 상자 최소 여유는 약8.04cm였다. 기존 실패 자료는 `../physical_transfer_20261105/cold_sensor_common_validated_failed/`에 보존됐다. 이 오프라인 감사는 실제 완주 성공을 뜻하지 않는다.

문제는 DWB 실행실패 예산이 아니라, 새 우회 경로가 즉시 빈 경로로 바뀌며 정상적인 준비 회전까지 반복 취소된 것이다. follower의 실제 material Spin 시작6회 예산은 실제 요청을 센 결과이므로 올리거나 환불하지 않았다.

| planner 로그의 취소 분류 | 건수 |
|---|---:|
| 전체 `Fresh observed cells persist on remaining route` 로그 | 20 |
| 로그의 실제 blocker가 현재 경로 start(31,96) 한 셀에만 교차 | 12 |
| 그중 기존 dynamic halo로 경로를 발행했지만 첫 간선 인증을 설치하지 않은 분기 | 5 |
| 위5건 중 동일82점 경로 | 3 |
| 나머지 start-only 취소: 기존 fallback 인증이 발행된 분기 | 7 |

여기서 blocker 로그는 모두16셀 미만이므로 로그 자체의 셀 목록 절단은 없다. 원래 경로를 다시 검사한 결과가 아니라 각 취소 직전에 실제 발행된 경로와 교차시켰다. 전체 observed·memory 구독 스냅샷은 planner 내부 DDS 처리 순서와 같다고 단정하지 않는다.

확정된 소유권 누락은 `escape_already_present`일 때이다. A*는 기존 동적 팽창 이탈 경로를 찾지만, 기존 코드가 연속 출발 인증 생성을 건너뛰므로 발행 이후 callback은 같은 경로의 출발 셀을 다시 일반 점유로 취급할 수 있었다. 기록된 선택 간선은 **(31,96)→(30,96)**, 발행 좌표는 **(−0.075,−0.425)→(−0.125,−0.425)m**이다.

수정은 검색·반경·점유 범위를 변경하지 않는다. 실제 선택된 첫 간선 하나를 같은 fresh 연속 차체 증명으로 인증하고 발행에 묶는다. 현재 출발 셀이 다음 scan에서 잠깐 free여도 같은 raw body/sweep 증명이 성립하는 동안 이 소유권을 유지한다. 인증에 필요한 start1비트는 인증 입력에만 쓰며 실측 hard mask는 변경하지 않는다. 실제 기억 팽창의 start, 이후 hard/static 셀, 재진입, start를 측면으로 사용하는 인접 대각은 계속 거부한다. 최종 fresh 재증명과 단일 간선 설치도 유지한다.

`fixture.json`은 t37.505 발행을 앞뒤로 둘러싼 두 **실제 관측 frame**을 담는다. 앞 frame(약80.7ms 전)은(36,94)를 포함하고 다음 frame(약67.5ms 후)은 포함하지 않는다. 앞 frame은 measured first-cell fallback, 다음 frame은 기존 conservative halo 경로를 재생한다. 둘 중 어느 것이 실제 planner 내부 발행 시점의 rawset이었는지는 녹화만으로 단정할 수 없다. 두 frame 모두 실제 첫 간선의 raw static/unknown·관측·기억 전체 cell-square 차체 간격은 양수다. 다른 위치의 관측도 함께 변하므로, 원래 발행된 전체 suffix가 두 frame 모두에서 계속 유효하다고 주장하지 않는다. 연결 회귀는 각 frame에서 다시 계획한 전체 경로의 이후 measured hard 셀 보존을 따로 확인한다. 소스 해시와 원래 진단·경로·관측 원본은 fixture에 보존했다.

별도 남은7건은 이번 소유권 수정으로 모두 해결됐다고 주장하지 않는다. 일부 시점에서 새 raw scan stamp가 마지막 독립 odom TF보다28~41ms 앞선 timing witness가 있다. 기존 `_prefix_sweep_clear`가 이런 TF 미확정으로 인증을 폐기했는지는 해당 내부 예외/전체 raw scan이 미녹화되어 확정할 수 없다. TF 거부 정책은 변경하지 않고 `start_prefix_validation`과 `start_certificate_state/reason` 진단만 추가해 다음 실제 시험에서 판별한다. 실제충돌·TF미확정·authority/context 변경을 서로 구분한다.

## 재현

저장소 루트에서 ROS Python 메시지 환경만 읽는다. 노드·DDS·가제보·동작 명령은 생성하지 않는다.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 benchmarks/driving_parameter_tuning_20261104/planner_start_cancel_audit_20261105/replay.py
```

`geometry_verification.json`이 두 실제 raster frame의 독립 body 간격과 선택 간선 증명을 기록한다. `cancellation_geometry_report.json`은20건의 현재 경로 교차·진단·fresh TF timing witness를 구분한다. 연결 회귀는 `test_dynamic_escape_certificate_ownership.py`에서 두 frame의 발행→인증 유지뿐 아니라 실제 충돌·기억·stale·authority·unsafe 첫/이후 간선·재진입·인접 대각을 확인한다. raw scan/TF callback 회귀에서는 신선한 합성 센서 증거를 사용했으며 기록된 원래 raw beam 재생으로 표현하지 않는다.


## 후속 확인: 원본 scan·visibility pair (2026-11-05)

위 감사 이후, 첫 간선 소유권만 보완한 `b5f2950f…` 실제 시험에서도 **FAIL**이 재현됐다. 후속 [독립 진단](../physical_transfer_20261105/independent_prefix_owned_tf_revocation_audit.json)은 ACTIVE→REVOKED 12회 중9회에서 **최신 raw 획득 시각이 TF보다34–121ms 앞선 실제 예외**를 확인했다. 임무·HOME은 완료했으나 live 우회와 actor4회 이동을 증명하지 못했으므로 이전 실패 판정은 그대로 보존한다. 위 본문의 TF 원인 ‘미확정’은 더 이전 시험 당시의 증거 한계이며 이 후속 진단과 구분한다.

동결 소스 `ee656f4e5eae08003e2db253bdd5e67cb1cf697fb64005bd8e9fd4ed4be300a6`은 raw8개·visibility8개의 **동일 원본 stamp pair** 중 획득 TF가 준비된 가장 최신 것을 선택한다. 원본 source 및 양쪽 최초 수신 나이는 각각250ms 미만이며, 중복 메시지는 이 시각을 갱신하지 않는다. 신규/기존 첫 간선에 같은 규칙을 적용하고, 발행 전에도 원본 raw sweep을 증명한다. 현재 관측·새 callback hit·기억·정적/unknown 차체 충돌 검사는 먼저 유지한다. 더 최신 미투영 raw를 free라고 선언하지 않으며 최종 mux raw guard는 그 새 근접점에서 계속 정지시킨다. pair가 없으면 물리 장애물 실패 대신 SENSOR_HOLD로 정지하고 목표·기존 LOCAL을 보존해 입력 회복 후 재개한다. restamp·Time(0) 대체·TF 외삽·시간 한도 확대는 사용하지 않는다.

집중 회귀155개가 통과했고 최종 epoch bookkeeping 정리 뒤 관련5개를 재확인했다(서로 중복이므로160개 고유 검사로 합산하지 않음). 실제 tf2 Buffer 미래 예외와 독립 planner→mux 신규 raw 정지 검사를 포함하지만, 기록 raster와 합성 callback을 결합한 테스트를 원 시험 전체 raw beam 재생으로 표현하지 않는다. 새 `final_common_coherent` 실제 Gazebo 시험은 이 추가 기록 작성 시 **RUNNING/PENDING**이며 여기서 PASS로 선언하지 않는다. 입력·소스·테스트 해시와 안전 경계는 [coherent_scan_fix_review.json](coherent_scan_fix_review.json)에 기록했다.
