# 통합 시험 Home 복귀 중단 분석

실제 시험 `/tmp/classroom-material-heading-validation-20261105/12_web_integrated_clean`은 Home 복귀 대기 상태에서 원인 분석을 위해 중단했습니다. **전체 실패·Home 미완료**입니다. 섹터별 후방 확인 이미지가 있어도 중단으로 누락된 최종 검증을 합격으로 바꾸지 않습니다. 분석 시 해당 시뮬레이터·관제·주행 프로세스가 정리된 상태도 확인했습니다.

| 기록과 같은 입력 | 오프라인 재현 |
|---|---:|
| 최신 관측 장애물 / 확인된 기억 | 0 / 0셀 |
| 실제 controller_aborted 기억의 방향 제약 | hard276개·soft비용740개 |
| 위 제약을 그대로 적용 | 경로 없음 |
| 같은 제약을 기존 bounded비용으로만 적용 | 경로33셀 |
| 실행 실패 기억만 없는 반사실 | 원래 지도 경로27셀 |

지도·unknown7508셀·A*5셀 팽창을 모두 그대로 보존했습니다. 물리 장애물 셀이나 차체 반경·충돌 여유를 줄인 실험이 아닙니다. 제약 입력은 flight recorder가 기록한 실제 실패 위치 `(0.732787593, -1.334261043)`, 북향 접근, `controller_aborted`, 같은 지도·Home 목표 범위입니다.

실제 기록에서 해당 실패는 먼저 새 라이다로 장애물이 아님을 확인해 `soft_replan`으로 분류됐습니다. 이후 한 셀의 관측 장애물로 별도 물리 복구 에피소드가 생겼고, 수정 전 `_controller_failure_policy`는 에피소드가 FOLLOW가 아니면 즉시 False를 반환했습니다. 이 때문에 이미 분류한 실행 실패276개까지 다시 hard 제약이 됐습니다. 관측·기억이0이 된 뒤에도 동일 hard 제약으로 NoPath가 남았습니다. `controller_failure_policy_before.txt`와 `fixture.json`의 실제 분류·실패 이벤트에 이 연결을 보존했습니다.

다른 PC에서도 ROS나 원래 `/tmp` 파일 없이 재현할 수 있습니다.

```bash
python3 benchmarks/driving_parameter_tuning_20261104/recovery_replacement_validation/clean_home_failure_audit/replay_failure_edges.py
```

`snapshot/`에는 시험 당시와 해시가 일치하는 순수 Python helper를 보존했습니다. 원본 PGM·입력9개 해시·실패 기억·분류 성공/실패 이벤트·관측0/기억0 스냅샷은 `fixture.json`, 재현 결과는 `oracle_report.json`에 있습니다. 원본 기록이 없는 별도 폴더에 복사한 뒤에도 같은276/740/0/33/27 결과를 확인했습니다.

이 분석은 최초 DWB abort의 충돌 원인이나 실제 후진 성공을 설명하지 않습니다. 오프라인 경로가 존재한다는 사실도 raw 라이다·국소 costmap·실제 주행의 충돌 검증을 대신하지 않습니다. 수정 및 재시험 결과는 별도로 판단해야 합니다.
