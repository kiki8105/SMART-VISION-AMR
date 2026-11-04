# Stage 17 반경 판정 정정

원본 `17_web_hardware_value_parity`는 **FAIL 그대로 보존**했습니다. 실차 파라미터 복사·
1→2→3→Home 도착·정지는 통과했으나, 별도 고정 상자 판정기가 실제 22cm Gazebo 차체를
24cm로 계산해 전체 판정을 실패시켰습니다. 복사된 SDF의 중심 원통과 runtime manifest가
모두 22cm임을 대조한 뒤 **동일한 주행 기록만 오프라인 재평가**했습니다.

| 판정 | 사용 반경 | 보수적 최소 여유 | 필수 여유 | 결과 |
|---|---:|---:|---:|---|
| 원본 판정기 | 24cm | 0.6cm | 2cm | FAIL, 원본 유지 |
| 실제 SDF 반경 재평가 | 22cm | 2.6cm | 2cm | REASSESSED_ACTUAL_RADIUS / PASS |

새 판정기는 `--robot-radius`를 받습니다. staged runner는 manifest의 두 반경 선언과
실제 복사 SDF 원통의 반경·중심이 일치할 때만 이 값을 전달합니다. 불일치·누락은 실패하며,
기존 stock Gazebo 기본값 24cm와 QA 여유 2cm·라이다/관측 셀·전 구간 truth·Home 검사는 유지합니다.

`stage17_reassessed_actual_radius_20261105.json`에 원본/재평가 판정, SDF 기하, 입력 해시를
보존했습니다. 새 주행·반복 성공·실차 성공을 의미하지 않습니다. 특히 이번 기록의 fault 범위는
scan이며 실제 Wi-Fi/DDS·PC/Pi 시계 차이·OpenCR 전원/serial·바퀴 슬립/IMU bias는 검증하지 않았습니다.

누적 탐색 횟수도 목표별 최대값 합산 오류를 별도로 정정했습니다.
`final_policy_corrected_counter_summary_20261105.json`의 첫 clean / 반복 clean / 재고 장애물 시험은
전체 탐색 **4 / 4 / 5**, 국소 시도 **0 / 0 / 3**입니다. 과거 JSON을 덮어쓰지 않았습니다.
