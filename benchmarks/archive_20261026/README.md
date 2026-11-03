# 2026-10-26 Gazebo 시험 원본 중 선별 보존본

이 자료는 과거 `~/Team2_workspace/log`에만 있던 지도·실험 결과 중 문서에서
인용하는 근거입니다. 새로 클론한 PC에서도 다음 파일을 확인할 수 있습니다.

- `curve_tuning/all_trials.json`: 속도·팽창·경로 pruning 비교 결과.
- `curve_tuning/E_supplement.json`, `E_comparison.png`, `final_manifest.json`:
  성공한 시험 E의 보충 결과와 출처.
- `slam_validation/complete/map.yaml`, `map.pgm`, `map.json`:
  Gazebo 미로 SLAM 최종 지도와 지도 검증 기록.
- `slam_validation/complete/navigation.json`, `controller_detour.png`,
  `slam_validation/controller_runtime.{json,png}`: 저장 지도 주행·조종기 검증.
- `verification/`: 직선·우회·취소·저장 지도 주행의 원본 실행 로그.

이 기록 안의 `/home/kiki/Team2_workspace/...` 경로는 **당시 PC의 역사적 경로**입니다.
새 PC에서 사용하려면 이 폴더의 실제 경로로 바꾸세요. 저장 지도를 Gazebo에서
열 때는 저장소 루트에서 실행합니다.

```bash
./scripts/run_sim.sh map:="$PWD/benchmarks/archive_20261026/slam_validation/complete/map.yaml"
```

최신 반복 시험 기록은 상위 [`benchmarks`](../)와
[`단계별 검증`](../../docs/classroom_staged_validation.md)에 있습니다.
