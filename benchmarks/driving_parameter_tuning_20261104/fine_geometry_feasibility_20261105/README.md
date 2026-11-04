# 1cm 기하 세분화: 미채택 오프라인 검토

**런타임 수정·실차·Gazebo 시험이 아닌 동결 snapshot의 계획 가능성 조사이다.** 원래 5cm occupied/unknown square와 관측11셀·기억23셀을 삭제하거나 축소하지 않고, 원형 차체 반경22cm를 유지했다. 원본 입력과 map 이미지 해시는 각 JSON에 기록했다.

| 조사 | 결과 | 전체 경로 여유 | 계산 범위 |
|---|---|---:|---|
| A: 5cm 후보 중심 + 실제 edge sweep | exact goal까지66노드·3.586m | 5mm | 검색+최종검증230ms; 공유 전처리 제외 |
| B: 1cm 후보 중심 + 동일 원본 사각형 | exact goal까지300노드·3.342m | **24.826mm** | 검색+최종검증1484ms/11538확장; 공유 전처리 제외 |
| 1cm 짧은 connector 단일 bounded 시도 | **TIME_BUDGET, 미채택** | 새 connector 미확정 | 입력26.84ms + tail17.75ms + field/8방향 edge 준비24.09ms =68.71ms; 검색 시작 전60ms 초과 |

B의 최초 결과는 [full_goal_reference_original.json](full_goal_reference_original.json)에 원본 그대로 보존했다. 당시1.484초는 거리장·원본 사각형 구성 이전 전처리를 **포함하지 않는다**. 노드·edge 증명에는5mm 거리 격자와 Lipschitz 샘플 간 reserve를 쓰며, 마지막에는 모든 실제 segment를 원본 사각형에 대한 별도 정확 거리로 검사했다. 현재 pose→첫 후보 및 exact goal connector도 포함한다. 다만 원 prototype의 시작 첫점에 작은 후진 접힘이 있어 production 전달 전에는 같은 정확한 검사로 canonicalization해야 한다.

마지막 단일 bounded 시도는 기존 owned104점 경로의 현재 진행보다0.4406m 앞인 anchor index36 `(0.59,−1.17)`를 선택했다. 원래 tail 전체와 exact goal connector의 최소 차체 여유는10.5cm로 검증했다. 그러나 거리를 계산할 local field와8방향 edge를 준비할 때 전체60ms를 이미 소진하여 **검색0확장으로 중단**했다. 새 connector가 실제로 시간 안에 나왔다고 주장하지 않는다. 기존1500확장 제한도 그대로다. 검색이 빠르더라도 입력·전처리·재검증·canonicalization 비용을 별도 시간으로 더 받지 않는다.

이 측정은 오프라인 JSON/지도 파일 decode까지60ms에 포함한 보수적인 단일 시도이다. 실제 노드는 이미 지도를 메모리에 보유하므로 이 결과만으로 모든 가능한 최적화가60ms 내 불가능하다고 단정하지 않는다. 반대로 캐시된 전처리·입력 개정 검사·출력 재검증을 빼고 런타임 예산을 통과했다고 주장해서도 안 된다. 추가 최적화·worker·native 구현은 이번 범위에서 하지 않았으며 기존 런타임 `ee656f4e…`를 유지한다.

## 재현

Python3, NumPy, **SciPy**, Pillow, PyYAML이 필요하다. ROS import/init, DDS, Gazebo 또는 주행 명령은 없다. 저장소 복제 후 임의 디렉터리에서 실행할 수 있다. 출력 JSON은 기존 증거를 덮지 않도록 `/tmp` 등에 지정한다. 시간은 기계 부하와 라이브러리에 따라 달라진다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  benchmarks/driving_parameter_tuning_20261104/fine_geometry_feasibility_20261105/connector_screen.py \
  --output /tmp/fine-connector-recheck.json

# 별도의 느린 전체 goal 참고 재생: 실행 중인 Gazebo와 함께 돌리지 않는다.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  benchmarks/driving_parameter_tuning_20261104/fine_geometry_feasibility_20261105/full_goal_reference_replay.py \
  --output /tmp/fine-full-goal-reference.json
```

현재 증거는 unrotated classroom saved-map에 한정한다. production 공통 기하에는 rotated origin, out-of-map 거부, 실제 시작·goal 연결, smoothed segment, 입력 revision 및 fresh pose/raw pair 검사가 모두 필요하다. 원래 whole-candidate-cell invalidation을 그대로 두면 fine 경로도 다시 취소되므로 planner·곡선화·실행 검증이 같은 world-body sweep 기하를 써야 한다. 이는 새 복구 상태/예산 예외를 더하는 작업이 아니다.24.8mm snapshot 여유는 약5.4cm 추정↔truth 오차에 대한 일반적 강건성이나 실제 폐루프 성공을 보장하지 않는다.

기존 출력 경로는 거부합니다. 출력 보호만 보완하기 전 도구 원문은 `*_captured_source.txt`로 보존했으며, 원본 측정 JSON의 도구 해시는 이 원문을 기준으로 합니다. 이 보완 뒤 계산 시간을 다시 측정하지 않았습니다.
