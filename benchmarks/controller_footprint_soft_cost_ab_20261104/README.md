# 차체 경계 비용 A/B — 2026-11-04

통합 시험의 Sector 1 → 2 이동이 느려진 지점에서 **같은 captured costmap·TF·경로·속도 제한**을 실제 Nav2 DWB에 입력했다. `ObstacleFootprint.scale`만 바꿨다. 차체 반경 20 cm, padding 0, 충돌 거부는 유지했다.

| 입력 | scale 0.5 선택 속도 | scale 0.02 선택 속도 | 충돌 후보 집합 |
|---|---|---|---|
| 실제 기록 ROS 69 s | 0.0232 m/s, -0.2436 rad/s | 0.1274 m/s, -0.1923 rad/s | 동일 |
| 실제 기록 ROS 71 s | 0.0116 m/s, -0.2436 rad/s | 0.1042 m/s, -0.3205 rad/s | 동일 |
| 근접 상자 합성 입력 | 유효 후보 452개 | 유효 후보 452개 | 차체 접촉 후보 2개를 양쪽 모두 거부 |

ROS 69 s의 전진 후보는 경계 비용 외 점수가 8.6점이었다. scale 0.5에서 경계 비용 6.275점이 더해져 14.875점이 되면서, 13.2점인 느린 후보에 밀렸다. scale 0.02에서는 경계 비용이 0.251점으로 줄어 같은 전진 후보가 선택됐다. **충돌 후보를 허용해서 빨라진 결과가 아니다.** 각 A/B의 거부 후보 속도·critic 목록 SHA-256도 동일하다.

세부 점수는 [results.json](results.json)에 있다.

## 재현

ROS Humble·Nav2가 설치되어 있고 이 저장소의 PC 런타임을 빌드한 상태에서 실행한다. 로봇과 Gazebo를 켤 필요는 없다. 출력 명령은 `/replay/controller_cmd_vel`, ROS 도메인은 186, 통신은 loopback으로 격리한다.

```bash
cd ~/Team2_workspace/src/SMART-VISION-AMR
source /opt/ros/humble/setup.bash
source ~/Team2_workspace/install/setup.bash

BENCH=benchmarks/controller_footprint_soft_cost_ab_20261104
OUT="/tmp/footprint-cost-ab-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

for SCALE in 0.5 0.02; do
  nice -n 19 python3 "$BENCH/replay_captured_costmap.py" \
    "$BENCH/input_69.jsonl.gz" --at-time 69 \
    --nav-params benchmarks/controller_replay_20261104/nav_profile_20cm.yaml \
    --override "FollowPath.ObstacleFootprint.scale=$SCALE" \
    --output "$OUT/scale-$SCALE" --duration 1 --domain 186
done
```

`input_71.jsonl.gz`는 `--at-time 71`, `near_box_input.jsonl.gz`는 `--at-time 1`로 같은 방식으로 실행한다. 각 출력 폴더의 `summary.json`에 선택 후보, 최선 전진 후보, critic별 점수와 거부 개수가 남는다. 서로 다른 controller replay를 도메인 186에서 동시에 실행하지 않는다.

이 기록은 반경 **20cm였던 당시**의 비교다. 현재 운용 반경은 22cm이며, 위 명령은
당시 설정을 보존한 `nav_profile_20cm.yaml`을 명시해 과거 결과와 구분한다.

## 재현 범위

- 원본: `incident_20261104T111553_718344Z_80e06852` 및 `12_web_integrated_clean/safety_timeline.jsonl`.
- 발행된 OccupancyGrid를 StaticLayer로 재생했다. lethal·unknown 셀 위치는 원본과 동일함을 확인했다. 0~100으로 발행된 중간 비용은 다시 양자화되므로 내부 costmap 바이트 전체의 완전 복제는 아니다.
- 경로 좌표는 관제 기록의 0.1 mm 단위 값이다. 당시 DWB가 잘라 보관한 점 개수(41개)를 진단 토픽에서 복원했다. 지역 목표도 원래 진단값과 약 0.1 mm 이내로 일치했다.
- 과거 Oscillation critic 상태는 저장되지 않아 새 컨트롤러에서 초기화된다. 실제 주행에서 관측한 모든 좌우 명령의 완전한 재현을 주장하지 않는다.
- 근접 상자 입력의 nominal 측면 간격은 15 mm다. 이는 차체 접촉 후보 거부의 동일성을 확인하는 입력이며, **2 cm 여유를 요구하는 폐루프 주행 시험의 합격 판정이 아니다.**
- 최종 채택은 별도 Gazebo 통합·장애물 시험으로 확인한다.
