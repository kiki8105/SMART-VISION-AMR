# 이전 SLAM 지도 아카이브

`~/Team2_workspace`를 지우고 다시 클론해도 이전 지도 파일을 확인할 수 있도록
GitHub에 보존한 자료입니다. **현재 실차 운용 지도는 이 아카이브가 아니라**
[`classroom_20261030_cleaned.yaml`](../ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml)입니다.

| 원래 작업공간 경로 | 클론 후 경로 | 용도 |
|---|---|---|
| `maps/classroom_remap_20261030/{map.yaml,map.pgm}` | `archive/slam_maps/classroom_remap_20261030/` | 10/30 교실 재매핑 중간본; 현재 운용 좌표계와 다를 수 있음 |
| `maps/slam_20261026_123419_49443/{map.yaml,map.pgm}` | `archive/slam_maps/slam_20261026_123419_49443/` | 10/26 Gazebo SLAM 지도 |
| `maps/slam_20261026_135720_61340/{map.yaml,map.pgm}` | `archive/slam_maps/slam_20261026_135720_61340/` | 10/26 Gazebo SLAM 지도 |
| `maps/maze_aruco_20261026/{posegraph.posegraph,posegraph.data,map_qa.json,astar_qa.json}` | `archive/slam_maps/maze_aruco_20261026/` | Gazebo 미로 SLAM 원본 posegraph와 지도·경로 QA; **실물 교실 posegraph는 아님** |

원래 작업공간에 있던 `classroom_before_remap_20261030_161933`은 저장소의
[`classroom_20261027_cleaned`](../ros2_ws/src/smart_vision_amr/maps/classroom_20261027_cleaned.yaml)와 동일하고,
`classroom_remap_20261030_1630`은 저장소의
[`classroom_20261030_raw`](../ros2_ws/src/smart_vision_amr/maps/classroom_20261030_raw.yaml)와
동일한 지도 데이터입니다. `maze_aruco_20261026` 점유 지도도 저장소의
[`maze_aruco_20261026`](../ros2_ws/src/smart_vision_amr/maps/maze_aruco_20261026.yaml)에
있습니다. 이미지 이름만 새 위치에 맞춰져 있습니다.

아카이브의 `map.yaml`은 같은 폴더의 `map.pgm`을 상대 경로로 가리킵니다.
QA JSON 안의 `/home/kiki/Team2_workspace/...` 경로는 당시 실행 PC의 기록이며,
새 PC에서 그대로 실행할 경로가 아닙니다.
과거 좌표계 지도를 현재 실차 주행에 바로 적용하지 마세요. 파일 무결성은
저장소 루트에서 다음과 같이 확인합니다.

```bash
sha256sum -c archive/slam_maps/SHA256SUMS
```

시험 요약과 일부 원본 기록은 [`benchmarks`](../benchmarks/) 및
[`2026-10-26 시험 아카이브`](../benchmarks/archive_20261026/README.md)에 있습니다.
개인 PC의 모든 임시 빌드 로그·배포 산출물까지 GitHub에 보존한 것은 아닙니다.
