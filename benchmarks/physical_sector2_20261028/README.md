# 2026-10-28 Sector 2 실차 재시도 진단 원본

기존 PC의 `~/Team2_workspace/diagnostics`에만 있던 선별 기록입니다.
`sector2_retry_bag/`은 약 179초 분량의 ROS 2 bag이며 `/scan`, `/odom`,
`/tf`, `/amcl_pose`, `/cmd_vel`, 계획·상태 및 배터리 토픽을 담습니다.
카메라 영상은 포함하지 않습니다. `opencr_soak_*.json`과
`pi_scan_receipt_monitor_*.json`은 하드웨어/센서 수신 점검 기록입니다.

새 PC에서 bag을 읽을 때는 ROS 2 Humble을 source한 뒤 아래처럼 실행합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 bag info benchmarks/physical_sector2_20261028/sector2_retry_bag
```

이 bag은 당시 실패 원인을 다시 분석하기 위한 자료이며, 현재 실차 주행의
성능이나 출발 성공을 보증하지 않습니다.
