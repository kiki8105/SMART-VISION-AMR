# Mux callback delivery correction

The original `runtime_common_v1` failure remains unchanged: the robot repeatedly received zero because the mux had stale internal wheel feedback while an independent observer had fresh odometry. See [the original audit](../mux_starvation_pre_fix_audit.json).

The correction separates latest-only velocity ingress from geometry, serializes ordinary application callbacks without occupying all executor workers on the same lock, and removes duplicate full guard evaluations. Each output retains original source/first-receipt freshness, the full signed braking tube and raw-point checks. A final 50 ms compute bound is included in pose/pending/velocity uncertainty and checked on both clocks; it is not a promise that a non-real-time OS always publishes every 50 ms.

Urgent zero publication cannot be followed by an older computed nonzero command. The original core authority, timestamp, sequence, reset/stop-ACK and ownership checks still decide control admission. A busy explicit AUTO transition is stored without retiming; redundant heartbeats coalesce, and newer accepted stop/manual/AUTO authority wins.

[Results and source hashes](results.json) include real loopback DDS/executor tests with 360/720-beam clouds. Both stationary-feedback near-obstacle brake-cap searches and nonzero `.22 m/s, .35 rad/s` feedback with a far-clear cloud were measured. The synthetic producer has stationary TF; these tests establish callback delivery/computation, not Gazebo or physical driving. The original failed and intermediate tests were not relabelled as vehicle success.

Final commands (repository root):

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/ros2_ws/src/smart_vision_amr:$PWD/scripts:$PYTHONPATH" \
/usr/bin/python3 -m pytest -q \
  ros2_ws/src/smart_vision_amr/test/test_motion_mux_delivery.py \
  ros2_ws/src/smart_vision_amr/test/test_braking_envelope.py \
  ros2_ws/src/smart_vision_amr/test/test_near_scan_outlier_guard.py \
  ros2_ws/src/smart_vision_amr/test/test_motion_mux.py \
  ros2_ws/src/smart_vision_amr/test/test_motion_mux_concurrency.py \
  ros2_ws/src/smart_vision_amr/test/test_motion_safety_services.py
```

The six-file focused run passed 135 tests. After a test-only cancelled-timer teardown cleanup, the exact final delivery suite passed all 23 tests without that warning. Broad suite and vehicle-simulation results are recorded separately by the root runner.
