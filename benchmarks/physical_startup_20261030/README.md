# 2026-10-30 real robot startup test

**Result: BLOCKED by real drive-controller communication failure. No waypoint
delivery or translation mission was completed.** See the
[Korean handoff](../../docs/PHYSICAL_TEST_HANDOFF_2026-10-30.md).

- [Baseline](baseline.json): stale control receipt from the same Pi. No rotation.
- [Fast DDS asynchronous comparison](async_result.json): stale control persisted;
  initialization RPC did not complete. This configuration was not adopted.
- [Cyclone trial](cyclone_result.json): map, service, guarded reset and actual
  localization rotation progressed; OpenCR/Dynamixel communication failed and
  `turtlebot3_ros` aborted with exit -6 after about 188.8 degrees of rotation.
  AMCL had not converged. Cyclone remains an optional comparison, not a claim
  that communication or navigation passed.
- [Required-process shutdown test](failfast_process_validation.json): a real
  local fake driver exit shut down its companion process in 0.34 seconds.
  No ROS participant or physical hardware was used in this test.
- [Robot file deployment](deployment.json): source hashes and original backups.

The owner confirmed physical standstill. Test-owned robot runtime and PC
navigation processes were stopped; only the web diagnostic server was retained.
Robot source backups are at `~/smart_vision_backups/20261030_handoff/` on the Pi.
The hardware was not restarted after deploying required-process shutdown.

Focused verification: **264 tests passed**, with localhost-only ROS test
participants on separate domains. The selected `smart_vision_amr` package build also passed (2.42 seconds). Firmware, vendor drivers, safety deadlines,
localization confidence thresholds and production RMW defaults were unchanged.
