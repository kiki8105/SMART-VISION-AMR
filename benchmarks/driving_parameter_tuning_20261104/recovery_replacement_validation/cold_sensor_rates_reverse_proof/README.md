# Actual cold-start / source-rate trial: one verified reverse

This archive preserves a successful **reverse maneuver within a strict FAIL
trial**, not an overall obstacle-test success or physical-robot validation.

Source trial: `/tmp/classroom-transfer-cold-rates-obstacle-20261105/25_web_inventory_sector3_moving_obstacle`.
The actual backend mission reached Sector 3 and Home. The actor performed all
four moves, but the robot did not bypass it while live. One stopping-rotation
QA window measured 0.1243 rad against the unchanged 0.12 rad bound. Both failures
remain failures.

`fixture.json` contains original telemetry rows around the reverse, the original
stop-QA window, actor positions, AMCL cold-start evidence, runtime configuration
and source hashes. No odometry, pose, sensor sample or terminal was generated.
Run the portable standard-Python replay from any directory:

```bash
python3 replay.py
```

| Evidence | Result |
|---|---:|
| Owning recovery episode | `d4407c00d51e45ef8a1076b24c7ec306` |
| Actual BackUp status | `RECOVERY_SUCCEEDED` |
| Requested reverse | 0.10505 m at 0.1 m/s |
| Negative `/cmd_vel` samples | 36 |
| Start-to-terminal sampled truth displacement | 0.11814 m |
| Previous 300 ms output samples | all zero |
| Conservative sampled box-sweep gap | 0.05509 m |
| Cold start before manual API | prior disabled, AMCL/map TF absent, ESTOP stopped |
| Manual initialization afterward | actual AMCL/map TF appeared; backend `VERIFIED` |
| Whole trial | **FAIL**: live bypass absent and stopping-rotation QA exceeded |

The gap calculation covers the AABB swept between successive logged actor
positions, retaining the full 0.22 m body radius. It is a sampled ground-truth
bound, not a continuous collision proof. `results.json` records the maximum
truth-sample gap and full derivation. The historical 10.81 cm observation used
only t30.3862–31.7883; the complete owned start-to-terminal window gives 11.81 cm.

The stopping violation was ordinary FollowPath curve motion at approximately
0.093 m/s and 0.474 rad/s, not a legitimate Spin action incorrectly counted as
a hold. The first zero navigation command followed the empty path by about
54 ms, and zero mux output followed the planner wait by 112 ms. Actual body
rotation continued during braking. This archive does not waive or repair that
failure; a subsequent controller change requires its own actual trial.
