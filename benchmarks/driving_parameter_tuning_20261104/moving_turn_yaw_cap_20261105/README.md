# Translating-turn yaw envelope — frozen Nav2 comparison

**This is an open-loop controller check, not a closed-loop, stop-QA or physical
robot PASS.** The earlier cold/source-rate actual trial remains strict FAIL.
Its ordinary moving turn selected approximately `(v=.09263,w=.47436)` and
rotated 0.1243 rad during a wait/stop against the unchanged 0.12 rad QA bound.

Previously the turn envelope limited only linear speed. A translating command
with yaw above .35 rad/s remained valid when its linear speed was below .10 m/s.
The new `FollowPath.moving_turn_max_angular=.35` rejects such translating
candidates **before critic scoring** when `hypot(vx,vy)>.01 m/s`. It never clips
an already evaluated command to a different curvature. The configured .5 rad/s
DWB pure-pivot range, separate Nav2 Spin settings, body radius, costmap,
collision critics, acceleration and stop/cancel protocols remain unchanged.

The .35 value is explicit in both official Nav2 profiles and matches the
existing follower curve-preview yaw value. Nonfinite candidate values or an
invalid cap are rejected. The C++ suite passed all 12 tests, including the
recorded bad command, both signs, norm/threshold boundaries, pure pivots,
custom cap and nonfinite inputs.

The actual Nav2 replay uses the same finite inputs and profile for each pair.
`.50` reproduces the previous permitted finite yaw range; `.35` enables the new
moving envelope. All runs used domain 186, loopback, with output remapped to
`/replay/controller_cmd_vel`. No Gazebo or robot was started.

| Input | Cap | Last valid forward candidates | Selected v / w | Valid moving candidates above .35, all evaluations | Selected pure pivots |
|---|---:|---:|---|---:|---:|
| Recorded folded physical incident | .50 | 127 | .115789 / .089744 | 192 | 0 |
| Recorded folded physical incident | .35 | 103 | .115789 / .089744 | 0 | 0 |
| Synthetic corner | .50 | 271 | .092632 / .320513 | 384 | 0 |
| Synthetic corner | .35 | 223 | .092632 / .320513 | 0 | 0 |

Each run produced four nonempty legal evaluations. The new cap preserved
forward solutions and selected commands in these inputs; maximum valid moving
yaw became .346154 rad/s because of the existing sampling grid. Collision
critics still rejected candidates. Their counts decrease when the envelope
rejects a candidate earlier; this does not remove a collision check.

`recorded_runs/` preserves input snapshots, applied parameters, compressed
observations, controller logs and SHA-256 manifest. The costmap was reconstructed
from each identical frozen scan, not a captured costmap time history. These are
regression inputs, not an exact replay of the new cold trial's braking window.
Actual stop performance must be checked in a separate full-system repeat.

Reproduce after building and sourcing ROS Humble plus the project overlay:

```bash
python3 benchmarks/driving_parameter_tuning_20261104/moving_turn_yaw_cap_20261105/run_frozen_ab.py /tmp/moving-yaw-new-run
```

Use a fresh output path. The runner refuses to overwrite evidence.
