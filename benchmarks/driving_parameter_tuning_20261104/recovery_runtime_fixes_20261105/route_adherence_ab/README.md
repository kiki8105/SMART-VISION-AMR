# Frozen controller comparison

The original failed permuted run remains unchanged. These two loopback-only domain 186 Nav2 evaluations use the same recorded 82-point path, scan, odometry, transforms and captured local costmap at ROS 62.68s. Only `FollowPath.route_adherence_scale` differs (0 versus 32); both use the new conservative dynamics. The costmap and path receipt times differ by 180ms, and the captured costmap predates the freeze by 500ms; see manifest. No historical critic state was recorded. This cannot prove the historical winning command or closed-loop success.

The selected command changes from v=0.150526 m/s, w=+0.064103 rad/s to v=0.115789 m/s, w=+0.217949 rad/s. The actual critic scores/scales and every candidate are preserved in each compressed summary. PathDist/PathAlign scale is 0.8 per cell at 5cm, GoalDist/Align 0.6, and the added metric cost scale is 32 per metre. Whole-body collision rejection remains enabled.

Reproduce after building and sourcing ROS Humble plus the Team2 install:

```bash
python3 run_replay.py --output /tmp/fresh-route-adherence-ab
```

This does not launch Gazebo or connect to a robot. Output is remapped to `/replay/controller_cmd_vel`.
