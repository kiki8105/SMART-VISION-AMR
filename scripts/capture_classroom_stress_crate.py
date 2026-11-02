#!/usr/bin/env python3
"""Read-only witness for the fixed, unmapped center crate in the stress world.

Run beside the web-owned mission in isolated domain 176. The trace contains
Gazebo model truth, raw laser returns matching the crate geometry, the
LiDAR-derived observed cells, goals, and actual planner paths. This process
creates no publisher, service client, or control action.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import time


CRATE_NAME = 'stress_unmapped_crate_center'
CRATE_CENTER = (.35, -.60)
CRATE_HALF_SIZE = (.10, .10)
LASER_FORWARD_OFFSET_M = -.064
MAX_TRUTH_AGE_S = .30


def yaw(quaternion):
    return math.atan2(
        2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
        1-2*(quaternion.y*quaternion.y+quaternion.z*quaternion.z))


def ray_rectangle_distance(origin, direction, center=CRATE_CENTER,
                           half_size=CRATE_HALF_SIZE):
    """Distance from a 2-D ray to the first axis-aligned box boundary."""
    near, far = -math.inf, math.inf
    for position, velocity, middle, half in zip(
            origin, direction, center, half_size):
        low, high = middle-half, middle+half
        if abs(velocity) < 1.e-12:
            if position < low or position > high:
                return None
            continue
        first, second = (low-position)/velocity, (high-position)/velocity
        near = max(near, min(first, second))
        far = min(far, max(first, second))
        if far < near:
            return None
    if far < 0:
        return None
    return max(0., near)


def scan_crate_matches(scan, robot, *, center=CRATE_CENTER,
                       half_size=CRATE_HALF_SIZE):
    """Compare *raw* scan beams with ray/box ranges from Gazebo robot truth."""
    x, y, heading = robot
    origin = (x+LASER_FORWARD_OFFSET_M*math.cos(heading),
              y+LASER_FORWARD_OFFSET_M*math.sin(heading))
    expected = []
    hits = []
    for index, reading in enumerate(scan.ranges):
        angle = heading+scan.angle_min+index*scan.angle_increment
        distance = ray_rectangle_distance(
            origin, (math.cos(angle), math.sin(angle)), center, half_size)
        if distance is None or not scan.range_min <= distance <= min(scan.range_max, 2.5):
            continue
        expected.append((index, distance))
        if (math.isfinite(reading) and scan.range_min <= reading <= scan.range_max
                and abs(reading-distance) <= .07):
            hits.append((index, reading, distance))
    return {'expected_beams': len(expected), 'matching_beams': len(hits),
            'nearest_expected_range_m': (round(min(distance for _, distance in expected), 4)
                                         if expected else None),
            'max_matching_residual_m': (round(max(abs(reading-distance)
                                              for _, reading, distance in hits), 4)
                                        if hits else None),
            'matching_indices': [index for index, _, _ in hits[:24]]}


def near_observed_indices(indices, map_info, *, center=CRATE_CENTER,
                          radius=.25):
    if map_info is None:
        return []
    width, height, resolution, ox, oy, map_yaw = map_info
    c, s = math.cos(map_yaw), math.sin(map_yaw)
    result = []
    for index in set(indices):
        if not 0 <= index < width*height:
            continue
        mx = ((index % width)+.5)*resolution
        my = ((index // width)+.5)*resolution
        x, y = ox+c*mx-s*my, oy+s*mx+c*my
        if math.hypot(x-center[0], y-center[1]) <= radius:
            result.append(index)
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '176' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Only ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 is supported')
    if args.output.exists():
        parser.error('Refusing to overwrite '+str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from gazebo_msgs.msg import LinkStates, ModelStates
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                          qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32MultiArray, String

    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(args=[])

    class Observer(Node):
        def __init__(self, stream):
            super().__init__('classroom_stress_crate_witness')
            self.stream = stream
            self.truth_received = None
            self.robot = None
            self.crate = None
            self.crate_link = None
            self.crate_link_received = None
            self.map_info = None
            self.last_model_emit = -math.inf
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
            self.create_subscription(ModelStates, '/gazebo/model_states',
                                     self.on_models, qos_profile_sensor_data)
            self.create_subscription(LinkStates, '/gazebo/link_states',
                                     self.on_links, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.on_scan,
                                     qos_profile_sensor_data)
            self.create_subscription(Int32MultiArray,
                                     '/smart_vision_amr/observed_obstacles',
                                     self.on_observed, 10)
            self.create_subscription(NavigationPath, '/smart_vision_amr/path',
                                     self.on_path, 10)
            self.create_subscription(PoseStamped, '/smart_vision_amr/goal',
                                     self.on_goal, 10)
            self.create_subscription(String, '/smart_vision_amr/failure_memory_status',
                                     self.on_failure_memory, durable)

        def emit(self, kind, **details):
            record = {'kind': kind, 'at': round(time.time(), 4), **details}
            self.stream.write(json.dumps(record, separators=(',', ':'))+'\n')
            self.stream.flush()

        def on_map(self, message):
            info = message.info
            self.map_info = (info.width, info.height, info.resolution,
                             info.origin.position.x, info.origin.position.y,
                             yaw(info.origin.orientation))
            self.emit('map', width=info.width, height=info.height,
                      resolution=info.resolution,
                      origin=[info.origin.position.x, info.origin.position.y,
                              self.map_info[-1]])

        def on_models(self, message):
            now = time.monotonic()
            if now-self.last_model_emit < .045:
                return
            self.last_model_emit = now
            if 'waffle_pi' not in message.name:
                return
            pose = message.pose[message.name.index('waffle_pi')]
            self.robot = (pose.position.x, pose.position.y,
                          yaw(pose.orientation))
            self.truth_received = now
            self.crate = None
            # In this SDF the model origin is (0,0,0). The box collision is
            # positioned on its child link at the configured CRATE_CENTER, so ModelStates
            # alone would falsely report an object at the world origin.
            model_present = CRATE_NAME in message.name
            link_age = (now-self.crate_link_received
                        if self.crate_link_received is not None else math.inf)
            if model_present and self.crate_link is not None and link_age <= .30:
                self.crate = self.crate_link
            self.emit('truth', robot=[round(value, 5) for value in self.robot],
                      crate=([round(value, 5) for value in self.crate]
                             if self.crate else None),
                      crate_model_present=model_present,
                      crate_link_age_s=(round(link_age, 4)
                                        if math.isfinite(link_age) else None))

        def on_links(self, message):
            name = CRATE_NAME+'::body'
            if name not in message.name:
                self.crate_link = None
                self.crate_link_received = None
                return
            pose = message.pose[message.name.index(name)]
            self.crate_link = (pose.position.x, pose.position.y,
                               yaw(pose.orientation))
            self.crate_link_received = time.monotonic()

        def on_scan(self, message):
            age = (time.monotonic()-self.truth_received
                   if self.truth_received is not None else math.inf)
            if self.robot is None or self.crate is None or age > MAX_TRUTH_AGE_S:
                return
            # Reject an inconsistent model pose instead of attributing some
            # other object at the expected coordinate to the fixed crate.
            if math.hypot(self.crate[0]-CRATE_CENTER[0],
                          self.crate[1]-CRATE_CENTER[1]) > .005 or abs(
                              self.crate[2]) > .02:
                return
            matches = scan_crate_matches(message, self.robot)
            self.emit('scan_crate', truth_age_s=round(age, 4),
                      stamp_ns=(message.header.stamp.sec*1_000_000_000+
                                message.header.stamp.nanosec),
                      **matches)

        def on_observed(self, message):
            # Store every source cell as well as the subset attributed to the
            # crate. The full set lets postmortems distinguish a crate-blocked
            # corridor from unrelated wall-edge returns. Typical messages
            # contain only a few dozen indices, so this stays compact.
            all_indices = sorted(set(int(index) for index in message.data))
            nearby = near_observed_indices(all_indices, self.map_info)
            self.emit('observed', nearby_indices=nearby,
                      observed_indices=all_indices,
                      nearby_count=len(nearby), total_count=len(message.data),
                      map_ready=self.map_info is not None)

        def on_goal(self, message):
            self.emit('goal', x=round(message.pose.position.x, 5),
                      y=round(message.pose.position.y, 5),
                      stamp_ns=(message.header.stamp.sec*1_000_000_000+
                                message.header.stamp.nanosec))

        def on_failure_memory(self, message):
            try:
                state = json.loads(message.data)
            except (ValueError, TypeError):
                self.emit('failure_memory', malformed=True)
                return
            if isinstance(state, dict):
                self.emit('failure_memory', **state)

        def on_path(self, message):
            self.emit('path', count=len(message.poses),
                      points=[[round(p.pose.position.x, 5),
                               round(p.pose.position.y, 5)]
                              for p in message.poses[:1000]],
                      stamp_ns=(message.header.stamp.sec*1_000_000_000+
                                message.header.stamp.nanosec))

    with args.output.open('w') as stream:
        node = Observer(stream)
        node.emit('meta', domain=176, localhost_only=True,
                  crate_name=CRATE_NAME, crate_center=list(CRATE_CENTER),
                  crate_size=[2*x for x in CRATE_HALF_SIZE],
                  laser_forward_offset_m=LASER_FORWARD_OFFSET_M)
        try:
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
