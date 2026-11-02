#!/usr/bin/env python3
"""Record synchronized ROS receipts for a headless, isolated classroom run.

The planner label alone cannot prove that the vehicle stopped. This observer
records planner and follower states alongside wheel commands, LiDAR clearances,
path invalidation, and Gazebo ground truth. It never publishes commands.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import time


def finite_sector_min(scan, center, half_width):
    valid = []
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index*scan.angle_increment
        difference = math.atan2(math.sin(angle-center), math.cos(angle-center))
        if (abs(difference) <= half_width and math.isfinite(value)
                and scan.range_min <= value <= scan.range_max):
            valid.append((value, math.degrees(angle)))
    return min(valid) if valid else (None, None)


def scan_neighborhood(ranges, index, half_width=3):
    """Retain the original adjacent beam values, including invalid readings."""
    result = []
    for beam in range(max(0, index-half_width), min(len(ranges), index+half_width+1)):
        value = float(ranges[beam])
        result.append({'index': beam,
                       'range_m': value if math.isfinite(value) else str(value)})
    return result


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '176' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Only isolated ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 is supported')
    if args.output.exists():
        parser.error('Refusing to overwrite '+str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                          qos_profile_sensor_data)
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as NavigationPath
    from nav2_msgs.msg import Costmap, SpeedLimit
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    started = time.monotonic()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(args=[])

    class Observer(Node):
        def __init__(self, stream):
            super().__init__('classroom_safety_timeline')
            self.stream = stream
            self.last_truth_emit = -1.
            self.last_cmd_emit = -1.
            self.last_odom_emit = -1.
            self.last_nav_cmd_emit = -1.
            self.last_costmap_emit = -1.
            self.clock_stamp_ns = None
            self.last_input_samples = {}
            self.previous_scans = deque(maxlen=3)
            self.near_followups = []
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            raw_tf_qos = QoSProfile(depth=20,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(String, '/smart_vision_amr/planner_status',
                                     lambda msg: self.status('planner', msg), latched)
            self.create_subscription(String, '/smart_vision_amr/navigation_status',
                                     lambda msg: self.status('navigation', msg), 10)
            self.create_subscription(String, '/smart_vision_amr/recovery_request',
                                     self.recovery_request, 10)
            self.create_subscription(String, '/smart_vision_amr/localization_health',
                                     self.health, latched)
            self.create_subscription(String, '/smart_vision_amr/motion_status',
                                     self.motion, latched)
            self.create_subscription(NavigationPath, '/smart_vision_amr/path',
                                     self.path, 10)
            self.create_subscription(PoseStamped, '/smart_vision_amr/goal',
                                     self.goal, 10)
            self.create_subscription(Twist, '/cmd_vel', self.command, 10)
            self.create_subscription(Twist, '/smart_vision_amr/navigation_cmd_vel',
                                     self.navigation_command, 10)
            self.create_subscription(SpeedLimit, '/speed_limit', self.speed_limit, 10)
            self.create_subscription(LaserScan, '/scan', self.scan,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.odom,
                                     qos_profile_sensor_data)
            self.create_subscription(PolygonStamped,
                                     '/local_costmap/published_footprint',
                                     self.footprint, 10)
            self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                     self.costmap, 10)
            self.create_subscription(TFMessage, '/tf',
                                     lambda msg: self.tf(msg, 'forwarded'), 100)
            self.create_subscription(TFMessage, '/classroom_fault/raw_tf',
                                     lambda msg: self.tf(msg, 'raw'),
                                     raw_tf_qos)
            # Gazebo /clock can be BEST_EFFORT; a RELIABLE subscriber would
            # silently receive nothing, hiding simulated-time lag in traces.
            self.create_subscription(Clock, '/clock', self.clock,
                                     qos_profile_sensor_data)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.truth,
                                     qos_profile_sensor_data)

        def emit(self, kind, **values):
            monotonic = time.monotonic()
            self.stream.write(json.dumps({'t': round(monotonic-started, 4),
                                          'monotonic': round(monotonic, 4),
                                          'wall_time': round(time.time(), 4),
                                          'kind': kind, **values},
                                         separators=(',', ':'))+'\n')
            self.stream.flush()

        def status(self, kind, message):
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {'status': 'INVALID_JSON'}
            self.emit(kind, status=payload.get('status'),
                      goal=payload.get('goal'),
                      goal_stamp=payload.get('goal_stamp'),
                      input_hold=payload.get('input_hold'),
                      manual_stop=payload.get('manual_stop'),
                      route_failure=payload.get('route_failure'),
                      local_detour_state=payload.get('local_detour_state'),
                      message=payload.get('message'),
                      details={key: value for key, value in payload.items()
                               if key not in {'status', 'goal', 'goal_stamp',
                                              'input_hold', 'manual_stop',
                                              'route_failure', 'local_detour_state',
                                              'message'}})
            if kind == 'navigation' and (payload.get('input_hold') is True or
                    'Footprint disagrees' in str(payload.get('message', ''))):
                self.emit('input_fault_snapshot', trigger='navigation',
                          status=payload.get('status'),
                          message=payload.get('message'),
                          clock_stamp_ns=self.clock_stamp_ns,
                          samples=self.last_input_samples.copy())

        def recovery_request(self, message):
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {'raw': message.data}
            self.emit('recovery_request', payload=payload)

        def health(self, message):
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {}
            self.emit('health', level=payload.get('level'),
                      reasons=payload.get('reasons'),
                      autonomous_allowed=payload.get('autonomous_allowed'),
                      stop_latched=payload.get('stop_latched'))
            if payload.get('stop_latched') is True and not getattr(
                    self, 'last_health_latched', False):
                self.emit('input_fault_snapshot', trigger='health_latch',
                          reasons=payload.get('reasons'),
                          clock_stamp_ns=self.clock_stamp_ns,
                          samples=self.last_input_samples.copy())
            self.last_health_latched = payload.get('stop_latched') is True

        def motion(self, message):
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {}
            self.emit('motion', mode=payload.get('mode'),
                      autonomous_allowed=payload.get('autonomous_allowed'),
                      estop_latched=payload.get('estop_latched'),
                      output_linear=payload.get('output_linear'),
                      output_angular=payload.get('output_angular'))

        def path(self, message):
            self.emit('path', count=len(message.poses),
                      stamp_ns=message.header.stamp.sec*1_000_000_000+
                      message.header.stamp.nanosec,
                      frame_id=message.header.frame_id,
                      points=[[round(p.pose.position.x, 4),
                               round(p.pose.position.y, 4)]
                              for p in message.poses[:400]])

        def goal(self, message):
            self.emit('goal', x=round(message.pose.position.x, 5),
                      y=round(message.pose.position.y, 5),
                      yaw=round(yaw(message.pose.orientation), 5),
                      stamp_ns=message.header.stamp.sec*1_000_000_000+
                      message.header.stamp.nanosec)

        def command(self, message):
            now = time.monotonic()
            if now-self.last_cmd_emit < .02:
                return
            self.last_cmd_emit = now
            self.emit('cmd_vel', linear=round(message.linear.x, 5),
                      angular=round(message.angular.z, 5))

        def navigation_command(self, message):
            now = time.monotonic()
            if now-self.last_nav_cmd_emit < .04:
                return
            self.last_nav_cmd_emit = now
            self.emit('navigation_cmd_vel', linear=round(message.linear.x, 5),
                      angular=round(message.angular.z, 5))

        def speed_limit(self, message):
            self.emit('speed_limit', percentage=bool(message.percentage),
                      speed_limit=round(message.speed_limit, 5))

        def costmap(self, message):
            now = time.monotonic()
            if now-self.last_costmap_emit < 2.:
                return
            self.last_costmap_emit = now
            meta = message.metadata
            odom = self.last_input_samples.get('odom') or {}
            resolution = float(meta.resolution)
            width, height = int(meta.size_x), int(meta.size_y)
            if (resolution <= 0 or width <= 0 or height <= 0 or
                    len(message.data) != width*height or
                    not all(isinstance(odom.get(key), (int, float))
                            for key in ('x', 'y'))):
                return
            center_x = round((odom['x']-meta.origin.position.x)/resolution)
            center_y = round((odom['y']-meta.origin.position.y)/resolution)
            radius = min(20, max(1, round(1./resolution)))
            left, right = max(0, center_x-radius), min(width, center_x+radius+1)
            bottom, top = max(0, center_y-radius), min(height, center_y+radius+1)
            if left >= right or bottom >= top:
                return
            rows = [list(message.data[y*width+left:y*width+right])
                    for y in range(bottom, top)]
            self.emit('local_costmap_window', stamp_ns=(
                          message.header.stamp.sec*1_000_000_000+
                          message.header.stamp.nanosec),
                      frame_id=message.header.frame_id,
                      resolution=resolution,
                      origin=[round(meta.origin.position.x, 4),
                              round(meta.origin.position.y, 4)],
                      size=[width, height], bounds=[left, bottom, right, top],
                      robot_cell=[center_x, center_y], rows=rows)

        def scan(self, message):
            stamp_ns = (message.header.stamp.sec*1_000_000_000+
                        message.header.stamp.nanosec)
            front, front_angle = finite_sector_min(
                message, 0., math.radians(30))
            core, core_angle = finite_sector_min(
                message, 0., math.radians(12))
            rear, rear_angle = finite_sector_min(
                message, math.pi, math.radians(30))
            self.emit('scan', front_min_m=front,
                front_min_angle_deg=front_angle,
                front_core_min_m=core,
                front_core_angle_deg=core_angle,
                rear_min_m=rear,
                rear_min_angle_deg=rear_angle,
                stamp_ns=stamp_ns)
            self.last_input_samples['scan'] = {
                'stamp_ns': stamp_ns, 'frame_id': message.header.frame_id,
                'range_min': message.range_min, 'front_min_m': front}
            for pending in self.near_followups[:]:
                pending['future_scans'].append({
                    'stamp_ns': stamp_ns,
                    'neighbors': scan_neighborhood(message.ranges, pending['index'])})
                if len(pending['future_scans']) >= 3:
                    self.emit('scan_near_followup', near_key=pending['near_key'],
                              future_scans=pending['future_scans'])
                    self.near_followups.remove(pending)
            close = []
            for index, value in enumerate(message.ranges):
                angle = message.angle_min+index*message.angle_increment
                if (abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.radians(30)
                        and math.isfinite(value) and message.range_min <= value <=
                        min(message.range_max, message.range_min+.10)):
                    close.append((value, index, angle))
            if close:
                value, index, angle = min(close)
                near_key = f'{stamp_ns}:{index}'
                self.emit('scan_near_beam', near_key=near_key, stamp_ns=stamp_ns,
                          index=index, angle_deg=round(math.degrees(angle), 3),
                          range_m=round(value, 5), range_min=message.range_min,
                          range_max=message.range_max,
                          angle_min=message.angle_min,
                          angle_increment=message.angle_increment,
                          neighbors=scan_neighborhood(message.ranges, index),
                          previous_scans=[{
                              'stamp_ns': sample['stamp_ns'],
                              'neighbors': scan_neighborhood(sample['ranges'], index)}
                              for sample in self.previous_scans])
                self.near_followups.append({'near_key': near_key, 'index': index,
                                            'future_scans': []})
            self.previous_scans.append({'stamp_ns': stamp_ns,
                                        'ranges': tuple(message.ranges)})

        def clock(self, message):
            self.clock_stamp_ns = (message.clock.sec*1_000_000_000+
                                   message.clock.nanosec)

        def odom(self, message):
            now = time.monotonic()
            pose = message.pose.pose
            stamp_ns = (message.header.stamp.sec*1_000_000_000+
                        message.header.stamp.nanosec)
            values = {'stamp_ns': stamp_ns,
                      'clock_stamp_ns': self.clock_stamp_ns,
                      'frame_id': message.header.frame_id,
                      'child_frame_id': message.child_frame_id,
                      'x': round(pose.position.x, 5),
                      'y': round(pose.position.y, 5),
                      'yaw': round(yaw(pose.orientation), 5),
                      'linear_x': round(message.twist.twist.linear.x, 5),
                      'angular_z': round(message.twist.twist.angular.z, 5)}
            self.last_input_samples['odom'] = values
            if now-self.last_odom_emit >= .05:
                self.last_odom_emit = now
                self.emit('odom', **values)

        def footprint(self, message):
            points = message.polygon.points
            stamp_ns = (message.header.stamp.sec*1_000_000_000+
                        message.header.stamp.nanosec)
            values = {'stamp_ns': stamp_ns,
                      'clock_stamp_ns': self.clock_stamp_ns,
                      'frame_id': message.header.frame_id,
                      'point_count': len(points),
                      'centroid_x': round(sum(p.x for p in points)/len(points), 5)
                      if points else None,
                      'centroid_y': round(sum(p.y for p in points)/len(points), 5)
                      if points else None,
                      'points': [[round(p.x, 5), round(p.y, 5)] for p in points[:32]]}
            self.last_input_samples['footprint'] = values
            self.emit('footprint', **values)

        def tf(self, message, source):
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip('/')
                child = transform.child_frame_id.lstrip('/')
                if (parent, child) not in {('odom', 'base_footprint'),
                                           ('odom', 'base_link'), ('map', 'odom')}:
                    continue
                stamp_ns = (transform.header.stamp.sec*1_000_000_000+
                            transform.header.stamp.nanosec)
                values = {'source': source, 'parent': parent, 'child': child,
                          'stamp_ns': stamp_ns, 'clock_stamp_ns': self.clock_stamp_ns,
                          'x': round(transform.transform.translation.x, 5),
                          'y': round(transform.transform.translation.y, 5),
                          'yaw': round(yaw(transform.transform.rotation), 5)}
                self.last_input_samples[f'{source}_tf_{parent}_{child}'] = values
                self.emit('tf_stamp', **values)

        def truth(self, message):
            if 'waffle_pi' not in message.name:
                return
            now = time.monotonic()
            if now-self.last_truth_emit < .04:
                return
            self.last_truth_emit = now
            pose = message.pose[message.name.index('waffle_pi')]
            self.emit('truth', x=round(pose.position.x, 5),
                      y=round(pose.position.y, 5), yaw=round(yaw(pose.orientation), 5))

    with args.output.open('w') as stream:
        node = Observer(stream)
        try:
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
