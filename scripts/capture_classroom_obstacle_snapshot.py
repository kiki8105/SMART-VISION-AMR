#!/usr/bin/env python3
"""Read-only map/pose/live-cell snapshot from an isolated Gazebo trial."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=176)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=12.)
    parser.add_argument('--raw-scan-topic', default='/classroom_fault/raw_scan')
    args = parser.parse_args()
    if args.domain == 16 or os.environ.get('ROS_DOMAIN_ID') != str(args.domain):
        raise SystemExit('Refusing physical or mismatched domain')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('ROS_LOCALHOST_ONLY=1 required')

    import rclpy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32MultiArray, String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from smart_vision_amr.map_utils import (occupancy_to_grid,
                                             first_static_obstacle_distance)

    rclpy.init(args=[])
    node = rclpy.create_node('classroom_fault_snapshot')
    tf_buffer = Buffer(cache_time=Duration(seconds=10.))
    tf_listener = TransformListener(tf_buffer, node)
    latest = {}
    latest_at = {}
    paired_pose = None

    def pose_data(p):
        q = p.orientation
        return {'x': p.position.x, 'y': p.position.y,
                'yaw': math.atan2(2*(q.w*q.z+q.x*q.y),
                                  1-2*(q.y*q.y+q.z*q.z))}

    def record(name, message):
        nonlocal paired_pose
        latest[name] = message
        latest_at[name] = time.monotonic()
        if 'amcl' in latest and 'truth' in latest:
            if abs(latest_at['amcl']-latest_at['truth']) <= .15:
                estimate = pose_data(latest['amcl'].pose.pose)
                truth = pose_data(latest['truth'])
                paired_pose = {'amcl': estimate, 'gazebo': truth,
                               'receipt_gap_s': abs(latest_at['amcl']-latest_at['truth']),
                               'position_error_m': math.hypot(estimate['x']-truth['x'],
                                                              estimate['y']-truth['y']),
                               'yaw_error_rad': math.atan2(
                                   math.sin(estimate['yaw']-truth['yaw']),
                                   math.cos(estimate['yaw']-truth['yaw']))}

    def on_truth(message):
        if 'waffle_pi' in message.name:
            record('truth', message.pose[message.name.index('waffle_pi')])

    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    specs = [('map', OccupancyGrid, '/map', durable),
             ('observed', Int32MultiArray, '/smart_vision_amr/observed_obstacles', 10),
             ('amcl', PoseWithCovarianceStamped, '/amcl_pose', 10),
             ('odom', Odometry, '/odom', qos_profile_sensor_data),
             ('raw_scan', LaserScan, args.raw_scan_topic, qos_profile_sensor_data),
             ('planner', String, '/smart_vision_amr/planner_status', durable)]
    for name, kind, topic, qos in specs:
        node.create_subscription(kind, topic,
                                 lambda message, field=name: record(field, message), qos)
    node.create_subscription(ModelStates, '/gazebo/model_states', on_truth,
                             qos_profile_sensor_data)
    deadline = time.monotonic()+args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        # AMCL deliberately publishes on movement, so it may be silent during
        # an obstacle hold. Ground truth plus scan-stamped TF remain useful.
        if any(name not in latest for name in ('map', 'observed', 'odom',
                                                'raw_scan', 'planner', 'truth')):
            continue
        if abs(latest_at['raw_scan']-latest_at['truth']) > .15:
            continue
        scan = latest['raw_scan']
        if tf_buffer.can_transform('map', scan.header.frame_id,
                                   Time.from_msg(scan.header.stamp)):
            break
    if 'map' not in latest:
        raise SystemExit('Map not received')
    grid = latest['map']
    width, height = grid.info.width, grid.info.height
    data = tuple(grid.data)
    hits = [int(i) for i in latest.get('observed', Int32MultiArray()).data]
    unique = sorted(set(i for i in hits if 0 <= i < width*height))
    classes = {'known_free': [], 'occupied': [], 'unknown': []}
    for index in unique:
        cell = [index % width, index // width]
        value = data[index]
        classes['known_free' if value == 0 else 'unknown' if value < 0 else 'occupied'].append(cell)
    pose = None
    if 'amcl' in latest:
        p = latest['amcl'].pose.pose
        pose = pose_data(p)
        pose['cell'] = [math.floor((pose['x']-grid.info.origin.position.x)/grid.info.resolution),
                        math.floor((pose['y']-grid.info.origin.position.y)/grid.info.resolution)]
    truth_pose = pose_data(latest['truth']) if 'truth' in latest else None
    odom_pose = pose_data(latest['odom'].pose.pose) if 'odom' in latest else None
    try:
        planner = json.loads(latest['planner'].data) if 'planner' in latest else None
    except ValueError:
        planner = latest['planner'].data if 'planner' in latest else None
    wall_probe = None
    if 'truth' in latest and 'raw_scan' in latest:
        scan = latest['raw_scan']
        known_walls = occupancy_to_grid(grid, 65, 0, unknown_is_obstacle=False)
        nonfree_map = occupancy_to_grid(grid, 65, 0, unknown_is_obstacle=True)
        gap = abs(latest_at['raw_scan']-latest_at['truth'])
        matches = []
        tf_at_scan = None
        tf_error = None
        try:
            tf_at_scan = tf_buffer.lookup_transform(
                'map', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        except TransformException as exc:
            tf_error = str(exc)
        # Both model origin and AMCL base pose use the Waffle Pi sensor's
        # explicit -6.4 cm fore-aft offset. A 15-cm receipt gap is approximate
        # synchrony, not Gazebo ground-truth scan stamping.
        for source in ('gazebo', 'amcl', 'scan_stamped_tf'):
            if source == 'amcl' and paired_pose is None:
                continue
            if source == 'scan_stamped_tf':
                if tf_at_scan is None:
                    continue
                q = tf_at_scan.transform.rotation
                sensor_x = tf_at_scan.transform.translation.x
                sensor_y = tf_at_scan.transform.translation.y
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                                 1-2*(q.y*q.y+q.z*q.z))
            else:
                base = truth_pose if source == 'gazebo' else paired_pose[source]
                sensor_x = base['x'] - .064*math.cos(base['yaw'])
                sensor_y = base['y'] - .064*math.sin(base['yaw'])
                yaw = base['yaw']
            values = []
            for beam, measured in enumerate(scan.ranges):
                if not math.isfinite(measured) or not scan.range_min <= measured <= min(1., scan.range_max):
                    continue
                angle = yaw + scan.angle_min + beam*scan.angle_increment
                expected = first_static_obstacle_distance(grid.info, known_walls,
                                                          sensor_x, sensor_y, angle,
                                                          scan.range_max)
                first_nonfree = first_static_obstacle_distance(
                    grid.info, nonfree_map, sensor_x, sensor_y, angle,
                    scan.range_max)
                if expected is None:
                    continue
                values.append({'beam': beam, 'measured_m': round(measured, 3),
                               'first_wall_m': round(expected, 3),
                               'first_nonfree_m': (round(first_nonfree, 3)
                                                   if first_nonfree is not None else None),
                               'wall_minus_measured_m': round(expected-measured, 3)})
            matches.append((source, values))
        wall_probe = {'scan_to_pose_receipt_gap_s': round(gap, 3),
                      'approximately_synchronized': gap <= .15,
                      'scan_stamped_tf_error': tf_error,
                      'scan_stamped_tf': ({'translation': {
                          'x': tf_at_scan.transform.translation.x,
                          'y': tf_at_scan.transform.translation.y,
                          'z': tf_at_scan.transform.translation.z},
                          'rotation': {
                          'x': tf_at_scan.transform.rotation.x,
                          'y': tf_at_scan.transform.rotation.y,
                          'z': tf_at_scan.transform.rotation.z,
                          'w': tf_at_scan.transform.rotation.w}}
                          if tf_at_scan is not None else None),
                      'message_stamps_s': {'raw_scan': scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,
                          'amcl': (latest['amcl'].header.stamp.sec+latest['amcl'].header.stamp.nanosec*1e-9
                                   if 'amcl' in latest else None),
                          'odom': latest['odom'].header.stamp.sec+latest['odom'].header.stamp.nanosec*1e-9},
                      'scan_geometry': {'frame_id': scan.header.frame_id,
                          'angle_min': scan.angle_min,
                          'angle_increment': scan.angle_increment,
                          'range_min': scan.range_min, 'range_max': scan.range_max},
                      'raw_ranges_m': [round(float(r), 5) if math.isfinite(r) else None
                                       for r in scan.ranges],
                      'per_pose': {name: {'beam_count': len(values),
                          'within_5cm': sum(abs(v['wall_minus_measured_m']) <= .05
                                            for v in values),
                          'within_8cm': sum(abs(v['wall_minus_measured_m']) <= .08
                                            for v in values),
                          'wall_over_5cm_farther': sum(v['wall_minus_measured_m'] > .05
                                                        for v in values),
                          'largest_wall_distance_excess': sorted(values,
                              key=lambda v: v['wall_minus_measured_m'], reverse=True)[:12],
                          'beam_samples': values if name == 'scan_stamped_tf' else None}
                          for name, values in matches}}
    result = {'domain': args.domain, 'map': {'width': width, 'height': height,
              'resolution': grid.info.resolution,
              'origin': [grid.info.origin.position.x, grid.info.origin.position.y],
              'sha256': hashlib.sha256(bytes(value+1 for value in data)).hexdigest()},
              'amcl_pose': pose, 'gazebo_truth_pose': truth_pose,
              'odom_pose': odom_pose, 'paired_ground_truth': paired_pose,
              'observed_indices': unique,
              'observed_by_class': classes, 'planner_status': planner,
              'wall_echo_probe': wall_probe,
              'odom_received': 'odom' in latest,
              'missing': [name for name, *_ in specs if name not in latest]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
    print(args.output)


if __name__ == '__main__':
    main()
