#!/usr/bin/env python3
"""Reconstruct a frozen incident input and inspect the real Nav2 DWB critics.

This is an offline controller diagnostic, not a robot or Gazebo launcher. It
uses a loopback-only ROS domain and remaps all velocity output to a replay
topic. Costmaps are rebuilt from the recorded scan; they were not captured in
older incident files, so the result is not an exact replay of costmap history.
"""

import argparse
import copy
import gzip
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def load_snapshot(incident, at_time):
    """Keep the latest received sensor/path/transform values at a source time."""
    latest = {}
    transforms = {}
    opener = gzip.open if str(incident).endswith('.gz') else open
    with opener(incident, 'rt') as stream:
        for line in stream:
            event = json.loads(line)
            received = event.get('ros_time_ns', 0) / 1.e9
            if received > at_time:
                continue
            topic = event.get('topic')
            data = event.get('data', {})
            if topic == '/tf':
                for transform in data.get('transforms', []):
                    key = (transform['header']['frame_id'], transform['child_frame_id'])
                    transforms[key] = transform
            elif topic in ('/scan', '/odom', '/speed_limit', '/smart_vision_amr/path'):
                latest[topic] = data
    required = ('/scan', '/odom', '/smart_vision_amr/path')
    missing = [topic for topic in required if topic not in latest]
    if missing or not latest.get('/smart_vision_amr/path', {}).get('poses'):
        raise ValueError(f'snapshot lacks an active route or sensor inputs: {missing}')
    if ('map', 'odom') not in transforms:
        raise ValueError('snapshot lacks map -> odom transform')
    latest['transforms'] = list(transforms.values())
    latest['source_time'] = at_time
    return latest


def build_parameters(source, overrides=()):
    """Diagnostic publication changes must not silently tune motion behavior."""
    import yaml
    profile = yaml.safe_load(Path(source).read_text())
    params = {name: copy.deepcopy(profile[name])
              for name in ('controller_server', 'local_costmap')}
    controller = params['controller_server']['ros__parameters']
    controller.update({'use_sim_time': False, 'bond_heartbeat_period': 0.0})
    controller['FollowPath'].update({
        'publish_evaluation': True,
        'publish_trajectories': False,
        'publish_cost_grid_pc': False,
        'publish_local_plan': True,
        'publish_transformed_plan': True,
        'short_circuit_trajectory_evaluation': False,
    })
    params['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] = False
    for assignment in overrides:
        key, separator, value = assignment.partition('=')
        if not separator or not key.startswith('FollowPath.'):
            raise ValueError('overrides use FollowPath.parameter=YAML-value')
        controller['FollowPath'][key[len('FollowPath.'):]] = yaml.safe_load(value)
    return params


def summarize_evaluation(evaluation):
    """Retain every critic of the winner and best forward option, plus rejects."""
    twists = evaluation.get('twists', [])
    valid = [item for item in twists if item.get('total', -1.) >= 0.]
    forward = [item for item in valid if item['traj']['velocity']['x'] >= .05]
    invalid = {}
    for item in twists:
        if item.get('total', -1.) < 0.:
            critic = next((score['name'] for score in item.get('scores', [])
                           if score.get('raw_score', 0.) < 0.), 'unknown')
            invalid[critic] = invalid.get(critic, 0) + 1

    def compact(item):
        if item is None:
            return None
        return {'velocity': item['traj']['velocity'], 'total': item['total'],
                'scores': item.get('scores', []),
                'end_pose': item['traj'].get('poses', [None])[-1]}

    index = evaluation.get('best_index', 0)
    best = twists[index] if index < len(twists) else None
    return {'best': compact(best),
            'best_forward': compact(min(forward, key=lambda item: item['total'])
                                    if forward else None),
            'valid_count': len(valid), 'forward_valid_count': len(forward),
            'invalid_by_critic': invalid, 'trajectory_count': len(twists)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('incident', type=Path)
    parser.add_argument('--at-time', type=float, required=True,
                        help='original absolute ROS seconds to freeze')
    parser.add_argument('--nav-params', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--domain', type=int, default=186)
    parser.add_argument('--override', action='append', default=[])
    args = parser.parse_args()
    if not 120 <= args.domain <= 220 or args.domain == 176:
        parser.error('use a dedicated offline domain 120..220 other than simulator domain 176')
    if not .5 <= args.duration <= 30.:
        parser.error('duration must be between 0.5 and 30 seconds')
    if args.output.exists():
        parser.error('output directory already exists')
    snapshot = load_snapshot(args.incident, args.at_time)
    parameters = build_parameters(args.nav_params, args.override)
    args.output.mkdir(parents=True)
    import yaml
    parameter_file = args.output / 'controller_parameters.yaml'
    parameter_file.write_text(yaml.safe_dump(parameters, sort_keys=False))
    (args.output / 'input_snapshot.json').write_text(json.dumps(snapshot))
    (args.output / 'manifest.json').write_text(json.dumps({
        'source_incident': str(args.incident.resolve()),
        'source_time': args.at_time, 'source_nav_parameters': str(args.nav_params.resolve()),
        'domain': args.domain, 'duration': args.duration, 'overrides': args.override,
        'output_command_topic': '/replay/controller_cmd_vel',
        'costmap_source': 'reconstructed from a frozen captured scan, not captured costmap history',
        'static_tf_source': 'TurtleBot3 Waffle Pi URDF: footprint->link z=.01; link->scan x=-.064,z=.122',
        'model': 'frozen open-loop inputs; no claim of closed-loop drive success',
    }, indent=2))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    run_controller(args, snapshot, parameter_file)


def run_controller(args, snapshot, parameter_file):
    # Import ROS only after setting the isolated environment, including rclpy's
    # default-context initialization. No publishers in this tool target cmd_vel.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    from dwb_msgs.msg import LocalPlanEvaluation
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import FollowPath
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init()
    node = Node('incident_controller_replay')
    records = []
    started = time.monotonic()

    def record(kind, data):
        records.append({'elapsed': time.monotonic()-started, 'kind': kind, 'data': data})

    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    speed_pub = node.create_publisher(SpeedLimit, '/speed_limit', 10)
    tf_pub = TransformBroadcaster(node)
    static_pub = StaticTransformBroadcaster(node)
    static_transforms = []
    for parent, child, translation in (
            ('base_footprint', 'base_link', (0., 0., .01)),
            ('base_link', 'base_scan', (-.064, 0., .122))):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y, \
            transform.transform.translation.z = translation
        transform.transform.rotation.w = 1.
        static_transforms.append(transform)
    static_pub.sendTransform(static_transforms)

    def publish_inputs():
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        set_message_fields(odom, snapshot['/odom'])
        odom.header.stamp = stamp
        odom_pub.publish(odom)
        transforms = []
        for source in snapshot['transforms']:
            if (source['header']['frame_id'], source['child_frame_id']) not in (
                    ('map', 'odom'), ('odom', 'base_footprint')):
                continue
            transform = TransformStamped()
            set_message_fields(transform, source)
            transform.header.stamp = stamp
            transforms.append(transform)
        tf_pub.sendTransform(transforms)
        scan = LaserScan()
        source_scan = copy.deepcopy(snapshot['/scan'])
        source_scan['ranges'] = [float(value) if isinstance(value, (float, int))
                                 else math.inf for value in source_scan['ranges']]
        set_message_fields(scan, source_scan)
        scan.header.stamp = stamp
        scan_pub.publish(scan)
        if '/speed_limit' in snapshot:
            speed = SpeedLimit()
            set_message_fields(speed, snapshot['/speed_limit'])
            speed.header.stamp = stamp
            speed_pub.publish(speed)

    timer = node.create_timer(.05, publish_inputs)
    subscriptions = [
        node.create_subscription(Twist, '/replay/controller_cmd_vel',
                                 lambda msg: record('command', message_to_ordereddict(msg)), 100),
        node.create_subscription(LocalPlanEvaluation, '/evaluation',
                                 lambda msg: record('evaluation', summarize_evaluation(
                                     message_to_ordereddict(msg))), 10),
        node.create_subscription(RosPath, '/transformed_global_plan',
                                 lambda msg: record('transformed_plan',
                                                    message_to_ordereddict(msg)), 10),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda msg: record('costmap', message_to_ordereddict(msg)), 10),
        node.create_subscription(String, '/smart_vision_amr/controller_diagnostics',
                                 lambda msg: record('controller_diagnostics', json.loads(msg.data)), 10),
    ]

    def spin_until(future, timeout=15.):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise RuntimeError('controller replay service/action timeout')
        return future.result()

    server_log = (args.output / 'controller.log').open('w')
    executable = '/opt/ros/humble/lib/nav2_controller/controller_server'
    server = subprocess.Popen([executable, '--ros-args', '--params-file', str(parameter_file),
                               '-r', 'cmd_vel:=/replay/controller_cmd_vel'],
                              stdout=server_log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    try:
        lifecycle = node.create_client(ChangeState, '/controller_server/change_state')
        deadline = time.monotonic() + 20.
        while not lifecycle.wait_for_service(timeout_sec=.05):
            rclpy.spin_once(node, timeout_sec=.05)
            if server.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('controller lifecycle service unavailable; see controller.log')
        for transition in (1, 3):  # configure then activate; child costmap follows controller.
            request = ChangeState.Request()
            request.transition.id = transition
            if not spin_until(lifecycle.call_async(request), 30.).success:
                raise RuntimeError(f'controller lifecycle transition {transition} failed')
        ready_deadline = time.monotonic()+1.
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        action = ActionClient(node, FollowPath, '/follow_path')
        if not action.wait_for_server(timeout_sec=5.):
            raise RuntimeError('FollowPath unavailable')
        goal = FollowPath.Goal()
        set_message_fields(goal.path, snapshot['/smart_vision_amr/path'])
        goal.path.header.stamp = node.get_clock().now().to_msg()
        for pose in goal.path.poses:
            pose.header.stamp = goal.path.header.stamp
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        handle = spin_until(action.send_goal_async(goal))
        record('goal_accepted', handle.accepted)
        if not handle.accepted:
            raise RuntimeError('FollowPath rejected replay route')
        result_future = handle.get_result_async()
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        spin_until(handle.cancel_goal_async(), 5.)
        if result_future.done():
            record('action_result_status', result_future.result().status)
    finally:
        timer.cancel()
        if server.poll() is None:
            os.killpg(server.pid, signal.SIGINT)
            try:
                server.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGTERM)
                server.wait(timeout=3.)
        server_log.close()
        with (args.output / 'observations.jsonl').open('w') as stream:
            for event in records:
                stream.write(json.dumps(event)+'\n')
        evaluations = [event['data'] for event in records if event['kind'] == 'evaluation']
        commands = [event['data'] for event in records if event['kind'] == 'command']
        summary = {'evaluation_count': len(evaluations), 'command_count': len(commands),
                   'best': evaluations[-1] if evaluations else None,
                   'linear_max': max((msg['linear']['x'] for msg in commands), default=None),
                   'angular_abs_max': max((abs(msg['angular']['z']) for msg in commands), default=None)}
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
