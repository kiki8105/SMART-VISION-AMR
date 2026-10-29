#!/usr/bin/env python3
"""Observe or explicitly execute a repeatable route on an already running stack.

No goals are sent unless --execute is supplied. Wall-clock/physical execution
also requires --allow-hardware. This script measures estimated TF tracking,
not independently verified physical positioning accuracy.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time
import uuid


def point_path_distance(x, y, points):
    if not points:
        return None
    distances = [math.hypot(x-points[0][0], y-points[0][1])]
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        t = 0.0 if length2 <= 1e-12 else max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/length2))
        distances.append(math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
    return min(distances)


def distribution(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return {'count': 0, 'mean': None, 'rmse': None, 'p95': None, 'max': None}
    ordered = sorted(abs(value) for value in values)
    return {'count': len(values), 'mean': statistics.mean(values),
            'rmse': math.sqrt(statistics.mean(value*value for value in values)),
            'p95': ordered[min(len(ordered)-1, math.ceil(.95*len(ordered))-1)],
            'max': max(ordered)}


class NavigationMetrics:
    BLOCKED = {'WAIT_REPLAN', 'RECOVERY_FAILED', 'SAFETY_STOP', 'BLOCKED',
               'INVALID_GOAL', 'FAILED'}

    def __init__(self):
        self.samples, self.commands, self.events, self.goals = [], [], [], []
        self.path, self.path_key = [], None
        self.path_count = self.replans = self.blocked_count = self.pose_jumps = 0
        self.status_by_source = {}
        self.distance = 0.0
        self.previous_pose = None
        self.active_goal = None
        self.latest_command = None
        self.latest_health = None
        self.last_derivative_command = self.last_acceleration = None
        self.linear_acceleration, self.angular_acceleration = [], []
        self.linear_jerk, self.angular_jerk = [], []

    def goal(self, x, y, stamp_ns, now):
        if self.active_goal is not None and self.active_goal['stamp_ns'] == stamp_ns:
            return
        self.active_goal = {'x': x, 'y': y, 'stamp_ns': stamp_ns,
                            'started_seconds': now, 'result': 'running'}
        self.goals.append(self.active_goal)
        self.path, self.path_key = [], None

    def planned(self, points, now):
        if not points:
            self.events.append({'time_seconds': now, 'event': 'path_cleared'})
            self.path = []
            return
        key = tuple((round(x, 3), round(y, 3)) for x, y in points)
        if key != self.path_key:
            self.path_count += 1
            if self.path_key is not None:
                self.replans += 1
            self.path_key = key
            self.events.append({'time_seconds': now, 'event': 'path', 'path_id': self.path_count})
        self.path = list(points)

    def status(self, source, payload, now):
        status = str(payload.get('status', 'UNKNOWN'))
        if status != self.status_by_source.get(source):
            self.events.append({'time_seconds': now, 'event': 'status', 'source': source,
                                'status': status, 'message': payload.get('message', '')})
            if status in self.BLOCKED:
                self.blocked_count += 1
        self.status_by_source[source] = status
        goal_stamp = payload.get('goal_stamp') or {}
        stamp = goal_stamp.get('sec', 0)*1_000_000_000+goal_stamp.get('nanosec', 0)
        if self.active_goal is not None and stamp == self.active_goal['stamp_ns']:
            if status in ('SUCCEEDED', 'CANCELED'):
                self.active_goal.update(result=status.lower(), ended_seconds=now)

    def command(self, now, linear, angular):
        if not all(math.isfinite(value) for value in (now, linear, angular)):
            return
        self.latest_command = (now, linear, angular)
        self.commands.append({'time_seconds': now, 'linear_mps': linear, 'angular_radps': angular})
        previous = self.last_derivative_command
        # Twist has no generation timestamp. Coalesce <20 ms callback bursts;
        # derivatives describe observed commands, not mechanical acceleration.
        if previous is None:
            self.last_derivative_command = (now, linear, angular)
        elif now-previous[0] >= 0.02:
            dt = now-previous[0]
            acceleration = ((linear-previous[1])/dt, (angular-previous[2])/dt)
            self.linear_acceleration.append(acceleration[0])
            self.angular_acceleration.append(acceleration[1])
            if self.last_acceleration is not None:
                self.linear_jerk.append((acceleration[0]-self.last_acceleration[1])/dt)
                self.angular_jerk.append((acceleration[1]-self.last_acceleration[2])/dt)
            self.last_acceleration = (now, *acceleration)
            self.last_derivative_command = (now, linear, angular)

    def pose(self, now, ros_seconds, x, y, yaw):
        if not all(math.isfinite(value) for value in (now, ros_seconds, x, y, yaw)):
            return
        if self.previous_pose is not None:
            dt, distance = now-self.previous_pose[0], math.hypot(x-self.previous_pose[1], y-self.previous_pose[2])
            if dt > 0.0 and distance > 0.25+1.0*dt:
                self.pose_jumps += 1
                self.events.append({'time_seconds': now, 'event': 'tf_pose_jump', 'distance_m': distance})
            elif dt > 0.0:
                self.distance += distance
        self.previous_pose = (now, x, y)
        cmd = self.latest_command
        fresh_cmd = cmd is not None and now-cmd[0] <= 0.5
        goal = self.active_goal
        self.samples.append({
            'time_seconds': now, 'ros_seconds': ros_seconds, 'x': x, 'y': y, 'yaw': yaw,
            'cross_track_m': point_path_distance(x, y, self.path), 'path_id': self.path_count,
            'goal_index': len(self.goals),
            'goal_distance_m': None if goal is None else math.hypot(x-goal['x'], y-goal['y']),
            'linear_mps': cmd[1] if fresh_cmd else None,
            'angular_radps': cmd[2] if fresh_cmd else None,
            'navigation_status': self.status_by_source.get('navigation', ''),
            'planner_status': self.status_by_source.get('planner', ''),
            'localization_level': self.latest_health,
        })

    def summary(self, duration):
        completed = sum(goal['result'] == 'succeeded' for goal in self.goals)
        commands = [row for row in self.samples if row['linear_mps'] is not None]
        stop_fraction = (sum(abs(row['linear_mps']) < .005 and abs(row['angular_radps']) < .02
                             for row in commands)/len(commands)) if commands else None
        return {'duration_seconds': duration, 'goal_count': len(self.goals),
                'completed_goals': completed,
                'completion_rate': completed/len(self.goals) if self.goals else None,
                'estimated_distance_m': self.distance, 'tf_pose_jumps': self.pose_jumps,
                'cross_track_m': distribution([row['cross_track_m'] for row in self.samples]),
                'plans': self.path_count, 'replans': self.replans,
                'blocked_transitions': self.blocked_count,
                'zero_command_fraction': stop_fraction,
                'fresh_command_sample_fraction': len(commands)/len(self.samples) if self.samples else None,
                'linear_acceleration_mps2': distribution(self.linear_acceleration),
                'angular_acceleration_radps2': distribution(self.angular_acceleration),
                'linear_jerk_mps3': distribution(self.linear_jerk),
                'angular_jerk_radps3': distribution(self.angular_jerk),
                'sample_count': len(self.samples), 'goals': self.goals}


def config_hashes(paths):
    return {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def execute_allowed(*, execute, use_sim_time, domain, allow_hardware):
    # Known deployed physical domains remain protected even if --use-sim-time
    # is accidentally copied into a robot terminal.
    if execute and (not use_sim_time or domain in (16, 226, 227)) and not allow_hardware:
        raise ValueError('Execution may reach hardware; explicitly pass --allow-hardware')


def benchmark_auto_packet(status, received_at, now, ros_now_ns, source_id, seq,
                          foreign_navigation_active=False):
    """Permit explicit benchmark execution without taking an operator lease."""
    if status is None or received_at is None or not 0.0 <= now-received_at <= 1.0:
        raise ValueError('Fresh motion mux status is required before benchmark execution')
    if status.get('available') is not True:
        raise ValueError('Motion mux does not have exclusive available output')
    if status.get('estop_latched') is not False or status.get('reset_pending'):
        raise ValueError('Operator must explicitly reset emergency stop first')
    if status.get('mode') not in ('hold', 'auto'):
        raise ValueError('Manual/localization control is active; benchmark will not take control')
    owner = status.get('source_id')
    if owner not in (None, source_id) and not str(owner).startswith('mux-service:'):
        raise ValueError('Another controller owns the mux; release it through the operator reset workflow')
    if foreign_navigation_active:
        raise ValueError('Another navigation mission is active; finish or cancel it first')
    if type(ros_now_ns) is not int or ros_now_ns <= 0 or type(seq) is not int or seq < 1:
        raise ValueError('Valid ROS clock and ordered sequence are required')
    return {'source_id': source_id, 'seq': seq, 'sent_at_ns': ros_now_ns,
            'mode': 'auto', 'linear': 0.0, 'angular': 0.0}


def run(args, route):
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path as RosPath
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener

    class Trial(Node):
        def __init__(self):
            super().__init__('navigation_benchmark', parameter_overrides=[
                Parameter('use_sim_time', value=args.use_sim_time)])
            self.metrics = NavigationMetrics()
            self.started = time.monotonic()
            self.clock_seen = False
            self.tf_buffer = Buffer()
            self.listener = TransformListener(self.tf_buffer, self)
            self.goal_pub = (self.create_publisher(PoseStamped, '/smart_vision_amr/goal', 10)
                             if args.execute else None)
            self.control_pub = (self.create_publisher(String, '/smart_vision_amr/manual_control', 10)
                                if args.execute else None)
            self.control_source = 'benchmark:'+uuid.uuid4().hex
            self.control_seq = 0
            self.motion_status = self.motion_status_at = None
            self.foreign_navigation_active = False
            self.auto_requested_at = None
            self.owns_route = False
            self.stop_client = self.create_client(Trigger, '/smart_vision_amr/emergency_stop')
            self.cancel_client = self.create_client(Trigger, '/smart_vision_amr/cancel_navigation')
            self.route_index = 0
            self.active_since = None
            self.failure = None
            self.done = False
            self.last_sample_at = None
            self.last_sent_stamp = None
            self.health = self.health_at = None
            self.cleanup_requests = []
            self.create_subscription(RosPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(PoseStamped, '/smart_vision_amr/goal', self.on_goal, 10)
            self.create_subscription(Twist, '/cmd_vel', lambda msg: self.metrics.command(
                self.elapsed(), msg.linear.x, msg.angular.z), qos_profile_sensor_data)
            for source, topic in (('planner', '/smart_vision_amr/planner_status'),
                                  ('navigation', '/smart_vision_amr/navigation_status')):
                self.create_subscription(String, topic,
                    lambda msg, label=source: self.on_status(label, msg), 10)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/smart_vision_amr/localization_health', self.on_health, durable)
            self.create_subscription(String, '/smart_vision_amr/motion_status', self.on_motion, durable)
            self.create_subscription(Clock, '/clock', lambda msg: setattr(
                self, 'clock_seen', msg.clock.sec+msg.clock.nanosec/1e9 > 0), qos_profile_sensor_data)

        def elapsed(self):
            return time.monotonic()-self.started

        def on_path(self, message):
            if message.header.frame_id == args.map_frame:
                self.metrics.planned([(pose.pose.position.x, pose.pose.position.y)
                                      for pose in message.poses], self.elapsed())

        def on_goal(self, message):
            if message.header.frame_id == args.map_frame:
                stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                self.metrics.goal(message.pose.position.x, message.pose.position.y, stamp, self.elapsed())

        def on_status(self, source, message):
            try:
                payload = json.loads(message.data)
                if isinstance(payload, dict):
                    self.metrics.status(source, payload, self.elapsed())
                    if source == 'navigation':
                        stamp = payload.get('goal_stamp') or {}
                        stamp_ns = stamp.get('sec', 0)*1_000_000_000+stamp.get('nanosec', 0)
                        self.foreign_navigation_active = (
                            payload.get('status') in ('MOVING', 'FOLLOWING', 'RECOVERING')
                            and stamp_ns != self.last_sent_stamp)
            except (ValueError, TypeError):
                pass

        def on_health(self, message):
            try:
                payload = json.loads(message.data)
                self.metrics.latest_health = payload.get('level')
                self.health, self.health_at = payload, self.elapsed()
            except (ValueError, TypeError, AttributeError):
                pass

        def on_motion(self, message):
            try:
                payload = json.loads(message.data)
                if isinstance(payload, dict):
                    self.motion_status, self.motion_status_at = payload, self.elapsed()
            except (ValueError, TypeError):
                pass

        def tick(self):
            from smart_vision_amr.navigation_graph import goal_consumers_ready
            now = self.elapsed()
            if self.last_sample_at is None or now-self.last_sample_at >= .1:
                self.last_sample_at = now
                try:
                    tf = self.tf_buffer.lookup_transform(args.map_frame, args.base_frame, rclpy.time.Time())
                    age = (self.get_clock().now().nanoseconds
                           -(tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec))/1e9
                    if -.25 <= age <= 1.0:
                        t, q = tf.transform.translation, tf.transform.rotation
                        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        self.metrics.pose(now, self.get_clock().now().nanoseconds/1e9, t.x, t.y, yaw)
                except TransformException:
                    pass
            if not args.execute:
                self.done = now >= args.duration
                return
            if self.active_since is None:
                ready = (goal_consumers_ready(endpoint.node_name for endpoint in
                            self.get_subscriptions_info_by_topic('/smart_vision_amr/goal'))
                         and self.health is not None and self.health.get('autonomous_allowed') is True
                         and now-self.health_at <= 1.0)
                if (not self.metrics.samples or (args.use_sim_time and not self.clock_seen)
                        or not ready):
                    if now > args.startup_timeout:
                        self.failure, self.done = 'Fresh TF, localization confidence or simulation /clock not ready', True
                    return
                if self.route_index >= len(route):
                    self.done = True
                    return
                if self.motion_status is None:
                    if now > args.startup_timeout:
                        self.failure, self.done = 'No fresh motion mux status before startup timeout', True
                    return
                try:
                    packet = benchmark_auto_packet(
                        self.motion_status, self.motion_status_at, now,
                        self.get_clock().now().nanoseconds, self.control_source,
                        self.control_seq+1, self.foreign_navigation_active)
                except ValueError as exc:
                    self.failure, self.done = str(exc), True
                    return
                if self.auto_requested_at is None:
                    self.control_seq += 1
                    self.control_pub.publish(String(data=json.dumps(packet)))
                    self.auto_requested_at = now
                    return
                if (self.motion_status.get('source_id') != self.control_source
                        or self.motion_status.get('mode') != 'auto'):
                    if now-self.auto_requested_at > 2.0:
                        self.failure, self.done = 'Motion mux did not acknowledge benchmark auto mode', True
                    return
                message = PoseStamped()
                message.header.frame_id = args.map_frame
                message.header.stamp = self.get_clock().now().to_msg()
                x, y, yaw = route[self.route_index]
                message.pose.position.x, message.pose.position.y = x, y
                message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                self.last_sent_stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                self.metrics.goal(x, y, self.last_sent_stamp, now)
                self.owns_route = True
                self.goal_pub.publish(message)
                self.auto_requested_at = None
                self.active_since = now
                self.get_logger().info(f'Benchmark goal {self.route_index+1}/{len(route)}: {x}, {y}')
            else:
                active = self.metrics.active_goal
                if active and active['stamp_ns'] == self.last_sent_stamp and active['result'] == 'succeeded':
                    self.route_index += 1
                    self.active_since = None
                elif now-self.active_since > args.goal_timeout:
                    self.failure, self.done = f'Goal {self.route_index+1} timeout', True
            if now >= args.duration:
                self.failure, self.done = 'Overall wall duration reached', True

        def stop_owned_route(self):
            if not args.execute or not self.owns_route:
                return
            # A route published by this tool is always explicitly canceled on
            # exit. Failure/interruption additionally latches the common stop.
            clients = [self.cancel_client]
            if self.failure:
                clients.append(self.stop_client)
            for client in clients:
                result = {'service': client.srv_name, 'requested': False, 'acknowledged': False}
                if client.wait_for_service(timeout_sec=.5):
                    future = client.call_async(Trigger.Request())
                    result['requested'] = True
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                    if future.done() and future.exception() is None:
                        response = future.result()
                        result.update(acknowledged=bool(response.success), message=response.message)
                self.cleanup_requests.append(result)

    rclpy.init()
    node = Trial()
    interrupted = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.05)
            node.tick()
    except KeyboardInterrupt:
        interrupted, node.failure = True, 'Interrupted by operator'
    finally:
        if not node.metrics.samples and node.failure is None:
            node.failure = 'No fresh map->base TF samples recorded'
        node.stop_owned_route()
        report = node.metrics.summary(node.elapsed())
        report.update(mode=args.mode, domain=args.domain, executed=args.execute,
                      use_sim_time=args.use_sim_time, failure=node.failure,
                      cleanup_requests=node.cleanup_requests,
                      requested_route=route, config_sha256=config_hashes(args.config),
                      created_utc=datetime.now(timezone.utc).isoformat(),
                      measurement='map->base TF estimate, not external ground truth',
                      derivative_clock='steady receipt time of final cmd_vel; not mechanical acceleration')
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        for filename, rows in (('samples.csv', node.metrics.samples),
                                ('commands.csv', node.metrics.commands)):
            with (args.output/filename).open('w', newline='') as handle:
                if rows:
                    writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
        (args.output/'events.json').write_text(json.dumps(node.metrics.events, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 130 if interrupted else 1 if node.failure else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '76')))
    parser.add_argument('--mode', required=True, help='Trial label, e.g. baseline or ekf_independent')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--execute', action='store_true', help='Publish the explicitly provided waypoint route')
    parser.add_argument('--allow-hardware', action='store_true')
    parser.add_argument('--waypoint', action='append', default=[], help='x,y or x,y,yaw (radians); repeat for route')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--duration', type=float, default=180.0, help='Overall wall-clock recording limit')
    parser.add_argument('--goal-timeout', type=float, default=90.0)
    parser.add_argument('--startup-timeout', type=float, default=20.0)
    parser.add_argument('--config', action='append', default=[], type=Path, help='Config file to hash into report')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--output', type=Path, default=Path('benchmarks')/('navigation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')))
    args = parser.parse_args()
    try:
        if args.repeat < 1 or not 0 <= args.domain <= 232 or min(args.duration, args.goal_timeout, args.startup_timeout) <= 0:
            raise ValueError('Invalid repeat, domain, or duration')
        route = []
        for text in args.waypoint:
            point = [float(value) for value in text.split(',')]
            if len(point) not in (2, 3) or not all(math.isfinite(value) for value in point):
                raise ValueError('Each waypoint must contain finite x,y[,yaw]')
            route.append(tuple(point if len(point) == 3 else point+[0.0]))
        route *= args.repeat
        if args.execute and not route:
            raise ValueError('--execute requires at least one --waypoint')
        execute_allowed(execute=args.execute, use_sim_time=args.use_sim_time,
                        domain=args.domain, allow_hardware=args.allow_hardware)
        config_hashes(args.config)  # Fail before sending a goal if a file is missing.
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    raise SystemExit(run(args, route))


if __name__ == '__main__':
    main()
