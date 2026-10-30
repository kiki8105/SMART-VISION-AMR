#!/usr/bin/env python3
"""Finite acceptance checks against an explicitly isolated, real Gazebo stack.

This observer never starts/stops the stack. A copied simulation-only model must
publish its unmodified laser messages on /startup_validation/raw_scan; the probe
relays them to /scan and temporarily suppresses that relay for the outage test.
No pose, covariance, odometry, health, navigation result, or scan is fabricated.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import urllib.error
import urllib.request
import uuid


DOMAIN = 186
URL = 'http://127.0.0.1:8796'
GAZEBO = 'http://127.0.0.1:11586'


def compact_summary(summary):
    """Bound versioned samples; assertions always use the complete raw record."""
    limits = {'navigation_input_samples': 24, 'observed_obstacle_samples': 24,
              'projection': 24, 'scan_receipts': 24, 'tf_samples': 64,
              'ground_truth_samples': 80, 'owned_amcl_process_samples': 4,
              'planner_samples': 32, 'memory_samples': 32, 'amcl_samples': 60}
    sampling = {}
    for name, limit in limits.items():
        samples = summary.get(name)
        if not isinstance(samples, list) or len(samples) <= limit:
            continue
        indices = sorted({round(i*(len(samples)-1)/(limit-1)) for i in range(limit)})
        summary[name] = [samples[i] for i in indices]
        sampling[name] = {'before': len(samples), 'retained': len(indices)}
    if sampling:
        summary['evidence_sampling'] = {
            'method': 'Uniform bounded samples including first/last. Assertions, events and selected failure proofs are retained separately. Full raw telemetry is unchanged at the referenced path/hash.',
            'arrays': sampling}
    return summary


def api(path='/api/state', payload=None, allow_error=False):
    request = urllib.request.Request(
        URL+path, data=None if payload is None else json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=3) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as exc:
        body = json.load(exc)
        if allow_error:
            return exc.code, body
        raise RuntimeError(f'{path}: HTTP {exc.code}: {body}') from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--raw-output', type=Path,
                        help='Full raw telemetry outside versioned evidence')
    parser.add_argument('--scenario', choices=('startup', 'suite', 'home_initialization', 'navigation', 'precision',
                                              'local_detour', 'barrier'), default='suite')
    parser.add_argument('--initial-std', type=float, default=.446)
    parser.add_argument('--require-amcl-tf-best-effort', action='store_true',
                        help='Diagnostic copied-launch override; verify the actual AMCL participant endpoints before motion')
    parser.add_argument('--require-nav2-tf-best-effort', action='store_true',
                        help='Verify AMCL/controller/behavior dynamic TF policies before motion')
    parser.add_argument('--simulation-launch-pid', type=int,
                        help='Own launch process group; required for bounded AMCL TF freeze')
    args = parser.parse_args()
    expected = {'ROS_DOMAIN_ID': str(DOMAIN), 'ROS_LOCALHOST_ONLY': '1',
                'GAZEBO_MASTER_URI': GAZEBO}
    for key, value in expected.items():
        if os.environ.get(key) != value:
            raise SystemExit(f'Refusing non-isolated environment: {key} must be {value}')
    if args.output.exists():
        raise SystemExit('Refusing to overwrite evidence: '+str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from gazebo_msgs.msg import ContactsState, ModelStates
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                          qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32MultiArray, String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    start = time.monotonic()
    web_verified = False
    result = {'passed': False, 'scenario': args.scenario, 'domain': DOMAIN,
              'gazebo_master_uri': GAZEBO, 'web_url': URL,
              'physical_vehicle_or_domain16_touched': False,
              'synthetic_sensor_or_localization_feedback': False,
              'initial_pose_is_actual_known_simulator_pose': True,
              'initial_pose_std_requested_m': (args.initial_std if args.scenario in ('startup', 'suite') else None),
              'checks': [], 'states': [], 'events': [], 'commands': [],
              'odom': [], 'ground_truth': [], 'amcl': [], 'health': [],
              'motion': [], 'navigation': [], 'planner': [], 'paths': [], 'tf': [],
              'initial_poses': [], 'memory': [], 'robot_contacts': []}
    result['path_events'] = []
    result['observed_obstacle_samples'] = []
    result['navigation_input_samples'] = []
    result['scan_receipts'] = []
    result['projection'] = []
    result['start_monotonic_ns'] = int(start*1e9)
    result['raw_dynamic_tf_receipts'] = []
    result['owned_amcl_process_samples'] = []
    owned_obstacles = []

    def elapsed():
        return round(time.monotonic()-start, 4)

    def event(name, **kwargs):
        value = {'t': elapsed(), 'event': name, **kwargs}
        result['events'].append(value)
        print(json.dumps(value, ensure_ascii=False), flush=True)
        return value['t']

    def check(name, condition, **evidence):
        value = {'name': name, 'passed': bool(condition), **evidence}
        result['checks'].append(value)
        if not condition:
            raise AssertionError(json.dumps(value, ensure_ascii=False))

    class Probe(Node):
        def __init__(self):
            super().__init__('startup_gazebo_acceptance',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.forward_scan = True
            self.raw_scans = self.forwarded_scans = 0
            self.last_odom = self.last_amcl = self.health = self.motion = None
            self.last_state = None
            self.last_state_at = 0.
            self.last_tf_record_at = {}
            self.map_message = None
            self.contact_messages = 0
            self.obstacle_truth = {}
            self.last_observed_at = -math.inf
            self.latest_raw_odom_tf_stamp = None
            self.amcl_process_pid = None
            self.last_process_sample_at = -math.inf
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')
            self.tf = Buffer(node=self)
            self.listener = TransformListener(self.tf, self)
            self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
            self.initial_pub = (None if args.scenario == 'home_initialization' else
                                self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10))
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            event_history = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(LaserScan, '/startup_validation/raw_scan', self.on_raw_scan,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, durable)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_truth, qos_profile_sensor_data)
            self.create_subscription(ContactsState, '/startup_validation/contacts', self.on_contacts,
                                     qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
            nav_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                     self.on_footprint, nav_qos)
            self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                     self.on_costmap, nav_qos)
            self.create_subscription(Int32MultiArray, '/smart_vision_amr/observed_obstacles',
                                     self.on_observed, 10)
            self.create_subscription(NavPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
            for topic, field, qos in [
                ('localization_health', 'health', durable), ('motion_status', 'motion', durable),
                ('scan_projection_status', 'projection', durable),
                ('failure_memory_status', 'memory', durable),
                ('navigation_status', 'navigation', 10), ('planner_status', 'planner', event_history)]:
                self.create_subscription(String, '/smart_vision_amr/'+topic,
                                         lambda msg, f=field: self.on_json(msg, f), qos)

        def on_raw_scan(self, message):
            self.raw_scans += 1
            if args.scenario in ('local_detour', 'barrier', 'navigation'):
                result['scan_receipts'].append({'t': elapsed(),
                    'receipt_ros_ns': self.get_clock().now().nanoseconds,
                    'stamp_ns': stamp_ns(message.header.stamp),
                    'latest_raw_odom_tf_stamp_ns': self.latest_raw_odom_tf_stamp,
                    'raw_count': self.raw_scans, 'relayed': self.forward_scan})
            if self.forward_scan:
                self.scan_pub.publish(message)
                self.forwarded_scans += 1

        def on_command(self, message):
            result['commands'].append({'t': elapsed(), 'v': message.linear.x, 'w': message.angular.z,
                                       'motion_status': self.motion})

        def on_odom(self, message):
            self.last_odom = message
            pose = message.pose.pose
            result['odom'].append({'t': elapsed(), 'stamp_ns': stamp_ns(message.header.stamp),
                                   'x': pose.position.x, 'y': pose.position.y,
                                   'yaw': yaw(pose.orientation), 'v': message.twist.twist.linear.x,
                                   'w': message.twist.twist.angular.z})

        def on_amcl(self, message):
            self.last_amcl = message
            covariance = message.pose.covariance
            cross = (covariance[1]+covariance[6])/2
            largest = (covariance[0]+covariance[7]+math.sqrt(
                (covariance[0]-covariance[7])**2+4*cross*cross))/2
            pose = message.pose.pose
            result['amcl'].append({'t': elapsed(), 'stamp_ns': stamp_ns(message.header.stamp),
                                  'x': pose.position.x, 'y': pose.position.y,
                                  'yaw': yaw(pose.orientation), 'position_std': math.sqrt(max(0., largest)),
                                  'yaw_std': math.sqrt(max(0., covariance[35])),
                                  'covariance': list(covariance)})

        def on_initial_pose(self, message):
            pose = message.pose.pose
            result['initial_poses'].append({'t': elapsed(),
                'stamp_ns': stamp_ns(message.header.stamp), 'x': pose.position.x,
                'y': pose.position.y, 'yaw': yaw(pose.orientation),
                'variance_x': message.pose.covariance[0],
                'variance_y': message.pose.covariance[7],
                'motion_status': self.motion})

        def on_truth(self, message):
            if 'waffle_pi' in message.name:
                pose = message.pose[message.name.index('waffle_pi')]
                result['ground_truth'].append({'t': elapsed(), 'x': pose.position.x,
                                              'y': pose.position.y, 'yaw': yaw(pose.orientation)})
            for obstacle in owned_obstacles:
                if obstacle in message.name:
                    pose = message.pose[message.name.index(obstacle)]
                    self.obstacle_truth[obstacle] = {'t': elapsed(), 'x': pose.position.x,
                                                      'y': pose.position.y, 'z': pose.position.z}

        def on_contacts(self, message):
            self.contact_messages += 1
            for state in message.states:
                if 'waffle_pi' in state.collision1_name or 'waffle_pi' in state.collision2_name:
                    result['robot_contacts'].append({'t': elapsed(),
                        'collision1': state.collision1_name, 'collision2': state.collision2_name})

        def on_map(self, message):
            self.map_message = message
            result['map_info'] = {'resolution': message.info.resolution,
                'width': message.info.width, 'height': message.info.height,
                'origin': [message.info.origin.position.x, message.info.origin.position.y,
                           yaw(message.info.origin.orientation)]}

        def on_observed(self, message):
            now = elapsed()
            if args.scenario in ('local_detour', 'barrier') and now-self.last_observed_at >= .1:
                result['observed_obstacle_samples'].append({'t': now,
                    'receipt_ros_ns': self.get_clock().now().nanoseconds, 'indices': list(message.data)})
                self.last_observed_at = now

        def navigation_input(self, message, kind, **geometry):
            if args.scenario not in ('local_detour', 'barrier', 'navigation'):
                return
            now_ns = self.get_clock().now().nanoseconds
            odom = self.last_odom
            result['navigation_input_samples'].append({'t': elapsed(), 'kind': kind,
                'receipt_ros_ns': now_ns, 'stamp_ns': stamp_ns(message.header.stamp),
                'header_age_seconds': (now_ns-stamp_ns(message.header.stamp))/1e9,
                'frame_id': message.header.frame_id,
                'latest_odom': None if odom is None else {
                    'stamp_ns': stamp_ns(odom.header.stamp), 'frame_id': odom.header.frame_id,
                    'x': odom.pose.pose.position.x, 'y': odom.pose.pose.position.y,
                    'yaw': yaw(odom.pose.pose.orientation)}, **geometry})

        def on_footprint(self, message):
            self.navigation_input(message, 'footprint',
                points=[[p.x, p.y, p.z] for p in message.polygon.points])

        def on_costmap(self, message):
            info = message.info
            self.navigation_input(message, 'costmap', resolution=info.resolution,
                width=info.width, height=info.height,
                origin=[info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation)])

        def on_path(self, message):
            result['path_events'].append({'t': elapsed(), 'points_count': len(message.poses),
                                          'stamp_ns': stamp_ns(message.header.stamp)})
            if message.poses:
                result['paths'].append({'t': elapsed(), 'stamp_ns': stamp_ns(message.header.stamp),
                                        'points': [[p.pose.position.x, p.pose.position.y]
                                                   for p in message.poses]})

        def on_tf(self, message):
            now = elapsed()
            for transform in message.transforms:
                pair = (transform.header.frame_id, transform.child_frame_id)
                if pair not in (('map', 'odom'), ('odom', 'base_footprint')):
                    continue
                stamp = stamp_ns(transform.header.stamp)
                if pair == ('odom', 'base_footprint'):
                    self.latest_raw_odom_tf_stamp = stamp
                if args.scenario in ('local_detour', 'barrier', 'navigation'):
                    result['raw_dynamic_tf_receipts'].append({'t': now,
                        'receipt_ros_ns': self.get_clock().now().nanoseconds,
                        'parent': pair[0], 'child': pair[1], 'stamp_ns': stamp,
                        'x': transform.transform.translation.x,
                        'y': transform.transform.translation.y,
                        'yaw': yaw(transform.transform.rotation)})
                if now-self.last_tf_record_at.get(pair, -math.inf) < .2:
                    continue
                self.last_tf_record_at[pair] = now
                result['tf'].append({'t': now, 'parent': pair[0], 'child': pair[1],
                                     'stamp_ns': stamp,
                                     'header_age_seconds': (self.get_clock().now().nanoseconds-stamp)/1e9})

        def on_json(self, message, field):
            try:
                value = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if not isinstance(value, dict):
                return
            if field in ('health', 'motion'):
                setattr(self, field, value)
            result[field].append({'t': elapsed(), **value})

        def state(self, force=False):
            nonlocal web_verified
            self.sample_owned_amcl_process()
            if force or time.monotonic()-self.last_state_at >= .2:
                _, state = api()
                if state['connection']['domain_id'] != DOMAIN:
                    raise RuntimeError('Web changed away from isolated domain')
                web_verified = True
                self.last_state, self.last_state_at = state, time.monotonic()
                result['states'].append({'t': elapsed(),
                    **{k: state.get(k) for k in ('connection', 'mission', 'localization', 'robot',
                                                'safety', 'localization_health', 'navigation', 'health',
                                                'marker_alignment', 'detection', 'delivery')},
                    'path_points': len(state.get('path', []))})
            return self.last_state

        def sample_owned_amcl_process(self):
            now = elapsed()
            if (args.scenario not in ('local_detour', 'barrier', 'navigation')
                    or args.simulation_launch_pid is None or now < 4
                    or now-self.last_process_sample_at < 1.):
                return
            self.last_process_sample_at = now
            try:
                if self.amcl_process_pid is None:
                    self.amcl_process_pid = own_amcl_pid()
                stat = Path(f'/proc/{self.amcl_process_pid}/stat').read_text().split(') ', 1)[1].split()
                ticks = os.sysconf('SC_CLK_TCK')
                threads = []
                for task in Path(f'/proc/{self.amcl_process_pid}/task').iterdir():
                    try:
                        detail = (task/'stat').read_text().split(') ', 1)[1].split()
                        threads.append({'tid': int(task.name), 'name': (task/'comm').read_text().strip(),
                            'state': detail[0], 'wchan': (task/'wchan').read_text().strip(),
                            'user_cpu_seconds': int(detail[11])/ticks,
                            'system_cpu_seconds': int(detail[12])/ticks})
                    except OSError:
                        continue
                result['owned_amcl_process_samples'].append({'t': now,
                    'pid': self.amcl_process_pid, 'state': stat[0],
                    'user_cpu_seconds': int(stat[11])/ticks,
                    'system_cpu_seconds': int(stat[12])/ticks,
                    'threads': int(stat[17]), 'start_ticks': int(stat[19]),
                    'wchan': Path(f'/proc/{self.amcl_process_pid}/wchan').read_text().strip(),
                    'thread_samples': threads,
                    'resident_bytes': int(stat[21])*os.sysconf('SC_PAGE_SIZE')})
            except (OSError, RuntimeError, ValueError, IndexError) as exc:
                result['owned_amcl_process_samples'].append({'t': now, 'error': repr(exc)})

        def wait(self, predicate, timeout, description):
            deadline = time.monotonic()+timeout
            next_report = time.monotonic()+10
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.03)
                state = self.state()
                if predicate(state):
                    return state
                if time.monotonic() >= next_report:
                    health = state.get('localization_health') or {}
                    metrics = health.get('metrics') or {}
                    event('waiting', for_condition=description,
                          mission_state=state.get('mission', {}).get('state'),
                          navigation=state.get('navigation'),
                          failure_memory=state.get('failure_memory'),
                          health={'level': health.get('level'), 'reasons': health.get('reasons'),
                                  **{k: metrics.get(k) for k in
                                     ('map_tf_age_seconds', 'odom_tf_age_seconds',
                                      'dynamic_tf_receive_age_seconds', 'scan_age_seconds',
                                      'odom_age_seconds')}},
                          safety={k: state.get('safety', {}).get(k) for k in
                                  ('mode', 'estop_latched', 'output_linear', 'output_angular')})
                    next_report = time.monotonic()+10
            latest = self.state(force=True)
            diagnostic = {key: latest.get(key) for key in
                          ('connection', 'mission', 'localization', 'localization_health', 'safety', 'navigation')}
            raise TimeoutError(description+'; latest='+json.dumps(diagnostic, ensure_ascii=False))

        def observe(self, seconds):
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=.03)
                self.state()

    def stamp_ns(stamp):
        return stamp.sec*1_000_000_000+stamp.nanosec

    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def route_key(state):
        return {key: state['mission'].get(key) for key in
                ('route', 'pending_route', 'current_index', 'completed_ids', 'home')}

    def zero_since(t):
        commands = [s for s in result['commands'] if s['t'] >= t]
        return bool(commands) and all(abs(s['v']) < 1e-5 and abs(s['w']) < 1e-5 for s in commands)

    def metric(key, default=None):
        return ((node.health or {}).get('metrics') or {}).get(key, default)

    def recoverable(state):
        health = state['localization_health']
        return (health.get('allow_localization') is True
                and not set(health.get('reasons', []))-{'operator_reset_required'}
                and health.get('stable_seconds', 0) >= 2.)

    def reset_safety():
        api('/api/emergency-stop', {'reset': True})
        return node.wait(lambda s: s['safety'].get('latched') is False
                         and s['safety'].get('estop_latched') is False, 10, 'explicit reset acknowledgment')

    def explicit_restart_after_fault(description):
        state = node.state(force=True)
        # The existing shared ESTOP reset deliberately cancels a mission.
        # PAUSED can resume; CANCELED requires a new explicit start of the
        # preserved route. Neither operation follows sensor recovery itself.
        endpoint = '/api/mission/resume' if state['mission']['state'] == 'PAUSED' else '/api/mission/start'
        event(description, mission_state=state['mission']['state'], endpoint=endpoint)
        api(endpoint, {})

    def own_amcl_pid():
        if args.simulation_launch_pid is None:
            raise RuntimeError('Own simulation launch PID required for TF fault test')
        candidates = []
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                pid = int(process.name)
                command = (process/'cmdline').read_bytes().split(b'\0')
                environment = (process/'environ').read_bytes().split(b'\0')
                if (command and command[0].endswith(b'/nav2_amcl/amcl')
                        and b'ROS_DOMAIN_ID=186' in environment
                        and os.getpgid(pid) == args.simulation_launch_pid):
                    candidates.append(pid)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
        if len(candidates) != 1:
            raise RuntimeError('Cannot identify exactly one owned AMCL: '+repr(candidates))
        return candidates[0]

    def service(future, description, timeout=10):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            node.state()
        if not future.done():
            raise TimeoutError(description)
        return future.result()

    def dynamic_obstacle():
        def guarded(state, condition):
            if state.get('safety', {}).get('estop_latched') is True:
                raise RuntimeError('Dynamic obstacle trial stopped by safety guard: '+json.dumps(
                    state.get('localization_health'), ensure_ascii=False))
            memory = state.get('failure_memory') or {}
            if (args.scenario == 'local_detour' and memory.get('state') == 'OBSTACLE_HOLD'
                    and memory.get('retry_count', 0) >= memory.get('retry_limit', math.inf)):
                raise RuntimeError('Local bypass exhausted retries with the obstacle still present: '
                                   +json.dumps(memory, ensure_ascii=False))
            return condition
        node.wait(lambda s: s['localization_health'].get('autonomous_allowed') is True
                  and node.map_message is not None and node.spawn.service_is_ready(),
                  45, 'actual dynamic-obstacle fixture readiness')
        reset_safety()
        api('/api/mission', {'route': [1]})
        map_hash = hashlib.sha256(bytes(value+1 for value in node.map_message.data)).hexdigest()
        t = event('explicit_dynamic_obstacle_goal')
        api('/api/mission/start', {})
        anchor = result['ground_truth'][-1]
        node.wait(lambda s: guarded(s, bool(result['paths']) and bool(result['ground_truth'])
                  and math.hypot(result['ground_truth'][-1]['x']-anchor['x'],
                                 result['ground_truth'][-1]['y']-anchor['y']) >= .25),
                  25, 'actual motion before sudden obstacle insertion')
        original = result['paths'][-1]['points']
        goal_stamp = next(p['goal_stamp'] for p in reversed(result['planner'])
                          if p['t'] >= t and p.get('goal_stamp') is not None)
        check('current_goal_has_real_planner_stamp', isinstance(goal_stamp, dict)
              and set(goal_stamp) == {'sec', 'nanosec'}, goal_stamp=goal_stamp)
        retained_route = route_key(node.state(force=True))
        global_before = max((p.get('global_plan_count', 0) for p in result['planner']), default=0)
        local_before = max((p.get('local_attempt_id', 0) for p in result['planner']), default=0)
        box_xy = (-1.5, -2.)
        dimensions = (.45, .50, 1.) if args.scenario == 'local_detour' else (.25, 2.2, 1.)
        info = node.map_message.info
        origin_yaw = yaw(info.origin.orientation)
        dx, dy = box_xy[0]-info.origin.position.x, box_xy[1]-info.origin.position.y
        cell_x = math.floor((math.cos(origin_yaw)*dx+math.sin(origin_yaw)*dy)/info.resolution)
        cell_y = math.floor((-math.sin(origin_yaw)*dx+math.cos(origin_yaw)*dy)/info.resolution)
        check('inserted_obstacle_center_was_known_free_in_original_map',
              0 <= cell_x < info.width and 0 <= cell_y < info.height
              and node.map_message.data[cell_y*info.width+cell_x] == 0)
        name = 'startup_validation_'+uuid.uuid4().hex[:12]
        request = SpawnEntity.Request()
        request.name = name
        sx, sy, sz = dimensions
        request.xml = f'''<sdf version="1.6"><model name="{name}"><static>true</static><link name="body">
          <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
          <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
          <sensor type="contact" name="bumper"><always_on>true</always_on><update_rate>50</update_rate>
          <contact><collision>collision</collision></contact><plugin name="contact_sensor" filename="libgazebo_ros_bumper.so">
          <ros><namespace>/startup_validation</namespace><remapping>bumper_states:=contacts</remapping></ros>
          <frame_name>world</frame_name></plugin></sensor></link></model></sdf>'''
        request.initial_pose.position.x, request.initial_pose.position.y = box_xy
        request.initial_pose.position.z, request.initial_pose.orientation.w = sz/2, 1.
        inserted_ros_ns = node.get_clock().now().nanoseconds
        # Sensor/planner callbacks may observe the inserted model before the
        # spawn RPC response reaches this recorder. Retain that whole bracket.
        inserted = event('gazebo_obstacle_spawn_requested', obstacle_name=name,
                         request_ros_ns=inserted_ros_ns)
        spawned = service(node.spawn.call_async(request), 'spawn actual collision/LiDAR box')
        check('actual_gazebo_obstacle_spawned', spawned.success, message=spawned.status_message)
        owned_obstacles.append(name)
        event('actual_obstacle_inserted', obstacle_name=name, xy=box_xy, size=dimensions,
              spawn_requested_at=inserted, goal_stamp=goal_stamp, robot=result['ground_truth'][-1])
        removal = None
        if args.scenario == 'barrier':
            def fresh_goal(packet):
                return (packet['t'] >= inserted and packet.get('goal_stamp') == goal_stamp
                        and packet.get('sent_at_ns', 0) >= inserted_ros_ns)
            def local_failed(packet):
                prior_active = any(fresh_goal(p) and p['t'] <= packet['t']
                                   and p.get('plan_kind') == 'LOCAL_DETOUR'
                                   and p.get('local_detour_state') == 'ACTIVE'
                                   for p in result['planner'])
                reason = packet.get('detour_reason')
                return (fresh_goal(packet) and packet.get('local_attempt_id', 0) > local_before
                        and packet.get('local_detour_state') == 'FAILED'
                        and (reason in (
                            'no_local_route', 'no_forward_rejoin', 'time_budget',
                            'expansion_budget', 'no_safe_anchor')
                             or (reason in ('active_detour_obstructed', 'attempt_budget')
                                 and prior_active)))
            def global_failed(packet):
                return (fresh_goal(packet) and packet.get('global_plan_count', 0) > global_before
                        and packet.get('plan_kind') == 'GLOBAL'
                        and packet.get('status') == 'WAIT_REPLAN'
                        and 'No safe A* route' in packet.get('message', ''))
            node.wait(lambda s: guarded(s, any(local_failed(p) for p in result['planner'])
                      and any(global_failed(p) for p in result['planner'])
                      and bool(result['memory'])
                      and result['memory'][-1].get('state') == 'OBSTACLE_HOLD'
                      and result['memory'][-1].get('goal_stamp') == goal_stamp
                      and abs(s['robot']['speed']['linear']) < .01),
                      60, 'full-width barrier bounded-local failure, actual global no-path and stopped hold')
            failed_local = next(p for p in result['planner'] if local_failed(p))
            failed_global = next(p for p in result['planner'] if global_failed(p))
            check('barrier_exhausts_bounded_local_then_uses_global_fallback',
                  failed_local['t'] <= failed_global['t']
                  and 0 <= failed_local.get('expansions', math.inf) <= 1500,
                  local_failure=failed_local, global_failure=failed_global)
            hold = event('full_barrier_hold_observed')
            stopped_pose = result['ground_truth'][-1]
            node.observe(3)
            held_truth = [p for p in result['ground_truth'] if p['t'] >= hold]
            held_odom = [p for p in result['odom'] if p['t'] >= hold+.3]
            check('full_barrier_holds_zero_without_goal_loss', zero_since(hold+.3)
                  and route_key(node.state()) == retained_route
                  and result['memory'][-1].get('goal_stamp') == goal_stamp
                  and len(held_truth) >= 10 and held_truth[-1]['t']-held_truth[0]['t'] >= 2.5
                  and elapsed()-held_truth[-1]['t'] <= .25
                  and len(held_odom) >= 10 and elapsed()-held_odom[-1]['t'] <= .25
                  and all(abs(p['v']) < .01 and abs(p['w']) < .02 for p in held_odom)
                  and all(math.hypot(p['x']-stopped_pose['x'], p['y']-stopped_pose['y']) < .02
                          and abs(math.atan2(math.sin(p['yaw']-stopped_pose['yaw']),
                                             math.cos(p['yaw']-stopped_pose['yaw']))) < .02
                          for p in held_truth), held_ground_truth_samples=len(held_truth),
                  held_odometry_samples=len(held_odom))
            deletion = DeleteEntity.Request()
            deletion.name = name
            response = service(node.delete.call_async(deletion), 'remove only owned full-width barrier')
            check('actual_barrier_removed_after_stopped_hold', response.success)
            owned_obstacles.remove(name)
            removal = event('actual_barrier_removed')
        else:
            def fresh_local(packet):
                return (packet['t'] >= inserted and packet.get('plan_kind') == 'LOCAL_DETOUR'
                        and packet.get('local_detour_state') == 'ACTIVE'
                        and packet.get('goal_stamp') == goal_stamp
                        and packet.get('local_attempt_id', 0) > local_before
                        and packet.get('sent_at_ns', 0) >= inserted_ros_ns)
            def fresh_path(packet):
                return packet['t'] >= inserted and packet['stamp_ns'] >= inserted_ros_ns
            node.wait(lambda s: guarded(s, any(fresh_local(p) for p in result['planner'])),
                      35, 'bounded local detour selected before full global search')
            chosen = next(p for p in result['planner'] if fresh_local(p))
            check('local_detour_precedes_global_replan',
                  chosen.get('global_plan_count') == global_before, selection=chosen,
                  global_plan_count_before_obstacle=global_before)
            node.wait(lambda s: any(fresh_path(p) for p in result['paths']),
                      5, 'actual nonempty locally patched path publication')
            patched = next(p['points'] for p in result['paths'] if fresh_path(p))
            first_patch_at = next(p['t'] for p in result['paths'] if fresh_path(p))
            check('local_patch_published_without_empty_path_barrier', not any(
                  inserted <= publication['t'] <= first_patch_at and publication['points_count'] == 0
                  for publication in result['path_events']))
            suffix = 0
            for before, after in zip(reversed(original), reversed(patched)):
                if math.hypot(before[0]-after[0], before[1]-after[1]) > 1e-6:
                    break
                suffix += 1
            check('local_patch_retains_original_goal_and_nontrivial_suffix', suffix >= 5
                  and patched[-1] == original[-1], exact_shared_suffix_points=suffix,
                  suffix_length_lower_bound_m=.05*max(0,suffix-1))
            prefix = patched[:len(patched)-suffix+1]
            prefix_length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(prefix,prefix[1:]))
            check('local_repaired_prefix_is_short_and_search_bounded',
                  prefix_length <= 4.05 and 0 < chosen.get('expansions', math.inf) <= 1500,
                  geometric_prefix_length_m=prefix_length,
                  configured_grid_search_max_length_m=4.,
                  search_duration_ms=chosen.get('search_duration_ms'), expansions=chosen.get('expansions'))
        state = node.wait(lambda s: guarded(s, any(n['t'] >= t and n.get('status') == 'SUCCEEDED'
                          and n.get('goal_stamp') == goal_stamp for n in result['navigation'])),
                          160, 'actual same-goal arrival after dynamic obstacle handling')
        obstacle_truth = [sample for sample in result['ground_truth']
                          if sample['t'] >= inserted and (removal is None or sample['t'] <= removal)]
        # A 0.24m disk encloses the stock SDF base box (offset -.064m), wheels
        # and casters. This is a conservative sampled footprint check.
        def clearance(sample):
            dx = max(abs(sample['x']-box_xy[0])-sx/2, 0.)
            dy = max(abs(sample['y']-box_xy[1])-sy/2, 0.)
            return math.hypot(dx, dy)-.24
        minimum = min(map(clearance, obstacle_truth)) if obstacle_truth else -math.inf
        check('actual_trajectory_footprint_clear_and_no_contact', minimum > 0
              and node.contact_messages > 0 and not result['robot_contacts'], conservative_disk_radius_m=.24,
              minimum_sampled_obstacle_clearance_m=minimum,
              contact_messages=node.contact_messages, robot_contacts=result['robot_contacts'])
        def static_clearance(sample):
            x, y = sample['x'], sample['y']
            distances = [5-abs(x), 4-abs(y)]
            if args.scenario == 'barrier':
                distances += [y-(-3.05), -.95-y]
            for rx, ry, hx, hy in ((0., 0., .6, 1.), (-2.5, 1.7, .8, .4), (2.5, -1.7, .8, .4)):
                distances.append(math.hypot(max(abs(x-rx)-hx, 0.), max(abs(y-ry)-hy, 0.)))
            return min(distances)-.24
        wall_clearance = min(static_clearance(sample) for sample in result['ground_truth'] if sample['t'] >= t)
        check('actual_sampled_footprint_clear_of_room_walls_and_furniture', wall_clearance > 0,
              minimum_sampled_static_clearance_m=wall_clearance)
        check('original_map_unchanged_by_dynamic_fixture', map_hash == hashlib.sha256(
              bytes(value+1 for value in node.map_message.data)).hexdigest(), map_sha256=map_hash)
        target = next(marker for marker in state['markers'] if marker['id'] == 1)
        actual = result['ground_truth'][-1]
        distance = math.hypot(actual['x']-target['x'], actual['y']-target['y'])
        if removal is None:
            box_truth = node.obstacle_truth.get(name) or {}
            check('actual_obstacle_still_present_at_goal_arrival',
                  elapsed()-box_truth.get('t', -math.inf) < .25
                  and math.hypot(box_truth.get('x', math.inf)-box_xy[0],
                                 box_truth.get('y', math.inf)-box_xy[1]) < .01,
                  actual_obstacle_pose=box_truth)
            rejoin = chosen['rejoin_point']
            rejoin_distance = min(math.hypot(p['x']-rejoin['x'], p['y']-rejoin['y'])
                                  for p in result['ground_truth'] if p['t'] >= chosen['t'])
            check('actual_body_rejoins_preserved_forward_suffix', rejoin_distance <= .35,
                  closest_actual_body_distance_to_rejoin_m=rejoin_distance,
                  selected_rejoin_point=rejoin,
                  global_plan_count_at_arrival=max(p.get('global_plan_count', 0) for p in result['planner']))
        check('same_goal_actual_arrival_with_obstacle_policy', distance <= .35,
              actual_gazebo_goal_distance_m=distance, obstacle_present_at_arrival=removal is None)
        if removal is not None:
            moving = [command for command in result['commands'] if command['t'] >= removal
                      and (abs(command['v']) > .01 or abs(command['w']) > .05)]
            result['barrier_removal_to_motion_seconds'] = None if not moving else moving[0]['t']-removal
        event('dynamic_obstacle_scenario_succeeded', actual_goal_distance_m=distance,
              minimum_sampled_clearance_m=minimum)
        api('/api/emergency-stop', {})
        return True

    def verify_amcl_tf_override():
        def endpoint(info):
            qos = info.qos_profile
            return {'node_name': info.node_name, 'node_namespace': info.node_namespace,
                'gid': list(info.endpoint_gid), 'reliability': qos.reliability.name,
                'durability': qos.durability.name, 'history': qos.history.name, 'depth': qos.depth}
        snapshot = {}
        def ready(_state):
            nonlocal snapshot
            snapshot = {topic+' '+kind: [endpoint(info) for info in getter(topic)]
                for topic, kind, getter in (
                    ('/amcl_pose', 'publishers', node.get_publishers_info_by_topic),
                    ('/tf', 'publishers', node.get_publishers_info_by_topic),
                    ('/tf', 'subscribers', node.get_subscriptions_info_by_topic),
                    ('/tf_static', 'subscribers', node.get_subscriptions_info_by_topic),
                    ('/controller_server/transition_event', 'publishers', node.get_publishers_info_by_topic),
                    ('/behavior_server/transition_event', 'publishers', node.get_publishers_info_by_topic))}
            result['tf_endpoint_snapshot'] = snapshot
            groups = [('amcl', '/amcl_pose publishers')]
            if args.require_nav2_tf_best_effort:
                groups += [('controller_server', '/controller_server/transition_event publishers'),
                           ('behavior_server', '/behavior_server/transition_event publishers')]
            for name, topic in groups:
                prefixes = {tuple(info['gid'][:12]) for info in snapshot[topic]
                            if info['node_name'] == name}
                dynamic = [info for info in snapshot['/tf subscribers']
                           if tuple(info['gid'][:12]) in prefixes]
                static = [info for info in snapshot['/tf_static subscribers']
                          if tuple(info['gid'][:12]) in prefixes]
                if not (prefixes and dynamic and static
                        and all(info['reliability'] == 'BEST_EFFORT'
                                and info['depth'] in (0, 20) for info in dynamic)
                        and all(info['reliability'] == 'RELIABLE'
                                and info['durability'] == 'TRANSIENT_LOCAL' for info in static)):
                    return False
            return True
        node.wait(ready, 12, 'actual AMCL participant hidden dynamic TF BEST_EFFORT depth20 and unchanged static TF')
        records = []
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                if os.getpgid(int(process.name)) != args.simulation_launch_pid:
                    continue
                argv = (process/'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
                if Path(argv[0]).name not in ('amcl', 'controller_server', 'behavior_server'):
                    continue
                if b'ROS_DOMAIN_ID=186' not in (process/'environ').read_bytes().split(b'\0'):
                    continue
                records.append({'pid': int(process.name), 'pgid': args.simulation_launch_pid,
                    'argv': argv, 'domain': DOMAIN, 'wall_monotonic_ns': time.monotonic_ns()})
            except (OSError, ValueError, IndexError):
                continue
        args.output.with_name('owned_nav2_process_identity.json').write_text(
            json.dumps(records, indent=2)+'\n')
        expected_names = {'amcl', 'controller_server', 'behavior_server'}
        check('owned_nav2_process_arguments_recorded_after_readiness',
              {Path(record['argv'][0]).name for record in records} == expected_names)
        check('nav2_dynamic_tf_reliability_and_static_policy_verified_before_motion', True,
              endpoints=snapshot, configured_dynamic_depth=20, actual_dynamic_depth=None,
              graph_depth_note='Humble rmw graph reports UNKNOWN history/depth0. Depth20 is configured in the validated process launch arguments; actual depth is not observable here. Hidden listener parameter services are disabled.')
        event('amcl_tf_qos_override_verified', configured_dynamic_depth=20,
              actual_dynamic_depth=None, actual_dynamic_reliability='BEST_EFFORT',
              actual_static_policy='RELIABLE/TRANSIENT_LOCAL')

    def assisted(allow_operator_retry=False, startup_home=False):
        prior = route_key(node.state(force=True))
        endpoint = '/api/localization/start-home' if startup_home else '/api/localization/start'
        t = event('assisted_start_home' if startup_home else 'assisted_start', endpoint=endpoint)
        payload = {'confirm_actual_home': True} if startup_home else {}
        api(endpoint, payload)
        status, body = api(endpoint, payload, allow_error=True)
        check('repeated_assisted_start_rejected', status == 400, response=body)
        # A retry follows a previous terminal state; never reuse that cached
        # HTTP snapshot as the result of the newly accepted operation.
        node.state(force=True)
        node.wait(lambda s: s['localization'].get('active') is True,
                  5, 'new assisted operation becomes active')
        state = node.wait(lambda s: s['localization']['state'] in ('SUCCEEDED', 'FAILED', 'CANCELED'),
                          85, 'real AMCL assisted-localization completion')
        if state['localization']['state'] == 'FAILED' and allow_operator_retry:
            failed_at = event('assisted_safe_failure_requires_operator_retry',
                              localization=state['localization'], health=state['localization_health'])
            result['required_operator_retry'] = True
            node.observe(.5)
            check('failed_assisted_operation_stays_stopped_and_preserves_goal',
                  zero_since(failed_at+.15) and prior == route_key(node.state()))
            node.wait(recoverable, 20, 'fresh TF/sensors stable before explicit assisted retry')
            if node.state(force=True)['safety'].get('estop_latched'):
                reset_safety()
            event('explicit_operator_assisted_retry_after_stable_tf')
            return assisted(allow_operator_retry=False, startup_home=False)
        check('assisted_succeeded', state['localization']['state'] == 'SUCCEEDED',
              localization=state['localization'])
        samples = [s for s in result['commands'] if s['t'] >= t]
        nonzero = [s for s in samples if abs(s['w']) > .01]
        check('assisted_actual_rotation_command', bool(nonzero))
        if startup_home:
            priors = [sample for sample in result['initial_poses'] if sample['t'] >= t]
            check('home_prior_after_mux_hold_ack_before_rotation', len(priors) == 1
                  and priors[0]['t'] < nonzero[0]['t']
                  and (priors[0]['motion_status'] or {}).get('mode') == 'hold'
                  and (priors[0]['motion_status'] or {}).get('stop_acknowledged') is True,
                  priors=priors)
        check('assisted_zero_linear_and_low_angular', bool(samples)
              and all(abs(s['v']) < 1e-5 and abs(s['w']) <= .25001 for s in samples),
              max_linear=max(abs(s['v']) for s in samples),
              max_angular=max(abs(s['w']) for s in samples))
        check('assisted_exclusive_mux_lease', all(
            (s['motion_status'] or {}).get('mode') == 'localization'
            and (s['motion_status'] or {}).get('source_id')
            and (s['motion_status'] or {}).get('available') is True for s in nonzero))
        odom = [s for s in result['odom'] if s['t'] >= t]
        check('assisted_real_gazebo_rotation', any(abs(s['w']) > .08 for s in odom))
        check('assisted_preserves_route', prior == route_key(state), before=prior, after=route_key(state))
        settled = event('assisted_finished', localization=state['localization'])
        node.observe(2)
        check('assisted_does_not_auto_start_mission', zero_since(settled)
              and node.state()['mission']['state'] not in ('NAVIGATING', 'RETURNING_HOME'))

    rclpy.init(args=[])
    node = Probe()
    try:
        state = node.wait(lambda s: s['connection']['connected'] and node.last_odom is not None
                          and node.raw_scans > 3 and s['localization_health'].get('level') != 'unknown'
                          and (args.scenario == 'home_initialization' or node.last_amcl is not None)
                          and metric('map_available', False) is True and s.get('map') is not None
                          and s['safety'].get('stop_acknowledged') is True,
                          100, 'active AMCL/map, actual sensors and latched stop acknowledgment')
        if args.require_amcl_tf_best_effort or args.require_nav2_tf_best_effort:
            verify_amcl_tf_override()
        if args.scenario in ('local_detour', 'barrier'):
            result['passed'] = dynamic_obstacle()
            return 0
        if args.scenario == 'home_initialization':
            client = node.create_client(GetParameters, '/amcl/get_parameters')
            node.wait(lambda s: client.service_is_ready(), 5, 'active AMCL parameter service')
            request = GetParameters.Request()
            request.names = ['set_initial_pose']
            response = service(client.call_async(request), 'read actual AMCL set_initial_pose', 5)
            check('home_initialization_actual_amcl_set_initial_pose_is_false',
                  len(response.values) == 1 and response.values[0].type == 1
                  and response.values[0].bool_value is False,
                  actual_set_initial_pose=response.values[0].bool_value if response.values else None)
            check('home_initialization_keeps_startup_estop_until_explicit_reset',
                  state['safety'].get('estop_latched') is True)
            node.observe(2)
            map_tf_available = node.tf.can_transform('map', 'odom', Time())
            check('home_initialization_boots_without_amcl_prior_or_map_tf',
                  node.last_amcl is None and not result['initial_poses']
                  and not any(t['parent'] == 'map' for t in result['tf'])
                  and not map_tf_available and node.initial_pub is None,
                  amcl_messages=len(result['amcl']), initial_pose_messages=len(result['initial_poses']),
                  map_tf_messages=sum(t['parent'] == 'map' for t in result['tf']),
                  map_tf_lookup_available=map_tf_available, verifier_initial_pose_publisher=False)
            rejected_at = elapsed()
            status, body = api('/api/localization/start-home', {'confirm_actual_home': True},
                               allow_error=True)
            node.observe(.3)
            check('home_initialization_does_not_automatically_reset_estop', status == 400
                  and node.state()['safety'].get('estop_latched') is True
                  and not node.state()['localization'].get('active')
                  and zero_since(rejected_at), response=body)
            node.wait(lambda s: s['localization'].get('home_seed_available') is True,
                      30, 'independent registered-Home LiDAR/map match while ESTOP remains latched')
            seed_started = event('confirmed_home_seed_while_estop_latched_without_map_tf')
            status, body = api('/api/localization/seed-home', {}, allow_error=True)
            node.observe(.3)
            check('home_seed_requires_actual_home_confirmation', status == 400
                  and not result['initial_poses'], response=body)
            api('/api/localization/seed-home', {'confirm_actual_home': True})
            state = node.wait(lambda s: len(result['initial_poses']) == 1
                              and node.last_amcl is not None and recoverable(s),
                              55, 'confirmed Home prior produces actual AMCL/map TF and stable reset readiness')
            check('confirmed_home_seed_is_only_pre_reset_pose_source_and_keeps_stop',
                  len(result['initial_poses']) == 1 and zero_since(seed_started)
                  and state['safety'].get('estop_latched') is True
                  and state['localization'].get('active') is not True,
                  initial_pose=result['initial_poses'][0], health=state['localization_health'])
            event('home_seed_established_real_amcl_and_tf_with_estop_still_latched')
            reset_safety()
            api('/api/mission', {'route': [1]})
            node.observe(.5)
            rejected_at = elapsed()
            status, body = api('/api/localization/start-home', {}, allow_error=True)
            node.observe(.3)
            check('home_initialization_requires_actual_home_confirmation', status == 400
                  and not any(p['t'] >= rejected_at for p in result['initial_poses'])
                  and not node.state()['localization'].get('active'), response=body)
            started = elapsed()
            assisted(startup_home=True)
            state = node.wait(lambda s: s.get('delivery', {}).get('home_ready') is True
                              and s['localization_health'].get('autonomous_allowed') is True,
                              5, 'Home heading is accepted by actual dispatch readiness')
            truth = [p for p in result['ground_truth'] if p['t'] >= started]
            check('home_initialization_has_fresh_actual_ground_truth', bool(truth)
                  and elapsed()-truth[-1]['t'] <= .5)
            latest = truth[-1]
            home = state['delivery']['registered_homepoint']
            priors = [p for p in result['initial_poses'] if p['t'] >= started]
            check('home_initialization_publishes_registered_home_pose_without_rviz',
                  len(priors) == 1 and all(abs(priors[0][key]-home[key]) <= 1e-9
                                          for key in ('x', 'y', 'yaw')),
                  initial_pose=priors, registered_home=home)
            excursion = max(abs(math.atan2(math.sin(p['yaw']-home['yaw']),
                                           math.cos(p['yaw']-home['yaw']))) for p in truth)
            check('home_initialization_really_sweeps_then_returns',
                  excursion >= math.radians(80)
                  and any(p['w'] > .08 for p in result['odom'] if p['t'] >= started)
                  and any(p['w'] < -.08 for p in result['odom'] if p['t'] >= started),
                  actual_maximum_heading_excursion_degrees=math.degrees(excursion))
            error = abs(math.atan2(math.sin(latest['yaw']-home['yaw']),
                                   math.cos(latest['yaw']-home['yaw'])))
            check('home_initialization_returns_actual_body_to_registered_heading',
                  error <= .10
                  and math.hypot(latest['x']-home['x'], latest['y']-home['y']) <= .30,
                  ground_truth=latest, registered_home=home,
                  actual_heading_error_degrees=math.degrees(error))
            check('home_initialization_removes_home_dispatch_gate',
                  state['delivery']['home_ready'] is True
                  and not any('Home' in str(reason) or '홈' in str(reason)
                              for reason in state['delivery'].get('dispatch_missing', [])),
                  delivery=state['delivery'])
            event('registered_home_initialization_succeeded', delivery=state['delivery'],
                  actual_heading_error_degrees=math.degrees(error), ground_truth=latest)
            result['passed'] = True
            return 0
        if args.scenario == 'precision':
            node.wait(lambda s: s['localization_health'].get('autonomous_allowed') is True,
                      45, 'precision fixture localization health')
            reset_safety()
            api('/api/mission', {'route': [1]})
            with urllib.request.urlopen(URL+'/api/camera.jpg', timeout=3) as response:
                args.output.with_name('rear_camera_initial.jpg').write_bytes(response.read())
            precision_start = event('rear_wall_precision_mission_start')
            api('/api/mission/start', {})
            state = node.wait(lambda s: s.get('marker_alignment', {}).get('state') == 'CONFIRMED',
                              80, 'real rear camera and LiDAR wall alignment confirmation')
            truth = [sample for sample in result['ground_truth'] if sample['t'] >= precision_start]
            check('actual_gazebo_ground_truth_available', bool(truth))
            latest = truth[-1]
            wall_error = abs(math.atan2(math.sin(latest['yaw']), math.cos(latest['yaw'])))
            alignment = state['marker_alignment']
            check('rear_camera_faces_marker_with_body_perpendicular_to_wall',
                  alignment.get('camera') == 'rear' and wall_error <= math.radians(3),
                  actual_body_yaw_radians=latest['yaw'], wall_normal_error_degrees=math.degrees(wall_error),
                  alignment=alignment)
            check('strict_marker_image_center_and_measured_stop',
                  abs(alignment.get('center_error', math.inf)) <= .05
                  and abs(state['robot']['speed']['linear']) < .025
                  and abs(state['robot']['speed']['angular']) < .06,
                  center_error=alignment.get('center_error'), speed=state['robot']['speed'])
            commands = [sample for sample in result['commands'] if sample['t'] >= precision_start]
            check('precision_real_rotation_through_existing_mux',
                  any(abs(sample['w']) > .02 for sample in commands)
                  and all(abs(sample['v']) < .00001 for sample in commands),
                  max_command_angular=max(abs(sample['w']) for sample in commands))
            try:
                with urllib.request.urlopen(URL+'/api/camera.jpg', timeout=3) as response:
                    args.output.with_name('rear_camera_confirmed.jpg').write_bytes(response.read())
            finally:
                api('/api/emergency-stop', {})
            event('rear_wall_precision_confirmed', alignment=alignment, ground_truth=latest)
            result['passed'] = True
            return 0
        if args.scenario != 'navigation':
            check('startup_stop_latched', state['safety'].get('estop_latched') is True)
            t = event('legitimate_initialpose_high_covariance')
            request = PoseWithCovarianceStamped()
            request.header.frame_id, request.header.stamp = 'map', node.get_clock().now().to_msg()
            request.pose.pose.position.x, request.pose.pose.position.y = -3., -2.
            request.pose.pose.orientation.w = 1.
            request.pose.covariance[0] = request.pose.covariance[7] = args.initial_std**2
            request.pose.covariance[35] = .15**2
            initial_count = metric('amcl_update_requests', 0)
            node.initial_pub.publish(request)
            state = node.wait(lambda s: s['localization_health'].get('autonomous_allowed') is True
                              and metric('amcl_update_requests', 0) > initial_count
                              and len([sample for sample in result['amcl'] if sample['t'] >= t]) >= 2,
                              55, 'stationary actual AMCL covariance convergence')
            samples = [s for s in result['amcl'] if s['t'] >= t]
            check('real_amcl_updates_after_high_covariance_seed', len(samples) >= 2,
                  samples=len(samples), position_std=[s['position_std'] for s in samples],
                  requests=metric('amcl_update_requests'),
                  responses=metric('amcl_update_responses'))
            check('stationary_convergence_never_moves', zero_since(t))
            check('stationary_convergence_keeps_startup_latch', state['safety'].get('estop_latched') is True)
            check('stationary_converged_without_threshold_relaxation',
                  state['localization_health'].get('metrics', {}).get('position_std_m', math.inf) <= .35,
                  health=state['localization_health'])
            event('stationary_convergence_complete', health=state['localization_health'])
            if args.scenario == 'startup':
                result['passed'] = True
                return 0
            reset_safety()
            api('/api/mission', {'route': [1]})
            node.observe(.5)
            assisted(startup_home=True)
        else:
            if state['safety'].get('estop_latched'):
                node.wait(lambda s: s['localization_health'].get('autonomous_allowed') is True,
                          45, 'localization healthy for explicit reset')
                reset_safety()
            api('/api/mission', {'route': [1]})

        t = event('explicit_mission_start')
        api('/api/mission/start', {})
        node.wait(lambda s: any(abs(c['v']) > .02 for c in result['commands'] if c['t'] > t)
                  and any(p['t'] > t for p in result['paths']), 35, 'path and actual motion')
        before_fault = route_key(node.state(force=True))
        if args.scenario == 'suite':
            fault = event('scan_relay_suppressed', raw_scan_count=node.raw_scans,
                          forwarded_scan_count=node.forwarded_scans)
            node.forward_scan = False
            state = node.wait(lambda s: s['safety'].get('estop_latched') is True, 8, 'scan outage safety latch')
            stopped = [c for c in result['commands'] if c['t'] >= fault and abs(c['v']) < 1e-5 and abs(c['w']) < 1e-5]
            check('stale_scan_stops_within_one_second', bool(stopped) and stopped[0]['t']-fault <= 1.,
                  stop_latency_seconds=None if not stopped else stopped[0]['t']-fault)
            raw_before_restore, sent_before_restore = node.raw_scans, node.forwarded_scans
            node.observe(.5)
            check('fault_is_scan_only_genuine_source_continues', node.raw_scans > raw_before_restore
                  and node.forwarded_scans == sent_before_restore)
            restored = event('scan_relay_restored')
            node.forward_scan = True
            node.wait(recoverable,
                      20, 'scan restoration healthy localization')
            node.observe(2)
            check('scan_restore_does_not_auto_resume', zero_since(restored+.25)
                  and node.state()['safety'].get('estop_latched') is True)
            check('control_failure_preserves_mission_route', route_key(node.state()) == before_fault,
                  before=before_fault, after=route_key(node.state()))
            reset_safety()
            assisted(allow_operator_retry=True)
            explicit_restart_after_fault('explicit_mission_restart_after_scan_fault')
            resumed = elapsed()
            node.wait(lambda s: any(c['t'] >= resumed and abs(c['v']) > .02 for c in result['commands']),
                      25, 'actual resumed navigation before TF outage')
            pid = own_amcl_pid()
            tf_fault = event('own_amcl_frozen_tf_outage', pid=pid)
            os.kill(pid, signal.SIGSTOP)
            try:
                state = node.wait(lambda s: s['safety'].get('estop_latched') is True,
                                  8, 'stale map TF safety latch')
                check('tf_outage_blocks_and_latches_stop',
                      any(reason in ('map_tf_stale', 'map_tf_unavailable', 'map_tf_invalid')
                          for health in result['health'] if health['t'] >= tf_fault
                          for reason in health.get('reasons', [])),
                      health=state['localization_health'])
            finally:
                os.kill(pid, signal.SIGCONT)
                tf_restored = event('own_amcl_restored')
            node.wait(recoverable, 20, 'AMCL/TF restoration healthy localization')
            node.observe(2)
            check('tf_restore_does_not_auto_resume', zero_since(tf_restored+.25)
                  and node.state()['safety'].get('estop_latched') is True)
            check('tf_failure_preserves_mission_route', route_key(node.state()) == before_fault,
                  before=before_fault, after=route_key(node.state()))
            reset_safety()
            explicit_restart_after_fault('explicit_mission_restart_after_tf_fault')

        state = node.wait(lambda s: any(n.get('status') == 'SUCCEEDED' and n['t'] >= t
                                       for n in result['navigation']), 120, 'actual Nav2 goal success')
        pose = state['robot'].get('pose') or {}
        target = next(marker for marker in state['markers'] if marker['id'] == 1)
        distance = math.hypot(pose.get('x', math.inf)-target['x'], pose.get('y', math.inf)-target['y'])
        odom = [s for s in result['odom'] if s['t'] >= t]
        displacement = max(math.hypot(s['x']-odom[0]['x'], s['y']-odom[0]['y']) for s in odom)
        check('explicit_goal_path_motion_nav2_success_and_arrival', distance <= .35 and displacement >= .3,
              goal_distance_m=distance, odom_displacement_m=displacement,
              nonempty_paths=sum(p['t'] >= t for p in result['paths']))
        event('navigation_arrived', pose=pose, goal=target, distance_m=distance)
        # Leave the finite trial stopped before camera/marker search can move it.
        api('/api/emergency-stop', {})
        node.wait(lambda s: s['safety'].get('stop_acknowledged') is True, 8, 'final explicit stop')
        result['passed'] = True
        return 0
    except Exception as exc:
        result['error'] = repr(exc)
        event('failure', error=repr(exc))
        return 1
    finally:
        if web_verified:
            try:
                _, current = api()
                if current.get('connection', {}).get('domain_id') == DOMAIN:
                    api('/api/emergency-stop', {})
            except Exception:
                pass
        for obstacle in owned_obstacles:
            try:
                request = DeleteEntity.Request()
                request.name = obstacle
                service(node.delete.call_async(request), 'delete only owned obstacle', 3)
            except Exception:
                pass
        result['raw_scans'] = node.raw_scans
        result['forwarded_scans'] = node.forwarded_scans
        result['contact_sensor_message_count'] = node.contact_messages
        result['duration_seconds'] = elapsed()
        node.destroy_node()
        rclpy.shutdown()
        raw = json.dumps(result, ensure_ascii=False, separators=(',', ':'))+'\n'
        raw_path = args.raw_output or args.output.with_suffix('.raw.json')
        raw_path.write_text(raw)
        summary = {key: value for key, value in result.items() if key not in
                   ('states', 'commands', 'odom', 'ground_truth', 'amcl', 'health',
                    'motion', 'navigation', 'planner', 'paths', 'tf', 'memory',
                    'raw_dynamic_tf_receipts')}
        summary['raw_telemetry'] = {'path': str(raw_path), 'sha256': hashlib.sha256(raw.encode()).hexdigest()}
        summary['sample_counts'] = {key: len(result[key]) for key in
            ('states', 'commands', 'odom', 'ground_truth', 'amcl', 'health', 'motion', 'navigation', 'planner', 'paths', 'tf')}
        summary['amcl_samples'] = result['amcl']
        summary['tf_samples'] = result['tf']
        if args.scenario in ('local_detour', 'barrier'):
            summary['path_samples'] = result['paths']
            summary['planner_samples'] = result['planner']
            summary['memory_samples'] = result['memory']
        summary['navigation_transitions'] = []
        previous = None
        for value in result['navigation']:
            identity = (value.get('status'), str(value.get('goal_stamp')))
            if identity != previous:
                summary['navigation_transitions'].append(value)
                previous = identity
        summary['timeline'] = []
        last_at, previous = -math.inf, None
        for value in result['states']:
            health = value.get('localization_health') or {}
            safety = value.get('safety') or {}
            localization = value.get('localization') or {}
            alignment = value.get('marker_alignment') or {}
            phase = (value['mission'].get('state'), localization.get('state'),
                     safety.get('mode'), safety.get('estop_latched'),
                     health.get('level'), str(health.get('reasons')), alignment.get('state'))
            if phase == previous and value['t']-last_at < 2.:
                continue
            metrics = health.get('metrics') or {}
            summary['timeline'].append({'t': value['t'], 'mission_state': phase[0],
                'localization_state': phase[1], 'control': localization.get('control'),
                'pose': (value.get('robot') or {}).get('pose'),
                'safety': {key: safety.get(key) for key in
                           ('mode', 'estop_latched', 'stop_acknowledged', 'control_seq')},
                'health': {'level': health.get('level'), 'reasons': health.get('reasons'),
                           **{key: metrics.get(key) for key in
                              ('position_std_m', 'scan_map_agreement', 'amcl_update_requests',
                               'amcl_update_responses', 'amcl_stationary_update_requests',
                               'scan_age_seconds', 'odom_age_seconds', 'map_tf_age_seconds',
                               'scan_tf_state', 'scan_tf_error')},
                           **{key: value for key, value in metrics.items()
                              if key.startswith('dynamic_tf_')}},
                'marker_alignment': alignment,
                'delivery_home_ready': (value.get('delivery') or {}).get('home_ready')})
            previous, last_at = phase, value['t']
        summary['ground_truth_samples'] = []
        last_at = -math.inf
        for sample in result['ground_truth']:
            if sample['t']-last_at >= .5:
                summary['ground_truth_samples'].append(sample)
                last_at = sample['t']
        args.output.write_text(json.dumps(compact_summary(summary), ensure_ascii=False, indent=2)+'\n')
        print(('PASS ' if result['passed'] else 'FAIL ')+str(args.output), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
