#!/usr/bin/env python3
"""Verify a real web mission pauses at every sector before the next goal.

HTTP requests are limited to the disposable loopback web on simulation domain
176. Gazebo truth is checked separately by the staged wrapper's ROS observer.
"""

import argparse
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import time
from urllib.error import URLError
from urllib.request import Request

from verify_classroom_web_disconnect import client, pair, request, xy


DWELL_SECONDS = 5.


def graph_prearm():
    """Read ROS graph and the guard's live battery limit, not launch text."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('classroom_prearm_observer')
    try:
        raw_samples = {topic: {'count': 0, 'last_received_mono': None}
                       for topic in ('/wheel/odom', '/imu')}

        def raw_callback(topic):
            def record(_message):
                sample = raw_samples[topic]
                sample['count'] += 1
                sample['last_received_mono'] = time.monotonic()
            return record

        node.create_subscription(Odometry, '/wheel/odom',
                                 raw_callback('/wheel/odom'), qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu', raw_callback('/imu'),
                                 qos_profile_sensor_data)

        def publishers(topic):
            return [f'{item.node_namespace.rstrip("/")}/{item.node_name}'
                    for item in node.get_publishers_info_by_topic(topic)]

        def subscribers(topic):
            return [f'{item.node_namespace.rstrip("/")}/{item.node_name}'
                    for item in node.get_subscriptions_info_by_topic(topic)]

        # DDS discovery needs time after the web/robot processes start. This
        # reads only graph metadata; it never sends a motion command.
        deadline = time.monotonic()+4.
        while True:
            services = dict(node.get_service_names_and_types())
            result = {
                'map_publishers': publishers('/map'),
                'scan_publishers': publishers('/scan'),
                'odom_publishers': publishers('/odom'),
                'battery_publishers': publishers('/battery_state'),
                'raw_wheel_publishers': publishers('/wheel/odom'),
                'raw_imu_publishers': publishers('/imu'),
                'goal_subscribers': subscribers('/smart_vision_amr/goal'),
                'planner_status_publishers': publishers('/smart_vision_amr/planner_status'),
                'cancel_service_types': services.get('/smart_vision_amr/cancel_navigation', []),
            }
            if (len(result['map_publishers']) == 1 and
                    result['scan_publishers'] and result['odom_publishers'] and
                    result['raw_wheel_publishers'] and result['raw_imu_publishers'] and
                    all(sample['count'] >= 2 for sample in raw_samples.values()) and
                    {'/astar_planner', '/path_follower'}.issubset(
                        set(result['goal_subscribers'])) and
                    result['planner_status_publishers'] and
                    result['cancel_service_types'] and
                    result['battery_publishers']) or time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=.25)
        result['raw_input_samples'] = {
            topic: {'count': sample['count'],
                    'last_receipt_age_s': (None if sample['last_received_mono'] is None
                                           else time.monotonic()-sample['last_received_mono'])}
            for topic, sample in raw_samples.items()}
        client = node.create_client(GetParameters, '/ekf_sensor_guard/get_parameters')
        result['guard_battery_minimum_v'] = None
        if client.wait_for_service(timeout_sec=2.):
            future = client.call_async(GetParameters.Request(names=['minimum_battery_voltage']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            if future.done() and future.exception() is None:
                values = future.result().values
                if len(values) == 1:
                    result['guard_battery_minimum_v'] = values[0].double_value
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def evaluate_prearm(state, graph, reset_requested):
    """Keep each physical departure gate and its measured value reviewable."""
    checks = {}

    def add(key, observed, valid, requirement):
        checks[key] = {'passed': bool(valid), 'observed': observed,
                       'requirement': requirement}

    def fresh(value, limit):
        return isinstance(value, (int, float)) and math.isfinite(value) and value < limit

    connection = state.get('connection') or {}
    mapped = state.get('map') or {}
    health = state.get('health') or {}
    configured = health.get('map') or {}
    robot = state.get('robot') or {}
    quality = state.get('localization_health') or {}
    metrics = quality.get('metrics') or {}
    power = robot.get('power') or {}
    safety = state.get('safety') or {}
    add('isolated_ros_domain', connection.get('domain_id'),
        connection.get('domain_id') == 176 and os.environ.get('ROS_LOCALHOST_ONLY') == '1',
        'domain 176, localhost only')
    add('configured_occupancy_map',
        {'preview': mapped.get('preview'), 'configured_match': configured.get('configured_match'),
         'version': mapped.get('version')},
        mapped.get('preview') is False and configured.get('configured_match') is True,
        'real /map matching configured YAML')
    add('single_map_publisher', graph.get('map_publishers'),
        len(graph.get('map_publishers') or []) == 1, 'exactly one /map publisher')
    add('fresh_map_base_tf',
        {'pose_age_s': robot.get('pose_age'), 'map_tf_age_s': metrics.get('map_tf_age_seconds'),
         'pose': robot.get('pose')},
        robot.get('pose') is not None and fresh(robot.get('pose_age'), 2.) and
        fresh(metrics.get('map_tf_age_seconds'), 1.),
        'current map→base pose and TF younger than 1 s')
    for topic, age_key in (('scan', 'scan_age'), ('odom', 'odom_age')):
        sensor_age = metrics.get(topic+'_age_seconds')
        stamp_age = metrics.get(topic+'_header_age_seconds')
        add('fresh_'+topic,
            {'web_age_s': robot.get(age_key), 'health_age_s': sensor_age,
             'header_age_s': stamp_age, 'valid': metrics.get(topic+'_valid'),
             'publishers': graph.get(topic+'_publishers')},
            bool(graph.get(topic+'_publishers')) and fresh(robot.get(age_key), 2.) and
            fresh(sensor_age, .5) and isinstance(stamp_age, (int, float)) and
            math.isfinite(stamp_age) and -.25 <= stamp_age <= .5 and
            metrics.get(topic+'_valid', True) is True,
            'live topic, health receipt/header age ≤0.5 s')
    add('amcl_position_uncertainty', metrics.get('position_std_m'),
        fresh(metrics.get('position_std_m'), .35), '<0.35 m')
    add('amcl_yaw_uncertainty', metrics.get('yaw_std_rad'),
        fresh(metrics.get('yaw_std_rad'), .5), '<0.5 rad')
    amcl_age = metrics.get('amcl_age_seconds')
    amcl_stamp_age = metrics.get('amcl_header_age_seconds')
    add('amcl_pose_freshness',
        {'receipt_age_s': amcl_age, 'header_age_s': amcl_stamp_age},
        fresh(amcl_age, 8.0) and isinstance(amcl_stamp_age, (int, float)) and
        math.isfinite(amcl_stamp_age) and -.25 <= amcl_stamp_age < 8.0,
        'AMCL receipt/header ages <8 s, future header ≤0.25 s')
    score = metrics.get('scan_map_agreement')
    add('scan_map_agreement', score,
        isinstance(score, (int, float)) and math.isfinite(score) and score >= .55,
        '≥0.55 health warning threshold')
    add('localization_health',
        {'level': quality.get('level'), 'reasons': quality.get('reasons'),
         'stop_latched': quality.get('stop_latched'),
         'autonomous_allowed': quality.get('autonomous_allowed')},
        quality.get('level') == 'ok' and quality.get('stop_latched') is False and
        quality.get('autonomous_allowed') is True,
        'acquired, level ok, no health stop latch')
    add('rear_camera', {'age_s': robot.get('camera_age'),
                        'raw_age_s': robot.get('raw_camera_age')},
        fresh(robot.get('camera_age'), 2.) and fresh(robot.get('raw_camera_age'), 2.),
        'fresh rear raw and processed frames')
    add('front_camera', robot.get('front_camera_age'),
        fresh(robot.get('front_camera_age'), 2.), 'fresh front frame')
    voltage = power.get('voltage')
    add('battery_sensor', {'power': power, 'publishers': graph.get('battery_publishers')},
        bool(graph.get('battery_publishers')) and power.get('status') == 'online' and
        fresh(power.get('age'), 5.) and isinstance(voltage, (int, float)) and
        math.isfinite(voltage) and voltage >= 10.8,
        'live /battery_state ≥10.8 V')
    add('robot_guard_battery_limit', graph.get('guard_battery_minimum_v'),
        graph.get('guard_battery_minimum_v') == 10.8,
        'live EKF guard parameter minimum_battery_voltage=10.8 V')
    for topic, publishers_key in (('/wheel/odom', 'raw_wheel_publishers'),
                                  ('/imu', 'raw_imu_publishers')):
        sample = (graph.get('raw_input_samples') or {}).get(topic) or {}
        add('fresh_raw_'+('wheel' if topic == '/wheel/odom' else 'imu'),
            {'publishers': graph.get(publishers_key), 'sample': sample},
            bool(graph.get(publishers_key)) and sample.get('count', 0) >= 2 and
            fresh(sample.get('last_receipt_age_s'), 1.0),
            'two independent raw sensor samples with last receipt <1 s')
    add('astar_and_cancel',
        {'goal_subscribers': graph.get('goal_subscribers'),
         'planner_status_publishers': graph.get('planner_status_publishers'),
         'cancel_service_types': graph.get('cancel_service_types')},
        {'/astar_planner', '/path_follower'}.issubset(
            set(graph.get('goal_subscribers') or [])) and
        bool(graph.get('planner_status_publishers')) and
        'std_srvs/srv/Trigger' in (graph.get('cancel_service_types') or []),
        'A* and follower goal subscribers, planner status, cancel service')
    add('mux_stop_ack_and_explicit_reset',
        {'requested': reset_requested, 'latched': safety.get('latched'),
         'stop_acknowledged': safety.get('stop_acknowledged'),
         'pending': safety.get('pending'), 'autonomous_allowed': safety.get('autonomous_allowed')},
        reset_requested is True and safety.get('latched') is False and
        safety.get('stop_acknowledged') is True and safety.get('pending') is None and
        safety.get('autonomous_allowed') is True,
        'explicit web reset, robot-side stopped ACK, autonomous handoff')
    add('web_departure_readiness',
        {'ready': health.get('ready'), 'missing': health.get('missing')},
        health.get('ready') is True and not health.get('missing'),
        'all web departure conditions ready')
    return {'passed': all(check['passed'] for check in checks.values()),
            'sampled_at': time.time(), 'checks': checks,
            'failed_gates': [name for name, check in checks.items() if not check['passed']]}


def event(events, message):
    return next((item for item in events if item.get('message') == message), None)


def assess_dwell(result):
    reasons = []
    route = result.get('route', [])
    if result.get('mission_accepted_via_web') is not True:
        reasons.append('web_mission_not_accepted')
    if result.get('dispatch_mode') == 'inventory' and (
            result.get('inventory_completion', {}).get('status') != 'DELIVERED' or
            result.get('inventory_completion', {}).get('service_active') is not False):
        reasons.append('inventory_request_not_delivered_and_closed')
    if route not in ([2], [3], [1, 2], [1, 2, 3], [1, 3, 2]) or result.get('final_route') != route:
        reasons.append('route_mismatch')
    if result.get('final_state') != 'HOME_REACHED' or result.get('completed_ids') != route:
        reasons.append('sector_arrival_and_home_unverified')
    if result.get('configured_dwell_seconds') != DWELL_SECONDS:
        reasons.append('configured_dwell_not_five_seconds')
    if result.get('final_home_distance_m') is None or result['final_home_distance_m'] > .35:
        reasons.append('home_position_unverified')
    if result.get('final_speed') is None or any(
            abs(value) > limit for value, limit in zip(result['final_speed'], (.025, .06))):
        reasons.append('home_not_stationary')
    if (result.get('home_stationary_verified') is not True or
            result.get('home_stationary_duration_s', 0.) < 1.2 or
            result.get('home_pose_drift_m', math.inf) > .025):
        reasons.append('home_continuous_settle_unverified')
    continuity = result.get('mission_continuity') or {}
    if (continuity.get('route_preserved') is not True or
            continuity.get('completed_prefix_valid') is not True or
            continuity.get('start_event_count') != 1 or
            continuity.get('cancel_event_count') != 0 or
            any(continuity.get('marker_confirmations', {}).get(str(marker)) != 1
                for marker in route)):
        reasons.append('web_mission_continuity_unverified')
    for marker_id in route:
        leg = result.get('dwell', {}).get(str(marker_id), {})
        if not leg.get('marker_confirmed'):
            reasons.append(f'sector_{marker_id}_camera_confirmation_missing')
        camera = (result.get('rear_camera_frames') or {}).get(str(marker_id)) or {}
        if (camera.get('expected_visible') is not True or
                not isinstance(camera.get('center_error'), (int, float)) or
                abs(camera['center_error']) > .36):
            reasons.append(f'sector_{marker_id}_independent_jpeg_marker_unverified')
        alignment = leg.get('alignment_at_dwell') or {}
        if (alignment.get('state') != 'CONFIRMED' or
                alignment.get('soft_center_result') != 'centered' or
                not isinstance(alignment.get('center_error'), (int, float)) or
                abs(alignment['center_error']) > .36):
            reasons.append(f'sector_{marker_id}_camera_centering_unverified')
        if not leg.get('observed_dwelling') or leg.get('sample_count', 0) < 2:
            reasons.append(f'sector_{marker_id}_dwell_state_missing')
        if leg.get('event_duration_s') is None or leg['event_duration_s'] < 4.9:
            reasons.append(f'sector_{marker_id}_dwell_less_than_five_seconds')
        if leg.get('next_goal_after_dwell') is not True:
            reasons.append(f'sector_{marker_id}_next_goal_not_after_dwell')
        if leg.get('max_linear_mps', 0.) >= .025 or leg.get('max_angular_rps', 0.) >= .06:
            reasons.append(f'sector_{marker_id}_moving_during_dwell')
        if leg.get('stale_odom_samples', 0) > 0:
            reasons.append(f'sector_{marker_id}_odom_stale_during_dwell')
        if leg.get('max_pose_delta_m', 0.) > .12:
            reasons.append(f'sector_{marker_id}_pose_drift_during_dwell')
        if result.get('path_points_per_sector', {}).get(str(marker_id), 0) < 1:
            reasons.append(f'sector_{marker_id}_path_missing')
    if result.get('home_path_points', 0) < 1:
        reasons.append('home_path_missing')
    if result.get('transient_requested') and result.get(
            'transient_profile_request', {}).get('profile_requested') != 'transient':
        reasons.append('mid_route_transient_not_requested')
    return reasons


def request_transient_profile():
    repo = Path(__file__).resolve().parents[1]
    workspace = repo.parents[1]
    profile = repo/'ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml'
    script = repo/'scripts/request_classroom_fault_profile.py'
    shell = (
        'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
        'ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH; '
        'source /opt/ros/humble/setup.bash; '
        f'source {shlex.quote(str(workspace/"install/local_setup.bash"))}; '
        'export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; '
        f'export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(profile))}; '
        'export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; '
        f'exec /usr/bin/python3 {shlex.quote(str(script))} --profile transient')
    result = subprocess.run(['bash', '-c', shell], cwd=repo, capture_output=True,
                            text=True, check=False, timeout=8)
    if result.returncode != 0:
        raise RuntimeError('Transient profile request failed: '+
                           (result.stderr or result.stdout)[-500:])
    return json.loads(result.stdout.strip().splitlines()[-1])


def save_rear_frame(browser, base, target, expected_marker_id=None,
                    raw_camera_age=None):
    """Preserve the unannotated detected frame for independent ArUco checks."""
    if expected_marker_id is not None and (
            not isinstance(raw_camera_age, (int, float)) or raw_camera_age >= .8):
        raise ValueError('Unannotated rear camera frame is stale')
    endpoint = ('/api/camera/raw.jpg' if expected_marker_id is not None else
                '/api/camera.jpg')
    with browser.open(Request(base+endpoint,
                              headers={'Connection': 'close'}), timeout=3) as response:
        data = response.read()
    if len(data) < 100 or not data.startswith(b'\xff\xd8'):
        raise ValueError('Rear camera did not return a usable JPEG')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    result = {'file': str(target), 'bytes': len(data),
              'raw_camera_age_s': raw_camera_age,
              'unannotated': expected_marker_id is not None}
    if expected_marker_id is not None:
        import cv2
        import numpy as np
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('Rear camera JPEG cannot be decoded')
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = (cv2.aruco.DetectorParameters_create() if hasattr(
            cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters())
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            corners, ids, _ = cv2.aruco.ArucoDetector(
                dictionary, parameters).detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, dictionary, parameters=parameters)
        detected = [] if ids is None else [int(value) for value in ids.flatten()]
        result['detected_ids'] = detected
        result['expected_visible'] = expected_marker_id in detected
        result['center_error'] = None
        if result['expected_visible']:
            matching = detected.index(expected_marker_id)
            result['center_error'] = float(
                2*np.mean(corners[matching][0, :, 0])/image.shape[1]-1)
    return result


def save_front_frame(browser, base, target, camera_age, marker_id):
    """Save one independently decoded, fresh front-camera frame per route leg."""
    if not isinstance(camera_age, (int, float)) or camera_age >= .8:
        raise ValueError('Front camera frame is stale')
    with browser.open(Request(base+'/api/camera/front.jpg',
                              headers={'Connection': 'close'}), timeout=3) as response:
        data = response.read()
    if len(data) < 100 or not data.startswith(b'\xff\xd8'):
        raise ValueError('Front camera did not return a usable JPEG')
    import cv2
    import numpy as np
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Front camera JPEG cannot be decoded')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    return {'file': str(target), 'bytes': len(data),
            'width': int(image.shape[1]), 'height': int(image.shape[0]),
            'camera_age_s': camera_age, 'route_marker_id': marker_id,
            'captured_at': time.time()}


def obstacle_hold_started_at(state, previous, now):
    """Time only a continuous, planner-confirmed obstacle hold."""
    mission = state.get('mission') or {}
    navigation = state.get('navigation') or {}
    memory = state.get('failure_memory') or {}
    if (mission.get('state') == 'WAIT_REPLAN' and
            navigation.get('status') == 'WAIT_REPLAN' and
            memory.get('state') == 'OBSTACLE_HOLD' and
            not state.get('path')):
        return now if previous is None else previous
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8792')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--readiness-timeout', type=float, default=100.)
    parser.add_argument('--mission-timeout', type=float, default=750.)
    parser.add_argument('--obstacle-hold-timeout', type=float, default=0.,
                        help='Fail a sustained safe obstacle hold after this many seconds; '
                             '0 keeps the overall mission timeout')
    parser.add_argument('--route', choices=('2', '3', '1,2', '1,2,3', '1,3,2'), default='1,2,3')
    parser.add_argument('--dispatch-mode', choices=('manual', 'inventory'), default='manual',
                        help='inventory submits a real manager shortage and starts operator dispatch')
    parser.add_argument('--inject-transient', action='store_true',
                        help='Inject an isolated 0.3–0.4 s relay fault after actual route motion begins')
    args = parser.parse_args()
    route = [int(item) for item in args.route.split(',')]
    if os.environ.get('ROS_DOMAIN_ID') != '176' or not args.url.startswith(
            'http://127.0.0.1:8792'):
        parser.error('This test only contacts disposable domain 176 loopback port 8792')
    result = {'passed': False, 'domain': 176, 'localhost_only': True,
              'physical_robot_touched': False, 'mission_accepted_via_web': False,
              'route': route, 'failure': None, 'dwell': {},
              'dispatch_mode': args.dispatch_mode,
              'transient_requested': args.inject_transient,
              'path_points_per_sector': {str(i): 0 for i in route},
              'home_path_points': 0}
    try:
        browser = client()
        base = args.url.rstrip('/')
        readiness_deadline = time.monotonic()+args.readiness_timeout
        while True:
            try:
                pair(browser, base)
                break
            except (URLError, TimeoutError):
                if time.monotonic() >= readiness_deadline:
                    raise TimeoutError('Web server 8792 did not start')
                time.sleep(.5)
        # The parity-mode mux/guard starts latched, just like the physical
        # robot. The guard itself enforces two seconds of continuous healthy
        # inputs. Do not duplicate that requirement with an HTTP polling
        # window: brief delivery jitter would prevent any reset request.
        reset_requested = False
        reset_attempts = []
        last_attempt_at = -float('inf')
        reset_stable_since = None
        while time.monotonic() < readiness_deadline:
            state = request(browser, base, '/api/state')
            safety = state.get('safety') or {}
            robot = state.get('robot') or {}
            stopped_inputs = (xy(state) is not None and
                     all(isinstance(robot.get(key), (int, float)) and
                         robot[key] < .8 for key in ('scan_age', 'odom_age')) and
                     safety.get('stop_acknowledged') is True and
                     safety.get('pending') is None)
            fresh = stopped_inputs and state.get('health', {}).get('ready') is True
            if reset_requested and safety.get('latched') is False and fresh:
                if reset_stable_since is None:
                    reset_stable_since = time.monotonic()
                if time.monotonic()-reset_stable_since >= 1.0:
                    break
            else:
                reset_stable_since = None
            # An independently acknowledged manual AMCL reset can leave the
            # health stop latch set until the operator explicitly releases it.
            # Requesting the reset while stationary is not departure; the
            # full prearm gates below still prohibit motion until health is ok.
            if (safety.get('latched') is True and stopped_inputs and
                    time.monotonic()-last_attempt_at >= 3.0):
                last_attempt_at = time.monotonic()
                try:
                    request(browser, base, '/api/emergency-stop', {'reset': True})
                    reset_attempts.append({'at': time.time(), 'accepted': True})
                    reset_requested = True
                except RuntimeError as exc:
                    reset_attempts.append({'at': time.time(), 'accepted': False,
                                           'reason': str(exc)})
            time.sleep(.2)
        else:
            raise TimeoutError('Startup mux/guard ESTOP remained latched; '
                               f'web reset attempts={reset_attempts[-5:]}')
        result['startup_reset_requested_via_web'] = reset_requested
        result['startup_reset_attempts'] = reset_attempts
        result['startup_reset_completed'] = reset_requested and (
            (state.get('safety') or {}).get('latched') is False)
        graph = graph_prearm()
        result['prearm_graph'] = graph
        prearm_trace = []
        prearm_deadline = min(readiness_deadline, time.monotonic()+15.)
        while True:
            state = request(browser, base, '/api/state')
            prearm = evaluate_prearm(state, graph, reset_requested)
            previous = prearm_trace[-1] if prearm_trace else None
            if previous is None or previous['failed_gates'] != prearm['failed_gates']:
                prearm_trace.append({'sampled_at': prearm['sampled_at'],
                                     'failed_gates': prearm['failed_gates']})
            if prearm['passed'] or time.monotonic() >= prearm_deadline:
                break
            time.sleep(.2)
        result['prearm'] = prearm
        result['prearm_trace'] = prearm_trace
        if not prearm['passed']:
            raise RuntimeError('Physical-style prearm failed: '+', '.join(
                prearm['failed_gates']))
        manager = None
        if args.dispatch_mode == 'inventory':
            if route != [2]:
                raise RuntimeError('Inventory repeat fixture requires one Sector 2 request')
            manager = client()
            paired = request(manager, base, '/api/inventory/pair',
                             {'user_id': 1, 'domain_id': 176})
            if paired.get('authenticated') is not True:
                raise RuntimeError('Inventory manager domain pairing failed')
            submitted = request(manager, base, '/api/inventory/requests', {
                'marker_id': 2, 'item': 'Gazebo 왕복 검증', 'quantity': 1,
                'client_request_id': f'classroom-repeat-{time.time_ns()}'})
            requests = submitted.get('requests') or []
            candidates = [item for item in requests if item.get('marker_id') == 2 and
                          item.get('status') == 'PENDING']
            if len(candidates) != 1:
                raise RuntimeError('Exactly one pending Sector 2 inventory request required')
            result['inventory_request'] = candidates[0]
            state = request(browser, base, '/api/state')
            if (state.get('delivery') or {}).get('dispatch_ready') is not True:
                raise RuntimeError('Inventory dispatch blocked: '+str(
                    (state.get('delivery') or {}).get('dispatch_missing')))
        else:
            request(browser, base, '/api/mission', {'route': route})
        marker_targets = {str(item['id']): [float(item['x']), float(item['y']),
                                              float(item['yaw'])]
                          for item in state['markers']}
        result['marker_targets'] = marker_targets
        mission_started_at = time.time()
        response = request(browser, base,
                           '/api/dispatch/start' if args.dispatch_mode == 'inventory'
                           else '/api/mission/start',
                           {'loaded': True} if args.dispatch_mode == 'inventory' else {})
        result['mission_started_at'] = mission_started_at
        mission = response.get('mission', {})
        if mission.get('state') != 'NAVIGATING' or mission.get('route') != route:
            raise RuntimeError('Web rejected route '+str(route)+': '+str(mission))
        result['mission_accepted_via_web'] = True
        result['configured_dwell_seconds'] = mission.get('sector_dwell_seconds')
        result['home_target'] = mission.get('home')
        result['start_pose'] = list(xy(state))
        observations = {str(i): {'sample_count': 0, 'first_pose': None,
                                  'max_pose_delta_m': 0., 'max_linear_mps': 0.,
                                  'max_angular_rps': 0., 'stale_odom_samples': 0}
                        for i in route}
        seen_next = {str(i): False for i in route}
        # The server retains only its latest event window. Keep the entire
        # route's confirmations as it runs instead of reading only the last
        # HTTP response after a potentially long three-sector mission.
        observed_events = []
        event_keys = set()
        continuity = {'samples': 0, 'route_preserved': True,
                      'completed_prefix_valid': True, 'states': {},
                      'web_wait_replan_seen': False}
        overlay_trace = []
        alignment_trace = []
        route_trace = []
        rear_frames = {}
        rear_frame_errors = {}
        front_frames = {}
        previous_overlay = None
        previous_alignment = None
        previous_route = None
        home_stable_since = None
        home_stable_first_pose = None
        home_stable_max_drift = 0.
        home_stable_samples = 0

        def keep_events(state):
            for item in state.get('events', []):
                # A second mission can run on the same web process. Ignore
                # the completed first mission's retained event window.
                if item.get('at', 0.) < mission_started_at-.2:
                    continue
                identity = (item.get('at'), item.get('message'))
                if identity not in event_keys:
                    event_keys.add(identity)
                    observed_events.append(item)

        def overlay_sample(state):
            observed = state.get('observed_obstacles') or {}
            mapped = state.get('map') or {}
            age = observed.get('age')
            return {'at': time.time(), 'cells': list(observed.get('cells') or []),
                    'map_version': mapped.get('version'),
                    'observed_map_version': observed.get('map_version'),
                    'observed_age': age,
                    'map_preview': mapped.get('preview'),
                    'configured_match': mapped.get('configured_match'),
                    'renderable': (mapped.get('preview') is False and
                                   mapped.get('configured_match') is True and
                                   observed.get('map_version') == mapped.get('version') and
                                   isinstance(age, (int, float)) and math.isfinite(age) and
                                   age < 2.)}

        keep_events(response)
        deadline = time.monotonic()+args.mission_timeout
        latest = response
        injected = False
        hold_started_at = None
        while time.monotonic() < deadline:
            latest = request(browser, base, '/api/state')
            keep_events(latest)
            mission = latest.get('mission', {})
            phase = mission.get('state')
            marker = mission.get('current_marker_id')
            overlay = overlay_sample(latest)
            overlay_key = (tuple(overlay['cells']), overlay['renderable'],
                           overlay['map_version'], overlay['observed_map_version'])
            if overlay_key != previous_overlay:
                overlay_trace.append(overlay)
                previous_overlay = overlay_key
            alignment = latest.get('marker_alignment') or {}
            if phase in {'CONFIRMING', 'DWELLING'}:
                robot = latest.get('robot') or {}
                localization = latest.get('localization') or {}
                alignment_trace.append({
                    'at': time.time(), 'marker_id': marker, 'state': alignment,
                    'localization_clearance_m': localization.get('clearance'),
                    'localization_available': localization.get('available'),
                    'scan_age_s': robot.get('scan_age'),
                    'odom_age_s': robot.get('odom_age'),
                    'rear_camera_age_s': robot.get('camera_age'),
                    'front_camera_age_s': robot.get('front_camera_age')})
            route_key = (phase, mission.get('current_marker_id'),
                         len(latest.get('path') or []),
                         (latest.get('navigation') or {}).get('planner_status'))
            if route_key != previous_route:
                route_trace.append({'at': time.time(), 'state': phase,
                                    'marker_id': mission.get('current_marker_id'),
                                    'path_points': len(latest.get('path') or []),
                                    'planner_status': route_key[-1]})
                previous_route = route_key
            if args.obstacle_hold_timeout > 0:
                now = time.monotonic()
                hold_started_at = obstacle_hold_started_at(latest, hold_started_at, now)
                if (hold_started_at is not None and
                        now-hold_started_at >= args.obstacle_hold_timeout):
                    result['bounded_obstacle_hold'] = {
                        'duration_s': round(now-hold_started_at, 3),
                        'mission': mission,
                        'navigation': latest.get('navigation'),
                        'failure_memory': latest.get('failure_memory'),
                        'observed_obstacles': latest.get('observed_obstacles'),
                        'pose': (latest.get('robot') or {}).get('pose'),
                        'speed': (latest.get('robot') or {}).get('speed'),
                        'safety': latest.get('safety'),
                        'localization_health': latest.get('localization_health'),
                        'path_points': len(latest.get('path') or []),
                    }
                    raise TimeoutError(
                        'Continuous planner-confirmed obstacle hold exceeded '
                        f'{args.obstacle_hold_timeout:g} s while retaining goal')
            continuity['samples'] += 1
            continuity['route_preserved'] &= mission.get('route') == route
            completed = mission.get('completed_ids')
            continuity['completed_prefix_valid'] &= (
                isinstance(completed, list) and completed == route[:len(completed)])
            continuity['states'][str(phase)] = continuity['states'].get(str(phase), 0)+1
            continuity['web_wait_replan_seen'] |= phase == 'WAIT_REPLAN'
            path_points = len(latest.get('path', []))
            if (args.inject_transient and not injected and phase == 'NAVIGATING'
                    and marker == route[0] and path_points > 0):
                current = xy(latest)
                start_pose = result['start_pose']
                if current is not None and math.hypot(
                        current[0]-start_pose[0], current[1]-start_pose[1]) >= .25:
                    result['transient_trigger_pose'] = list(current)
                    result['transient_profile_request'] = request_transient_profile()
                    injected = True
            if phase == 'NAVIGATING' and marker in route:
                result['path_points_per_sector'][str(marker)] = max(
                    result['path_points_per_sector'][str(marker)], path_points)
                index = route.index(marker)
                if index > 0:
                    seen_next[str(route[index-1])] = True
            elif phase == 'RETURNING_HOME':
                result['home_path_points'] = max(result['home_path_points'], path_points)
                seen_next[str(route[-1])] = True
            if (phase in {'NAVIGATING', 'CONFIRMING', 'DWELLING'} and marker in route
                    and str(marker) not in front_frames):
                camera_age = (latest.get('robot') or {}).get('front_camera_age')
                if isinstance(camera_age, (int, float)) and camera_age < .8:
                    try:
                        front_frames[str(marker)] = save_front_frame(
                            browser, base,
                            args.output.with_name(f'sector_{marker}_front_live.jpg'),
                            camera_age, marker)
                    except (OSError, URLError, TimeoutError, ValueError):
                        # A later fresh frame can still prove camera delivery.
                        pass
            if phase == 'DWELLING' and mission.get('dwell_marker_id') in route:
                marker_id = str(mission['dwell_marker_id'])
                if observations[marker_id].get('alignment_at_dwell') is None:
                    observations[marker_id]['alignment_at_dwell'] = alignment
                if marker_id not in rear_frames:
                    target = args.output.with_name('sector_'+marker_id+'_rear_confirmed.jpg')
                    try:
                        rear_frames[marker_id] = save_rear_frame(
                            browser, base, target, int(marker_id),
                            (latest.get('robot') or {}).get('raw_camera_age'))
                    except Exception as exc:
                        # Keep trying during the five-second stop. The first
                        # HTTP poll can race the next fresh camera frame.
                        rear_frame_errors[marker_id] = str(exc)
                item = observations[marker_id]
                item['sample_count'] += 1
                pose = xy(latest)
                if pose is not None:
                    if item['first_pose'] is None:
                        item['first_pose'] = list(pose)
                    item['max_pose_delta_m'] = max(item['max_pose_delta_m'], math.hypot(
                        pose[0]-item['first_pose'][0], pose[1]-item['first_pose'][1]))
                speed = latest['robot']['speed']
                item['max_linear_mps'] = max(item['max_linear_mps'], abs(speed['linear']))
                item['max_angular_rps'] = max(item['max_angular_rps'], abs(speed['angular']))
                age = latest['robot']['odom_age']
                if age is None or age > 1.:
                    item['stale_odom_samples'] += 1
            if phase in {'PAUSED', 'ESTOP', 'ERROR', 'CANCELED'}:
                if phase == 'PAUSED' and (latest.get('marker_alignment') or {}).get('state') == 'FAILED':
                    try:
                        rear_frames['failed_marker'] = save_rear_frame(
                            browser, base, args.output.with_name('marker_alignment_failed_rear.jpg'))
                    except Exception as exc:
                        rear_frames['failed_marker'] = {'error': str(exc)}
                result['pause_marker_alignment'] = latest.get('marker_alignment')
                result['pause_goal'] = mission.get('current_marker_id')
                raise RuntimeError(f'Mission stopped in {phase}: {mission.get("reason")}')
            if phase == 'HOME_REACHED':
                robot = latest.get('robot') or {}
                pose = xy(latest)
                speed = robot.get('speed') or {}
                stable = (pose is not None and
                          latest.get('health', {}).get('ready') is True and
                          (latest.get('safety') or {}).get('latched') is False and
                          isinstance(robot.get('pose_age'), (int, float)) and
                          robot['pose_age'] < .5 and
                          isinstance(robot.get('odom_age'), (int, float)) and
                          robot['odom_age'] < .5 and
                          abs(speed.get('linear', math.inf)) < .025 and
                          abs(speed.get('angular', math.inf)) < .06)
                if stable:
                    if home_stable_since is None:
                        home_stable_since = time.monotonic()
                        home_stable_first_pose = pose
                        home_stable_max_drift = 0.
                        home_stable_samples = 0
                    home_stable_samples += 1
                    home_stable_max_drift = max(home_stable_max_drift, math.hypot(
                        pose[0]-home_stable_first_pose[0],
                        pose[1]-home_stable_first_pose[1]))
                    elapsed = time.monotonic()-home_stable_since
                    if (elapsed >= 1.2 and home_stable_samples >= 5 and
                            home_stable_max_drift <= .025):
                        result['home_stationary_verified'] = True
                        result['home_stationary_duration_s'] = round(elapsed, 3)
                        result['home_pose_drift_m'] = round(home_stable_max_drift, 4)
                        break
                    if home_stable_max_drift > .025:
                        stable = False
                if not stable:
                    home_stable_since = None
                    home_stable_first_pose = None
                    home_stable_max_drift = 0.
                    home_stable_samples = 0
            time.sleep(.2)
        else:
            raise TimeoutError('Web route '+str(route)+'→Home mission timed out: '+str(
                latest.get('mission', {})))
        result['final_state'] = mission.get('state')
        result['transient_injected_mid_route'] = injected
        result['final_route'] = mission.get('route')
        result['completed_ids'] = mission.get('completed_ids')
        result['final_observed_at'] = time.time()
        home = mission.get('home')
        pose = xy(latest)
        result['final_home_distance_m'] = (math.hypot(pose[0]-home['x'], pose[1]-home['y'])
                                           if pose is not None and isinstance(home, dict)
                                           else None)
        speed = latest['robot']['speed']
        result['final_speed'] = [speed['linear'], speed['angular']]
        if manager is not None:
            inventory = request(manager, base, '/api/inventory/state')
            ident = result['inventory_request']['id']
            record = next((item for item in inventory.get('requests', [])
                           if item.get('id') == ident), {})
            result['inventory_completion'] = {
                'request_id': ident, 'status': record.get('status'),
                'service_active': (inventory.get('service') or {}).get('active')}
        events = observed_events
        result['events_observed'] = len(events)
        continuity['start_event_count'] = sum(
            ('운용자 0이 적재를 확인하고 배송 1건을 출발시켰습니다.'
             if args.dispatch_mode == 'inventory' else '사용자가 임무를 시작했습니다.')
            == item.get('message') for item in events)
        continuity['cancel_event_count'] = sum(
            '사용자가 임무를 취소' in item.get('message', '') for item in events)
        continuity['marker_confirmations'] = {str(marker_id): sum(
            item.get('message') == f'마커 {marker_id}을 실제 카메라 영상에서 확인했습니다.'
            for item in events) for marker_id in route}
        result['mission_continuity'] = continuity
        overlay_trace.append(dict(overlay_sample(latest), terminal_snapshot=True))
        result['rear_camera_frames'] = rear_frames
        result['rear_camera_frame_errors'] = rear_frame_errors
        result['front_camera_frames'] = front_frames
        result['observed_obstacle_overlay_trace'] = overlay_trace
        result['marker_alignment_trace'] = alignment_trace
        result['web_route_trace'] = route_trace
        for marker_id in route:
            k = str(marker_id)
            start = event(events, f'섹터 {marker_id}에서 5초 대기합니다.')
            end = event(events, f'섹터 {marker_id} 대기가 끝나 다음 목표로 이동합니다.')
            confirmed = event(events, f'마커 {marker_id}을 실제 카메라 영상에서 확인했습니다.')
            item = dict(observations[k])
            item['observed_dwelling'] = item['sample_count'] > 0
            item['marker_confirmed'] = confirmed is not None
            item['start_event_at'] = start.get('at') if start else None
            item['end_event_at'] = end.get('at') if end else None
            item['event_duration_s'] = (round(end['at']-start['at'], 3)
                                        if start and end else None)
            item['next_goal_after_dwell'] = bool(
                start and end and end['at'] >= start['at'] and seen_next[k])
            result['dwell'][k] = item
        result['reasons'] = assess_dwell(result)
        result['passed'] = not result['reasons']
    except Exception as exc:
        result['failure'] = repr(exc)
        if 'latest' in locals() and isinstance(latest, dict):
            terminal_mission = latest.get('mission') or {}
            result['final_state'] = terminal_mission.get('state')
            result['final_route'] = terminal_mission.get('route')
            result['completed_ids'] = terminal_mission.get('completed_ids')
            result['final_observed_at'] = time.time()
            result['terminal_snapshot'] = {
                'mission': terminal_mission,
                'navigation': latest.get('navigation'),
                'failure_memory': latest.get('failure_memory'),
                'observed_obstacles': latest.get('observed_obstacles'),
                'pose': (latest.get('robot') or {}).get('pose'),
                'safety': latest.get('safety'),
                'localization_health': latest.get('localization_health'),
            }
        if 'rear_frames' in locals():
            result['rear_camera_frames'] = rear_frames
        if 'rear_frame_errors' in locals():
            result['rear_camera_frame_errors'] = rear_frame_errors
        if 'front_frames' in locals():
            result['front_camera_frames'] = front_frames
        if 'alignment_trace' in locals():
            result['marker_alignment_trace'] = alignment_trace
        if 'route_trace' in locals():
            result['web_route_trace'] = route_trace
        if 'overlay_trace' in locals():
            result['observed_obstacle_overlay_trace'] = overlay_trace
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('PASS' if result['passed'] else 'FAIL', args.output, flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
