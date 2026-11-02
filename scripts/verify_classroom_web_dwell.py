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


def event(events, message):
    return next((item for item in events if item.get('message') == message), None)


def assess_dwell(result):
    reasons = []
    route = result.get('route', [])
    if result.get('mission_accepted_via_web') is not True:
        reasons.append('web_mission_not_accepted')
    if route not in ([3], [1, 2], [1, 2, 3]) or result.get('final_route') != route:
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8792')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--readiness-timeout', type=float, default=100.)
    parser.add_argument('--mission-timeout', type=float, default=750.)
    parser.add_argument('--route', choices=('3', '1,2', '1,2,3'), default='1,2,3')
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
            fresh = (state.get('health', {}).get('ready') is True and
                     xy(state) is not None and
                     all(isinstance(robot.get(key), (int, float)) and
                         robot[key] < .8 for key in ('scan_age', 'odom_age')) and
                     safety.get('stop_acknowledged') is True and
                     safety.get('pending') is None)
            if reset_requested and safety.get('latched') is False and fresh:
                if reset_stable_since is None:
                    reset_stable_since = time.monotonic()
                if time.monotonic()-reset_stable_since >= 1.0:
                    break
            else:
                reset_stable_since = None
            if (safety.get('latched') is True and fresh and
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
        request(browser, base, '/api/mission', {'route': route})
        marker_targets = {str(item['id']): [float(item['x']), float(item['y']),
                                              float(item['yaw'])]
                          for item in state['markers']}
        result['marker_targets'] = marker_targets
        response = request(browser, base, '/api/mission/start', {})
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
        events = observed_events
        result['events_observed'] = len(events)
        continuity['start_event_count'] = sum(
            item.get('message') == '사용자가 임무를 시작했습니다.' for item in events)
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
