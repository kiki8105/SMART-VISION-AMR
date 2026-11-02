#!/usr/bin/env python3
"""Verify two Sector 2 web deliveries on ONE isolated Gazebo/Web/ROS runtime.

Between deliveries the operator latches ESTOP, sets the actual Home pose as
an AMCL prior through the web, waits for independent health reacquisition,
then the second delivery verifier explicitly resets ESTOP. No safety check is
disabled and no robot on physical ROS domain 16 can be contacted.
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from verify_classroom_web_disconnect import client, pair, request


def process_identity(pid):
    """Linux process start ticks rule out a transparent process restart."""
    if pid <= 0:
        raise ValueError('A positive runtime PID is required')
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return {'pid': pid, 'start_ticks': int(fields[19])}


def wait_state(browser, base, predicate, timeout, label, trace):
    deadline = time.monotonic()+timeout
    latest = None
    while time.monotonic() < deadline:
        latest = request(browser, base, '/api/state')
        safety = latest.get('safety') or {}
        mission = latest.get('mission') or {}
        manual_pose = (latest.get('localization') or {}).get('manual_pose') or {}
        health = latest.get('localization_health') or {}
        sample = {'at': time.time(), 'phase': label,
                  'mission': mission.get('state'),
                  'estop_latched': safety.get('latched'),
                  'stop_acknowledged': safety.get('stop_acknowledged'),
                  'manual_pose': manual_pose.get('state'),
                  'manual_pose_result': manual_pose.get('result'),
                  'position_std_m': (health.get('metrics') or {}).get('position_std_m'),
                  'yaw_std_rad': (health.get('metrics') or {}).get('yaw_std_rad'),
                  'scan_map_agreement': (health.get('metrics') or {}).get('scan_map_agreement'),
                  'localization_level': health.get('level'),
                  'health_stop_latched': health.get('stop_latched'),
                  'autonomous_allowed': health.get('autonomous_allowed'),
                  'readiness_missing': (latest.get('health') or {}).get('missing')}
        if not trace or {k: v for k, v in sample.items() if k != 'at'} != {
                k: v for k, v in trace[-1].items() if k != 'at'}:
            trace.append(sample)
        if predicate(latest):
            return latest
        time.sleep(.2)
    raise TimeoutError(f'{label} timed out: '+str(trace[-3:]))


def run_dwell(base, output, timeout, dispatch_mode):
    command = [sys.executable, str(Path(__file__).with_name(
        'verify_classroom_web_dwell.py')), '--url', base,
        '--route', '2', '--dispatch-mode', dispatch_mode,
        '--mission-timeout', str(timeout),
        '--output', str(output)]
    done = subprocess.run(command, check=False, timeout=timeout+160,
                          env={**os.environ, 'ROS_DOMAIN_ID': '176',
                               'ROS_LOCALHOST_ONLY': '1'})
    if not output.is_file():
        raise RuntimeError('Sector 2 verifier produced no evidence')
    result = json.loads(output.read_text())
    if done.returncode != 0 or result.get('passed') is not True:
        raise RuntimeError('Sector 2→Home failed: '+str(result.get('reasons') or
                           result.get('failure') or done.returncode))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8792')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--web-pid', type=int, required=True)
    parser.add_argument('--runner-pid', type=int, required=True)
    parser.add_argument('--mission-timeout', type=float, default=750.)
    parser.add_argument('--dispatch-mode', choices=('manual', 'inventory'), default='manual')
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID') != '176' or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1' or
            args.url.rstrip('/') != 'http://127.0.0.1:8792'):
        parser.error('Only isolated domain 176 and loopback web 8792 are allowed')
    result = {'passed': False, 'domain': 176, 'localhost_only': True,
              'physical_robot_touched': False, 'route': [2],
              'dispatch_mode': args.dispatch_mode,
              'cycles': [], 'reentry': {'trace': []}, 'failure': None}
    try:
        base = args.url.rstrip('/')
        identities = {'web': process_identity(args.web_pid),
                      'gazebo_runner': process_identity(args.runner_pid)}
        result['runtime_identity_initial'] = identities
        first_path = args.output.parent/'cycle_1'/'acceptance.json'
        first = run_dwell(base, first_path, args.mission_timeout, args.dispatch_mode)
        result['cycles'].append(first)
        result['runtime_identity_after_first'] = {
            key: process_identity(value['pid']) for key, value in identities.items()}
        browser = client()
        pair(browser, base)
        before = request(browser, base, '/api/state')
        home = before.get('mission', {}).get('home')
        if (before.get('mission', {}).get('state') != 'HOME_REACHED' or
                not isinstance(home, dict) or
                not all(isinstance(home.get(k), (int, float))
                        and math.isfinite(home[k]) for k in ('x', 'y', 'yaw'))):
            raise RuntimeError('First delivery is not stationary at a valid Home')
        result['reentry']['home'] = home
        result['reentry']['before'] = {
            'localization_health': before.get('localization_health'),
            'readiness': before.get('health'),
            'pose': (before.get('robot') or {}).get('pose')}
        pose = (before.get('robot') or {}).get('pose') or {}
        quality = before.get('localization_health') or {}
        score = (quality.get('metrics') or {}).get('scan_map_agreement')
        if (not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                    for k in ('x', 'y', 'yaw')) or
                not isinstance(score, (int, float)) or score < .60 or
                quality.get('level') != 'ok'):
            raise RuntimeError('Current post-Home AMCL pose is not a validated manual seed')
        result['reentry']['manual_seed_source'] = 'validated_post_home_amcl'
        result['reentry']['manual_seed'] = dict(pose)
        result['reentry']['registered_home_offset_m'] = math.hypot(
            pose['x']-home['x'], pose['y']-home['y'])
        result['reentry']['registered_home_yaw_offset_rad'] = math.atan2(
            math.sin(pose['yaw']-home['yaw']), math.cos(pose['yaw']-home['yaw']))
        request(browser, base, '/api/emergency-stop', {'reset': False})
        trace = result['reentry']['trace']
        wait_state(browser, base, lambda s: (
            (s.get('safety') or {}).get('latched') is True and
            (s.get('safety') or {}).get('stop_acknowledged') is True and
            (s.get('mission') or {}).get('state') == 'ESTOP'),
            20., 'estop_acknowledged', trace)
        ready = wait_state(browser, base, lambda s: (
            ((s.get('localization') or {}).get('manual_pose') or {}).get(
                'available') is True), 25., 'manual_home_pose_available', trace)
        result['reentry']['pose_requested_at'] = time.time()
        request(browser, base, '/api/localization/initial-pose', {
            'x': float(pose['x']), 'y': float(pose['y']),
            'yaw_deg': math.degrees(float(pose['yaw'])), 'confirm': True})
        verified = wait_state(browser, base, lambda s: (
            ((s.get('localization') or {}).get('manual_pose') or {}).get(
                'verified') is True),
            50., 'amcl_and_health_acknowledged', trace)
        result['reentry']['pose_verified_at'] = time.time()
        result['reentry']['manual_pose_state'] = (
            ((verified.get('localization') or {}).get('manual_pose') or {}).get('state'))
        result['reentry']['after'] = {
            'localization_health': verified.get('localization_health'),
            'readiness': verified.get('health'),
            'pose': (verified.get('robot') or {}).get('pose')}
        if ((verified.get('safety') or {}).get('latched') is not True or
                (verified.get('safety') or {}).get('stop_acknowledged') is not True):
            raise RuntimeError('Pose verification unexpectedly released ESTOP')
        # The second verifier performs its own explicit web ESTOP reset and
        # checks the corresponding robot-side acknowledgment before departure.
        second_path = args.output.parent/'cycle_2'/'acceptance.json'
        second = run_dwell(base, second_path, args.mission_timeout, args.dispatch_mode)
        result['cycles'].append(second)
        result['runtime_identity_after_second'] = {
            key: process_identity(value['pid']) for key, value in identities.items()}
        result['same_runtime'] = all(value == result['runtime_identity_after_first'][key]
                                     == result['runtime_identity_after_second'][key]
                                     for key, value in identities.items())
        result['passed'] = bool(
            result['same_runtime'] and
            first.get('final_observed_at', math.inf) <
            second.get('mission_started_at', -math.inf) and
            first.get('startup_reset_completed') is True and
            second.get('startup_reset_completed') is True and
            result['reentry']['manual_pose_state'] == 'VERIFIED' and
            all(cycle.get('passed') is True and
                cycle.get('final_state') == 'HOME_REACHED' and
                cycle.get('completed_ids') == [2]
                for cycle in result['cycles']))
        if not result['passed']:
            result['failure'] = 'same_process_reentry_evidence_incomplete'
    except Exception as exc:
        result['failure'] = repr(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('PASS' if result['passed'] else 'FAIL', args.output, flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
