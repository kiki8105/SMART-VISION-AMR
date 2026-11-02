#!/usr/bin/env python3
"""Start one web mission, discard its browser session, and check it keeps driving.

This deliberately uses HTTP only. It must be run against a disposable domain
176 simulation/web server, never against the physical robot's domain 16.
"""

import argparse
import gc
import http.cookiejar
import json
import math
import os
from pathlib import Path
import time
from urllib.error import HTTPError, URLError
from urllib.request import HTTPCookieProcessor, Request, build_opener


def client():
    return build_opener(HTTPCookieProcessor(http.cookiejar.CookieJar()))


def request(opener, base, route, payload=None):
    body = None if payload is None else json.dumps(payload).encode()
    headers = {'Connection': 'close'}
    if body is not None:
        headers['Content-Type'] = 'application/json'
    query = Request(base+route, data=body, headers=headers,
                    method='GET' if body is None else 'POST')
    try:
        with opener.open(query, timeout=3) as response:
            result = json.load(response)
    except HTTPError as exc:
        result = json.loads(exc.read())
        raise RuntimeError(f'{route}: HTTP {exc.code}: {result.get("error", result)}') from exc
    if not isinstance(result, dict) or 'error' in result:
        raise RuntimeError(f'{route}: {result}')
    return result


def pair(opener, base):
    value = request(opener, base, '/api/pair', {'domain_id': 176})
    if value.get('authenticated') is not True:
        raise RuntimeError('Domain 176 web pairing failed')


def xy(state):
    pose = state.get('robot', {}).get('pose')
    if not isinstance(pose, dict):
        return None
    x, y = pose.get('x'), pose.get('y')
    return (float(x), float(y)) if isinstance(x, (int, float)) and isinstance(
        y, (int, float)) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8792')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--readiness-timeout', type=float, default=100.)
    parser.add_argument('--disconnect-seconds', type=float, default=25.)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') == '16' or not args.url.startswith(
            'http://127.0.0.1:8792'):
        parser.error('This test only contacts disposable loopback port 8792')
    result = {'passed': False, 'domain': 176, 'localhost_only': True,
              'physical_robot_touched': False, 'web_client_disconnected': False,
              'mission_accepted_via_web': False, 'route': [3],
              'connection_scope': 'browser HTTP client only; ROS and web stay up',
              'failure': None}
    try:
        base = args.url.rstrip('/')
        browser = client()
        deadline = time.monotonic()+args.readiness_timeout
        while True:
            try:
                pair(browser, base)
                break
            except (URLError, TimeoutError):
                if time.monotonic() >= deadline:
                    raise TimeoutError('Web server 8792 did not start')
                time.sleep(.5)
        request(browser, base, '/api/mission', {'route': [3]})
        state = None
        while time.monotonic() < deadline:
            state = request(browser, base, '/api/state')
            if (state.get('health', {}).get('ready') is True and
                    xy(state) is not None and state.get('connection', {}).get('connected') is True):
                break
            time.sleep(.5)
        else:
            raise TimeoutError('Web mission readiness did not become true: '+
                               str(state.get('health', {}).get('missing') if state else None))
        start_pose = xy(state)
        response = request(browser, base, '/api/mission/start', {})
        started = response.get('mission', {})
        if started.get('state') != 'NAVIGATING' or started.get('route') != [3]:
            raise RuntimeError('Web did not accept Sector 3 mission: '+str(started))
        result['mission_accepted_via_web'] = True
        result['start_pose'] = list(start_pose)
        result['state_on_acceptance'] = started.get('state')

        # The initiating browser is gone. No API requests or manual commands
        # are sent throughout this interval; only the ROS/web server runs.
        del browser
        gc.collect()
        result['web_client_disconnected'] = True
        result['disconnect_seconds'] = args.disconnect_seconds
        result['disconnect_start_at'] = time.time()
        time.sleep(args.disconnect_seconds)
        result['disconnect_end_at'] = time.time()

        # A separate, read-only observer connects *after* the dropout window.
        observer = client()
        pair(observer, base)
        state = request(observer, base, '/api/state')
        after_pose = xy(state)
        mission = state.get('mission', {})
        displacement = (math.hypot(after_pose[0]-start_pose[0],
                                   after_pose[1]-start_pose[1])
                        if after_pose is not None else None)
        result.update({'state_after_disconnect': mission.get('state'),
                       'mission_route_after_disconnect': mission.get('route'),
                       'position_after_disconnect': list(after_pose) if after_pose else None,
                       'displacement_m': displacement,
                       'path_points_after_disconnect': len(state.get('path', [])),
                       'navigation_after_disconnect': state.get('navigation')})
        result['passed'] = (mission.get('route') == [3] and
                            mission.get('state') in {'NAVIGATING', 'CONFIRMING',
                                                     'DWELLING', 'RETURNING_HOME',
                                                     'HOME_REACHED', 'COMPLETED'} and
                            displacement is not None and displacement >= .08 and
                            (len(state.get('path', [])) > 0 or
                             mission.get('state') in {'CONFIRMING', 'DWELLING',
                                                      'RETURNING_HOME', 'HOME_REACHED',
                                                      'COMPLETED'}))
        if not result['passed']:
            result['failure'] = 'Mission did not visibly continue after browser disconnect'
    except Exception as exc:
        result['failure'] = repr(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('PASS' if result['passed'] else 'FAIL', args.output, flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
