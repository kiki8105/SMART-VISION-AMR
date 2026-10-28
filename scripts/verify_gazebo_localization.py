#!/usr/bin/env python3
"""Explicit isolated simulation test only: domain220, localhost port8795."""
import json
from pathlib import Path
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

ROOT = Path(__file__).resolve().parents[1]
URL = 'http://127.0.0.1:8795'


def api(path='/api/state', payload=None):
    request = urllib.request.Request(URL+path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=4) as response:
        return json.load(response)


def main():
    assert api()['connection']['domain_id'] == 220, 'Only isolated Gazebo domain220 is permitted'
    context = Context()
    rclpy.init(args=[], context=context, domain_id=220, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('localization_test_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    done = threading.Event()
    samples, states, odom = [], [], []
    start = time.monotonic()
    node.create_subscription(Twist, '/cmd_vel', lambda m: samples.append(
        {'t': time.monotonic()-start, 'v': m.linear.x, 'w': m.angular.z}), 10)
    node.create_subscription(Odometry, '/odom', lambda m: odom.append(
        {'t': time.monotonic()-start, 'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y,
         'w': m.twist.twist.angular.z}), qos_profile_sensor_data)
    def spin():
        while not done.is_set():
            executor.spin_once(timeout_sec=.05)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    success, error = False, None
    try:
        before = api()
        assert before['localization']['available'], before['localization']
        route_before = {k: before['mission'][k] for k in ('route', 'current_index', 'completed_ids', 'home')}
        api('/api/localization/start', {})
        last = None
        deadline = time.monotonic()+75
        while time.monotonic() < deadline:
            state = api()
            assert state['connection']['domain_id'] == 220
            states.append({'t': time.monotonic()-start, 'localization': state['localization'],
                           'pose': state['robot']['pose'], 'mission_state': state['mission']['state']})
            if state['localization']['state'] != last:
                last = state['localization']['state']
                print(last, state['localization'], flush=True)
            if last in {'SUCCEEDED', 'FAILED', 'CANCELED'}:
                break
            time.sleep(.2)
        assert last == 'SUCCEEDED', state['localization']
        assert state['localization']['scan_match'] >= .90
        assert state['localization']['rotated_degrees'] >= 90
        assert state['mission']['state'] not in {'NAVIGATING', 'RETURNING_HOME'}
        assert route_before == {k: state['mission'][k] for k in route_before}
        assert samples and all(abs(s['v']) < .0001 for s in samples), 'Localization must never translate'
        assert any(s['w'] > .15 for s in samples), 'Actual angular commands required'
        assert odom and max(s['w'] for s in odom) > .10, 'Actual Gazebo rotation required'
        assert max(s['x'] for s in odom)-min(s['x'] for s in odom) < .12
        estimate = state['localization']['estimate']
        assert ((estimate['x']+4.)**2+(estimate['y']+2.)**2)**.5 < .25, 'Wrong map location despite low covariance'
        success = True
    except Exception as exc:
        error = repr(exc)
        print('FAIL', error, flush=True)
    finally:
        try:
            api('/api/localization/stop', {})
        except Exception:
            pass
        done.set()
        thread.join(2)
        node.destroy_node()
        executor.shutdown()
        context.shutdown()
        path = ROOT/'docs/gazebo_localization_e2e_20261027.json'
        path.write_text(json.dumps({'success': success, 'error': error, 'domain': 220,
            'real_vehicle_domain_16_touched': False, 'states': states,
            'commands': samples, 'odom': odom}, ensure_ascii=False, indent=2)+'\n')
        print('PASS' if success else 'FAIL', path, flush=True)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
