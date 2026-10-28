#!/usr/bin/env python3
"""Actual Gazebo handover smoke test: fixed isolated ROS 220 and HTTP 8795 only."""
import json
from pathlib import Path
import threading
import time
import urllib.request
from uuid import uuid4

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/gazebo_joystick_e2e_20261027.json'
BASE = 'http://127.0.0.1:8795'


def api(path, data=None):
    request = urllib.request.Request(BASE+path, data=None if data is None else json.dumps(data).encode(),
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=4) as response:
        return json.load(response)


def main():
    assert api('/api/state')['connection']['domain_id'] == 220, 'Isolated simulation server required'
    context = Context()
    rclpy.init(args=[], context=context, domain_id=220, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('gazebo_joystick_e2e_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    done = threading.Event()
    start = time.monotonic()
    phase = ['ready']
    records = {'commands': [], 'auto_commands': [], 'odom': [], 'goals': [], 'navigation': [], 'mux': []}

    def record(kind, values):
        records[kind].append({'t': round(time.monotonic()-start, 4), 'phase': phase[0], **values})
    node.create_subscription(Twist, '/cmd_vel', lambda m: record('commands', {'v': m.linear.x, 'w': m.angular.z}), 10)
    node.create_subscription(Twist, '/smart_vision_amr/navigation_cmd_vel',
                             lambda m: record('auto_commands', {'v': m.linear.x, 'w': m.angular.z}), 10)
    node.create_subscription(Odometry, '/odom', lambda m: record('odom', {
        'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y,
        'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z}), qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/smart_vision_amr/goal', lambda m: record('goals', {
        'x': m.pose.position.x, 'y': m.pose.position.y,
        'stamp': [m.header.stamp.sec, m.header.stamp.nanosec]}), 10)
    for kind, topic in (('navigation', '/smart_vision_amr/navigation_status'), ('mux', '/smart_vision_amr/motion_status')):
        node.create_subscription(String, topic, lambda m, k=kind: record(k, json.loads(m.data)), 10)
    def spin():
        while not done.is_set():
            executor.spin_once(timeout_sec=.05)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    checkpoints = {}
    def wait_for(predicate, seconds, label):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            state = api('/api/state')
            assert state['connection']['domain_id'] == 220
            if predicate(state):
                checkpoints[label] = state
                print(label, state['mission']['state'], state['manual']['state'], state['robot']['pose'], flush=True)
                return state
            time.sleep(.1)
        raise AssertionError(f'{label} timed out: '+json.dumps(state, ensure_ascii=False))
    success, error = False, None
    try:
        wait_for(lambda s: s['health']['ready'] and s['manual']['available'], 30, 'ready')
        api('/api/home/set', {})
        api('/api/mission', {'route': [0]})
        phase[0] = 'auto_before'
        api('/api/mission/start', {})
        baseline = wait_for(lambda s: s['navigation']['status'] == 'MOVING' and s['robot']['speed']['linear'] > .02,
                            20, 'automatic_motion')
        control_id, seq = str(uuid4()), 0
        def joystick(active, linear=0., angular=0.):
            nonlocal seq
            seq += 1
            return api('/api/manual', {'control_id': control_id, 'seq': seq, 'active': active,
                                       'linear': linear, 'angular': angular})['manual']
        phase[0] = 'acquiring'
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            manual = joystick(True)
            if manual['state'] == 'ACTIVE':
                break
            time.sleep(.1)
        assert manual['state'] == 'ACTIVE', manual
        phase[0] = 'manual'
        deadline = time.monotonic()+1.2
        while time.monotonic() < deadline:
            joystick(True, -.05, .25)
            time.sleep(.1)
        held = api('/api/state')
        checkpoints['manual_motion'] = held
        for key in ('route', 'current_index', 'completed_ids', 'home'):
            assert held['mission'][key] == baseline['mission'][key], key
        assert held['mission']['state'] == 'PAUSED'
        assert any(item['phase'] == 'manual' and item['v'] < -.04 and item['w'] > .2 for item in records['commands'])
        assert all(item['v'] <= .001 for item in records['commands'] if item['phase'] == 'manual'), 'Auto speed leaked into manual'
        phase[0] = 'release'
        joystick(False)
        resumed = wait_for(lambda s: s['manual']['state'] == 'IDLE' and s['navigation']['status'] == 'MOVING'
                           and s['robot']['speed']['linear'] > .02, 30, 'automatic_resumed')
        for key in ('route', 'current_index', 'completed_ids', 'home'):
            assert resumed['mission'][key] == baseline['mission'][key], key
        phase[0] = 'auto_after'
        final = wait_for(lambda s: s['mission']['state'] == 'HOME_REACHED', 90, 'mission_finished')
        assert final['mission']['completed_ids'] == [0], 'Actual ArUco confirmation required'
        assert final['mission']['current_index'] == 1
        target = baseline['markers'][0]
        goal_samples = [g for g in records['goals'] if abs(g['x']-target['x']) < .01 and abs(g['y']-target['y']) < .01]
        assert len({tuple(g['stamp']) for g in goal_samples}) >= 2, 'Same target was not reissued with a fresh stamp'
        success = True
    except Exception as exc:
        error = repr(exc)
        print('FAILED', error, flush=True)
        try:
            api('/api/emergency-stop', {})
        except Exception:
            pass
    finally:
        done.set()
        thread.join(2)
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        context.shutdown()
        report = {'date': '2026-10-27', 'simulation': 'Gazebo Classic / measured ArUco maze',
                  'domain': 220, 'http_port': 8795, 'real_vehicle_domain_16_touched': False,
                  'success': success, 'error': error, 'duration_seconds': time.monotonic()-start,
                  'checkpoints': checkpoints, 'records': records}
        OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print('PASS' if success else 'FAIL', OUT, flush=True)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
