"""Read-only DDS owner discovery uses its existing four-second budget.

The actual graph-prearm callback loop is exercised with delayed endpoint names,
real raw-sample callbacks and an immediate fake parameter-read response. No ROS
context, publisher, lifecycle request or motion action is created.
"""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
import verify_classroom_web_dwell as dwell


@pytest.fixture
def discovery(monkeypatch):
    import rclpy
    clock = [0.]
    monkeypatch.setattr(dwell.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: None)

    def run(*, resolved_after=4, final_odom=('/classroom_fault_relay',),
            final_raw_tf=('/ekf_independent',), enabled=True):
        state = {'spins': 0, 'callbacks': {}, 'clients': [], 'destroyed': False}

        def info(name):
            if name == 'unknown':
                return SimpleNamespace(node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                                       node_name='_NODE_NAME_UNKNOWN_')
            return SimpleNamespace(node_namespace='/', node_name=name.lstrip('/'))

        def publishers(topic):
            names = {'/map': ['/map_server'], '/scan': ['/classroom_fault_relay'],
                     '/odom': list(final_odom), '/battery_state': ['/classroom_sim_battery'],
                     '/classroom_fault/raw_odom': ['/ekf_independent'],
                     '/classroom_fault/raw_tf': list(final_raw_tf),
                     '/wheel/odom': ['/wheel_joint_odometry'], '/imu': ['/turtlebot3_imu'],
                     '/smart_vision_amr/planner_status': ['/astar_planner']}.get(topic, [])
            if topic == '/odom' and state['spins'] < resolved_after:
                names = ['unknown']
            return [info(name) for name in names]

        class Probe:
            def create_subscription(self, _type, topic, callback, _qos):
                state['callbacks'][topic] = callback
            def get_publishers_info_by_topic(self, topic):
                return publishers(topic)
            def get_subscriptions_info_by_topic(self, topic):
                assert topic == '/smart_vision_amr/goal'
                return [info('/astar_planner'), info('/path_follower')]
            def get_service_names_and_types(self):
                return [('/smart_vision_amr/cancel_navigation', ['std_srvs/srv/Trigger'])]
            def create_client(self, _type, topic):
                state['clients'].append(topic)
                future = SimpleNamespace(done=lambda: True, exception=lambda: None,
                    result=lambda: SimpleNamespace(values=[SimpleNamespace(double_value=10.8)]))
                return SimpleNamespace(wait_for_service=lambda **_: True,
                                       call_async=lambda _: future)
            def destroy_node(self):
                state['destroyed'] = True

        node = Probe()
        monkeypatch.setattr(rclpy, 'create_node', lambda _name: node)
        def spin_once(received_node, *, timeout_sec):
            assert received_node is node and timeout_sec == .25
            clock[0] += timeout_sec
            state['spins'] += 1
            for callback in state['callbacks'].values():
                callback(object())
        monkeypatch.setattr(rclpy, 'spin_once', spin_once)
        monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *_a, **_k: None)
        result = dwell.graph_prearm(ekf_delivery_faults=enabled)
        assert state['destroyed']
        assert state['clients'] == ['/ekf_sensor_guard/get_parameters']
        assert result['guard_battery_minimum_v'] == 10.8
        return result, state, clock[0]
    return run


def test_unknown_endpoint_names_wait_for_actual_relay_within_existing_budget(discovery):
    graph, state, elapsed = discovery()
    assert state['spins'] == 4 and elapsed == 1.
    assert dwell._ekf_output_relay_graph_ready(graph)
    assert graph['raw_input_samples']['/wheel/odom']['count'] == 4
    assert graph['raw_input_samples']['/imu']['count'] == 4


def test_default_scan_only_discovery_keeps_existing_early_completion(discovery):
    graph, state, elapsed = discovery(resolved_after=100, enabled=False)
    assert state['spins'] == 2 and elapsed == .5
    assert graph['odom_publishers'] == ['_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_']


def test_unknown_owners_exhaust_four_seconds_and_remain_rejected(discovery):
    graph, state, elapsed = discovery(resolved_after=100)
    assert state['spins'] == 16 and elapsed == 4.
    assert not dwell._ekf_output_relay_graph_ready(graph)


@pytest.mark.parametrize('kwargs', [
    {'final_odom': ('/ekf_independent',)},
    {'final_odom': ('/classroom_fault_relay', '/ekf_independent')},
    {'final_raw_tf': ('/other_tf_producer',)},
    {'final_raw_tf': ('/ekf_independent', '/other_tf_producer')},
])
def test_wrong_or_duplicate_ownership_never_becomes_permission(discovery, kwargs):
    graph, _, elapsed = discovery(resolved_after=0, **kwargs)
    assert elapsed == 4.
    assert not dwell._ekf_output_relay_graph_ready(graph)
