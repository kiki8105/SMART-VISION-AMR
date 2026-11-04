"""Exercise opt-in simulation graph and cold-start proof without launching ROS."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

import pytest
from geometry_msgs.msg import TransformStamped
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import assess_classroom_fault_delivery as delivery
import classroom_fault_relay as relay
import run_classroom_staged_validation as staged
import verify_classroom_web_dwell as dwell
from test_sim_ekf_parity_launch import simulation, context_for, parameters


@pytest.mark.parametrize('enabled', [False, True])
def test_only_actual_ekf_process_moves_to_relay_and_guard_checks_real_owner(
        enabled, tmp_path, monkeypatch):
    module = simulation(tmp_path, monkeypatch)
    description = module.generate_launch_description()
    context = context_for(description, localization_mode='ekf_independent',
                          use_ekf_sensor_guard='true',
                          ekf_delivery_faults=str(enabled).lower())
    module._validate_localization_mode(context)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    ekf = next(node for node in nodes if node.node_executable == 'ekf_node'
               and node.condition.evaluate(context))
    remaps = {perform_substitutions(context, source): perform_substitutions(context, target)
              for source, target in ekf._Node__remappings}
    assert remaps == {'odometry/filtered': '/classroom_fault/raw_odom' if enabled else '/odom',
                      '/tf': '/classroom_fault/raw_tf' if enabled else '/tf'}
    for node in nodes:
        if node is not ekf:
            assert all('raw_tf' not in perform_substitutions(context, target)
                       for _, target in (node._Node__remappings or []))
    guard = next(node for node in nodes if node.node_executable == 'ekf_sensor_guard')
    assert guard.condition.evaluate(context)
    expected = next(value['expected_ekf_odom_publisher']
                    for value in parameters(context, guard)
                    if isinstance(value, dict) and 'expected_ekf_odom_publisher' in value)
    assert expected == ('classroom_fault_relay' if enabled else 'ekf_independent')
    assert '/tf_static' not in remaps


def test_ekf_delivery_cannot_be_misapplied_to_baseline(tmp_path, monkeypatch):
    module = simulation(tmp_path, monkeypatch)
    description = module.generate_launch_description()
    context = context_for(description, ekf_delivery_faults='true')
    with pytest.raises(ValueError, match='requires localization_mode'):
        module._validate_localization_mode(context)


def test_relay_scope_and_tf_filter_preserve_source_identity_and_stamp():
    assert set(relay.relay_topics(scan_only=True)) == {'scan'}
    assert set(relay.relay_topics(ekf_delivery_faults=True)) == {'scan', 'odom', 'tf'}
    with pytest.raises(ValueError, match='mutually exclusive'):
        relay.relay_topics(scan_only=True, ekf_delivery_faults=True)
    owned = TransformStamped()
    owned.header.frame_id = 'odom'
    owned.child_frame_id = 'base_footprint'
    owned.header.stamp.sec, owned.header.stamp.nanosec = 23, 987654321
    map_tf = TransformStamped()
    map_tf.header.frame_id, map_tf.child_frame_id = 'map', 'odom'
    static_edge = TransformStamped()
    static_edge.header.frame_id, static_edge.child_frame_id = 'base_link', 'base_scan'
    original = TFMessage(transforms=[map_tf, owned, static_edge])
    selected = relay.ekf_odom_transforms(original)
    assert selected == [owned] and selected[0] is owned
    assert (owned.header.stamp.sec, owned.header.stamp.nanosec) == (23, 987654321)
    assert original.transforms == [map_tf, owned, static_edge]


def test_ekf_output_fault_acceptance_requires_all_three_actual_deliveries():
    events = [{'event': 'profile', 'name': 'mixed'},
              {'event': 'sample', 'profile': 'mixed',
               'counts': {'scan_forwarded': 5, 'scan_dropped_outage': 4},
               'max_delivery_delay_s': {'scan': .35}}]
    assert delivery.assess(events, 'mixed', 'ekf_independent')['passed']
    result = delivery.assess(events, 'mixed', 'ekf_independent', ekf_delivery_faults=True)
    assert not result['passed']
    assert 'odom_not_forwarded' in result['missing'] and 'tf_outage_not_measured' in result['missing']
    sample = events[-1]
    for stream in ('odom', 'tf'):
        sample['counts'].update({stream+'_forwarded': 5, stream+'_dropped_outage': 4})
        sample['max_delivery_delay_s'][stream] = .35
    assert delivery.assess(events, 'mixed', 'ekf_independent', ekf_delivery_faults=True)['passed']


def test_web_stages_wire_explicit_realism_without_changing_default_independent(tmp_path):
    default = staged.BY_KEY['25_web_inventory_sector3_moving_obstacle']
    nominal = staged.web_stage_runner_command(default, tmp_path/'nominal')
    assert '--cold-start' not in nominal and '--ekf-delivery-faults' not in nominal
    candidate = replace(default, options=(*default.options, '--cold-start',
        '--ekf-delivery-faults', '--hardware-profile', 'sensor_rates'))
    command = staged.web_stage_runner_command(candidate, tmp_path/'candidate')
    assert command[command.index('--mode')+1] == 'ekf_independent'
    assert command.count('--ekf-delivery-faults') == command.count('--cold-start') == 1
    assert command[command.index('--hardware-profile')+1] == 'sensor_rates'
    transient = staged.web_stage_runner_command(
        staged.BY_KEY['16_web_transient_resume'], tmp_path/'transient')
    assert '--ekf-delivery-faults' in transient


@pytest.mark.parametrize('arguments,message', [
    (['--cold-start'], 'external web verifier'),
    (['--ekf-delivery-faults'], 'requires --mode ekf_independent'),
])
def test_runner_rejects_incompatible_scope_before_starting_process(arguments, message):
    result = subprocess.run(['bash', str(ROOT/'scripts/run_classroom_fault_sim.sh'),
                             *arguments], capture_output=True, text=True)
    assert result.returncode == 2 and message in result.stderr


def test_cold_start_requires_genuine_missing_prior_and_stopped_ack():
    state = {'robot': {'pose': None},
             'safety': {'latched': True, 'stop_acknowledged': True}}
    sample = {'amcl_set_initial_pose': False, 'initial_pose_messages': 0,
              'amcl_messages': 0, 'map_tf_messages': 0}
    assert dwell.cold_start_unseeded(state, sample)
    for key, value in [('amcl_set_initial_pose', True), ('initial_pose_messages', 1),
                       ('amcl_messages', 1), ('map_tf_messages', 1)]:
        assert not dwell.cold_start_unseeded(state, {**sample, key: value})
    assert not dwell.cold_start_unseeded({**state, 'robot': {'pose': {'x': 0.}}}, sample)
    assert not dwell.cold_start_unseeded({**state, 'safety': {'latched': True}}, sample)


@pytest.mark.parametrize('already_seeded,restore_tf', [(False, True), (True, True), (False, False)])
def test_cold_start_observer_requires_actual_post_manual_amcl_and_tf(
        already_seeded, restore_tf, monkeypatch):
    import rclpy
    import rclpy.node
    from rcl_interfaces.msg import ParameterValue, ParameterType
    clock = [10.]
    callbacks = {}
    state = {'robot': {'pose': None},
             'safety': {'latched': True, 'stop_acknowledged': True},
             'localization': {'manual_pose': {'available': True}},
             'delivery': {'registered_homepoint': {'x': -.18, 'y': -.22, 'yaw': 0.}}}
    initialized = [False]
    calls = []
    value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=already_seeded)
    future = SimpleNamespace(done=lambda: True, exception=lambda: None,
                             result=lambda: SimpleNamespace(values=[value]))

    class Probe:
        def __init__(self, _name):
            pass
        def create_subscription(self, _type, topic, callback, _qos):
            callbacks[topic] = callback
        def create_client(self, _type, _topic):
            return SimpleNamespace(wait_for_service=lambda **_: True,
                                   call_async=lambda _: future)
        def destroy_node(self):
            pass

    def spin(_node, **_kwargs):
        clock[0] += .1
        if initialized[0]:
            callbacks['/amcl_pose'](object())
            if restore_tf:
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
                callbacks['/tf'](TFMessage(transforms=[transform]))

    def request(_browser, _base, endpoint, payload=None):
        calls.append(endpoint)
        if endpoint == '/api/localization/initial-pose':
            initialized[0] = True
            state['localization']['manual_pose'] = {'state': 'VERIFIED', 'verified': True}
            state['robot']['pose'] = dict(state['delivery']['registered_homepoint'])
        return state

    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(rclpy, 'spin_once', spin)
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *_, **__: None)
    monkeypatch.setattr(rclpy.node, 'Node', Probe)
    monkeypatch.setattr(dwell.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(dwell, 'request', request)
    result = {}
    if already_seeded or not restore_tf:
        with pytest.raises(RuntimeError):
            dwell.verify_cold_start_home_pose(object(), 'http://127.0.0.1:8792', result, timeout=2.)
        assert result['cold_start_proof']['passed'] is False
    else:
        dwell.verify_cold_start_home_pose(object(), 'http://127.0.0.1:8792', result, timeout=2.)
        assert result['cold_start_proof']['passed']
        assert result['cold_start_proof']['before']['amcl_messages'] == 0
        assert result['cold_start_proof']['after']['amcl_messages'] > 0
        assert result['cold_start_proof']['after']['initial_pose_messages'] == 0
    if already_seeded:
        assert '/api/localization/initial-pose' not in calls
