"""Regressions for physical EKF and direct-launch rollback; no robot is driven."""

import importlib.util
import math
from pathlib import Path

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, GroupAction, Shutdown,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap
import pytest
import yaml

from smart_vision_amr.imu_ekf_conditioner import StationaryGyroBias


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
PI_LAUNCH = PACKAGE/'launch/hardware_robot_ekf.launch.py'
PC_LAUNCH = PACKAGE/'launch/hardware.launch.py'
PREFLIGHT = REPO/'scripts/check_ekf_robot_preflight.py'


def pi_module():
    spec = importlib.util.spec_from_file_location('hardware_robot_ekf', PI_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def preflight_module():
    spec = importlib.util.spec_from_file_location('check_ekf_robot_preflight', PREFLIGHT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pc_module():
    spec = importlib.util.spec_from_file_location('hardware_ekf_pc', PC_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stock_params():
    return '''/**:
  turtlebot3_node:
    ros__parameters:
      opencr: {id: 200, baud_rate: 1000000}
      wheels: {separation: 0.287, radius: 0.033}
/**:
  diff_drive_controller:
    ros__parameters:
      odometry:
        publish_tf: true
        use_imu: true
        frame_id: odom
        child_frame_id: base_footprint
'''


def test_pi_derives_wheel_only_parameters_without_modifying_stock(tmp_path):
    source = tmp_path/'stock.yaml'
    target = tmp_path/'wheel.yaml'
    original = stock_params()
    source.write_text(original)
    pi_module()._write_wheel_only_params(source, target)
    assert source.read_text() == original
    assert target.read_text() == original.replace('publish_tf: true', 'publish_tf: false').replace(
        'use_imu: true', 'use_imu: false')
    assert 'baud_rate: 1000000' in target.read_text()


@pytest.mark.parametrize('change', [
    lambda text: text.replace('diff_drive_controller:', 'wrong_controller:'),
    lambda text: text.replace('use_imu: true', 'use_imu: false'),
    lambda text: text.replace('publish_tf: true', 'publish_tf: true\n        publish_tf: true'),
])
def test_pi_refuses_unexpected_upstream_odom_shape(tmp_path, change):
    source = tmp_path/'stock.yaml'
    target = tmp_path/'wheel.yaml'
    source.write_text(change(stock_params()))
    with pytest.raises(ValueError):
        pi_module()._write_wheel_only_params(source, target)
    assert not target.exists()


def test_pi_scopes_raw_odom_remap_to_stock_bringup(tmp_path, monkeypatch):
    source = tmp_path/'stock.yaml'
    source.write_text(stock_params())
    monkeypatch.setenv('TURTLEBOT3_MODEL', 'waffle_pi')
    monkeypatch.setenv('LDS_MODEL', 'LDS-03')
    context = LaunchContext()
    context.launch_configurations['stock_params'] = str(source)
    context.launch_configurations['usb_port'] = '/dev/ttyACM0'
    actions = pi_module()._launch_bringup(context)
    assert any(isinstance(item, RegisterEventHandler) for item in actions)
    group = next(item for item in actions if isinstance(item, GroupAction))
    children = group.get_sub_entities()
    remap = next(item for item in children if isinstance(item, SetRemap))
    assert perform_substitutions(context, remap._SetRemap__src) == 'odom'
    assert perform_substitutions(context, remap._SetRemap__dst) == '/wheel/odom_raw'
    include = next(item for item in children if isinstance(item, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    generated = Path(arguments['tb3_param_dir'])
    try:
        # TurtleBot3's upstream parameter file repeats /** for each node;
        # generic YAML loaders collapse that key, so check both sections as
        # preserved text instead of parsing with PyYAML here.
        assert 'turtlebot3_node:' in generated.read_text()
        assert 'diff_drive_controller:' in generated.read_text()
        assert 'use_imu: false' in generated.read_text()
        assert 'publish_tf: false' in generated.read_text()
    finally:
        generated.unlink(missing_ok=True)


def test_hardware_ekf_owns_only_odom_tf_and_nonoverlapping_sensor_axes():
    parameters = yaml.safe_load((PACKAGE/'config/ekf_hardware.yaml').read_text())[
        'ekf_hardware']['ros__parameters']
    assert parameters['world_frame'] == parameters['odom_frame'] == 'odom'
    assert parameters['base_link_frame'] == 'base_footprint'
    assert parameters['publish_tf'] is True
    assert parameters['reset_on_time_jump'] is True
    assert parameters['odom0'] == '/wheel/odom'
    assert parameters['imu0'] == '/imu/ekf'
    assert [i for i, enabled in enumerate(parameters['odom0_config']) if enabled] == [6, 7]
    assert [i for i, enabled in enumerate(parameters['imu0_config']) if enabled] == [11]
    assert parameters['sensor_timeout'] <= 0.5
    assert parameters['odom0_twist_rejection_threshold'] > 0
    assert parameters['imu0_twist_rejection_threshold'] > 0


def test_direct_hardware_launch_defaults_to_baseline_and_can_enable_ekf_nodes():
    module = pc_module()
    description = module.generate_launch_description()
    declaration = next(action for action in description.entities
                       if isinstance(action, DeclareLaunchArgument)
                       and action.name == 'localization_mode')
    assert perform_substitutions(LaunchContext(), declaration.default_value) == 'baseline'
    ekf_nodes = [action for action in description.entities
                 if isinstance(action, Node)
                 and action.node_executable in ('wheel_joint_odometry',
                                                'imu_ekf_conditioner', 'ekf_node',
                                                'ekf_sensor_guard')]
    assert {(node.node_package, node.node_executable) for node in ekf_nodes} == {
        ('smart_vision_amr', 'wheel_joint_odometry'),
        ('smart_vision_amr', 'imu_ekf_conditioner'),
        ('smart_vision_amr', 'ekf_sensor_guard'),
        ('robot_localization', 'ekf_node')}
    assert all(isinstance(node._ExecuteLocal__on_exit, Shutdown) for node in ekf_nodes)
    for mode, active in (('baseline', False), ('ekf_independent', True)):
        context = LaunchContext()
        context.launch_configurations['localization_mode'] = mode
        context.launch_configurations['estimation_location'] = 'pc'
        assert all(node.condition.evaluate(context) is active for node in ekf_nodes)
    invalid = LaunchContext()
    invalid.launch_configurations['localization_mode'] = 'hybrid'
    with pytest.raises(ValueError):
        module._validate_localization_mode(invalid)


def test_robot_graph_preflight_rejects_mixed_or_incomplete_modes():
    assess = preflight_module().assess_publishers
    ready = {'/wheel/odom_raw': 1, '/joint_states': 1, '/imu': 1,
             '/scan': 1, '/odom': 0}
    assert assess(ready) == []
    assert '/odom' in '; '.join(assess({**ready, '/odom': 1}))
    assert '/wheel/odom_raw' in '; '.join(assess({**ready, '/wheel/odom_raw': 0}))
    assert '/wheel/odom_raw' in '; '.join(assess({**ready, '/wheel/odom_raw': 2}))
    for topic in ('/joint_states', '/imu', '/scan'):
        assert topic in '; '.join(assess({**ready, topic: 0}))
        duplicate = '; '.join(assess({**ready, topic: 2}))
        assert topic in duplicate and 'found 2' in duplicate


def test_robot_preflight_requires_fresh_sensor_samples_and_matching_clocks():
    assess = preflight_module().assess_sensor_stamps
    now_ns = 10_000_000_000
    fresh = {'/joint_states': now_ns-200_000_000,
             '/imu': now_ns-100_000_000, '/scan': now_ns-300_000_000}
    assert assess(fresh, now_ns) == []
    assert '/scan: no sample' in '; '.join(assess({**fresh, '/scan': None}, now_ns))
    assert 'zero/invalid' in '; '.join(assess({**fresh, '/imu': 0}, now_ns))
    old = '; '.join(assess({**fresh, '/joint_states': now_ns-1_200_000_000}, now_ns))
    assert '/joint_states' in old and '+1.200s' in old and 'clock sync' in old
    future = '; '.join(assess({**fresh, '/imu': now_ns+500_000_000}, now_ns))
    assert '/imu' in future and '-0.500s' in future and 'clock sync' in future


def test_robot_preflight_battery_has_five_second_freshness_but_motion_keeps_one_second():
    preflight = preflight_module()
    now_ns = 20_000_000_000
    monitored, advancing = preflight.monitored_topics('robot', 'ekf_independent')
    assert '/battery_state' in monitored
    assert '/battery_state' not in advancing
    assert '/wheel/odom_raw' in advancing
    stamps = {topic: now_ns-100_000_000 for topic in monitored}
    stamps['/battery_state'] = now_ns-4_900_000_000
    assert preflight.assess_sensor_stamps(stamps, now_ns, topics=monitored) == []
    assert '/scan' in '; '.join(preflight.assess_sensor_stamps(
        {**stamps, '/scan': now_ns-1_100_000_000}, now_ns, topics=monitored))
    assert '/battery_state' in '; '.join(preflight.assess_sensor_stamps(
        {**stamps, '/battery_state': now_ns-5_100_000_000},
        now_ns, topics=monitored))


def test_robot_preflight_three_second_dwell_does_not_require_three_battery_messages():
    preflight = preflight_module()
    monitored, advancing = preflight.monitored_topics('robot', 'ekf_independent')
    window = preflight.StableSensorWindow(advancing, dwell_seconds=3.0)
    battery_stamp = 19_000_000_000
    for elapsed in (0.0, 1.0, 2.0, 3.1):
        now_ns = 20_000_000_000 + int(elapsed * 1e9)
        stamps = {topic: now_ns-100_000_000 for topic in advancing}
        stamps['/battery_state'] = battery_stamp
        reasons = preflight.assess_sensor_stamps(stamps, now_ns, topics=monitored)
        assert reasons == []
        assert window.observe(stamps, elapsed, reasons) is (elapsed == 3.1)
    assert '/battery_state' in '; '.join(preflight.assess_sensor_stamps(
        stamps, 24_100_000_000, topics=monitored))


def test_robot_local_preflight_rejects_stock_odom_in_ekf_mode():
    assess = preflight_module().assess_runtime_owners
    ready = {'/odom': ['ekf_hardware'], '/wheel/odom': ['wheel_joint_odometry'],
             '/imu/ekf': ['imu_ekf_conditioner'], '/cmd_vel': ['motion_mux'],
             '/smart_vision_amr/motion_status': ['motion_mux'],
             '/smart_vision_amr/localization_health': ['localization_health']}
    assert assess(ready) == []
    assert '/odom' in '; '.join(assess({**ready, '/odom': ['turtlebot3_node']}))
    assert '/cmd_vel' in '; '.join(assess({**ready, '/cmd_vel': ['motion_mux', 'legacy']}))
    assert assess({**ready, '/odom': ['turtlebot3_node']}, 'baseline') == []


def test_robot_preflight_requires_live_driver_node_and_usable_battery_sample():
    preflight = preflight_module()
    nodes = [(name, '/') for name in ('turtlebot3_node', 'lidar_node',
        'motion_mux', 'localization_health', 'wheel_joint_odometry',
        'imu_ekf_conditioner', 'ekf_hardware', 'ekf_sensor_guard')]
    assert preflight.assess_required_nodes(nodes) == []
    assert '/turtlebot3_node' in '; '.join(preflight.assess_required_nodes(nodes[1:]))
    assert '/ekf_sensor_guard' in '; '.join(preflight.assess_required_nodes(nodes[:-1]))
    assert preflight.valid_battery_voltage(10.8)
    assert preflight.valid_battery_voltage(10.8, minimum_voltage=10.8)
    assert preflight.valid_battery_voltage(10.80000019, minimum_voltage=10.8)
    assert not preflight.valid_battery_voltage(10.798, minimum_voltage=10.8)
    assert preflight.valid_battery_voltage(10.97, minimum_voltage=10.8)
    assert preflight.DEFAULT_MIN_BATTERY_VOLTAGE == 10.8
    for voltage in (0.0, -1.0, math.nan, math.inf):
        assert not preflight.valid_battery_voltage(voltage)
    assert not preflight.valid_battery_voltage(None, minimum_voltage=10.8)


def test_robot_preflight_needs_advancing_raw_stamps_for_full_healthy_dwell():
    preflight = preflight_module()
    topics = ('/joint_states', '/imu', '/scan', '/odom', '/battery_state')
    window = preflight.StableSensorWindow(topics, dwell_seconds=3.0)
    first = {topic: 10_000_000_000 for topic in topics}
    assert not window.observe(first, 0.0, [])
    assert not window.observe(first, 3.1, []), 'one fresh-looking sample is insufficient'
    second = {topic: 11_000_000_000 for topic in topics}
    assert not window.observe(second, 3.2, [])
    third = {topic: 12_000_000_000 for topic in topics}
    assert window.observe(third, 3.4, [])
    assert not window.observe(third, 3.5, ['/turtlebot3_node disappeared'])
    assert not window.observe(third, 6.8, []), 'the entire dwell restarts after graph loss'
    assert not window.observe({topic: 13_000_000_000 for topic in topics}, 7.0, [])
    assert window.observe({topic: 14_000_000_000 for topic in topics}, 10.0, [])


def test_robot_preflight_rejects_rewound_raw_stamp_and_missing_battery_topic():
    preflight = preflight_module()
    window = preflight.StableSensorWindow(('/joint_states', '/battery_state'), 2.0)
    assert not window.observe({'/joint_states': 200, '/battery_state': 200}, 0.0, [])
    assert not window.observe({'/joint_states': 300, '/battery_state': 300}, 1.0, [])
    assert not window.observe({'/joint_states': 100, '/battery_state': 400}, 2.0, [])
    assert 'window not started' in window.pending(2.0)
    reasons = preflight.assess_sensor_stamps(
        {'/joint_states': 4_900_000_000}, 5_000_000_000,
        topics=('/joint_states', '/battery_state'))
    assert '/battery_state: no sample received' in reasons


def test_stationary_gyro_calibration_reduces_bias_but_does_not_learn_motion():
    conditioner = StationaryGyroBias()
    for sample in range(25):
        now = sample * 0.1
        conditioner.update_wheel(0.0, 0.0, now)
        result = conditioner.condition(0.03, 0.0, now)
        if sample < 24:
            assert result is None
    assert result is not None
    corrected, variance = result
    assert math.isclose(corrected, 0.0, abs_tol=1e-8)
    assert variance >= 0.02**2
    assert math.isclose(conditioner.bias, 0.03)

    conditioner.update_wheel(0.2, 0.0, 2.5)
    moving = conditioner.condition(0.23, 0.01, 2.5)
    assert moving is not None
    assert math.isclose(moving[0], 0.20, abs_tol=1e-8)
    assert math.isclose(conditioner.bias, 0.03)
    assert conditioner.condition(0.03, 0.0, 3.0) is None  # stale wheel data
    conditioner.update_wheel(0.0, 0.0, 3.1)
    assert conditioner.condition(float('nan'), 0.0, 3.1) is None
    assert conditioner.condition(3.1, 0.0, 3.1) is None
