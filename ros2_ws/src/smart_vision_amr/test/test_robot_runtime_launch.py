"""Build runtime actions without launching hardware or resolving a real Pi."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]


def load_launch():
    specification = importlib.util.spec_from_file_location(
        'robot_runtime_launch', ROOT/'launch/hardware_robot_runtime.launch.py')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def make_context(tmp_path, mode='ekf_independent', **overrides):
    stock = tmp_path/'stock.yaml'
    # The repeated wildcard key in the vendor file must not erase the wheel
    # dimensions while the runtime copies values into its independent EKF.
    stock.write_text('''/**:
  turtlebot3_node:
    ros__parameters:
      wheels:
        separation: 0.321
        radius: 0.041
/**:
  diff_drive_controller:
    ros__parameters:
      odometry:
        publish_tf: true
        use_imu: true
''')
    context = LaunchContext()
    context.launch_configurations.update({
        'localization_mode': mode, 'use_sim_time': 'false',
        'enable_bringup': 'false', 'stock_params': str(stock),
        'wheel_radius': '', 'wheel_separation': '',
        'usb_port': '/dev/ttyACM-test', **overrides})
    return context, stock


def test_robot_runtime_has_one_mux_and_local_health_and_uses_pi_geometry(tmp_path, monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context, stock = make_context(tmp_path)
    original = stock.read_text()
    actions = module._runtime(context)
    nodes = {action['name']: action for action in actions}
    assert set(nodes) == {'wheel_joint_odometry', 'imu_ekf_conditioner', 'ekf_hardware',
                         'ekf_sensor_guard', 'motion_mux', 'localization_health'}
    wheel = nodes['wheel_joint_odometry']['parameters']
    assert wheel[-1] == {'wheel_radius': .041, 'wheel_separation': .321}
    assert wheel[0].endswith('/config/ekf_hardware.yaml')
    assert nodes['motion_mux']['parameters'][-1] == {
        'allow_legacy_auto_input': False, 'require_localization_health': True,
        'startup_estop': True, 'allow_topic_reset': False}
    assert nodes['ekf_hardware']['remappings'] == [('odometry/filtered', '/odom')]
    assert all(node['on_exit'] is not None for node in nodes.values())
    assert stock.read_text() == original


def test_baseline_runtime_has_no_duplicate_ekf_or_wheel_processing(tmp_path, monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context, _ = make_context(tmp_path, mode='baseline')
    assert {node['name'] for node in module._runtime(context)} == {
        'motion_mux', 'localization_health'}


def test_explicit_calibrated_dimensions_override_vendor_defaults(tmp_path, monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context, _ = make_context(tmp_path, wheel_radius='0.0327', wheel_separation='0.289')
    nodes = {node['name']: node for node in module._runtime(context)}
    assert nodes['wheel_joint_odometry']['parameters'][-1] == {
        'wheel_radius': .0327, 'wheel_separation': .289}


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', 'not-a-number'])
def test_invalid_calibrated_dimension_is_rejected(tmp_path, monkeypatch, value):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    context, _ = make_context(tmp_path, wheel_radius=value)
    with pytest.raises(ValueError):
        module._runtime(context)


@pytest.mark.parametrize('overrides', [
    {'localization_mode': 'unknown'}, {'use_sim_time': 'true'},
    {'enable_bringup': 'yes'},
])
def test_invalid_runtime_arguments_fail_before_starting_a_process(tmp_path, overrides):
    module = load_launch()
    context, _ = make_context(tmp_path, **overrides)
    with pytest.raises(ValueError):
        module._runtime(context)
