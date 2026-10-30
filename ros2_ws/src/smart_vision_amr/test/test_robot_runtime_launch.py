"""Build runtime actions without launching hardware or resolving a real Pi."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess, GroupAction, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.events import Shutdown as ShutdownEvent
from launch.events.process import ProcessExited
from launch.utilities import perform_substitutions


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
        'timing_diagnostics': 'false',
        'minimum_battery_voltage': '0.0',
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
    assert {'enable_timing_diagnostics': False} in nodes['motion_mux']['parameters']
    assert {'enable_timing_diagnostics': False} in nodes['ekf_sensor_guard']['parameters']
    assert {'minimum_battery_voltage': 0.0} in nodes['ekf_sensor_guard']['parameters']


def test_physical_runtime_passes_battery_limit_only_to_ekf_guard(tmp_path, monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context, _ = make_context(tmp_path, minimum_battery_voltage='10.8')
    nodes = {node['name']: node for node in module._runtime(context)}
    assert {'minimum_battery_voltage': 10.8} in nodes['ekf_sensor_guard']['parameters']
    assert all({'minimum_battery_voltage': 10.8} not in node['parameters']
               for name, node in nodes.items() if name != 'ekf_sensor_guard')


@pytest.mark.parametrize('value', ['-0.1', 'nan', 'inf', 'bad'])
def test_physical_runtime_rejects_invalid_battery_limit(tmp_path, monkeypatch, value):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    context, _ = make_context(tmp_path, minimum_battery_voltage=value)
    with pytest.raises(ValueError):
        module._runtime(context)


def test_timing_flag_only_enables_observations_in_mux_and_guard(tmp_path, monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context, _ = make_context(tmp_path, timing_diagnostics='true')
    nodes = {node['name']: node for node in module._runtime(context)}
    enabled = {'enable_timing_diagnostics': True}
    assert enabled in nodes['motion_mux']['parameters']
    assert enabled in nodes['ekf_sensor_guard']['parameters']
    assert all(enabled not in node['parameters'] for name, node in nodes.items()
               if name not in {'motion_mux', 'ekf_sensor_guard'})


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
    {'timing_diagnostics': 'yes'},
])
def test_invalid_runtime_arguments_fail_before_starting_a_process(tmp_path, overrides):
    module = load_launch()
    context, _ = make_context(tmp_path, **overrides)
    with pytest.raises(ValueError):
        module._runtime(context)


def required_exit_handler():
    module = load_launch()
    description = module.generate_launch_description()
    actions = description.entities
    registration = next(action for action in actions if isinstance(action, RegisterEventHandler))
    runtime_group = next(action for action in actions if isinstance(action, GroupAction))
    # The handler exists before OpaqueFunction can expand any included stock
    # launch, including a driver that crashes immediately on startup.
    assert actions.index(registration) < actions.index(runtime_group)
    marker = next(action for action in runtime_group.get_sub_entities()
                  if isinstance(action, SetEnvironmentVariable))
    context = LaunchContext()
    environment = {perform_substitutions(context, marker.name):
                   perform_substitutions(context, marker.value)}
    assert module.RUNTIME_INSTANCE_ENV in environment
    return registration.event_handler, context, environment


def process_exit(name, code, environment):
    # A real launch ProcessExited event, without executing any command or
    # creating a ROS participant. Its action can represent a nested driver.
    command = ['/fake/'+name]
    return ProcessExited(action=ExecuteProcess(cmd=command), name=name+'-1',
                         cmd=command, cwd=None, env=environment, pid=12345,
                         returncode=code)


@pytest.mark.parametrize('name', ['turtlebot3_ros', 'ld08_driver', 'robot_state_publisher',
                                 'ekf_hardware', 'motion_mux'])
@pytest.mark.parametrize('code', [-6, 0])
def test_unexpected_required_process_exit_stops_entire_robot_runtime(name, code):
    handler, context, environment = required_exit_handler()
    event = process_exit(name, code, environment)
    assert handler.matches(event)
    result = handler.handle(event, context)
    assert len(result) == 1 and isinstance(result[0], Shutdown)
    shutdown = result[0].event
    assert isinstance(shutdown, ShutdownEvent)
    assert name in shutdown.reason
    assert f'code {code}' in shutdown.reason


def test_required_process_exit_during_normal_shutdown_does_not_restart_shutdown():
    handler, context, environment = required_exit_handler()
    context._set_is_shutdown(True)
    event = process_exit('turtlebot3_ros', -2, environment)
    assert handler.handle(event, context) == []


@pytest.mark.parametrize('environment', [None, {}, {'SMART_VISION_REQUIRED_RUNTIME_INSTANCE': 'other-runtime'}])
def test_unrelated_process_exit_does_not_stop_robot_runtime(environment):
    handler, context, _ = required_exit_handler()
    event = process_exit('unrelated-camera-or-tool', -6, environment)
    assert handler.handle(event, context) == []


def test_runtime_ownership_marker_is_inherited_by_children_and_scoped_from_parent(monkeypatch):
    module = load_launch()
    monkeypatch.setenv(module.RUNTIME_INSTANCE_ENV, 'parent-runtime')
    description = module.generate_launch_description()
    group = next(action for action in description.entities if isinstance(action, GroupAction))
    context = LaunchContext()
    child_environment = None
    for action in group.execute(context):
        if isinstance(action, OpaqueFunction):
            # Expand only the fake executable's environment. No process or ROS
            # node starts, and the real hardware _runtime function is not run.
            child = ExecuteProcess(cmd=['/fake/required-driver'])
            child.process_description.prepare(context, child)
            child_environment = child.env
        else:
            action.execute(context)
    assert child_environment[module.RUNTIME_INSTANCE_ENV] != 'parent-runtime'
    sibling = ExecuteProcess(cmd=['/fake/unrelated-tool'])
    sibling.process_description.prepare(context, sibling)
    assert sibling.env[module.RUNTIME_INSTANCE_ENV] == 'parent-runtime'
