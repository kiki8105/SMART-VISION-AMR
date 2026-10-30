"""Evaluate real launch scopes without executing any process or ROS context."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE = Path(__file__).resolve().parents[1]
RELIABILITY = 'qos_overrides./tf.subscription.reliability'
DEPTH = 'qos_overrides./tf.subscription.depth'
FLAG = 'nav2_dynamic_tf_best_effort'


def module_for(name, tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('tf_launch_'+name, PACKAGE/'launch'/f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if name == 'simulation':
        stock = tmp_path/'turtlebot3_gazebo'
        (stock/'urdf').mkdir(parents=True)
        (stock/'urdf/turtlebot3_waffle_pi.urdf').write_text('<robot name="test"/>')
        shares = {'smart_vision_amr': PACKAGE, 'turtlebot3_gazebo': stock,
                  'gazebo_ros': tmp_path/'gazebo_ros'}
        monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(shares[package]))
    return module


def configured(description, value=None, **arguments):
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    if value is not None:
        context.launch_configurations[FLAG] = value
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.visit(context)
    return context


def inspect_scope(actions, context, records):
    """Execute only launch configuration actions; never visit Nodes/includes."""
    for action in actions:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, (Node, IncludeLaunchDescription)):
            records.append((action, deepcopy(context.launch_configurations.get('global_params', []))))
        elif isinstance(action, GroupAction):
            inspect_scope(action.get_sub_entities(), context, records)
        else:
            action.visit(context)


@pytest.mark.parametrize('name', ['hardware', 'simulation'])
def test_both_launches_expose_default_and_reversible_tf_policy(name, tmp_path, monkeypatch):
    description = module_for(name, tmp_path, monkeypatch).generate_launch_description()
    declarations = [action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == FLAG]
    assert len(declarations) == 1
    assert perform_substitutions(LaunchContext(), declarations[0].default_value) == 'true'


@pytest.mark.parametrize('enabled', ['true', 'false'])
@pytest.mark.parametrize('inherit', [False, True])
def test_hardware_tf_overrides_reach_only_nav2_include_and_preserve_parent_scope(
        enabled, inherit, tmp_path, monkeypatch):
    description = module_for('hardware', tmp_path, monkeypatch).generate_launch_description()
    context = configured(description, enabled, map=str(PACKAGE/'maps/classroom_map.yaml'),
                         params_file=str(PACKAGE/'config/curve_tuning_waffle_pi.yaml'),
                         nodes_file=str(PACKAGE/'config/nodes.yaml'))
    inherited = [('caller_parameter', 42)] if inherit else []
    if inherit:
        context.launch_configurations['global_params'] = inherited
    records = []
    # Skip root OpaqueFunction validators; only traverse the existing Nav2
    # group. Other root Nodes are observed without ever executing them.
    for action in description.entities:
        if isinstance(action, (Node, GroupAction)):
            inspect_scope([action], context, records)
    include = [(action, params) for action, params in records if isinstance(action, IncludeLaunchDescription)]
    assert len(include) == 1
    effective = dict(include[0][1])
    if enabled == 'true':
        assert effective[RELIABILITY] == 'best_effort' and effective[DEPTH] == 20
    else:
        assert RELIABILITY not in effective and DEPTH not in effective
    assert effective['speed_limit_topic'] == '/speed_limit'
    assert all('/tf_static' not in key for key in effective)
    for node, params in records:
        if isinstance(node, Node):
            assert dict(params) == dict(inherited), node.node_executable
    assert inherited == ([('caller_parameter', 42)] if inherit else [])
    assert context.launch_configurations.get('global_params', []) == inherited


@pytest.mark.parametrize('enabled', ['true', 'false'])
@pytest.mark.parametrize('slam', ['false', 'true'])
def test_simulation_overrides_only_scoped_nav2_listeners_and_respects_slam_condition(
        enabled, slam, tmp_path, monkeypatch):
    description = module_for('simulation', tmp_path, monkeypatch).generate_launch_description()
    context = configured(description, enabled, slam=slam)
    inherited = [('caller_parameter', 42)]
    context.launch_configurations['global_params'] = inherited
    selected = []
    for action in description.entities:
        if isinstance(action, (Node, GroupAction)):
            inspect_scope([action], context, selected)
    targets = {'amcl', 'controller_server', 'behavior_server'}
    seen = set()
    for node, params in selected:
        assert isinstance(node, Node)
        effective = dict(params)
        assert effective['caller_parameter'] == 42
        assert all('/tf_static' not in key for key in effective)
        if node.node_executable in targets:
            seen.add(node.node_executable)
            if enabled == 'true':
                assert effective[RELIABILITY] == 'best_effort' and effective[DEPTH] == 20
            else:
                assert set(effective) == {'caller_parameter'}
            # The original node arguments remain: real-time model is untouched,
            # AMCL's start hypothesis and controller remaps are not recreated.
            values = evaluate_parameters(context, node._Node__parameters)
            assert any(isinstance(value, dict) and value.get('use_sim_time') is True for value in values)
            if node.node_executable == 'amcl':
                assert any(isinstance(value, dict) and value.get('set_initial_pose') is True for value in values)
        else:
            assert set(effective) == {'caller_parameter'}, node.node_executable
    assert seen == (targets if slam == 'false' else set())
    assert context.launch_configurations['global_params'] == inherited == [('caller_parameter', 42)]


def test_disabled_scope_preserves_an_explicit_site_qos_instead_of_rewriting_defaults(tmp_path, monkeypatch):
    module = module_for('simulation', tmp_path, monkeypatch)
    node = Node(package='nav2_amcl', executable='amcl')
    context = LaunchContext()
    context.launch_configurations[FLAG] = 'false'
    site = [(RELIABILITY, 'reliable'), (DEPTH, 100), ('site_parameter', 'keep')]
    context.launch_configurations['global_params'] = site
    records = []
    inspect_scope([module._nav2_dynamic_tf_group(node)], context, records)
    assert records == [(node, site)]
    assert context.launch_configurations['global_params'] == site


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_simulated_home_boot_can_wait_for_real_home_action_without_changing_tf_scope(
        enabled, tmp_path, monkeypatch):
    description = module_for('simulation', tmp_path, monkeypatch).generate_launch_description()
    declarations = [action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == 'set_initial_pose']
    assert len(declarations) == 1
    assert perform_substitutions(LaunchContext(), declarations[0].default_value) == 'true'
    context = configured(description, set_initial_pose=enabled)
    records = []
    for action in description.entities:
        if isinstance(action, (Node, GroupAction)):
            inspect_scope([action], context, records)
    amcl = [(node, parameters) for node, parameters in records if node.node_executable == 'amcl']
    assert len(amcl) == 1
    node, parameters = amcl[0]
    values = evaluate_parameters(context, node._Node__parameters)
    overrides = next(value for value in values if isinstance(value, dict) and 'set_initial_pose' in value)
    assert type(overrides['set_initial_pose']) is bool
    assert overrides['set_initial_pose'] is (enabled == 'true')
    assert overrides['initial_pose.x'] == -3. and overrides['initial_pose.y'] == -2.
    assert overrides['initial_pose.yaw'] == 0.
    assert dict(parameters)[RELIABILITY] == 'best_effort'
    assert dict(parameters)[DEPTH] == 20


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_amcl_real_parameter_resolution_overrides_node_specific_yaml(
        enabled, tmp_path, monkeypatch):
    """Parse actual generated launch files with RCL, never start AMCL.

    The only Node is an empty parameter reader in an explicit localhost domain
    218 Context. It has no timers or actuation/pose publishers and no executor.
    This catches /** dictionary overrides losing to the amcl: YAML section.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node as ParameterReader
    from rclpy.signals import SignalHandlerOptions

    description = module_for('simulation', tmp_path, monkeypatch).generate_launch_description()
    context = configured(description, set_initial_pose=enabled,
                         initial_x='1.25', initial_y='-1.75', initial_yaw='0.4')
    records = []
    for action in description.entities:
        if isinstance(action, (Node, GroupAction)):
            inspect_scope([action], context, records)
    action, scope = next((node, parameters) for node, parameters in records
                         if node.node_executable == 'amcl')
    context.launch_configurations['global_params'] = scope
    action._perform_substitutions(context)  # Builds args/files; executes nothing.
    arguments = []
    generated = []
    for parameter, is_file in action._Node__expanded_parameter_arguments:
        assert is_file
        path = Path(parameter)
        arguments.extend(['--params-file', str(path)])
        if path != PACKAGE/'config/simulation.yaml':
            generated.append(path)
    overrides = [yaml.safe_load(path.read_text()) for path in generated]
    assert any('/amcl' in data and data['/amcl']['ros__parameters'].get('set_initial_pose')
               is (enabled == 'true') for data in overrides)
    ros_context = Context()
    reader = None
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    try:
        rclpy.init(args=[], context=ros_context, domain_id=218,
                   signal_handler_options=SignalHandlerOptions.NO)
        reader = ParameterReader('amcl', namespace='/', context=ros_context,
            cli_args=['--ros-args', *arguments], use_global_arguments=False,
            automatically_declare_parameters_from_overrides=True)
        assert reader.get_parameter('set_initial_pose').value is (enabled == 'true')
        assert reader.get_parameter('initial_pose.x').value == 1.25
        assert reader.get_parameter('initial_pose.y').value == -1.75
        assert reader.get_parameter('initial_pose.yaw').value == .4
        # Exact resolution must retain the original AMCL particle/laser model.
        assert reader.get_parameter('max_beams').value == 60
        assert reader.get_parameter('min_particles').value == 500
        assert reader.get_parameter('max_particles').value == 2000
        assert reader.get_parameter('transform_tolerance').value == 1.
    finally:
        if reader is not None:
            reader.destroy_node()
        if ros_context.ok():
            ros_context.shutdown()
        for path in generated:
            path.unlink(missing_ok=True)
