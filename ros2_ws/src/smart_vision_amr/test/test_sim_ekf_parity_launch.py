"""Inspect the guarded simulator launch without starting ROS or Gazebo."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE = Path(__file__).resolve().parents[1]


def simulation(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'sim_ekf_parity_launch', PACKAGE/'launch/simulation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stock = tmp_path/'turtlebot3_gazebo'
    (stock/'urdf').mkdir(parents=True)
    (stock/'urdf/turtlebot3_waffle_pi.urdf').write_text('<robot name="test"/>')
    shares = {'smart_vision_amr': PACKAGE, 'turtlebot3_gazebo': stock,
              'gazebo_ros': tmp_path/'gazebo_ros'}
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(shares[package]))
    return module


def context_for(description, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.visit(context)
    return context


def parameters(context, node):
    return evaluate_parameters(context, node._Node__parameters)


def test_guarded_independent_sim_uses_physical_wheel_limits_and_latched_mux(tmp_path, monkeypatch):
    module = simulation(tmp_path, monkeypatch)
    description = module.generate_launch_description()
    context = context_for(description, localization_mode='ekf_independent',
                          use_ekf_sensor_guard='true')
    module._validate_localization_mode(context)
    nodes = {node.node_executable: node for node in description.entities
             if isinstance(node, Node)}
    wheel = nodes['wheel_joint_odometry']
    assert wheel.condition.evaluate(context)
    assert any(value == PACKAGE/'config/ekf_hardware.yaml'
               for value in parameters(context, wheel))
    guard = nodes['ekf_sensor_guard']
    assert guard.condition.evaluate(context)
    context.launch_configurations['minimum_battery_voltage'] = '10.8'
    physical_overrides = next(value for value in parameters(context, guard)
                              if isinstance(value, dict)
                              and 'expected_ekf_odom_publisher' in value)
    assert physical_overrides == {'expected_ekf_odom_publisher': 'ekf_independent',
                                  'expected_raw_wheel_odom_topic': '/gazebo/odom_raw',
                                  'minimum_battery_voltage': 10.8}
    mux = next(node for node in description.entities
               if isinstance(node, Node) and node.node_executable == 'motion_mux')
    guarded_mux = next(value for value in parameters(context, mux)
                       if isinstance(value, dict) and 'startup_estop' in value)
    assert guarded_mux == {'allow_legacy_auto_input': False,
                           'max_linear_velocity': .22,
                           'scan_guard_robot_radius': .24,
                           'scan_guard_base_frame': 'base_footprint',
                           'auto_turn_slowdown_enabled': True,
                           'auto_turn_start_angular': .08,
                           'auto_turn_full_angular': .35,
                           'auto_turn_entry_linear_velocity': .15,
                           'auto_turn_min_linear_velocity': .10,
                           'gentle_start_enabled': True,
                           'gentle_start_linear_acceleration': .20,
                           'gentle_start_angular_acceleration': .80,
                           'gentle_start_max_publish_gap': .50,
                           'startup_estop': True,
                           'require_localization_health': True,
                           'allow_topic_reset': False}
    physical_description = module.generate_launch_description()
    physical_context = context_for(physical_description, localization_mode='ekf_independent',
                                   use_ekf_sensor_guard='true', scan_guard_robot_radius='0.22')
    physical_mux_node = next(node for node in physical_description.entities
                             if isinstance(node, Node) and node.node_executable == 'motion_mux')
    physical_mux = next(value for value in parameters(physical_context, physical_mux_node)
                        if isinstance(value, dict) and 'startup_estop' in value)
    assert physical_mux['scan_guard_robot_radius'] == .22


def test_guard_is_opt_in_and_rejects_incompatible_sim_modes(tmp_path, monkeypatch):
    module = simulation(tmp_path, monkeypatch)
    description = module.generate_launch_description()
    guard = next(node for node in description.entities
                 if isinstance(node, Node) and node.node_executable == 'ekf_sensor_guard')
    default = context_for(description)
    assert not guard.condition.evaluate(default)
    baseline = context_for(description, localization_mode='baseline',
                           use_ekf_sensor_guard='true')
    with pytest.raises(ValueError, match='requires localization_mode'):
        module._validate_localization_mode(baseline)
    no_mux = context_for(description, localization_mode='ekf_independent',
                         use_ekf_sensor_guard='true', use_motion_mux='false')
    with pytest.raises(ValueError, match='requires use_motion_mux'):
        module._validate_localization_mode(no_mux)
