"""Robot-local estimation, independent safety watchdog, and final velocity mux.

PC navigation/web processes send stamped commands; their scheduling and camera
load cannot delay the robot's steady-clock command deadman. This launch starts
no navigation, map server, web server, camera, or Gazebo process.
"""

import os
import math
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from smart_vision_amr.wheel_joint_odometry import load_stock_wheel_geometry


RUNTIME_INSTANCE_ENV = 'SMART_VISION_REQUIRED_RUNTIME_INSTANCE'


def _required_process_exited(event, context, instance):
    """A stock OpenCR/LDS exit must stop estimation, guards and final output.

    Included bringup actions are created later, so per-Node exit hooks here
    cannot cover them. The scoped environment tag identifies every process
    owned by this runtime without affecting unrelated parent-launch processes.
    There are no intentionally short-lived subprocesses in this group.
    """
    if context.is_shutdown or (event.env or {}).get(RUNTIME_INSTANCE_ENV) != instance:
        return []
    return [Shutdown(reason=(
        f'Required robot process {event.process_name} exited unexpectedly '
        f'(code {event.returncode}); stopping robot runtime'))]


def _bool_argument(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(name+' must be true or false')
    return value == 'true'


def _runtime(context):
    mode = LaunchConfiguration('localization_mode').perform(context)
    if mode not in ('baseline', 'ekf_independent'):
        raise ValueError('localization_mode must be baseline or ekf_independent')
    use_sim_time = _bool_argument(context, 'use_sim_time')
    if use_sim_time:
        raise ValueError('The physical robot runtime requires use_sim_time:=false')
    enable_bringup = _bool_argument(context, 'enable_bringup')
    mapping_mode = _bool_argument(context, 'mapping_mode')
    if mapping_mode and mode != 'ekf_independent':
        raise ValueError('mapping_mode requires ekf_independent so sensor guard remains active')
    timing = {'enable_timing_diagnostics': _bool_argument(context, 'timing_diagnostics')}
    own = get_package_share_directory('smart_vision_amr')
    stock_params = LaunchConfiguration('stock_params').perform(context)
    usb_port = LaunchConfiguration('usb_port').perform(context)
    common = {'use_sim_time': False}
    ekf_params = os.path.join(own, 'config', 'ekf_hardware.yaml')
    health_params = os.path.join(own, 'config', 'localization_health.yaml')
    actions = []
    if enable_bringup:
        if mode == 'ekf_independent':
            bringup_path = os.path.join(own, 'launch', 'hardware_robot_ekf.launch.py')
        else:
            bringup_path = os.path.join(get_package_share_directory('turtlebot3_bringup'),
                                         'launch', 'robot.launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_path),
            launch_arguments={'stock_params': stock_params, 'tb3_param_dir': stock_params,
                              'usb_port': usb_port, 'use_sim_time': 'false'}.items()))
    if mode == 'ekf_independent':
        minimum_battery_voltage = float(
            LaunchConfiguration('minimum_battery_voltage').perform(context))
        if not math.isfinite(minimum_battery_voltage) or minimum_battery_voltage < 0.0:
            raise ValueError('minimum_battery_voltage must be finite and nonnegative')
        # Reading the installed Pi's geometry avoids silently using dimensions
        # from a PC simulation file. The parser preserves repeated /** roots.
        dimensions = load_stock_wheel_geometry(stock_params)
        for name in ('wheel_radius', 'wheel_separation'):
            explicit = LaunchConfiguration(name).perform(context).strip()
            if explicit:
                value = float(explicit)
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError(name+' must be a finite positive number')
                dimensions[name] = value
        actions.extend([
            Node(package='smart_vision_amr', executable='wheel_joint_odometry',
                 name='wheel_joint_odometry', parameters=[ekf_params, common, dimensions],
                 output='screen', on_exit=Shutdown(reason='Wheel odometry exited')),
            Node(package='smart_vision_amr', executable='imu_ekf_conditioner',
                 name='imu_ekf_conditioner', parameters=[ekf_params, common],
                 output='screen', on_exit=Shutdown(reason='IMU conditioner exited')),
            Node(package='robot_localization', executable='ekf_node', name='ekf_hardware',
                 parameters=[ekf_params, common], remappings=[('odometry/filtered', '/odom')],
                 output='screen', on_exit=Shutdown(reason='EKF output exited')),
            Node(package='smart_vision_amr', executable='ekf_sensor_guard',
                 name='ekf_sensor_guard', parameters=[ekf_params, common, timing,
                                                      {'minimum_battery_voltage': minimum_battery_voltage}],
                 output='screen', on_exit=Shutdown(reason='EKF sensor guard exited')),
        ])
    actions.extend([
        Node(package='smart_vision_amr', executable='motion_mux', name='motion_mux',
             parameters=[os.path.join(own, 'config', 'motion_mux_hardware.yaml'),
                         common, timing,
                         ({'braking_wheel_separation': dimensions['wheel_separation']}
                          if mode == 'ekf_independent' else {}),
                         {'allow_legacy_auto_input': False,
                                  'require_localization_health': True,
                                  'max_linear_velocity': 0.22,
                                  'scan_guard_robot_radius': 0.22,
                                  'scan_guard_base_frame': 'base_footprint',
                                  'startup_estop': True, 'allow_topic_reset': False,
                                  # Final physical command coupling. Straight
                                  # travel stays at 0.22 m/s; a commanded turn
                                  # retains yaw authority but reduces forward
                                  # speed before the chassis cuts a wide arc.
                                  'auto_turn_slowdown_enabled': True,
                                  'auto_turn_start_angular': 0.08,
                                  'auto_turn_full_angular': 0.35,
                                  'auto_turn_entry_linear_velocity': 0.15,
                                  'auto_turn_min_linear_velocity': 0.10,
                                  # Only the actual final publisher advances
                                  # this ramp; safety/zero commands bypass it.
                                  'gentle_start_enabled': True,
                                  'gentle_start_linear_acceleration': 0.15,
                                  'gentle_start_angular_acceleration': 0.80,
                                  'gentle_start_max_publish_gap': 0.50}],
             output='screen', on_exit=Shutdown(reason='Final velocity mux exited')),
        Node(package='smart_vision_amr', executable='localization_health',
             name='localization_health', parameters=[health_params, common,
                                                     {'mapping_mode': mapping_mode}],
             output='screen', on_exit=Shutdown(reason='Localization health monitor exited')),
    ])
    return actions


def generate_launch_description():
    stock = FindPackageShare('turtlebot3_bringup')
    instance = uuid.uuid4().hex
    return LaunchDescription([
        # Register before starting even the nested stock bringup. Clean exit
        # code zero is still unexpected for these continuous runtime nodes.
        RegisterEventHandler(OnProcessExit(
            on_exit=lambda event, context: _required_process_exited(event, context, instance))),
        DeclareLaunchArgument('localization_mode', default_value='ekf_independent'),
        DeclareLaunchArgument('enable_bringup', default_value='true'),
        DeclareLaunchArgument('mapping_mode', default_value='false',
                              description='SLAM manual mapping only: no AMCL/map agreement; autonomous motion blocked'),
        DeclareLaunchArgument('timing_diagnostics', default_value='false',
                              description='Observe runtime delays without changing safety or QoS'),
        DeclareLaunchArgument('minimum_battery_voltage', default_value='0.0',
                              description='Volts; 0 disables robot-local battery cutoff'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('wheel_radius', default_value='',
                              description='Metres; empty reads stock Pi radius, explicit value is calibrated override'),
        DeclareLaunchArgument('wheel_separation', default_value='',
                              description='Metres; empty reads stock Pi separation, explicit value is calibrated override'),
        DeclareLaunchArgument('stock_params', default_value=PathJoinSubstitution([
            stock, 'param', 'humble', 'waffle_pi.yaml'])),
        GroupAction(actions=[
            SetEnvironmentVariable(RUNTIME_INSTANCE_ENV, instance),
            OpaqueFunction(function=_runtime),
        ], scoped=True),
    ])
