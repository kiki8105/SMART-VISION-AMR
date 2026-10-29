"""PC-side navigation. Start turtlebot3_bringup robot.launch.py on the Pi separately."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, Shutdown,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_localization_mode(context):
    mode = LaunchConfiguration('localization_mode').perform(context)
    if mode not in ('baseline', 'ekf_independent'):
        raise ValueError('localization_mode must be baseline or ekf_independent')
    location = LaunchConfiguration('estimation_location').perform(context)
    if location not in ('pc', 'robot'):
        raise ValueError('estimation_location must be pc or robot')
    if location == 'robot' and LaunchConfiguration('use_motion_mux').perform(context).lower() in ('true', '1'):
        raise ValueError('Robot runtime owns motion_mux; use_motion_mux:=false is required')
    return []


def generate_launch_description():
    own = FindPackageShare('smart_vision_amr')
    nav2 = FindPackageShare('nav2_bringup')
    tb3 = FindPackageShare('turtlebot3_navigation2')
    map_default = PathJoinSubstitution([own, 'maps', 'classroom_map.yaml'])
    params_default = PathJoinSubstitution([tb3, 'param', 'humble', 'waffle_pi.yaml'])
    nodes_default = PathJoinSubstitution([own, 'config', 'nodes.yaml'])
    nodes = LaunchConfiguration('nodes_file')
    rviz_config = PathJoinSubstitution([own, 'rviz', 'minimal.rviz'])
    memory_default = '~/.ros/smart_vision_amr/classroom_memory.json'
    ekf = PathJoinSubstitution([own, 'config', 'ekf_hardware.yaml'])
    pc_ekf = IfCondition(PythonExpression(["'", LaunchConfiguration('localization_mode'),
        "' == 'ekf_independent' and '", LaunchConfiguration('estimation_location'), "' == 'pc'"]))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('nodes_file', default_value=nodes_default),
        DeclareLaunchArgument('memory_enabled', default_value='false'),
        DeclareLaunchArgument('memory_path', default_value=memory_default),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='baseline',
                              description='baseline (stock TurtleBot3 odom) or ekf_independent'),
        DeclareLaunchArgument('estimation_location', default_value='pc',
                              description='robot: robot-local EKF/guard/mux; pc: previous deployment'),
        DeclareLaunchArgument('record_incidents', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='Run Nav2 in a shared container; false uses separate processes'),
        DeclareLaunchArgument('lifecycle_bond_timeout', default_value='4.0',
                              description='Nav2 lifecycle bond wait, seconds, for separate-process bringup'),
        DeclareLaunchArgument('use_motion_mux', default_value='true',
                              description='Disable only when the web service owns the velocity mux'),
        OpaqueFunction(function=_validate_localization_mode),
        # In EKF mode the Pi's separate hardware_robot_ekf.launch.py must
        # suppress stock odom TF and remap its /odom to /wheel/odom_raw.
        # This joint integrator and IMU conditioner use independent raw
        # measurements; the EKF is the only /odom and odom->base publisher.
        Node(package='smart_vision_amr', executable='wheel_joint_odometry',
             name='wheel_joint_odometry',
             parameters=[ekf, {'use_sim_time': False, 'max_wheel_speed': 12.0,
                          'reject_stale_stamps': True}],
             output='screen',
             on_exit=Shutdown(reason='EKF wheel odometry exited'),
             condition=pc_ekf),
        Node(package='smart_vision_amr', executable='imu_ekf_conditioner',
             name='imu_ekf_conditioner', parameters=[ekf, {'use_sim_time': False}],
             output='screen',
             on_exit=Shutdown(reason='EKF IMU conditioner exited'),
             condition=pc_ekf),
        Node(package='robot_localization', executable='ekf_node', name='ekf_hardware',
             parameters=[PathJoinSubstitution([own, 'config', 'ekf_hardware.yaml']),
                         {'use_sim_time': False}],
             remappings=[('odometry/filtered', '/odom')], output='screen',
             on_exit=Shutdown(reason='EKF localization exited'),
             condition=pc_ekf),
        Node(package='smart_vision_amr', executable='ekf_sensor_guard',
             name='ekf_sensor_guard', parameters=[ekf, {'use_sim_time': False}],
             output='screen', on_exit=Shutdown(reason='EKF safety guard exited'),
             condition=pc_ekf),
        Node(package='smart_vision_amr', executable='localization_health', name='localization_health',
             parameters=[PathJoinSubstitution([own, 'config', 'localization_health.yaml']), {'use_sim_time': False}],
             output='screen', condition=LaunchConfigurationEquals('estimation_location', 'pc')),
        GroupAction([
            SetParameter(name='speed_limit_topic', value='/speed_limit'),
            # The classroom PC can be slow to schedule map_server during
            # concurrent Nav2 startup. Keep bond failure detection enabled,
            # but allow a longer connection window for separate processes.
            SetParameter(name='bond_timeout',
                         value=ParameterValue(LaunchConfiguration('lifecycle_bond_timeout'),
                                              value_type=float),
                         condition=UnlessCondition(LaunchConfiguration('use_composition'))),
            # Preserve controller -> cmd_vel_nav -> smoother wiring. Remap
            # only the final outputs, including recovery behavior commands.
            SetRemap(src='velocity_smoother:cmd_vel_smoothed',
                     dst='/smart_vision_amr/navigation_cmd_vel'),
            SetRemap(src='behavior_server:cmd_vel',
                     dst='/smart_vision_amr/navigation_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav2, 'launch', 'bringup_launch.py'])),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': 'false', 'autostart': 'true',
                    'use_localization': 'true',
                    # Humble's child launch evaluates ``not <value>`` as a
                    # Python expression, so pass True/False even for lower-case CLI values.
                    'use_composition': PythonExpression([
                        "'", LaunchConfiguration('use_composition'), "'.lower() in ('true', '1')"]),
                }.items(),
            ),
        ]),
        Node(package='smart_vision_amr', executable='motion_mux', name='motion_mux',
             parameters=[{'use_sim_time': False, 'allow_legacy_auto_input': False,
                          'require_localization_health': True, 'startup_estop': True,
                          'allow_topic_reset': False}], output='screen',
             condition=IfCondition(LaunchConfiguration('use_motion_mux'))),
        Node(package='smart_vision_amr', executable='command_stamper', name='command_stamper',
             parameters=[{'use_sim_time': False}], output='screen'),
        Node(package='smart_vision_amr', executable='flight_recorder', name='flight_recorder',
             parameters=[PathJoinSubstitution([own, 'config', 'flight_recorder.yaml']), {'use_sim_time': False}],
             output='screen', condition=IfCondition(LaunchConfiguration('record_incidents'))),
        Node(package='smart_vision_amr', executable='astar_planner', name='astar_planner',
             parameters=[nodes, {'use_sim_time': False}], output='screen'),
        Node(package='smart_vision_amr', executable='path_follower', name='path_follower',
             parameters=[nodes, {'use_sim_time': False}], output='screen'),
        Node(package='smart_vision_amr', executable='change_memory', name='change_memory',
             parameters=[nodes, {'use_sim_time': False,
                                 'memory_enabled': ParameterValue(
                                     LaunchConfiguration('memory_enabled'), value_type=bool),
                                 'memory_path': LaunchConfiguration('memory_path')}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], parameters=[{'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
    ])
