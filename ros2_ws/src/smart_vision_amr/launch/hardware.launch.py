"""PC-side navigation. Start turtlebot3_bringup robot.launch.py on the Pi separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('nodes_file', default_value=nodes_default),
        DeclareLaunchArgument('memory_enabled', default_value='false'),
        DeclareLaunchArgument('memory_path', default_value=memory_default),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='Run Nav2 in a shared container; false uses separate processes'),
        DeclareLaunchArgument('lifecycle_bond_timeout', default_value='4.0',
                              description='Nav2 lifecycle bond wait, seconds, for separate-process bringup'),
        DeclareLaunchArgument('use_motion_mux', default_value='true',
                              description='Disable only when the web service owns the velocity mux'),
        GroupAction([
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
             parameters=[{'use_sim_time': False}], output='screen',
             condition=IfCondition(LaunchConfiguration('use_motion_mux'))),
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
