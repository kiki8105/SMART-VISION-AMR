"""PC-side navigation. Start turtlebot3_bringup robot.launch.py on the Pi separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    own = FindPackageShare('smart_vision_amr')
    nav2 = FindPackageShare('nav2_bringup')
    tb3 = FindPackageShare('turtlebot3_navigation2')
    map_default = PathJoinSubstitution([own, 'maps', 'classroom_map.yaml'])
    params_default = PathJoinSubstitution([tb3, 'param', 'humble', 'waffle_pi.yaml'])
    nodes = PathJoinSubstitution([own, 'config', 'nodes.yaml'])
    rviz_config = PathJoinSubstitution([own, 'rviz', 'minimal.rviz'])
    memory_default = '~/.ros/smart_vision_amr/classroom_memory.json'
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('memory_enabled', default_value='false'),
        DeclareLaunchArgument('memory_path', default_value=memory_default),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav2, 'launch', 'bringup_launch.py'])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false', 'autostart': 'true',
                'use_localization': 'true',
            }.items(),
        ),
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
