"""Portable maze lab: choose world, fresh SLAM, or saved-map Team2 navigation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_lab(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    mode = get('mode')
    if mode not in ('world', 'slam', 'navigation'):
        raise RuntimeError('mode must be world, slam, or navigation')
    own = Path(get_package_share_directory('smart_vision_amr'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    sim = {'use_sim_time': True}
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gzserver.launch.py')),
            launch_arguments={'world': get('world')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (tb3 / 'urdf/turtlebot3_waffle_pi.urdf').read_text()}],
             output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'waffle_pi', '-file',
                        str(tb3 / 'models/turtlebot3_waffle_pi/model.sdf'),
                        '-x', get('x_pose'), '-y', get('y_pose'), '-Y', get('yaw'), '-z', '0.01']),
    ]
    if get('gui').lower() in ('true', '1'):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gzclient.launch.py'))))
    if mode == 'slam':
        actions.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                            name='slam_toolbox', parameters=[get('slam_params_file'), sim], output='screen'))
    if mode == 'navigation':
        config = get('nav_params_file')
        nodes = get('nodes_params_file')
        actions.extend([
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[config, sim, {'yaml_filename': get('map')}], output='screen'),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[config, sim, {'set_initial_pose': True,
                    'initial_pose.x': float(get('initial_x')), 'initial_pose.y': float(get('initial_y')),
                    'initial_pose.z': 0.0, 'initial_pose.yaw': float(get('initial_yaw'))}]),
            Node(package='nav2_controller', executable='controller_server', name='controller_server',
                 parameters=[config, sim], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_simulation', output='screen',
                 parameters=[sim, {'autostart': True,
                                   'node_names': ['map_server', 'amcl', 'controller_server']}]),
            Node(package='smart_vision_amr', executable='astar_planner', name='astar_planner',
                 parameters=[nodes, sim], output='screen'),
            Node(package='smart_vision_amr', executable='path_follower', name='path_follower',
                 parameters=[nodes, sim], output='screen'),
            Node(package='smart_vision_amr', executable='change_memory', name='change_memory',
                 parameters=[nodes, sim, {'memory_enabled': get('memory_enabled').lower() in ('true', '1'),
                                         'memory_path': get('memory_path')}], output='screen'),
        ])
    if get('use_rviz').lower() in ('true', '1'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', str(own / 'rviz/simulation.rviz')], parameters=[sim], output='screen'))
    return actions


def generate_launch_description():
    own = Path(get_package_share_directory('smart_vision_amr'))
    defaults = {
        'mode': 'world', 'world': str(own / 'worlds/simulation_maze_aruco.world'),
        'map': str(own / 'maps/maze_aruco_20261026.yaml'),
        'nav_params_file': str(own / 'config/curve_tuning_simulation.yaml'),
        'nodes_params_file': str(own / 'config/curve_tuning_simulation_nodes.yaml'),
        'slam_params_file': str(own / 'config/slam_simulation.yaml'),
        'x_pose': '-3.0', 'y_pose': '-2.0', 'yaw': '0.0',
        'initial_x': '-3.000799585626671', 'initial_y': '-2.0010944835652884',
        'initial_yaw': '0.0008724678696151052',
        'gui': 'true', 'use_rviz': 'false', 'memory_enabled': 'false',
        'memory_path': str(Path.home() / '.ros/turtlebot3_aruco_lab/memory.json'),
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=start_lab)])
