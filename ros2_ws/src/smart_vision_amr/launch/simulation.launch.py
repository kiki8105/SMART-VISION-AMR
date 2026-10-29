"""Gazebo Waffle Pi: fresh SLAM mapping or AMCL + the team's A* navigation."""

from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from smart_vision_amr.gazebo_model_variant import make_independent_ekf_model, set_camera_update_rate


LOCALIZATION_MODES = ('baseline', 'ekf_overlay', 'ekf_independent')


def _validate_localization_mode(context):
    mode = LaunchConfiguration('localization_mode').perform(context)
    if mode not in LOCALIZATION_MODES:
        raise ValueError(f'localization_mode must be one of {LOCALIZATION_MODES}; got {mode!r}')
    return []


def _spawn_robot(context, source_model):
    mode = LaunchConfiguration('localization_mode').perform(context)
    actions = []
    temporary_dir = Path(tempfile.mkdtemp(prefix='smart_vision_amr_model_'))
    model = temporary_dir / 'model.sdf'
    try:
        rate = float(LaunchConfiguration('camera_update_rate', default='8.0').perform(context))
        set_camera_update_rate(source_model, model, rate)
        if mode == 'ekf_independent':
            # Only this mode moves Gazebo odom away from the EKF's output.
            make_independent_ekf_model(model, model)
    except Exception:
        shutil.rmtree(temporary_dir, ignore_errors=True)
        raise

    def cleanup(_context):
        shutil.rmtree(temporary_dir, ignore_errors=True)
        return []

    actions.append(RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    actions.append(Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'waffle_pi', '-file', str(model),
                   '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                   '-Y', LaunchConfiguration('yaw'), '-z', '0.01']))
    return actions


def generate_launch_description():
    own = Path(get_package_share_directory('smart_vision_amr'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    robot_description = (tb3 / 'urdf/turtlebot3_waffle_pi.urdf').read_text()
    config = LaunchConfiguration('nav_params_file')
    navigation = UnlessCondition(LaunchConfiguration('slam'))
    sim = {'use_sim_time': True}
    nodes = LaunchConfiguration('nodes_params_file')
    floating = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(own / 'worlds/simulation_room.world')),
        DeclareLaunchArgument('map', default_value=str(own / 'maps/simulation_room.yaml')),
        DeclareLaunchArgument('nav_params_file', default_value=str(own / 'config/simulation.yaml')),
        DeclareLaunchArgument('nodes_params_file', default_value=str(own / 'config/simulation_nodes.yaml')),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Build a fresh map using manual driving; disables navigation'),
        DeclareLaunchArgument('slam_params_file', default_value=str(own / 'config/slam_simulation.yaml')),
        DeclareLaunchArgument('gazebo_params_file', default_value=str(own / 'config/gazebo_clock.yaml')),
        DeclareLaunchArgument('x_pose', default_value='-3.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_x', default_value=LaunchConfiguration('x_pose'),
                              description='AMCL start x in the saved map frame; use the captured SLAM pose'),
        DeclareLaunchArgument('initial_y', default_value=LaunchConfiguration('y_pose'),
                              description='AMCL start y in the saved map frame; use the captured SLAM pose'),
        DeclareLaunchArgument('initial_yaw', default_value=LaunchConfiguration('yaw'),
                              description='AMCL start yaw in the saved map frame'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera_update_rate', default_value='8.0',
                              description='Simulated camera FPS; lowers preview/transport load without changing geometry'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_motion_mux', default_value='true',
                              description='Disable only when the web service owns the velocity mux'),
        DeclareLaunchArgument('record_incidents', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='baseline',
                              description='baseline, ekf_overlay (diagnostic), or ekf_independent'),
        DeclareLaunchArgument('memory_enabled', default_value='false'),
        DeclareLaunchArgument('memory_path', default_value='~/.ros/smart_vision_amr/simulation_memory.json'),
        OpaqueFunction(function=_validate_localization_mode),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', str(tb3 / 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'params_file': LaunchConfiguration('gazebo_params_file')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': robot_description}], output='screen'),
        OpaqueFunction(function=_spawn_robot,
                       args=[tb3 / 'models/turtlebot3_waffle_pi/model.sdf']),
        Node(package='robot_localization', executable='ekf_node', name='ekf_overlay',
             parameters=[str(own / 'config/ekf_overlay.yaml'), sim], output='screen',
             condition=LaunchConfigurationEquals('localization_mode', 'ekf_overlay')),
        Node(package='smart_vision_amr', executable='wheel_joint_odometry',
             name='wheel_joint_odometry', parameters=[sim], output='screen',
             condition=LaunchConfigurationEquals('localization_mode', 'ekf_independent')),
        Node(package='robot_localization', executable='ekf_node', name='ekf_independent',
             parameters=[str(own / 'config/ekf_independent.yaml'), sim], output='screen',
             remappings=[('odometry/filtered', '/odom')],
             condition=LaunchConfigurationEquals('localization_mode', 'ekf_independent')),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[LaunchConfiguration('slam_params_file'), sim], output='screen',
             condition=IfCondition(LaunchConfiguration('slam'))),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[config, sim, {'yaml_filename': LaunchConfiguration('map')}], output='screen',
             condition=navigation),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[config, sim, {'set_initial_pose': True,
                                      'initial_pose.x': floating('initial_x'),
                                      'initial_pose.y': floating('initial_y'),
                                      'initial_pose.z': 0.0,
                                      'initial_pose.yaw': floating('initial_yaw')}],
             condition=navigation),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[config, sim, {'speed_limit_topic': '/speed_limit'}], output='screen', condition=navigation,
             remappings=[('cmd_vel', '/smart_vision_amr/navigation_cmd_vel')]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[config, sim], output='screen', condition=navigation,
             remappings=[('cmd_vel', '/smart_vision_amr/navigation_cmd_vel')]),
        Node(package='smart_vision_amr', executable='motion_mux', name='motion_mux',
             parameters=[sim, {'allow_legacy_auto_input': False}], output='screen',
             condition=IfCondition(LaunchConfiguration('use_motion_mux'))),
        Node(package='smart_vision_amr', executable='command_stamper', name='command_stamper',
             parameters=[sim], output='screen', condition=navigation),
        Node(package='smart_vision_amr', executable='localization_health', name='localization_health',
             parameters=[str(own/'config/localization_health.yaml'), sim], output='screen', condition=navigation),
        Node(package='smart_vision_amr', executable='flight_recorder', name='flight_recorder',
             parameters=[str(own/'config/flight_recorder.yaml'), sim], output='screen',
             condition=IfCondition(LaunchConfiguration('record_incidents'))),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_simulation', output='screen',
             parameters=[sim, {'autostart': True,
                               'node_names': ['map_server', 'amcl', 'controller_server',
                                              'behavior_server']}],
             condition=navigation),
        Node(package='smart_vision_amr', executable='astar_planner', name='astar_planner',
             parameters=[nodes, sim], output='screen', condition=navigation),
        Node(package='smart_vision_amr', executable='path_follower', name='path_follower',
             parameters=[nodes, sim], output='screen', condition=navigation),
        Node(package='smart_vision_amr', executable='change_memory', name='change_memory',
             parameters=[nodes, sim, {
                 'memory_enabled': ParameterValue(LaunchConfiguration('memory_enabled'), value_type=bool),
                 'memory_path': LaunchConfiguration('memory_path')}], output='screen',
             condition=navigation),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(own / 'rviz/simulation.rviz')], parameters=[sim],
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
    ])
