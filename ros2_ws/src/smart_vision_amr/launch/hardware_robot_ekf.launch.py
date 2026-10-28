"""Pi-side Waffle Pi bringup for independent wheel/IMU EKF on the PC.

This file can be copied to the Pi and launched by absolute path. It needs
only the stock turtlebot3_bringup package installed on the Pi. The stock
parameter file is copied and modified in a temporary file at launch time;
the installed TurtleBot3 files are never edited.
"""

import os
from pathlib import Path
import re
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def _write_wheel_only_params(stock_file, target_file):
    """Keep Pi-specific stock settings, changing exactly the two odom flags."""
    source = Path(stock_file).read_text(encoding='utf-8')
    controller = re.search(r'(?m)^[ \t]*diff_drive_controller[ \t]*:[ \t]*$', source)
    if controller is None:
        raise ValueError(f'{stock_file}: diff_drive_controller section is missing')
    before, section = source[:controller.start()], source[controller.start():]
    if re.search(r'(?m)^[ \t]*odometry[ \t]*:[ \t]*$', section) is None:
        raise ValueError(f'{stock_file}: odometry section is missing')
    for key in ('publish_tf', 'use_imu'):
        pattern = rf'(?m)^([ \t]*{key}[ \t]*:[ \t]*)true([ \t]*(?:\#.*)?)$'
        section, count = re.subn(pattern, r'\g<1>false\2', section)
        if count != 1:
            raise ValueError(f'{stock_file}: expected exactly one {key}: true')
    Path(target_file).write_text(before + section, encoding='utf-8')


def _launch_bringup(context):
    if os.environ.get('TURTLEBOT3_MODEL') != 'waffle_pi':
        raise ValueError('TURTLEBOT3_MODEL=waffle_pi is required')
    if not os.environ.get('LDS_MODEL'):
        raise ValueError('LDS_MODEL must be set for TurtleBot3 bringup')

    stock_file = LaunchConfiguration('stock_params').perform(context)
    handle, temporary_path = tempfile.mkstemp(prefix='tb3_wheel_only_', suffix='.yaml')
    os.close(handle)
    try:
        _write_wheel_only_params(stock_file, temporary_path)
    except Exception:
        Path(temporary_path).unlink(missing_ok=True)
        raise

    def cleanup(_context):
        Path(temporary_path).unlink(missing_ok=True)
        return []

    bringup = FindPackageShare('turtlebot3_bringup')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        GroupAction([
            # The stock node still computes wheel-only odom for diagnostics,
            # but must not own the Nav2 /odom topic or odom->base_footprint TF.
            SetRemap(src='odom', dst='/wheel/odom_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    bringup, 'launch', 'robot.launch.py'])),
                launch_arguments={
                    'tb3_param_dir': temporary_path,
                    'use_sim_time': 'false',
                    'usb_port': LaunchConfiguration('usb_port'),
                }.items()),
        ]),
    ]


def generate_launch_description():
    bringup = FindPackageShare('turtlebot3_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('stock_params', default_value=PathJoinSubstitution([
            bringup, 'param', 'humble', 'waffle_pi.yaml'])),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        OpaqueFunction(function=_launch_bringup),
    ])
