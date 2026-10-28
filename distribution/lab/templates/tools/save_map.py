#!/usr/bin/env python3
"""Save a fresh SLAM /map without stopping its clock or overwriting an existing map."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


def read_pose():
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = Node('lab_map_pose_capture', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    end = time.monotonic() + 5.0
    result = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform('map', 'base_footprint', Time(), timeout=Duration(seconds=0.0))
            p, q = transform.transform.translation, transform.transform.rotation
            result = [p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]
            break
        except Exception:
            pass
    del listener
    node.destroy_node()
    rclpy.shutdown()
    return result


def main():
    prefix = Path(sys.argv[1])
    if not prefix.is_absolute() or any(Path(str(prefix) + ext).exists() for ext in ('.yaml', '.pgm', '.png')):
        raise SystemExit('Choose an absolute new map prefix without an existing map.')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=15.0',
        '-p', 'map_subscribe_transient_local:=true', '-p', 'free_thresh_default:=0.196',
        '-p', 'occupied_thresh_default:=0.65'])
    if result.returncode:
        return result.returncode
    if not Path(str(prefix) + '.yaml').is_file() or not Path(str(prefix) + '.pgm').is_file():
        raise SystemExit('map_saver did not produce both YAML and PGM.')
    lab = Path(os.environ['TURTLEBOT3_LAB_ROOT'])
    try:
        final_pose = read_pose()
    except Exception as exc:
        print(f'Map saved; optional final pose capture failed: {exc}', file=sys.stderr)
        final_pose = None
    info = {
        'map_source': 'Live Gazebo LaserScan + slam_toolbox; saved before SLAM shutdown',
        'world': 'worlds/maze.world',
        'world_sha256': hashlib.sha256((lab / 'worlds/maze.world').read_bytes()).hexdigest(),
        'saved_map_yaml': prefix.name + '.yaml',
        'robot_map_pose_at_save_x_y_yaw': final_pose,
        'note': 'For AMCL startup use the robot pose at the initial spawn, not its final mapping pose.',
    }
    prefix.with_name(prefix.name + '_mapping_info.json').write_text(json.dumps(info, indent=2) + '\n')
    print(f'Saved {prefix}.yaml and {prefix}.pgm')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
