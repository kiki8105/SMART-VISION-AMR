#!/usr/bin/env python3
"""Read live map/obstacles and check routes offline; never publish a drive goal."""
import json
import os
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformListener, TransformException

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.map_utils import occupancy_to_grid, world_to_cell, inflate_cells
from smart_vision_amr.web_mission_server import load_markers


def main():
    rclpy.init()
    node = rclpy.create_node('classroom_route_observer')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received = {}
    node.create_subscription(OccupancyGrid, '/map', lambda m: received.update(map=m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Int32MultiArray, '/smart_vision_amr/observed_obstacles',
        lambda m: received.update(observed=m, observed_at=time.monotonic()), 10)
    root = Path(__file__).resolve().parents[1]
    try:
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if 'map' not in received or 'observed' not in received:
                continue
            try:
                transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            except TransformException:
                continue
            if time.monotonic()-received['observed_at'] < .8:
                break
        else:
            raise RuntimeError('Fresh map, live observations, and TF are required')
        # Let recently reset localization replace the initial transient scans.
        deadline = time.monotonic()+2.2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        message = received['map']
        grid = occupancy_to_grid(message, 65, 5)
        start = world_to_cell(message.info, transform.transform.translation.x, transform.transform.translation.y)
        raw = {(int(i)%message.info.width, int(i)//message.info.width)
               for i in received['observed'].data}
        blocked = inflate_cells(raw, message.info.width, message.info.height, 5)
        markers = load_markers(root/'ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers.yaml')
        result = []
        for marker in markers:
            goal = world_to_cell(message.info, marker['x'], marker['y'])
            route = AStarPlanner().plan(grid, start, goal, blocked)
            result.append({'id': marker['id'], 'name': marker['name'], 'goal': goal,
                           'static_goal_blocked': grid.is_static_obstacle(goal),
                           'raw_observed_goal': goal in raw, 'observed_goal_blocked': goal in blocked,
                           'path_cells': len(route), 'reachable': bool(route)})
        report = {'domain': int(os.environ.get('ROS_DOMAIN_ID', '0')), 'start': start,
                  'raw_obstacle_count': len(raw), 'routes': result, 'drive_goal_sent': False}
        path = root/'docs/classroom_routes_live_20261027.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if all(item['reachable'] for item in result) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
