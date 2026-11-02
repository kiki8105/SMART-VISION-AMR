#!/usr/bin/env python3
"""Ask the isolated Gazebo relay to inject one mid-mission timing profile."""

import argparse
import json
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('transient',), required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '176' or os.environ.get(
            'ROS_LOCALHOST_ONLY') != '1':
        parser.error('Only isolated ROS domain 176, localhost DDS may inject faults')

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node('classroom_web_transient_request')
    publisher = node.create_publisher(String, '/classroom_fault/control', 10)
    try:
        deadline = time.monotonic()+3.
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if publisher.get_subscription_count() < 1:
            raise RuntimeError('Isolated fault relay is not subscribed')
        request_at = time.monotonic()
        publisher.publish(String(data=json.dumps({'profile': args.profile})))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.05)
        print(json.dumps({'profile_requested': args.profile,
                          'requested_monotonic': request_at,
                          'requested_wall_time': time.time()}))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
