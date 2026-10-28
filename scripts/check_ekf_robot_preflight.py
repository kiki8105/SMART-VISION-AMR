#!/usr/bin/env python3
"""Fail closed when the physical robot is not in wheel-only EKF mode.

Query the live DDS graph directly instead of the ros2 daemon cache. This
script never publishes a command or changes robot state.
"""

import argparse
import sys
import time


REQUIRED_TOPICS = ('/wheel/odom_raw', '/joint_states', '/imu', '/scan')
FORBIDDEN_TOPICS = ('/odom',)
STAMPED_TOPICS = ('/joint_states', '/imu', '/scan')


def assess_publishers(counts):
    """Return concrete reasons the PC must not become the /odom owner yet."""
    reasons = [f'{topic}: expected exactly one robot publisher, '
               f'found {counts.get(topic, 0)}' for topic in REQUIRED_TOPICS
               if counts.get(topic, 0) != 1]
    reasons += [f'{topic}: {counts[topic]} existing publisher(s)' for topic in FORBIDDEN_TOPICS
                if counts.get(topic, 0) > 0]
    return reasons


def assess_sensor_stamps(stamps, now_ns, *, max_age_sec=1.0,
                         future_tolerance_sec=0.25):
    """Check that real sensor messages have recently arrived in PC time."""
    reasons = []
    for topic in STAMPED_TOPICS:
        stamp_ns = stamps.get(topic)
        if stamp_ns is None:
            reasons.append(f'{topic}: no sample received')
            continue
        if stamp_ns <= 0:
            reasons.append(f'{topic}: zero/invalid header timestamp')
            continue
        offset_sec = (now_ns - stamp_ns) / 1e9
        if offset_sec > max_age_sec or offset_sec < -future_tolerance_sec:
            reasons.append(
                f'{topic}: PC clock minus sensor stamp = {offset_sec:+.3f}s '
                '(check Pi/PC clock sync, transport delay, and sensor freshness)')
    return reasons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=8.0,
                        help='Seconds to wait for the robot DDS endpoints')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, LaserScan
    rclpy.init()
    node = rclpy.create_node('hardware_ekf_preflight')
    try:
        stamps = {}

        def receive(topic, message):
            stamps[topic] = (message.header.stamp.sec * 1_000_000_000 +
                             message.header.stamp.nanosec)

        subscriptions = [node.create_subscription(
            message_type, topic, lambda message, topic=topic: receive(topic, message),
            qos_profile_sensor_data)
            for topic, message_type in (('/joint_states', JointState),
                                        ('/imu', Imu), ('/scan', LaserScan))]
        deadline = time.monotonic() + args.timeout
        stable = 0
        reasons = []
        while time.monotonic() < deadline:
            # Allow discovery to settle, then inspect endpoints from this
            # participant's graph. Do not use ros2 daemon results.
            rclpy.spin_once(node, timeout_sec=0.2)
            counts = {topic: len(node.get_publishers_info_by_topic(topic))
                      for topic in (*REQUIRED_TOPICS, *FORBIDDEN_TOPICS)}
            reasons = assess_publishers(counts) + assess_sensor_stamps(
                stamps, node.get_clock().now().nanoseconds)
            if not reasons:
                stable += 1
                if stable >= 2:
                    print('Independent EKF robot preflight passed: '
                          'raw wheels, fresh joint states/IMU/scan present; '
                          '/odom unowned.')
                    return 0
            else:
                stable = 0
        print('Independent EKF robot preflight failed: ' + '; '.join(reasons),
              file=sys.stderr, flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
