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


def assess_publishers(counts, estimation_location='pc', localization_mode='ekf_independent'):
    """Return concrete reasons the PC must not become the /odom owner yet."""
    required = list(REQUIRED_TOPICS)
    forbidden = FORBIDDEN_TOPICS
    if estimation_location == 'robot':
        required = ['/joint_states', '/imu', '/scan', '/odom',
                    '/smart_vision_amr/motion_status', '/smart_vision_amr/localization_health']
        if localization_mode == 'ekf_independent':
            required += ['/wheel/odom_raw', '/wheel/odom', '/imu/ekf']
        forbidden = ()
    reasons = [f'{topic}: expected exactly one robot publisher, '
               f'found {counts.get(topic, 0)}' for topic in REQUIRED_TOPICS
               if topic in required and counts.get(topic, 0) != 1]
    reasons += [f'{topic}: expected exactly one robot publisher, found {counts.get(topic, 0)}'
                for topic in required if topic not in REQUIRED_TOPICS and counts.get(topic, 0) != 1]
    reasons += [f'{topic}: {counts[topic]} existing publisher(s)' for topic in forbidden
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


def assess_runtime_owners(owners, localization_mode='ekf_independent'):
    """Counts alone cannot distinguish an old stock odom from the selected EKF."""
    expected = {'/cmd_vel': 'motion_mux',
                '/smart_vision_amr/motion_status': 'motion_mux',
                '/smart_vision_amr/localization_health': 'localization_health'}
    if localization_mode == 'ekf_independent':
        expected.update({'/odom': 'ekf_hardware', '/wheel/odom': 'wheel_joint_odometry',
                         '/imu/ekf': 'imu_ekf_conditioner'})
    return [f'{topic}: expected sole {name} publisher, found {owners.get(topic, [])}'
            for topic, name in expected.items() if owners.get(topic, []) != [name]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=8.0,
                        help='Seconds to wait for the robot DDS endpoints')
    parser.add_argument('--estimation-location', choices=('pc', 'robot'), default='pc')
    parser.add_argument('--localization-mode', choices=('baseline', 'ekf_independent'), default='ekf_independent')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('hardware_ekf_preflight')
    try:
        stamps = {}

        def receive(topic, message):
            stamps[topic] = (message.header.stamp.sec * 1_000_000_000 +
                             message.header.stamp.nanosec)

        topics = [('/joint_states', JointState), ('/imu', Imu), ('/scan', LaserScan)]
        if args.estimation_location == 'robot':
            topics.append(('/odom', Odometry))
        subscriptions = [node.create_subscription(
            message_type, topic, lambda message, topic=topic: receive(topic, message),
            qos_profile_sensor_data)
            for topic, message_type in topics]
        deadline = time.monotonic() + args.timeout
        stable = 0
        reasons = []
        while time.monotonic() < deadline:
            # Allow discovery to settle, then inspect endpoints from this
            # participant's graph. Do not use ros2 daemon results.
            rclpy.spin_once(node, timeout_sec=0.2)
            counts = {topic: len(node.get_publishers_info_by_topic(topic))
                      for topic in (*REQUIRED_TOPICS, *FORBIDDEN_TOPICS, '/wheel/odom', '/imu/ekf',
                                    '/smart_vision_amr/motion_status', '/smart_vision_amr/localization_health')}
            reasons = assess_publishers(counts, args.estimation_location, args.localization_mode) + assess_sensor_stamps(
                stamps, node.get_clock().now().nanoseconds)
            if args.estimation_location == 'robot':
                now_ns = node.get_clock().now().nanoseconds
                if '/odom' not in stamps or not -.25 <= (now_ns-stamps['/odom'])/1e9 <= 1.:
                    reasons.append('/odom: no fresh robot-local estimate received')
                owner_topics = ('/cmd_vel', '/odom', '/wheel/odom', '/imu/ekf',
                                '/smart_vision_amr/motion_status',
                                '/smart_vision_amr/localization_health')
                owners = {topic: [endpoint.node_name for endpoint in
                                  node.get_publishers_info_by_topic(topic)]
                          for topic in owner_topics}
                reasons.extend(assess_runtime_owners(owners, args.localization_mode))
            if not reasons:
                stable += 1
                if stable >= 2:
                    print('Robot preflight passed: '+('robot-local odom/mux/health present; fresh sensors.'
                        if args.estimation_location == 'robot' else 'fresh raw wheels/IMU/scan; /odom unowned.'))
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
