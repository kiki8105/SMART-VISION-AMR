#!/usr/bin/env python3
"""Fail closed when the physical robot is not in wheel-only EKF mode.

Query the live DDS graph directly instead of the ros2 daemon cache. This
script never publishes a command or changes robot state.
"""

import argparse
import math
import os
import sys
import time


REQUIRED_TOPICS = ('/wheel/odom_raw', '/joint_states', '/imu', '/scan')
FORBIDDEN_TOPICS = ('/odom',)
STAMPED_TOPICS = ('/joint_states', '/imu', '/scan')
ROBOT_STAMPED_TOPICS = STAMPED_TOPICS + ('/odom', '/wheel/odom_raw', '/battery_state')
BATTERY_MAX_AGE_SEC = 5.0
# Provisional physical departure floor; tune from measured battery/load behavior.
DEFAULT_MIN_BATTERY_VOLTAGE = 10.8


def assess_publishers(counts, estimation_location='pc', localization_mode='ekf_independent'):
    """Return concrete reasons the PC must not become the /odom owner yet."""
    required = list(REQUIRED_TOPICS)
    forbidden = FORBIDDEN_TOPICS
    if estimation_location == 'robot':
        required = ['/joint_states', '/imu', '/scan', '/odom', '/battery_state',
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


def assess_sensor_stamps(stamps, now_ns, *, topics=STAMPED_TOPICS,
                         max_age_sec=1.0, future_tolerance_sec=0.25):
    """Check that real sensor messages have recently arrived in PC time."""
    reasons = []
    for topic in topics:
        stamp_ns = stamps.get(topic)
        if stamp_ns is None:
            reasons.append(f'{topic}: no sample received')
            continue
        if stamp_ns <= 0:
            reasons.append(f'{topic}: zero/invalid header timestamp')
            continue
        offset_sec = (now_ns - stamp_ns) / 1e9
        # BatteryState is published less often than the motion sensors. It
        # must still be fresh, but should not inherit their 1 s deadline.
        age_limit = BATTERY_MAX_AGE_SEC if topic == '/battery_state' else max_age_sec
        if offset_sec > age_limit or offset_sec < -future_tolerance_sec:
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


def assess_required_nodes(nodes, localization_mode='ekf_independent'):
    """A publisher count alone can outlive the robot driver during discovery."""
    required = {'turtlebot3_node', 'lidar_node', 'motion_mux', 'localization_health'}
    if localization_mode == 'ekf_independent':
        required.update({'wheel_joint_odometry', 'imu_ekf_conditioner',
                         'ekf_hardware', 'ekf_sensor_guard'})
    present = {name for name, namespace in nodes if namespace == '/'}
    return [f'/{name}: required robot node not present' for name in sorted(required-present)]


class StableSensorWindow:
    """Require continuously healthy graph and advancing raw sensor stamps."""

    def __init__(self, topics, dwell_seconds=3.0, min_distinct_samples=3):
        if dwell_seconds <= 0 or min_distinct_samples < 2:
            raise ValueError('Dwell must be positive and distinct sample count at least 2')
        self.topics = tuple(topics)
        self.dwell_seconds = dwell_seconds
        self.min_distinct_samples = min_distinct_samples
        self.reset()

    def reset(self):
        self.healthy_since = None
        self.last_stamp = {}
        self.distinct_samples = {topic: 0 for topic in self.topics}

    def observe(self, stamps, monotonic_now, reasons):
        if reasons:
            self.reset()
            return False
        if self.healthy_since is None:
            self.healthy_since = monotonic_now
        for topic in self.topics:
            stamp = stamps.get(topic)
            previous = self.last_stamp.get(topic)
            if stamp is None or (previous is not None and stamp < previous):
                self.reset()
                return False
            if previous is None or stamp > previous:
                self.distinct_samples[topic] += 1
                self.last_stamp[topic] = stamp
        return (monotonic_now-self.healthy_since >= self.dwell_seconds and
                all(count >= self.min_distinct_samples
                    for count in self.distinct_samples.values()))

    def pending(self, monotonic_now):
        if self.healthy_since is None:
            return 'healthy observation window not started'
        missing = [topic for topic, count in self.distinct_samples.items()
                   if count < self.min_distinct_samples]
        if missing:
            return 'raw sensor stamps not advancing: '+', '.join(missing)
        return (f'healthy observation window '
                f'{monotonic_now-self.healthy_since:.1f}/{self.dwell_seconds:.1f}s')


def monitored_topics(estimation_location, localization_mode):
    """Separate freshness deadlines from 3 s advancement requirements."""
    freshness = (ROBOT_STAMPED_TOPICS if estimation_location == 'robot'
                 and localization_mode == 'ekf_independent' else
                 STAMPED_TOPICS + ('/odom', '/battery_state')
                 if estimation_location == 'robot' else STAMPED_TOPICS)
    progress = tuple(topic for topic in freshness if topic != '/battery_state')
    return freshness, progress


def valid_battery_voltage(voltage, minimum_voltage=0.0):
    """Reject missing, nonfinite, or below-configured physical voltage."""
    try:
        value = float(voltage)
    except (TypeError, ValueError, OverflowError):
        return False
    # BatteryState.voltage is float32; 10.8 V can arrive slightly below 10.8.
    return math.isfinite(value) and value > 0.0 and value >= minimum_voltage - 0.001


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=12.0,
                        help='Seconds to wait for the robot DDS endpoints')
    parser.add_argument('--dwell', type=float, default=3.0,
                        help='Continuous healthy observation seconds before passing')
    parser.add_argument('--minimum-battery-voltage', type=float,
                        default=os.environ.get('SMART_VISION_MIN_BATTERY_VOLTAGE',
                                               str(DEFAULT_MIN_BATTERY_VOLTAGE)),
                        help='Physical start floor in V (default 10.8; env SMART_VISION_MIN_BATTERY_VOLTAGE)')
    parser.add_argument('--estimation-location', choices=('pc', 'robot'), default='pc')
    parser.add_argument('--localization-mode', choices=('baseline', 'ekf_independent'), default='ekf_independent')
    args = parser.parse_args()
    if args.timeout <= 0 or args.dwell <= 0 or args.dwell >= args.timeout:
        parser.error('--timeout must exceed positive --dwell')
    if not math.isfinite(args.minimum_battery_voltage) or args.minimum_battery_voltage <= 0:
        parser.error('--minimum-battery-voltage must be a finite positive number')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('hardware_ekf_preflight')
    try:
        stamps = {}
        battery_voltage = None

        def receive(topic, message):
            nonlocal battery_voltage
            stamps[topic] = (message.header.stamp.sec * 1_000_000_000 +
                             message.header.stamp.nanosec)
            if topic == '/battery_state':
                battery_voltage = message.voltage

        topics = [('/joint_states', JointState), ('/imu', Imu), ('/scan', LaserScan)]
        if args.estimation_location == 'robot':
            topics.extend((('/odom', Odometry), ('/battery_state', BatteryState)))
            if args.localization_mode == 'ekf_independent':
                topics.append(('/wheel/odom_raw', Odometry))
        subscriptions = [node.create_subscription(
            message_type, topic, lambda message, topic=topic: receive(topic, message),
            qos_profile_sensor_data)
            for topic, message_type in topics]
        deadline = time.monotonic() + args.timeout
        # Require three distinct motion/scan/odom stamps during the full
        # healthy dwell. A slower battery publisher must remain fresh and
        # above the voltage floor, but need not produce three samples in 3 s.
        freshness_topics, progress_topics = monitored_topics(
            args.estimation_location, args.localization_mode)
        progress = StableSensorWindow(progress_topics, args.dwell)
        reasons = []
        while time.monotonic() < deadline:
            # Allow discovery to settle, then inspect endpoints from this
            # participant's graph. Do not use ros2 daemon results.
            rclpy.spin_once(node, timeout_sec=0.2)
            counts = {topic: len(node.get_publishers_info_by_topic(topic))
                      for topic in (*REQUIRED_TOPICS, *FORBIDDEN_TOPICS, '/wheel/odom', '/imu/ekf',
                                    '/battery_state', '/smart_vision_amr/motion_status',
                                    '/smart_vision_amr/localization_health')}
            now_ns = node.get_clock().now().nanoseconds
            reasons = assess_publishers(counts, args.estimation_location,
                                        args.localization_mode) + assess_sensor_stamps(
                stamps, now_ns, topics=freshness_topics)
            if args.estimation_location == 'robot':
                owner_topics = ('/cmd_vel', '/odom', '/wheel/odom', '/imu/ekf',
                                '/smart_vision_amr/motion_status',
                                '/smart_vision_amr/localization_health')
                owners = {topic: [endpoint.node_name for endpoint in
                                  node.get_publishers_info_by_topic(topic)]
                          for topic in owner_topics}
                reasons.extend(assess_runtime_owners(owners, args.localization_mode))
                reasons.extend(assess_required_nodes(
                    node.get_node_names_and_namespaces(), args.localization_mode))
                if not valid_battery_voltage(battery_voltage, args.minimum_battery_voltage):
                    reasons.append('/battery_state: voltage missing, invalid, or below '
                                   f'{args.minimum_battery_voltage:.2f}V '
                                   f'(received {battery_voltage!r}V)')
            if progress.observe(stamps, time.monotonic(), reasons):
                print('Robot preflight passed: '+(
                    'robot-local odom/mux/health, continuously advancing raw sensors, '
                    f'battery {battery_voltage:.2f}V >= {args.minimum_battery_voltage:.2f}V.'
                    if args.estimation_location == 'robot'
                    else 'fresh raw wheels/IMU/scan; /odom unowned.'))
                return 0
        if not reasons:
            reasons = [progress.pending(time.monotonic())]
        print('Independent EKF robot preflight failed: ' + '; '.join(reasons),
              file=sys.stderr, flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
