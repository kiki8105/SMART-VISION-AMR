#!/usr/bin/env python3
"""Read-only physical TurtleBot3/OpenCR stability check (default: five minutes).

Run after the robot's EKF bringup, before allowing a physical mission::

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /usr/bin/python3 scripts/soak_robot_opencr_16.py --json-out "$HOME/robot_opencr_soak.json"

This program subscribes to sensors and /rosout only. It does not publish,
call services, reset safety state, or command the robot to move. A pass means
the observed streams remained healthy for the configured interval; it cannot
prove that the battery or OpenCR will remain healthy on a later drive.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


MIN_VOLTAGE_DEFAULT = 11.2  # Provisional trial floor, not a battery specification.
VOLTAGE_FLOAT32_EPSILON = 0.001
STREAM_MAX_GAP = {
    '/joint_states': 2.0,
    '/imu': 2.0,
    '/wheel/odom_raw': 2.0,
    '/scan': 2.0,
    '/odom': 2.0,
    '/battery_state': 5.0,
}
DRIVER_NODE = ('turtlebot3_node', '/')


class SoakWindow:
    """Pure state machine so fault behavior can be tested without ROS hardware."""

    def __init__(self, *, duration=300.0, startup_grace=15.0,
                 min_voltage=MIN_VOLTAGE_DEFAULT, start_mono=0.0,
                 max_stamp_age=2.0, future_tolerance=0.5):
        if not all(math.isfinite(v) for v in (
                duration, startup_grace, min_voltage, start_mono,
                max_stamp_age, future_tolerance)):
            raise ValueError('Soak limits must be finite')
        if (duration <= 0 or startup_grace <= 0 or min_voltage <= 0 or
                max_stamp_age <= 0 or future_tolerance < 0):
            raise ValueError('Invalid soak limit')
        self.duration = duration
        self.startup_grace = startup_grace
        self.min_voltage = min_voltage
        self.start_mono = start_mono
        self.max_stamp_age = max_stamp_age
        self.future_tolerance = future_tolerance
        self.ready_since = None
        self.failure = None
        self.complete = False
        self.voltage = None
        self.streams = {name: {
            'count': 0, 'advances': 0, 'last_arrival': None,
            'last_advance': None, 'last_stamp': None,
            'last_skew_sec': None, 'min_skew_sec': None, 'max_skew_sec': None,
        } for name in STREAM_MAX_GAP}

    def fail(self, reason):
        if self.failure is None:
            self.failure = str(reason)

    def stamp_age_limit(self, topic):
        # BatteryState can have a slower publication period than motion data.
        return max(self.max_stamp_age, STREAM_MAX_GAP['/battery_state']) \
            if topic == '/battery_state' else self.max_stamp_age

    def record(self, topic, stamp_ns, now_mono, now_wall_ns, *, voltage=None):
        if self.failure or self.complete:
            return
        if topic not in self.streams:
            raise KeyError(topic)
        stream = self.streams[topic]
        if stamp_ns <= 0:
            self.fail(f'{topic}: invalid header stamp')
            return
        skew = (now_wall_ns - stamp_ns) / 1e9
        age_limit = self.stamp_age_limit(topic)
        if skew > age_limit or skew < -self.future_tolerance:
            self.fail(f'{topic}: sensor stamp offset {skew:+.3f}s exceeds '
                      f'[-{self.future_tolerance:.1f}, +{age_limit:.1f}]s')
            return
        prior = stream['last_stamp']
        if prior is not None and stamp_ns < prior:
            self.fail(f'{topic}: sensor stamp moved backwards')
            return
        stream['count'] += 1
        stream['last_arrival'] = now_mono
        stream['last_skew_sec'] = skew
        stream['min_skew_sec'] = (skew if stream['min_skew_sec'] is None else
                                  min(stream['min_skew_sec'], skew))
        stream['max_skew_sec'] = (skew if stream['max_skew_sec'] is None else
                                  max(stream['max_skew_sec'], skew))
        if prior is None or stamp_ns > prior:
            stream['advances'] += 1
            stream['last_advance'] = now_mono
            stream['last_stamp'] = stamp_ns
        if topic == '/battery_state':
            try:
                self.voltage = float(voltage)
            except (TypeError, ValueError, OverflowError):
                self.voltage = None
            if self.voltage is None or not math.isfinite(self.voltage):
                self.fail('/battery_state: no valid voltage')
            elif self.voltage < self.min_voltage - VOLTAGE_FLOAT32_EPSILON:
                self.fail(f'/battery_state: {self.voltage:.2f}V below trial floor '
                          f'{self.min_voltage:.2f}V')

    def record_log(self, logger, severity, message):
        if self.failure or self.complete:
            return
        name = str(logger).lower()
        body = str(message).lower()
        if severity < 40:  # rcl_interfaces/msg/Log.ERROR
            return
        if ('dynamixelsdkwrapper' in name and
                any(term in body for term in ('failed', 'no status packet', 'error'))):
            self.fail(f'OpenCR/Dynamixel /rosout error: {message}')
        elif ('turtlebot3_node' in name and
              'failed connection with devices' in body):
            self.fail(f'TurtleBot3 driver /rosout error: {message}')

    def tick(self, now_mono, now_wall_ns, *, nodes, publisher_counts):
        if self.failure or self.complete:
            return
        elapsed = now_mono - self.start_mono
        reasons = []
        if DRIVER_NODE not in set(nodes):
            reasons.append('/turtlebot3_node: driver node missing')
        for topic, limit in STREAM_MAX_GAP.items():
            count = publisher_counts.get(topic, 0)
            if count != 1:
                reasons.append(f'{topic}: expected one publisher, found {count}')
            stream = self.streams[topic]
            if stream['advances'] < 2:
                reasons.append(f'{topic}: fewer than two advancing samples')
                continue
            if now_mono - stream['last_arrival'] > limit:
                reasons.append(f'{topic}: no message for >{limit:.1f}s')
            if now_mono - stream['last_advance'] > limit:
                reasons.append(f'{topic}: header stamp stopped advancing for >{limit:.1f}s')
            skew = (now_wall_ns - stream['last_stamp']) / 1e9
            if skew > self.stamp_age_limit(topic) or skew < -self.future_tolerance:
                reasons.append(f'{topic}: latest stamp offset {skew:+.3f}s')
        if self.voltage is None:
            reasons.append('/battery_state: voltage missing')
        if reasons:
            # Give DDS discovery and the first two samples a bounded startup
            # grace. Once healthy observation begins, any regression fails.
            if self.ready_since is not None or elapsed >= self.startup_grace:
                self.fail('; '.join(reasons))
            return
        if self.ready_since is None:
            self.ready_since = now_mono
        if now_mono - self.ready_since >= self.duration:
            self.complete = True

    def report(self, now_mono):
        status = ('failed' if self.failure else 'passed' if self.complete
                  else 'interrupted')
        return {
            'status': status,
            'reason': self.failure,
            'observed_sec': round(max(0.0, now_mono - self.ready_since), 1)
            if self.ready_since is not None else 0.0,
            'required_sec': self.duration,
            'minimum_battery_voltage': self.min_voltage,
            'last_battery_voltage': self.voltage,
            'streams': {topic: {
                'messages': stream['count'],
                'advancing_samples': stream['advances'],
                'last_stamp_offset_sec': stream['last_skew_sec'],
                'min_stamp_offset_sec': stream['min_skew_sec'],
                'max_stamp_offset_sec': stream['max_skew_sec'],
            } for topic, stream in self.streams.items()},
        }


def _save_report(path, report):
    if path is None:
        return
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + '.tmp')
    temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(destination)


def rosout_qos_profile():
    """Match the reliable, volatile ROS 2 /rosout publisher."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=300.0,
                        help='Continuous healthy observation seconds (default 300)')
    parser.add_argument('--startup-grace', type=float, default=15.0,
                        help='DDS discovery and first-sample grace seconds (default 15)')
    parser.add_argument('--minimum-battery-voltage', type=float,
                        default=float(os.environ.get('SMART_VISION_MIN_BATTERY_VOLTAGE',
                                                     MIN_VOLTAGE_DEFAULT)))
    parser.add_argument('--json-out', help='Optional compact diagnostic JSON path')
    args = parser.parse_args()
    try:
        monitor = SoakWindow(duration=args.duration, startup_grace=args.startup_grace,
                             min_voltage=args.minimum_battery_voltage,
                             start_mono=time.monotonic())
    except ValueError as error:
        parser.error(str(error))

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Log

    rclpy.init()
    node = rclpy.create_node('robot_opencr_soak')
    topic_types = {
        '/joint_states': JointState,
        '/imu': Imu,
        '/wheel/odom_raw': Odometry,
        '/scan': LaserScan,
        '/odom': Odometry,
        '/battery_state': BatteryState,
    }

    def receive_sensor(topic, msg):
        stamp = msg.header.stamp
        monitor.record(topic, stamp.sec * 1_000_000_000 + stamp.nanosec,
                       time.monotonic(), time.time_ns(),
                       voltage=msg.voltage if topic == '/battery_state' else None)

    subscriptions = [node.create_subscription(
        msg_type, topic, lambda msg, topic=topic: receive_sensor(topic, msg),
        qos_profile_sensor_data) for topic, msg_type in topic_types.items()]
    subscriptions.append(node.create_subscription(
        Log, '/rosout', lambda msg: monitor.record_log(msg.name, msg.level, msg.msg),
        rosout_qos_profile()))
    next_report = time.monotonic()
    try:
        while rclpy.ok() and not monitor.failure and not monitor.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_report:
                monitor.tick(now, time.time_ns(),
                             nodes=node.get_node_names_and_namespaces(),
                             publisher_counts={topic: len(node.get_publishers_info_by_topic(topic))
                                               for topic in STREAM_MAX_GAP})
                observed = (now - monitor.ready_since) if monitor.ready_since else 0.0
                print(f'OpenCR soak: {observed:.0f}/{monitor.duration:.0f}s '
                      f'healthy; battery={monitor.voltage!r}V', flush=True)
                next_report = now + 1.0
    except KeyboardInterrupt:
        pass
    finally:
        report = monitor.report(time.monotonic())
        print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
        _save_report(args.json_out, report)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if monitor.complete else 1


if __name__ == '__main__':
    raise SystemExit(main())
