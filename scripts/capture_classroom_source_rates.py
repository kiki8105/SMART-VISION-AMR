#!/usr/bin/env python3
"""Observe actual Gazebo source stamps and receipt cadence; never relay data.

Run in an isolated localhost-only domain, for example:
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 python3 scripts/capture_classroom_source_rates.py \
  --output /tmp/source-rates.json --duration 60 --ekf-delivery-faults

The EKF option selects its producer /classroom_fault/raw_odom, not delayed /odom.
Source Hz uses ROS header stamps; receipt Hz uses time.monotonic_ns(). Neither
rate is inferred from a YAML/SDF update_rate. Source ages use the actual /clock.
"""

import argparse
from array import array
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import time


def require_isolated_domain(domain, environment):
    if (type(domain) is not int or not 0 <= domain <= 232 or domain == 16
            or environment.get('ROS_DOMAIN_ID') != str(domain)
            or environment.get('ROS_LOCALHOST_ONLY') != '1'):
        raise ValueError('Requires a matching non-16 ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')


def selected_topics(ekf_delivery_faults=False):
    return {'raw_scan': '/classroom_fault/raw_scan', 'raw_imu': '/imu',
            'wheel_joint_states': '/joint_states',
            'ekf_output': '/classroom_fault/raw_odom' if ekf_delivery_faults else '/odom'}


def positive_duration(value):
    number = float(value)
    if not math.isfinite(number) or number < 0.:
        raise ValueError('duration must be finite and nonnegative')
    return number


def header_stamp_ns(message):
    try:
        sec = message.header.stamp.sec
        nanosec = message.header.stamp.nanosec
    except AttributeError:
        return None
    if (type(sec) is not int or sec < 0 or type(nanosec) is not int
            or not 0 <= nanosec < 1_000_000_000):
        return None
    stamp = sec*1_000_000_000+nanosec
    return stamp if stamp > 0 else None


def interval_metrics(intervals):
    if not intervals:
        return {'sample_count': 0, 'minimum_seconds': None,
                'p95_seconds': None, 'maximum_seconds': None}
    ordered = sorted(intervals)
    return {'sample_count': len(ordered), 'minimum_seconds': ordered[0],
            'p95_seconds': ordered[math.ceil(.95*len(ordered))-1],
            'maximum_seconds': ordered[-1]}


class SourceCadence:
    """Pure measurement accumulator; input stamps are never rewritten/reset.

    Duplicate and backward source stamps do not enter source cadence. The last
    accepted stamp is a high-water mark: a reset remains visible as backward
    samples until surpassed. Start a new observer for a new simulation epoch.
    Receipt counts still include these messages, and their valid stamp ages
    remain useful evidence of stale/out-of-order delivery.
    """

    def __init__(self):
        self.received = 0
        self.first_receipt = self.last_receipt = None
        self.receipt_valid = self.receipt_nonincreasing = self.receipt_invalid = 0
        self.first_source = self.last_source = None
        self.source_forward = self.source_duplicate = self.source_backward = 0
        self.source_invalid = self.clock_unavailable = 0
        self.receipt_intervals = array('d')
        self.source_intervals = array('d')
        self.age_count = self.age_future = 0
        self.age_min = self.age_max = None
        self.head, self.tail = [], deque(maxlen=8)

    def observe(self, source_stamp_ns, receipt_monotonic_ns, ros_now_ns):
        self.received += 1
        witness = {name: value if type(value) is int else None for name, value in
                   (('source_stamp_ns', source_stamp_ns),
                    ('receipt_monotonic_ns', receipt_monotonic_ns),
                    ('ros_now_ns', ros_now_ns))}
        if len(self.head) < 8:
            self.head.append(witness)
        self.tail.append(witness)
        if type(receipt_monotonic_ns) is not int or receipt_monotonic_ns < 0:
            self.receipt_invalid += 1
        elif self.last_receipt is None:
            self.first_receipt = self.last_receipt = receipt_monotonic_ns
            self.receipt_valid += 1
        elif receipt_monotonic_ns <= self.last_receipt:
            self.receipt_nonincreasing += 1
        else:
            self.receipt_intervals.append((receipt_monotonic_ns-self.last_receipt)/1.e9)
            self.last_receipt = receipt_monotonic_ns
            self.receipt_valid += 1

        if type(source_stamp_ns) is not int or source_stamp_ns <= 0:
            self.source_invalid += 1
            return
        if type(ros_now_ns) is int and ros_now_ns > 0:
            age = (ros_now_ns-source_stamp_ns)/1.e9
            self.age_count += 1
            self.age_future += age < 0.
            self.age_min = age if self.age_min is None else min(self.age_min, age)
            self.age_max = age if self.age_max is None else max(self.age_max, age)
        else:
            self.clock_unavailable += 1
        if self.last_source is None:
            self.first_source = self.last_source = source_stamp_ns
            self.source_forward += 1
        elif source_stamp_ns == self.last_source:
            self.source_duplicate += 1
        elif source_stamp_ns < self.last_source:
            self.source_backward += 1
        else:
            self.source_intervals.append((source_stamp_ns-self.last_source)/1.e9)
            self.last_source = source_stamp_ns
            self.source_forward += 1

    def summary(self):
        def rate(count, first, last):
            return ((count-1)*1.e9/(last-first)
                    if count > 1 and last is not None and first is not None and last > first else None)
        return {'receipt_sample_count': self.received,
                'receipt_wall_hz': rate(self.receipt_valid, self.first_receipt, self.last_receipt),
                'receipt_monotonic_intervals': interval_metrics(self.receipt_intervals),
                'receipt_first_monotonic_ns': self.first_receipt,
                'receipt_last_monotonic_ns': self.last_receipt,
                'receipt_valid_clock_samples': self.receipt_valid,
                'receipt_invalid_clock_samples': self.receipt_invalid,
                'receipt_nonincreasing_clock_samples': self.receipt_nonincreasing,
                'source_stamp_hz': rate(self.source_forward, self.first_source, self.last_source),
                'source_stamp_intervals': interval_metrics(self.source_intervals),
                'source_first_accepted_stamp_ns': self.first_source,
                'source_last_accepted_stamp_ns': self.last_source,
                'source_forward_samples': self.source_forward,
                'source_duplicate_samples': self.source_duplicate,
                'source_backward_samples': self.source_backward,
                'source_missing_or_invalid_samples': self.source_invalid,
                'source_age_ros_seconds': {'sample_count': self.age_count,
                    'minimum': self.age_min, 'maximum': self.age_max,
                    'future_stamp_samples': self.age_future,
                    'clock_unavailable_samples': self.clock_unavailable},
                'witness_first_samples': self.head, 'witness_last_samples': list(self.tail)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=176)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=positive_duration, default=0.,
                        help='Wall seconds; 0 observes until SIGINT/SIGTERM')
    parser.add_argument('--report-period', type=float, default=2.,
                        help='Atomic summary refresh period in wall seconds (default 2)')
    parser.add_argument('--ekf-delivery-faults', action='store_true',
                        help='Observe actual EKF producer /classroom_fault/raw_odom instead of /odom')
    args = parser.parse_args()
    try:
        require_isolated_domain(args.domain, os.environ)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.report_period) or args.report_period <= 0.:
        parser.error('--report-period must be finite and positive')
    if args.output.exists():
        parser.error('Refusing to overwrite '+str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, LaserScan

    rclpy.init(args=[])
    node = Node('classroom_source_rate_witness', enable_rosout=False,
                start_parameter_services=False,
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    # Humble creates this internal telemetry publisher even for subscriber-only
    # nodes. Drop it before spinning: this observer has no active publishers,
    # parameter services, source restamping, relay or motion client.
    parameter_events = getattr(node, '_parameter_event_publisher', None)
    if parameter_events is not None:
        node.destroy_publisher(parameter_events)
        node._parameter_event_publisher = None
    topics = selected_topics(args.ekf_delivery_faults)
    measurements = {key: SourceCadence() for key in topics}
    kinds = {'raw_scan': LaserScan, 'raw_imu': Imu,
             'wheel_joint_states': JointState, 'ekf_output': Odometry}
    subscriptions = []
    for key, topic in topics.items():
        def received(message, name=key):
            measurements[name].observe(header_stamp_ns(message), time.monotonic_ns(),
                                       node.get_clock().now().nanoseconds)
        subscriptions.append(node.create_subscription(
            kinds[key], topic, received, qos_profile_sensor_data))

    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started, started_wall = time.monotonic(), time.time()
    temporary = args.output.with_name(args.output.name+'.tmp')

    def write(final=False):
        report = {'schema': 1, 'domain': args.domain, 'localhost_only': True,
                  'read_only': True, 'active_publishers': len(list(node.publishers)),
                  'use_sim_time': node.get_parameter('use_sim_time').value,
                  'ekf_delivery_faults': args.ekf_delivery_faults,
                  'started_at_wall': started_wall, 'updated_at_wall': time.time(),
                  'observed_wall_seconds': time.monotonic()-started, 'final': final,
                  'topics': {name: {'topic': topics[name], **value.summary()}
                             for name, value in measurements.items()},
                  'definitions': {
                      'source_stamp_hz': '(accepted forward stamps - 1) / source ROS-time span; not configuration or receipt rate',
                      'receipt_wall_hz': '(valid monotonic receipt samples - 1) / monotonic wall-time span; includes messages with duplicate/backward/missing source stamps',
                      'p95': 'nearest-rank 95th percentile of all measured positive intervals; no configured rate or zero interval is substituted',
                      'age': 'actual ROS clock at receipt minus original valid header stamp; includes duplicate/backward messages and preserves negative ages',
                      'clock_reset': 'backward stamps are excluded until previous high-water stamp is surpassed; use a new observer per simulation epoch',
                      'limitations': 'subscriber receipt cadence can include observer scheduling/DDS delay; source stamps prove timestamp cadence, not sensor accuracy or real-hardware rates'}}
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temporary.replace(args.output)

    try:
        write()
        next_report = started+args.report_period
        while running and rclpy.ok() and (not args.duration or time.monotonic()-started < args.duration):
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_report:
                write()
                next_report = time.monotonic()+args.report_period
    except KeyboardInterrupt:
        pass
    finally:
        write(final=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(args.output, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
