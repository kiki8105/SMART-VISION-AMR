#!/usr/bin/env python3
"""Relay real Gazebo scan/odom/TF with reproducible timing faults.

Only run against a copied Waffle Pi model whose publishers are remapped to
/classroom_fault/raw_*.  No synthetic scan, odometry, pose or TF is created.
The original message and timestamp are forwarded to the normal ROS topics.
The process refuses the physical robot domain and non-local DDS.
"""

import argparse
from collections import Counter
import heapq
import json
import os
from pathlib import Path
import time

from classroom_fault_profile import FaultProfile


def relay_topics(*, scan_only=False, ekf_delivery_faults=False):
    """Declare the graph; defaults retain the historical baseline relay."""
    if type(scan_only) is not bool or type(ekf_delivery_faults) is not bool:
        raise ValueError('relay scope flags must be boolean')
    if scan_only and ekf_delivery_faults:
        raise ValueError('scan-only and EKF output delivery are mutually exclusive')
    topics = {'scan': ('/classroom_fault/raw_scan', '/scan')}
    if not scan_only:
        topics.update(odom=('/classroom_fault/raw_odom', '/odom'),
                      tf=('/classroom_fault/raw_tf', '/tf'))
    return topics


def ekf_odom_transforms(message):
    """Select only the EKF-owned edge, preserving original transform objects."""
    return [transform for transform in message.transforms
            if transform.header.frame_id == 'odom'
            and transform.child_frame_id == 'base_footprint']


def tf_source_qos(best_effort):
    """Only the Gazebo raw-TF subscription changes in the QoS experiment."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    if type(best_effort) is not bool:
        raise ValueError('best_effort must be boolean')
    return QoSProfile(depth=20 if best_effort else 100,
                      reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort
                                   else ReliabilityPolicy.RELIABLE),
                      durability=DurabilityPolicy.VOLATILE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=176)
    parser.add_argument('--tf-source-best-effort', action='store_true',
                        help='Experiment: subscribe to raw Gazebo TF with best effort depth 20; '
                             'forwarded /tf remains reliable depth 100')
    parser.add_argument('--scan-only', action='store_true',
                        help='Independent-EKF simulation: relay only LiDAR; EKF alone owns /odom and odom TF')
    parser.add_argument('--ekf-delivery-faults', action='store_true',
                        help='Relay actual EKF output /odom and odom->base TF; not raw-input or CPU faults')
    args = parser.parse_args()
    try:
        topics = relay_topics(scan_only=args.scan_only,
                              ekf_delivery_faults=args.ekf_delivery_faults)
    except ValueError as exc:
        parser.error(str(exc))
    if args.domain == 16 or os.environ.get('ROS_DOMAIN_ID') != str(args.domain):
        raise SystemExit('Refusing physical or mismatched ROS domain')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Fault relay requires ROS_LOCALHOST_ONLY=1')
    args.output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class FaultRelay(Node):
        def __init__(self):
            super().__init__('classroom_fault_relay')
            self.policy = FaultProfile()
            self.pending = []
            self.order = 0
            self.counts = Counter()
            self.max_age = Counter()
            self.events = []
            self.last_report_at = time.monotonic()
            self.last_due = {topic: 0. for topic in ('scan', 'odom', 'tf')}
            self.file = args.output.open('w', encoding='utf-8')
            tf_qos = tf_source_qos(False)
            raw_tf_qos = tf_source_qos(args.tf_source_best_effort)
            qos = {'scan': qos_profile_sensor_data, 'odom': qos_profile_sensor_data,
                   'tf': tf_qos}
            message_types = {'scan': LaserScan, 'odom': Odometry, 'tf': TFMessage}
            spec = {name: (message_types[name], *endpoints)
                    for name, endpoints in topics.items()}
            self.relay_publishers = {}
            for topic, (message_type, source, destination) in spec.items():
                self.relay_publishers[topic] = self.create_publisher(message_type, destination, qos[topic])
                self.create_subscription(message_type, source,
                                         lambda message, kind=topic: self.received(kind, message),
                                         raw_tf_qos if topic == 'tf' else qos[topic])
            self.create_subscription(String, '/classroom_fault/control', self.control, 10)
            self.create_timer(.01, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(1., self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.write({'event': 'relay_config',
                        'tf_source_reliability': ('BEST_EFFORT' if args.tf_source_best_effort
                                                  else 'RELIABLE'),
                        'tf_source_depth': raw_tf_qos.depth,
                        'tf_output_reliability': 'RELIABLE', 'tf_output_depth': tf_qos.depth,
                        'scan_only': args.scan_only,
                        'ekf_delivery_faults': args.ekf_delivery_faults,
                        'streams': list(topics),
                        'source_stamps_preserved': True,
                        'ekf_cpu_or_raw_input_faults': False})

        def write(self, event):
            self.file.write(json.dumps(event, sort_keys=True)+'\n')
            self.file.flush()
            self.events.append(event)

        def control(self, message):
            try:
                payload = json.loads(message.data)
                profile = payload['profile']
                now = time.monotonic()
                self.policy.set_profile(profile, now)
                self.write({'event': 'profile', 'name': profile, 'monotonic': now})
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.get_logger().warning('Invalid fault control: '+str(exc))

        def received(self, topic, message):
            if topic == 'tf' and args.ekf_delivery_faults:
                selected = ekf_odom_transforms(message)
                self.counts['tf_filtered_unowned_edges'] += len(message.transforms)-len(selected)
                if not selected:
                    return
                message = TFMessage(transforms=selected)
            now = time.monotonic()
            decision = self.policy.decide(topic, now)
            self.counts[topic+'_received'] += 1
            if decision.drop:
                self.counts[topic+'_dropped_'+decision.reason] += 1
                return
            # Per-stream order cannot overtake an earlier delayed packet.
            due = max(now+decision.delay_seconds, self.last_due[topic]+1e-6)
            self.last_due[topic] = due
            self.order += 1
            heapq.heappush(self.pending, (due, self.order, topic, message, now))
            if len(self.pending) > 500:
                # Overload is explicit loss, never an unbounded stale queue.
                _, _, discarded, _, _ = heapq.heappop(self.pending)
                self.counts[discarded+'_dropped_overload'] += 1

        def flush(self):
            now = time.monotonic()
            while self.pending and self.pending[0][0] <= now:
                _, _, topic, message, received = heapq.heappop(self.pending)
                self.relay_publishers[topic].publish(message)
                self.counts[topic+'_forwarded'] += 1
                self.max_age[topic] = max(self.max_age[topic], round(now-received, 3))

        def report(self):
            self.write({'event': 'sample', 'monotonic': time.monotonic(),
                        'profile': self.policy.name, 'counts': dict(self.counts),
                        'max_delivery_delay_s': dict(self.max_age),
                        'pending': len(self.pending)})

        def close(self):
            self.report()
            self.file.close()

    rclpy.init(args=[])
    node = FaultRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
