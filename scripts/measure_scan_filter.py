#!/usr/bin/env python3
"""Pair raw and optional filtered scans by ROS stamp during isolated Gazebo A/B."""

import argparse
from collections import OrderedDict
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    return round(ordered[lower] + (ordered[upper] - ordered[lower]) *
                 (position - lower), 3)


class ScanPairObserver(Node):
    def __init__(self, output: Path):
        super().__init__('scan_filter_pair_observer')
        self.output = output
        self.raw_count = 0
        self.filtered_count = 0
        self.paired = 0
        self.removed_beams = 0
        self.singleton_beams = 0
        self.singleton_removed = 0
        self.lags_ms = []
        self.raw = OrderedDict()
        self.filtered = OrderedDict()
        self.create_subscription(LaserScan, '/scan', self.on_raw, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_filtered_for_change',
                                 self.on_filtered, qos_profile_sensor_data)
        self.create_timer(5.0, self.write)

    @staticmethod
    def key(scan: LaserScan):
        return (scan.header.frame_id, scan.header.stamp.sec,
                scan.header.stamp.nanosec)

    def on_raw(self, scan: LaserScan):
        self.raw_count += 1
        key = self.key(scan)
        self.raw[key] = (time.monotonic_ns(), scan)
        self.raw.move_to_end(key)
        self.pair(key)

    def on_filtered(self, scan: LaserScan):
        self.filtered_count += 1
        key = self.key(scan)
        self.filtered[key] = (time.monotonic_ns(), scan)
        self.filtered.move_to_end(key)
        self.pair(key)

    def pair(self, key):
        if key not in self.raw or key not in self.filtered:
            # Keep memory bounded if one topic stops or a simulated clock resets.
            for pending in (self.raw, self.filtered):
                while len(pending) > 2000:
                    pending.popitem(last=False)
            return
        raw_at, raw_scan = self.raw.pop(key)
        filtered_at, filtered_scan = self.filtered.pop(key)
        self.paired += 1
        self.lags_ms.append((filtered_at - raw_at) / 1e6)
        ranges = raw_scan.ranges
        filtered = filtered_scan.ranges
        for index, (before, after) in enumerate(zip(ranges, filtered)):
            if not (math.isfinite(before) and raw_scan.range_min <= before <= 1.0):
                continue
            removed = math.isnan(after)
            self.removed_beams += removed
            left = ranges[index - 1] if index else ranges[-1]
            right = ranges[(index + 1) % len(ranges)]
            singleton = all(not math.isfinite(neighbor) or
                            abs(neighbor - before) > .10 for neighbor in (left, right))
            self.singleton_beams += singleton
            self.singleton_removed += singleton and removed

    def write(self):
        data = {
            'raw_frames': self.raw_count,
            'filtered_frames': self.filtered_count,
            'same_stamp_paired': self.paired,
            'filtered_drop_fraction': (round(max(0, self.raw_count - self.paired) /
                                             self.raw_count, 5)
                                       if self.raw_count and self.filtered_count else None),
            'filter_added_arrival_lag_ms_p50': percentile(self.lags_ms, .50),
            'filter_added_arrival_lag_ms_p95': percentile(self.lags_ms, .95),
            'filter_added_arrival_lag_ms_p99': percentile(self.lags_ms, .99),
            'raw_near_singleton_beams': self.singleton_beams,
            'raw_near_singleton_beams_removed': self.singleton_removed,
            'raw_near_beams_removed': self.removed_beams,
            'note': 'Callback arrival lag includes DDS and scheduling; it is not pure filter CPU time.',
        }
        temporary = self.output.with_suffix(self.output.suffix + '.tmp')
        temporary.write_text(json.dumps(data, indent=2) + '\n')
        temporary.replace(self.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    observer = ScanPairObserver(args.output)
    signal.signal(signal.SIGTERM, lambda _signum, _frame: rclpy.shutdown())
    try:
        rclpy.spin(observer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        observer.write()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
