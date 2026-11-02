"""Conservative two-scan check for returns at the LiDAR minimum range.

This filters only the *map-change evidence* stream. The original LaserScan is
left intact for Nav2's collision costmap and the immediate motion interlock.
Gazebo and real LiDAR edge echoes can cover several adjacent beams in *one*
scan, so spatial support alone is not evidence of a persistent obstacle.
Every near-minimum cluster is unknown until the next scan, not free space.
"""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class NearScanDecision:
    unconfirmed: frozenset[int]
    confirmed: frozenset[int]
    accepted: bool = True


class NearScanOutlierGuard:
    def __init__(self, *, above_min_m=0.10, absolute_max_m=0.30,
                 temporal_delta_m=0.12, temporal_window_sec=0.75):
        values = (above_min_m, absolute_max_m, temporal_delta_m,
                  temporal_window_sec)
        if not all(isfinite(value) and value > 0 for value in values):
            raise ValueError("near-scan guard limits must be finite and positive")
        self.above_min_m = above_min_m
        self.absolute_max_m = absolute_max_m
        self.temporal_delta_m = temporal_delta_m
        self.temporal_window_ns = int(temporal_window_sec * 1e9)
        self.reset()

    def reset(self):
        self.last_stamp_ns = -1
        self.pending = ()  # (bearing, distance) seen in the immediately previous scan

    def classify(self, scan, stamp_ns):
        """Return suspect beam indices; never infer a free ray from a suspect.

        A matching bearing and range on the *next* scan establishes temporal
        support, including for a wide adjacent-beam cluster. Out-of-order or
        replayed samples cannot confirm a ghost or clear the pending hold.
        """
        if (type(stamp_ns) is not int or stamp_ns <= self.last_stamp_ns
                or not isfinite(scan.range_min) or not isfinite(scan.range_max)
                or not isfinite(scan.angle_min) or not isfinite(scan.angle_increment)
                or scan.angle_increment == 0 or not 0 <= scan.range_min < scan.range_max
                or not scan.ranges
                or not any((isfinite(value) and scan.range_min <= value <= scan.range_max)
                           or value == float('inf') for value in scan.ranges)):
            return NearScanDecision(frozenset(), frozenset(), False)
        limit = min(self.absolute_max_m, scan.range_min + self.above_min_m)
        candidates = [(i, scan.angle_min + i * scan.angle_increment, float(value))
                      for i, value in enumerate(scan.ranges)
                      if isfinite(value) and scan.range_min <= value <= limit]
        previous = self.pending if stamp_ns - self.last_stamp_ns <= self.temporal_window_ns else ()
        unconfirmed, confirmed = set(), set()
        for index, bearing, distance in candidates:
            temporal = any(
                abs(old_bearing-bearing) <= max(.04, 2.5*abs(scan.angle_increment))
                and abs(old_distance-distance) <= self.temporal_delta_m
                for old_bearing, old_distance in previous)
            (confirmed if temporal else unconfirmed).add(index)
        self.pending = tuple((bearing, distance) for i, bearing, distance in candidates
                             if i in unconfirmed or i in confirmed)
        self.last_stamp_ns = stamp_ns
        return NearScanDecision(frozenset(unconfirmed), frozenset(confirmed))
