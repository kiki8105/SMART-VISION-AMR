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


@dataclass(frozen=True)
class SweptBodyDecision:
    inside: frozenset[int]
    swept: frozenset[int]
    minimum_body_clearance: float
    minimum_swept_clearance: float


def scan_points_in_base(scan, extrinsic):
    """Transform valid raw returns by the known planar base <- laser mounting.

    No ranges, evidence classes or sensor messages are modified. Infinite and
    invalid returns contain no measured obstacle point and remain unchanged.
    """
    from math import cos, sin
    tx, ty, yaw = extrinsic
    if not all(isfinite(value) for value in extrinsic):
        raise ValueError('Laser mounting transform must be finite')
    c, s = cos(yaw), sin(yaw)
    points = []
    for index, value in enumerate(scan.ranges):
        if not isfinite(value) or not scan.range_min <= value <= scan.range_max:
            continue
        bearing = scan.angle_min+index*scan.angle_increment
        x, y = value*cos(bearing), value*sin(bearing)
        points.append((index, tx+c*x-s*y, ty+s*x+c*y))
    return tuple(points)


def rebase_scan_points(points, scan_pose, current_pose):
    """Move acquisition-time base coordinates through continuous odometry."""
    from math import cos, sin
    if not all(isfinite(value) for pose in (scan_pose, current_pose) for value in pose):
        raise ValueError('Scan and current odometry poses must be finite')
    sx, sy, sa = scan_pose
    tx, ty, ta = current_pose
    c, d = cos(sa-ta), sin(sa-ta)
    dx, dy = cos(ta)*(sx-tx)+sin(ta)*(sy-ty), -sin(ta)*(sx-tx)+cos(ta)*(sy-ty)
    return tuple((index, dx+c*x-d*y, dy+d*x+c*y) for index, x, y in points)


def swept_body_decision(points, linear, angular, *, radius=.22, horizon=.5,
                        longitudinal_uncertainty=0., lateral_uncertainty=0.,
                        yaw_uncertainty=0.):
    """Point/circular-body sweep, including bounded age of the latest pose.

    Scan delay is handled by odometry rebasing, never by radial padding. Only
    motion since the latest odometry pose adds a longitudinal interval and a
    bounded lateral/yaw uncertainty. Parallel walls therefore do not inherit
    the full speed*scan_age as a fictitious all-direction collision radius.
    """
    from math import atan2, cos, hypot, pi, sin, sqrt
    if (not all(isfinite(v) for v in (linear, angular, radius, horizon,
            longitudinal_uncertainty, lateral_uncertainty, yaw_uncertainty))
            or radius <= 0. or not 0. < horizon <= 2.
            or min(longitudinal_uncertainty, lateral_uncertainty, yaw_uncertainty) < 0.):
        raise ValueError('Invalid scan collision sweep limits')
    inside, swept = set(), set()
    minimum_body = minimum_sweep = float('inf')
    theta, distance = angular*horizon, linear*horizon
    margin = lateral_uncertainty+2*abs(distance)*sin(min(pi, yaw_uncertainty)/2)
    straight = abs(theta) < 1.e-6
    if straight:
        end_x, end_y = distance, 0.
        reserve = abs(distance*theta)*.5
    else:
        turn_radius = linear/angular
        end_x, end_y = turn_radius*sin(theta), turn_radius*(1.-cos(theta))
        reserve = 0.
        initial_angle = -pi/2 if turn_radius > 0. else pi/2

    def on_arc(x, y):
        advance = ((atan2(y-turn_radius, x)-initial_angle) if angular > 0.
                   else (initial_angle-atan2(y-turn_radius, x))) % (2*pi)
        return abs(theta) >= 2*pi or advance <= abs(theta)+1.e-12

    def point_arc_distance(x, y):
        result = min(hypot(x, y), hypot(x-end_x, y-end_y))
        if on_arc(x, y):
            result = min(result, abs(hypot(x, y-turn_radius)-abs(turn_radius)))
        return result

    for index, x, y in points:
        if not all(isfinite(v) for v in (x, y)):
            raise ValueError('Obstacle points must be finite')
        lo, hi = x-longitudinal_uncertainty, x+longitudinal_uncertainty
        horizontal_gap = max(lo, 0., -hi)
        body_clearance = hypot(horizontal_gap, y)-radius-lateral_uncertainty
        minimum_body = min(minimum_body, body_clearance)
        if body_clearance <= 1.e-9:
            inside.add(index)
        if linear == 0.:
            path_distance = hypot(horizontal_gap, y)
        elif straight:
            gap = max(min(0., end_x)-hi, lo-max(0., end_x), 0.)
            path_distance = hypot(gap, y)-reserve
        else:
            # Distance between a horizontal uncertainty interval and the arc.
            # Its minimum occurs at an endpoint, a vertical arc extremum, or
            # an arc/interval intersection; these candidates are exhaustive.
            path_distance = min(point_arc_distance(lo, y), point_arc_distance(hi, y),
                hypot(horizontal_gap, y),
                hypot(max(lo-end_x, 0., end_x-hi), y-end_y))
            if abs(theta) >= pi:
                path_distance = min(path_distance, hypot(horizontal_gap, y-2*turn_radius))
            discriminant = turn_radius*turn_radius-(y-turn_radius)**2
            if discriminant >= 0.:
                root = sqrt(discriminant)
                if any(lo <= value <= hi and on_arc(value, y) for value in (-root, root)):
                    path_distance = 0.
        clearance = path_distance-radius-margin
        minimum_sweep = min(minimum_sweep, clearance)
        if clearance <= 1.e-9:
            swept.add(index)
    return SweptBodyDecision(frozenset(inside), frozenset(swept),
                             minimum_body, minimum_sweep)
