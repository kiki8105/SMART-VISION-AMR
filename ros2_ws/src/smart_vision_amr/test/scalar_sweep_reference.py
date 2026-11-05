"""Frozen pre-vectorization raw-sweep oracle, independent of NumPy geometry."""
from math import isfinite
from smart_vision_amr.scan_outlier_guard import SweptBodyDecision

def scalar_swept_body_decision(points, linear, angular, *, radius=.22, horizon=.5,
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
