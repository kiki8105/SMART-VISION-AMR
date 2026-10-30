"""Conservative LiDAR wall geometry selected by the observed marker's bearing.

All coordinates are in metres/radians. Camera x/y/yaw is an explicit calibrated
input; a camera role does not imply a mounting direction. A calibrated FOV is
required to select a marker ray away from the strict image-center tolerance.
No ROS, motor commands, localization estimate, or arrival decision lives here.
"""

import math
from numbers import Real

import numpy as np


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _line(points):
    center = points.mean(axis=0)
    covariance = (points-center).T @ (points-center)/len(points)
    values, vectors = np.linalg.eigh(covariance)
    normal = vectors[:, 0]
    if np.dot(normal, center) < 0:
        normal = -normal
    residuals = np.abs((points-center) @ normal)
    tangent = np.array([-normal[1], normal[0]])
    return center, normal, tangent, residuals


def estimate_marker_wall(scan, laser_pose, *, camera_pose, center_x, horizontal_fov=None,
                         cone_half_angle=math.radians(30), min_support=12,
                         min_span=.25, max_residual=.025, min_inlier_fraction=.85,
                         min_distance=.25, max_distance=3.5, max_axis_center_error=.05):
    """Fit the single wall covering a marker's ray, or return a failed proof.

    ``scan`` has LaserScan-compatible ranges/angle_min/angle_increment/range_min/
    range_max attributes. ``laser_pose`` and ``camera_pose`` are (x, y, yaw) in
    the same base frame. Pixel-right is negative camera bearing for an upright,
    non-mirrored image. Rotate/calibrate the image before calling this function.
    With no calibrated FOV, only an already centered marker (absolute error at
    most ``max_axis_center_error``) permits fitting across the camera axis.
    Off-center observations require a calibrated FOV to select a marker ray.
    The 30-degree fitting cone retains at least 25 cm of wall support near the
    minimum camera-to-wall distance, including a rear camera offset from LiDAR.

    The wall normal points from the base origin toward the surface. A positive
    ``heading_error_rad`` asks for positive robot yaw to make the camera axis
    perpendicular to the wall. ``tangent_yaw_base`` supports callers that
    deliberately need a direction parallel to the wall instead.

    Confidence is geometric fit/support quality, never a calibrated probability.
    One result is not a stable arrival proof; adapters must require fresh,
    consistent scans, marker centering and measured vehicle stop separately.
    """
    result = {'reliable': False, 'reason': 'invalid_inputs', 'normal_yaw_base': None,
              'tangent_yaw_base': None, 'heading_error_rad': None, 'distance_m': None,
              'residual_m': None, 'support_count': 0, 'span_m': 0.,
              'inlier_fraction': 0., 'confidence': 0., 'marker_bearing_base': None}
    try:
        poses = tuple(laser_pose)+tuple(camera_pose)
        limits = (cone_half_angle, min_span, max_residual,
                  min_inlier_fraction, min_distance, max_distance, max_axis_center_error)
        axis_only = horizontal_fov is None
        if (len(poses) != 6 or not all(_finite(value) for value in poses+limits)
                or not _finite(center_x) or not -1 <= center_x <= 1
                or not 0 < max_axis_center_error <= .1
                or (axis_only and abs(center_x) > max_axis_center_error)
                or (not axis_only and (not _finite(horizontal_fov)
                                      or not 0 < horizontal_fov < math.pi))
                or not 0 < cone_half_angle < math.pi/2
                or type(min_support) is not int or min_support < 6
                or min_span <= 0 or max_residual <= 0
                or not .5 < min_inlier_fraction <= 1
                or not 0 < min_distance < max_distance
                or not all(_finite(getattr(scan, name)) for name in
                           ('angle_min', 'angle_increment', 'range_min', 'range_max'))
                or scan.angle_increment == 0 or scan.range_min < 0
                or scan.range_max <= scan.range_min):
            return result
        lx, ly, lyaw = laser_pose
        cx, cy, cyaw = camera_pose
        bearing = _angle(cyaw if axis_only else
                         cyaw-math.atan(center_x*math.tan(horizontal_fov/2)))
        result['marker_bearing_base'] = bearing
        result['selection'] = 'centered_camera_axis' if axis_only else 'calibrated_marker_bearing'
        selected = []
        step = abs(scan.angle_increment)
        if step > cone_half_angle/3:
            result['reason'] = 'insufficient_angular_resolution'
            return result
        for index, distance in enumerate(scan.ranges):
            if not _finite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min+index*scan.angle_increment+lyaw
            point = np.array([lx+distance*math.cos(angle), ly+distance*math.sin(angle)])
            relative = _angle(math.atan2(point[1]-cy, point[0]-cx)-bearing)
            if abs(relative) <= cone_half_angle:
                selected.append((relative, index, point, distance))
        selected.sort(key=lambda record: record[0])
        if len(selected) < min_support:
            result['reason'] = 'insufficient_returns'
            return result
        # Missing rays and physical depth discontinuities delimit surfaces.
        # Wrap at +/-pi is harmless because records are sorted around bearing.
        groups = [[]]
        for record in selected:
            if groups[-1]:
                previous = groups[-1][-1]
                gap = record[0]-previous[0]
                discontinuity = np.linalg.norm(record[2]-previous[2])
                if gap > 2.5*step or discontinuity > .12+.04*min(record[3], previous[3]):
                    groups.append([])
            groups[-1].append(record)
        # A nearby side wall must never win because it has more scan points.
        central = [group for group in groups if len(group) >= min_support
                   and group[0][0] <= -2*step and group[-1][0] >= 2*step]
        if len(central) != 1:
            result['reason'] = 'no_single_surface_spans_marker_bearing'
            return result
        group = central[0]
        points = np.asarray([record[2] for record in group])
        if group[-1][0]-group[0][0] < max(8*step, math.radians(8)):
            result['reason'] = 'insufficient_angular_span'
            return result
        # Deterministic bounded RANSAC tolerates isolated noisy/outlier returns.
        indices = sorted(set(np.linspace(0, len(points)-1, min(14, len(points)), dtype=int)))
        best = None
        for offset, first in enumerate(indices):
            for second in indices[offset+1:]:
                direction = points[second]-points[first]
                length = float(np.linalg.norm(direction))
                if length < min_span/2:
                    continue
                normal = np.array([-direction[1], direction[0]])/length
                residuals = np.abs((points-points[first]) @ normal)
                mask = residuals <= 2*max_residual
                score = (int(mask.sum()), -float(np.median(residuals[mask])))
                if best is None or score > best[0]:
                    best = (score, mask)
        if best is None:
            result['reason'] = 'insufficient_surface_span'
            return result
        mask = best[1]
        fraction = float(mask.mean())
        result['inlier_fraction'] = fraction
        result['support_count'] = int(mask.sum())
        if fraction < min_inlier_fraction or mask.sum() < min_support:
            result['reason'] = 'mixed_surfaces_or_corner'
            return result
        inliers = points[mask]
        center, normal, tangent, residuals = _line(inliers)
        rms = float(np.sqrt(np.mean(residuals**2)))
        span = float(np.ptp(inliers @ tangent))
        result['residual_m'], result['span_m'] = rms, span
        if rms > max_residual or span < min_span:
            result['reason'] = 'nonplanar_or_short_surface'
            return result
        inlier_bearings = [group[index][0] for index in np.flatnonzero(mask)]
        if min(inlier_bearings) > -2*step or max(inlier_bearings) < 2*step:
            result['reason'] = 'fitted_wall_does_not_span_marker_bearing'
            return result
        # Ray/surface intersection must lie on the observed fitted segment.
        direction = np.array([math.cos(bearing), math.sin(bearing)])
        camera = np.array([cx, cy])
        denominator = float(np.dot(normal, direction))
        if denominator <= .35:
            result['reason'] = 'wall_observed_too_obliquely'
            return result
        ray_distance = float(np.dot(normal, center-camera))/denominator
        intersection = camera+ray_distance*direction
        along = float(np.dot(intersection, tangent))
        extent = inliers @ tangent
        distance = float(np.dot(normal, center))
        if (not min_distance <= ray_distance <= max_distance or distance <= 0
                or not float(extent.min())+.03 <= along <= float(extent.max())-.03):
            result['reason'] = 'wall_distance_or_bearing_intersection_invalid'
            return result
        normal_yaw = math.atan2(normal[1], normal[0])
        result.update(reliable=True, reason='single_marker_bearing_wall',
                      normal_yaw_base=normal_yaw,
                      tangent_yaw_base=_angle(normal_yaw+math.pi/2),
                      heading_error_rad=_angle(normal_yaw-cyaw), distance_m=distance,
                      confidence=min(1., fraction*max(0., 1-rms/(2*max_residual))
                                     *min(1., span/(2*min_span))))
        return result
    except (AttributeError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
        return result


def estimate_camera_axis_wall(scan, laser_pose, *, camera_pose, **limits):
    """Fit across an explicitly mounted camera axis after separate centering.

    The caller must first verify a fresh target detection within the strict
    image-center tolerance. No camera FOV is inferred or needed at the axis.
    """
    return estimate_marker_wall(scan, laser_pose, camera_pose=camera_pose,
                                center_x=0., horizontal_fov=None, **limits)
