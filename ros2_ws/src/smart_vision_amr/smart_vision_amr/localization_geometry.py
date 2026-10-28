"""Conservative in-place sweep clearance and independent scan/map agreement."""
import math

import cv2
import numpy as np


def scan_points(message, laser_pose):
    """Return finite hit endpoints in base frame, respecting the real sensor offset."""
    ranges = np.asarray(message.ranges, dtype=float)
    angles = message.angle_min + np.arange(len(ranges))*message.angle_increment
    valid = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
    angles = angles[valid] + laser_pose[2]
    return np.column_stack((laser_pose[0]+ranges[valid]*np.cos(angles),
                            laser_pose[1]+ranges[valid]*np.sin(angles)))


def rotation_clearance(message, laser_pose, radius=.30):
    """No translation: all observed obstacle endpoints must clear the sweep circle.

    Positive infinity means no return within the sensor range. A few isolated
    zero/NaN LDS dropouts may be bounded by nearby finite hits: no adjacent gaps,
    at most 2% of beams, and at most two degrees between the bounding beams.
    Such inferred clearance needs an extra 8 cm after the laser-to-base offset
    and angular uncertainty are applied. Larger or unbounded gaps stay unsafe;
    neither the physical sweep radius nor valid close returns are relaxed.
    """
    values = np.asarray(message.ranges, dtype=float)
    if (values.ndim != 1 or len(values) < 30
            or not math.isfinite(radius) or radius <= 0.
            or len(laser_pose) != 3 or not all(math.isfinite(value) for value in laser_pose)
            or not math.isfinite(message.angle_min)
            or not math.isfinite(message.angle_increment)
            or abs(message.angle_increment)*(len(values)-1) < 6.0
            or not math.isfinite(message.range_min) or message.range_min < 0.
            or not math.isfinite(message.range_max) or message.range_max <= radius
            or message.range_max <= message.range_min):
        return False, None

    finite_hits = (np.isfinite(values) & (values >= max(.001, message.range_min))
                   & (values <= message.range_max))
    dropouts = np.isnan(values) | (values == 0.)
    # Negative infinity, negative ranges, positive sub-minimum hits and invalid
    # over-range finite values must never be converted into free space.
    if not np.all(finite_hits | np.isposinf(values) | dropouts):
        return False, None

    hit_angles = (message.angle_min+np.flatnonzero(finite_hits)*message.angle_increment
                  + laser_pose[2])
    points = np.column_stack((laser_pose[0]+values[finite_hits]*np.cos(hit_angles),
                              laser_pose[1]+values[finite_hits]*np.sin(hit_angles)))
    if len(points) < 8:
        return False, None
    distance = float(np.min(np.linalg.norm(points, axis=1)))

    if np.any(dropouts):
        step = abs(message.angle_increment)
        if np.count_nonzero(dropouts)/len(values) > .02 or 2.*step > math.radians(2.)+1e-12:
            return False, None
        for index in np.flatnonzero(dropouts):
            before, after = (index-1) % len(values), (index+1) % len(values)
            if not finite_hits[before] or not finite_hits[after]:
                return False, None
            neighbor_ranges = values[[before, after]]
            neighbor_angles = (message.angle_min
                + np.array([before, after])*message.angle_increment + laser_pose[2])
            neighbors = np.column_stack((laser_pose[0]+neighbor_ranges*np.cos(neighbor_angles),
                                         laser_pose[1]+neighbor_ranges*np.sin(neighbor_angles)))
            neighbor_distance = float(np.min(np.linalg.norm(neighbors, axis=1)))
            bounded_range = float(np.min(neighbor_ranges))
            angle = message.angle_min+int(index)*message.angle_increment+laser_pose[2]
            bounded_distance = math.hypot(laser_pose[0]+bounded_range*math.cos(angle),
                                          laser_pose[1]+bounded_range*math.sin(angle))
            # Moving an endpoint by one beam angle can reduce its body-frame
            # distance by at most this chord length, regardless of laser offset.
            bounded_distance -= 2.*bounded_range*math.sin(step*.5)
            clearance = min(neighbor_distance, bounded_distance)
            if clearance < radius+.08:
                return False, clearance
            distance = min(distance, clearance)
    return distance >= radius, distance


def occupied_distance(data, resolution):
    data = np.asarray(data)
    distance = cv2.distanceTransform((data < 65).astype(np.uint8),
                                     cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*resolution
    distance[data < 0] = np.inf
    return distance


def scan_map_agreement(points, pose, metadata, distance, tolerance=.15):
    """Fraction of static-wall hits with an unobstructed mapped line of sight.

    Matching endpoints behind an intervening wall are inconsistent, even when
    AMCL covariance is small. Unknown/outside cells also cannot validate a ray.
    Rays start at the base center: skip its first 25 cm (including the 6.4 cm
    Waffle Pi sensor offset) and the last 20 cm near the measured wall surface.
    This geometric check rejects aliases, but cannot distinguish perfectly
    symmetric rooms without another observation or operator confirmation.
    """
    if len(points) < 12:
        return 0.
    points = np.asarray(points)[np.linspace(0, len(points)-1, min(160, len(points)), dtype=int)]
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    world = points @ np.array([[c, s], [-s, c]]) + [pose['x'], pose['y']]
    origin = metadata['origin']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    rotation = np.array([[c, -s], [s, c]])
    local = (world-[origin['x'], origin['y']]) @ rotation
    robot = (np.array([pose['x'], pose['y']])-[origin['x'], origin['y']]) @ rotation
    cells = np.floor(local/metadata['resolution']).astype(int)
    valid = ((cells[:, 0] >= 0) & (cells[:, 0] < metadata['width'])
             & (cells[:, 1] >= 0) & (cells[:, 1] < metadata['height']))
    matched = np.zeros(len(points), dtype=bool)
    matched[valid] = distance[cells[valid, 1], cells[valid, 0]] <= tolerance
    for index in np.flatnonzero(matched):
        delta = local[index]-robot
        length = float(np.linalg.norm(delta))
        if length <= .45:
            continue
        # Half-cell spacing samples walls even when the ray crosses obliquely.
        count = max(2, int(math.ceil((length-.45)/(metadata['resolution']*.5)))+1)
        fractions = np.linspace(.25/length, 1.-.20/length, count)
        ray = np.floor((robot+fractions[:, None]*delta)/metadata['resolution']).astype(int)
        inside = ((ray[:, 0] >= 0) & (ray[:, 0] < metadata['width'])
                  & (ray[:, 1] >= 0) & (ray[:, 1] < metadata['height']))
        if not np.all(inside):
            matched[index] = False
            continue
        clearance = distance[ray[:, 1], ray[:, 0]]
        if not np.all(np.isfinite(clearance) & (clearance > 0.)):
            matched[index] = False
    return float(np.mean(matched))
