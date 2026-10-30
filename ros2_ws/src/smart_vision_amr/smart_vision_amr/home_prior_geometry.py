"""Bounded scan evidence for a confirmed registered Home, never a pose estimate.

A caller may use positive evidence to seed a *broad* AMCL prior at the registered
Home. No candidate pose is returned, and AMCL convergence/stop/safety checks must
still succeed independently. The caller owns observation freshness, map identity,
movement invalidation, and operator confirmation that the robot is really Home.
"""
import math

import numpy as np

from .localization_geometry import scan_map_agreement


_MAX_TRANSLATION = .30
_MAX_YAW = math.radians(15.)
_MIN_AGREEMENT = .65
_MIN_SCANS = 3
_MIN_CLUSTER = 3


def _result(reason, distinct_scans=0):
    return {'supported': False, 'reason': reason, 'distinct_scans': distinct_scans,
            'candidate_count': 0, 'footprint_candidates': 0,
            'shortlisted_candidates': 0, 'consistent_candidates': 0,
            'support_clusters': 0, 'best_min_agreement': None, 'checked_candidates': 0}


def _free_footprint(center, occupancy, resolution, radius):
    """Require every cell touched by the circular footprint to be known/free."""
    low = np.floor((center-radius)/resolution).astype(int)
    high = np.floor((center+radius)/resolution).astype(int)
    height, width = occupancy.shape
    if low[0] < 0 or low[1] < 0 or high[0] >= width or high[1] >= height:
        return False
    xs = np.arange(low[0], high[0]+1)
    ys = np.arange(low[1], high[1]+1)
    xx, yy = np.meshgrid(xs, ys)
    dx = np.maximum(np.maximum(xx*resolution-center[0],
                               center[0]-(xx+1)*resolution), 0.)
    dy = np.maximum(np.maximum(yy*resolution-center[1],
                               center[1]-(yy+1)*resolution), 0.)
    touched = dx*dx+dy*dy <= radius*radius+1e-12
    cells = occupancy[yy, xx]
    return bool(np.all((cells[touched] >= 0) & (cells[touched] < 65)))


def _endpoint_scores(points, candidates, metadata, distance):
    """Vectorized upper bound of the existing line-of-sight agreement score."""
    count = len(candidates)
    c = np.cos(candidates[:, 2])[:, None]
    s = np.sin(candidates[:, 2])[:, None]
    wx = c*points[None, :, 0]-s*points[None, :, 1]+candidates[:, 0, None]
    wy = s*points[None, :, 0]+c*points[None, :, 1]+candidates[:, 1, None]
    origin = metadata['origin']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    dx, dy = wx-origin['x'], wy-origin['y']
    # This is the same world-to-map rotation used by scan_map_agreement.
    ix = np.floor((c*dx+s*dy)/metadata['resolution']).astype(int)
    iy = np.floor((-s*dx+c*dy)/metadata['resolution']).astype(int)
    valid = ((ix >= 0) & (ix < metadata['width'])
             & (iy >= 0) & (iy < metadata['height']))
    matched = np.zeros((count, len(points)), dtype=bool)
    matched[valid] = distance[iy[valid], ix[valid]] <= .15
    return np.mean(matched, axis=1)


def _cluster_sizes(candidates, translation_step, yaw_step):
    """Connected local grid neighborhoods, common to every observed scan."""
    remaining = set(range(len(candidates)))
    sizes = []
    while remaining:
        queue = [remaining.pop()]
        size = 0
        while queue:
            index = queue.pop()
            size += 1
            if not remaining:
                continue
            indices = np.array(sorted(remaining))
            delta = candidates[indices]-candidates[index]
            neighbors = ((np.linalg.norm(delta[:, :2], axis=1)
                          <= translation_step*math.sqrt(2.)+1e-10)
                         & (np.abs(delta[:, 2]) <= yaw_step+1e-10))
            for other in indices[neighbors]:
                remaining.remove(int(other))
                queue.append(int(other))
        sizes.append(size)
    return sizes


def home_neighborhood_evidence(
        observations, home, metadata, distance, occupancy, *, footprint_radius=.22,
        translation_radius=.30, yaw_limit=math.radians(15.), translation_step=.05,
        yaw_step=math.radians(3.), min_agreement=.65, max_shortlist=96):
    """Return bounded repeated evidence, without selecting an actual robot pose.

    ``observations`` contains ``(stamp_ns, base_frame_hit_points_Nx2)`` pairs.
    Three distinct positive stamps are required. A common candidate must pass the
    unchanged scan/map line-of-sight check in *every* distinct observation, and at
    least three adjacent common candidates must form a local cluster. Endpoint
    matching only shortlists candidates; it cannot itself validate a prior.

    Translation is a circular radius around registered Home (hard cap 0.30 m),
    yaw is within Home +/- 15 degrees. This deliberately does not search the map
    globally, report a best pose, alter Home, or establish localization confidence.
    Multiple nearby clusters may fit: all are within the same broad Home prior;
    ``support_clusters`` is diagnostic, not proof of a unique position.
    Evaluation returns as soon as one repeated cluster is fully verified; counts
    describe checked evidence, not all possible poses in the neighborhood.
    """
    parameters = (footprint_radius, translation_radius, yaw_limit,
                  translation_step, yaw_step, min_agreement)
    if not all(math.isfinite(value) for value in parameters):
        raise ValueError('Home evidence bounds must be finite')
    if (footprint_radius < .22 or not 0. < translation_radius <= _MAX_TRANSLATION
            or not 0. < yaw_limit <= _MAX_YAW
            or not .025 <= translation_step <= translation_radius
            or not math.radians(1.) <= yaw_step <= yaw_limit
            or not _MIN_AGREEMENT <= min_agreement <= 1.
            or isinstance(max_shortlist, bool) or not isinstance(max_shortlist, int)
            or not 3 <= max_shortlist <= 256):
        raise ValueError('Home evidence cannot exceed the search bounds or weaken safety limits')
    result = _result('insufficient_distinct_scans')
    unique = {}
    try:
        for stamp, points in observations:
            if isinstance(stamp, bool) or not isinstance(stamp, (int, np.integer)) or stamp <= 0:
                return _result('invalid_scan_identity', len(unique))
            if int(stamp) in unique:
                continue
            points = np.asarray(points, dtype=float)
            if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 12
                    or not np.all(np.isfinite(points))):
                return _result('invalid_scan_geometry', len(unique))
            unique[int(stamp)] = points[np.linspace(
                0, len(points)-1, min(160, len(points)), dtype=int)]
    except (TypeError, ValueError):
        return _result('invalid_scan_geometry', len(unique))
    result['distinct_scans'] = len(unique)
    if len(unique) < _MIN_SCANS:
        return result
    try:
        home_values = np.array([home['x'], home['y'], home['yaw']], dtype=float)
        origin = metadata['origin']
        origin_values = np.array([origin['x'], origin['y'], origin['yaw']], dtype=float)
        resolution = float(metadata['resolution'])
        shape = (metadata['height'], metadata['width'])
        occupancy = np.asarray(occupancy)
        distance = np.asarray(distance, dtype=float)
        if (not np.all(np.isfinite(home_values)) or not np.all(np.isfinite(origin_values))
                or not math.isfinite(resolution) or resolution <= 0.
                or occupancy.ndim != 2 or occupancy.shape != shape or distance.shape != shape
                or not np.all(np.isfinite(occupancy))
                or np.any((occupancy < -1) | (occupancy > 100))
                or np.any(np.isnan(distance)) or np.any(distance < 0.)):
            return _result('invalid_map_geometry', len(unique))
    except (TypeError, ValueError, KeyError):
        return _result('invalid_map_geometry', len(unique))

    half = int(math.floor(translation_radius/translation_step+1e-9))
    offsets = np.arange(-half, half+1)*translation_step
    xy = np.array([(x, y) for x in offsets for y in offsets
                   if x*x+y*y <= translation_radius*translation_radius+1e-12])
    xy += home_values[:2]
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    local = (xy-origin_values[:2]) @ np.array([[c, -s], [s, c]])
    free = np.array([_free_footprint(center, occupancy, resolution, footprint_radius)
                     for center in local], dtype=bool)
    yaw_half = int(math.floor(yaw_limit/yaw_step+1e-9))
    yaws = home_values[2]+np.arange(-yaw_half, yaw_half+1)*yaw_step
    result['candidate_count'] = len(xy)*len(yaws)
    result['footprint_candidates'] = int(np.count_nonzero(free))*len(yaws)
    if not np.any(free):
        result['reason'] = 'no_free_footprint_near_home'
        return result
    candidates = np.array([(x, y, yaw) for x, y in xy[free] for yaw in yaws])
    endpoint_min = np.ones(len(candidates))
    for points in unique.values():
        endpoint_min = np.minimum(endpoint_min, _endpoint_scores(
            points, candidates, metadata, distance))
    indices = np.flatnonzero(endpoint_min >= min_agreement)
    if not len(indices):
        result['reason'] = 'no_repeated_endpoint_match'
        return result
    # Bounds memory and line-of-sight work; ties retain deterministic grid order.
    rank = np.argsort(-endpoint_min[indices], kind='stable')[:max_shortlist]
    candidates = candidates[indices[rank]]
    result['shortlisted_candidates'] = len(candidates)
    consistent = []
    best = 0.
    for candidate in candidates:
        result['checked_candidates'] += 1
        pose = dict(zip(('x', 'y', 'yaw'), map(float, candidate)))
        scores = [scan_map_agreement(points, pose, metadata, distance)
                  for points in unique.values()]
        minimum = min(scores)
        best = max(best, minimum)
        if minimum >= min_agreement:
            consistent.append(candidate)
            if len(consistent) >= _MIN_CLUSTER:
                sizes = _cluster_sizes(np.asarray(consistent), translation_step, yaw_step)
                if any(size >= _MIN_CLUSTER for size in sizes):
                    result.update(supported=True, reason='home_neighborhood_supported',
                                  best_min_agreement=best,
                                  consistent_candidates=len(consistent),
                                  support_clusters=sum(size >= _MIN_CLUSTER for size in sizes))
                    return result
    result['best_min_agreement'] = best
    result['consistent_candidates'] = len(consistent)
    if not consistent:
        result['reason'] = 'no_repeated_line_of_sight_match'
        return result
    sizes = _cluster_sizes(np.asarray(consistent), translation_step, yaw_step)
    result['support_clusters'] = sum(size >= _MIN_CLUSTER for size in sizes)
    result['supported'] = result['support_clusters'] > 0
    result['reason'] = ('home_neighborhood_supported' if result['supported']
                        else 'no_consistent_candidate_cluster')
    return result
