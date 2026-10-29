"""Small ROS-independent helpers for comparing navigation path geometry."""

from __future__ import annotations

from math import ceil, floor, hypot, isfinite, sqrt
from typing import Callable, Sequence


Point = tuple[float, float]


def segment_cells(start: Point, end: Point) -> set[tuple[int, int]]:
    """Cells touched by a continuous segment, including corner/edge contact.

    Coordinates use tile boundaries at integers, so cell (x, y)'s center is
    (x + .5, y + .5). Checking only Bresenham cell centers misses corner cuts.
    Grid-crossing events and their intervening intervals cover the exact line.
    """
    if not all(isfinite(v) for point in (start, end) for v in point):
        raise ValueError("segment coordinates must be finite")
    dx, dy = end[0] - start[0], end[1] - start[1]
    events = {0.0, 1.0}
    for origin, delta, finish in ((start[0], dx, end[0]), (start[1], dy, end[1])):
        if abs(delta) < 1e-12:
            continue
        for boundary in range(ceil(min(origin, finish)), floor(max(origin, finish)) + 1):
            event = (boundary - origin) / delta
            if 0.0 <= event <= 1.0:
                events.add(event)
    times = sorted(events)
    samples = times + [(a + b) * .5 for a, b in zip(times, times[1:])]
    touched = set()
    for t in samples:
        x, y = start[0] + t * dx, start[1] + t * dy
        xs, ys = {floor(x)}, {floor(y)}
        if abs(x - round(x)) < 1e-10:
            xs.update((round(x) - 1, round(x)))
        if abs(y - round(y)) < 1e-10:
            ys.update((round(y) - 1, round(y)))
        touched.update((cx, cy) for cx in xs for cy in ys)
    return touched


def route_cells(points: Sequence[Point]) -> set[tuple[int, int]]:
    if len(points) == 1:
        return segment_cells(points[0], points[0])
    return {cell for a, b in zip(points, points[1:]) for cell in segment_cells(a, b)}


def collision_free(points: Sequence[Point], blocked: Callable[[tuple[int, int]], bool]) -> bool:
    return bool(points) and not any(blocked(cell) for cell in route_cells(points))


def resample_path(points: Sequence[Point], spacing: float) -> list[Point]:
    """Densify every segment without moving endpoints or connecting corners."""
    if not points or not isfinite(spacing) or spacing <= 0:
        raise ValueError("a nonempty path and positive finite spacing are required")
    result = [points[0]]
    for a, b in zip(points, points[1:]):
        length = hypot(b[0] - a[0], b[1] - a[1])
        if length < 1e-12:
            continue
        count = max(1, ceil(length / spacing))
        result.extend((a[0] + (b[0] - a[0]) * index / count,
                       a[1] + (b[1] - a[1]) * index / count)
                      for index in range(1, count + 1))
    return result


def smooth_grid_path(cells: Sequence[tuple[int, int]],
                     blocked: Callable[[tuple[int, int]], bool], *,
                     spacing: float = 1.0, shortcut_distance: float = 12.0,
                     iterations: int = 2) -> tuple[list[Point], bool]:
    """Visibility shortcuts + Chaikin corner rounding, certified on inflated grid.

    All original cells, unknown cells, dynamic inflation, and map edges are
    hard blocks. An already certified start-egress path may begin in inflation;
    leave that original route untouched rather than weakening this checker.
    If any rounded segment is unsafe, return the original grid-center route.
    """
    original = [(x + .5, y + .5) for x, y in cells]
    if len(original) < 3 or not collision_free(original, blocked):
        return original, False
    if (not isfinite(shortcut_distance) or shortcut_distance <= 0
            or not isfinite(spacing) or spacing <= 0):
        return original, False
    simplified = [original[0]]
    index = 0
    while index < len(original) - 1:
        candidate = index + 1
        for end in range(index + 2, len(original)):
            if hypot(original[end][0] - original[index][0],
                     original[end][1] - original[index][1]) > shortcut_distance:
                break
            if not any(blocked(cell) for cell in segment_cells(original[index], original[end])):
                candidate = end
        simplified.append(original[candidate])
        index = candidate
    candidate = simplified
    for _ in range(max(0, min(4, int(iterations)))):
        rounded = [candidate[0]]
        for a, b in zip(candidate, candidate[1:]):
            rounded.extend(((.75*a[0] + .25*b[0], .75*a[1] + .25*b[1]),
                            (.25*a[0] + .75*b[0], .25*a[1] + .75*b[1])))
        rounded.append(candidate[-1])
        if not collision_free(rounded, blocked):
            return original, False
        candidate = rounded
    result = resample_path(candidate, spacing)
    if not collision_free(result, blocked):
        return original, False
    return result, result != original


def path_length(points: Sequence[Point]) -> float:
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def path_progress(points: Sequence[Point], position: Point) -> float:
    """Arc length at the nearest projection, preferring earlier ties at crossings."""
    if not all(isfinite(value) for value in position):
        raise ValueError('position must be finite')
    best = (float('inf'), 0.)
    traveled = 0.
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        squared = dx*dx+dy*dy
        length = sqrt(squared)
        t = 0. if squared < 1e-12 else max(0., min(1., ((position[0]-a[0])*dx+(position[1]-a[1])*dy)/squared))
        distance_squared = (position[0]-a[0]-t*dx)**2+(position[1]-a[1]-t*dy)**2
        candidate = (distance_squared, traveled+t*length)
        if candidate < best:
            best = candidate
        traveled += length
    return best[1]


def point_at_progress(points: Sequence[Point], distance: float) -> Point:
    """Interpolate a look-ahead target by distance along the polyline."""
    if not points:
        raise ValueError('a nonempty path is required')
    remaining = max(0., distance)
    for a, b in zip(points, points[1:]):
        length = hypot(b[0]-a[0], b[1]-a[1])
        if length > 1e-12 and remaining <= length:
            t = remaining/length
            return a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
        remaining -= length
    return points[-1]


def upcoming_curvature(points: Sequence[Point], progress: float, horizon: float,
                       minimum_chord: float = .15) -> float:
    """Largest circumcircle curvature in the upcoming arc-length interval."""
    if len(points) < 3:
        return 0.0
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + hypot(b[0]-a[0], b[1]-a[1]))
    largest = 0.0
    # Look one segment behind the estimated progress, since distance feedback
    # is discrete and should not release a corner limit before the turn ends.
    for index in range(1, len(points)-1):
        if distances[index+1] < progress or distances[index-1] > progress + horizon:
            continue
        before, after = index-1, index+1
        while before > 0 and distances[index]-distances[before] < minimum_chord:
            before -= 1
        while after < len(points)-1 and distances[after]-distances[index] < minimum_chord:
            after += 1
        a, b, c = points[before], points[index], points[after]
        ab, bc, ac = hypot(b[0]-a[0], b[1]-a[1]), hypot(c[0]-b[0], c[1]-b[1]), hypot(c[0]-a[0], c[1]-a[1])
        denominator = ab * bc * ac
        if denominator <= 1e-12:
            continue
        cross = abs((b[0]-a[0]) * (c[1]-a[1]) - (b[1]-a[1]) * (c[0]-a[0]))
        largest = max(largest, 2.0 * cross / denominator)
    return largest


def curvature_speed_limit(curvature: float, max_speed: float,
                          max_angular_speed: float, max_lateral_acceleration: float) -> float:
    """Upper bound from angular velocity and lateral acceleration (v² κ)."""
    if not all(isfinite(v) and v > 0 for v in (max_speed, max_angular_speed, max_lateral_acceleration)):
        raise ValueError("speed/acceleration limits must be finite and positive")
    if not isfinite(curvature) or curvature < 0:
        raise ValueError("curvature must be finite and nonnegative")
    if curvature < 1e-9:
        return max_speed
    return min(max_speed, max_angular_speed / curvature,
               sqrt(max_lateral_acceleration / curvature))


def paths_are_equivalent(reference: Sequence[Point], candidate: Sequence[Point],
                         tolerance: float = 0.20, start_tolerance: float = 0.35) -> bool:
    """Return whether a new path follows the same corridor after robot progress.

    The new path may start farther along the old path. Compare its geometry with
    the corresponding remaining suffix, so advancing the start pose alone does
    not look like a route change. A sufficiently large deviation still replaces
    the active controller goal.
    """
    if not reference or not candidate:
        return False

    start = candidate[0]
    nearest = min(range(len(reference)),
                  key=lambda i: hypot(reference[i][0] - start[0], reference[i][1] - start[1]))
    if hypot(reference[nearest][0] - start[0], reference[nearest][1] - start[1]) > start_tolerance:
        return False
    suffix = reference[nearest:]

    def directed_distance(source: Sequence[Point], target: Sequence[Point]) -> float:
        return max(min(hypot(x - tx, y - ty) for tx, ty in target) for x, y in source)

    deviation = max(directed_distance(suffix, candidate), directed_distance(candidate, suffix))
    return deviation <= tolerance
