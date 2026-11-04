"""Exact obstacle/corner checks and curvature limits, independent of ROS runtime."""
from math import hypot, pi
from pathlib import Path

import pytest

from smart_vision_amr.grid import Grid
from smart_vision_amr.path_geometry import (collision_free, curvature_speed_limit,
    path_length, resample_path, route_cells, segment_cells, smooth_grid_path,
    upcoming_curvature, path_traversal_cost, anticipatory_curve_speed_limit)


def test_supercover_catches_diagonal_corner_and_boundary_contact():
    assert {(0, 0), (1, 0), (0, 1), (1, 1)} <= segment_cells((.5, .5), (1.5, 1.5))
    assert not collision_free([(.5, .5), (1.5, 1.5)], lambda cell: cell == (1, 0))
    assert not collision_free([(1., .5), (1., 3.5)], lambda cell: cell == (0, 2))


def test_curve_braking_ceiling_falls_before_turn_then_releases_after_it():
    route = resample_path([(0., 0.), (1., 0.), (1., 2.)], .05)
    def limit(progress):
        return anticipatory_curve_speed_limit(route, progress, 1., .22, .35, .10)
    limits = [limit(progress) for progress in (.1, .65, .7, .75, .8, .85, .9)]
    assert limits[0] == pytest.approx(.22)
    assert all(a >= b for a, b in zip(limits, limits[1:]))
    assert len(set(round(v, 4) for v in limits)) >= 4  # no max→bend step
    assert limits[-1] == pytest.approx(.12)
    assert limit(1.25) == pytest.approx(.22)
    # The envelope satisfies the selected deceleration before the bend margin.
    for s in (.65, .7, .75, .8):
        assert limit(s)**2 <= .12**2+2*.10*max(0., 1.-s-.15)+1e-9


def test_curve_braking_keeps_straight_speed_and_holds_corner_cap_at_vertex():
    straight = [(0., 0.), (1., 0.), (2., 0.)]
    assert anticipatory_curve_speed_limit(straight, .5, 1., .22, .35, .10) == .22
    corner = resample_path([(0., 0.), (1., 0.), (1., 2.)], .05)
    assert anticipatory_curve_speed_limit(corner, 1., 1., .22, .35, .10) == .12


@pytest.mark.parametrize('deceleration', [0., -1., float('nan')])
def test_curve_braking_rejects_invalid_deceleration(deceleration):
    with pytest.raises(ValueError):
        anticipatory_curve_speed_limit([(0., 0.), (1., 0.)], 0., 1., .22,
                                        .35, .10, deceleration=deceleration)


def test_supercover_catches_tiny_intersection_missed_by_fixed_sampling():
    a, b = (.5, .5), (100.5, 99.5)
    # Close to the final corner, a very short part of the segment crosses this tile.
    assert (99, 99) in segment_cells(a, b)
    assert (0, 0) in segment_cells(a, b)
    assert segment_cells(a, b) == segment_cells(b, a)


def test_empty_map_corners_are_rounded_and_dense_without_moving_endpoints():
    grid = Grid.from_ascii('\n'.join(['.'*20]*20))
    cells = [(2, 2), (3, 2), (4, 2), (5, 2), (5, 3), (5, 4), (5, 5)]
    result, changed = smooth_grid_path(cells, grid.is_static_obstacle,
                                      spacing=.3, shortcut_distance=3.)
    assert changed
    assert result[0] == (2.5, 2.5) and result[-1] == (5.5, 5.5)
    assert collision_free(result, grid.is_static_obstacle)
    assert all(hypot(b[0]-a[0], b[1]-a[1]) <= .300001 for a, b in zip(result, result[1:]))
    assert path_length(result) < 6.


def test_concave_inflated_obstacle_prevents_unsafe_corner_rounding():
    grid = Grid.from_ascii('''
        ........
        ...###..
        ...###..
        ...###..
        ........
        ........
    ''')
    cells = [(1, 2), (1, 3), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (6, 3)]
    result, _ = smooth_grid_path(cells, grid.is_static_obstacle,
                                 spacing=.2, shortcut_distance=10.)
    assert collision_free(result, grid.is_static_obstacle)
    assert all(not grid.is_static_obstacle(cell) for cell in route_cells(result))


def test_classroom_route_keeps_safe_smoothing_when_one_corner_is_too_tight():
    """The 5 cm physical-test route used to lose every safe shortcut at one wall."""
    image = Path(__file__).parents[1] / 'maps' / 'classroom_20261030_cleaned.pgm'
    magic, dimensions, maximum, pixels = image.read_bytes().split(b'\n', 3)
    assert (magic, dimensions, maximum) == (b'P5', b'86 140', b'255')
    width, height = 86, 140
    occupied = {(x, y) for y in range(height) for x in range(width)
                if pixels[(height - y - 1) * width + x] <= 89 or
                pixels[(height - y - 1) * width + x] == 205}
    inflated = {(x + dx, y + dy) for x, y in occupied
                for dx in range(-4, 5) for dy in range(-4, 5)}
    blocked = lambda cell: (cell in inflated or not 0 <= cell[0] < width
                            or not 0 <= cell[1] < height)
    cells = [(30, 101)]
    for dx, dy, count in ((0, -1, 3), (1, -1, 3), (0, -1, 4),
                          (1, -1, 10), (0, -1, 28), (-1, 0, 10),
                          (-1, 1, 4), (0, 1, 6), (-1, 1, 2)):
        for _ in range(count):
            x, y = cells[-1]
            cells.append((x + dx, y + dy))
    original = [(x + .5, y + .5) for x, y in cells]
    assert len(cells) == 71 and collision_free(original, blocked)

    result, changed = smooth_grid_path(cells, blocked,
                                        spacing=1., shortcut_distance=16.,
                                        iterations=2)
    assert changed
    assert result[0] == original[0] and result[-1] == original[-1]
    assert collision_free(result, blocked)
    assert path_length(result) < path_length(original) - 1.0  # >5 cm shorter
    assert any((x % 1, y % 1) != (.5, .5) for x, y in result[1:-1])
    assert (43.5, 80.5) in result  # wall-side bend stays sharp
    assert (37.5, 87.5) not in result  # open bend is rounded


def test_live_obstacle_and_unknown_cells_are_identical_hard_constraints():
    cells = [(1, 1), (1, 2), (1, 3), (2, 3), (3, 3), (4, 3), (4, 2), (4, 1)]
    blocked = {(2, 2), (3, 2)}
    result, _ = smooth_grid_path(cells, lambda cell: cell in blocked,
                                 spacing=.2, shortcut_distance=10.)
    assert not route_cells(result) & blocked


def test_start_in_inflation_retains_original_certified_egress():
    cells = [(1, 1), (2, 1), (3, 1)]
    result, changed = smooth_grid_path(cells, lambda cell: cell == (1, 1))
    assert not changed
    assert result == [(1.5, 1.5), (2.5, 1.5), (3.5, 1.5)]


def test_resampling_keeps_corners_and_removes_duplicate_points():
    assert resample_path([(0., 0.), (0., 0.), (1., 0.), (1., 1.)], .25)[4] == (1., 0.)
    with pytest.raises(ValueError):
        resample_path([(0., 0.)], 0)


def test_cost_integral_is_exact_and_independent_of_resampling():
    path = [(.5, 1.5), (3.5, 1.5)]
    penalties = {(1, 1): 2., (2, 1): 1.}
    assert path_traversal_cost(path, penalties) == pytest.approx(6.)
    assert path_traversal_cost(resample_path(path, .13), penalties) == pytest.approx(6.)
    # A zero-length corner contact adds no charge from the other two tiles.
    assert path_traversal_cost([(.5, .5), (1.5, 1.5)],
                               {(1, 0): 100., (0, 1): 100.}) == pytest.approx(2**.5)
    assert path_traversal_cost([(1., .5), (1., 2.5)],
                               {(0, 1): 2.}) == pytest.approx(4.)


def test_smoothing_retains_clearance_aware_detour_in_open_space():
    cells = [(1, 1), (2, 2), (3, 3), (4, 3), (5, 3), (6, 2), (7, 1)]
    penalties = {(x, 1): 3. for x in range(2, 7)}
    original = [(x+.5, y+.5) for x, y in cells]
    unsafe_preference, _ = smooth_grid_path(cells, lambda _: False,
                                             shortcut_distance=10.)
    result, changed = smooth_grid_path(cells, lambda _: False, spacing=.2,
                                        shortcut_distance=10., cell_penalty=penalties)
    assert changed
    assert path_traversal_cost(unsafe_preference, penalties) > path_traversal_cost(original, penalties)
    assert path_traversal_cost(result, penalties) <= path_traversal_cost(original, penalties) + 1e-8
    assert max(y for _, y in result) > 2.
    assert result[0] == original[0] and result[-1] == original[-1]
    assert path_length(result) < path_length(original)


def test_curvature_preview_and_speed_bounds_release_after_corner():
    points = [(0., 0.), (1., 0.), (1., 1.), (1., 2.), (1., 3.)]
    k = upcoming_curvature(points, 0., 1.5)
    assert k > 1.
    limit = curvature_speed_limit(k, .25, .20, .05)
    assert limit*k <= .20 + 1e-12
    assert limit*limit*k <= .05 + 1e-12
    assert upcoming_curvature(points, 3., 1.) == 0.
    assert curvature_speed_limit(0., .25, .20, .05) == .25


def test_map_pose_progress_follows_arc_length_not_euclidean_goal_distance():
    from smart_vision_amr.path_geometry import path_progress
    points = [(0., 0.), (2., 0.), (2., 2.), (0., 2.)]
    assert path_progress(points, (1., .1)) == 1.
    assert path_progress(points, (2.1, 1.)) == 3.
    assert path_progress(points, (1., 2.1)) == 5.
