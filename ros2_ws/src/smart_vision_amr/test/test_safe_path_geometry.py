"""Exact obstacle/corner checks and curvature limits, independent of ROS runtime."""
from math import hypot, pi

import pytest

from smart_vision_amr.grid import Grid
from smart_vision_amr.path_geometry import (collision_free, curvature_speed_limit,
    path_length, resample_path, route_cells, segment_cells, smooth_grid_path,
    upcoming_curvature)


def test_supercover_catches_diagonal_corner_and_boundary_contact():
    assert {(0, 0), (1, 0), (0, 1), (1, 1)} <= segment_cells((.5, .5), (1.5, 1.5))
    assert not collision_free([(.5, .5), (1.5, 1.5)], lambda cell: cell == (1, 0))
    assert not collision_free([(1., .5), (1., 3.5)], lambda cell: cell == (0, 2))


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
