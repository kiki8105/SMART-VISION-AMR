"""Dynamic escape exceptions use the robot footprint; no ROS node is started."""

from copy import deepcopy
from math import hypot, sqrt

import pytest

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.astar_planner import AStarPlannerNode
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells
from test_planner_failure_memory import prepared
from test_planner_obstacle_hold import ready


def free_grid(size=30):
    return Grid(tuple(tuple(False for _ in range(size)) for _ in range(size)))


def escape(start, observed=(), remembered=(), *, raw_grid=None):
    grid = free_grid()
    raw_grid = grid if raw_grid is None else raw_grid
    observed, remembered = set(observed), set(remembered)
    blocked = inflate_cells(observed | remembered, grid.width, grid.height, 5)
    allowed = AStarPlannerNode._dynamic_escape_halo(
        start, blocked, observed, remembered, grid, raw_grid, 5,
        4.4, .4 + sqrt(2.) / 2.)
    return grid, blocked, allowed


@pytest.mark.parametrize('kind', ['observed', 'remembered'])
def test_inflation_exception_cannot_leave_a_footprint_overlapping_raw_obstacle(kind):
    _, _, allowed = escape((13, 15), **{kind: {(15, 15)}})
    assert allowed == set()


def test_physically_clear_dynamic_halo_can_leave_without_relaxing_raw_obstacle():
    start, goal, obstacle = (15, 14), (23, 14), (10, 10)
    grid, blocked, allowed = escape(start, {obstacle})
    assert start in allowed
    assert obstacle not in allowed
    path = AStarPlanner().plan(grid, start, goal, blocked,
                              allow_blocked_start=True, escape_cells=allowed)
    assert path[0] == start
    assert path[-1] == goal
    assert obstacle not in path
    assert (15, 10) not in allowed


def test_diagonal_escape_segments_keep_the_swept_footprint_clear():
    start, goal, obstacle = (15, 14), (23, 22), (10, 10)
    grid, blocked, allowed = escape(start, {obstacle})
    path = AStarPlanner().plan(grid, start, goal, blocked,
                              allow_blocked_start=True, escape_cells=allowed)
    assert path
    assert any(a[0] != b[0] and a[1] != b[1] for a, b in zip(path, path[1:]))
    # Check the continuous segments, including the transition out of the halo,
    # against the raw occupied square and the physical radius plus margin.
    for a, b in zip(path, path[1:]):
        for step in range(101):
            x = a[0] + (b[0] - a[0]) * step / 100.
            y = a[1] + (b[1] - a[1]) * step / 100.
            distance = hypot(max(abs(x-obstacle[0])-.5, 0.),
                             max(abs(y-obstacle[1])-.5, 0.))
            assert distance >= 4.8


def test_escape_footprint_cannot_overhang_map_boundary():
    _, _, allowed = escape((1, 14), {(5, 10)})
    assert allowed == set()


def test_escape_footprint_checks_raw_static_map_even_outside_dynamic_hit_cell():
    rows = [list(row) for row in free_grid().rows]
    rows[14][14] = True
    raw_grid = Grid(tuple(tuple(row) for row in rows))
    _, _, allowed = escape((15, 14), {(10, 10)}, raw_grid=raw_grid)
    assert allowed == set()


def test_escape_footprint_checks_unknown_map_cells_as_raw_obstacles():
    # Grid stores unknown occupancy as blocked, just like occupied cells.
    rows = [list(row) for row in free_grid().rows]
    rows[17][15] = True
    raw_grid = Grid(tuple(tuple(row) for row in rows))
    _, _, allowed = escape((15, 14), {(10, 10)}, raw_grid=raw_grid)
    assert allowed == set()


@pytest.mark.parametrize('clearance', [float('nan'), float('inf'), -1.])
def test_invalid_clearance_never_authorizes_escape(clearance):
    grid = free_grid()
    allowed = AStarPlannerNode._dynamic_escape_halo(
        (15, 14), {(15, 14)}, set(), set(), grid, grid, 5, clearance, 0.)
    assert allowed == set()


@pytest.mark.parametrize('layer', ['observed', 'remembered'])
def test_classified_near_wall_blocker_can_enter_hold_after_three_attempts(prepared, layer):
    node = prepared
    changed = deepcopy(node.map[0])
    changed.data[14*changed.info.width+10] = 100
    node.on_map(changed)
    node.params['mapped_wall_return_tolerance_cells'] = 2
    setattr(node, layer, {(10, 15)})  # Known-free cell beside the mapped wall.
    goal = node.last_goal
    for index in range(3):
        ready(node, 101.+index*.1)
        result = node._register_obstacle_failure(.55, 1.55)
        assert result is (index == 2)
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 3
    assert set(node.obstacle_hold.blocked_cells) == {(10, 15)}
    assert node.last_goal is goal
    assert not node.planned_cells
