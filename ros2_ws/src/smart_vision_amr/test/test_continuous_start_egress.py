"""Replay the fixed-crate Home-return start cell without relaxing occupancy."""

from math import sqrt
from pathlib import Path

from PIL import Image
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.continuous_start_egress import certified_start_only_edges
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells_circular
from test_planner_failure_memory import prepared, refresh


MAP = Path(__file__).resolve().parents[1] / "maps/classroom_20261030_cleaned.pgm"
CAPTURED = (
    # Composed from the nearest map->odom and odom->base_footprint TF samples
    # in the 2026-11-02 stress-clean-sector2-1525-1 safety timeline.
    ((41, 83), (41.598942, 83.583139), {
        (41, 89), (37, 90), (38, 90), (39, 90), (40, 90), (41, 90),
        (38, 91), (39, 91), (41, 91), (41, 93),
    }),
    ((38, 83), (38.880154, 83.293440), {
        (38, 89), (39, 89), (40, 89), (37, 90), (38, 90),
        (39, 90), (40, 90), (41, 90), (47, 94), (47, 101), (47, 102),
    }),
)


def classroom_grids():
    image = Image.open(MAP).convert("L")
    width, height = image.size
    # In the saved trinary PGM, 205 is unknown, 0 occupied, 254 free.
    raw = Grid(tuple(tuple(image.getpixel((x, height - y - 1)) != 254
                           for x in range(width)) for y in range(height)))
    occupied = {(x, y) for y in range(height) for x in range(width)
                if raw.is_static_obstacle((x, y))}
    inflated = {(x + dx, y + dy) for x, y in occupied
                for dx in range(-5, 6) for dy in range(-5, 6)
                if 0 <= x + dx < width and 0 <= y + dy < height}
    static = Grid(tuple(tuple((x, y) in inflated for x in range(width))
                        for y in range(height)))
    return raw, static


def blocked(raw, observed):
    radius = (0.22 + 0.02) / 0.05 + sqrt(2.0) / 2.0
    return inflate_cells_circular(observed, raw.width, raw.height, radius)


@pytest.mark.parametrize("start,pose,observed", CAPTURED)
def test_captured_home_return_has_a_certified_only_start_cell_route(start, pose, observed):
    raw, static = classroom_grids()
    dynamic = blocked(raw, observed)
    assert start in dynamic and start not in observed
    assert not AStarPlanner().plan(static, start, (29, 100), dynamic)
    safe = certified_start_only_edges(raw, static, observed, dynamic,
                                      start, pose, .22 / .05, .02 / .05)
    assert (start, (start[0] - 1, start[1])) in safe
    unsafe_edges = {(start, (start[0] + dx, start[1] + dy))
                    for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                    if (dx or dy) and (start, (start[0] + dx, start[1] + dy)) not in safe}
    path = AStarPlanner().plan(static, start, (29, 100), dynamic,
                               allow_blocked_start=True, escape_cells={start},
                               blocked_edges=unsafe_edges)
    assert path and path[0] == start and path[-1] == (29, 100)
    assert (path[0], path[1]) in safe
    assert all(cell not in dynamic for cell in path[1:])
    assert all(not static.is_static_obstacle(cell) for cell in path)


def test_unsafe_raw_hit_blocks_start_only_exception():
    raw, static = classroom_grids()
    start, pose, observed = CAPTURED[0]
    observed = observed | {(41, 87)}
    assert certified_start_only_edges(raw, static, observed, blocked(raw, observed),
                                      start, pose, .22 / .05, .02 / .05) == set()


def test_static_inflation_start_clears_only_certified_cell_and_never_reenters():
    size, start, occupied = 40, (18, 18), (24, 18)
    raw = Grid(tuple(tuple((x, y) == occupied for x in range(size))
                     for y in range(size)))
    inflated = Grid(tuple(tuple(abs(x-24) <= 6 and abs(y-18) <= 6
                                for x in range(size)) for y in range(size)))
    assert inflated.is_static_obstacle(start) and not raw.is_static_obstacle(start)
    safe = certified_start_only_edges(raw, inflated, set(), set(), start,
                                      (18.5, 18.5), .24/.05, .02/.05)
    assert (start, (17, 18)) in safe
    assert (start, (19, 18)) not in safe
    view = Grid(tuple(tuple(False if (x, y) == start else inflated.rows[y][x]
                            for x in range(size)) for y in range(size)))
    denied = {(start, (18+dx, 18+dy)) for dx in (-1, 0, 1)
              for dy in (-1, 0, 1) if (dx or dy) and (start, (18+dx, 18+dy)) not in safe}
    path = AStarPlanner().plan(view, start, (10, 18), blocked_edges=denied)
    assert path and path[:2] == [start, (17, 18)]
    assert all(not inflated.is_static_obstacle(cell) for cell in path[1:])
    assert inflated.is_static_obstacle(start)


def test_static_start_diagonal_does_not_cut_adjacent_inflation_corner():
    raw = Grid(tuple(tuple(False for _ in range(40)) for _ in range(40)))
    inflated = Grid(tuple(tuple((x, y) in {(18, 18), (17, 18), (18, 17)}
                                for x in range(40)) for y in range(40)))
    safe = certified_start_only_edges(raw, inflated, set(), set(), (18, 18),
                                      (18.5, 18.5), 4.8, .4)
    assert ((18, 18), (17, 17)) not in safe


def test_clear_start_can_have_one_unsafe_first_edge_but_keep_safe_alternative():
    size = 40
    raw = Grid(tuple(tuple(False for _ in range(size)) for _ in range(size)))
    start = (18, 18)
    observed = {(24, 18)}
    dynamic = blocked(raw, observed)
    assert start in dynamic
    safe = certified_start_only_edges(raw, raw, observed, dynamic,
                                      start, (18.5, 18.5), 4.4, .4)
    assert (start, (19, 18)) not in safe
    assert (start, (17, 18)) in safe


@pytest.mark.parametrize("pose,radius", [((18.5, 18.5), float("nan")),
                                          ((18.5, 18.5), float("inf")),
                                          ((17.9, 18.5), 4.4)])
def test_invalid_or_outside_pose_never_authorizes_exception(pose, radius):
    size = 40
    raw = Grid(tuple(tuple(False for _ in range(size)) for _ in range(size)))
    start, observed = (18, 18), {(24, 18)}
    assert certified_start_only_edges(raw, raw, observed, blocked(raw, observed),
                                      start, pose, radius, .4) == set()


def test_unknown_or_occupied_map_and_raw_memory_remain_hard_blocks():
    size = 40
    rows = [[False] * size for _ in range(size)]
    rows[18][18] = True  # Includes unknown: Grid uses True for both.
    raw = Grid(tuple(tuple(row) for row in rows))
    start, observed = (18, 18), {(24, 18)}
    assert certified_start_only_edges(raw, raw, observed, blocked(raw, observed),
                                      start, (18.5, 18.5), 4.4, .4) == set()
    free = Grid(tuple(tuple(False for _ in range(size)) for _ in range(size)))
    assert certified_start_only_edges(free, free, observed | {start},
                                      blocked(free, observed | {start}), start,
                                      (18.5, 18.5), 4.4, .4) == set()


def test_ros_adapter_uses_fresh_continuous_start_exception_only(prepared):
    node = prepared
    image = Image.open(MAP).convert("L")
    width, height = image.size
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width, message.info.height = width, height
    message.info.resolution = .05
    message.info.origin.position.x, message.info.origin.position.y = -1.65, -5.25
    message.info.origin.orientation.w = 1.
    message.data = [100 if image.getpixel((x, height - y - 1)) == 0 else
                    -1 if image.getpixel((x, height - y - 1)) == 205 else 0
                    for y in range(height) for x in range(width)]
    raw, static = classroom_grids()
    node.map = (message, static)
    node.raw_static_grid = raw
    node.map_signature = (65, 5, 0)
    node.params["obstacle_inflation_cells"] = 5
    node.observed = set(CAPTURED[0][2])
    node.last_observation_time = node.clock["steady"]
    refresh(node, x=.4299471, y=-1.0708430, yaw=3.02)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = -.188431, -.220497
    goal.pose.orientation.w = 1.
    node.last_goal = goal
    node.plan_to(goal)
    path = node.path_pub.publish.call_args.args[0]
    assert path.poses and len(path.poses) > 2
    assert (path.poses[0].pose.position.x, path.poses[0].pose.position.y) == pytest.approx((.425, -1.075))
    assert path.poses[1].pose.position.x < path.poses[0].pose.position.x
    assert node.planned_cells[0] == CAPTURED[0][0]
    assert all(cell not in blocked(raw, node.observed) for cell in node.planned_cells[1:])


def test_ros_adapter_refuses_stale_pose_exception(prepared):
    node = prepared
    start = (18, 18)
    size = 40
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = message.info.height = size
    message.info.resolution = .05
    message.info.origin.orientation.w = 1.
    message.data = [0] * (size * size)
    free = Grid(tuple(tuple(False for _ in range(size)) for _ in range(size)))
    node.map = (message, free)
    node.raw_static_grid = free
    node.map_signature = (65, 5, 0)
    node.params["obstacle_inflation_cells"] = 5
    # Five-cell offset still overlaps the complete 24 cm footprint. Six
    # cells are now geometrically clear under the exact full-cell mask.
    node.observed = {(23, 18)}
    node.last_observation_time = node.clock["steady"]
    refresh(node, x=.925, y=.925)
    node.latest_scan_at = node.clock["steady"] - 2.0
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = 1.5, 1.5
    goal.pose.orientation.w = 1.
    node.last_goal = goal
    node.plan_to(goal)
    assert node.planned_cells == []
