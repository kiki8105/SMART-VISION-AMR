"""Focused checks for the Gazebo memory transition fixture."""

import importlib.util
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
ACTOR = ROOT/'scripts/inject_classroom_memory_transition.py'
spec = importlib.util.spec_from_file_location('memory_transition_actor', ACTOR)
actor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(actor)


def test_nearby_cells_only_reports_owned_obstacle_area():
    info = SimpleNamespace(width=4, height=3)
    def cell_to_world(_info, cell):
        return (cell[0] * .2, cell[1] * .2)
    assert actor.nearby_indices([0, 1, 2, 3, 12, -1], info, (.4, 0.),
                                cell_to_world, radius=.21) == {1, 2, 3}


def test_removal_outcome_preserves_valid_path_but_reports_new_route():
    assert actor.removal_outcome('same', 'same', 4, 4) == 'CONTINUED_EXISTING_PATH'
    assert actor.removal_outcome('same', 'other', 4, 4) == 'NEW_PATH_OR_REPLAN'
    assert actor.removal_outcome('same', 'same', 4, 5) == 'NEW_PATH_OR_REPLAN'
    assert actor.removal_outcome(None, 'same', 4, 5) == 'UNDETERMINED'


def test_box_fixture_stays_clear_of_first_route_but_changes_second_plan():
    """Catch a map/box mismatch before an expensive Gazebo mission starts."""
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.clearance_cost import clearance_penalties
    from smart_vision_amr.map_utils import (
        bresenham, cell_to_world, inflate_cells, inflate_cells_circular,
        occupancy_to_grid, world_to_cell,
    )
    from inject_classroom_web_obstacle import (
        box_cells_on_raw_map, current_observed_bypass_route,
        path_clearance_from_box, preflight_box_route,
    )

    image = ROOT/'ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.pgm'
    magic, dimensions, max_value, pixels = image.read_bytes().split(b'\n', 3)
    assert magic == b'P5' and max_value == b'255'
    width, height = map(int, dimensions.split())
    assert len(pixels) == width * height
    info = SimpleNamespace(
        width=width, height=height, resolution=.05,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=-1.65, y=-5.25),
            orientation=SimpleNamespace(x=0., y=0., z=0., w=1.),
        ),
    )
    map_msg = SimpleNamespace(
        info=info,
        data=[0 if value >= 251 else 100 if value <= 2 else -1
              for row in range(height - 1, -1, -1)
              for value in pixels[row * width:(row + 1) * width]],
    )
    inflated = occupancy_to_grid(map_msg, 65, 5)
    raw = occupancy_to_grid(map_msg, 65, 0)
    planner = AStarPlanner()
    home = (-.188431, -.220497)
    sector1, sector2 = actor.SECTOR1, actor.SECTOR2
    first = [cell_to_world(info, cell) for cell in planner.plan(
        inflated, world_to_cell(info, *home), world_to_cell(info, *sector1))]
    second = [cell_to_world(info, cell) for cell in planner.plan(
        inflated, world_to_cell(info, *sector1), world_to_cell(info, *sector2))]

    # The first leg only establishes the memory. Its obstacle must be visible
    # for several early scans without forcing the real controller to turn
    # toward a square box; Stage 23 separately tests live-path obstruction.
    assert .70 <= math.dist(home, actor.BOX) <= .95
    assert path_clearance_from_box(first, actor.BOX) >= .15
    first_with_box = current_observed_bypass_route(
        map_msg, home, sector1, actor.BOX, (), world_to_cell, cell_to_world)
    assert first_with_box
    assert path_clearance_from_box(
        [cell_to_world(info, cell) for cell in first_with_box], actor.BOX) >= .15
    assert path_clearance_from_box(second, actor.BOX) <= .05
    assert preflight_box_route(map_msg, sector1, sector2, actor.BOX, second,
                               world_to_cell, cell_to_world)
    # The remembered second-leg plan must retain room beyond the 2-cm hard
    # stop, and the fixture needs sufficient approach distance from Home.
    box_cells = box_cells_on_raw_map(raw, info, actor.BOX,
                                     world_to_cell, cell_to_world)
    live_blocked = inflate_cells_circular(
        box_cells, width, height, .24 / info.resolution + math.sqrt(2) / 2)
    remembered_blocked = inflate_cells(box_cells, width, height, 5)
    bypass = [cell_to_world(info, cell) for cell in planner.plan(
        inflated, world_to_cell(info, *home),
        world_to_cell(info, *sector1), live_blocked)]
    # Use the same clearance cost as the running A* node. An unweighted A*
    # tie can choose a wall-side corridor that the real planner does not use.
    later_route = [cell_to_world(info, cell) for cell in planner.plan(
        inflated, world_to_cell(info, *sector1),
        world_to_cell(info, *sector2), remembered_blocked,
        cell_penalty=clearance_penalties(raw, box_cells, info.resolution,
                                         .35, 1.5))]
    assert path_clearance_from_box(bypass, actor.BOX) >= .15
    assert path_clearance_from_box(later_route, actor.BOX) >= .02
    # Stage 24 confirmed four cells along the visible north-west face before
    # Sector 2 planning. Guard the partial-memory path as well as the full-box
    # extreme; both must clear the physical box by at least 2 cm.
    bx, by = world_to_cell(info, *actor.BOX)
    visible_face = {(bx + dx, by + 2) for dx in (-3, -2, -1, 0)}
    partial_route = [cell_to_world(info, cell) for cell in planner.plan(
        inflated, world_to_cell(info, *sector1),
        world_to_cell(info, *sector2),
        inflate_cells(visible_face, width, height, 5),
        cell_penalty=clearance_penalties(raw, visible_face,
                                         info.resolution, .35, 1.5))]
    assert path_clearance_from_box(partial_route, actor.BOX) >= .02
    # The mapped partition routinely produces these live returns before the
    # box exists; they must not be mistaken for the injected obstacle.
    existing_returns = (8647, 8733, 8819, 8905, 8991)
    assert not actor.nearby_indices(
        existing_returns, info, actor.BOX, cell_to_world)
    ray = list(bresenham(world_to_cell(info, *sector1),
                         world_to_cell(info, *actor.BOX)))
    assert any(raw.is_static_obstacle(cell) for cell in ray[1:-1])
    home_ray = list(bresenham(world_to_cell(info, *home),
                              world_to_cell(info, *actor.BOX)))
    assert all(not raw.is_static_obstacle(cell) for cell in home_ray[1:-1])
    for pose in first[:8]:
        early_ray = list(bresenham(world_to_cell(info, *pose),
                                  world_to_cell(info, *actor.BOX)))
        assert all(not raw.is_static_obstacle(cell) for cell in early_ray[1:-1])


def test_actor_cannot_start_in_robot_domain(tmp_path):
    result = subprocess.run(
        [sys.executable, str(ACTOR), '--output', str(tmp_path/'evidence.json')],
        env={'PATH': '/usr/bin:/bin', 'ROS_DOMAIN_ID': '16',
             'ROS_LOCALHOST_ONLY': '0', 'PYTHONPATH': str(ROOT/'scripts')},
        capture_output=True, text=True)
    assert result.returncode != 0
    assert 'requires ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1' in result.stderr
    assert not (tmp_path/'evidence.json').exists()
