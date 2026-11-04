"""Live scan geometry must preserve a clear diagonal without hiding contact."""

from math import hypot, sqrt
from pathlib import Path
from types import SimpleNamespace

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.map_utils import (
    inflate_cells, inflate_cells_circular, inflate_cells_footprint, occupancy_to_grid,
)
from smart_vision_amr.web_mission_server import load_saved_map


MAP = Path(__file__).resolve().parents[1] / 'maps/classroom_20261030_cleaned.yaml'
START, HOME = (32, 53), (29, 100)
# Captured with the robot near Sector 3, after the Oct 30 map was loaded.
OBSERVED_INDICES = (
    3457, 3458, 3474, 3543, 4145, 4231, 4232, 4317, 4318, 4403, 4404,
    4920, 5006, 5025, 5026, 5092, 5111, 5112, 5178, 5264, 5350, 5351,
    5436, 5437, 5522, 5523, 5609, 5695, 5781, 5867, 5953, 6039, 6229,
)


def test_classroom_live_diagonal_clearance_recovers_a_home_route():
    metadata, data = load_saved_map(MAP)
    width, height, resolution = metadata['width'], metadata['height'], metadata['resolution']
    message = SimpleNamespace(
        info=SimpleNamespace(width=width, height=height, resolution=resolution),
        data=data.ravel().tolist(),
    )
    static_grid = occupancy_to_grid(message, 65, 5)
    hits = {(index % width, index // width) for index in OBSERVED_INDICES}
    square = inflate_cells(hits, width, height, 5)
    physical_radius = .22 + .02
    # One cell's center can be anywhere in its square; half of its diagonal
    # preserves clearance even at the farthest center-position error.
    uncertainty = sqrt(2.) / 2.
    circular = inflate_cells_circular(
        hits, width, height, physical_radius/resolution + uncertainty)
    circular_with_2cm_observation = inflate_cells_circular(
        hits, width, height, (physical_radius+.02)/resolution + uncertainty)
    assert START in square
    assert START not in circular
    assert START not in circular_with_2cm_observation
    assert not AStarPlanner().plan(static_grid, START, HOME, square)
    for mask in (circular, circular_with_2cm_observation):
        route = AStarPlanner().plan(static_grid, START, HOME, mask)
        assert route and route[0] == START and route[-1] == HOME
        assert not any(cell in mask or static_grid.is_static_obstacle(cell)
                       for cell in route)


def test_circular_live_mask_still_blocks_real_footprint_overlap():
    hit = (35, 55)
    radius = (.22 + .02)/.05 + sqrt(2.)/2.
    blocked = inflate_cells_circular({hit}, 86, 140, radius)
    assert hit in blocked
    assert START in blocked
    # The raw occupied square is closer than the robot radius.
    nearest = hypot(max(abs(START[0]-hit[0])-.5, 0.),
                    max(abs(START[1]-hit[1])-.5, 0.)) * .05
    assert nearest < .24
    # Confirmed-memory inflation deliberately stays unchanged and conservative.
    assert START in inflate_cells({(37, 58)}, 86, 140, 5)


def test_exact_live_footprint_has_no_extra_axial_cell_but_no_geometric_overlap():
    hit = (15, 15)
    resolution = .05
    radius = .24 / resolution
    mask = inflate_cells_footprint({hit}, 31, 31, radius)
    assert (20, 15) in mask  # Worst-case square separation: 0.20 m.
    assert (21, 15) not in mask  # Worst-case separation: 0.25 m.
    assert (19, 19) in mask  # Diagonal nearest corners are only 0.212 m apart.
    assert (20, 19) not in mask  # 3-4-5 triangle has 0.25 m clearance.
    for x in range(31):
        for y in range(31):
            if (x, y) in mask:
                continue
            nearest = hypot(max(abs(x-hit[0])-1, 0),
                            max(abs(y-hit[1])-1, 0)) * resolution
            assert nearest > .24
