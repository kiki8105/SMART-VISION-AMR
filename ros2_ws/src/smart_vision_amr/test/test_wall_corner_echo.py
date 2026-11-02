"""A mapped corner grazing return needs neighboring wall evidence, not a wider range gate."""

from copy import deepcopy
from math import pi
from pathlib import Path

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from smart_vision_amr.map_utils import (
    cell_to_world, distance_to_known_wall_area, occupancy_to_grid,
    wall_echo_reference_grid)
from test_live_obstacles import harness


def corner_map():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = .05
    msg.info.width, msg.info.height = 40, 20
    msg.info.origin.position.x = -1.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * (40 * 20)
    msg.data[5 * 40 + 25] = 100  # x=[.25,.30], y=[.25,.30]
    return msg


def scan_beams(node, ranges):
    node.ros_ns += 100_000_000
    scan = LaserScan()
    scan.header.frame_id = 'base_scan'
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(node.ros_ns, 1_000_000_000)
    node.transform.header.stamp = deepcopy(scan.header.stamp)
    scan.angle_min, scan.angle_increment = 0.0, pi / 180.0
    scan.range_min, scan.range_max = .12, 4.0
    scan.ranges = ranges
    node.on_scan(scan)
    return node.observed()


def configure_corner(node):
    node.on_map(corner_map())
    node.transform.transform.translation.x = -.7
    node.transform.transform.translation.y = .275


def test_corner_graze_uses_endpoint_area_even_without_neighboring_wall_return():
    # The 3-degree beam stops just outside the occupied corner. Its exact
    # map ray goes above the wall, but the measured endpoint is within 5 cm
    # of a wall area that A* already excludes.
    with harness() as node:
        configure_corner(node)
        assert scan_beams(node, [.95, float('inf'), float('inf'), .931]) == set()

    with harness() as node:
        configure_corner(node)
        assert scan_beams(node, [float('inf')] * 3 + [.931]) == set()


def test_real_object_75mm_ahead_of_wall_remains_dynamic_even_with_wall_neighbor():
    with harness() as node:
        configure_corner(node)
        # The wall is .95m away. The object at .875m is 7.5cm in front;
        # adjacent one-degree beam still sees the actual mapped wall.
        assert scan_beams(node, [.875, .95]) == {5 * 40 + 23}


def test_unknown_boundary_is_not_reinflated_as_a_new_obstacle():
    with harness() as node:
        msg = corner_map()
        msg.data[5 * 40 + 25] = -1
        msg.data[5 * 40 + 26] = 100  # unknown fringe touches a known wall
        node.on_map(msg)
        node.transform.transform.translation.x = -.7
        node.transform.transform.translation.y = .275
        assert scan_beams(node, [.95, float('inf'), float('inf'), .931]) == set()


def test_new_object_75mm_in_front_of_unknown_boundary_stays_live():
    with harness() as node:
        msg = corner_map()
        msg.data[5 * 40 + 25] = -1
        msg.data[5 * 40 + 26] = 100
        node.on_map(msg)
        node.transform.transform.translation.x = -.7
        node.transform.transform.translation.y = .275
        assert scan_beams(node, [.875]) == {5 * 40 + 23}


def test_actual_classroom_wall_boundary_cells_are_narrowly_classified():
    pgm = Path(__file__).parents[1] / 'maps/classroom_20261030_cleaned.pgm'
    image = pgm.read_bytes().split(b'\n', 3)[3]
    width, height = 86, 140
    assert len(image) == width * height
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = width, height
    msg.info.resolution = .05
    msg.info.origin.position.x = -1.65
    msg.info.origin.position.y = -5.25
    msg.info.origin.orientation.w = 1.0
    msg.data = [0 if pixel >= 250 else -1 if pixel > 100 else 100
                for pixel in (image[(height - 1 - y)*width + x]
                              for y in range(height) for x in range(width))]
    grid = occupancy_to_grid(msg, 65, 0, unknown_is_obstacle=False)
    echo_grid = wall_echo_reference_grid(msg)
    for cell in ((46, 109), (65, 124)):
        assert distance_to_known_wall_area(msg.info, grid, *cell_to_world(msg.info, cell), .05) is not None
    # The third observed point is 7.5 cm from known wall area. It stays live;
    # being next to unknown map pixels alone is insufficient suppression.
    assert distance_to_known_wall_area(
        msg.info, grid, *cell_to_world(msg.info, (64, 124)), .05) is None
    assert echo_grid.is_static_obstacle((56, 109))
