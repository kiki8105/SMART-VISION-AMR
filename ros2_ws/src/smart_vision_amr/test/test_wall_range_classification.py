"""Range-aware wall filtering retains stamped projection and visibility evidence.

The scan harness bypasses ROS node initialization and mocks every publisher,
transform lookup and persistence operation. These tests never start ROS nodes.
"""

from contextlib import contextmanager
from copy import deepcopy
from math import atan2, cos, hypot, pi, sin, sqrt
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from smart_vision_amr import map_utils
from smart_vision_amr.change_memory import wall_aligned_echo_indices
from smart_vision_amr.grid import Grid

from test_live_obstacles import harness


def wall_grid_info(*, origin_x=0.0, origin_y=0.0, yaw=0.0):
    info = SimpleNamespace(
        resolution=0.05,
        width=10,
        height=10,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=origin_x, y=origin_y),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=sin(yaw / 2), w=cos(yaw / 2)),
        ),
    )
    grid = Grid(tuple(tuple(x == 5 for x in range(info.width))
                      for _ in range(info.height)))
    return info, grid


def test_first_wall_uses_cell_entry_and_respects_maximum_range():
    info, grid = wall_grid_info()
    ray = map_utils.first_static_obstacle_distance

    assert ray(info, grid, 0.125, 0.125, 0.0, 1.0) == pytest.approx(0.125)
    assert ray(info, grid, 0.125, 0.125, 0.0, 0.124) is None
    assert ray(info, grid, 0.125, 0.125, pi / 2, 1.0) is None


def test_first_wall_respects_rotated_map_origin():
    info, grid = wall_grid_info(origin_x=1.0, origin_y=-2.0, yaw=pi / 2)

    assert map_utils.first_static_obstacle_distance(
        info, grid, 0.875, -1.875, pi / 2, 1.0) == pytest.approx(0.125)


def test_first_wall_checks_both_cells_touched_at_a_grid_corner():
    info, _ = wall_grid_info()
    rows = [[False] * info.width for _ in range(info.height)]
    rows[2][3] = True
    grid = Grid(tuple(tuple(row) for row in rows))

    assert map_utils.first_static_obstacle_distance(
        info, grid, 0.125, 0.125, pi / 4, 1.0) == pytest.approx(0.025 * sqrt(2))


@pytest.mark.parametrize('vertical', (False, True))
@pytest.mark.parametrize('yaw', (0.0, pi / 2))
def test_cardinal_ray_on_grid_boundary_checks_both_touched_rows(vertical, yaw):
    info, _ = wall_grid_info(yaw=yaw)
    rows = [[False] * info.width for _ in range(info.height)]
    rows[2 if vertical else 0][0 if vertical else 2] = True
    grid = Grid(tuple(tuple(row) for row in rows))
    lx, ly = (0.05, 0.025) if vertical else (0.025, 0.05)
    x, y = cos(yaw)*lx-sin(yaw)*ly, sin(yaw)*lx+cos(yaw)*ly
    angle = yaw + (pi / 2 if vertical else 0.0)
    assert map_utils.first_static_obstacle_distance(
        info, grid, x, y, angle, 1.0) == pytest.approx(0.075)


def test_unknown_blocks_planning_but_is_not_a_wall_reference():
    msg = OccupancyGrid()
    msg.info.resolution = 0.05
    msg.info.width = msg.info.height = 10
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 100
    msg.data[2 * 10 + 4] = -1
    planning_grid = map_utils.occupancy_to_grid(msg, 65, 0)
    known_walls = map_utils.occupancy_to_grid(msg, 65, 0, unknown_is_obstacle=False)
    ray = map_utils.first_static_obstacle_distance

    assert planning_grid.is_static_obstacle((4, 2))
    assert ray(msg.info, planning_grid, 0.125, 0.125, 0.0, 1.0) == pytest.approx(0.075)
    assert ray(msg.info, known_walls, 0.125, 0.125, 0.0, 1.0) is None
    assert ray(msg.info, known_walls, 0.125, 0.125, pi, 0.2) is None


@contextmanager
def wall_scan_harness(resolution=0.05):
    with harness() as node:
        node.params.update({
            'mapped_wall_range_tolerance_m': 0.05,
            'save_every_scans': 0,
        })
        node.visibility_pub = Mock()
        node.persist = Mock()
        node.scans_since_save = 0
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = msg.info.height = 50
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * 2500
        for index in range(50):
            msg.data[index] = msg.data[49 * 50 + index] = 100
            msg.data[index * 50] = msg.data[index * 50 + 49] = 100
        node.on_map(msg)
        node.transform.transform.translation.x = 43.5 * resolution
        node.transform.transform.translation.y = 10.5 * resolution
        yield node


def wall_band_scan(node, protrusion_m):
    """A 41-beam wall/object face with an unchanged raw LiDAR measurement."""
    info = node.map['message'].info
    node.transform.transform.translation.x = 41.5 * info.resolution
    node.ros_ns += 200_000_000
    scan = LaserScan()
    scan.header.frame_id = 'base_scan'
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(node.ros_ns, 1_000_000_000)
    node.transform.header.stamp = deepcopy(scan.header.stamp)
    scan.angle_min, scan.angle_increment = -0.30, 0.015
    scan.range_min, scan.range_max = 0.12, 4.0
    origin = node.transform.transform.translation
    scan.ranges = [map_utils.first_static_obstacle_distance(
        info, node.raw_static_grid, origin.x, origin.y,
        scan.angle_min + index*scan.angle_increment, scan.range_max) - protrusion_m
        for index in range(41)]
    raw_ranges = list(scan.ranges)
    node.on_scan(scan)
    assert list(scan.ranges) == raw_ranges  # Nav2's /scan is never modified.
    return scan


def test_wall_band_requires_multiple_beams_and_spatial_extent():
    assert wall_aligned_echo_indices({0: (0, 0), 1: (0, .02), 2: (0, .04)}) == set()
    assert wall_aligned_echo_indices({i: (0, i*.005) for i in range(20)}) == set()
    assert wall_aligned_echo_indices({i: (0, i*.01) for i in range(20)}) == set(range(20))


@pytest.mark.parametrize('wall_offset_m', (0.055, 0.08))
def test_extended_wall_echo_near_map_is_not_a_new_obstacle(wall_offset_m):
    with wall_scan_harness() as node:
        original_map = list(node.map['message'].data)
        wall_band_scan(node, wall_offset_m)
        assert node.observed() == set()
        assert 100 not in node.visibility_pub.publish.call_args.args[0].data
        assert list(node.map['message'].data) == original_map


def test_extended_object_fifteen_centimeters_before_wall_stays_detected():
    with wall_scan_harness() as node:
        wall_band_scan(node, 0.15)
        observed = node.observed()
        assert observed
        assert all(index % 50 <= 46 for index in observed)
        assert 100 in node.visibility_pub.publish.call_args.args[0].data


@pytest.mark.parametrize('resolution', (0.05, 0.1))
@pytest.mark.parametrize('echo_offset', (0.0, 0.025))
def test_wall_echo_within_five_centimeters_is_excluded_at_source(resolution, echo_offset):
    with wall_scan_harness(resolution) as node:
        node.params['memory_enabled'] = True
        original = list(node.map['message'].data)
        wall_distance = 5.5 * resolution
        node.scan(wall_distance - echo_offset)

        assert node.observed() == set()
        assert not node.memory.cells
        visibility = node.visibility_pub.publish.call_args.args[0]
        assert 100 not in visibility.data
        assert visibility.data[10 * 50 + 49] == -1
        assert list(node.map['message'].data) == original


@pytest.mark.parametrize('resolution', (0.05, 0.1))
def test_range_tolerance_stays_five_centimeters_across_map_resolutions(resolution):
    with wall_scan_harness(resolution) as node:
        # A 7.5 cm nearer return is real even on a 10 cm grid. Converting the
        # old one/two-cell proximity tolerance into range would hide this hit.
        distance = 5.5 * resolution - 0.075
        node.scan(distance)
        # The 5 cm map's return is also in the near-min range. It needs a
        # second independent scan before it can modify live obstacle cells.
        node.scan(distance)
        cell = map_utils.world_to_cell(
            node.map['message'].info,
            node.transform.transform.translation.x + distance,
            node.transform.transform.translation.y,
        )

        assert node.observed() == {cell[1] * 50 + cell[0]}
        assert node.visibility_pub.publish.call_args.args[0].data[cell[1] * 50 + cell[0]] == 100


def test_wall_adjacent_object_uses_full_confirmation_and_free_clearing_rules():
    with wall_scan_harness() as node:
        node.params['memory_enabled'] = True
        original = list(node.map['message'].data)
        angle = atan2(0.75, 0.20)
        distance = hypot(0.75, 0.20)
        cell = (47, 25)
        raw = map_utils.occupancy_to_grid(node.map['message'], 65, 0)
        assert not map_utils.split_mapped_wall_returns({cell}, raw, 2)[0]

        with patch('smart_vision_amr.change_memory.time.time', return_value=100.0) as observed_at:
            for index in range(5):
                observed_at.return_value = 100.0 + index * 0.525
                node.clock.return_value = 10.0 + index * 0.525
                node.scan(distance, angle=angle, advance=0.525)
                if index < 4:
                    assert not node.memory.confirmed_cells

        assert node.observed() == {25 * 50 + 47}
        assert node.memory.confirmed_cells == {cell}
        visibility = node.visibility_pub.publish.call_args.args[0]
        assert visibility.data[25 * 50 + 47] == 100
        assert visibility.data[10 * 50 + 43] == 0
        assert visibility.data[10 * 50 + 10] == -1
        assert list(node.map['message'].data) == original

        wall_distance = map_utils.first_static_obstacle_distance(
            node.map['message'].info, raw,
            node.transform.transform.translation.x,
            node.transform.transform.translation.y, angle, 4.0)
        assert wall_distance is not None
        with patch('smart_vision_amr.change_memory.time.time', return_value=104.0) as observed_at:
            for index in range(5):
                observed_at.return_value = 104.0 + index * 0.525
                node.clock.return_value = 14.0 + index * 0.525
                node.scan(wall_distance, angle=angle, advance=0.525)
                if index < 4:
                    assert node.memory.confirmed_cells == {cell}

        assert node.observed() == set()
        assert not node.memory.confirmed_cells
        assert list(node.map['message'].data) == original


def test_unknown_frontier_does_not_hide_a_nearby_live_return():
    with wall_scan_harness() as node:
        msg = node.map['message']
        msg.data[10 * 50 + 47] = -1
        node.on_map(msg)
        node.scan(0.15)
        node.scan(0.15)

        assert node.observed() == {10 * 50 + 46}
        visibility = node.visibility_pub.publish.call_args.args[0]
        assert visibility.data[10 * 50 + 46] == 100
        assert visibility.data[10 * 50 + 47] == -1


def test_all_invalid_rays_do_not_publish_fresh_visibility_or_clear_old_hits():
    with wall_scan_harness() as node:
        node.scan(0.15)
        node.scan(0.15)
        counts = (node.visibility_pub.publish.call_count, node.observation_pub.publish.call_count)
        stamp = node.last_scan_stamp_ns
        for value in (float('nan'), -float('inf'), 0.0, -0.1, 5.0):
            node.scan(value)

        assert (node.visibility_pub.publish.call_count, node.observation_pub.publish.call_count) == counts
        assert node.observed() == {10 * 50 + 46}
        assert node.last_scan_stamp_ns == stamp


def test_wall_classification_and_visibility_keep_exact_scan_measurement_time():
    with wall_scan_harness() as node:
        scan = node.scan(0.25)
        stamp_ns = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        calls = node.tf_buffer.lookup_transform.call_args_list

        assert node.observed() == set()
        assert {call.args[1] for call in calls} == {'base_scan', 'odom'}
        assert all(call.args[2].nanoseconds == stamp_ns for call in calls)
        assert node.visibility_pub.publish.call_args.args[0].header.stamp == scan.header.stamp
