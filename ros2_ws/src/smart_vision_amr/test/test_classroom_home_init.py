"""Home initialisation must reject stale sensors and competing maps."""
from math import cos, nan, sin
from types import SimpleNamespace as NS

import pytest
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from smart_vision_amr.classroom_home_init import (map_from_message,
    only_expected_map_server, source_fresh, stationary, valid_scan,
    fresh_home_transform)


def test_map_publishers_require_only_map_server():
    server = NS(node_name='map_server')
    slam = NS(node_name='cartographer_occupancy_grid_node')
    assert only_expected_map_server([server])
    assert not only_expected_map_server([])
    assert not only_expected_map_server([slam])
    assert not only_expected_map_server([server, slam])


def test_saved_map_conversion_preserves_ros_cell_order():
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width = 2
    message.info.height = 2
    message.info.resolution = .05
    message.info.origin.position.x = -1.5
    message.info.origin.orientation.w = 1.
    message.data = [0, 100, -1, 0]
    metadata, data = map_from_message(message)
    assert metadata['width'] == metadata['height'] == 2
    assert metadata['origin'] == {'x': -1.5, 'y': 0., 'yaw': 0.}
    assert data.tolist() == [[0, 100], [-1, 0]]
    message.data = [0, 1]
    with pytest.raises(ValueError):
        map_from_message(message)


def test_sensor_freshness_stationary_and_real_scan():
    stamp = NS(sec=100, nanosec=0)
    assert source_fresh(stamp, 101_000_000_000)
    assert not source_fresh(stamp, 103_000_000_000)
    assert not source_fresh(NS(sec=0, nanosec=0), 1_000_000_000)
    odom = Odometry()
    assert stationary(odom)
    odom.twist.twist.linear.x = .08
    assert not stationary(odom)
    scan = LaserScan()
    scan.range_min, scan.range_max = .1, 3.
    scan.ranges = [1.] * 12 + [nan]
    assert valid_scan(scan)
    scan.ranges = [nan] * 360
    assert not valid_scan(scan)


def test_home_transform_requires_fresh_position_and_heading():
    node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    transform = NS(header=NS(stamp=NS(sec=9, nanosec=500_000_000)),
                   transform=NS(translation=NS(x=-.11, y=-.79),
                                rotation=NS(x=0., y=0., z=0., w=1.)))
    buffer = NS(lookup_transform=lambda *_: transform)
    home = {'x': -.11, 'y': -.79, 'yaw': 0.}
    assert fresh_home_transform(node, buffer, home)
    transform.transform.rotation.z = sin(.5 / 2)
    transform.transform.rotation.w = cos(.5 / 2)
    assert not fresh_home_transform(node, buffer, home)
    transform.transform.rotation.z = 0.
    transform.transform.rotation.w = 1.
    transform.transform.translation.x = 1.
    assert not fresh_home_transform(node, buffer, home)
    transform.transform.translation.x = -.11
    transform.header.stamp.sec = 5
    assert not fresh_home_transform(node, buffer, home)
