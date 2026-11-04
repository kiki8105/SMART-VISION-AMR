"""Run the scan-to-web sweep gate on isolated ROS, never motor output."""
import math
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, build_parser, create_ros_bridge,
)


@pytest.fixture
def bridge(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=221, signal_handler_options=SignalHandlerOptions.NO)
    app = MissionApplication(DEFAULT_MARKERS, 221, False, tmp_path/'settings.json')
    node = create_ros_bridge(app, SimpleNamespace(camera_topic='/isolated_radius_camera',
        base_frame='isolated_radius_base', rotation_clearance_radius=.22), context)
    # No executor/timers are run; on_scan only computes observations.
    node.source_fresh = lambda *_args: True
    node.tf_buffer.lookup_transform = lambda *_args: SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    yield app, node
    node.destroy_node()
    context.shutdown()


def scan(distance):
    message = LaserScan()
    message.header.frame_id = 'isolated_radius_scan'
    message.header.stamp.sec = 1
    message.angle_min = -math.pi
    message.angle_increment = 2.*math.pi/400
    message.range_min, message.range_max = .12, 3.5
    message.ranges = [1.]*401
    message.ranges[200] = distance
    return message


def test_selected_physical_radius_avoids_hidden_thirty_centimeter_rotation_gate(bridge):
    app, node = bridge
    assert app.rotation_clearance_radius == .22
    node.on_scan(scan(.24))
    assert app.localization_clear
    assert app.localization_clearance == pytest.approx(.24)
    # Localization, marker search and Home trim consume this same gate.
    node.on_scan(scan(.21))
    assert not app.localization_clear
    assert app.localization_clearance == pytest.approx(.21)


def test_gazebo_sweep_keeps_larger_body_and_raw_hit_checks(bridge):
    app, node = bridge
    app.rotation_clearance_radius = .24
    node.on_scan(scan(.22))
    assert not app.localization_clear
    node.on_scan(scan(.25))
    assert app.localization_clear


def test_smaller_radius_never_reuses_clearance_when_transform_is_lost(bridge):
    app, node = bridge
    node.on_scan(scan(.24))
    assert app.localization_clear

    def lost(*_args):
        raise TransformException('sensor mount transform missing')

    node.tf_buffer.lookup_transform = lost
    node.on_scan(scan(.22))
    assert not app.localization_clear
    assert app.localization_clearance is None


def test_generic_sweep_default_is_preserved_and_invalid_options_fail_before_ros_node(tmp_path):
    assert build_parser().parse_args([]).rotation_clearance_radius == .30
    app = MissionApplication(DEFAULT_MARKERS, 221, False, tmp_path/'generic.json')
    for value in (0., -.2, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='rotation_clearance_radius'):
            create_ros_bridge(app, SimpleNamespace(rotation_clearance_radius=value), None)
