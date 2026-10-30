"""Source/receipt time geometry regressions; no ROS context or DDS is created.

These callbacks deliberately exercise asynchronous odometry and Nav2 polygon
updates. A moving robot's current pose is not the pose of an older polygon.
"""

from copy import deepcopy
from math import cos, pi, sin
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped

from test_navigation_input_preflight import healthy_follower, polygon


def ros_now(node, seconds):
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(seconds*1_000_000_000)))


def odom(node, seconds, x, *, yaw=0., frame='odom'):
    message = deepcopy(node.latest_odom)
    nanoseconds = round(seconds*1_000_000_000)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
    message.header.frame_id = frame
    message.pose.pose.position.x = x
    message.pose.pose.orientation.z = sin(yaw/2.)
    message.pose.pose.orientation.w = cos(yaw/2.)
    message.twist.twist.linear.x = .15
    node.on_odom(message)


def circle(node, center_x, *, stamp=4.2, frame='odom'):
    message = PolygonStamped()
    message.header.frame_id = frame
    for index in range(16):
        point = Point32()
        point.x, point.y = center_x+.22*cos(index*pi/8), .22*sin(index*pi/8)
        message.polygon.points.append(point)
    nanoseconds = round(stamp*1_000_000_000)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        nanoseconds, 1_000_000_000)
    node.on_footprint(message)


def delayed_polygon(*, center_x=0.):
    node = healthy_follower()
    # A valid .15m/s motion: the footprint is .8s behind the newest odometry,
    # but both header and receipt remain inside the existing freshness bounds.
    odom(node, 4.1, -.015)
    odom(node, 4.3, .015)
    for tenth in range(44, 51):
        odom(node, tenth/10., .15*(tenth/10.-4.2))
    circle(node, center_x)
    return node


def test_delayed_fresh_polygon_uses_interpolated_source_time_odometry():
    node = delayed_polygon()
    assert node._navigation_input_error() is None


def test_forged_fresh_polygon_cannot_match_newer_pose_instead_of_its_source_time():
    # .20 is only .08m from the newest pose, but .20m from its stamped pose.
    # The unchanged .10m geometry bound must reject that false agreement.
    node = delayed_polygon(center_x=.20)
    assert node._navigation_input_error() is not None


def test_fresh_polygon_transform_uses_its_acquisition_time_not_latest_tf():
    node = delayed_polygon()
    circle(node, 0., frame='moving_grid')

    def transform(target, source, stamp, *args, **kwargs):
        message = TransformStamped()
        message.header.stamp.sec = 4
        message.header.stamp.nanosec = 200_000_000
        message.transform.rotation.w = 1.
        # The source frame subsequently moved by a metre. Only its stamped
        # transform maps the old polygon into the matching odometry frame.
        message.transform.translation.x = 0. if stamp.nanoseconds == 4_200_000_000 else 1.
        return message

    node.tf_buffer.lookup_transform.side_effect = transform
    assert node._navigation_input_error() is None
    assert any(call.args[:2] == ('odom', 'moving_grid')
               and call.args[2].nanoseconds == 4_200_000_000
               for call in node.tf_buffer.lookup_transform.call_args_list)


@pytest.mark.parametrize('fault', ['gap', 'frame_change', 'jump'])
def test_source_time_interpolation_rejects_unproven_or_discontinuous_motion(fault):
    node = healthy_follower()
    odom(node, 4.1, -.015)
    odom(node, 4.5 if fault == 'gap' else 4.3,
         1. if fault == 'jump' else .015,
         frame='other_odom' if fault == 'frame_change' else 'odom')
    for tenth in range(46 if fault == 'gap' else 44, 51):
        odom(node, tenth/10., .15*(tenth/10.-4.2))
    circle(node, 0.)
    assert node._navigation_input_error() is not None


def test_source_time_history_cannot_hide_stale_current_odometry_receipt():
    node = delayed_polygon()
    node.latest_odom_at -= 1.
    assert 'receipt' in node._navigation_input_error()


def test_old_header_current_geometry_still_has_receipt_time_pose_proof():
    node = delayed_polygon()
    circle(node, .12, stamp=1.)
    assert node._navigation_input_error() is None


def test_old_header_does_not_allow_stale_coordinates_with_a_new_receipt():
    node = delayed_polygon()
    circle(node, 0., stamp=1.)
    assert node._navigation_input_error() is not None


@pytest.mark.parametrize('stamp, allowed', [(1., False), (4.2, True)])
def test_only_valid_source_stamp_can_compensate_more_than_ten_cm_of_later_motion(stamp, allowed):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        ros_now(node, 4.2)
        odom(node, 4.2, 0.)
        circle(node, 0., stamp=stamp)
        # Same physical geometry/motion in both cases. Only the truthful source
        # stamp lets downstream consumers reconstruct the historical body pose.
        for tenth in range(43, 51):
            clock.return_value = 100.+tenth/10.-4.2
            ros_now(node, tenth/10.)
            odom(node, tenth/10., .15*(tenth/10.-4.2))
        for source in ('scan', 'costmap'):
            getattr(node, 'latest_'+source).header.stamp.sec = 5
            setattr(node, 'latest_'+source+'_at', 100.8)
        issue = node._navigation_input_error()
        assert (issue is None) is allowed
        if not allowed:
            assert 'Old footprint position' in issue


def test_old_asymmetric_polygon_still_requires_current_body_orientation_agreement():
    node = healthy_follower()
    rectangle = [(-.3, -.15), (.3, -.15), (.3, .15), (-.3, .15)]
    polygon(node, rectangle, stamp=5)
    assert node._navigation_input_error() is None
    polygon(node, rectangle, stamp=1)
    ros_now(node, 5.2)
    odom(node, 5.2, 0., yaw=.2)
    assert 'orientation' in node._navigation_input_error()
