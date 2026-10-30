"""Navigation geometry and cancellation checks using messages and mocked actions only.

FollowerHarness never invokes Node.__init__; no ROS context, node, publisher,
executor, discovery, service, or robot connection is created by these tests.
"""

import json
import math
import time
from unittest.mock import Mock, patch

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from test_path_follower_cancel import FollowerHarness, goal_for, make_path, result_future


def polygon(node, vertices=None, *, stamp=5):
    message = PolygonStamped()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = stamp
    vertices = vertices or [(.22*math.cos(index*math.pi/8), .22*math.sin(index*math.pi/8))
                            for index in range(16)]
    for x, y in vertices:
        point = Point32()
        point.x, point.y = float(x), float(y)
        message.polygon.points.append(point)
    node.on_footprint(message)


def healthy_follower():
    node = FollowerHarness()
    node.navigation_inputs_required = True
    node.navigation_input_fault = None
    node.navigation_input_cutoff_ns = None
    node.navigation_footprint_reference = None
    node.execution_failure_pub = Mock()
    node.parameters['curve_base_frame'] = 'base_footprint'
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.side_effect = TransformException('No test transform')
    scan = LaserScan()
    scan.header.frame_id = 'base_scan'
    scan.header.stamp.sec = 5
    scan.angle_min, scan.angle_increment = -math.pi, math.pi/180
    scan.range_min, scan.range_max = .12, 3.5
    scan.ranges = [2.]*360
    node.on_scan(scan)
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
    odom.header.stamp.sec = 5
    odom.pose.pose.orientation.w = 1.
    node.on_odom(odom)
    costmap = OccupancyGrid()
    costmap.header.frame_id = 'odom'
    costmap.header.stamp.sec = 5
    costmap.info.width = costmap.info.height = 60
    costmap.info.resolution = .05
    costmap.info.origin.position.x = costmap.info.origin.position.y = -1.5
    costmap.info.origin.orientation.w = 1.
    costmap.data = [0]*3600
    node.on_costmap(costmap)
    polygon(node)
    return node


def statuses(node):
    return [json.loads(call.args[0].data) for call in node.status_pub.publish.call_args_list]


def start(node):
    path = make_path(stamp=5)
    node.on_goal(goal_for(path, stamp=4))
    node.on_path(path)
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)), node.request_context)
    return path, handle


def test_fresh_consistent_inputs_and_old_circular_header_are_accepted_at_startup():
    node = healthy_follower()
    assert node._navigation_input_error() is None
    node.navigation_footprint_reference = None
    node.latest_footprint.header.stamp.sec = 1
    node.latest_odom.pose.pose.orientation.z = math.sin(.8)
    node.latest_odom.pose.pose.orientation.w = math.cos(.8)
    assert node._navigation_input_error() is None


def test_footprint_timestamp_exception_does_not_accept_a_future_header():
    node = healthy_follower()
    node.latest_footprint.header.stamp.sec = 6
    assert 'timestamp' in node._navigation_input_error()


@pytest.mark.parametrize('source', ['scan', 'odom', 'costmap', 'footprint'])
@pytest.mark.parametrize('receipt', [None, float('nan'), -2., .1])
def test_missing_nonfinite_stale_and_future_receipts_fail_closed(source, receipt):
    node = healthy_follower()
    setattr(node, 'latest_'+source+'_at',
            receipt if receipt is None or not math.isfinite(receipt) else time.monotonic()+receipt)
    assert 'receipt' in node._navigation_input_error()


@pytest.mark.parametrize('source', ['scan', 'odom', 'costmap'])
def test_fresh_receipt_cannot_hide_old_source_header(source):
    node = healthy_follower()
    getattr(node, 'latest_'+source).header.stamp.sec = 1
    assert 'header' in node._navigation_input_error()


@pytest.mark.parametrize('fault', ['empty', 'nan', 'angle', 'range', 'frame'])
def test_invalid_scan_content_or_unconnected_frame_blocks_navigation(fault):
    node = healthy_follower()
    if fault == 'empty': node.latest_scan.ranges = []
    if fault == 'nan': node.latest_scan.ranges = [float('nan')]*360
    if fault == 'angle': node.latest_scan.angle_increment = 0.
    if fault == 'range': node.latest_scan.range_max = float('nan')
    if fault == 'frame': node.latest_scan.header.frame_id = 'unconnected_scan'
    assert node._navigation_input_error() is not None


@pytest.mark.parametrize('fault', ['zero_quaternion', 'nan_position', 'nan_twist', 'nan_covariance'])
def test_invalid_odometry_cannot_validate_footprint(fault):
    node = healthy_follower()
    if fault == 'zero_quaternion': node.latest_odom.pose.pose.orientation.w = 0.
    if fault == 'nan_position': node.latest_odom.pose.pose.position.x = float('nan')
    if fault == 'nan_twist': node.latest_odom.twist.twist.angular.y = float('nan')
    if fault == 'nan_covariance': node.latest_odom.pose.covariance[0] = float('nan')
    assert node._navigation_input_error() is not None


@pytest.mark.parametrize('fault', ['missing_data', 'zero_width', 'nan_resolution', 'quaternion',
                                  'off_center', 'invalid_occupancy'])
def test_invalid_costmap_payload_and_geometry_fail_closed(fault):
    node = healthy_follower()
    if fault == 'missing_data': node.latest_costmap.data = []
    if fault == 'zero_width': node.latest_costmap.info.width = 0
    if fault == 'nan_resolution': node.latest_costmap.info.resolution = float('nan')
    if fault == 'quaternion': node.latest_costmap.info.origin.orientation.w = 0.
    if fault == 'off_center': node.latest_costmap.info.origin.position.x += 1.
    if fault == 'invalid_occupancy': node.latest_costmap.data[0] = 101
    assert node._navigation_input_error() is not None


@pytest.mark.parametrize('vertices', [[], [(0., 0.)], [(0., 0.), (.1, .1), (.2, .2)],
                                    [(-.2, -.2), (.2, .2), (-.2, .2), (.2, -.2)],
                                    [(float('nan'), 0.), (.2, .2), (-.2, .2)],
                                    [(-2., -2.), (2., -2.), (2., 2.), (-2., 2.)]])
def test_invalid_degenerate_crossed_and_unreasonable_footprints_are_rejected(vertices):
    node = healthy_follower()
    if vertices:
        polygon(node, vertices)
    else:
        node.latest_footprint.polygon.points.clear()
    assert node._navigation_input_error() is not None


def test_old_footprint_must_match_current_position_and_have_fresh_receipt():
    node = healthy_follower()
    node.latest_odom.pose.pose.position.x = .2
    node.on_odom(node.latest_odom)
    polygon(node, stamp=1)  # A new receipt carrying stale coordinates.
    assert 'Footprint disagrees' in node._navigation_input_error()
    node.latest_odom.pose.pose.position.x = 0.
    node.latest_footprint_at = time.monotonic()-2.
    assert 'receipt' in node._navigation_input_error()


def test_old_asymmetric_footprint_requires_body_reference_and_matching_orientation():
    node = healthy_follower()
    rectangle = [(-.3, -.15), (.3, -.15), (.3, .15), (-.3, .15)]
    polygon(node, rectangle, stamp=1)
    assert 'reference' in node._navigation_input_error()
    polygon(node, rectangle, stamp=5)
    assert node._navigation_input_error() is None
    node.latest_odom.pose.pose.orientation.z = math.sin(math.pi/4)
    node.latest_odom.pose.pose.orientation.w = math.cos(math.pi/4)
    node.on_odom(node.latest_odom)
    polygon(node, rectangle, stamp=1)  # Newly received, but the old orientation.
    assert 'orientation' in node._navigation_input_error()
    polygon(node, [(-y, x) for x, y in rectangle], stamp=1)
    assert node._navigation_input_error() is None


def test_rotated_costmap_origin_and_static_connected_sensor_frame_are_valid():
    node = healthy_follower()
    origin = node.latest_costmap.info.origin
    origin.position.x, origin.position.y = 1.5, -1.5
    origin.orientation.z, origin.orientation.w = math.sin(math.pi/4), math.cos(math.pi/4)
    node.latest_scan.header.frame_id = 'robot/base_scan'
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    node.tf_buffer.lookup_transform.side_effect = None
    node.tf_buffer.lookup_transform.return_value = transform
    assert node._navigation_input_error() is None
    transform.header.stamp.sec = 1
    assert 'transform' in node._navigation_input_error()


def test_costmap_and_footprint_in_connected_frames_are_compared_in_odometry_frame():
    node = healthy_follower()
    node.latest_costmap.header.frame_id = node.latest_footprint.header.frame_id = 'local_grid'
    node.latest_odom.pose.pose.position.x = 1.
    node.on_odom(node.latest_odom)
    transform = TransformStamped()
    transform.transform.translation.x = 1.
    transform.transform.rotation.w = 1.
    node.tf_buffer.lookup_transform.side_effect = None
    node.tf_buffer.lookup_transform.return_value = transform
    assert node._navigation_input_error() is None


def test_input_fault_uses_cancel_barrier_then_retries_retained_goal_without_route_failure():
    node = healthy_follower()
    path, handle = start(node)
    retained = node.requested_goal
    node.pending_execution_failure = {'unconfirmed': True}
    node.latest_scan_at = time.monotonic()-2.
    node._watch_navigation_inputs()
    handle.cancel_goal_async.assert_called_once()
    assert node.follow_cancel_since is not None
    assert node.requested_goal is retained and node.path_invalidated
    assert node.pending_execution_failure is None
    assert statuses(node)[-1]['route_failure'] is False
    assert statuses(node)[-1]['input_hold'] is True
    node.latest_scan_at = time.monotonic()
    replacement = make_path(stamp=6)
    node.on_path(replacement)
    assert node.client.send_goal_async.call_count == 1
    node.on_result(result_future(), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    node.on_path(path)  # A delayed route computed before terminal stop is discarded.
    assert node.client.send_goal_async.call_count == 1
    node.on_path(replacement)
    assert node.client.send_goal_async.call_count == 2
    assert node.navigation_input_fault is None


def test_pending_acceptance_is_canceled_after_input_fault_and_timeout_keeps_safety_stop():
    node = healthy_follower()
    node.requested_goal = goal_for(make_path(), stamp=4)
    node.on_path(make_path(stamp=5))
    node.latest_costmap.data = []
    node._watch_navigation_inputs()
    assert node.cancel_when_accepted and node.goal_request_pending
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)))
    handle.cancel_goal_async.assert_called_once()
    with patch('smart_vision_amr.path_follower.time.monotonic',
               return_value=node.follow_cancel_since+3.1):
        node._monitor_follow_path()
    assert node.follow_estop_sent and node.navigation_suspended
    assert node.active_goal is handle


def test_terminal_success_wins_input_fault_cancellation_race():
    node = healthy_follower()
    _, handle = start(node)
    node.latest_scan_at = time.monotonic()-2.
    node._watch_navigation_inputs()
    node.status_pub.publish.reset_mock()
    node.on_result(result_future(GoalStatus.STATUS_SUCCEEDED), handle=handle)
    assert [status['status'] for status in statuses(node)] == ['SUCCEEDED']
    assert node.navigation_input_fault is None and node.navigation_input_cutoff_ns is None
    node.execution_failure_pub.publish.assert_not_called()


def test_rejected_pending_request_during_input_hold_is_not_a_route_failure():
    node = healthy_follower()
    node.requested_goal = goal_for(make_path(), stamp=4)
    node.on_path(make_path(stamp=5))
    node.latest_costmap.data = []
    node._watch_navigation_inputs()
    node.status_pub.publish.reset_mock()
    node.on_goal_response(Mock(result=Mock(return_value=Mock(accepted=False))))
    assert statuses(node)[-1]['status'] == 'WAIT_REPLAN'
    assert statuses(node)[-1]['route_failure'] is False
    assert not node.goal_request_pending and node.active_goal is None


def test_new_explicit_goal_resets_the_sensor_hold_path_cutoff():
    node = healthy_follower()
    _, handle = start(node)
    node.latest_scan_at = time.monotonic()-2.
    node._watch_navigation_inputs()
    node.on_result(result_future(), handle=handle)
    node.latest_scan_at = time.monotonic()
    path = make_path(stamp=6)
    node.on_goal(goal_for(path, stamp=6))
    assert node.navigation_input_cutoff_ns is None
    node.on_path(path)
    assert node.client.send_goal_async.call_count == 2


def test_recovery_also_requires_current_costmap_and_footprint():
    node = healthy_follower()
    node.latest_costmap.data = []
    assert 'local costmap' in node._recovery_preflight({}, require_stationary=False)


def test_repeated_input_rejections_rearm_the_planner_until_inputs_recover():
    from test_planner_recovery import PlannerHarness

    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        node.latest_costmap.data = []
        goal = goal_for(make_path(), stamp=4)
        planner = PlannerHarness()
        planner.last_goal = goal
        planner.geometry_signature = tuple(planner.params[name] for name in (
            'path_smoothing_enabled', 'path_smoothing_iterations',
            'path_smoothing_shortcut_distance', 'path_resample_spacing'))
        # Run the real status/retry callbacks; route computation is independent
        # of this handshake, so emulate its successful publication boundary.
        def publish_route(retained_goal):
            assert retained_goal is goal
            planner.waiting_for_replan = False
            planner.last_plan_time = time.monotonic()
            node.on_path(make_path(stamp=node.clock_seconds))
        planner.plan_to = Mock(side_effect=publish_route)
        node.status_pub.publish.side_effect = planner.on_navigation_status
        node.on_goal(goal)
        node.on_path(make_path(stamp=5))
        assert planner.waiting_for_replan
        for seconds, monotonic in ((6, 103.), (7, 106.), (8, 109.)):
            clock.return_value = monotonic
            node.clock_seconds = seconds
            for source in ('scan', 'odom', 'costmap', 'footprint'):
                getattr(node, 'latest_'+source).header.stamp.sec = seconds
                setattr(node, 'latest_'+source+'_at', monotonic)
            node.on_odom(node.latest_odom)
            node.on_footprint(node.latest_footprint)
            if seconds == 8:
                node.latest_costmap.data = [0]*3600
            planner.retry_transform()
            assert planner.waiting_for_replan is (seconds != 8)
        assert planner.plan_to.call_count == 3
        assert node.client.send_goal_async.call_count == 1
        assert node.requested_goal is goal and planner.last_goal is goal
        assert node.navigation_input_fault is None
        assert len(statuses(node)) == 3


def test_manual_stop_cannot_be_undone_by_healthy_inputs():
    node = healthy_follower()
    _, handle = start(node)
    node.cancel_navigation(Trigger.Request(), Trigger.Response())
    node.on_result(result_future(), handle=handle)
    node._watch_navigation_inputs()
    node.on_path(make_path(stamp=6))
    assert node.navigation_suspended
    assert node.client.send_goal_async.call_count == 1


def test_path_before_goal_waits_without_uncorrelated_sensor_hold_status():
    node = healthy_follower()
    node.latest_costmap.data = []
    path = make_path(stamp=5)
    node.on_path(path)
    node.status_pub.publish.assert_not_called()
    assert list(node.suspended_paths) == [path]
