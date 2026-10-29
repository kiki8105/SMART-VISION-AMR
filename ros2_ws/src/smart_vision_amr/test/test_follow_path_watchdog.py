"""Lost Nav2 action responses must never start a second uncontrolled action."""
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus

from test_path_follower_cancel import FollowerHarness, make_path, result_future


def accepted_follow(node):
    node.on_path(make_path())
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)), node.request_context)
    return handle


def test_goal_response_timeout_retains_barrier_and_requests_confirmed_safety_stop():
    node = FollowerHarness()
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=10.):
        node.on_path(make_path())
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=16.):
        node._monitor_follow_path()
    assert node.goal_request_pending
    assert node.navigation_suspended
    assert node.follow_estop_sent
    assert json.loads(node.safety_control_pub.publish.call_args.args[0].data)['mode'] == 'estop'
    node.on_path(make_path(1.))
    assert node.client.send_goal_async.call_count == 1
    # A late acceptance is canceled; the old action remains a barrier until its result.
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)))
    handle.cancel_goal_async.assert_called_once()
    assert node.active_goal is handle


def test_feedback_timeout_waits_for_terminal_result_and_replans_after_confirmed_stop():
    node = FollowerHarness()
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=10.):
        handle = accepted_follow(node)
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=16.):
        node._monitor_follow_path()
    assert node.cancel_requested and not node.follow_estop_sent
    assert node.active_goal is handle
    node.on_result(result_future())
    assert node.active_goal is None
    assert not node.navigation_suspended
    statuses = [json.loads(call.args[0].data)['status'] for call in node.status_pub.publish.call_args_list]
    assert statuses[-1] == 'FAILED'  # Planner retains the goal and retries from the stopped pose.
    assert node.speed_limit_pub.publish.call_args.args[0].speed_limit == 0.


def test_cancel_ack_without_terminal_result_retries_then_latches_stop():
    node = FollowerHarness()
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=10.):
        handle = accepted_follow(node)
        node._cancel_active_goal()
    response = SimpleNamespace(goals_canceling=[object()])
    node._on_follow_cancel_response(Mock(result=Mock(return_value=response)), handle)
    assert node.follow_cancel_acknowledged
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=11.):
        node._monitor_follow_path()
    assert handle.cancel_goal_async.call_count == 2
    assert node.active_goal is handle
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=13.1):
        node._monitor_follow_path()
    assert node.follow_estop_sent and node.navigation_suspended
    assert node.active_goal is handle


def test_result_transport_exception_cannot_release_handle_or_start_pending_route():
    node = FollowerHarness()
    handle = accepted_follow(node)
    node.pending_path = make_path(1.)
    node.on_result(Mock(result=Mock(side_effect=RuntimeError('lost result'))), handle=handle)
    assert node.active_goal is handle and node.navigation_suspended
    assert node.follow_estop_sent
    assert node.client.send_goal_async.call_count == 1


def test_delayed_old_result_and_feedback_do_not_clear_new_action_or_watchdog():
    node = FollowerHarness()
    old = accepted_follow(node)
    node.on_result(result_future())
    new = accepted_follow(node)
    node.on_result(result_future(GoalStatus.STATUS_SUCCEEDED), handle=old)
    assert node.active_goal is new
    node.follow_feedback_at = 20.
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=1., speed=.1)),
                     node.follow_request_seq-1)
    assert node.follow_feedback_at == 20.


def test_valid_feedback_refreshes_watchdog_and_reports_curve_limit():
    node = FollowerHarness()
    accepted_follow(node)
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=20.):
        node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=1., speed=.1)))
    assert node.follow_feedback_at == 20.
    assert 0. <= node.speed_limit_pub.publish.call_args.args[0].speed_limit <= .25
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['controller_distance_remaining'] == 1.


def test_zero_controller_remaining_distance_cannot_release_upcoming_corner_limit():
    node = FollowerHarness()
    path = make_path()
    path.poses[2].pose.position.x = 1.
    path.poses[2].pose.position.y = 1.
    node.on_path(path)
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)))
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=0., speed=.1)))
    # No map pose: apply the conservative whole-route curve cap.
    assert node.follow_path_progress is None
    assert 0. < node.follow_speed_limit < .25


def test_fresh_map_pose_advances_curve_preview_and_stale_pose_restores_conservative_cap():
    from geometry_msgs.msg import PoseStamped, TransformStamped
    node = FollowerHarness()
    node.parameters.update({'curve_base_frame': 'base_footprint', 'curve_pose_timeout': 1.})
    path = make_path()
    path.poses[2].pose.position.x = 1.
    path.poses[2].pose.position.y = 1.
    for y in (2., 3.):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = 1., y
        path.poses.append(pose)
    transform = TransformStamped()
    transform.header.stamp.sec = node.clock_seconds
    transform.transform.translation.x, transform.transform.translation.y = 1., 2.5
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = transform
    node.on_path(path)
    assert node.follow_path_progress == 3.5
    assert node.follow_speed_limit == 0.  # Explicit Nav2 restore on the straight segment.
    transform.header.stamp.sec = node.clock_seconds-2
    node._publish_curve_limit()
    assert node.follow_path_progress is None
    assert node.follow_speed_limit < .25


def test_dwb_absolute_limit_above_base_is_explicitly_reset_not_ignored():
    node = FollowerHarness()
    node.controller_base_speed = .12
    node.on_path(make_path())
    assert node.follow_speed_limit == 0.
    node._publish_speed_limit(.04)
    node._publish_curve_limit()
    assert node.speed_limit_pub.publish.call_args.args[0].speed_limit == 0.


def test_controller_maximum_query_and_runtime_parameter_update_are_nonblocking():
    from rcl_interfaces.msg import ParameterEvent, Parameter, ParameterValue, ParameterType
    node = FollowerHarness()
    node.parameters['controller_id'] = 'FollowPath'
    node.controller_speed_client = Mock()
    node.controller_speed_client.service_is_ready.return_value = True
    node.controller_speed_future = None
    node.controller_speed_requested_at = None
    node.controller_speed_event_at = float('-inf')
    node.controller_speed_next_request = float('-inf')
    future = Mock()
    future.done.return_value = False
    node.controller_speed_client.call_async.return_value = future
    node._update_controller_speed(10.)
    assert node.controller_speed_client.call_async.call_count == 1
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(values=[
        ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=.22),
        ParameterValue(type=ParameterType.PARAMETER_NOT_SET)])
    node._update_controller_speed(10.1)
    assert node.controller_base_speed == .22
    node.on_controller_parameters(ParameterEvent(node='/controller_server', changed_parameters=[
        Parameter(name='FollowPath.max_speed_xy', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=.12))]))
    assert node.controller_base_speed == .12
