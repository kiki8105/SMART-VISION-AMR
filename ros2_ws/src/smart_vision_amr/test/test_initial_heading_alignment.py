"""A large initial route turn must finish before Nav2 can drive forward."""

import json
import time
from math import cos, pi, sin
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from test_path_follower_cancel import FollowerHarness, goal_for, make_path, result_future


def south_path():
    path = make_path()
    for index, pose in enumerate(path.poses):
        pose.pose.position.x = 0.
        pose.pose.position.y = -.4*index
    return path


def setup_node():
    node = FollowerHarness()
    node.parameters.update({'initial_heading_alignment_enabled': True,
                            'initial_heading_alignment_prune_distance': .40,
                            'curve_base_frame': 'base_footprint'})
    tf = TransformStamped()
    tf.header.stamp.sec = node.clock_seconds
    tf.transform.rotation.w = 1.
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = tf
    return node, tf


def accept_spin(node):
    handle = Mock(accepted=True)
    state = node.initial_alignment
    node._on_initial_alignment_response(Mock(result=Mock(return_value=handle)), state)
    return handle, state


def test_initial_90_degree_route_uses_collision_checked_spin_before_follow_path():
    node, tf = setup_node()
    path = south_path()

    node.on_path(path)

    node.spin_client.send_goal_async.assert_called_once()
    spin_goal = node.spin_client.send_goal_async.call_args.args[0]
    assert abs(spin_goal.target_yaw + pi/2) < .01
    assert 12 <= spin_goal.time_allowance.sec <= 20
    node.client.send_goal_async.assert_not_called()
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['status'] == 'MOVING'

    handle, state = accept_spin(node)
    tf.transform.rotation.z, tf.transform.rotation.w = sin(-pi/4), cos(-pi/4)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_SUCCEEDED), state)

    assert node.initial_alignment is None
    node.client.send_goal_async.assert_called_once()
    assert node.client.send_goal_async.call_args.args[0].path is path
    handle.cancel_goal_async.assert_not_called()


def test_new_prefix_for_same_goal_waits_for_previous_follow_terminal():
    node, tf = setup_node()
    node.on_path(south_path())
    _, state = accept_spin(node)
    tf.transform.rotation.z, tf.transform.rotation.w = sin(-pi/4), cos(-pi/4)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_SUCCEEDED), state)
    assert node._route_alignment_reference['generation'] == node.goal_generation

    # A direct dispatch cannot bypass the old request's real terminal barrier.
    # The material-prefix policy subsequently obtains fresh measured stop
    # evidence; it no longer skips every detour solely because a goal began.
    replacement = make_path()
    node._send_path(replacement)
    assert node.spin_client.send_goal_async.call_count == 1
    assert node.client.send_goal_async.call_count == 1
    assert node.pending_path is replacement
    assert node.cancel_when_accepted


def test_tight_curvature_has_nonzero_nav2_floor_before_turn():
    node, tf = setup_node()
    node.parameters.update({'curve_min_linear_speed_limit': .12,
                            'curve_max_linear_velocity': .24,
                            'curve_max_angular_velocity': .35,
                            'curve_max_lateral_acceleration': .10,
                            'curve_preview_distance': 1.,
                            'curve_alignment_angle': 10.})
    path = make_path()
    for pose, (x, y) in zip(path.poses, ((0., 0.), (.05, .05), (.1, 0.))):
        pose.pose.position.x, pose.pose.position.y = x, y
    node.active_path = path
    node.controller_base_speed = .24
    node._publish_curve_limit()
    msg = node.speed_limit_pub.publish.call_args.args[0]
    assert msg.speed_limit == .12
    assert not msg.percentage


def test_manual_stop_cancels_unaccepted_spin_and_cannot_start_follow_path():
    node, _ = setup_node()
    node.on_path(south_path())
    state = node.initial_alignment

    response = node.cancel_navigation(Trigger.Request(), Trigger.Response())
    assert not response.success
    handle, _ = accept_spin(node)
    handle.cancel_goal_async.assert_called_once()
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)

    assert node.initial_alignment is None
    assert node.navigation_suspended
    assert node.cancel_navigation(Trigger.Request(), Trigger.Response()).success
    node.client.send_goal_async.assert_not_called()


def test_replan_during_spin_waits_for_terminal_cancel_without_another_spin():
    node, _ = setup_node()
    node.motion_status = {'mode': 'auto', 'available': True,
                          'autonomous_allowed': True, 'estop_latched': False}
    node.motion_status_at = time.monotonic()
    node.on_path(south_path())
    handle, state = accept_spin(node)

    replacement = make_path()
    node.on_path(replacement)
    handle.cancel_goal_async.assert_called_once()
    node.client.send_goal_async.assert_not_called()
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)

    assert node.initial_alignment_total_starts == 1
    node.client.send_goal_async.assert_called_once()
    assert node.client.send_goal_async.call_args.args[0].path is replacement


def test_volatile_prefix_changes_cannot_bypass_total_spin_start_budget():
    node, _ = setup_node()
    node.motion_status = {'mode': 'auto', 'available': True,
                          'autonomous_allowed': True, 'estop_latched': False}
    node.motion_status_at = time.monotonic()
    south = south_path()
    north = south_path()
    for point in north.poses:
        point.pose.position.y *= -1
    node.on_path(south)
    _, first = accept_spin(node)
    node.on_path(north)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), first)
    assert node.initial_alignment_total_starts == 2
    _, second = accept_spin(node)
    node.on_path(south)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), second)

    assert node.initial_alignment_total_starts == 2  # Physical starts never get refunded.
    assert node.initial_alignment is None
    assert node.navigation_suspended
    assert node.pending_path is None
    assert node.spin_client.send_goal_async.call_count == 2
    node.client.send_goal_async.assert_not_called()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['status'] == 'CONTROLLER_HOLD'
    assert status['initial_alignment_starts'] == status['initial_alignment_start_limit'] == 2
    node.on_path(north)
    node._send_path(north)
    assert node.spin_client.send_goal_async.call_count == 2

    # Only deliberate dispatch of a new, stamped goal grants another budget.
    resumed = south_path()
    resumed.header.stamp.sec = 7
    node.on_goal(goal_for(resumed, stamp=6))
    node.on_path(resumed)
    assert not node.navigation_suspended
    assert node.initial_alignment_total_starts == 1
    assert node.spin_client.send_goal_async.call_count == 3


def test_two_aborted_initial_spins_require_explicit_resume():
    node, _ = setup_node()
    for _ in range(2):
        node.on_path(south_path())
        _, state = accept_spin(node)
        node._on_initial_alignment_result(result_future(GoalStatus.STATUS_ABORTED), state)
    node.on_path(south_path())
    assert node.spin_client.send_goal_async.call_count == 2
    assert node.navigation_suspended
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['status'] == 'CONTROLLER_HOLD'


def test_stale_map_pose_blocks_departure_when_navigation_inputs_are_required():
    node, tf = setup_node()
    node.navigation_inputs_required = True
    node._navigation_input_error = Mock(return_value=None)
    tf.header.stamp.sec = node.clock_seconds-2

    node.on_path(south_path())

    node.client.send_goal_async.assert_not_called()
    node.spin_client.send_goal_async.assert_not_called()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['status'] == 'WAIT_REPLAN'
    assert 'map pose' in status['message']


def test_cross_track_offset_inside_dwb_prune_distance_uses_follow_path_without_spin():
    node, tf = setup_node()
    node.navigation_inputs_required = True
    node._navigation_input_error = Mock(return_value=None)
    tf.transform.translation.y = .27

    node.on_path(make_path())

    node.spin_client.send_goal_async.assert_not_called()
    node.client.send_goal_async.assert_called_once()


def test_cross_track_offset_outside_dwb_prune_distance_still_blocks_departure():
    node, tf = setup_node()
    node.navigation_inputs_required = True
    node._navigation_input_error = Mock(return_value=None)
    tf.transform.translation.y = .41

    node.on_path(make_path())

    node.spin_client.send_goal_async.assert_not_called()
    node.client.send_goal_async.assert_not_called()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['status'] == 'WAIT_REPLAN'


def test_incomplete_spin_never_forwards_to_follow_path():
    node, _ = setup_node()
    node.on_path(south_path())
    _, state = accept_spin(node)

    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_ABORTED), state)

    node.client.send_goal_async.assert_not_called()
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['status'] == 'WAIT_REPLAN'


def test_replan_start_on_later_path_segment_still_checks_heading():
    node, tf = setup_node()
    tf.transform.translation.x = .5
    tf.transform.rotation.z, tf.transform.rotation.w = sin(pi/4), cos(pi/4)

    node.on_path(make_path())

    node.spin_client.send_goal_async.assert_called_once()
    node.client.send_goal_async.assert_not_called()


def test_old_auto_status_cannot_restart_route_after_spin_cancel():
    node, _ = setup_node()
    node.motion_status = {'mode': 'auto', 'available': True,
                          'autonomous_allowed': True, 'estop_latched': False}
    node.motion_status_at = time.monotonic()-2.
    node.on_path(south_path())
    _, state = accept_spin(node)
    node.on_path(make_path())
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)

    node.client.send_goal_async.assert_not_called()
    assert node.pending_path is None


def test_planner_empty_path_cancels_spin_and_does_not_restart():
    node, _ = setup_node()
    node.on_path(south_path())
    handle, state = accept_spin(node)

    node.on_path(Path())
    handle.cancel_goal_async.assert_called_once()
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)

    node.client.send_goal_async.assert_not_called()
    assert node.pending_path is None


def test_confirmed_spin_stop_clears_its_timeout_watch_but_keeps_explicit_resume_barrier():
    node, _ = setup_node()
    node.on_path(south_path())
    state = node.initial_alignment
    state['requested_at'] -= 6.
    node._monitor_initial_alignment()
    assert node.follow_estop_sent and node.navigation_suspended
    accept_spin(node)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)

    assert node.initial_alignment is None
    assert node.follow_watchdog_fault is None
    assert not node.follow_estop_sent
    assert node.navigation_suspended
    assert node.pending_path is None
    node.client.send_goal_async.assert_not_called()
    # No reset was sent to the physical mux; its latch still needs the user.
    assert all(json.loads(call.args[0].data)['mode'] == 'estop'
               for call in node.safety_control_pub.publish.call_args_list)

    resumed = south_path()
    resumed.header.stamp.sec = 7
    node.on_goal(goal_for(resumed, stamp=6))
    node.on_path(resumed)
    assert not node.navigation_suspended
    assert node.spin_client.send_goal_async.call_count == 2


def test_rejected_spin_after_acceptance_timeout_also_releases_owned_watch():
    node, _ = setup_node()
    node.on_path(south_path())
    state = node.initial_alignment
    state['requested_at'] -= 6.
    node._monitor_initial_alignment()
    node._on_initial_alignment_response(
        Mock(result=Mock(return_value=Mock(accepted=False))), state)
    assert node.initial_alignment is None
    assert node.follow_watchdog_fault is None
    assert not node.follow_estop_sent
    assert node.navigation_suspended
    node.client.send_goal_async.assert_not_called()


def test_confirmed_spin_stop_cannot_clear_another_followpath_watch_fault():
    node, _ = setup_node()
    node.on_path(south_path())
    _, state = accept_spin(node)
    node._hold_unconfirmed_initial_alignment(state, 'Spin cancellation/result timed out')
    node.follow_watchdog_fault = 'FollowPath feedback timed out'
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)
    assert node.follow_watchdog_fault == 'FollowPath feedback timed out'
    assert node.follow_estop_sent
    assert node.navigation_suspended


def test_spin_result_must_be_terminal_before_releasing_its_watch():
    node, _ = setup_node()
    node.on_path(south_path())
    _, state = accept_spin(node)
    node._hold_unconfirmed_initial_alignment(state, 'Spin cancellation/result timed out')
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_EXECUTING), state)
    assert node.initial_alignment is state
    assert node.follow_estop_sent
    assert state['watchdog_fault'].startswith('Spin result unconfirmed:')
    assert node.follow_watchdog_fault == state['watchdog_fault']
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), state)
    assert node.follow_watchdog_fault is None
    assert node.navigation_suspended


def test_successful_spin_after_safety_timeout_does_not_dispatch_follow_path():
    node, tf = setup_node()
    node.on_path(south_path())
    _, state = accept_spin(node)
    node._hold_unconfirmed_initial_alignment(state, 'Spin cancellation/result timed out')
    tf.transform.rotation.z, tf.transform.rotation.w = sin(-pi/4), cos(-pi/4)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_SUCCEEDED), state)
    assert node.follow_watchdog_fault is None
    assert not node.follow_estop_sent
    assert node.navigation_suspended
    node.client.send_goal_async.assert_not_called()


def test_old_spin_callback_cannot_release_current_spin_watch():
    node, _ = setup_node()
    node.on_path(south_path())
    _, old_state = accept_spin(node)
    node._hold_unconfirmed_initial_alignment(old_state, 'Spin cancellation/result timed out')
    new_state = dict(old_state)
    new_state['watchdog_fault'] = 'new Spin fault'
    node.initial_alignment = new_state
    node.follow_watchdog_fault = 'new Spin fault'
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_CANCELED), old_state)
    assert node.initial_alignment is new_state
    assert node.follow_watchdog_fault == 'new Spin fault'
    assert node.follow_estop_sent
