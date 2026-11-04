"""Independent real selector -> side route -> owned Spin/FollowPath handoff.

The map and measured/memory squares are recorded. Travel, fresh stationary
sensor evidence, exhaustion of the first local share and action callbacks are
controlled test evidence, not a historical or physical-robot success claim.
The fixture explicitly allows 100 ms and reserves 90 ms after the controlled
first share; this functional handoff is not the production 60 ms timing test.
No ROS context, motor publisher, service or action server is started.
"""

from copy import deepcopy
from math import degrees, pi
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from smart_vision_amr.map_utils import world_to_cell
from smart_vision_amr.recovery_episode import RecoveryPhase

from body_geometry_oracle import continuous_raw_body_gap
from test_local_detour_planner import ready
from test_metric_connector import recorded
from test_path_follower_cancel import goal_for, result_future
from test_sharp_replan_alignment_independent import (
    confirm_stop, finish_spin, rig,
)


def _rear_rejected_side_query(node, pose, *, body_blocked=False):
    """Supply an actually received odom trail, then invoke the real rear probe."""
    yaw = -pi/2  # Rear is north; the recorded side opening is east.
    for index in range(13):
        ready(node, at=100.1+.05*index, x=pose[0], y=pose[1]+.3-.025*index,
              yaw=yaw, scan_frame=node.params['base_frame'])
    for index in range(12):
        ready(node, at=100.75+.05*index, x=pose[0], y=pose[1],
              yaw=yaw, scan_frame=node.params['base_frame'])
    node._record_start_visibility(node.get_clock().now().nanoseconds, node.clock['steady'])
    node.last_observation_time = node.clock['steady']
    assert node._reverse_start_stopped() and node._record_breadcrumb()
    assert len(node.breadcrumbs) == 13

    # Recorded rear squares really intersect a 30 cm straight reverse sweep.
    # This is not a missing-history mock masquerading as a blocked rear.
    still = [tuple(pose[:2]), tuple(pose[:2])]
    assert continuous_raw_body_gap(node, still) > .02
    assert continuous_raw_body_gap(node, [tuple(pose[:2]), (pose[0], pose[1]+.3)]) < 0.
    if body_blocked:
        node.observed.add(world_to_cell(node.map[0].info, *pose[:2]))
        assert continuous_raw_body_gap(node, still) < 0.

    blockers = node.observed | node.remembered
    assert node.recovery_coordinator.begin('independent-side-opening', node._local_route_key(),
                                          blockers, node.observed, node.remembered)
    # The initial bounded local availability is controlled. The second query
    # must consume its remaining share once, without refunding an attempt.
    assert node.recovery_coordinator.local_result(False)
    node.params['local_detour_max_planning_time'] = .10
    node.local_detour_attempts = node.local_attempt_id = 1
    node.metric_reverse_budget = {'key': node._local_route_key(),
                                  'episode_id': node.recovery_coordinator.current.episode_id,
                                  'seconds': .09, 'expansions': 1500}
    original_reference = deepcopy(node.local_reference_points)
    old = Path()
    old.header.frame_id, old.header.stamp.sec = 'map', 100
    for x, y in original_reference:
        item = PoseStamped()
        item.pose.position.x, item.pose.position.y = x, y
        item.pose.orientation.w = 1.
        old.poses.append(item)
    old.poses[-1].pose.orientation = deepcopy(node.last_goal.pose.orientation)
    node.plan_to = Mock(wraps=node.plan_to)
    node.recovery_pub.publish.reset_mock()
    node.path_pub.publish.reset_mock()
    assert node._queue_reverse_before_global(blockers, 'First local share exhausted'), node.plan_metrics
    node._independent_reverse_query_debug = {
        'metrics': deepcopy(node.plan_metrics),
        'remaining_budget': deepcopy(node.metric_reverse_budget),
        'episode': node.recovery_coordinator.snapshot(),
    }
    assert node.last_reverse_rejection_code == 'NO_SAFE_EGRESS', node._independent_reverse_query_debug
    node.recovery_pub.publish.assert_not_called()
    assert node.metric_reverse_budget is None
    assert node.local_detour_attempts == node.local_attempt_id == 1
    return old, [call.args[0] for call in node.path_pub.publish.call_args_list]


def _follower(monkeypatch, source, pose):
    node, clock, set_pose, sample = rig.__wrapped__(monkeypatch)
    clock['source'] = source
    sample(.01)
    set_pose(pose[0], pose[1], -pi/2)
    return node, set_pose, sample


def test_rejected_real_rear_uses_remaining_side_budget_then_stop_spin_and_follow(recorded, monkeypatch):
    planner, pose = recorded
    old, published = _rear_rejected_side_query(planner, pose)
    assert [bool(path.poses) for path in published] == [False, True], planner._independent_reverse_query_debug
    replacement = published[-1]
    assert planner.plan_metrics['plan_kind'] == 'LOCAL_METRIC_CONNECTOR'
    assert planner.plan_metrics['recovery_stage'] == 'side_connector_after_rear_rejected', planner._independent_reverse_query_debug
    assert planner.plan_metrics['metric_connector_side_only'] is True
    assert planner.recovery_coordinator.current.phase is RecoveryPhase.FOLLOW
    assert planner.recovery_coordinator.current.path_stamp_ns == planner.current_path_stamp_ns
    planner.plan_to.assert_not_called()
    assert continuous_raw_body_gap(planner, planner.planned_world_points) >= .02-1e-8

    follower, set_pose, sample = _follower(monkeypatch, planner.clock['ros'], pose)
    follower.on_goal(goal_for(old, stamp=99))
    follower._dispatch_follow_path(old)
    owned = follower.request_context
    handle = Mock(accepted=True)
    follower.on_goal_response(Mock(result=Mock(return_value=handle)), owned)
    for path in published:
        follower.on_path(path)
    assert 80 < abs(degrees(follower._initial_heading_error(replacement))) < 110
    assert follower.pending_path is replacement
    assert follower.client.send_goal_async.call_count == 1
    follower.spin_client.send_goal_async.assert_not_called()
    # A cancel acknowledgement plus zero odometry is not an action terminal.
    follower._on_follow_cancel_response(Mock(result=Mock(return_value=
        SimpleNamespace(goals_canceling=[object()]))), handle)
    confirm_stop(follower, sample)
    follower.spin_client.send_goal_async.assert_not_called()
    follower.on_result(result_future(GoalStatus.STATUS_CANCELED), owned, handle)
    follower.spin_client.send_goal_async.assert_not_called()
    confirm_stop(follower, sample)
    assert follower.initial_alignment['kind'] == 'material_replan'
    assert follower.spin_client.send_goal_async.call_count == 1
    assert follower.client.send_goal_async.call_count == 1
    state, _ = finish_spin(follower, set_pose, replacement)
    assert state['context']['path_stamp'] == {
        'sec': replacement.header.stamp.sec, 'nanosec': replacement.header.stamp.nanosec}
    assert follower.client.send_goal_async.call_count == 2
    assert follower.client.send_goal_async.call_args.args[0].path is replacement
    assert follower.request_context['generation'] == owned['generation']


def test_actual_body_blockage_produces_no_side_route_spin_or_backup(recorded, monkeypatch):
    planner, pose = recorded
    old, published = _rear_rejected_side_query(planner, pose, body_blocked=True)
    assert published and not any(path.poses for path in published)
    assert not planner.planned_world_points
    assert planner.recovery_coordinator.current.phase is RecoveryPhase.WAIT_OBSTACLE
    follower, _, sample = _follower(monkeypatch, planner.clock['ros'], pose)
    follower.on_goal(goal_for(old, stamp=99))
    follower._dispatch_follow_path(old)
    owned = follower.request_context
    handle = Mock(accepted=True)
    follower.on_goal_response(Mock(result=Mock(return_value=handle)), owned)
    for path in published:
        follower.on_path(path)
    confirm_stop(follower, sample)
    follower.on_result(result_future(GoalStatus.STATUS_CANCELED), owned, handle)
    confirm_stop(follower, sample)
    follower.spin_client.send_goal_async.assert_not_called()
    assert follower.client.send_goal_async.call_count == 1
    planner.recovery_pub.publish.assert_not_called()
