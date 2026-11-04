"""A passed obstacle must not cancel motion; unknown turn space cannot reverse."""
from copy import deepcopy
from unittest.mock import Mock

from std_msgs.msg import Int32MultiArray

from smart_vision_amr.astar_planner import AStarPlannerNode
from test_planner_failure_memory import prepared, refresh, failure_event
from test_planner_obstacle_hold import hit_message, newly_blocking_cell, ready, visibility


def cell_message(node, x, y):
    return Int32MultiArray(data=[y*node.map[0].info.width+x])


def moved_halfway(node):
    refresh(node, x=1.55)
    node.path_pub.publish.reset_mock()
    node._request_replan = Mock()
    assert (6, 15) not in node._remaining_path_cells()
    assert (22, 15) in node._remaining_path_cells()


def test_new_live_obstacle_behind_robot_does_not_invalidate_remaining_route(prepared):
    node = prepared
    moved_halfway(node)
    node.on_observed(cell_message(node, 6, 15))
    node.path_pub.publish.assert_not_called()
    node._request_replan.assert_not_called()
    assert not node.waiting_for_replan


def test_new_live_obstacle_ahead_invalidates_and_requests_replan(prepared):
    node = prepared
    moved_halfway(node)
    node.on_observed(cell_message(node, 22, 15))
    assert not node.path_pub.publish.call_args.args[0].poses
    node._request_replan.assert_called_once()
    assert node.waiting_for_replan


def test_distant_single_scan_does_not_cancel_but_persistent_hit_does(prepared):
    node = prepared
    node.params['live_route_confirmation_scans'] = 3
    node.params['local_detour_enabled'] = False
    blocker = newly_blocking_cell(node, min_x=10)
    before = node.path_pub.publish.call_count
    for stamp in (100.1, 100.2):
        ready(node, stamp)
        node.on_observed(hit_message(node, {blocker}))
        node.on_visibility(visibility(node, occupied={blocker}))
        assert node.path_pub.publish.call_count == before
        assert not node.waiting_for_replan
    ready(node, 100.3)
    node.on_observed(hit_message(node, {blocker}))
    node.on_visibility(visibility(node, occupied={blocker}))
    assert node.waiting_for_replan
    assert not node.path_pub.publish.call_args.args[0].poses


def test_disappearing_distant_hit_resets_live_confirmation(prepared):
    node = prepared
    node.params['live_route_confirmation_scans'] = 3
    node.params['local_detour_enabled'] = False
    blocker = newly_blocking_cell(node, min_x=10)
    before = node.path_pub.publish.call_count
    for stamp, cells in ((100.1, {blocker}), (100.2, set()),
                         (100.3, {blocker}), (100.4, {blocker})):
        ready(node, stamp)
        node.on_observed(hit_message(node, cells))
        node.on_visibility(visibility(node, occupied=cells))
    assert node.path_pub.publish.call_count == before
    assert not node.waiting_for_replan


def test_close_live_hit_cancels_immediately_even_with_confirmation_enabled(prepared):
    node = prepared
    node.params['live_route_confirmation_scans'] = 3
    node.params['local_detour_enabled'] = False
    blocker = newly_blocking_cell(node, min_x=8)
    ready(node, 100.1)
    node.on_observed(hit_message(node, {blocker}))
    node.on_visibility(visibility(node, occupied={blocker}))
    assert node.waiting_for_replan
    assert not node.path_pub.publish.call_args.args[0].poses


def test_cached_overlay_repeated_without_new_scan_hits_does_not_cancel(prepared):
    node = prepared
    node.params['live_route_confirmation_scans'] = 3
    node.params['local_detour_enabled'] = False
    blocker = newly_blocking_cell(node, min_x=10)
    before = node.path_pub.publish.call_count
    for stamp in (100.1, 100.2, 100.3, 100.4):
        ready(node, stamp)
        node.on_observed(hit_message(node, {blocker}))  # TTL overlay repeats.
        node.on_visibility(visibility(node, occupied={blocker} if stamp == 100.1 else set()))
    assert node.path_pub.publish.call_count == before
    assert not node.waiting_for_replan


def test_remembered_obstacle_behind_robot_does_not_cancel_current_action(prepared):
    node = prepared
    moved_halfway(node)
    node.on_memory(cell_message(node, 6, 15))
    node.path_pub.publish.assert_not_called()
    node._request_replan.assert_not_called()


def test_cleared_remembered_obstacle_off_active_detour_keeps_current_sector_route(prepared):
    node = prepared
    goal = node.last_goal
    node.params['local_detour_enabled'] = False
    # The first observation blocks the direct remaining leg. The retained
    # goal is replanned around it after the normal retry throttle.
    node.on_memory(cell_message(node, 22, 15))
    assert node.waiting_for_replan
    node.clock.update(steady=102.1, ros=102.1)
    refresh(node, x=1.55)
    node.retry_transform()
    assert node.planned_cells and (22, 15) not in node.planned_cells
    assert not node.waiting_for_replan

    current_path = node.path_pub.publish.call_args.args[0]
    node.path_pub.publish.reset_mock()
    node._request_replan = Mock()
    node.on_memory(Int32MultiArray(data=[]))  # Direct free-space evidence arrived.

    # The old obstacle was already off the active detour. Clearing it cannot
    # cancel a safe FollowPath action just to seek a potentially shorter line.
    assert node.remembered == set()
    assert node.last_goal is goal
    assert current_path.poses
    node.path_pub.publish.assert_not_called()
    node._request_replan.assert_not_called()
    assert not node.waiting_for_replan


def test_cleared_memory_retries_retained_goal_when_obstacle_blocked_all_routes(prepared):
    node = prepared
    goal = node.last_goal
    width, height = node.map[0].info.width, node.map[0].info.height
    barrier = Int32MultiArray(data=[y * width + 15 for y in range(height)])
    node.on_memory(barrier)
    node.clock.update(steady=102.1, ros=102.1)
    refresh(node)
    node.retry_transform()
    assert node.waiting_for_replan
    assert node.planned_cells == []

    node.clock.update(steady=104.2, ros=104.2)
    refresh(node)
    node.on_memory(Int32MultiArray(data=[]))
    assert node.remembered == set()
    assert node.last_goal is goal
    assert node.planned_cells
    assert node.path_pub.publish.call_args.args[0].poses
    assert not node.waiting_for_replan


def test_map_change_behind_robot_may_replan_but_does_not_cancel(prepared):
    node = prepared
    moved_halfway(node)
    changed = deepcopy(node.map[0])
    changed.data[15*changed.info.width+6] = 100
    node.on_map(changed)
    node.path_pub.publish.assert_not_called()
    node._request_replan.assert_called_once()


def test_map_change_ahead_cancels_unsafe_remaining_route(prepared):
    node = prepared
    moved_halfway(node)
    changed = deepcopy(node.map[0])
    changed.data[15*changed.info.width+22] = 100
    node.on_map(changed)
    assert not node.path_pub.publish.call_args.args[0].poses
    node._request_replan.assert_called_once()


def test_unknown_turn_clearance_holds_without_requesting_reverse(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    assert node.failure_memory.snapshot(now=100.)['count'] == 1
    node._turn_ready = Mock(return_value=None)
    node._maybe_request_reverse = Mock()
    node.path_pub.publish.reset_mock()
    node.plan_to(node.last_goal)
    assert node.failure_state == 'SENSOR_HOLD'
    node._maybe_request_reverse.assert_not_called()
    assert not node.path_pub.publish.call_args.args[0].poses


def test_known_insufficient_turn_clearance_can_evaluate_verified_retreat(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    node._turn_ready = Mock(return_value=False)
    node._maybe_request_reverse = Mock(return_value=False)
    node.plan_to(node.last_goal)
    assert node.failure_state == 'WAIT_ALTERNATIVE'
    node._maybe_request_reverse.assert_called_once()
    assert not node.path_pub.publish.call_args.args[0].poses


def test_partial_scan_cannot_authorize_in_place_turn(prepared):
    node = prepared
    # The harness scan covers 5.9 rad: insufficient for a full sweep. Call the
    # production method directly because its normal harness stub is permissive.
    result = AStarPlannerNode._turn_ready(node, [(.55, 1.55), (.55, 2.55)], (.55, 1.55, 0.))
    assert result is None


def test_missing_or_stale_map_pose_uses_complete_route_for_safety(prepared):
    node = prepared
    moved_halfway(node)
    node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 98
    assert node._remaining_path_cells() == node.planned_cells
    node.on_observed(cell_message(node, 6, 15))
    assert not node.path_pub.publish.call_args.args[0].poses
