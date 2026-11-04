"""Remaining route geometry is planned; actual rotation belongs to the follower."""
from copy import deepcopy
from unittest.mock import Mock

from std_msgs.msg import Int32MultiArray

from smart_vision_amr.astar_planner import AStarPlannerNode
from smart_vision_amr.recovery_episode import RecoveryPhase
from test_planner_failure_memory import prepared, refresh, failure_event
from test_planner_obstacle_hold import hit_message, newly_blocking_cell, ready, visibility
from recovery_test_evidence import stationary_evidence


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


def test_new_live_obstacle_ahead_invalidates_before_any_new_plan(prepared):
    node = prepared
    moved_halfway(node)
    node.on_observed(cell_message(node, 22, 15))
    assert not node.path_pub.publish.call_args.args[0].poses
    # The canceled route must first yield stop/reverse evidence. An obstacle
    # callback cannot bypass that phase with an immediate global request.
    node._request_replan.assert_not_called()
    assert node.waiting_for_replan


def test_distant_single_scan_does_not_cancel_but_persistent_hit_does(prepared):
    node = prepared
    node.params['live_route_confirmation_scans'] = 3
    node.params['local_detour_enabled'] = False
    blocker = newly_blocking_cell(node, min_x=10)
    events = []
    def trace(name, call):
        def wrapped(*args, **kwargs):
            events.append(name)
            return call(*args, **kwargs)
        return wrapped
    for method, label in (('_try_local_detour', 'local'),
                          ('_maybe_request_reverse', 'reverse'), ('plan_to', 'global')):
        setattr(node, method, trace(label, getattr(node, method)))
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
    # This fixture already has fresh stationary source evidence. A proved
    # unavailable retreat may advance to a safe replacement immediately; it
    # need not remain artificially waiting until another timer tick.
    assert events == ['local', 'reverse', 'global']
    publications = node.path_pub.publish.call_args_list[before:]
    assert not publications[0].args[0].poses
    assert publications[-1].args[0].poses
    assert not node.recovery_active and not node.waiting_for_replan
    assert node.recovery_coordinator.current.phase is RecoveryPhase.FOLLOW


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
    # Local repair is disabled here. Only after actual stamped stillness and
    # a rejected reverse verdict (there is no traversed trail) may A* replace it.
    # Keep the actual 22 cm full-body mask clear of the final goal. The former
    # inflation=0 memory mask admitted this same point only 30 cm from it.
    node.on_memory(cell_message(node, 18, 15))
    assert node.waiting_for_replan
    stationary_evidence(node, 102.1, x=.55, y=1.55)
    node.retry_transform()
    assert node.planned_cells and (18, 15) not in node.planned_cells
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
    stationary_evidence(node, 102.1, x=.55, y=1.55)
    node.retry_transform()
    assert node.waiting_for_replan
    assert node.planned_cells == []

    stationary_evidence(node, 104.2, x=.55, y=1.55)
    # The original scan has now also produced its stamped visibility event.
    node.on_visibility(visibility(node))
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


def test_unknown_rotation_space_does_not_create_a_second_planner_dispatch_policy(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    assert node.failure_memory.snapshot(now=100.)['count'] == 1
    # An unused compatibility attribute must not turn a route planner into a
    # second Spin controller. This does not claim that Nav2 will accept a turn.
    node._turn_ready = Mock(return_value=None)
    node._maybe_request_reverse = Mock()
    node.path_pub.publish.reset_mock()
    node.plan_to(node.last_goal)
    node._turn_ready.assert_not_called()
    node._maybe_request_reverse.assert_not_called()
    assert node.path_pub.publish.call_args.args[0].poses
    assert all(not node.map[1].is_static_obstacle(c) for c in node.planned_cells)


def test_old_rotation_veto_cannot_redirect_a_free_route_into_reverse(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    node._turn_ready = Mock(return_value=False)
    node._maybe_request_reverse = Mock(return_value=False)
    node.plan_to(node.last_goal)
    node._turn_ready.assert_not_called()
    node._maybe_request_reverse.assert_not_called()
    assert node.path_pub.publish.call_args.args[0].poses
    assert node.last_goal is not None


def test_partial_scan_does_not_veto_map_route_or_authorize_spin(prepared):
    node = prepared
    # This valid5.9-rad input is enough for the planner's fresh sensor check,
    # but no planner callback claims whole-circle Spin collision clearance.
    assert (len(node.latest_scan.ranges)-1)*node.latest_scan.angle_increment < 6.0
    assert not hasattr(AStarPlannerNode, '_turn_ready')
    node.on_execution_failure(failure_event(node))
    assert node.path_pub.publish.call_args.args[0].poses
    assert node.last_goal is not None


def test_missing_or_stale_map_pose_uses_complete_route_for_safety(prepared):
    node = prepared
    moved_halfway(node)
    node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 98
    assert node._remaining_path_cells() == node.planned_cells
    node.on_observed(cell_message(node, 6, 15))
    assert not node.path_pub.publish.call_args.args[0].poses
