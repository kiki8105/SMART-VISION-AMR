"""A passed obstacle must not cancel motion; unknown turn space cannot reverse."""
from copy import deepcopy
from unittest.mock import Mock

from std_msgs.msg import Int32MultiArray

from smart_vision_amr.astar_planner import AStarPlannerNode
from test_planner_failure_memory import prepared, refresh, failure_event


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


def test_remembered_obstacle_behind_robot_does_not_cancel_current_action(prepared):
    node = prepared
    moved_halfway(node)
    node.on_memory(cell_message(node, 6, 15))
    node.path_pub.publish.assert_not_called()
    node._request_replan.assert_not_called()


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
