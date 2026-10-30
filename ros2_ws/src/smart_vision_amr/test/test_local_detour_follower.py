"""A real local patch keeps the existing Nav2 cancel/terminal safety barrier.

All action handles and publishers are mocks; no ROS context, Nav2 or robot runs.
"""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from test_local_detour_planner import planner_factory, patch, paths, ready
from test_path_follower_cancel import FollowerHarness, result_future


def active_follower(node):
    original = Path()
    original.header.frame_id = 'map'
    original.header.stamp = node.last_goal.header.stamp
    for x, y in node.planned_world_points:
        pose = PoseStamped()
        pose.header = original.header
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.
        original.poses.append(pose)
    original.poses[-1].pose.orientation = deepcopy(node.last_goal.pose.orientation)
    follower = FollowerHarness()
    follower.clock_seconds = 100
    follower.on_goal(deepcopy(node.last_goal))
    follower.active_path = original
    follower.active_context = follower._goal_context(original)
    follower.latest_path_key = follower._path_key(original)
    handle = follower.active_goal = Mock()
    return follower, handle


def test_nonempty_local_patch_waits_for_terminal_result_after_cancel_ack(planner_factory):
    node = planner_factory()
    follower, handle = active_follower(node)
    patch(node)
    replacement = paths(node)[-1]
    follower.on_path(replacement)
    handle.cancel_goal_async.assert_called_once_with()
    assert follower.pending_path is replacement
    follower.client.send_goal_async.assert_not_called()
    follower._on_follow_cancel_response(
        Mock(result=Mock(return_value=SimpleNamespace(goals_canceling=[object()]))), handle)
    assert follower.follow_cancel_acknowledged
    follower._send_pending_path()
    follower.client.send_goal_async.assert_not_called()
    ready(node, 100.2)
    follower.on_result(result_future())
    follower.client.send_goal_async.assert_called_once()
    sent = follower.client.send_goal_async.call_args.args[0]
    assert sent.path == replacement
    assert sent.path.poses[-1].pose.orientation == node.last_goal.pose.orientation
    assert follower.follow_requested_at == 100.2, 'no mandatory three-second wait after a confirmed terminal result'


def test_manual_cancel_during_patch_handoff_clears_patch_and_never_auto_restarts(planner_factory):
    node = planner_factory()
    follower, handle = active_follower(node)
    patch(node)
    replacement = paths(node)[-1]
    follower.on_path(replacement)
    follower.cancel_navigation(Trigger.Request(), Trigger.Response())
    assert follower.navigation_suspended
    assert follower.pending_path is None
    follower.on_result(result_future())
    follower.on_path(replacement)
    follower._send_pending_path()
    follower.client.send_goal_async.assert_not_called()


def test_second_unsafe_observation_cancels_pending_patch_before_it_can_start(planner_factory):
    node = planner_factory()
    follower, _ = active_follower(node)
    patch(node)
    follower.on_path(paths(node)[-1])
    next_cell = next((int(x/.05), int(y/.05)) for x, y in node.planned_world_points
                     if .85 < x < 1.35 and abs(y-1.275) > .15)
    patch(node, {(24, 25), next_cell}, at=100.2)
    invalid = paths(node)[-1]
    assert not invalid.poses
    follower.on_path(invalid)
    assert follower.pending_path is None
    follower.on_result(result_future())
    follower.client.send_goal_async.assert_not_called()
