"""A discarded guard diagnosis must not orphan a fresh replacement route.

Route/guard packets are recorded; stop, sensor and action evidence is controlled
and checked through the existing callbacks. No ROS nodes or servers are started.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from test_execution_failure_follower import detect, prepared, refresh
from test_path_follower_cancel import goal_for, make_path, result_future
from test_sharp_replan_alignment_independent import (
    confirm_stop, finish_spin, path_message, rig,
)


FIXTURE = Path(__file__).parent / 'fixtures' / 'guard_veto_handoff_20261105.json'


def guard(node, held):
    node.on_motion_status(String(data=json.dumps(dict(
        node.motion_status, scan_guard_hold=held,
        scan_guard_code='SWEPT_INTERSECTION' if held else 'CLEAR',
        scan_guard_reason='LiDAR return intersects requested chassis sweep' if held else ''))))


def test_recorded_guard_veto_queues_exact_local_route_without_early_motion(rig):
    node, clock, pose, sample = rig
    record = json.loads(FIXTURE.read_text())
    old, replacement = (path_message(record[key])
                        for key in ('old_path', 'replacement_path'))
    assert (len(old.poses), len(replacement.poses)) == (94, 93)
    clock['source'] = 13.
    sample(.01)
    pose(-.05, -.44, -.85)
    goal = goal_for(old, stamp=7)
    goal.header.stamp.nanosec = 700_000_000
    goal.pose.position.x, goal.pose.position.y = -.25, -1.98
    node.on_goal(goal)
    node._dispatch_follow_path(old)
    context = node.request_context
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)), context)
    for key in ('guard_hold', 'guard_clear'):
        node.on_motion_status(String(data=json.dumps(record[key])))
    assert node.pending_execution_failure is None
    assert node.execution_discarded_reason is not None
    assert node.active_goal is handle
    assert node._navigation_input_error() is None

    node.on_path(replacement)
    assert node.pending_path is replacement
    handle.cancel_goal_async.assert_called_once()
    node._on_follow_cancel_response(Mock(result=Mock(return_value=
        SimpleNamespace(goals_canceling=[object()]))), handle)
    confirm_stop(node, sample)
    assert node.follow_cancel_acknowledged
    assert node.client.send_goal_async.call_count == 1
    node.spin_client.send_goal_async.assert_not_called()

    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle)
    assert node.client.send_goal_async.call_count == 1
    node.spin_client.send_goal_async.assert_not_called()
    confirm_stop(node, sample)
    assert node.initial_alignment['kind'] == 'material_replan'
    finish_spin(node, pose, replacement)
    assert node.client.send_goal_async.call_count == 2
    assert node.client.send_goal_async.call_args.args[0].path is replacement
    assert node.request_context['path_stamp'] == {'sec': 12, 'nanosec': 520_000_000}
    assert context['path_stamp'] == {'sec': 7, 'nanosec': 700_000_000}
    node.execution_failure_pub.publish.assert_not_called()


def test_guard_discards_pending_diagnosis_but_preserves_queued_route_and_terminal_barrier():
    node, handle = prepared()
    detect(node)
    assert node.pending_execution_failure is not None and node.cancel_requested
    guard(node, True)
    guard(node, False)
    assert node.pending_execution_failure is None
    refresh(node)
    replacement = make_path(y=.15, stamp=node.clock_seconds+1)
    # Keep the same mission endpoint while changing the actual route.
    replacement.poses[-1] = deepcopy(node.active_path.poses[-1])
    node.on_path(replacement)
    assert node.pending_path is replacement
    node._on_follow_cancel_response(Mock(result=Mock(return_value=
        SimpleNamespace(goals_canceling=[object()]))), handle)
    assert node.client.send_goal_async.call_count == 1
    node.on_result(result_future(), handle=handle)
    assert node.active_goal is None
    assert node.client.send_goal_async.call_count == 2
    assert node.client.send_goal_async.call_args.args[0].path is replacement
    node.execution_failure_pub.publish.assert_not_called()


def test_real_unconfirmed_execution_diagnosis_still_rejects_intermediate_route():
    node, handle = prepared()
    detect(node)
    replacement = make_path(stamp=node.clock_seconds+1)
    node.on_path(replacement)
    assert node.pending_execution_failure is not None
    assert node.pending_path is None and node.active_goal is handle
    assert node.client.send_goal_async.call_count == 1


@pytest.mark.parametrize('cutoff_name', ['execution_replan_cutoff_ns', 'navigation_input_cutoff_ns'])
def test_guard_discard_does_not_bypass_stale_path_cutoffs(cutoff_name):
    node, handle = prepared()
    guard(node, True)
    guard(node, False)
    replacement = make_path(stamp=9)
    setattr(node, cutoff_name, 9_000_000_000)
    node.on_path(replacement)
    assert node.active_goal is handle and node.pending_path is None
    handle.cancel_goal_async.assert_not_called()
    assert node.client.send_goal_async.call_count == 1


def test_guard_discard_does_not_accept_wrong_goal_route():
    node, handle = prepared()
    node.require_goal_match = True
    guard(node, True)
    guard(node, False)
    node.on_path(make_path(y=1., stamp=9))
    assert node.active_goal is handle and node.pending_path is None
    assert node.suspended_paths
    handle.cancel_goal_async.assert_not_called()
