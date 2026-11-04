"""Meaningful local detours must cross the existing confirmed-cancel barrier."""
import json
from pathlib import Path
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath

from smart_vision_amr.path_geometry import (paths_are_equivalent,
    paths_share_remaining_geometry)
from test_path_follower_cancel import FollowerHarness, result_future

FIXTURE = Path(__file__).resolve().parents[4] / 'benchmarks/controller_replay_20261104/local_detour_delivery_fixture.json'


def route(points, stamp_ns=0):
    path = RosPath()
    path.header.frame_id = 'map'
    path.header.stamp.sec, path.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    return path


def recorded_routes():
    source = json.loads(FIXTURE.read_text())
    return tuple(route(source[k]['points'], source[k]['stamp_ns']) for k in ('original', 'detour'))


def test_recorded_eighteen_centimeter_detour_is_not_only_start_progress():
    old, new = recorded_routes()
    a, b = FollowerHarness._points(old), FollowerHarness._points(new)
    assert len(a) == 93 and len(b) == 81
    assert paths_are_equivalent(a, b)  # The previous defect is reproduced.
    assert not paths_share_remaining_geometry(a, b)


def test_recorded_detour_waits_for_terminal_cancel_then_reaches_follow_path():
    old, new = recorded_routes()
    node = FollowerHarness()
    handle = Mock()
    node.active_goal, node.active_path = handle, old
    node.latest_path_key = node._path_key(old)
    node.on_path(new)
    assert node.pending_path is new
    handle.cancel_goal_async.assert_called_once_with()
    node.client.send_goal_async.assert_not_called()
    # An ack alone cannot dispatch a concurrent replacement action.
    ack = Mock(result=Mock(return_value=Mock(goals_canceling=[Mock()])))
    node._on_follow_cancel_response(ack, handle)
    node.client.send_goal_async.assert_not_called()
    node.on_result(result_future())
    node.client.send_goal_async.assert_called_once()
    assert node.client.send_goal_async.call_args.args[0].path is new


def test_pure_mid_segment_pruning_and_sparse_dense_resampling_keep_action():
    old = route([(0, 0), (1, 0), (1, 1)])
    new = route([(.437, 0), (.7, 0), (1, 0), (1, .37), (1, .8), (1, 1)])
    assert paths_share_remaining_geometry(FollowerHarness._points(old), FollowerHarness._points(new))
    node = FollowerHarness()
    node.active_goal, node.active_path = Mock(), old
    node.latest_path_key = node._path_key(old)
    node.on_path(new)
    node.active_goal.cancel_goal_async.assert_not_called()
    assert node.pending_path is None


def test_small_continuous_jitter_is_kept_but_one_grid_cell_shift_replaces():
    a = [(0., 0.), (1., 0.), (2., 0.)]
    assert paths_share_remaining_geometry(a, [(.213, .01), (1.1, .01), (2., .01)])
    assert not paths_share_remaining_geometry(a, [(.213, .05), (1.1, .05), (2., .05)])


def test_cut_corner_and_reversed_loop_order_are_not_the_same_geometry():
    assert not paths_share_remaining_geometry([(0, 0), (1, 1), (2, 0)], [(0, 0), (2, 0)])
    a = [(0, 0), (1, 0), (1, 1), (0, 0), (-1, 0), (-1, 1), (0, 0)]
    b = [(0, 0), (-1, 0), (-1, 1), (0, 0), (1, 0), (1, 1), (0, 0)]
    assert not paths_share_remaining_geometry(a, b)


def test_degenerate_or_nonfinite_paths_require_a_replacement():
    assert paths_share_remaining_geometry([(1, 2)], [(1, 2)])
    assert not paths_share_remaining_geometry([], [(1, 2)])
    assert not paths_share_remaining_geometry([(0, 0), (float('nan'), 1)], [(0, 0)])
