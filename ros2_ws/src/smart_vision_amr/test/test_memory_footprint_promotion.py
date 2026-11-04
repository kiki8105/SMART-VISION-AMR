"""Confirmation changes persistence, never the physical collision footprint."""
import importlib.util
from pathlib import Path
from copy import deepcopy
from unittest.mock import Mock

import pytest
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray

from smart_vision_amr.local_detour import LocalDetourResult
from smart_vision_amr.map_utils import inflate_cells, occupancy_to_grid
from test_local_detour_planner import planner_factory, ready
from test_planner_failure_memory import FailureHarness
from test_planner_recovery import make_goal


ROOT = Path(__file__).resolve().parents[4]
BENCH = ROOT/'benchmarks/driving_parameter_tuning_20261104/memory_footprint_promotion_20261105'
SPEC = importlib.util.spec_from_file_location('memory_promotion_replay', BENCH/'replay.py')
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


def packet(node, cells):
    return Int32MultiArray(data=[y*node.map[0].info.width+x for x,y in sorted(cells)])


@pytest.fixture
def recorded(monkeypatch):
    context, data = replay.load_fixture()
    clock = {'steady': 100., 'ros': 100.}
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: clock['steady'])
    node = FailureHarness(clock)
    node.params.update(data['parameters'])
    source = context.map[0]
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    for key in ('width', 'height', 'resolution'):
        setattr(message.info, key, getattr(source.info, key))
    message.info.origin.position.x, message.info.origin.position.y = data['map']['origin'][:2]
    message.info.origin.orientation.w = 1.
    message.data = source.data
    node.on_map(message)
    node.last_goal = make_goal(stamp=100)
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = -.25, -1.98
    node.local_reference_points = list(context.local_reference_points)
    node.local_reference_key = node._local_route_key()
    node.observed, node.remembered = set(context.observed), set(context.remembered)
    ready(node, x=data['current_pose'][0], y=data['current_pose'][1], yaw=data['current_pose'][2])
    node.last_observation_time = clock['steady']
    return node, data


def test_recorded_old_square_start_lock_and_consistent_67cell_fallback():
    node, data = replay.load_fixture()
    before_memory, before_observed = set(node.remembered), set(node.observed)
    result = replay.compare(node, data, measure_clearance=False)
    old, new = result['old_square_memory'], result['consistent_footprint_memory']
    assert (old['remembered_mask_cells'], old['observed_mask_cells'],
            old['virtual_cells'], old['escape_cells']) == (242, 239, 36, 11)
    assert old['start_in_remembered_mask'] and old['primary_route_cells'] == 0
    assert not old['fallback_permitted'] and old['fallback_route_cells'] == 0
    assert new['remembered_mask_cells'] == 215 and not new['start_in_remembered_mask']
    assert new['primary_route_cells'] == 0 and new['fallback_route_cells'] == 67
    assert node.remembered == before_memory and node.observed == before_observed
    assert data['original_trial_result']['strict_passed'] is False


def test_real_plan_to_uses_same_memory_body_and_retains_all_raw_hits(recorded):
    node, _ = recorded
    before = set(node.remembered)
    source = list(node.map[0].data)
    node.plan_to(node.last_goal)
    assert node.planned_world_points and node.path_pub.publish.call_args.args[0].poses
    assert node.plan_metrics['occlusion_fallback'] is True
    assert node.plan_metrics['occlusion_measured_start_certificate'] is False
    assert node.remembered == before
    assert all(cell not in node._live_obstacle_blocked(before) for cell in node.planned_cells)
    assert list(node.map[0].data) == source


def test_promoting_same_safe_scan_hits_to_memory_does_not_cancel_actual_route(recorded):
    node, _ = recorded
    promotion = set(node.remembered)
    node.observed |= promotion
    node.remembered = set()
    node.plan_to(node.last_goal)
    assert node.planned_world_points
    remaining = set(node._remaining_path_cells())
    info = node.map[0].info
    assert remaining & inflate_cells(promotion, info.width, info.height, 5), 'Must exercise old square-only overlap'
    assert not remaining & node._live_obstacle_blocked(promotion)
    current_path = node.current_path_stamp_ns
    node.path_pub.publish.reset_mock()
    node._request_replan = Mock()
    node.on_memory(packet(node, promotion))
    assert node.remembered == promotion
    assert node.current_path_stamp_ns == current_path and node.planned_world_points
    node.path_pub.publish.assert_not_called()
    node._request_replan.assert_not_called()


def test_genuine_memory_body_intersection_still_cancels_and_retains_goal(recorded):
    node, data = recorded
    node.plan_to(node.last_goal)
    goal = node.last_goal
    collision = tuple(data['start_cell'])
    assert node._changes_block_path({collision}, layer='memory')
    node.on_memory(packet(node, node.remembered|{collision}))
    assert collision in node.remembered
    assert not node.path_pub.publish.call_args.args[0].poses
    assert node.last_goal is goal and node.waiting_for_replan


def test_empty_overlay_and_unobserved_visibility_do_not_delete_retained_memory(recorded):
    node, data = recorded
    before = set(node.remembered)
    node.plan_to(node.last_goal)
    node.clock.update(steady=100.1, ros=100.1)
    ready(node, 100.1, x=data['current_pose'][0], y=data['current_pose'][1], yaw=data['current_pose'][2])
    node.on_observed(Int32MultiArray(data=[]))
    unseen = deepcopy(node.map[0])
    unseen.header.stamp = node.get_clock().now().to_msg()
    unseen.data = [-1]*len(unseen.data)
    node.on_visibility(unseen)
    assert node.remembered == before
    assert node._live_obstacle_blocked(before)


@pytest.mark.parametrize('margin', [0., .025])
def test_local_observed_and_memory_use_identical_fullbody_margin(planner_factory, monkeypatch, margin):
    captured = []
    def search(_grid, _start, _reference, blocked, **_kwargs):
        captured.append(set(blocked))
        return LocalDetourResult(None, [], None, 0, 'no_local_route', 0.)
    monkeypatch.setattr('smart_vision_amr.astar_planner.plan_local_detour', search)
    masks = []
    for layer in ('observed', 'remembered'):
        node = planner_factory()
        node.params.update(local_detour_clearance_margin=.01,
                           local_detour_observation_margin=margin,
                           local_detour_occlusion_extension=0.)
        raw = {(25, 25)}
        node.observed, node.remembered = (raw, set()) if layer == 'observed' else (set(), raw)
        node.last_observation_time = node.clock['steady']
        assert not node._try_local_detour(raw)
        masks.append(captured[-1])
        assert raw <= masks[-1]
    assert masks[0] == masks[1]


def test_static_square_corners_unknown_and_raw_memory_stay_hard(recorded):
    node, _ = recorded
    original = node.map[1].rows
    expected = occupancy_to_grid(node.map[0], 65, 5)
    assert original == expected.rows
    unknown = next((x,y) for y in range(node.map[0].info.height)
                   for x in range(node.map[0].info.width)
                   if node.map[0].data[y*node.map[0].info.width+x] < 0)
    assert node.raw_static_grid.is_static_obstacle(unknown)
    assert node.map[1].is_static_obstacle(unknown)
    assert node.remembered <= node._live_obstacle_blocked(node.remembered)
    assert node.map[1].rows == original
