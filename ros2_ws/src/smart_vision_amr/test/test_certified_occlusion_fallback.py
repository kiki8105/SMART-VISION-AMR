"""A moved box may reuse this call's exact, fresh start-only certificate."""

from unittest.mock import patch

import pytest

from smart_vision_amr.continuous_start_egress import certified_start_only_edges
from smart_vision_amr.map_utils import inflate_cells, world_to_cell
from smart_vision_amr.path_geometry import collision_free
from test_local_detour_planner import path_box_clearance, ready
from test_occlusion_reserve_incident import FIXTURE, load_incident


@pytest.fixture
def moved_box(monkeypatch):
    node, recorded = load_incident(FIXTURE.parent/'occlusion_reserve_certified_start_20261104.json')
    node.params['local_detour_occlusion_extension'] = recorded['recorded_extension_m']
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    ready(node, x=recorded['pose_world'][0], y=recorded['pose_world'][1], yaw=recorded['pose_world'][2],
          scan_frame=node.params['base_frame'])
    node._record_start_visibility(node.get_clock().now().nanoseconds, node.clock['steady'])
    node.last_observation_time = node.clock['steady']
    return node, recorded


def certify_snapshot(node, recorded):
    start = tuple(recorded['start_cell'])
    info, grid = node.map[0].info, node.map[1]
    reserved_observed = node._global_occlusion_reserve(start, node.observed)
    reserved_memory = node._global_occlusion_reserve(start, node.remembered, remembered=True)
    conservative = (node._live_obstacle_blocked(reserved_observed)
                    | inflate_cells(reserved_memory, info.width, info.height, 5))
    pose = recorded['pose_world']
    return certified_start_only_edges(
        node.raw_static_grid, grid, node.observed|node.remembered, conservative, start,
        ((pose[0]+1.65)/.05, (pose[1]+5.25)/.05), .22/.05, 0.)


def test_recorded_certificate_is_reused_without_any_new_exit_or_escape_cell(moved_box):
    node, recorded = moved_box
    expected = {tuple(map(tuple, edge)) for edge in recorded['recorded_certificate_edges']}
    assert certify_snapshot(node, recorded) == expected == {((33, 91), (32, 91))}
    original_observed, original_memory = set(node.observed), set(node.remembered)
    with patch.object(node.planner, 'plan', wraps=node.planner.plan) as search:
        node.plan_to(node.last_goal)
    assert node.planned_world_points and node.plan_metrics['occlusion_fallback']
    assert node.plan_metrics['occlusion_reused_start_certificate']
    # Conservative search and soft search retain the exact same single-cell
    # permission and all outgoing, re-entry and diagonal-side restrictions.
    assert len(search.call_args_list) == 2
    first, second = [call.kwargs for call in search.call_args_list]
    forbidden = {tuple(map(tuple, edge)) for edge in recorded['recorded_forbidden_edges']}
    assert first['blocked_edges'] == second['blocked_edges'] == forbidden
    assert len(forbidden) == 23
    assert first['escape_cells'] == second['escape_cells'] == {(33, 91)}
    assert first['allow_blocked_start'] is second['allow_blocked_start'] is True
    info, grid = node.map[0].info, node.map[1]
    cells = [world_to_cell(info, *point) for point in node.planned_world_points]
    assert tuple(cells[:2]) in expected
    assert cells.count((33, 91)) == 1
    assert all((a, b) not in forbidden for a, b in zip(cells, cells[1:]))
    measured = (node._live_obstacle_blocked(node.observed)
                | inflate_cells(node.remembered, info.width, info.height, 5))
    # Only the originally certified first cell is exceptional. Every later
    # cell and the complete first-edge sweep retain their existing checks.
    points = [((x+1.65)/.05, (y+5.25)/.05) for x, y in node.planned_world_points]
    assert collision_free(points, lambda c:grid.is_static_obstacle(c) or (c in measured and c != (33, 91)))
    assert path_box_clearance(node.planned_world_points, recorded['actor']['bounds'])-.22 == pytest.approx(
        .1000188444122567, abs=1e-6)
    assert node.observed == original_observed and node.remembered == original_memory


def test_same_cell_pose_drift_after_primary_search_cannot_reuse_unsafe_certificate(moved_box):
    node, recorded = moved_box
    original = tuple(recorded['pose_world'])
    unsafe = (.049, original[1], original[2])
    assert world_to_cell(node.map[0].info, *unsafe[:2]) == tuple(recorded['start_cell'])
    calls = [0]
    def fresh_pose():
        calls[0] += 1
        return original if calls[0] == 1 else unsafe
    with patch.object(node, '_fresh_motion_pose', side_effect=fresh_pose), \
         patch('smart_vision_amr.astar_planner.certified_start_only_edges',
               wraps=certified_start_only_edges) as certify:
        node.plan_to(node.last_goal)
    assert certify.call_count >= 2
    assert not node.planned_world_points
    assert not node.plan_metrics['occlusion_fallback']


@pytest.mark.parametrize('change', ['pose', 'scan', 'manual', 'memory', 'observed_inplace'])
def test_final_publication_rechecks_certificate_after_soft_search(moved_box, change):
    node, recorded = moved_box
    real_search, real_pose = node.planner.plan, node._fresh_motion_pose
    after_search = [False]
    def search(*args, **kwargs):
        result = real_search(*args, **kwargs)
        if result:  # The primary conservative search returns no path.
            after_search[0] = True
            if change == 'scan': node.latest_scan_at -= 2.
            if change == 'manual': node.latest_motion_status['mode'] = 'manual'
            if change == 'memory':
                node.remembered = {tuple(c) for c in recorded['later_memory_block_snapshot']['remembered_cells']}
            if change == 'observed_inplace': node.observed.add((44, 82))
        return result
    def pose():
        if after_search[0] and change == 'pose':
            return (.049, recorded['pose_world'][1], recorded['pose_world'][2])
        return real_pose()
    with patch.object(node.planner, 'plan', side_effect=search), \
         patch.object(node, '_fresh_motion_pose', side_effect=pose):
        node.plan_to(node.last_goal)
    assert after_search[0]
    assert not node.planned_world_points
    assert not any(call.args[0].poses for call in node.path_pub.publish.call_args_list)
    assert not node.plan_metrics['occlusion_fallback']


def test_later_confirmed_memory_block_cannot_borrow_the_previous_calls_certificate(moved_box):
    node, recorded = moved_box
    node.plan_to(node.last_goal)
    assert node.planned_world_points
    later = recorded['later_memory_block_snapshot']
    node.observed = {tuple(cell) for cell in later['observed_cells']}
    node.remembered = {tuple(cell) for cell in later['remembered_cells']}
    assert len(node.remembered) == 14
    assert tuple(recorded['start_cell']) in inflate_cells(node.remembered, 86, 140, 5)
    ready(node, at=101., x=recorded['pose_world'][0], y=recorded['pose_world'][1],
          yaw=recorded['pose_world'][2])
    node.last_observation_time = 101.
    node.plan_to(node.last_goal)
    assert not node.planned_world_points
    assert not node.plan_metrics['occlusion_fallback']
    assert not node.plan_metrics['occlusion_reused_start_certificate']
