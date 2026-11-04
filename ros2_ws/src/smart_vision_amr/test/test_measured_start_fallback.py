"""Measured-mask first-edge fallback preserves actual occupancy and permissions."""
from unittest.mock import patch

import pytest

from smart_vision_amr.map_utils import world_to_cell
from smart_vision_amr.path_geometry import collision_free
from body_geometry_oracle import continuous_raw_body_gap, full_cell_body_mask
from test_local_detour_planner import path_box_clearance, ready
from test_occlusion_reserve_incident import FIXTURE, load_incident


@pytest.fixture
def fixed_sample(monkeypatch):
    node, record = load_incident(FIXTURE.parent/'occlusion_reserve_virtual_memory_start_20261104.json')
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    ready(node, x=record['pose_world'][0], y=record['pose_world'][1], yaw=record['pose_world'][2],
          scan_frame=node.params['base_frame'])
    node._record_start_visibility(node.get_clock().now().nanoseconds, node.clock['steady'])
    node.last_observation_time = node.clock['steady']
    return node, record


def test_sampled_fixed_baseline_virtual_neighbors_can_use_measured_first_edge(fixed_sample):
    node, record = fixed_sample
    observed, memory = set(node.observed), set(node.remembered)
    with patch.object(node.planner, 'plan', wraps=node.planner.plan) as search:
        node.plan_to(node.last_goal)
    assert node.plan_metrics['occlusion_fallback']
    assert node.plan_metrics['occlusion_measured_start_certificate']
    assert node.plan_metrics['start_certificate_diagnostics']['allowed_edges'] == []
    assert len(search.call_args_list) == 2
    primary, fallback = [c.kwargs for c in search.call_args_list]
    start, destination = (34, 93), (33, 93)
    assert not primary['escape_cells'] and not primary['allow_blocked_start']
    assert fallback['escape_cells'] == {start} and fallback['allow_blocked_start']
    assert len(fallback['blocked_edges']) == 23
    info, grid = node.map[0].info, node.map[1]
    measured = full_cell_body_mask(observed | memory, info, .22)
    assert search.call_args_list[-1].args[3] == measured
    cells = [world_to_cell(info, *point) for point in node.planned_world_points]
    assert cells[:2] == [start, destination] and cells.count(start) == 1
    points = [((x+1.65)/.05, (y+5.25)/.05) for x, y in node.planned_world_points]
    assert collision_free(points, lambda cell:grid.is_static_obstacle(cell) or cell in measured-{start})
    assert path_box_clearance(node.planned_world_points, record['actor']['bounds'])-.22 > .08
    assert continuous_raw_body_gap(node, node.planned_world_points) >= -1e-10
    assert node.active_start_certificate['edge'] == (start, destination)
    assert node.observed == observed and node.remembered == memory


def test_logged_virtual_memory_gate_does_not_become_actual_memory_policy(fixed_sample):
    node, _ = fixed_sample
    original = node._global_occlusion_reserve
    # The exact final callback/path was not recorded. Supply only the logged
    # reserve-only memory fact; measured six-cell memory remains untouched.
    def reserve(start, hits, *, remembered=False):
        result = original(start, hits, remembered=remembered)
        return result | {(39, 93)} if remembered else result
    memory = set(node.remembered)
    with patch.object(node, '_global_occlusion_reserve', side_effect=reserve):
        node.plan_to(node.last_goal)
    diagnostic = node.plan_metrics['start_certificate_diagnostics']
    assert diagnostic['remembered_inflated_start'] is True
    assert diagnostic['measured_remembered_inflated_start'] is False
    assert diagnostic['first_failed_reason'] == 'remembered_inflation_start'
    assert diagnostic['helper_called'] is False
    assert node.plan_metrics['occlusion_measured_start_certificate']
    assert node.planned_world_points and node.active_start_certificate
    assert node.remembered == memory


@pytest.mark.parametrize('blocker', ['memory', 'raw', 'static', 'unknown'])
def test_actual_hard_start_occupancy_cannot_borrow_virtual_fallback(fixed_sample, blocker):
    node, _ = fixed_sample
    if blocker == 'memory':
        node.remembered.add((39, 93))  # Its actual full-cell body mask covers start.
    elif blocker == 'raw':
        node.observed.add((34, 93))
    else:
        message = node.map[0]
        data = list(message.data)
        data[93*message.info.width+34] = 100 if blocker == 'static' else -1
        message.data = data
        node.raw_static_grid = None
        from smart_vision_amr.map_utils import occupancy_to_grid
        node.raw_static_grid = occupancy_to_grid(message, 65, 0)
        node.map = (message, occupancy_to_grid(message, 65, 5))
    node.plan_to(node.last_goal)
    assert not node.planned_world_points
    assert not node.active_start_certificate


def test_same_cell_pose_drift_before_measured_certificate_rejects_body_collision(fixed_sample):
    node, record = fixed_sample
    real = node.planner.plan
    def search(*args, **kwargs):
        result = real(*args, **kwargs)
        if not result:
            ready(node, x=.099, y=record['pose_world'][1], yaw=record['pose_world'][2])
        return result
    with patch.object(node.planner, 'plan', side_effect=search):
        node.plan_to(node.last_goal)
    assert not node.planned_world_points
    assert not node.active_start_certificate
