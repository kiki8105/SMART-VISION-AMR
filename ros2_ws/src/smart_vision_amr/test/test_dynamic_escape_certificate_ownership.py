"""Recorded halo-selected first edge has the same callback proof as a fallback.

No ROS node/executor or actions are started. Recorded raster geometry is real;
raw scan/TF receipts in callback tests are explicitly fresh synthetic evidence.
"""
import json
from pathlib import Path
from unittest.mock import patch

import pytest
from tf2_ros import TransformException

from body_geometry_oracle import continuous_raw_body_gap, full_cell_body_mask
from smart_vision_amr.map_utils import world_to_cell
from test_local_detour_planner import ready, hits, path_box_clearance
from test_occlusion_reserve_incident import load_incident

FIXTURE = (Path(__file__).resolve().parents[4] / 'benchmarks/driving_parameter_tuning_20261104'
           / 'planner_start_cancel_audit_20261105/fixture.json')


@pytest.fixture
def incident(monkeypatch):
    node, record = load_incident(FIXTURE)
    # These are two actual observer frames bracketing publication, not a
    # claim that the planner's internal DDS snapshot was fully captured.
    node.observed = {tuple(cell) for cell in record['adjacent_observed_cells']}
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = record['actual_goal_world']
    node.local_reference_key = node._local_route_key()
    ready(node, x=record['pose_world'][0], y=record['pose_world'][1], yaw=record['pose_world'][2],
          scan_frame=node.params['base_frame'])
    node.last_observation_time = node.clock['steady']
    node.last_visibility_stamp_ns = node.get_clock().now().nanoseconds
    node._record_start_visibility(node.last_visibility_stamp_ns, node.clock['steady'])
    return node, record


def test_recorded_halo_route_owns_only_its_chosen_first_edge(incident):
    node, record = incident
    observed, memory = set(node.observed), set(node.remembered)
    node.plan_to(node.last_goal)
    assert node.planned_world_points
    chosen = ((31, 96), (30, 96))
    assert node.active_start_certificate['edge'] == chosen
    assert node.plan_metrics['selected_start_certificate_diagnostics'] == {
        'reason': 'selected_first_edge_certified', 'edge': [[31, 96], [30, 96]]}
    info = node.map[0].info
    cells = [world_to_cell(info, *point) for point in node.planned_world_points]
    assert cells[:2] == list(chosen) and cells.count(chosen[0]) == 1
    measured = full_cell_body_mask(observed | memory, info, .22)
    assert cells[0] not in measured and not set(cells[1:]) & measured
    assert continuous_raw_body_gap(node, node.planned_world_points) >= 0.
    assert path_box_clearance(node.planned_world_points, record['actor']['bounds'])-.22 > .08
    assert node.observed == observed and node.remembered == memory
    assert node.plan_metrics['start_certificate_state'] == 'ACTIVE'


def test_bracketed_free_and_blocked_frames_keep_only_the_fresh_owned_edge(incident):
    node, record = incident
    node.plan_to(node.last_goal)
    assert node.active_start_certificate
    current = list(node.planned_world_points)
    assert not node._live_route_blockers(node.observed)
    # The neighbouring real frame contains36,94: its full-cell body mask
    # reaches start, while the entire actual first-edge sweep stays clear.
    blocked_frame = {tuple(cell) for cell in record['observed_cells']}
    assert (36, 94) in blocked_frame and (36, 94) not in node.observed
    assert not node._live_route_blockers(blocked_frame)
    node.on_observed(hits(node, blocked_frame))
    assert node.planned_world_points == current and not node.waiting_for_replan
    assert node.active_start_certificate


def test_recorded_measured_start_frame_keeps_existing_fallback_certificate(incident):
    node, record = incident
    node.observed = {tuple(cell) for cell in record['observed_cells']}
    node.plan_to(node.last_goal)
    assert node.planned_world_points and node.active_start_certificate
    assert node.active_start_certificate['edge'] == ((31, 96), (30, 96))
    assert node.plan_metrics['occlusion_measured_start_certificate']


@pytest.mark.parametrize('kind', ['raw_body', 'memory_start', 'stale_scan', 'authority'])
def test_selected_edge_keeps_actual_occupancy_and_fresh_authority_gates(incident, kind):
    node, record = incident
    start = (31, 96)
    if kind == 'raw_body':
        node.observed.add(start)
    elif kind == 'memory_start':
        node.remembered.add((36, 94))
    elif kind == 'stale_scan':
        node.latest_scan_at = node.clock['steady']-1.
    else:
        ready(node, x=record['pose_world'][0], y=record['pose_world'][1], yaw=record['pose_world'][2],
              motion={'mode': 'manual', 'autonomous_allowed': False})
    measured = full_cell_body_mask(node.observed | node.remembered, node.map[0].info, .22)
    assert not node._selected_dynamic_escape_certificate(start, [start, (30, 96), (30, 97)],
        node.map[1], node.raw_static_grid, measured, set(node.observed), set(node.remembered))
    assert not node.active_start_certificate


def test_unsafe_adjacent_exit_is_not_borrowed_from_the_safe_west_edge(incident):
    node, _ = incident
    start = (31, 96)
    measured = full_cell_body_mask(node.observed | node.remembered, node.map[0].info, .22)
    assert not node._selected_dynamic_escape_certificate(start, [start, (32, 96)],
        node.map[1], node.raw_static_grid, measured, set(node.observed), set(node.remembered))


def test_second_measured_cell_is_not_exempted_by_a_safe_first_edge(incident):
    node, _ = incident
    start = (31, 96)
    measured = full_cell_body_mask(node.observed | node.remembered, node.map[0].info, .22)
    assert (31, 95) in measured
    assert not node._selected_dynamic_escape_certificate(start, [start, (30, 96), (31, 95)],
        node.map[1], node.raw_static_grid, measured, set(node.observed), set(node.remembered))
    assert node.plan_metrics['selected_start_certificate_diagnostics']['reason'] == 'later_measured_or_static_cell_blocked'


@pytest.mark.parametrize('tail', [[(31, 96)], [(31, 97)]], ids=['reentry', 'start_side_diagonal'])
def test_first_edge_cannot_authorize_reentry_or_an_adjacent_diagonal(incident, tail):
    node, _ = incident
    start = (31, 96)
    measured = full_cell_body_mask(node.observed | node.remembered, node.map[0].info, .22)
    assert not set(tail) & measured
    assert not node._selected_dynamic_escape_certificate(start, [start, (30, 96), *tail],
        node.map[1], node.raw_static_grid, measured, set(node.observed), set(node.remembered))
    assert node.plan_metrics['selected_start_certificate_diagnostics']['reason'] == 'reentry_or_adjacent_diagonal_forbidden'


def test_scan_tf_diagnostic_does_not_soften_existing_rejection(incident):
    node, record = incident
    points = [record['pose_world'][:2], *record['recorded_emitted_world'][:2]]
    with patch.object(node.tf_buffer, 'lookup_transform', side_effect=TransformException('future acquisition stamp')):
        assert not node._prefix_sweep_clear(points)
    detail = node.plan_metrics['start_prefix_validation']
    assert detail['reason'] == 'coherent_raw_scan_input_pending' and detail['input_pending']
    assert 'future acquisition stamp' in node.plan_metrics['start_scan_selection']['lookup_error']


def test_raw_body_diagnostic_stays_a_hard_rejection(incident):
    node, record = incident
    node.observed.add((31, 96))
    assert not node._prefix_sweep_clear([record['pose_world'][:2], record['recorded_emitted_world'][1]])
    assert node.plan_metrics['start_prefix_validation']['reason'] == 'raw_static_unknown_or_dynamic_cell_intersection'
