"""Real reverse adapter/masks with frozen hits and an explicit synthetic trail.

The source incident did not record breadcrumbs. These are necessary-gate
counterfactuals, not a replay of an actual executed reverse or a drive pass.
"""
from collections import deque
from copy import deepcopy
import json
from math import cos, sin
from pathlib import Path
from unittest.mock import patch

import pytest

from smart_vision_amr.astar_planner import FAILURE_DEFAULTS
from smart_vision_amr.map_utils import inflate_cells
from smart_vision_amr.reverse_recovery import Breadcrumb, select_reverse_egress
from body_geometry_oracle import continuous_raw_body_gap, full_cell_body_mask
from test_local_detour_planner import ready
from test_occlusion_reserve_incident import load_incident


REPO = Path(__file__).resolve().parents[4]
FIXTURE = REPO/'benchmarks/driving_parameter_tuning_20261104/recovery_audit/fixture.json'


@pytest.fixture
def rear_scene(monkeypatch):
    data = json.loads(FIXTURE.read_text())
    node, _ = load_incident()
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    node.observed = {tuple(c) for c in data['observed_cells']}
    node.remembered = {tuple(c) for c in data['remembered_cells']}
    node.local_reference_points = [tuple(p) for p in data['reference_world']]
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = node.local_reference_points[-1]
    node.local_reference_key = node._local_route_key()
    pose = tuple(data['pose'])
    ready(node, x=pose[0], y=pose[1], yaw=pose[2])
    node.latest_odom_child_frame = node.params['base_frame']
    node.last_observation_time = 100.
    node.odom_samples = deque((
        (99.75+i*.05, 99_750_000_000+i*50_000_000, *node.latest_odom[2:])
        for i in range(6)), maxlen=100)
    node.breadcrumbs = deque((Breadcrumb(100.-distance/.1,
        pose[0]-distance*cos(pose[2]), pose[1]-distance*sin(pose[2]), pose[2])
        for distance in (.30,.275,.25,.225,.20,.175,.15,.125,.10,.075,.05,.025,0.)), maxlen=300)
    node.breadcrumb_odom_anchor = node.latest_odom
    info = node.map[0].info
    start = tuple(data['start_cell'])
    reserved_observed = node._global_occlusion_reserve(start, node.observed)
    reserved_memory = node._global_occlusion_reserve(start, node.remembered, remembered=True)
    assert reserved_observed == {tuple(c) for c in data['reserved_observed_cells']}
    assert reserved_memory == {tuple(c) for c in data['reserved_remembered_cells']}
    # Retain this archived conservative input, including its historical
    # square memory border. The measured fallback below must still use exact
    # body geometry and can soften only this mask's unmeasured reserve.
    blocked = (node._live_obstacle_blocked(reserved_observed)
        | inflate_cells(reserved_memory, info.width, info.height, 5))
    return node, data, blocked


def probe(scene, *, evidence=()):
    node, data, blocked = scene
    return node._maybe_request_reverse(tuple(data['start_cell']), tuple(data['goal_cell']),
        node.map[0], node.map[1], node.raw_static_grid, node.observed | node.remembered,
        blocked, {}, True, blocking_evidence=set(evidence))


def test_clearance_neutral_candidate_is_off_by_default(rear_scene):
    node, _, _ = rear_scene
    assert FAILURE_DEFAULTS['reverse_clearance_neutral_egress_enabled'] is False
    assert node.params['reverse_clearance_neutral_egress_enabled'] is False
    assert not probe(rear_scene)
    node.recovery_pub.publish.assert_not_called()


def test_frozen_full_inflation_exit_requires_primary_then_fresh_measured_goal_oracle(rear_scene):
    node, _, blocked = rear_scene
    node.params['reverse_clearance_neutral_egress_enabled'] = True
    with patch.object(node.planner, 'plan', wraps=node.planner.plan) as search:
        assert probe(rear_scene)
    request = json.loads(node.recovery_pub.publish.call_args.args[0].data)
    assert request['distance'] == pytest.approx(.125)
    assert tuple(request['exit_cell']) not in blocked
    assert request['recovery_trigger'] == 'global_no_path'
    assert node.plan_metrics['reverse_goal_occlusion_fallback']
    assert node.plan_metrics['reverse_clearance_neutral_egress']
    assert len(search.call_args_list) == 2
    first, second = search.call_args_list
    assert set(first.args[3]) == blocked
    measured = full_cell_body_mask(node.observed | node.remembered, node.map[0].info, .22)
    assert set(second.args[3]) == measured
    assert first.kwargs['blocked_edges'] == second.kwargs['blocked_edges']
    assert first.kwargs['edge_penalty'] == second.kwargs['edge_penalty']
    assert set(second.kwargs['cell_penalty']) >= blocked-measured
    cert = request['certified_start']
    assert cert['max_position_drift_m'] == .003
    assert cert['max_yaw_drift_rad'] == .01
    assert cert['sweep_reserve_m'] == .01
    diag = node.plan_metrics['reverse_probe_diagnostics']
    assert diag['trail_count'] == len(diag['recent_trail']) == 13
    assert diag['candidate']['distance_m'] == pytest.approx(.125)
    distance = request['distance']
    # Use map-projected physical pose, not the unrelated odom frame origin.
    pose = node.plan_metrics['reverse_probe_diagnostics']['current_pose']
    target = (pose[0]-distance*cos(pose[2]), pose[1]-distance*sin(pose[2]))
    assert continuous_raw_body_gap(node, [pose[:2], target]) >= -1e-10


@pytest.mark.parametrize('when', ['primary', 'fallback'])
@pytest.mark.parametrize('change', ['scan', 'manual', 'odom_stop', 'goal', 'observed', 'memory'])
def test_goal_oracle_cannot_borrow_stale_or_changed_motion_evidence(rear_scene, when, change):
    node, _, _ = rear_scene
    node.params['reverse_clearance_neutral_egress_enabled'] = True
    original = node.planner.plan
    count = 0
    def search(*args, **kwargs):
        nonlocal count
        count += 1
        result = original(*args, **kwargs)
        if count == (1 if when == 'primary' else 2):
            if change == 'scan': node.latest_scan_at -= 2.
            elif change == 'manual': node.latest_motion_status['mode'] = 'manual'
            elif change == 'odom_stop': node.odom_samples.clear()
            elif change == 'goal': node.last_goal.header.stamp.nanosec += 1
            elif change == 'observed': node.observed.add((33,87))
            else: node.remembered.add((33,87))
        return result
    with patch.object(node.planner, 'plan', side_effect=search):
        assert not probe(rear_scene)
    assert count >= (1 if when == 'primary' else 2)
    node.recovery_pub.publish.assert_not_called()


def test_neutral_adapter_does_not_enable_preference_waiver_for_focused_front_hit(rear_scene):
    node, data, _ = rear_scene
    node.params['reverse_clearance_neutral_egress_enabled'] = True
    from smart_vision_amr.map_utils import world_to_cell
    x,y,yaw = data['pose']
    cell = world_to_cell(node.map[0].info, x+.6*cos(yaw), y+.6*sin(yaw))
    assert not node.raw_static_grid.is_static_obstacle(cell)
    node.observed.add(cell)
    with patch('smart_vision_amr.astar_planner.select_reverse_egress', wraps=select_reverse_egress) as selector:
        probe(rear_scene, evidence={cell})
    assert selector.call_args.kwargs['blocking_cells'] == {cell}
    assert selector.call_args.kwargs['clearance_neutral_egress_enabled'] is False
