"""Read-only gate instrumentation must preserve planner validation behavior."""
import json
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from tf2_ros import TransformException

from test_planner_failure_memory import FailureHarness, refresh
from test_local_detour_planner import ready
from test_occlusion_reserve_incident import load_incident, FIXTURE
from test_planner_recovery import PlannerHarness, make_goal, make_map
from smart_vision_amr.map_utils import occupancy_to_grid


@pytest.fixture
def node(monkeypatch):
    value = FailureHarness({'steady': 100., 'ros': 100.})
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: value.clock['steady'])
    refresh(value)
    value.tf_buffer.lookup_transform.return_value.header.stamp = deepcopy(
        value.tf_buffer.lookup_transform.return_value.header.stamp)
    return value


def set_odom(value, index, replacement):
    updated = list(value.latest_odom)
    updated[index] = replacement
    value.latest_odom = tuple(updated)


@pytest.mark.parametrize('fault,reason', [
    ('health_missing','health_missing'), ('health_receipt','health_receipt_age'),
    ('health_stamp_bool','health_stamp_invalid'), ('health_source','health_source_age'),
    ('health_level','health_level'), ('health_autonomy','autonomy_not_allowed'),
    ('health_stop','health_stop_latched'), ('odom_missing','odom_missing'),
    ('odom_receipt','odom_receipt_age'), ('odom_source','odom_source_age'),
    ('scan_missing','scan_missing'), ('scan_receipt','scan_receipt_age'),
    ('scan_stamp','scan_stamp_invalid'), ('scan_source','scan_source_age'),
    ('scan_empty','scan_empty'), ('scan_max','scan_range_max'),
    ('scan_min','scan_range_min'), ('scan_angle','scan_angles'), ('scan_coverage','scan_coverage'),
    ('tf_stamp','tf_stamp_invalid'), ('tf_future','tf_source_age'),
    ('tf_stale','tf_source_age'), ('tf_geometry','tf_geometry'),
    ('tf_exception','tf_lookup_or_geometry_exception'),
])
def test_existing_rejected_inputs_still_reject_with_first_failure_reason(node, fault, reason):
    if fault == 'health_missing': node.latest_health_at = None
    elif fault == 'health_receipt': node.latest_health_at = 99.
    elif fault == 'health_stamp_bool': node.localization_health['sent_at_ns'] = True
    elif fault == 'health_source': node.localization_health['sent_at_ns'] -= 1_000_000_000
    elif fault == 'health_level': node.localization_health['level'] = 'warning'
    elif fault == 'health_autonomy': node.localization_health['autonomous_allowed'] = False
    elif fault == 'health_stop': node.localization_health['stop_latched'] = True
    elif fault == 'odom_missing': node.latest_odom = None
    elif fault == 'odom_receipt': set_odom(node, 0, 99.25)
    elif fault == 'odom_source': set_odom(node, 1, 99_000_000_000)
    elif fault == 'scan_missing': node.latest_scan = None
    elif fault == 'scan_receipt': node.latest_scan_at = 99.25
    elif fault == 'scan_stamp': node.latest_scan.header.stamp.sec = 0
    elif fault == 'scan_source': node.latest_scan.header.stamp.sec = 99
    elif fault == 'scan_empty': node.latest_scan.ranges = []
    elif fault == 'scan_max': node.latest_scan.range_max = float('nan')
    elif fault == 'scan_min': node.latest_scan.range_min = -1.
    elif fault == 'scan_angle': node.latest_scan.angle_increment = 0.
    elif fault == 'scan_coverage': node.latest_scan.ranges = [float('nan')]*60
    elif fault == 'tf_stamp': node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 0
    elif fault == 'tf_future':
        node.tf_buffer.lookup_transform.return_value.header.stamp.nanosec = 251_000_000
    elif fault == 'tf_stale': node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 99
    elif fault == 'tf_geometry': node.tf_buffer.lookup_transform.return_value.transform.rotation.w = 0.
    elif fault == 'tf_exception': node.tf_buffer.lookup_transform.side_effect = TransformException('missing')
    assert node._fresh_motion_pose() is None
    diagnostic = node.plan_metrics['motion_pose_diagnostics']
    assert diagnostic['first_failed_reason'] == reason
    assert diagnostic['node_clock_ns'] == 100_000_000_000
    json.dumps(diagnostic, allow_nan=False)


def test_healthy_pose_clears_previous_failure_and_preserves_exact_pose_and_stamp(node):
    node.latest_health_at = None
    assert node._fresh_motion_pose() is None
    refresh(node, x=.7, y=1.3, yaw=.4)
    assert node._fresh_motion_pose() == pytest.approx((.7,1.3,.4))
    diagnostic = node.plan_metrics['motion_pose_diagnostics']
    assert diagnostic['first_failed_reason'] == 'OK'
    assert diagnostic['lookup_error'] == ''
    assert diagnostic['tf_stamp_ns'] == node.last_fresh_motion_pose_stamp_ns == 100_000_000_000
    assert diagnostic['tf_source_age_s'] == 0.
    json.dumps(diagnostic, allow_nan=False)


@pytest.mark.parametrize('age,accepted', [(.749,True),(.75,False)])
def test_odometry_receipt_boundary_is_unchanged(node, age, accepted):
    set_odom(node,0,100.-age)
    assert (node._fresh_motion_pose() is not None) is accepted


@pytest.mark.parametrize('future_ns,accepted', [(250_000_000,True),(250_000_001,False)])
def test_tf_future_boundary_is_unchanged(node, future_ns, accepted):
    node.tf_buffer.lookup_transform.return_value.header.stamp.nanosec = future_ns
    assert (node._fresh_motion_pose() is not None) is accepted
    assert node.plan_metrics['motion_pose_diagnostics']['tf_source_age_s'] == -future_ns/1e9


def test_multiple_bad_inputs_report_same_first_short_circuit_as_before(node):
    node.latest_health_at = None
    node.latest_odom = None
    node.latest_scan = None
    assert node._fresh_motion_pose() is None
    assert node.plan_metrics['motion_pose_diagnostics']['first_failed_reason'] == 'health_missing'


@pytest.fixture
def certificate_scene(monkeypatch):
    value, recorded = load_incident(FIXTURE.parent/'occlusion_reserve_certified_start_20261104.json')
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: value.clock['steady'])
    x,y,yaw = recorded['pose_world']
    ready(value,x=x,y=y,yaw=yaw,scan_frame=value.params['base_frame'])
    # This fixture includes the projector input, not only a raw scan receipt.
    from test_planner_obstacle_hold import visibility
    value.on_visibility(visibility(value,occupied=value.observed))
    value.last_observation_time = value.clock['steady']
    value.planned_world_points = list(value.local_reference_points)
    return value


def test_certificate_instrumentation_preserves_success_and_exact_existing_edge(certificate_scene):
    value = certificate_scene
    value.plan_to(value.last_goal)
    diagnostic = value.plan_metrics['start_certificate_diagnostics']
    assert diagnostic['first_failed_reason'] == 'CERTIFIED'
    assert diagnostic['helper_called']
    assert diagnostic['allowed_edges'] == [[[33,91],[32,91]]]
    assert diagnostic['motion_pose']['first_failed_reason'] == 'OK'
    assert value.planned_world_points
    json.dumps(diagnostic, allow_nan=False)


def test_certificate_instrumentation_records_helper_not_called_for_unavailable_pose(certificate_scene):
    value = certificate_scene
    value.latest_scan_at -= 2.
    value.plan_to(value.last_goal)
    diagnostic = value.plan_metrics['start_certificate_diagnostics']
    assert not diagnostic['helper_called']
    assert diagnostic['first_failed_reason'] == 'fresh_motion_pose_unavailable'
    assert diagnostic['motion_pose']['first_failed_reason'] == 'scan_receipt_age'
    assert not value.planned_world_points


def test_no_path_structured_diagnostic_is_json_and_does_not_change_blocking(certificate_scene):
    value = certificate_scene
    value.params['start_egress_enabled'] = False
    value.plan_to(value.last_goal)
    diagnostic = value.plan_metrics['start_certificate_diagnostics']
    assert diagnostic['first_failed_reason'] == 'disabled'
    assert not diagnostic['helper_called']
    assert not value.planned_world_points
    logs=[call.args[0] for call in value.get_logger().warning.call_args_list
          if call.args and call.args[0].startswith('ASTAR_START_CERTIFICATE_DIAGNOSTICS ')]
    assert logs
    payload=json.loads(logs[-1].split(' ',1)[1])
    assert payload['start_certificate']==diagnostic


@pytest.mark.parametrize('header', [
    None, SimpleNamespace(), SimpleNamespace(stamp=None),
    SimpleNamespace(stamp=SimpleNamespace(sec=10)),
    SimpleNamespace(stamp=SimpleNamespace(sec=True, nanosec=0)),
    SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=float('nan'))),
])
def test_optional_tf_diagnostic_metadata_does_not_change_existing_plan(header, monkeypatch):
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: 100.)
    baseline, value = PlannerHarness(), PlannerHarness()
    for planner in (baseline, value):
        message = make_map()
        planner.map = message, occupancy_to_grid(message)
    baseline.tf_buffer.lookup_transform.return_value.header = SimpleNamespace(
        stamp=SimpleNamespace(sec=10, nanosec=0))
    if header is not None:
        value.tf_buffer.lookup_transform.return_value.header = header
    goal = make_goal()
    baseline.on_goal(goal)
    value.on_goal(goal)
    assert value.last_goal is goal
    assert value.planned_cells == baseline.planned_cells
    assert value.planned_world_points == baseline.planned_world_points
    assert value.path_pub.publish.call_args.args[0] == baseline.path_pub.publish.call_args.args[0]
    diagnostic = value.plan_metrics['start_certificate_diagnostics']
    assert diagnostic['start_tf_stamp_ns'] is None
    json.dumps(diagnostic, allow_nan=False)
