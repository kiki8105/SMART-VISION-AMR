"""Actual recorded quantization pinch and independent full-square counterexamples."""
import json
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from smart_vision_amr.grid import Grid
from smart_vision_amr.metric_connector import occupied_rectangles, plan_connector, polyline_clearance
from smart_vision_amr.path_geometry import path_progress, route_cells
from test_local_detour_planner import hits, ready, segment_box_clearance, planner_factory
from test_occlusion_reserve_incident import load_incident
from body_geometry_oracle import continuous_raw_body_gap

SOURCE = (Path(__file__).resolve().parents[4]/'benchmarks/driving_parameter_tuning_20261104'
          /'physical_transfer_20261105/common_coherent_v2_geometry_tracking_inputs.json')


@pytest.fixture
def recorded(monkeypatch):
    node, _ = load_incident()
    # Functional/correlation tests use the explicitly allowed 100ms fixture
    # budget. Production remains 60ms and is timed separately; host scheduling
    # cannot turn a timing miss into a geometry/ownership expectation.
    node.params['local_detour_max_planning_time'] = .10
    source = json.loads(SOURCE.read_text())
    snapshot = source['snapshot']
    pose = snapshot['planner']['details']['start_certificate_diagnostics']['fresh_pose']
    node.observed = {tuple(c) for c in snapshot['observed_snapshot_cells']}
    node.remembered = {tuple(c) for c in snapshot['remembered_snapshot_cells']}
    reference = [tuple(p) for p in source['owned_tracking']['path']['points']]
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = reference[-1]
    node.local_reference_points = reference
    node.local_reference_key = node._local_route_key()
    node.planned_world_points = list(reference)
    info = node.map[0].info
    node.planned_cells = sorted(route_cells([((x-info.origin.position.x)/info.resolution,
                                             (y-info.origin.position.y)/info.resolution) for x,y in reference]))
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    ready(node, x=pose[0], y=pose[1], yaw=pose[2], scan_frame=node.params['base_frame'])
    node._record_start_visibility(node.get_clock().now().nanoseconds, node.clock['steady'])
    node.last_observation_time = node.clock['steady']
    node.path_pub.publish.reset_mock()
    return node, pose


def test_rectangles_preserve_exact_raw_unknown_and_dynamic_union():
    grid = Grid(((False,True,True,False),(False,True,True,False),(False,False,False,False)))
    rect = occupied_rectangles(grid, {(0,2)})
    assert set(map(tuple,rect)) == {(1.,0.,3.,2.),(0.,2.,1.,3.)}
    assert polyline_clearance([(3.5,.5),(3.5,1.5)],rect,4,3) == pytest.approx(.5)
    assert polyline_clearance([(1.5,.5),(1.5,1.5)],rect,4,3) == 0.
    assert polyline_clearance([(-.01,.5),(.5,.5)],rect,4,3) < 0.


def test_recorded_5cm_no_edge_has_complete_bounded_metric_route(recorded):
    node, _ = recorded
    before = set(node.observed), set(node.remembered), node.raw_static_grid.rows
    assert node._try_metric_connector(node._local_configuration(),max_seconds=.10,max_expansions=1500), tuple(
        node.plan_metrics.get(k) for k in ('metric_connector_reason','metric_connector_duration_ms','metric_connector_expansions'))
    assert node.plan_metrics['metric_connector_duration_ms'] <= 100
    assert node.plan_metrics['metric_connector_expansions'] <= 1500
    assert node.plan_metrics['plan_kind'] == 'LOCAL_METRIC_CONNECTOR'
    assert continuous_raw_body_gap(node,node.planned_world_points) >= .02-1e-8
    assert (node.planned_world_points[-1][0],node.planned_world_points[-1][1]) == pytest.approx(
        (node.last_goal.pose.position.x,node.last_goal.pose.position.y))
    assert node.metric_route_scope and not node.active_start_certificate
    assert before == (node.observed,node.remembered,node.raw_static_grid.rows)
    # The old coarse cells really are blocked; this isn't an easy empty world.
    assert node.map[1].is_static_obstacle((36,81))
    assert not node._live_route_blockers(node.observed)
    assert node.metric_route_scope and not node.waiting_for_replan


def test_actual_global_no_certified_edge_uses_metric_connector_without_a_second_goal(recorded):
    node,_=recorded
    goal=node.last_goal
    node.plan_to(goal)
    assert node.last_goal is goal and node.planned_world_points
    assert node.plan_metrics['start_certificate_diagnostics']['allowed_edges']==[]
    assert node.plan_metrics['plan_kind']=='LOCAL_METRIC_CONNECTOR'
    assert continuous_raw_body_gap(node,node.planned_world_points)>=0.
    assert not node._live_route_blockers(node.observed)


def test_actual_stress_corner_has_a_safe_continuous_departure(recorded):
    node,_=recorded
    source=SOURCE.parent.parent/'metric_connector_recorded_20261105/stress_first_no_path.json'
    record=json.loads(source.read_text())
    pose=record['planner']['details']['start_certificate_diagnostics']['fresh_pose']
    node.observed={(i%86,i//86)for i in record['observed_observer']['cells']}
    node.remembered={(i%86,i//86)for i in record['remembered_observer']['cells']}
    node.local_reference_points=[tuple(p)for p in record['last_published_nonempty_path']['points']]
    node.local_reference_key=node._local_route_key()
    ready(node,at=100.1,x=pose[0],y=pose[1],yaw=pose[2],scan_frame=node.params['base_frame'])
    node._record_start_visibility(node.get_clock().now().nanoseconds,node.clock['steady'])
    node.last_observation_time=node.clock['steady']
    # The wall ends at x=.30; treating its top as an infinite line would
    # falsely report overlap. Exact corner distance is 224.070mm.
    from math import hypot
    assert hypot(pose[0]-.30,pose[1]-(-1.40))-.22==pytest.approx(.0040704592964991)
    assert node._try_metric_connector(node._local_configuration(),max_seconds=.10,max_expansions=1500), node.plan_metrics
    assert continuous_raw_body_gap(node,node.planned_world_points)>=0.


def test_same_actual_colliding_input_does_not_repeat_global_search(recorded):
    node,_=recorded
    ready(node,at=100.1,x=.275,y=-1.1837927548023606,
          yaw=-.7422773331592574,scan_frame=node.params['base_frame'])
    node._record_start_visibility(node.get_clock().now().nanoseconds,node.clock['steady'])
    node.last_observation_time=node.clock['steady']
    node.plan_to(node.last_goal)
    assert not node.planned_world_points and node.metric_no_path_key is not None
    count=node.global_plan_count
    for _ in range(3):node.plan_to(node.last_goal)
    assert node.global_plan_count==count


def test_side_probe_does_not_relabel_the_same_forward_anchor(recorded):
    node,_=recorded
    assert not node._try_metric_connector(node._local_configuration(),max_seconds=.06,
                                           max_expansions=1500,after_reverse=True)
    assert not node.path_pub.publish.called


def test_meaningful_yaw_and_verified_space_trail_reopen_same_center_query(recorded):
    node,_=recorded
    x,y=.275,-1.1837927548023606
    def evidence(at,px=x,yaw=-.74):
        ready(node,at=at,x=px,y=y,yaw=yaw,scan_frame=node.params['base_frame'])
        node._record_start_visibility(node.get_clock().now().nanoseconds,node.clock['steady'])
        node.last_observation_time=node.clock['steady']
    evidence(100.1)
    node.plan_to(node.last_goal)
    count=node.global_plan_count
    evidence(100.15,px=x+.0001)
    node.plan_to(node.last_goal)
    assert node.global_plan_count==count  # Source/receipt and sub-mm jitter aren't new scenes.
    evidence(100.2,yaw=-.54)
    node.plan_to(node.last_goal)
    assert node.global_plan_count==count+1
    for i,px in enumerate((x-.025,x-.05,x-.025,x)):
        evidence(100.25+i*.05,px=px,yaw=-.54)
        node._record_breadcrumb()
    node.plan_to(node.last_goal)
    assert node.global_plan_count==count+2  # Real spatial trail, not a newer timestamp alone.
    assert not node.planned_world_points  # The actual raw-wall overlap remains hard.


def test_same_measured_squares_promoted_to_memory_do_not_change_metric_collision(recorded):
    node,_=recorded
    assert node._try_metric_connector(node._local_configuration(),max_seconds=.10,max_expansions=1500)
    original=list(node.planned_world_points)
    node.on_memory(hits(node,node.remembered|node.observed))
    assert node.planned_world_points==original and not node.waiting_for_replan
    assert node.metric_route_scope


@pytest.mark.parametrize('layer',['observed','memory'])
def test_new_full_square_on_current_body_still_blocks(recorded,layer):
    node,pose=recorded
    assert node._try_metric_connector(node._local_configuration(),max_seconds=.10,max_expansions=1500)
    # Actual occupied square contains the robot centre, not just its coarse halo.
    cell=(36,81)
    assert node._changes_block_path({cell},layer=layer)
    assert node._live_route_blockers({cell})=={cell}


def test_stale_inputs_and_manual_authority_cannot_publish_connector(recorded):
    node,pose=recorded
    node.latest_scan_at-=1.
    assert not node._try_metric_connector(node._local_configuration(),max_seconds=.06,max_expansions=1500)
    assert not node.path_pub.publish.called
    ready(node,x=pose[0],y=pose[1],yaw=pose[2],motion={'mode':'manual','autonomous_allowed':False})
    assert not node._try_metric_connector(node._local_configuration(),max_seconds=.06,max_expansions=1500)
    assert not node.path_pub.publish.called


def test_whole_tail_collision_is_never_removed_for_a_safe_first_connector():
    grid=Grid(tuple(tuple(x==12 for x in range(30))for y in range(20)))
    r=plan_connector(grid,set(),(5.5,10.),[[(8.,10.),(20.,10.)]],radius=1.,
                     spacing=.2,margin=.4,max_seconds=.06)
    assert not r.points and r.reason=='no_safe_complete_tail'


def test_body_cannot_cross_a_real_wall_or_unknown_and_map_exterior():
    grid=Grid(tuple(tuple(x==12 for x in range(30))for y in range(20)))
    r=plan_connector(grid,set(),(5.5,10.),[[(20.,10.),(25.,10.)]],radius=1.,
                     spacing=.2,margin=.4,lateral=4.,max_seconds=.06)
    assert not r.points
    assert not plan_connector(grid,set(),(-.1,10.),[[(20.,10.),(25.,10.)]],radius=1.,
                              spacing=.2,margin=.4,max_seconds=.06).points


def test_budget_is_a_hard_bound_not_permission_to_return_partial_path():
    grid=Grid(tuple(tuple(False for x in range(30))for y in range(20)))
    result=plan_connector(grid,set(),(3.,3.),[[(25.,15.),(26.,15.)]],radius=1.,
                          spacing=.2,margin=.4,max_seconds=.06,max_expansions=1)
    assert not result.points and result.reason=='expansion_budget'


def test_time_budget_expiry_never_becomes_cached_geometric_no_path(recorded,monkeypatch):
    node,_=recorded
    ticks=iter(i*.2 for i in range(1000))
    monkeypatch.setattr('smart_vision_amr.metric_connector.perf_counter',lambda: next(ticks))
    node.plan_to(node.last_goal)
    assert node.plan_metrics['metric_connector_reason']=='time_budget'
    assert not node.planned_world_points
    assert getattr(node,'metric_no_path_key',None) is None


def test_missing_acquisition_pair_keeps_sensor_hold_and_goal_not_obstacle_cycle(recorded):
    node,_=recorded
    goal=node.last_goal
    node._clear_start_scan_history()
    node.plan_to(goal)
    assert node.last_goal is goal and node.failure_state=='SENSOR_HOLD'
    assert not node.planned_world_points and node.metric_connector_input_pending
    assert node.recovery_coordinator.current is None
    assert getattr(node,'metric_no_path_key',None) is None


def test_raw_only_hit_then_fresh_paired_free_retries_without_projected_cell_change(recorded):
    node,pose=recorded
    node.params['replan_retry_period']=.25
    observed,memory=set(node.observed),set(node.remembered)
    node._invalidate_path('Test begins awaiting a fresh retained-goal query')
    def frame(at,near_hit):
        ready(node,at=at,x=pose[0],y=pose[1],yaw=pose[2],scan_frame=node.params['base_frame'])
        # Mutate the ORIGINAL message before its receive callback, never a
        # cached source lease. The 10cm raw endpoint is deliberately absent
        # from the unchanged projected layer (e.g. asynchronous classification).
        raw=deepcopy(node.latest_scan)
        node.clock.update(ros=at+.01,steady=at+.01)
        raw.header.stamp=node.get_clock().now().to_msg()
        if near_hit:raw.ranges[0]=.10
        node.tf_buffer.lookup_transform.return_value.header.stamp=raw.header.stamp
        node.on_scan(raw)
        projected=deepcopy(node.map[0]);projected.header.stamp=raw.header.stamp
        projected.data=[0]*(projected.info.width*projected.info.height)
        for x,y in observed:projected.data[y*projected.info.width+x]=100
        node.on_visibility(projected)
        node.on_observed(hits(node,observed))
    frame(100.1,True)
    assert node.plan_metrics['metric_connector_reason']=='raw_scan_body_or_sweep_intersection'
    assert not node.planned_world_points and getattr(node,'metric_no_path_key',None) is None
    first=node.global_plan_count
    frame(100.4,False)
    assert node.global_plan_count>first and node.planned_world_points
    assert node.plan_metrics['metric_connector_reason']=='certified_connector'
    assert node.observed==observed and node.remembered==memory


def test_exact_polyline_geometry_independently_matches_rectangle_oracle():
    rect=np.asarray([(5.,5.,8.,9.),(12.,11.,14.,13.)])
    points=[(3.,4.),(9.,3.),(16.,4.)]
    expected=min(segment_box_clearance(a,b,box)for a,b in zip(points,points[1:])for box in rect)
    expected=min(expected,3.)
    assert polyline_clearance(points,rect,30,30)==pytest.approx(expected)


def test_no_hidden_two_centimetre_padding_in_a_genuinely_narrow_corridor():
    # Original squares bound a 45cm passage for a 44cm-diameter circle.
    # Its exact 5mm side clearance is not silently replaced by a 2cm margin.
    grid=Grid(tuple(tuple(y<10 or y>=19 for x in range(60))for y in range(30)))
    result=plan_connector(grid,set(),(10.,14.5),[[(20.,14.5),(30.,14.5)]],
                          radius=4.4,spacing=.2,margin=0.,max_seconds=.10)
    assert result.points and result.reason=='certified_connector'
    assert polyline_clearance(result.points,occupied_rectangles(grid),60,30)-4.4==pytest.approx(.1)


def test_raw_forward_validation_preserves_short_corner_instead_of_testing_shortcut(planner_factory):
    from math import atan2,hypot
    node=planner_factory()
    node.params.update(start_egress_clearance_margin=0.,local_detour_clearance_margin=0.)
    a,b,c=(.525,1.275),(.580,1.275),(.580,1.330)
    q=(b[0]-.183,b[1]+.183)
    node.clock.update(ros=100.01,steady=100.01)
    raw=deepcopy(node.latest_scan);raw.header.stamp=node.get_clock().now().to_msg()
    raw.angle_min=atan2(q[1]-a[1],q[0]-a[0]);raw.ranges=[float('inf')]*64
    raw.ranges[0]=hypot(q[0]-a[0],q[1]-a[1])
    node.tf_buffer.lookup_transform.return_value.header.stamp=raw.header.stamp
    node.on_scan(raw);node._record_start_visibility(node.get_clock().now().nanoseconds,node.clock['steady'])
    assert node._prefix_sweep_clear([a,b,c])
    assert not node._prefix_sweep_clear([a,c])  # This shortcut is not the delivered route.
    node.planned_world_points=[a,b,c,(3.275,1.275)]
    node.metric_route_scope=(node._local_route_key(),node.raw_static_grid,
                             node.current_path_stamp_ns,node._start_certificate_geometry())
    assert not node._metric_route_blockers({(70,50)})
    assert node.metric_route_scope and node.planned_world_points
    assert not node.waiting_for_replan


@pytest.mark.parametrize('raw_hits_actual_route', [False, True])
def test_clear_suffix_raw_proof_keeps_delivered_corner_and_rejects_real_hit(
        planner_factory, raw_hits_actual_route):
    from math import atan2, hypot
    from unittest.mock import patch
    node = planner_factory()
    node.params.update(start_egress_clearance_margin=0., local_detour_clearance_margin=0.)
    a, b, c = (.525, 1.275), (.580, 1.275), (.580, 1.330)
    reference = [a, b, c, (3.275, 1.330), (3.275, 1.275)]
    node.local_reference_points = list(reference)
    node.local_reference_key = node._local_route_key()
    # In the clear case the raw point is >22cm from the actual L but <22cm
    # from its shortcut. The negative point intersects the second real leg,
    # while the initial circular body is still free.
    q = (c[0], c[1]+.21) if raw_hits_actual_route else (b[0]-.183, b[1]+.183)
    assert hypot(q[0]-a[0], q[1]-a[1]) > node.params['robot_radius']
    node.clock.update(ros=100.01, steady=100.01)
    raw = deepcopy(node.latest_scan)
    raw.header.stamp = node.get_clock().now().to_msg()
    raw.angle_min = atan2(q[1]-a[1], q[0]-a[0])
    raw.ranges = [float('inf')]*64
    raw.ranges[0] = hypot(q[0]-a[0], q[1]-a[1])
    node.tf_buffer.lookup_transform.return_value.header.stamp = raw.header.stamp
    node.on_scan(raw)
    node._record_start_visibility(node.get_clock().now().nanoseconds, node.clock['steady'])
    assert not node._prefix_sweep_clear([a, c])
    if not raw_hits_actual_route:
        assert node._prefix_sweep_clear([a, b, c])
    original_reference = list(node.local_reference_points)
    with patch.object(node, '_prefix_sweep_clear', wraps=node._prefix_sweep_clear) as proof:
        assert node._reuse_clear_forward_suffix() is not raw_hits_actual_route
    actual_prefix = proof.call_args.args[0]
    assert len(actual_prefix) == 3
    for supplied, expected in zip(actual_prefix, (a, b, c)):
        assert supplied == pytest.approx(expected)
    assert node.local_reference_points == original_reference
    assert not node.clear_suffix_inputs_pending
    if raw_hits_actual_route:
        assert node.plan_metrics['start_prefix_validation']['reason'] == 'raw_scan_body_or_sweep_intersection'
        assert not node.path_pub.publish.called
    else:
        path = node.path_pub.publish.call_args.args[0]
        delivered = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        assert delivered == reference == node.planned_world_points
        assert node.plan_metrics['detour_reason'] == 'verified_clear_suffix'
