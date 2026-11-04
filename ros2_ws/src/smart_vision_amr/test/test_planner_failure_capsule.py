"""A progress abort must not turn nearby side walls into an aisle closure."""
import copy
from math import atan2, cos, hypot, pi, sin
from pathlib import Path

import pytest
from nav_msgs.msg import OccupancyGrid

from smart_vision_amr.map_utils import occupancy_to_grid
from smart_vision_amr.recovery_episode import RecoveryPhase
from smart_vision_amr.reverse_recovery import GridGeometry
from smart_vision_amr.web_mission_server import load_saved_map
from test_planner_failure_memory import prepared, refresh, failure_event, path_points
from test_planner_obstacle_hold import failure_edge_corridor, ready, PHYSICAL_HITS
from test_local_detour_planner import segment_box_clearance


def test_recorded_side_wall_mismatch_does_not_close_only_sector2_aisle(prepared):
    """Replay the real map, directed restriction and first offending scan hits.

    Integrated trial 2026-11-04: the .50 m proximity test included the east
    wall, 35 cm to the side of the actual attempted segment. That created 278
    hard directed edges and no path for ~36 s, despite no dynamic obstacle.
    """
    node = prepared
    metadata, pixels = load_saved_map(
        Path(__file__).resolve().parents[1]/'maps/classroom_20261030_cleaned.yaml')
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height = metadata['width'], metadata['height']
    msg.info.resolution = metadata['resolution']
    msg.info.origin.position.x, msg.info.origin.position.y = -1.65, -5.25
    msg.info.origin.orientation.w = 1.
    msg.data = pixels.flatten().tolist()
    node.params.update(robot_radius=.20, obstacle_inflation_cells=4)
    grid = occupancy_to_grid(msg, 65, 4)
    node.map = msg, grid
    node.raw_static_grid = occupancy_to_grid(msg, 65, 0)
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = 1.525, -2.65
    ready(node, 100.)
    refresh(node, x=1.468, y=-1.2827, yaw=-.8698)
    node.latest_scan.header.frame_id = 'base_scan'
    node.latest_scan.angle_increment = 0.01749303564429283
    node.latest_scan.range_min, node.latest_scan.range_max = .12, 3.5
    node.latest_scan.ranges = [float('inf')]*360
    for index, reading in enumerate((.7165, .6918, .6894, .6612), start=2):
        node.latest_scan.ranges[index] = reading
    body = copy.deepcopy(node.tf_buffer.lookup_transform.return_value)
    laser = copy.deepcopy(body)
    laser.transform.translation.x -= .064*cos(-.8698)
    laser.transform.translation.y -= .064*sin(-.8698)
    node.tf_buffer.lookup_transform.side_effect = (
        lambda target, source, when: laser if source == 'base_scan' else body)
    scopes = node._memory_scopes()
    key = node.failure_memory.record_failure(
        x=1.5018203821712688, y=-1.479776620590288,
        approach_heading=-1.4007988424827031, reason='execution_no_progress',
        now=100., map_scope=scopes[0], goal_scope=scopes[1], failure_id='recorded-stall')
    node.execution_failure_segments[key] = [
        ((1.4863995264339973, -1.277398119746631),
         (1.5235125251471484, -1.4936060058194516))]
    node.controller_pending_retry_events.add('recorded-stall')
    edges, costs = node._failure_constraints(grid)
    assert not edges and len(costs) == 1021
    legacy, legacy_costs = node.failure_memory.constraints(grid,
        GridGeometry(msg.info.resolution, -1.65, -5.25, 0.),
        now=100., map_scope=scopes[0], goal_scope=scopes[1])
    assert len(legacy) == 278 and len(legacy_costs) == 743
    assert legacy <= costs.keys()
    assert not node.planner.plan(grid, (62, 79), (63, 51), blocked_edges=legacy)
    soft, costs = node._controller_retry_policy(costs)
    assert soft
    assert len(node.planner.plan(grid, (62, 79), (63, 51), edge_penalty=costs)) == 29
    assert node.plan_metrics['controller_retry_count'] == 1
    assert node.obstacle_hold.attempts == 0


@pytest.mark.parametrize('yaw', [0., pi/2, -1.4])
def test_unmatched_raw_hit_inside_actual_attempted_capsule_stays_hard(prepared, yaw):
    node = prepared
    ready(node, 100.)
    refresh(node, yaw=yaw)
    node.latest_scan.header.frame_id = 'base_scan'
    node.latest_scan.angle_min = 0.
    node.latest_scan.ranges[0] = .30
    # Preserve exactly the world segment supplied by the follower, including a
    # rotated route. A return .30 m ahead intersects its swept body.
    segment = {'start': {'x': .55, 'y': 1.55},
               'end': {'x': .55+.22*cos(yaw), 'y': 1.55+.22*sin(yaw)},
               'heading': yaw}
    node.on_execution_failure(failure_event(
        node, pose={'x': .55, 'y': 1.55, 'yaw': yaw}, segment=segment))
    assert node.plan_metrics['failure_edge_fallback'] is False
    raw_hit = (.55+.30*cos(yaw), 1.55+.30*sin(yaw))
    cell = (int(raw_hit[0]/.1), int(raw_hit[1]/.1))
    assert cell in node.recovery_coordinator.current.blockers
    phase = node.recovery_coordinator.current.phase
    assert phase in (RecoveryPhase.LOCAL, RecoveryPhase.STOP_PROOF, RecoveryPhase.FOLLOW)
    if phase is RecoveryPhase.LOCAL:
        # A metric candidate can finish inside the bounded search on a fast
        # host. Missing independent projection must retain LOCAL+zero path,
        # rather than pretending that its raw-only obstacle disappeared.
        assert node.metric_connector_input_pending
        assert node.failure_state == 'SENSOR_HOLD'
        assert not node.planned_world_points
        assert node.plan_metrics['start_prefix_validation']['input_pending']
    assert not node.controller_pending_retry_events
    assert not node.controller_retry_events

    # Supply a genuinely newer source and its separately received projection,
    # preserving the original .30m hit and exact acquisition orientation.
    # This must resolve input waiting, not count a permanent hold as success.
    raw = copy.deepcopy(node.latest_scan)
    node.clock.update(steady=100.1, ros=100.1)
    raw.header.stamp = node.get_clock().now().to_msg()
    node.tf_buffer.lookup_transform.return_value.header.stamp = raw.header.stamp
    node.on_scan(raw)
    projected = copy.deepcopy(node.map[0])
    projected.header.stamp = raw.header.stamp
    projected.data = [0]*(projected.info.width*projected.info.height)
    projected.data[cell[1]*projected.info.width+cell[0]] = 100
    node.on_visibility(projected)
    node.retry_transform()
    assert node.recovery_coordinator.current.phase in (
        RecoveryPhase.STOP_PROOF, RecoveryPhase.FOLLOW)
    assert not node.metric_connector_input_pending
    assert cell in node.recovery_coordinator.current.blockers
    if node.planned_world_points:
        # Independent complete published-polyline/circle oracle: a valid
        # local bypass is allowed; a route through the original hit is not.
        required = node.params['robot_radius']
        square = (cell[0]*.1, cell[1]*.1, (cell[0]+1)*.1, (cell[1]+1)*.1)
        for a, b in zip(node.planned_world_points, node.planned_world_points[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length_sq = dx*dx+dy*dy
            t = max(0., min(1., ((raw_hit[0]-a[0])*dx+(raw_hit[1]-a[1])*dy)/length_sq)) if length_sq else 0.
            assert hypot(raw_hit[0]-a[0]-t*dx, raw_hit[1]-a[1]-t*dy) >= required-1.e-9
            assert segment_box_clearance(a, b, square) >= required-1.e-9
    else:
        assert node.recovery_coordinator.current.phase is RecoveryPhase.STOP_PROOF
    assert not node.controller_pending_retry_events
    assert not node.controller_retry_events


def test_missing_attempt_segment_does_not_certify_clear_space(prepared):
    node = prepared
    failure_edge_corridor(node)
    node.execution_failure_segments.clear()
    node.plan_to(node.last_goal)
    assert path_points(node)
    assert node.plan_metrics['failure_edge_fallback'] is True
    assert node.plan_metrics['controller_retry_count'] == 1
    assert not node._failure_constraints(node.map[1])[0]


def test_side_return_at_24cm_does_not_enlarge_20cm_body(prepared):
    node = prepared
    node.params['robot_radius'] = .20
    ready(node, 100.)
    node.latest_scan.header.frame_id = 'base_scan'
    # Endpoint (.70, 1.79): exactly 24 cm to the side of the measured
    # eastbound segment. No mapped wall explains it, but it does not intersect
    # this 20 cm body and must not become a hidden 25 cm hard approach barrier.
    node.latest_scan.angle_min = atan2(.24, .15)
    node.latest_scan.ranges[0] = hypot(.24, .15)
    node.on_execution_failure(failure_event(node))
    assert node.plan_metrics['failure_edge_fallback'] is True
    assert node.plan_metrics['failure_class'] == 'controller'
    assert node.plan_metrics['controller_retry_count'] == 1


def test_transient_physical_blocker_is_separate_from_accepted_controller_retry(prepared):
    node = prepared
    ready(node, 100.)
    node.latest_scan.header.frame_id = 'base_scan'
    node.latest_scan.angle_min = 0.
    node.latest_scan.ranges[0] = .30
    node.controller_retry_events.add('earlier-controller-failure')
    node.on_execution_failure(failure_event(node, event_id='temporary-raw-hit'))
    assert node.recovery_coordinator.current.phase is RecoveryPhase.STOP_PROOF
    assert not node.controller_pending_retry_events
    assert node.controller_retry_events == {'earlier-controller-failure'}
    # Merely losing the raw hit is not a new free-space certificate and must
    # not skip the pending stop/reverse stage or charge a controller retry.
    ready(node, 100.5)
    node.plan_to(node.last_goal)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.STOP_PROOF
    assert node.controller_retry_events == {'earlier-controller-failure'}
    assert not path_points(node)


def test_measured_physical_blocker_does_not_spend_controller_retry_budget(prepared):
    node = prepared
    # These cells intersect the .22 m swept body of the measured attempted
    # segment. The older (10,15) fixture was 23 cm beyond its end and was
    # proximity evidence, rather than an actual collision for this chassis.
    node.observed.update({(8, 15), (8, 16)})
    ready(node, 100.)
    node.latest_scan.header.frame_id = 'base_scan'
    node.on_execution_failure(failure_event(node, event_id='physical-blocker'))
    assert not node.controller_pending_retry_events and not node.controller_retry_events
    assert node.recovery_coordinator.current is not None
    assert node.plan_metrics['failure_class'] != 'controller'
