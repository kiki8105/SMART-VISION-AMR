"""Execution history is bounded routing cost; only physical evidence is hard."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock

import pytest
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.failure_memory import FailureMemory
from smart_vision_amr.map_utils import occupancy_to_grid
from smart_vision_amr.recovery_episode import RecoveryPhase
from smart_vision_amr.reverse_recovery import GridGeometry
from test_planner_failure_memory import prepared, refresh, failure_event


AUDIT = (Path(__file__).resolve().parents[4] / 'benchmarks' /
         'driving_parameter_tuning_20261104' / 'recovery_replacement_validation' /
         'clean_home_failure_audit')


def authorize_and_scan(node):
    node.latest_scan.header.frame_id = 'base_scan'
    node.latest_motion_status = {'mode': 'auto', 'available': True,
                                'autonomous_allowed': True, 'estop_latched': False,
                                'reset_pending': False,
                                'sent_at_ns': node.get_clock().now().nanoseconds}
    node.latest_motion_status_at = node.clock['steady']


def seed(node, *, event='controller-A'):
    scopes = node._memory_scopes()
    key = node.failure_memory.record_failure(x=.75, y=1.55, approach_heading=0.,
        reason='controller_aborted', now=node.clock['steady'], map_scope=scopes[0],
        goal_scope=scopes[1], failure_id=event)
    node.execution_failure_segments[key] = [((.55, 1.55), (.77, 1.55))]
    return key


def global_episode(node):
    c = node.recovery_coordinator
    assert c.begin('physical-cycle', node._local_route_key(), {(17, 25)}, set(), set())
    assert c.local_result(False) and c.stop_proved() and c.reverse_verdict(False)
    assert c.current.phase is RecoveryPhase.GLOBAL


def raw_constraints(node):
    info = node.map[0].info
    scopes = node._memory_scopes()
    return node.failure_memory.constraints(node.map[1],
        GridGeometry(info.resolution, info.origin.position.x, info.origin.position.y, 0.),
        now=node.clock['steady'], map_scope=scopes[0], goal_scope=scopes[1])


def test_all_execution_constraints_are_costs_before_and_after_physical_episode(prepared):
    node = prepared
    seed(node)
    original, old_costs = raw_constraints(node)
    assert original
    hard, costs = node._failure_constraints(node.map[1])
    assert not hard and set(costs) == original | old_costs.keys()
    assert all(value == node.params['failure_memory_penalty'] for value in costs.values())
    authorize_and_scan(node)
    node.controller_pending_retry_events.add('accepted-controller-event')
    assert node._controller_retry_policy(costs)[0]
    assert node.plan_metrics['controller_retry_count'] == 1
    original_entry = node.failure_memory.snapshot(now=100.)['entries'][0]
    global_episode(node)
    node.observed = {(17, 25)}
    node.controller_pending_retry_events.add('do-not-charge-during-physical-cycle')
    after_hard, after_costs = node._failure_constraints(node.map[1])
    assert not after_hard and after_costs == costs
    node._controller_retry_policy(after_costs)
    assert node.controller_pending_retry_events == {'do-not-charge-during-physical-cycle'}
    assert node.plan_metrics['controller_retry_count'] == 1
    assert node.recovery_coordinator.current.phase is RecoveryPhase.GLOBAL
    assert node.failure_memory.snapshot(now=100.)['entries'][0] == original_entry
    assert (17, 25) in node._live_obstacle_blocked(node.observed)


def test_actual_276_edge_fixture_is_cost_only_and_physical_wall_stays_hard(prepared):
    fixture = json.loads((AUDIT/'fixture.json').read_text())
    spec = importlib.util.spec_from_file_location('read_recorded_failure_map', AUDIT/'replay_failure_edges.py')
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    width, height, pixels = helper.read_p5(AUDIT/fixture['map']['image'])
    mapping = {int(k): v for k,v in fixture['map']['pixel_mapping'].items()}
    message = OccupancyGrid(); message.header.frame_id = 'map'
    message.info.width, message.info.height = width, height
    message.info.resolution = fixture['map']['resolution']
    message.info.origin.position.x, message.info.origin.position.y = fixture['map']['origin'][:2]
    message.info.origin.orientation.w = 1.
    message.data = [mapping[value] for y in range(height-1, -1, -1)
                    for value in pixels[y*width:(y+1)*width]]
    node = prepared
    grid = occupancy_to_grid(message, 65, 5)
    node.map = (message, grid)
    node.failure_memory = FailureMemory(**fixture['failure_settings'])
    entry = fixture['failure_entry']
    scopes = node._memory_scopes()
    node.failure_memory.record_failure(x=entry['x'], y=entry['y'],
        approach_heading=entry['approach_heading'], reason=entry['reason'],
        now=100., map_scope=scopes[0], goal_scope=scopes[1])
    legacy_hard, legacy_cost = raw_constraints(node)
    assert len(legacy_hard) == 276 and len(legacy_cost) == 740
    hard, costs = node._failure_constraints(grid)
    assert hard == set() and len(costs) == 1016
    assert all(value == 2. for value in costs.values())
    start, end = tuple(fixture['start']), tuple(fixture['goal'])
    assert not AStarPlanner().plan(grid, start, end, blocked_edges=legacy_hard, edge_penalty=legacy_cost)
    route = AStarPlanner().plan(grid, start, end, blocked_edges=hard, edge_penalty=costs)
    assert len(route) == 33 and route[0] == start and route[-1] == end
    assert all(not grid.is_static_obstacle(cell) for cell in route)
    # Same immutable map/unknown/static-body model plus a real full-width
    # obstacle. Execution costs cannot manufacture a path through that wall.
    wall = {(x, 90) for x in range(width)}
    assert not AStarPlanner().plan(grid, start, end, wall, blocked_edges=hard, edge_penalty=costs)


def test_missing_historical_segment_does_not_add_a_second_dispatch_policy(prepared):
    node = prepared
    key = seed(node); authorize_and_scan(node)
    node.execution_failure_segments.clear()
    node.controller_pending_retry_events.add('accepted-without-archived-segment')
    hard, costs = node._failure_constraints(node.map[1])
    assert not hard and costs
    node.path_pub.publish.reset_mock()
    node.plan_to(node.last_goal)
    assert node.failure_state != 'SENSOR_HOLD'
    assert any(call.args[0].poses for call in node.path_pub.publish.call_args_list)
    assert node.controller_retry_events == {'accepted-without-archived-segment'}
    assert node.failure_memory.snapshot(now=100.)['entries'][0]['id'] == key


def test_historical_raw_transform_is_not_a_controller_retry_gate(prepared):
    node = prepared
    seed(node); authorize_and_scan(node)
    node.controller_pending_retry_events.add('accepted-terminal')
    normal = node.tf_buffer.lookup_transform.return_value
    def lookup(target, source, stamp):
        if source == 'base_scan':
            raise TransformException('historical source transform unavailable')
        return normal
    node.tf_buffer.lookup_transform.side_effect = lookup
    node.path_pub.publish.reset_mock()
    node.plan_to(node.last_goal)
    assert node.failure_state != 'SENSOR_HOLD'
    assert any(call.args[0].poses for call in node.path_pub.publish.call_args_list)
    assert node.controller_retry_events == {'accepted-terminal'}
    assert not node._failure_constraints(node.map[1])[0]


def test_side_object_outside_actual_body_corridor_cannot_bypass_retry_budget(prepared):
    node = prepared
    seed(node); authorize_and_scan(node)
    node.observed = {(7, 19)}  #40cm to the side of the attempted22cm sweep.
    blockers, _ = node._terminal_obstacle_blockers(((.55, 1.55), (.77, 1.55)))
    assert not blockers and node.recovery_coordinator.current is None
    node.controller_pending_retry_events.update({'first', 'second', 'third'})
    node._controller_retry_policy(node._failure_constraints(node.map[1])[1])
    assert node.controller_retry_events == {'first', 'second', 'third'}
    assert node.failure_state == 'CONTROLLER_HOLD'


def test_three_owned_events_hold_even_when_directed_hard_constraints_are_empty(prepared):
    node = prepared
    seed(node); authorize_and_scan(node)
    hard, costs = node._failure_constraints(node.map[1])
    assert not hard
    for i in range(3):
        node.controller_pending_retry_events.add('owned-event-'+str(i))
        node._controller_retry_policy(costs)
    assert len(node.controller_retry_events) == 3
    assert node.failure_state == 'CONTROLLER_HOLD'
    assert node.last_goal is not None and not node.waiting_for_replan
    node._controller_retry_policy(costs)
    assert len(node.controller_retry_events) == 3


def test_new_coalesced_failure_and_physical_association_are_still_soft_costs(prepared):
    node = prepared
    key = seed(node)
    node.clock.update(steady=100.1, ros=100.1)
    assert seed(node, event='later-failure') == key
    node.obstacle_failure_cells[key] = {(7, 15)}
    hard, costs = node._failure_constraints(node.map[1])
    assert not hard and costs
    assert node.failure_memory.snapshot(now=100.1)['entries'][0]['occurrences'] == 2


def test_expiry_removes_costs_without_extending_memory_or_retry_budget(prepared):
    node = prepared
    seed(node); node.controller_retry_events.add('accepted-earlier')
    node.clock.update(steady=221., ros=221.)
    assert node._failure_constraints(node.map[1]) == (set(), {})
    assert node.controller_retry_events == {'accepted-earlier'}
