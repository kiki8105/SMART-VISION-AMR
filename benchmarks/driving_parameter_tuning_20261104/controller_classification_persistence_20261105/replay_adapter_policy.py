#!/usr/bin/env python3
"""Recorded map/entry through the exact current adapter methods, without ROS."""
import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace


def replay():
    folder = Path(__file__).resolve().parent
    repo = folder.parents[2]
    audit = folder.parent/'recovery_replacement_validation'/'clean_home_failure_audit'
    fixture = json.loads((audit/'fixture.json').read_text())
    sys.path.insert(0, str(repo/'ros2_ws/src/smart_vision_amr'))
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.failure_memory import FailureMemory
    from smart_vision_amr.map_utils import occupancy_to_grid, yaw_from_quaternion
    from smart_vision_amr.recovery_episode import RecoveryCoordinator, RecoveryPhase
    from smart_vision_amr.reverse_recovery import GridGeometry
    reader_spec = importlib.util.spec_from_file_location('recorded_map_reader', audit/'replay_failure_edges.py')
    reader = importlib.util.module_from_spec(reader_spec); reader_spec.loader.exec_module(reader)
    w, h, pixels = reader.read_p5(audit/fixture['map']['image'])
    conversion = {int(k): v for k, v in fixture['map']['pixel_mapping'].items()}
    occupancy = [conversion[v] for y in range(h-1, -1, -1) for v in pixels[y*w:(y+1)*w]]
    origin = fixture['map']['origin']
    message = SimpleNamespace(info=SimpleNamespace(width=w, height=h,
        resolution=fixture['map']['resolution'], origin=SimpleNamespace(
            position=SimpleNamespace(x=origin[0], y=origin[1]),
            orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))), data=occupancy)
    grid = occupancy_to_grid(message, 65, 5)
    entry = fixture['failure_entry']
    now = entry['last_seen']+.5
    memory = FailureMemory(**fixture['failure_settings'])
    memory.record_failure(x=entry['x'], y=entry['y'], approach_heading=entry['approach_heading'],
        reason=entry['reason'], now=entry['first_seen'], map_scope=entry['map_scope'],
        goal_scope=entry['goal_scope'])
    source_path = repo/'ros2_ws/src/smart_vision_amr/smart_vision_amr/astar_planner.py'
    source = source_path.read_text()
    definition = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name=='AStarPlannerNode')
    methods = [n for n in definition.body if isinstance(n, ast.FunctionDef)
               and n.name in {'_failure_constraints', '_controller_retry_policy'}]
    assert len(methods)==2
    # Compile the source AST verbatim. Do not import or construct its ROS Node.
    namespace = dict(GridGeometry=GridGeometry, yaw_from_quaternion=yaw_from_quaternion,
        time=SimpleNamespace(monotonic=lambda: now), RecoveryPhase=RecoveryPhase)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),
                 str(source_path), 'exec'), namespace)
    limits = dict(failure_memory_enabled=True, failure_memory_penalty=2., controller_retry_limit=3)
    node = SimpleNamespace(map=(message, grid), failure_memory=memory,
        get_parameter=lambda name: SimpleNamespace(value=limits[name]),
        _memory_scopes=lambda: (entry['map_scope'], entry['goal_scope']),
        recovery_coordinator=RecoveryCoordinator(), controller_retry_events=set(),
        controller_pending_retry_events={'recorded-controller-terminal'}, plan_metrics={},
        failure_state='CLEAR', failure_reason='', waiting_for_replan=True,
        _invalidate_path=lambda reason: None, publish_status=lambda *args: None)
    legacy_hard, legacy_costs = memory.constraints(grid, GridGeometry(message.info.resolution,*origin),
        now=now, map_scope=entry['map_scope'], goal_scope=entry['goal_scope'])
    hard, costs = namespace['_failure_constraints'](node, grid)
    start, end = tuple(fixture['start']), tuple(fixture['goal'])
    planner = AStarPlanner()
    old = planner.plan(grid,start,end,blocked_edges=legacy_hard,edge_penalty=legacy_costs)
    new = planner.plan(grid,start,end,blocked_edges=hard,edge_penalty=costs)
    wall = {(x,90) for x in range(w)}
    blocked = planner.plan(grid,start,end,wall,blocked_edges=hard,edge_penalty=costs)
    assert (len(legacy_hard),len(legacy_costs),len(old),len(hard),len(costs),len(new),len(blocked)) == (276,740,0,0,1016,33,0)
    assert all(v==2. for v in costs.values())
    namespace['_controller_retry_policy'](node,costs)
    first = dict(node.plan_metrics)
    c=node.recovery_coordinator
    assert c.begin('controlled-physical-cycle',('same-map','same-goal'),{(52,76)},set(),set())
    assert c.local_result(False) and c.stop_proved() and c.reverse_verdict(False)
    node.controller_pending_retry_events.add('controlled-second-event')
    namespace['_controller_retry_policy'](node,costs)
    assert len(node.controller_retry_events)==1 and node.controller_pending_retry_events
    c.reset()
    node.controller_pending_retry_events.add('controlled-third-event')
    namespace['_controller_retry_policy'](node,costs)
    assert len(node.controller_retry_events)==3 and node.failure_state=='CONTROLLER_HOLD'
    return dict(scope='Exact adapter-method AST on recorded map/entry; no ROS node, discovery, Gazebo or motor command.',
        actual_trial_outcome=fixture['outcome'], observed=0, remembered=0,
        comparison=[dict(policy='archived hard-execution geometry', hard_edges=len(legacy_hard),
                         cost_edges=len(legacy_costs), route_cells=len(old)),
                    dict(policy='current cost-only adapter', hard_edges=len(hard),
                         cost_edges=len(costs), maximum_cost=max(costs.values()),route_cells=len(new))],
        physical_wall_route_cells=len(blocked), unknown_cells_preserved=occupancy.count(-1),
        first_accepted_event_retry_count=first['controller_retry_count'],
        physical_global_defers_new_charge=True, three_controlled_event_state=node.failure_state,
        current_route=new,
        adapter_method_sha256={n.name:hashlib.sha256(ast.get_source_segment(source,n).encode()).hexdigest() for n in methods},
        sha256={str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [source_path,audit/'fixture.json',audit/fixture['map']['image'],folder/'replay_adapter_policy.py']},
        limitations=['Offline path existence is not closed-loop tracking or verified reverse success.',
            'One recorded execution restriction is replayed; additional retry IDs and the physical phase are controlled test inputs.',
            'The cost-only adapter does not replace measured occupancy, start-only geometry, follower Spin, current-source freshness or Nav2/raw guard collision checks.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    rendered=json.dumps(replay(),ensure_ascii=False,indent=2)+'\n'
    if args.output:
        with args.output.open('x') as f:f.write(rendered)
    else:print(rendered,end='')

if __name__=='__main__':main()
