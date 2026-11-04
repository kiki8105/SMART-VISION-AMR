#!/usr/bin/env python3
"""Portable, ROS-free replay of the archived spawn predicate correction."""
import ast
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIXTURE = json.loads((ROOT / 'fixture.json').read_text())
FUNCTIONS = {'segment_box_distance', 'path_clearance_from_box', 'candidate_still_ahead',
             'movement_step_readiness', 'spawn_placement_readiness', 'route_readiness'}


def source_functions(name):
    data = (ROOT / name).read_bytes()
    expected = FIXTURE['code_sha256']['before' if name == 'injector_before.py' else 'after']
    assert hashlib.sha256(data).hexdigest() == expected
    # Execute the exact archived pure function definitions, without importing
    # ROS or a mutable installed workspace. No hand-reimplemented geometry.
    parsed = ast.parse(data)
    selected = ast.Module(body=[node for node in parsed.body
                               if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS],
                          type_ignores=[])
    scope = {'math': math}
    exec(compile(selected, name, 'exec'), scope)
    return scope


def main():
    old = source_functions('injector_before.py')
    new = source_functions('injector_after.py')
    assert hashlib.sha256((ROOT / 'spawn_readiness.patch').read_bytes()).hexdigest() == FIXTURE['code_sha256']['unapplied_patch']
    candidate = FIXTURE['candidate']
    path = FIXTURE['recorded_path']['points']
    radius = FIXTURE['geometry']['robot_radius']
    stamp = FIXTURE['expected_goal_stamp']

    def spawn(robot, age=.05, route=None, current_stamp=stamp, selected=candidate,
              current_path=path):
        return new['spawn_placement_readiness'](
            selected, robot, current_path, age, radius, route_reason=route,
            current_goal_stamp=current_stamp, expected_goal_stamp=stamp)

    actual = []
    for name in ('actual_before', 'actual_after'):
        row = FIXTURE[name]
        xy = row['x'], row['y']
        decisions = []
        for age in (0., .05, .20):
            allowed, metrics = spawn(xy, age)
            assert allowed
            decisions.append({'truth_age_s': age, 'allowed': allowed, 'metrics': metrics})
        actual.append({'input': name, 'wall_time': row['wall_time'], 'robot_xy': xy,
                       'old_selection_range_predicate': old['candidate_still_ahead'](
                           candidate, xy, path, radius),
                       'new_spawn_decisions': decisions})
    assert actual[0]['old_selection_range_predicate']
    assert not actual[1]['old_selection_range_predicate']

    axis_candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    axis_path = [(-2., 0.), (2., 0.)]
    ranges = []
    for distance in (.5999, .60, 1., 1.0001):
        before = old['candidate_still_ahead'](axis_candidate, (-distance, 0.), axis_path, radius)
        after = new['candidate_still_ahead'](axis_candidate, (-distance, 0.), axis_path, radius)
        assert before == after == (.60 <= distance <= 1.)
        ranges.append({'distance_m': distance, 'initial_selection_before': before,
                       'initial_selection_after': after})

    negatives = []
    tests = [
        ('body_overlap', dict(robot=(-.30, 0.), selected=axis_candidate, current_path=axis_path),
         'robot_near_box_sweep'),
        ('clear_body_but_insufficient_reaction_room',
         dict(robot=(-.40, 0.), selected=axis_candidate, current_path=axis_path), 'robot_near_box_sweep'),
        ('actually_behind', dict(robot=(.60, 0.), selected=axis_candidate, current_path=axis_path),
         'candidate_actually_passed_or_not_ahead'),
        ('stale_truth', dict(robot=(-.7, 0.), age=.20001, selected=axis_candidate, current_path=axis_path),
         'truth_unavailable_or_stale'),
        ('path_no_longer_blocked', dict(robot=(-.7, 0.), selected=axis_candidate,
                                      current_path=[(-1., .6), (1., .6)]), 'current_route_no_longer_blocked'),
        ('changed_goal_generation', dict(robot=(-.10724, -.31660), current_stamp=[10, 0]),
         'goal_rearmed_during_preflight'),
        ('not_moving', dict(robot=(-.10724, -.31660), route='navigation_not_moving'),
         'navigation_not_moving'),
    ]
    stale_route = new['route_readiness'](
        True, 'MOVING', (-.25, -1.98), 10., path, 10., 10., 12.01,
        (-.25, -1.98), stamp, stamp)
    tests.append(('route_lease_expired_during_bypass',
                  dict(robot=(-.10724, -.31660), route=stale_route), 'navigation_status_stale'))
    for name, args, expected in tests:
        allowed, metrics = spawn(**args)
        assert not allowed and metrics['reason'] == expected
        negatives.append({'case': name, 'allowed': allowed, 'metrics': metrics})

    report = {
        'passed': True,
        'scope': 'Recorded historical actor-input regression plus controlled negative predicate cases; not a Gazebo or real-robot pass.',
        'original_failure_preserved': FIXTURE['original_actor_failure'],
        'search_path_sha256': FIXTURE['candidate_search_events'][0]['path_sha256'],
        'published_paths_in_preflight': 1,
        'published_path_stamp_ns': FIXTURE['recorded_path']['stamp_ns'],
        'recorded_path_clearance_m': new['path_clearance_from_box'](path, (.44, -.6), radius),
        'path_precision_limitation': FIXTURE['path_hash_note'],
        'source_sha256': FIXTURE['code_sha256'],
        'initial_selection_range_unchanged': ranges,
        'recorded_points': actual,
        'negative_cases': negatives,
        'unchanged_checks': ['same goal and fresh MOVING route lease', '>1cm genuine route obstruction',
                             'ahead along tangent <=-.25m', 'exact25cm square versus selected22cm circular body',
                             '2cm + .22m/s*(.20s reaction + truth age)', 'truth age <=.20s',
                             'existing static/observed/remembered bypass proof', 'fixed center and movement side',
                             'four later box-movement sweep checks and live2cm QA'],
        'actual_validated_trial_result': 'NOT_EVALUATED_HERE; root records the later actual trial separately.'}
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
