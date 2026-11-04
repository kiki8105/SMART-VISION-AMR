"""Cancellation stop QA shares existing bounds and requires owned motion exits."""
from copy import deepcopy
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import assess_classroom_safety_timeline as qa


def stamp(sec):
    return {'sec': sec, 'nanosec': 0}


def nav(t, status, *, path=10, goal=5, phase=None, message='', **extra):
    details = dict(extra)
    if path is not None:
        details['path_stamp'] = stamp(path)
    if phase is not None:
        details['heading_alignment_phase'] = phase
    return {'t': t, 'kind': 'navigation', 'status': status,
            'goal_stamp': stamp(goal), 'message': message, 'details': details}


def records(*extra, phase='WAIT_STOP'):
    result = [
        {'t': 0., 'kind': 'planner', 'status': 'READY'},
        {'t': .05, 'kind': 'goal', 'stamp_ns': 5_000_000_000},
        {'t': .1, 'kind': 'path', 'count': 3, 'stamp_ns': 10_000_000_000},
        {'t': .9, 'kind': 'scan', 'front_min_m': 2.,
         'front_core_min_m': 2., 'front_core_angle_deg': 0.},
        {'t': .9, 'kind': 'truth', 'x': 0., 'y': 0., 'yaw': 0.},
        {'t': .95, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.},
        nav(1., 'WAIT_CANCEL', phase=phase),
        {'t': 1.05, 'kind': 'truth', 'x': .005, 'y': 0., 'yaw': 0.},
        {'t': 1.05, 'kind': 'cmd_vel', 'linear': .05, 'angular': 0.},
        {'t': 1.15, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': 1.3, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': 0.},
        {'t': 1.7, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': 1.7, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': 0.},
        nav(2., 'MOVING', phase='SPIN'),
        {'t': 2.05, 'kind': 'cmd_vel', 'linear': 0., 'angular': .3},
        {'t': 2.2, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': .4},
    ]
    return sorted(result+list(extra), key=lambda record: record['t'])


def kinds(result):
    return {item['kind'] for item in result['violations']}


def test_bounded_cancel_stop_passes_and_owned_spin_is_outside_hold():
    result = qa.assess(records())
    assert result['passed'], result
    interval, = result['wait_replan_intervals']
    assert interval['start_t'] == 1. and interval['duration_s'] == 1.
    assert interval['stop_sources'] == ['navigation_cancel_wait']
    assert interval['max_heading_change_rad'] == 0.


def test_repeated_and_overlapping_waits_never_refund_stop_clock():
    result = qa.assess(records(
        {'t': 1.1, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        nav(1.2, 'WAIT_CANCEL', phase='WAIT_STOP'),
        {'t': 1.4, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.}))
    assert 'late_nonzero_command_after_wait_replan' in kinds(result)
    assert result['wait_replan_intervals'][0]['start_t'] == 1.


def test_fast_zero_does_not_hide_inertial_travel():
    result = qa.assess(records({'t': 1.8, 'kind': 'truth',
                               'x': .14, 'y': 0., 'yaw': 0.}))
    assert 'excessive_ground_truth_stopping_distance' in kinds(result)


def test_circular_fixture_does_not_reuse_stock_short_front_clearance():
    trace = records()
    scan = next(item for item in trace if item['kind'] == 'scan')
    scan['front_min_m'] = scan['front_core_min_m'] = .30
    assert qa.assess(trace)['passed']  # Original stock front extent is 13.2 cm.
    circle = qa.assess(trace, robot_collision_radius_m=.22,
                       front_laser_to_body_m=.22-(-.064))
    assert 'insufficient_front_stopping_clearance' in kinds(circle)
    assert circle['bounds']['front_laser_to_body_m'] == pytest.approx(.284)
    interval, = circle['wait_replan_intervals']
    assert interval['estimated_front_body_clearance_m'] < .02


def test_circle_front_witness_uses_fixture_sensor_pose_and_rejects_unresolved_frame(tmp_path):
    model = tmp_path/'circle.sdf'
    model.write_text('''<sdf><model name="test"><link name="base_scan">
      <sensor name="laser" type="ray"><pose>-0.064 0 0.121 0 0 0</pose></sensor>
      </link></model></sdf>''')
    assert qa.centered_circle_front_extent(model, .22) == pytest.approx(.284)
    model.write_text(model.read_text().replace('<pose>', '<pose relative_to="unknown">'))
    with pytest.raises(ValueError, match='explicit frame'):
        qa.centered_circle_front_extent(model, .22)


@pytest.mark.parametrize('tilt', ['0.1 0 0', '0 0.1 0'])
def test_circle_front_witness_rejects_tilt_before_planar_composition(tmp_path, tilt):
    model = tmp_path/'tilted.sdf'
    model.write_text(f'''<sdf><model><link name="scan"><pose>0 0 0 {tilt}</pose>
      <sensor type="ray"><pose>-0.064 0 0.121 0 0 0</pose></sensor>
      </link></model></sdf>''')
    with pytest.raises(ValueError, match='three-dimensional'):
        qa.centered_circle_front_extent(model, .22)


def test_circle_front_witness_rejects_two_ray_sensors_in_one_link(tmp_path):
    model = tmp_path/'ambiguous.sdf'
    model.write_text('''<sdf><model><link name="scan">
      <sensor name="a" type="ray"/><sensor name="b" type="ray"/>
      </link></model></sdf>''')
    with pytest.raises(ValueError, match='One known ray sensor'):
        qa.centered_circle_front_extent(model, .22)


def test_action_terminal_alone_cannot_end_physical_stop_interval():
    result = qa.assess(records(nav(1.2, 'FAILED'),
        {'t': 1.8, 'kind': 'truth', 'x': .14, 'y': 0., 'yaw': 0.}))
    assert 'excessive_ground_truth_stopping_distance' in kinds(result)
    assert result['wait_replan_intervals'][0]['duration_s'] == 1.


@pytest.mark.parametrize('late', [
    nav(1.2, 'MOVING', path=9, phase='SPIN'),
    nav(1.2, 'MOVING', goal=4, phase='SPIN'),
    nav(1.2, 'MOVING', message='Following A* route'),
])
def test_old_movement_status_cannot_erase_generic_cancel_wait(late):
    values = records(late, {'t': 1.4, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.},
                     phase=None)
    result = qa.assess(values)
    assert 'late_nonzero_command_after_wait_replan' in kinds(result)
    assert result['wait_replan_intervals'][0]['start_t'] == 1.


def test_new_current_follow_path_ends_generic_wait_but_same_stamp_ack_does_not():
    values = records(nav(1.2, 'MOVING', message='Following A* route'),
        {'t': 1.9, 'kind': 'path', 'count': 3, 'stamp_ns': 11_000_000_000},
        nav(2., 'MOVING', path=11, message='Following A* route'), phase=None)
    values = [x for x in values if not (x['t'] == 2. and
              (x.get('details') or {}).get('heading_alignment_phase') == 'SPIN')]
    result = qa.assess(values)
    assert result['passed'], result
    assert result['wait_replan_intervals'][0]['duration_s'] == 1.


def test_reverse_wait_is_not_motion_and_only_owned_actual_start_exits():
    request = {'goal_id': '5:0', 'episode_id': 'owned', 'source_path_stamp': stamp(10)}
    values = records({'t': 1.1, 'kind': 'recovery_request', 'payload': request},
        nav(1.2, 'RECOVERING', path=None, episode_id='owned',
            message='Waiting for FollowPath to stop before checked reverse'),
        {'t': 1.9, 'kind': 'planner', 'status': 'WAIT_REPLAN',
         'goal_stamp': stamp(5), 'details': {'recovery_episode_id': 'owned', 'recovery_phase': 'REVERSING'}},
        nav(2., 'RECOVERING', path=None, episode_id='owned',
            message='Nav2 collision-checked reverse started'),
        {'t': 2.1, 'kind': 'cmd_vel', 'linear': -.06, 'angular': 0.})
    values = [x for x in values if not (x['t'] == 2. and
              (x.get('details') or {}).get('heading_alignment_phase') == 'SPIN')]
    result = qa.assess(values)
    assert result['passed'], result
    assert result['wait_replan_intervals'][0]['duration_s'] == 1.
    assert result['recovery_transitions'][0]['source'] == 'navigation'


def test_old_reverse_cannot_refresh_exemption_after_new_goal_and_empty_path():
    request = {'goal_id': '5:0', 'episode_id': 'old', 'source_path_stamp': stamp(10)}
    values = records({'t': 1.1, 'kind': 'recovery_request', 'payload': request},
        {'t': 1.9, 'kind': 'planner', 'status': 'WAIT_REPLAN', 'goal_stamp': stamp(5),
         'details': {'recovery_episode_id': 'old', 'recovery_phase': 'REVERSING'}},
        nav(2., 'RECOVERING', path=None, episode_id='old',
            message='Nav2 collision-checked reverse started'),
        {'t': 2.4, 'kind': 'goal', 'stamp_ns': 6_000_000_000},
        {'t': 2.45, 'kind': 'planner', 'status': 'PLANNING', 'goal_stamp': stamp(6)},
        {'t': 2.46, 'kind': 'path', 'count': 0, 'stamp_ns': 12_000_000_000},
        nav(2.5, 'RECOVERING', path=None, episode_id='old',
            message='Nav2 collision-checked reverse started'),
        {'t': 2.8, 'kind': 'cmd_vel', 'linear': -.06, 'angular': 0.})
    values = [x for x in values if not (x['t'] == 2. and
              (x.get('details') or {}).get('heading_alignment_phase') == 'SPIN')]
    result = qa.assess(values)
    assert 'motion_during_unverified_planning' in kinds(result)


def test_new_current_goal_wait_replaces_owner_without_resetting_anchor():
    values = records({'t': 1.1, 'kind': 'goal', 'stamp_ns': 6_000_000_000},
        {'t': 1.15, 'kind': 'path', 'count': 3, 'stamp_ns': 11_000_000_000},
        nav(1.2, 'WAIT_CANCEL', goal=6, path=11, phase='WAIT_STOP'),
        nav(2., 'MOVING', goal=6, path=11, phase='SPIN'))
    values = [x for x in values if not (x['t'] == 2. and
              (x.get('goal_stamp') or {}).get('sec') == 5)]
    result = qa.assess(values)
    assert result['passed'], result
    interval, = result['wait_replan_intervals']
    assert interval['start_t'] == 1. and interval['duration_s'] == 1.


def test_old_episode_cannot_end_new_same_goal_recovery_wait():
    values = records({'t': 1.1, 'kind': 'recovery_request',
        'payload': {'goal_id': '5:0', 'episode_id': 'old', 'source_path_stamp': stamp(10)}},
        {'t': 1.15, 'kind': 'planner', 'status': 'WAIT_REPLAN', 'goal_stamp': stamp(5),
         'details': {'recovery_episode_id': 'new', 'recovery_phase': 'STOP_PROOF'}},
        nav(1.2, 'RECOVERING', path=None, episode_id='old',
            message='Nav2 collision-checked reverse started'),
        {'t': 1.4, 'kind': 'cmd_vel', 'linear': -.06, 'angular': 0.})
    result = qa.assess(values)
    assert 'late_nonzero_command_after_wait_replan' in kinds(result)
    assert result['counts']['recovery_exempt_samples'] == 0


def test_missing_stop_context_is_explicitly_unverified_not_success():
    values = [x for x in records() if not (x['kind']=='navigation' and x['t']==1.)]
    values.append(nav(1., 'WAIT_CANCEL', path=None, phase='WAIT_STOP'))
    result = qa.assess(sorted(values,key=lambda x:x['t']))
    assert not result['passed']
    assert 'navigation_stop_ownership_unverified' in result['missing_evidence']


def test_actual_lag_cancel_window_uses_original_entry_truth_and_dynamic_bound():
    source = ROOT/'benchmarks/driving_parameter_tuning_20261104/stop_wait_qa_20261105/actuator_lag_wait_stop_fixture.json'
    fixture = json.loads(source.read_text())
    result = qa.assess(fixture['records'], max_linear_speed_mps=.22,
                       decel_limit_mps2=.4, robot_collision_radius_m=.22)
    assert fixture['original_assessor_passed'] is True
    assert 'excessive_ground_truth_stopping_distance' in kinds(result)
    interval, = result['wait_replan_intervals']
    assert interval['start_t'] == pytest.approx(26.0381)
    assert interval['duration_s'] == pytest.approx(28.0873-26.0381, abs=1e-4)
    assert interval['max_displacement_m'] == pytest.approx(.1442, abs=1e-4)
    assert interval['ground_truth_stopping_distance_bound_m'] == .1155
