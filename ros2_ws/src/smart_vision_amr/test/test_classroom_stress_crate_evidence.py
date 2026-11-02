"""A fixed unmapped box needs independent laser, path, and truth evidence."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import assess_classroom_stress_crate as assessment
import capture_classroom_stress_crate as capture


def complete_web_evidence():
    return {'passed': True, 'mission_accepted_via_web': True,
            'marker_targets': {'2': [1.45, -2.65, 1.54]},
            'route': [1, 2, 3], 'final_route': [1, 2, 3],
            'completed_ids': [1, 2, 3], 'final_state': 'HOME_REACHED',
            'home_stationary_verified': True, 'final_observed_at': 4.8,
            'mission_continuity': {'route_preserved': True,
                                   'start_event_count': 1,
                                   'cancel_event_count': 0}}


def safe_trace():
    records = [
        {'kind': 'meta', 'domain': 176, 'localhost_only': True,
         'crate_name': capture.CRATE_NAME,
         'crate_center': list(capture.CRATE_CENTER),
         'crate_size': [.2, .2]},
        {'kind': 'goal', 'at': 1., 'x': 1.2, 'y': .55},
        {'kind': 'scan_crate', 'at': 1.2, 'truth_age_s': .08,
         'matching_beams': 3},
        {'kind': 'observed', 'at': 1.3, 'map_ready': True,
         'nearby_indices': [120, 121],
         'observed_indices': [120, 121, 222], 'nearby_count': 2},
        {'kind': 'goal', 'at': 2., 'x': 1.45, 'y': -2.65},
        {'kind': 'path', 'at': 2.1, 'count': 3,
         'points': [[1., .5], [1., -1.], [1.45, -2.65]]},
    ]
    for step in range(40):
        at = 1.+step*.1
        y = .55 if step < 12 else .55-(step-12)*.06
        records.append({'kind': 'truth', 'at': at,
                        'robot': [1., y, 0.],
                        'crate': list(capture.CRATE_CENTER)+[0.],
                        'crate_model_present': True,
                        'crate_link_age_s': .05})
    return sorted(records, key=lambda record: record.get('at', 0.))


def test_exact_rectangle_geometry_protects_corner_and_path():
    assert assessment.point_rectangle_distance(capture.CRATE_CENTER) == 0.
    assert math.isclose(assessment.point_rectangle_distance((.55, -.4)),
                        math.sqrt(.02))
    assert assessment.segment_rectangle_distance((0., -.6), (1., -.6)) == 0.
    assert math.isclose(assessment.segment_rectangle_distance(
        (.55, -.4), (.65, -.3)), math.sqrt(.02))
    assert assessment.path_rectangle_clearance(
        [[1., .5], [1., -1.], [1.45, -2.65]]) >= .02
    assert assessment.path_rectangle_clearance(
        [[-.2, -.6], [.8, -.6]]) < 0.


def test_raw_scan_ray_matches_only_actual_box_returns():
    assert math.isclose(capture.ray_rectangle_distance(
        (-.2, -.6), (1., 0.)), .45)
    assert capture.ray_rectangle_distance((-.2, -.6), (-1., 0.)) is None
    assert capture.ray_rectangle_distance((-.2, 0.), (1., 0.)) is None
    # Scanner lies 6.4 cm behind the robot center, so a front-facing robot
    # at x=-.136 has its laser at x=-.2 and sees the box at 45 cm.
    scan = SimpleNamespace(ranges=[.45, .45, .45], angle_min=-.01,
                           angle_increment=.01, range_min=.12, range_max=8.)
    matching = capture.scan_crate_matches(scan, (-.136, -.6, 0.))
    assert matching['matching_beams'] >= 2
    scan.ranges = [1., 1., 1.]
    assert capture.scan_crate_matches(scan, (-.136, -.6, 0.))['matching_beams'] == 0


def test_fresh_cells_safe_first_path_actual_crossing_and_home_pass():
    result = assessment.assess(safe_trace(), complete_web_evidence())
    assert result['passed'], result
    assert result['checks']['sector2_path_after_lidar_is_box_clear']
    assert result['checks']['gazebo_truth_bypassed_crate']
    assert result['metrics']['route_reaction_kind'] == 'box_clear_initial_route_after_lidar'
    assert not result['checks']['unsafe_route_replanned_after_lidar']
    assert next(record for record in safe_trace()
                if record.get('kind') == 'observed')['observed_indices'] == [120, 121, 222]


def test_unsafe_then_changed_safe_path_is_replanned_bypass():
    records = safe_trace()
    records.append({'kind': 'path', 'at': 2.05, 'count': 3,
                    'points': [[.2, .5], [.2, -1.], [1.45, -2.65]]})
    result = assessment.assess(records, complete_web_evidence())
    assert result['passed'], result
    assert result['checks']['unsafe_route_replanned_after_lidar']
    assert result['metrics']['route_reaction_kind'] == 'replanned_bypass'


def test_missing_live_crate_or_two_cells_or_raw_return_fails():
    for mutation, reason in (
        (lambda records: next(record for record in records if record.get('kind') ==
                              'observed').update(nearby_indices=[120]),
         'two_fresh_lidar_observed_cells_unproven'),
        (lambda records: next(record for record in records if record.get('kind') ==
                              'scan_crate').update(matching_beams=0),
         'raw_lidar_crate_return_unproven'),
        (lambda records: [record.update(crate=None) for record in records
                          if record.get('kind') == 'truth'],
         'fixed_crate_runtime_presence_unproven'),
    ):
        records = safe_trace()
        mutation(records)
        verdict = assessment.assess(records, complete_web_evidence())
        assert not verdict['passed']
        assert reason in verdict['reasons']


def test_route_through_box_or_truth_near_box_fails_even_if_web_says_home():
    records = safe_trace()
    path = next(record for record in records if record['kind'] == 'path')
    path['points'] = [[-.1, -.6], [.5, -.6], [1.45, -2.65]]
    result = assessment.assess(records, complete_web_evidence())
    assert not result['passed']
    assert 'sector2_box_aware_safe_path_unproven' in result['reasons']

    records = safe_trace()
    truth = next(record for record in records if record['kind'] == 'truth')
    truth['robot'] = [.2, -.35, 0.]
    result = assessment.assess(records, complete_web_evidence())
    assert not result['passed']
    assert 'gazebo_body_clearance_below_2cm_or_unproven' in result['reasons']


def test_no_web_completion_cannot_pass_crate_route():
    evidence = complete_web_evidence()
    evidence['final_state'] = 'RETURNING_HOME'
    result = assessment.assess(safe_trace(), evidence)
    assert not result['passed']
    assert 'web_sector123_home_completion_unproven' in result['reasons']


def test_safe_path_only_after_crate_passage_is_not_bypass_evidence():
    records = safe_trace()
    next(record for record in records if record.get('kind') == 'path')['at'] = 4.8
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'sector2_box_aware_safe_path_unproven' in verdict['reasons']


def test_old_lidar_hit_cannot_certify_later_safe_route():
    records = safe_trace()
    next(record for record in records if record.get('kind') == 'path')['at'] = 3.5
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'sector2_box_aware_safe_path_unproven' in verdict['reasons']


def test_observer_cannot_stop_before_home_and_still_pass():
    records = [record for record in safe_trace()
               if record.get('kind') != 'truth' or record['at'] <= 4.]
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'gazebo_truth_sampling_insufficient' in verdict['reasons']


def test_model_origin_is_not_collision_link_witness():
    records = safe_trace()
    for record in records:
        if record.get('kind') == 'truth':
            record['crate'] = [0., 0., 0.]
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'fixed_crate_runtime_presence_unproven' in verdict['reasons']
