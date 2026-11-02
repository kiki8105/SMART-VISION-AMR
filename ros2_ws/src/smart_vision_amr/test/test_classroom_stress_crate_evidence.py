"""A fixed unmapped box needs independent laser, path, and truth evidence."""

import math
import copy
import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import assess_classroom_stress_crate as assessment
import capture_classroom_stress_crate as capture


def complete_web_evidence():
    return {'passed': True, 'mission_accepted_via_web': True,
            'marker_targets': {'2': [1.525, -2.65, 1.54]},
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
        {'kind': 'scan_crate', 'at': 2.02, 'truth_age_s': .08,
         'matching_beams': 3},
        {'kind': 'observed', 'at': 2.04, 'map_ready': True,
         'nearby_indices': [120, 121],
         'observed_indices': [120, 121, 222], 'nearby_count': 2},
        {'kind': 'goal', 'at': 2., 'x': 1.525, 'y': -2.65},
        {'kind': 'path', 'at': 2.1, 'count': 3,
         'points': [[1., .5], [1., -1.], [1.525, -2.65]]},
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


def dense_safe_trace():
    """Enough real-valued truth ticks to make one callback gap <0.5%."""
    records = safe_trace()
    dense = [record for record in records if record.get('kind') != 'truth']
    for record in (item for item in records if item.get('kind') == 'truth'):
        for substep in range(10):
            sample = copy.deepcopy(record)
            sample['at'] = round(record['at'] + substep*.01, 4)
            dense.append(sample)
    return sorted(dense, key=lambda record: record.get('at', 0.))


def test_one_bracketed_stale_link_tick_keeps_full_mission_witness():
    records = dense_safe_trace()
    truth = [record for record in records if record.get('kind') == 'truth']
    truth[123]['crate'] = None
    truth[123]['crate_link_age_s'] = .54
    verdict = assessment.assess(records, complete_web_evidence())
    assert verdict['passed'], verdict
    assert verdict['metrics']['interpolated_link_samples'] == 1
    assert verdict['metrics']['direct_link_fraction'] >= .995


def test_stale_link_cannot_hide_absent_or_displaced_crate():
    for mutate in (
        lambda t: t[123].update(crate_model_present=False, crate=None,
                                crate_link_age_s=.54),
        lambda t: t[123].update(crate=[.37, -.6, 0.],
                                crate_link_age_s=.05),
        lambda t: [t[index].update(crate=None, crate_link_age_s=.54)
                   for index in (123, 124, 125)],
    ):
        records = dense_safe_trace()
        mutate([record for record in records if record.get('kind') == 'truth'])
        verdict = assessment.assess(records, complete_web_evidence())
        assert not verdict['passed']
        assert 'fixed_crate_runtime_presence_unproven' in verdict['reasons']


def test_unbracketed_link_gap_and_long_truth_gap_fail_closed():
    records = dense_safe_trace()
    truth = [record for record in records if record.get('kind') == 'truth']
    truth[-1]['crate'] = None
    truth[-1]['crate_link_age_s'] = .54
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'fixed_crate_runtime_presence_unproven' in verdict['reasons']

    records = dense_safe_trace()
    records[:] = [record for record in records if record.get('kind') != 'truth'
                  or not 2.0 < record['at'] < 2.9]
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'gazebo_truth_sampling_insufficient' in verdict['reasons']

    # Link callbacks can be stale without creating a robot-truth gap. A
    # too-wide pose interpolation is still insufficient evidence of the box.
    records = dense_safe_trace()
    last = max(record['at'] for record in records
               if record.get('kind') == 'truth')
    model = next(record for record in records if record.get('kind') == 'truth')
    missing, recovered = copy.deepcopy(model), copy.deepcopy(model)
    missing.update(at=round(last+.41, 4), crate=None, crate_link_age_s=.54)
    recovered.update(at=round(last+.81, 4))
    records.extend((missing, recovered))
    records.sort(key=lambda record: record.get('at', 0.))
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'fixed_crate_runtime_presence_unproven' in verdict['reasons']
    assert verdict['checks']['truth_sampling_sufficient']


def test_segment_between_safe_truth_points_detects_collision():
    truth = [{'at': 1., 'robot': [-.15, -.6, 0.]},
             {'at': 1.1, 'robot': [.85, -.6, 0.]}]
    assert all(assessment.point_rectangle_distance(record['robot'])-
               assessment.BODY_RADIUS_M >= .02 for record in truth)
    assert assessment.conservative_truth_body_clearance(truth) < .02

    records = safe_trace()
    samples = [record for record in records if record.get('kind') == 'truth']
    samples[23]['robot'] = [-.15, -.6, 0.]
    samples[24]['robot'] = [.85, -.6, 0.]
    assert all(assessment.point_rectangle_distance(record['robot'])-
               assessment.BODY_RADIUS_M >= .02 for record in samples)
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'gazebo_body_clearance_below_2cm_or_unproven' in verdict['reasons']


def test_reverse_out_and_back_between_truth_samples_is_not_assumed_clear():
    # Both sampled poses are safe, but at the mux's 0.25 m/s limit the robot
    # could reverse 9.375 cm toward the box and return within 0.75 seconds.
    # Its body would then violate the required 2 cm clearance unseen.
    truth = [{'at': 1., 'robot': [.8, -.6, 0.]},
             {'at': 1.75, 'robot': [.8, -.6, 0.]}]
    endpoint_clearance = (assessment.point_rectangle_distance(
        truth[0]['robot'], capture.CRATE_CENTER,
        tuple(value+assessment.CRATE_POSE_TOLERANCE_M
              for value in capture.CRATE_HALF_SIZE))-
        assessment.BODY_RADIUS_M)
    assert endpoint_clearance > assessment.REQUIRED_BODY_MARGIN_M
    assert assessment.conservative_truth_body_clearance(truth) < .02
    # A yaw-only curvature allowance would have incorrectly accepted it.
    old_turn_reserve = assessment.MAX_ENVELOPE_LINEAR_MPS*1.*(.75**2)/4.
    assert endpoint_clearance-old_turn_reserve > .02


def test_exact_rectangle_geometry_protects_corner_and_path():
    assert assessment.point_rectangle_distance(capture.CRATE_CENTER) == 0.
    assert math.isclose(assessment.point_rectangle_distance((.55, -.4)),
                        math.sqrt(.02))
    assert assessment.segment_rectangle_distance((0., -.6), (1., -.6)) == 0.
    assert math.isclose(assessment.segment_rectangle_distance(
        (.55, -.4), (.65, -.3)), math.sqrt(.02))
    assert assessment.path_rectangle_clearance(
        [[1., .5], [1., -1.], [1.525, -2.65]]) >= .02
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


def test_permuted_route_certifies_the_sector3_crate_crossing_leg():
    records = safe_trace()
    sector_goal = next(record for record in records if record.get('kind') == 'goal'
                       and record['at'] == 2.)
    sector_goal.update(x=-.25, y=-1.98)
    path = next(record for record in records if record.get('kind') == 'path')
    path['points'][-1] = [-.25, -1.98]
    evidence = complete_web_evidence()
    evidence['marker_targets']['3'] = [-.25, -1.98, -1.4]
    evidence['route'] = evidence['final_route'] = evidence['completed_ids'] = [1, 3, 2]

    verdict = assessment.assess(records, evidence, expected_route=(1, 3, 2))
    assert verdict['passed'], verdict
    assert verdict['metrics']['crossing_leg_marker_id'] == 3
    assert verdict['checks']['sector3_path_after_lidar_is_box_clear']
    assert verdict['checks']['gazebo_truth_bypassed_crate']

    # A route claim cannot be silently accepted under the wrong trial order.
    wrong_order = assessment.assess(records, evidence, expected_route=(1, 2, 3))
    assert not wrong_order['passed']
    assert 'web_sector123_home_completion_unproven' in wrong_order['reasons']

    path['points'] = [[-.1, -.6], [.5, -.6], [-.25, -1.98]]
    unsafe = assessment.assess(records, evidence, expected_route=(1, 3, 2))
    assert not unsafe['passed']
    assert 'sector3_box_aware_safe_path_unproven' in unsafe['reasons']


def test_distant_southbound_route_is_not_called_a_crate_bypass():
    records = safe_trace()
    path = next(record for record in records if record.get('kind') == 'path')
    path['points'] = [[2.0, .5], [2.0, -1.], [1.525, -2.65]]
    for record in records:
        if record.get('kind') == 'truth':
            record['robot'][0] = 2.0
    verdict = assessment.assess(records, complete_web_evidence())
    assert not verdict['passed']
    assert 'crate_crossing_leg_never_near_box' in verdict['reasons']


def test_unsafe_then_changed_safe_path_is_replanned_bypass():
    records = safe_trace()
    records.append({'kind': 'path', 'at': 2.05, 'count': 3,
                    'points': [[.2, .5], [.2, -1.], [1.525, -2.65]]})
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
    path['points'] = [[-.1, -.6], [.5, -.6], [1.525, -2.65]]
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
    next(record for record in records if record.get('kind') == 'path')['at'] = 4.1
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
