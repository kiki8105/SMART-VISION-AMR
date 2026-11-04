"""A cold boot is proved once, without waiving either delivery's live gates."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
import run_classroom_staged_validation as staged


def _cycle(start):
    return {
        'passed': True, 'domain': 176, 'localhost_only': True,
        'physical_robot_touched': False, 'mission_accepted_via_web': True,
        'route': [2], 'final_route': [2], 'completed_ids': [2],
        'final_state': 'HOME_REACHED', 'mission_started_at': start,
        'final_observed_at': start + 10., 'dispatch_mode': 'inventory',
        'inventory_completion': {'status': 'DELIVERED'},
        'startup_reset_requested_via_web': True, 'startup_reset_completed': True,
        'prearm': {'passed': True}, 'front_camera_enabled': False,
        'dwell': {'2': {'event_duration_s': 5., 'marker_confirmed': True,
            'alignment_at_dwell': {'state': 'CONFIRMED',
                'soft_center_result': 'centered', 'center_error': .05,
                'rotation_degrees': 10.}}},
        'rear_camera_frames': {'2': {'bytes': 1000, 'unannotated': True,
            'raw_camera_age_s': .1, 'expected_visible': True, 'center_error': .05}},
        'web_ground_truth_checks': {'2': {'passed': True}, 'home': {'passed': True}},
        'web_independent_dwell_checks': {'2': {'passed': True}},
        'home_independent_settle_check': {'passed': True},
    }


def _reentry():
    proof = {'passed': True,
        'before': {'amcl_set_initial_pose': False, 'initial_pose_messages': 0,
            'amcl_messages': 0, 'map_tf_messages': 0, 'web_pose': None,
            'observed_seconds': .75},
        'after': {'amcl_set_initial_pose': False, 'initial_pose_messages': 0,
            'amcl_messages': 2, 'map_tf_messages': 1}}
    first, second = _cycle(10.), _cycle(23.)
    first.update(cold_start_requested=True, cold_start_proof=deepcopy(proof),
        manual_initial_pose={'state': 'VERIFIED',
            'estop_latched_after_verification': True,
            'stop_acknowledged_after_verification': True})
    second['cold_start_requested'] = False
    identity = {'web': {'pid': 101, 'start_ticks': 1001},
                'gazebo_runner': {'pid': 102, 'start_ticks': 1002}}
    return {'passed': True, 'domain': 176, 'localhost_only': True,
        'physical_robot_touched': False, 'cycles': [first, second],
        'same_runtime': True, 'cold_start_proof': proof,
        'runtime_identity_initial': deepcopy(identity),
        'runtime_identity_after_first': deepcopy(identity),
        'runtime_identity_after_second': deepcopy(identity),
        'web_runtime_manifest': {'cold_start': True, 'mode': 'ekf_independent',
            'record_incidents': True},
        'cycle_independent': [{}, {}],
        'reentry': {'manual_pose_state': 'VERIFIED',
            'pose_requested_at': 21., 'pose_verified_at': 22.}}


def _assess(evidence):
    return staged.assess(staged.BY_KEY['19_web_inventory_sector2_same_session_reentry'], evidence)


def test_second_delivery_reuses_proved_cold_boot_only_on_identical_runtime():
    evidence = _reentry()
    before = deepcopy(evidence)
    result = _assess(evidence)
    assert result['passed'], result['reasons']
    assert evidence == before  # No fake cycle 2 cold proof or changed manifest.
    assert 'cold_start_proof' not in evidence['cycles'][1]
    assert evidence['web_runtime_manifest']['cold_start'] is True


@pytest.mark.parametrize('fault', [
    'parent_proof_missing', 'first_proof_missing', 'first_not_cold',
    'proof_mismatch', 'passed_flag_only', 'seeded_before', 'no_post_amcl',
    'no_stop_ack', 'no_observation_window'])
def test_missing_or_non_genuine_first_cold_proof_cannot_exempt_second(fault):
    evidence = _reentry()
    first = evidence['cycles'][0]
    if fault == 'parent_proof_missing':
        evidence.pop('cold_start_proof')
    elif fault == 'first_proof_missing':
        first.pop('cold_start_proof')
    elif fault == 'first_not_cold':
        first['cold_start_requested'] = False
    elif fault == 'proof_mismatch':
        first['cold_start_proof']['after']['amcl_messages'] += 1
    elif fault == 'passed_flag_only':
        evidence['cold_start_proof'] = first['cold_start_proof'] = {'passed': True}
    elif fault == 'no_stop_ack':
        first['manual_initial_pose']['stop_acknowledged_after_verification'] = False
    else:
        proof = deepcopy(evidence['cold_start_proof'])
        if fault == 'seeded_before':
            proof['before']['map_tf_messages'] = 1
        elif fault == 'no_post_amcl':
            proof['after']['amcl_messages'] = 0
        else:
            proof['before'].pop('observed_seconds')
        evidence['cold_start_proof'] = first['cold_start_proof'] = proof
    result = _assess(evidence)
    assert not result['passed']
    assert 'cold_reentry_first_cycle_proof_unverified' in result['reasons']
    assert 'cycle_2:cold_start_without_prior_and_web_initialization_unproven' in result['reasons']


@pytest.mark.parametrize('fault', ['restart_pid', 'reused_pid_new_ticks',
    'first_checkpoint_missing', 'second_checkpoint_missing', 'identity_invalid',
    'same_runtime_false'])
def test_runtime_restart_or_missing_process_proof_keeps_cold_gate(fault):
    evidence = _reentry()
    if fault == 'restart_pid':
        evidence['runtime_identity_after_second']['web']['pid'] += 1
    elif fault == 'reused_pid_new_ticks':
        evidence['runtime_identity_after_second']['gazebo_runner']['start_ticks'] += 1
    elif fault == 'first_checkpoint_missing':
        evidence.pop('runtime_identity_after_first')
    elif fault == 'second_checkpoint_missing':
        evidence.pop('runtime_identity_after_second')
    elif fault == 'identity_invalid':
        for key in ('runtime_identity_initial', 'runtime_identity_after_first',
                    'runtime_identity_after_second'):
            evidence[key]['web']['pid'] = True
    else:
        evidence['same_runtime'] = False
    result = _assess(evidence)
    assert not result['passed']
    assert 'cold_reentry_runtime_identity_unverified' in result['reasons']
    assert 'cycle_2:cold_start_without_prior_and_web_initialization_unproven' in result['reasons']


def test_second_cycle_assessed_alone_still_requires_its_own_cold_boot_proof():
    parent = _reentry()
    second = {**parent['cycles'][1],
              'web_runtime_manifest': parent['web_runtime_manifest']}
    result = staged.assess(staged.Stage('12_web_integrated_clean',
        'standalone Sector 2', (), ('Sector 2', 'Home'), 'web_dwell'), second)
    assert not result['passed']
    assert result['reasons'] == ['cold_start_without_prior_and_web_initialization_unproven']


@pytest.mark.parametrize('fault,reason', [
    ('prearm', 'cycle_2:prearm:evidence_missing'),
    ('home_truth', 'cycle_2:home_gazebo_truth_unverified'),
    ('startup_reset', 'cycle_2:physical_style_web_startup_reset_unverified'),
    ('inventory', 'cycle_2:inventory_delivery_unverified')])
def test_cold_continuation_does_not_waive_second_delivery_gates(fault, reason):
    evidence = _reentry()
    second = evidence['cycles'][1]
    if fault == 'prearm':
        second['prearm']['passed'] = False
    elif fault == 'home_truth':
        second['web_ground_truth_checks']['home']['passed'] = False
    elif fault == 'startup_reset':
        second['startup_reset_completed'] = False
    else:
        second['inventory_completion']['status'] = 'QUEUED'
    result = _assess(evidence)
    assert not result['passed'] and reason in result['reasons']
    assert 'cycle_2:cold_start_without_prior_and_web_initialization_unproven' not in result['reasons']


def test_non_cold_reentry_keeps_existing_assessment_requirements():
    evidence = _reentry()
    evidence['web_runtime_manifest']['cold_start'] = False
    evidence.pop('cold_start_proof')
    for key in ('runtime_identity_initial', 'runtime_identity_after_first',
                'runtime_identity_after_second'):
        evidence.pop(key)
    evidence['cycles'][0].pop('cold_start_proof')
    assert _assess(evidence)['passed']
    evidence['same_runtime'] = False
    assert 'two_deliveries_same_runtime_unverified' in _assess(evidence)['reasons']
