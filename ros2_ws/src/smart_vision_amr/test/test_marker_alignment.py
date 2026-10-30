"""Rotation-only alignment, bounded search and fresh selected-camera proof."""

import math

import pytest

from smart_vision_amr.marker_alignment import MarkerAlignment


def detection(error=0., marker_id=3, quality=1.):
    return [{'id': marker_id, 'center_x': error, 'confidence': quality}]


def confirm(alignment):
    alignment.start(3, 'rear', 0., 0.)
    for frame, now in enumerate((0., .2, .4, .61)):
        assert alignment.observe(frame, detection(), 'rear', now, now)
        assert alignment.tick(now, 0., stationary=True) == 0.
    assert alignment.state == 'CONFIRMED'


def test_recognition_mode_confirms_offcenter_marker_without_extra_rotation():
    alignment = MarkerAlignment(confirmation_mode='recognized')
    alignment.start(3, 'rear', 0., 0.)
    for frame, now in enumerate((0., .2, .4, .61)):
        assert alignment.observe(frame, detection(.85), 'rear', now, now)
        assert alignment.tick(now, 0., stationary=True) == 0.
    assert alignment.state == 'CONFIRMED'
    assert alignment.snapshot()['confirmation_mode'] == 'recognized'
    assert alignment.snapshot()['center_error'] == .85
    assert alignment.snapshot()['wall_required'] is False
    assert alignment.snapshot()['rotation_degrees'] == 0.
    assert '인식' in alignment.reason


def test_recognition_mode_still_searches_until_correct_marker_and_fresh_stable_proof():
    alignment = MarkerAlignment(confirmation_mode='recognized')
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(marker_id=2), 'rear', .1, .1)
    assert alignment.tick(.1, 0., stationary=True) == .12
    assert alignment.state == 'SEARCHING'
    alignment.observe(2, detection(quality=.2), 'rear', .2, .2)
    assert alignment.tick(.2, 0., stationary=True) == .12
    alignment.observe(3, detection(.9), 'rear', .3, .3)
    assert alignment.tick(.3, 0., stationary=False) == 0.
    assert alignment.frames == 1
    assert not alignment.observe(3, detection(.9), 'rear', .3, .3)
    assert alignment.frames == 1
    for frame, now in ((4, .5), (5, .7), (6, .91)):
        alignment.observe(frame, detection(.9), 'rear', now, now)
        assert alignment.tick(now, 0., stationary=True) == 0.
    assert alignment.state != 'CONFIRMED'  # Stop proof began after the first detection.
    alignment.observe(7, detection(.9), 'rear', 1.11, 1.11)
    assert alignment.tick(1.11, 0., stationary=True) == 0.
    assert alignment.state == 'CONFIRMED'


def test_recognition_mode_revokes_proof_on_wrong_id_stale_image_or_scan_gap():
    for failure in ('wrong_id', 'stale', 'scan'):
        alignment = MarkerAlignment(confirmation_mode='recognized')
        alignment.start(3, 'rear', 0., 0.)
        for frame, now in enumerate((0., .2, .4, .61)):
            alignment.observe(frame, detection(.95), 'rear', now, now)
            alignment.tick(now, 0., stationary=True)
        assert alignment.state == 'CONFIRMED'
        if failure == 'wrong_id':
            alignment.observe(4, detection(marker_id=2), 'rear', .7, .7)
            alignment.tick(.7, 0., stationary=True)
        elif failure == 'stale':
            alignment.tick(1.2, 0., stationary=True)
        else:
            alignment.tick(.7, 0., stationary=True, scan_clear=False)
        assert alignment.state != 'CONFIRMED'
        assert alignment.frames == 0
        if failure == 'wrong_id':
            assert alignment.state == 'SEARCHING'  # Bounded search may continue.
        else:
            assert alignment.angular == 0.


@pytest.mark.parametrize('failure', [{'manual_active': True}, {'estop': True},
                                     {'pose_ok': False}, {'translation_stopped': False}])
def test_recognition_mode_safety_priority_still_fails_closed(failure):
    alignment = MarkerAlignment(confirmation_mode='recognized')
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(.9), 'rear', .1, .1)
    assert alignment.tick(.1, 0., stationary=True, **failure) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.frames == 0


def test_recognition_mode_rejects_wall_requirement_and_ignores_wall_reports():
    with pytest.raises(ValueError):
        MarkerAlignment(confirmation_mode='recognized', require_wall_alignment=True)
    alignment = MarkerAlignment(confirmation_mode='recognized')
    alignment.start(3, 'rear', 0., 0.)
    for frame, now in enumerate((0., .2, .4, .61)):
        alignment.observe(frame, detection(.8), 'rear', now, now)
        assert not alignment.observe_wall({'reason': 'wall unavailable'}, now)
        alignment.tick(now, 0., stationary=True)
    assert alignment.state == 'CONFIRMED'


def test_centered_distinct_frames_need_both_stationary_stability_and_minimum_count():
    alignment = MarkerAlignment()
    alignment.start(3, 'rear', 0., 0.)
    for frame, now in enumerate((0., .2, .4, .61)):
        alignment.observe(frame, detection(), 'rear', now, now)
        alignment.tick(now, 0., stationary=False)
    assert alignment.state == 'SETTLING'
    assert alignment.tick(.62, 0., stationary=True) == 0.
    for frame, now in enumerate((.8, 1., 1.23), start=4):
        alignment.observe(frame, detection(), 'rear', now, now)
        alignment.tick(now, 0., stationary=True)
    assert alignment.state == 'CONFIRMED'
    assert alignment.snapshot()['linear'] == 0.


@pytest.mark.parametrize('camera', ['front', 'rear'])
def test_upright_camera_right_error_commands_negative_robot_yaw(camera):
    alignment = MarkerAlignment()
    alignment.start(3, camera, 0., 0.)
    alignment.observe(1, detection(.6), camera, .1, .1)
    angular = alignment.tick(.1, 0.)
    assert angular < 0.
    assert abs(angular) <= .15
    assert alignment.state == 'CENTERING'
    assert alignment.snapshot()['linear'] == 0.
    alignment.observe(2, detection(-.5), camera, .2, .2)
    assert alignment.tick(.2, -.01) > 0.


def test_opt_in_mirrored_camera_sign_and_no_automatic_rear_inversion():
    alignment = MarkerAlignment(rear_yaw_sign=1.)
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(.5), 'rear', .1, .1)
    assert alignment.tick(.1, 0.) > 0.


def test_blank_fresh_frames_enable_bounded_bouncing_search_only():
    alignment = MarkerAlignment(max_total_rotation=10.)
    alignment.start(3, 'front', 0., 0.)
    assert alignment.tick(0., 0.) == 0.  # No post-arrival frame yet.
    alignment.observe(1, [], 'front', .1, .1)
    assert alignment.tick(.1, 0.) == .12
    alignment.observe(2, [], 'front', .2, .2)
    assert alignment.tick(.2, 1.05) == -.12
    alignment.observe(3, [], 'front', .3, .3)
    assert alignment.tick(.3, -1.05) == .12
    alignment.observe(4, [], 'front', .4, .4)
    assert alignment.tick(.4, 1.05) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.marker_id == 3


def test_wrong_camera_id_low_quality_and_duplicate_frames_cannot_confirm():
    alignment = MarkerAlignment()
    alignment.start(3, 'rear', 0., 1.)
    assert not alignment.observe(1, detection(), 'front', 1.1, 1.1)
    assert not alignment.observe(1, detection(), 'rear', .9, 1.1)
    assert alignment.observe(1, detection(quality=.3), 'rear', 1.1, 1.1)
    assert alignment.state == 'SEARCHING'
    assert alignment.observe(2, detection(marker_id=2), 'rear', 1.2, 1.2)
    assert alignment.frames == 0
    alignment.observe(3, detection(), 'rear', 1.3, 1.3)
    assert not alignment.observe(3, detection(), 'rear', 1.4, 1.4)
    assert alignment.frames == 1
    assert not alignment.observe(4, detection(), 'rear', .1, 1.5)


@pytest.mark.parametrize('failure', [{'manual_active': True}, {'estop': True},
                                   {'pose_ok': False},
                                   {'translation_stopped': False}])
def test_priority_or_safety_failures_stop_and_require_explicit_retry(failure):
    alignment = MarkerAlignment()
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(.5), 'rear', .1, .1)
    assert alignment.tick(.1, 0.) != 0.
    assert alignment.tick(.2, 0., **failure) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.tick(.3, 0.) == 0.
    assert alignment.marker_id == 3


def test_single_clearance_gap_stops_and_recovers_without_resuming_old_proof():
    alignment = MarkerAlignment(require_wall_alignment=True)
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(), 'rear', .1, .1)
    alignment.observe_wall(wall_evidence(1, .1), .1)
    assert alignment.tick(.1, 0., stationary=True) == 0.
    assert alignment.frames == alignment.wall_samples == 1

    assert alignment.tick(.2, 0., stationary=True, scan_clear=False) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.frames == alignment.wall_samples == 0
    assert alignment.center_error is None
    alignment.observe(2, detection(.5), 'rear', .3, .3)
    assert alignment.tick(.3, 0., stationary=True, scan_clear=False) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.angular == 0.

    assert alignment.tick(.4, 0., stationary=True, scan_clear=True) == .12
    assert alignment.state != 'CONFIRMED'
    assert alignment.frames == alignment.wall_samples == 0
    alignment.observe(3, detection(.5), 'rear', .45, .45)
    assert alignment.tick(.45, 0., stationary=True, scan_clear=True) < 0.
    assert alignment.state == 'CENTERING'


def test_sustained_clearance_loss_fails_after_one_second_and_keeps_stop():
    alignment = MarkerAlignment()
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(.5), 'rear', .1, .1)
    assert alignment.tick(.1, 0.) < 0.
    assert alignment.tick(.2, 0., scan_clear=False) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.tick(1.19, 0., scan_clear=False) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.tick(1.2, 0., scan_clear=False) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.tick(1.3, 0., scan_clear=True) == 0.


def test_camera_gap_stops_immediately_but_has_bounded_recovery_grace():
    alignment = MarkerAlignment()
    alignment.start(3, 'front', 0., 0.)
    alignment.observe(1, detection(.5), 'front', 0., 0.)
    assert alignment.tick(.1, 0.) < 0.
    assert alignment.tick(.2, 0., camera_fresh=False) == 0.
    assert alignment.state == 'CENTERING'
    assert alignment.tick(1.21, 0., camera_fresh=False) == 0.
    assert alignment.state == 'FAILED'


def test_confirmed_proof_is_revoked_by_offcenter_wrong_id_or_stale_camera():
    for kind in ('offcenter', 'wrong_id', 'stale'):
        alignment = MarkerAlignment()
        confirm(alignment)
        if kind == 'offcenter':
            alignment.observe(10, detection(.3), 'rear', .8, .8)
            assert alignment.state == 'CENTERING'
        elif kind == 'wrong_id':
            alignment.observe(10, detection(marker_id=2), 'rear', .8, .8)
            assert alignment.state == 'SEARCHING'
        else:
            alignment.tick(1.2, 0., stationary=True)
            assert alignment.state == 'SETTLING'
        assert alignment.frames == 0
        assert alignment.state != 'CONFIRMED'


def test_timeout_and_total_rotation_never_fake_confirmation():
    alignment = MarkerAlignment(timeout=1.)
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, [], 'rear', .9, .9)
    assert alignment.tick(1., 0.) == 0.
    assert alignment.state == 'FAILED'
    bounded = MarkerAlignment(max_total_rotation=.2)
    bounded.start(3, 'front', 0., 0.)
    bounded.observe(1, detection(.9), 'front', .1, .1)
    assert bounded.tick(.1, .21) == 0.
    assert bounded.state == 'FAILED'


@pytest.mark.parametrize('change', [{'search_speed': .3}, {'center_tolerance': 2.},
                                    {'required_frames': 1}, {'timeout': math.nan},
                                    {'rear_yaw_sign': 0.}])
def test_invalid_limits_are_rejected(change):
    with pytest.raises(ValueError):
        MarkerAlignment(**change)


@pytest.mark.parametrize('yaw', [None, math.nan, 'unknown', {}, True, 10**1000])
def test_non_numeric_or_invalid_pose_yaw_stops_without_throwing(yaw):
    alignment = MarkerAlignment()
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(.5), 'rear', .1, .1)
    assert alignment.tick(.1, yaw) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.snapshot()['max_seconds'] == 45.


@pytest.mark.parametrize('failure', [{'estop': True}, {'manual_active': True},
                                   {'pose_ok': False}])
def test_confirmed_approval_is_immediately_revoked_by_priority_stop(failure):
    alignment = MarkerAlignment()
    confirm(alignment)
    assert alignment.tick(.7, 0., stationary=True, **failure) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.frames == 0


def test_confirmed_approval_is_revoked_during_clearance_gap():
    alignment = MarkerAlignment()
    confirm(alignment)
    assert alignment.tick(.7, 0., stationary=True, scan_clear=False) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.frames == 0
    assert alignment.tick(.8, 0., stationary=True, scan_clear=True) == .12
    assert alignment.state != 'CONFIRMED'


def wall_evidence(stamp, now, error=0., reliable=True, reason='single wall'):
    return {'frame_id': stamp, 'observed_at': now, 'heading_error_rad': error,
            'reliable': reliable, 'reason': reason}


def wall_alignment():
    result = MarkerAlignment(require_wall_alignment=True)
    result.start(3, 'rear', 0., 0.)
    return result


def test_wall_required_pixel_center_alone_holds_and_never_confirms():
    alignment = wall_alignment()
    for frame, now in enumerate((0., .2, .4, .61)):
        alignment.observe(frame, detection(), 'rear', now, now)
        assert alignment.tick(now, 0., stationary=True) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.frames >= alignment.required_frames
    assert alignment.snapshot()['wall_required']
    assert '벽' in alignment.reason


def test_wall_and_pixel_need_independent_distinct_stable_samples_and_actual_stop():
    alignment = wall_alignment()
    for frame, now in enumerate((0., .2, .4, .61)):
        alignment.observe(frame, detection(), 'rear', now, now)
        alignment.observe_wall(wall_evidence(frame, now), now)
        assert alignment.tick(now, 0., stationary=False) == 0.
    assert alignment.state == 'SETTLING'
    for frame, now in enumerate((.7, .9, 1.1, 1.31), start=4):
        alignment.observe(frame, detection(), 'rear', now, now)
        alignment.observe_wall(wall_evidence(frame, now), now)
        assert alignment.tick(now, 0., stationary=True) == 0.
    assert alignment.state == 'CONFIRMED'
    assert alignment.snapshot()['linear'] == 0.
    assert alignment.snapshot()['wall_samples'] >= 3


def test_wall_normal_yaw_feedback_is_independent_of_pixel_and_bounded():
    alignment = wall_alignment()
    alignment.observe(1, detection(), 'rear', .1, .1)
    alignment.observe_wall(wall_evidence(1, .1, error=.5), .1)
    assert alignment.tick(.1, 0.) == .08
    assert alignment.state == 'WALL_ALIGNING'
    alignment.observe(2, detection(), 'rear', .2, .2)
    alignment.observe_wall(wall_evidence(2, .2, error=-.4), .2)
    assert alignment.tick(.2, 0.) == -.08
    # Image center gets priority if wall correction moves it away from center.
    alignment.observe(3, detection(.3), 'rear', .3, .3)
    alignment.observe_wall(wall_evidence(3, .3, error=.4), .3)
    assert alignment.tick(.3, 0.) < 0.
    assert alignment.state == 'CENTERING'
    assert alignment.wall_samples == 0


def test_replayed_scan_frames_and_same_acquisition_time_cannot_build_proof():
    alignment = wall_alignment()
    alignment.observe(1, detection(), 'rear', .1, .1)
    assert alignment.observe_wall(wall_evidence(10, .1), .1)
    for frame, now in enumerate((.2, .3, .4), start=2):
        alignment.observe(frame, detection(), 'rear', now, now)
        assert not alignment.observe_wall(wall_evidence(10, .1), now)
        alignment.tick(now, 0., stationary=True)
    assert alignment.wall_samples == 1
    assert not alignment.observe_wall(wall_evidence(11, .1), .4)
    assert alignment.state != 'CONFIRMED'


@pytest.mark.parametrize('problem', [
    {'reliable': False, 'reason': 'corner or mixed surfaces'},
    {'heading_error_rad': math.nan}, {'observed_at': -1.},
    {'observed_at': .8}, {'frame_id': True},
])
def test_unreliable_corner_invalid_or_stale_wall_revokes_confirmation(problem):
    alignment = wall_alignment()
    for frame, now in enumerate((0., .2, .4, .61)):
        alignment.observe(frame, detection(), 'rear', now, now)
        alignment.observe_wall(wall_evidence(frame, now), now)
        alignment.tick(now, 0., stationary=True)
    assert alignment.state == 'CONFIRMED'
    evidence = {**wall_evidence(10, .7), **problem}
    assert not alignment.observe_wall(evidence, .7)
    assert alignment.tick(.7, 0., stationary=True) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.wall_samples == 0


def test_fresh_camera_cannot_mask_stale_scan_or_wall_loss():
    alignment = wall_alignment()
    alignment.observe(1, detection(), 'rear', 0., 0.)
    alignment.observe_wall(wall_evidence(1, 0.), 0.)
    alignment.tick(0., 0., stationary=True)
    alignment.observe(2, detection(), 'rear', .51, .51)
    assert alignment.tick(.51, 0., stationary=True) == 0.
    assert alignment.state == 'HOLD'
    assert alignment.wall_samples == 0


def test_wall_rotation_respects_body_envelope_and_estop_priority():
    alignment = wall_alignment()
    alignment.observe(1, detection(), 'rear', .1, .1)
    alignment.observe_wall(wall_evidence(1, .1, error=.4), .1)
    assert alignment.tick(.1, 1.01) == 0.
    assert alignment.state == 'FAILED'
    alignment = wall_alignment()
    alignment.observe(1, detection(), 'rear', .1, .1)
    alignment.observe_wall(wall_evidence(1, .1, error=.4), .1)
    assert alignment.tick(.1, 0., estop=True) == 0.
    assert alignment.state == 'FAILED'


def test_sensor_sampling_gap_cannot_reuse_previous_stability_duration():
    alignment = wall_alignment()
    for frame, now in enumerate((0., .2, .4)):
        alignment.observe(frame, detection(), 'rear', now, now)
        alignment.observe_wall(wall_evidence(frame, now), now)
        alignment.tick(now, 0., stationary=True)
    # New samples arriving together after an unobserved gap do not prove
    # continuous camera/wall agreement or physical standstill during that gap.
    alignment.observe(10, detection(), 'rear', 1.5, 1.5)
    alignment.observe_wall(wall_evidence(10, 1.5), 1.5)
    assert alignment.tick(1.5, 0., stationary=True) == 0.
    assert alignment.frames == alignment.wall_samples == 1
    assert alignment.state == 'SETTLING'
    assert not alignment.observe(11, detection(), 'rear', 1.5, 1.5)


def test_custom_lower_legacy_velocity_limit_still_constructs_and_caps_wall_servo():
    alignment = MarkerAlignment(search_speed=.03, max_angular_velocity=.04,
                                require_wall_alignment=True)
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(), 'rear', .1, .1)
    alignment.observe_wall(wall_evidence(1, .1, error=.4), .1)
    assert alignment.tick(.1, 0.) == .04
