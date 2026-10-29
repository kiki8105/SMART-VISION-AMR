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
                                   {'pose_ok': False}, {'scan_clear': False},
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


@pytest.mark.parametrize('yaw', [None, math.nan, 'unknown', {}, True])
def test_non_numeric_or_invalid_pose_yaw_stops_without_throwing(yaw):
    alignment = MarkerAlignment()
    alignment.start(3, 'rear', 0., 0.)
    alignment.observe(1, detection(.5), 'rear', .1, .1)
    assert alignment.tick(.1, yaw) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.snapshot()['max_seconds'] == 45.


@pytest.mark.parametrize('failure', [{'estop': True}, {'manual_active': True},
                                   {'pose_ok': False}, {'scan_clear': False}])
def test_confirmed_approval_is_immediately_revoked_by_priority_stop(failure):
    alignment = MarkerAlignment()
    confirm(alignment)
    assert alignment.tick(.7, 0., stationary=True, **failure) == 0.
    assert alignment.state == 'FAILED'
    assert alignment.frames == 0
