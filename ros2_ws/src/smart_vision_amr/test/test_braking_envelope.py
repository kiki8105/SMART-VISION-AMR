"""Physical brake-envelope regressions, without changing the stop assessor."""
import json
import math
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry

from smart_vision_amr.braking_envelope import BrakingEnvelope
from smart_vision_amr.scan_outlier_guard import swept_body_decision
from test_near_scan_outlier_guard import _MuxHarness, scan


def model():
    return BrakingEnvelope(reaction_seconds=.25)


def test_clear_straight_and_parallel_walls_retain_full_requested_speed():
    points = [(i, x, y) for i, (x, y) in enumerate(
        [(-.5, .235), (0., .235), (.1, .235), (.8, .235), (1., 0.)])]
    scale, result = model().limit(points, .22, 0.)
    assert scale == 1. and not result.collision
    assert result.distance == pytest.approx(.22*.25+.22**2/(2*.15)+.02)
    assert model().limit([], -.10, 0.)[0] == 1.


def test_approach_slowdown_starts_before_old_half_second_raw_sweep():
    points = [(0, .40, 0.)]  # 18 cm body clearance, formerly clear at .22.
    assert not swept_body_decision(points, .22, 0.).swept
    scale, result = model().limit(points, .22, 0.)
    assert .10 < .22*scale < .22
    assert not result.collision
    assert result.distance < .18


def test_common_scaling_preserves_the_checked_curve_and_reverse_direction():
    for v, w, point in [(.15, .3, (.31, .06)), (-.10, -.3, (-.29, .02))]:
        scale, result = model().limit([(0, *point)], v, w)
        assert 0 < scale < 1
        assert (v*scale)/(w*scale) == pytest.approx(v/w)
        assert not result.collision
        assert not model().check([(0, *point)], v*scale, w*scale).collision


def test_zero_request_does_not_erase_measured_forward_stopping_distance():
    cloud = [(0, .37, 0.)]
    assert not model().check(cloud, 0., 0.).collision
    measured = BrakingEnvelope(reaction_seconds=.10).check(cloud, .22, 0.)
    assert measured.collision == {0}
    assert measured.distance > .15


def test_delayed_feedback_yaw_sign_reversal_is_included_in_continuous_tube():
    # Independent review counterexample: the old same-sign bound (.26,-.17)
    # missed actual current (.22,+.15) after source-age .20 seconds.
    cloud = [(0, .18, .228)]
    brake = BrakingEnvelope(reaction_seconds=.10)
    assert not brake.check(cloud, .26, -.17).collision
    actual = brake.check_feedback(cloud, .22, -.01, linear_delta=.04, angular_delta=.16)
    assert actual.collision == {0}
    assert actual.clearance < 0.


def test_signed_velocity_interval_encloses_interior_twists_and_between_time_samples():
    # Independently execute many interior initial twists, not just rectangle
    # corners. Every actual center point from 1ms wheel integration is in the
    # circle sweep reported by the interval tube, including sign crossings.
    brake = BrakingEnvelope(reaction_seconds=.10, stop_reserve=0.)
    points = []
    for v in [-.11, -.045, .023, .097, .13]:
        for w in [-.27, -.13, .017, .11, .23]:
            x, y, yaw = 0., 0., 0.
            left, right = v-w*.287/2, v+w*.287/2
            duration = .10+max(abs(left), abs(right))/.15
            for k in range(math.ceil(duration/.001)):
                t = max(0., (k+.5)*.001-.10)
                a = math.copysign(max(0., abs(left)-.15*t), left)
                b = math.copysign(max(0., abs(right)-.15*t), right)
                x, y, yaw = brake._advance(x, y, yaw, (a+b)/2, (b-a)/.287, .001)
                if k % 73 == 0:
                    # Probe a point just inside the 22cm swept body at random
                    # bearings, so coverage is tested beyond centerline hits.
                    bearing = (k*.17+v*15+w*13)
                    points.append((len(points), x+.219*math.cos(bearing), y+.219*math.sin(bearing)))
    result = brake.check_feedback(points, .01, -.02, linear_delta=.12, angular_delta=.25)
    assert result.collision == {p[0] for p in points}


def test_fresh_parallel_motion_is_not_a_radial_velocity_age_padding():
    brake = BrakingEnvelope(reaction_seconds=.10)
    points = [(i, i*.01, .235) for i in range(-10, 40)]
    result = brake.check_feedback(points, .22, 0., linear_delta=.0075, angular_delta=.04)
    assert not result.collision


def test_current_body_intersection_is_never_cleared_by_slowing_or_moving_away():
    for v, w in [(.22, 0.), (-.10, 0.), (0., .5)]:
        scale, result = model().limit([(0, 0., .219)], v, w)
        assert scale == 0. and result.collision == {0}


def test_independent_wheel_braking_curve_is_covered():
    brake = model()
    v, w = .15, .35
    result = brake.check([], v, w)
    assert result.wheel_curve_bound > 0.
    left, right = v-w*.287/2, v+w*.287/2
    # Independently integrate actual per-wheel .15 braking at 1 kHz, then
    # put raw returns on the resulting center line: every one must collide.
    x, y, yaw = brake._advance(0., 0., 0., v, w, .25)
    points = []
    for i in range(math.ceil(result.wheel_brake_seconds/.001)):
        t = (i+.5)*.001
        a, b = max(0., left-.15*t), max(0., right-.15*t)
        x, y, yaw = brake._advance(x, y, yaw, (a+b)/2, (b-a)/.287, .001)
        if i % 50 == 0:
            points.append((i, x, y))
    assert brake.check(points, v, w).collision == {p[0] for p in points}


def test_vector_brake_sweep_matches_independent_scalar_guard_for_all_motion_directions():
    brake = model()
    points = [(i, x/30, y/30) for i, (x, y) in enumerate(
        ( (x, y) for x in range(-20, 21) for y in range(-20, 21) ))]
    for v, w in [(.22, 0.), (-.10, 0.), (.15, .35), (-.10, .35),
                 (.10, -.35), (-.15, -.35), (0., .5), (.001, .5)]:
        actual = brake.check(points, v, w, longitudinal_uncertainty=.015,
                             lateral_uncertainty=.001, yaw_uncertainty=.02)
        effective = (brake.reaction_seconds+actual.wheel_brake_seconds/2
                     + (brake.stop_reserve/abs(v) if v else 0.))
        scalar = swept_body_decision(points, v*effective, w*effective,
            radius=.22, horizon=1., longitudinal_uncertainty=.015,
            lateral_uncertainty=.001+actual.wheel_curve_bound, yaw_uncertainty=.02)
        assert actual.collision == scalar.swept
        assert actual.clearance == pytest.approx(scalar.minimum_swept_clearance, abs=1.e-12)


@pytest.mark.parametrize('changes', [dict(wheel_deceleration=0.),
    dict(wheel_separation=math.nan), dict(reaction_seconds=-.1), dict(stop_reserve=.1)])
def test_invalid_braking_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        BrakingEnvelope(**changes)


def test_known_slow_plant_stops_with_two_centimeters_remaining_without_global_creep():
    """Controlled plant regression; this is not a Gazebo/physical PASS claim."""
    brake = model()
    x, v = 0., .22
    commands = []
    for _ in range(150):
        scale, _ = brake.limit([(0, .45-x, 0.)], .22, 0.)
        command = .22*scale
        commands.append(command)
        v += max(-.15*.05, min(.15*.05, command-v))
        x += v*.05
        assert .45-x-.22 >= .02-1.e-9
    assert v == 0. and max(commands) > .21
    assert model().limit([(0, 2., 0.)], .22, 0.)[0] == 1.


def test_recorded_lag_coast_is_longer_than_old_sweep_but_covered_by_new_brake_bound():
    root = Path(__file__).resolve().parents[4]
    fixture = json.loads((root/'benchmarks/driving_parameter_tuning_20261104/'
        'stop_wait_qa_20261105/actuator_lag_wait_stop_fixture.json').read_text())
    assert fixture['max_linear_speed_mps'] == .22
    assert fixture['decel_limit_mps2'] == .4  # Original evidence is not rewritten.
    actual_wait_stop_travel = .144193
    old_sweep = .22*.5
    new_bound = model().check([], .22, 0.).distance
    assert old_sweep < actual_wait_stop_travel < new_bound


def odom(node, v=0., w=0., *, stamp=None, frame='odom', child='base_footprint'):
    message = Odometry()
    message.header.frame_id, message.child_frame_id = frame, child
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        node._ros_now if stamp is None else stamp, 1000000000)
    message.twist.twist.linear.x, message.twist.twist.angular.z = v, w
    return message


def brake_mux(monkeypatch):
    node = _MuxHarness()
    node.braking_envelope = model()
    node.measured_braking_envelope = BrakingEnvelope(reaction_seconds=.10, stop_reserve=0.)
    node._braking_feedback = None
    node._braking_last_stamp_ns = -1
    monkeypatch.setattr('smart_vision_amr.motion_mux.time.monotonic', lambda: 100.)
    return node


def test_mux_missing_speed_feedback_cannot_authorize_auto(monkeypatch):
    node = brake_mux(monkeypatch)
    node.on_scan_guard(scan([math.inf]*101))
    assert node.core.scan_guard_code == 'BRAKING_FEEDBACK_UNAVAILABLE'
    assert node.output_pub.publish.call_args.args[0].linear.x == 0.
    node.on_braking_odom(odom(node))
    node._publish_output_locked()
    assert not node.core.scan_guard_hold
    assert node.output_pub.publish.call_args.args[0].linear.x > 0.


def test_mux_zero_and_pivot_requests_still_check_measured_linear_coast(monkeypatch):
    node = brake_mux(monkeypatch)
    node.on_braking_odom(odom(node, .22))
    points = [math.inf]*101
    points[50] = .37
    node.core.receive_auto(0., .5)
    node.on_scan_guard(scan(points))
    assert node.core.scan_guard_code == 'BRAKING_INTERSECTION'
    assert node.output_pub.publish.call_args.args[0].linear.x == 0.
    assert node.output_pub.publish.call_args.args[0].angular.z == 0.
    assert node.core.scan_guard_metrics['braking_measured_distance_m'] > .15


def test_mux_caps_only_approaching_command_and_keeps_clear_aisle_full_speed(monkeypatch):
    node = brake_mux(monkeypatch)
    node.core.receive_auto(.22, 0.)
    node.on_braking_odom(odom(node))
    points = [math.inf]*101
    points[50] = .4
    node.on_scan_guard(scan(points))
    assert not node.core.scan_guard_hold
    assert .10 < node.output_pub.publish.call_args.args[0].linear.x < .22
    node._ros_now += 100000000
    node.on_braking_odom(odom(node))
    node.on_scan_guard(scan([math.inf]*101, node._ros_now))
    assert node.output_pub.publish.call_args.args[0].linear.x == .22


def test_stationary_pivot_does_not_inherit_a_two_centimeter_fictitious_translation(monkeypatch):
    node = brake_mux(monkeypatch)
    node.core.receive_auto(0., .35)
    node.on_braking_odom(odom(node))
    points = [math.inf]*101
    points[50] = .235
    node.on_scan_guard(scan(points))
    assert not node.core.scan_guard_hold
    assert node.output_pub.publish.call_args.args[0].angular.z == .35


@pytest.mark.parametrize('measured_v', [0., .22])
def test_new_pending_raw_object_participates_in_cap_and_measured_stop_tube(monkeypatch, measured_v):
    node = brake_mux(monkeypatch)
    node.core.receive_auto(.22, 0.)
    node.on_braking_odom(odom(node, measured_v))
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-50000000))
    node._ros_now += 100000000
    node.on_braking_odom(odom(node, measured_v))
    pending_stamp = node._ros_now
    node._scan_guard_pose = lambda stamp=0: (None if stamp == pending_stamp else
        ((0., 0., 0.), stamp or node._ros_now))
    ranges = [math.inf]*101
    ranges[50] = .40  # Beyond the old .5s sweep, inside the .22 brake reach.
    node.on_scan_guard(scan(ranges, pending_stamp))
    assert node._scan_guard_pending is not None
    assert node._scan_guard_sample['stamp_ns'] == pending_stamp-150000000
    status = node.core.scan_guard_metrics
    assert status['braking_pending_acquisition_tf_verified'] is False
    assert status['braking_pending_source_stamp_ns'] == pending_stamp
    assert 0. < status['braking_speed_scale'] < 1.
    output = node.output_pub.publish.call_args.args[0]
    if measured_v:
        assert node.core.scan_guard_code == 'BRAKING_INTERSECTION'
        assert output.linear.x == output.angular.z == 0.
    else:
        assert not node.core.scan_guard_hold
        assert 0. < output.linear.x < .22


def test_pending_cloud_keeps_its_mounting_and_source_frame_with_bounded_unknown_motion(monkeypatch):
    node = brake_mux(monkeypatch)
    node.core.max_linear_velocity = .22
    node.core.receive_auto(.22, 0.)
    node._scan_guard_tf_cache['base_scan'] = (-.064, 0., 0.)
    node.on_braking_odom(odom(node, 0.))
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-100000000))
    pending_stamp = node._ros_now-50000000
    node._scan_guard_pose = lambda stamp=0: (None if stamp == pending_stamp else
        ((.04, 0., 0.), stamp or node._ros_now))
    ranges = [math.inf]*101
    ranges[50] = .464  # Known laser mounting maps it to x=.400 at acquisition.
    message = scan(ranges, pending_stamp)
    node.on_scan_guard(message)
    cloud, bounds = node._braking_cloud
    assert cloud[-1][1] == pytest.approx(.4, abs=1.e-7)
    assert cloud[-1][2] == 0.
    assert bounds['longitudinal_uncertainty'] >= .22*.05-1.e-9
    assert bounds['yaw_uncertainty'] >= node.core.max_angular_velocity*.05-1.e-9
    pending = node._scan_guard_pending
    node._ros_now += 50000000
    node.on_braking_odom(odom(node))
    node.on_scan_guard(message)  # Same stamp cannot renew first receipt.
    node._publish_output_locked()
    assert node._scan_guard_pending is pending
    assert node.core.scan_guard_metrics['braking_pending_motion_bound_seconds'] >= .1-1.e-9


def test_explicit_invalid_pending_acquisition_geometry_is_not_bounded_as_a_missing_tf(monkeypatch):
    node = brake_mux(monkeypatch)
    node.on_braking_odom(odom(node))
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-50000000))
    def pose(stamp=0):
        node._scan_guard_pose_lookup = ({'failure_kind': 'geometry'}
            if stamp == node._ros_now else {})
        return None if stamp == node._ros_now else ((0., 0., 0.), stamp or node._ros_now)
    node._scan_guard_pose = pose
    node.on_scan_guard(scan([math.inf]*101))
    assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'
    assert node.last_published_output == (0., 0.)


def test_feedback_duplicate_and_out_of_order_never_extend_receipt(monkeypatch):
    node = brake_mux(monkeypatch)
    node.on_braking_odom(odom(node, .1))
    first = node._braking_feedback
    monkeypatch.setattr('smart_vision_amr.motion_mux.time.monotonic', lambda: 100.2)
    node.on_braking_odom(odom(node, .1))
    assert node._braking_feedback is first and first['received_at'] == 100.
    node.on_braking_odom(odom(node, .1, stamp=node._ros_now-1))
    assert node._braking_feedback is None
    node.on_braking_odom(odom(node, .1))  # Cannot renew the discarded source stamp.
    assert node._braking_feedback is None


def test_ros_clock_reset_requires_a_fresh_new_epoch_feedback_sample(monkeypatch):
    node = brake_mux(monkeypatch)
    node.on_braking_odom(odom(node, .1))
    old_stamp = node._ros_now
    node._ros_now = 50000000000
    node.on_braking_odom(odom(node, .1, stamp=old_stamp))
    assert node._braking_feedback is None
    node.on_braking_odom(odom(node, 0.))
    assert node._braking_feedback['stamp_ns'] == node._ros_now


@pytest.mark.parametrize('changes', [dict(frame='map'), dict(child='other_robot'),
                                   dict(v=math.nan), dict(stamp=99000000000)])
def test_invalid_or_stale_speed_feedback_fails_closed(monkeypatch, changes):
    node = brake_mux(monkeypatch)
    node.on_braking_odom(odom(node, **changes))
    assert node._braking_feedback is None
