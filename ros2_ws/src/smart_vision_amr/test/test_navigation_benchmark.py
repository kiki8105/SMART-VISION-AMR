"""Route scoring and execution safeguards without sending ROS goals."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[4]/'scripts/benchmark_navigation.py'
spec = importlib.util.spec_from_file_location('navigation_benchmark', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_cross_track_measures_segments_not_only_waypoints():
    assert module.point_path_distance(1.0, .2, [(0., 0.), (2., 0.)]) == pytest.approx(.2)
    assert module.point_path_distance(3., 1., [(0., 0.), (2., 0.)]) == pytest.approx(2**.5)
    assert module.point_path_distance(0., 0., []) is None


def test_summary_counts_matching_goal_completion_replans_and_blocked_transitions():
    metrics = module.NavigationMetrics()
    metrics.goal(2., 0., 10, 0.)
    metrics.planned([(0., 0.), (2., 0.)], .1)
    metrics.planned([(0., 0.), (2., 0.)], .2)  # Duplicate publisher heartbeat.
    metrics.planned([(0., 0.), (1., .5), (2., 0.)], .3)
    status = {'status': 'WAIT_REPLAN', 'goal_stamp': {'sec': 0, 'nanosec': 10}}
    metrics.status('navigation', status, .4)
    metrics.status('navigation', status, .5)
    metrics.status('navigation', {'status': 'SUCCEEDED', 'goal_stamp': {'nanosec': 9}}, .6)
    assert metrics.active_goal['result'] == 'running'
    metrics.status('navigation', {'status': 'SUCCEEDED', 'goal_stamp': {'nanosec': 10}}, .7)
    report = metrics.summary(1.)
    assert report['completion_rate'] == 1.
    assert report['replans'] == 1 and report['plans'] == 2
    assert report['blocked_transitions'] == 1


def test_successful_goal_cannot_be_overwritten_by_shutdown_cancel_status():
    metrics = module.NavigationMetrics()
    metrics.goal(0.35, -0.59, 123, 0.0)
    stamp = {'sec': 0, 'nanosec': 123}
    metrics.status('navigation', {'status': 'SUCCEEDED', 'goal_stamp': stamp}, 3.0)
    metrics.status('navigation', {'status': 'CANCELED', 'goal_stamp': stamp}, 3.1)
    assert metrics.summary(3.2)['completed_goals'] == 1
    assert metrics.active_goal['result'] == 'succeeded'
    assert metrics.active_goal['ended_seconds'] == 3.0


def test_cleanup_cancels_only_owned_active_goal_and_always_latches_stop_after_route():
    running = {'stamp_ns': 123, 'result': 'running'}
    succeeded = {'stamp_ns': 123, 'result': 'succeeded'}
    assert module.benchmark_cleanup_actions(True, True, running, 123) == (
        'cancel_navigation', 'emergency_stop')
    assert module.benchmark_cleanup_actions(True, True, succeeded, 123) == (
        'emergency_stop',)
    assert module.benchmark_cleanup_actions(True, True, running, 999) == (
        'emergency_stop',)
    assert module.benchmark_cleanup_actions(True, False, running, 123) == ()
    assert module.benchmark_cleanup_actions(False, True, running, 123) == ()
    # A benchmark that never acquired control must leave another operator alone.
    assert module.benchmark_cleanup_actions(True, False, None, None,
                                            auto_owned=False) == ()
    # An ACKed zero-speed handover requires a stop even if no goal was sent.
    assert module.benchmark_cleanup_actions(True, False, None, None,
                                            auto_owned=True) == ('emergency_stop',)


def test_startup_timeout_does_not_abort_an_active_bounded_auto_handover():
    handover = module.AutoModeHandshake('benchmark:wait', timeout_seconds=25.)
    assert not module.benchmark_predeparture_wait_expired(19.9, 0., 20., None)
    assert module.benchmark_predeparture_wait_expired(20.1, 0., 20., None)

    packet, acknowledged = handover.step(mux_status(), 10., 10., 20_000_000_000, True)
    assert packet is not None and not acknowledged
    # Readiness can briefly drop after the 20 s startup deadline. The zero
    # handover has its own deadline and must not start a goal on stale data.
    assert not module.benchmark_predeparture_wait_expired(20.1, 0., 20., handover)
    assert not module.benchmark_predeparture_wait_expired(34.9, 0., 20., handover)
    assert module.benchmark_predeparture_wait_expired(35.1, 0., 20., handover)
    # Each later waypoint gets a fresh, bounded readiness interval.
    assert not module.benchmark_predeparture_wait_expired(45., 40., 20., None)
    assert module.benchmark_predeparture_wait_expired(60.1, 40., 20., None)


def test_stop_fraction_does_not_treat_missing_commands_as_stopped():
    metrics = module.NavigationMetrics()
    metrics.pose(0., 1., 0., 0., 0.)  # No cmd received.
    assert metrics.summary(.1)['zero_command_fraction'] is None
    metrics.command(.1, 0., 0.)
    metrics.pose(.2, 1.2, .01, 0., 0.)
    metrics.command(.3, .1, 0.)
    metrics.pose(.4, 1.4, .02, 0., 0.)
    report = metrics.summary(.5)
    assert report['zero_command_fraction'] == .5
    assert report['fresh_command_sample_fraction'] == pytest.approx(2/3)


def test_tf_jump_is_reported_without_inflating_distance():
    metrics = module.NavigationMetrics()
    metrics.pose(0., 1., 0., 0., 0.)
    metrics.pose(.1, 1.1, .01, 0., 0.)
    metrics.pose(.2, 1.2, 2., 0., 0.)
    report = metrics.summary(.2)
    assert report['tf_pose_jumps'] == 1
    assert report['estimated_distance_m'] == pytest.approx(.01)


def test_command_acceleration_and_jerk_are_finite_and_ignore_bursty_callbacks():
    metrics = module.NavigationMetrics()
    metrics.command(0., 0., 0.)
    metrics.command(.1, .01, .02)
    metrics.command(.2, .03, .03)
    metrics.command(.200001, .03, .03)
    report = metrics.summary(.3)
    assert report['linear_acceleration_mps2']['max'] == pytest.approx(.2)
    assert report['linear_jerk_mps3']['max'] == pytest.approx(1.)


@pytest.mark.parametrize('sim,domain', [(False, 76), (True, 16), (True, 226), (True, 227)])
def test_hardware_like_execution_requires_explicit_authorization(sim, domain):
    with pytest.raises(ValueError, match='allow-hardware'):
        module.execute_allowed(execute=True, use_sim_time=sim, domain=domain, allow_hardware=False)
    module.execute_allowed(execute=False, use_sim_time=sim, domain=domain, allow_hardware=False)
    module.execute_allowed(execute=True, use_sim_time=sim, domain=domain, allow_hardware=True)


def test_simulation_execution_and_config_hashes(tmp_path):
    module.execute_allowed(execute=True, use_sim_time=True, domain=76, allow_hardware=False)
    config = tmp_path/'trial.yaml'
    config.write_text('speed: 0.1\n')
    before = module.config_hashes([config])
    config.write_text('speed: 0.2\n')
    assert module.config_hashes([config]) != before


def mux_status(**changes):
    return dict({'mode': 'hold', 'available': True, 'estop_latched': False,
                 'source_id': 'mux-service:operator-reset', 'reset_pending': False}, **changes)


def test_authorized_auto_handover_is_ordered_and_never_resets_estop():
    from smart_vision_amr.motion_mux import MotionMuxCore
    mux = MotionMuxCore(clock=lambda: 10.)
    packet = module.benchmark_auto_packet(mux_status(), 10., 10.1, 20_000_000_000,
                                          'benchmark:test', 1)
    assert packet['mode'] == 'auto'
    assert mux.receive_control(packet, 20_000_000_000)
    assert mux.mode == 'auto' and mux.output() == (0., 0.)
    assert not mux.receive_control(packet, 20_000_000_000)  # No sequence replay.
    mux.estop_latched = True
    assert not mux.receive_control({**packet, 'seq': 2}, 20_000_000_000)
    assert mux.estop_latched


@pytest.mark.parametrize('changes', [
    {'mode': 'manual'}, {'estop_latched': True}, {'available': False},
    {'source_id': 'active-web-controller'}, {'reset_pending': True},
])
def test_benchmark_declines_other_operator_or_safety_state(changes):
    with pytest.raises(ValueError):
        module.benchmark_auto_packet(mux_status(**changes), 10., 10.1,
                                     20_000_000_000, 'benchmark:test', 1)


def test_benchmark_requires_fresh_mux_and_no_foreign_navigation():
    with pytest.raises(ValueError, match='Fresh'):
        module.benchmark_auto_packet(mux_status(), 8., 10., 20_000_000_000, 'benchmark:test', 1)
    with pytest.raises(ValueError, match='Another navigation'):
        module.benchmark_auto_packet(mux_status(), 10., 10.1, 20_000_000_000,
                                     'benchmark:test', 1, foreign_navigation_active=True)


def test_benchmark_waits_for_matched_motion_mux_subscriber():
    endpoint = SimpleNamespace(node_name='motion_mux')
    unrelated = SimpleNamespace(node_name='debug_listener')
    assert not module.benchmark_control_subscriber_ready(0, [endpoint])
    assert not module.benchmark_control_subscriber_ready(1, [unrelated])
    assert module.benchmark_control_subscriber_ready(1, [unrelated, endpoint])


def test_auto_handover_retries_lost_packet_and_accepts_only_own_sequence_receipt():
    handover = module.AutoModeHandshake('benchmark:test')
    status = mux_status()
    assert handover.step(status, 10., 10.1, 20_000_000_000, False) == (None, False)
    packet, acknowledged = handover.step(status, 10.1, 10.2, 20_100_000_000, True)
    assert not acknowledged
    assert packet == {'source_id': 'benchmark:test', 'seq': 1,
                      'sent_at_ns': 20_100_000_000, 'mode': 'auto',
                      'linear': 0.0, 'angular': 0.0}
    assert handover.step(status, 10.2, 10.3, 20_200_000_000, True) == (None, False)
    packet, acknowledged = handover.step(status, 10.3, 10.7, 20_600_000_000, True)
    assert not acknowledged and packet['seq'] == 2
    assert packet['sent_at_ns'] > 20_100_000_000  # Fresh stamp, not replay.

    # A stale/no receipt cannot be mistaken for ACK, even if mode/source match.
    own = mux_status(mode='auto', source_id='benchmark:test', control_seq=1,
                     last_control={'source_id': 'other', 'seq': 1,
                                   'mode': 'auto', 'origin': 'topic', 'accepted': True})
    assert handover.step(own, 10.7, 10.8, 20_700_000_000, True) == (None, False)
    own['last_control']['source_id'] = 'benchmark:test'
    assert handover.step(own, 10.8, 10.9, 20_800_000_000, True) == (None, True)
    assert handover.seq == 2

    next_goal = module.AutoModeHandshake('benchmark:test', initial_seq=handover.seq)
    packet, acknowledged = next_goal.step(own, 10.9, 11., 20_900_000_000, True)
    assert not acknowledged and packet['seq'] == 3


def test_auto_handover_does_not_send_on_stale_status_or_override_other_control():
    handover = module.AutoModeHandshake('benchmark:test')
    status = mux_status()
    assert handover.step(status, 8.9, 10., 20_000_000_000, True) == (None, False)
    packet, _ = handover.step(status, 10., 10.1, 20_100_000_000, True)
    assert packet['seq'] == 1
    for changed in (mux_status(mode='manual', source_id='operator'),
                    mux_status(estop_latched=True),
                    mux_status(mode='auto', source_id='other-benchmark')):
        with pytest.raises(ValueError):
            handover.step(changed, 10.1, 10.2, 20_200_000_000, True)
    with pytest.raises(ValueError, match='Another navigation'):
        handover.step(status, 10.1, 10.2, 20_200_000_000, True,
                      foreign_navigation_active=True)


def test_auto_handover_times_out_without_subscription_or_ack():
    no_subscriber = module.AutoModeHandshake('benchmark:test', timeout_seconds=1.)
    assert no_subscriber.step(mux_status(), 10., 10., 20_000_000_000, False) == (None, False)
    with pytest.raises(TimeoutError, match='subscriber did not match'):
        no_subscriber.step(mux_status(), 11.1, 11.1, 21_100_000_000, False)

    missing_ack = module.AutoModeHandshake('benchmark:test', timeout_seconds=1.)
    packet, _ = missing_ack.step(mux_status(), 10., 10., 20_000_000_000, True)
    assert packet['seq'] == 1
    rejection = mux_status(last_control_rejection={'source_id': 'benchmark:test',
                                                   'reason': 'Clock skew'})
    with pytest.raises(TimeoutError, match='Clock skew'):
        missing_ack.step(rejection, 11.1, 11.1, 21_100_000_000, True)


def test_auto_handover_accepts_real_mux_receipt_before_goal():
    from smart_vision_amr.motion_mux import MotionMuxCore
    mux = MotionMuxCore(clock=lambda: 10.)
    handover = module.AutoModeHandshake('benchmark:receipt')
    lost_packet, acknowledged = handover.step(mux.status(), 1., 1.1, 20_000_000_000, True)
    assert not acknowledged and lost_packet['seq'] == 1
    packet, acknowledged = handover.step(mux.status(), 1.6, 1.7, 20_600_000_000, True)
    assert not acknowledged and packet['seq'] == 2
    assert mux.receive_control(packet, 20_600_000_000, origin='topic')
    assert handover.step(mux.status(), 1.7, 1.8, 20_700_000_000, True) == (None, True)
    assert mux.output() == (0.0, 0.0)


def test_auto_handover_tolerates_short_outage_but_has_bounded_deadline():
    handover = module.AutoModeHandshake('benchmark:jitter')
    status = mux_status()
    packet, acknowledged = handover.step(status, 0., 0., 20_000_000_000, True)
    assert packet['seq'] == 1 and not acknowledged
    # The former 8 s deadline would abort even while zero-speed retries continue.
    packet, acknowledged = handover.step(status, 8.5, 8.5, 28_500_000_000, True)
    assert packet['seq'] == 2 and not acknowledged
    own = mux_status(mode='auto', source_id='benchmark:jitter', control_seq=2,
                     last_control={'source_id': 'benchmark:jitter', 'seq': 2,
                                   'mode': 'auto', 'origin': 'topic', 'accepted': True})
    assert handover.step(own, 8.7, 8.8, 28_800_000_000, True) == (None, True)

    expired = module.AutoModeHandshake('benchmark:expired')
    expired.step(status, 0., 0., 20_000_000_000, True)
    with pytest.raises(TimeoutError, match='did not acknowledge'):
        expired.step(status, 25.1, 25.1, 45_100_000_000, True)


def test_auto_handover_does_not_accept_old_receipt_with_fresh_status():
    handover = module.AutoModeHandshake('benchmark:stale')
    first, _ = handover.step(mux_status(), 0., 0., 20_000_000_000, True)
    own = mux_status(mode='auto', source_id='benchmark:stale', control_seq=first['seq'],
                     last_control={'source_id': 'benchmark:stale', 'seq': first['seq'],
                                   'mode': 'auto', 'origin': 'topic', 'accepted': True})
    next_packet, acknowledged = handover.step(own, 1.0, 1.0, 21_000_000_000, True)
    assert not acknowledged and next_packet['seq'] == 2
    assert next_packet['linear'] == next_packet['angular'] == 0.0
    own['control_seq'] = 2
    own['last_control']['seq'] = 2
    assert handover.step(own, 1.3, 1.3, 21_300_000_000, True) == (None, True)
