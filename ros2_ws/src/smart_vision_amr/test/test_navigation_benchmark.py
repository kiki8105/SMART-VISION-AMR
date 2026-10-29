"""Route scoring and execution safeguards without sending ROS goals."""

import importlib.util
from pathlib import Path

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
