"""Spatial execution failures require sustained intent and continuous valid time."""
import math
import pytest

from smart_vision_amr.execution_progress import ExecutionProgress, ProgressSample


def sample(t, x=0., y=0., progress=0., error=.8, linear=.1, angular=0., ros=None):
    return ProgressSample(t, int((t+10)*1e9) if ros is None else ros,
                          x, y, error, progress, linear, angular)


def run(detector, factory, route='route'):
    decisions = []
    for step in range(71):
        t = step*.1
        result = detector.observe(factory(t), route)
        if result is not None:
            decisions.append(result)
    return decisions


def test_stationary_commanded_motion_reports_no_progress():
    results = run(ExecutionProgress(), lambda t: sample(t))
    assert results and results[0][0] == 'execution_no_progress'
    assert results[0][1]['window_seconds'] >= 6.


def test_alternating_turns_report_oscillation_with_bounded_evidence():
    results = run(ExecutionProgress(), lambda t: sample(t, linear=0., angular=.2 if int(t*2)%2 else -.2))
    assert results[0][0] == 'execution_oscillation'
    assert results[0][1]['sign_changes'] >= 4
    assert len(results[0][1]) == 6


@pytest.mark.parametrize('speed', [.02, .10])
def test_forward_progress_is_productive(speed):
    assert not run(ExecutionProgress(), lambda t: sample(t, x=speed*t, progress=speed*t))


def test_deliberate_in_place_alignment_is_productive():
    assert not run(ExecutionProgress(), lambda t: sample(t, error=1.5-.1*t, linear=0., angular=.2))


def test_pause_does_not_create_route_failure():
    assert not run(ExecutionProgress(), lambda t: sample(t, linear=0., angular=0.))


def test_intermittent_intent_cannot_accumulate_stuck_time():
    assert not run(ExecutionProgress(), lambda t: sample(t, linear=.1 if t<3 or t>4 else 0.))


def test_equivalent_route_identity_keeps_history():
    detector = ExecutionProgress()
    for step in range(60):
        assert detector.observe(sample(step*.1), 'same-route') is None
    assert detector.observe(sample(6.1), 'same-route')[0] == 'execution_no_progress'


def test_changed_route_resets_progress_window():
    detector = ExecutionProgress()
    for step in range(71):
        assert detector.observe(sample(step*.1), 'old' if step<40 else 'new') is None


@pytest.mark.parametrize('bad', ['duplicate_time', 'backward_time', 'wall_gap', 'invalid_numeric'])
def test_time_or_numeric_fault_resets_history_without_route_failure(bad):
    detector = ExecutionProgress()
    for step in range(60):
        detector.observe(sample(step*.1), 'same')
    last = detector.samples[-1]
    value = {'duplicate_time': sample(6., ros=last.ros_ns),
             'backward_time': sample(6., ros=last.ros_ns-1),
             'wall_gap': sample(7.),
             'invalid_numeric': sample(6., x=math.nan)}[bad]
    assert detector.observe(value, 'same') is None
    assert len(detector.samples) <= 1


def test_clock_pause_cannot_be_turned_into_route_failure():
    assert not run(ExecutionProgress(), lambda t: sample(t, ros=10_000_000_000))


def test_invalid_limits_rejected():
    for kwargs in ({'window': 0}, {'min_progress': math.nan}, {'oscillation_changes': 1}):
        with pytest.raises(ValueError):
            ExecutionProgress(**kwargs)


def test_oscillation_survives_brief_zero_crossings_but_not_long_intentional_pause():
    commands = [.2, .2, 0., -.2, -.2, 0.]
    results = run(ExecutionProgress(), lambda t: sample(t, linear=0., angular=commands[round(t*10)%6]))
    assert results and results[0][0] == 'execution_oscillation'
