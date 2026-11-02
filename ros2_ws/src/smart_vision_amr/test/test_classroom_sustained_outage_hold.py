"""A long sensor outage may pass only as a latched, stationary safe hold."""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from verify_classroom_faults import assess_sustained_outage_hold


def sample(**overrides):
    values = dict(terminal='SAFE_HOLD', safe_hold=True,
                  health=[{'t': 10., 'stop_latched': True,
                           'autonomous_allowed': False}],
                  motion=[{'t': 12.5, 'estop_latched': True}],
                  planner={'t': 12., 'status': 'WAIT_REPLAN',
                           'goal': {'x': -.25, 'y': -1.98}},
                  navigation=[{'status': 'MOVING', 'manual_stop': False}],
                  goal_xy=(-.25, -1.98), goal_sent_t=8., sampled_at=13.)
    values.update(overrides)
    return assess_sustained_outage_hold(**values)


def test_latched_stationary_retained_goal_is_safe_hold():
    metrics, passed = sample()
    assert passed
    assert metrics['outage_manual_reset_required'] is True
    assert metrics['outage_goal_retained'] is True


def test_transient_warning_does_not_pass_as_sustained_hold():
    assert not sample(health=[{'t': 10., 'stop_latched': False,
                               'autonomous_allowed': True}])[1]
    assert not sample(motion=[{'estop_latched': False}])[1]
    assert not sample(motion=[{'t': 11., 'estop_latched': True}])[1]


def test_moving_or_cancelled_goal_does_not_pass():
    assert not sample(safe_hold=False)[1]
    assert not sample(planner={'status': 'IDLE',
                               'goal': {'x': -.25, 'y': -1.98}})[1]
    assert not sample(planner={'t': 9., 'status': 'WAIT_REPLAN',
                               'goal': {'x': -.25, 'y': -1.98}})[1]
    assert not sample(navigation=[{'status': 'CANCELED',
                                   'manual_stop': True}])[1]


def test_startup_latch_before_goal_does_not_count():
    assert not sample(health=[{'t': 3., 'stop_latched': True,
                               'autonomous_allowed': False}])[1]
