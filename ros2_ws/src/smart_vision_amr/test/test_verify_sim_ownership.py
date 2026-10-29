"""A failed simulator check must not stop a mission it never published."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[4] / 'scripts/verify_sim.py'
spec = importlib.util.spec_from_file_location('simulation_verifier', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def cleanup(goal_sent=False, goal_stamp=None, status=None):
    events = []
    node = SimpleNamespace(goal_sent=goal_sent, goal_stamp=goal_stamp,
                           latest_status_observed=status,
                           cancel_on_failure=lambda: events.append('cancel'))
    window = SimpleNamespace(emergency_stop=lambda: events.append('estop'))
    result = module.stop_failed_verification(node, window)
    return result, events


def test_not_ready_or_invalid_goal_sends_neither_estop_nor_cancel():
    assert cleanup() == (False, [])
    assert cleanup(status={'goal_stamp': {'sec': 4, 'nanosec': 0}}) == (False, [])


def test_published_goal_failure_stops_its_own_mission():
    stamp = {'sec': 4, 'nanosec': 2}
    assert cleanup(True, stamp, {'goal_stamp': stamp}) == (True, ['estop', 'cancel'])
    assert cleanup(True, stamp) == (True, ['estop', 'cancel'])


def test_newer_foreign_goal_and_unknown_stamp_are_not_canceled():
    assert cleanup(True, {'sec': 4}, {'goal_stamp': {'sec': 5}}) == (False, [])
    assert cleanup(True, None, {'goal_stamp': {'sec': 5}}) == (False, [])
    assert cleanup(True, {'sec': 4}, {'goal_stamp': None}) == (False, [])
