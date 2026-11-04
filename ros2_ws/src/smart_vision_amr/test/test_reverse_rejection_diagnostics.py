"""The planner reports why a checked reverse was rejected without retry spam."""

from unittest.mock import Mock

from smart_vision_amr import astar_planner as planner_module
from test_planner_failure_memory import prepared
from test_planner_start_reverse_evidence import scenario


def attempt(node, *, trail_verified=True):
    message, grid = node.map
    return node._maybe_request_reverse(
        (13, 3), (25, 3), message, grid, node.raw_static_grid,
        set(node.observed), set(), {}, trail_verified,
    )


def rejection_messages(node):
    return [call.args[0] for call in node.logger.warning.call_args_list
            if call.args and call.args[0].startswith('REVERSE_REJECT[')]


def test_reverse_logs_first_failed_gate_and_throttles_subsequent_retries(prepared):
    node = prepared
    scenario(node)
    node.logger.reset_mock()
    assert attempt(node, trail_verified=False) is False
    assert rejection_messages(node) == ['REVERSE_REJECT[TRAIL_UNVERIFIED] start=(13, 3)']

    # A new first-failing gate is visible immediately, while repeated
    # attempts with the same reason cannot flood the robot log.
    node.recovery_attempts = node.params['reverse_recovery_max_attempts']
    node.clock['steady'] = 100.5
    assert attempt(node) is False
    assert node.last_reverse_rejection_code == 'ATTEMPT_LIMIT'
    assert rejection_messages(node)[-1] == 'REVERSE_REJECT[ATTEMPT_LIMIT] start=(13, 3)'
    assert len(rejection_messages(node)) == 2

    node.clock['steady'] = 100.6
    assert attempt(node) is False
    assert len(rejection_messages(node)) == 2

    node.clock['steady'] = 105.5
    assert attempt(node) is False
    assert rejection_messages(node)[-1] == 'REVERSE_REJECT[ATTEMPT_LIMIT] start=(13, 3)'
    assert len(rejection_messages(node)) == 3
    node.recovery_pub.publish.assert_not_called()


def test_reverse_logs_when_no_collision_checked_exit_exists(prepared, monkeypatch):
    node = prepared
    scenario(node)
    node.logger.reset_mock()
    selector = Mock(return_value=None)
    monkeypatch.setattr(planner_module, 'select_reverse_egress', selector)

    assert attempt(node) is False
    selector.assert_called_once()
    assert node.last_reverse_rejection_code == 'NO_SAFE_EGRESS'
    assert rejection_messages(node) == ['REVERSE_REJECT[NO_SAFE_EGRESS] start=(13, 3)']
    node.recovery_pub.publish.assert_not_called()
    assert node.recovery_attempts == 0
