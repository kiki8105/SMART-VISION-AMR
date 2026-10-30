"""A past path repair may never override live safety or a different goal."""
from copy import deepcopy

import pytest

from smart_vision_amr.operation_status import local_detour_snapshot, operation_summary


def packet():
    return {'goal_stamp': {'sec': 15, 'nanosec': 7}, 'plan_kind': 'LOCAL_DETOUR',
            'local_detour_state': 'ACTIVE', 'local_attempt_id': 1, 'global_plan_count': 2,
            'search_duration_ms': 7.3, 'adapter_duration_ms': 9.5, 'expansions': 72,
            'rejoin_point': {'x': 1., 'y': 2.}, 'rejoin_progress': 1.8,
            'detour_reason': 'local_detour_ready'}


def test_detour_status_is_current_bounded_and_read_only():
    value = packet()
    original = deepcopy(value)
    status = local_detour_snapshot(value, (15, 7), 'NAVIGATING', .1)
    assert status['available'] and status['state'] == 'ACTIVE'
    assert status['rejoin_point'] == {'x': 1., 'y': 2.}
    assert status['search_duration_ms'] == 7.3
    assert value == original


@pytest.mark.parametrize('goal,state,age', [
    ((16, 7), 'NAVIGATING', .1), (None, 'NAVIGATING', .1),
    ((15, 7), 'PAUSED', .1), ((15, 7), 'COMPLETED', .1),
    ((15, 7), 'CONFIRMING', .1), ((15, 7), 'NAVIGATING', 2.),
    ((15, 7), 'NAVIGATING', -.1), ((True, 7), 'NAVIGATING', .1)])
def test_stale_wrong_goal_and_terminal_mission_do_not_show_detour(goal, state, age):
    assert not local_detour_snapshot(packet(), goal, state, age)['available']


def test_nonfinite_or_malformed_optional_metrics_are_omitted():
    value = packet()
    value.update(search_duration_ms=float('nan'), expansions=True,
                 rejoin_point={'x': float('inf'), 'y': 2.})
    status = local_detour_snapshot(value, [15, 7], 'RETURNING_HOME', .1)
    assert status['available']
    assert not {'search_duration_ms', 'expansions', 'rejoin_point'} & status.keys()


def test_detour_summary_keeps_safety_and_operator_priority():
    state = {'connection': {'connected': True}, 'health': {'missing': []},
             'mission': {'state': 'NAVIGATING'},
             'local_detour': local_detour_snapshot(packet(), (15, 7), 'NAVIGATING', .1)}
    assert operation_summary(state)['phase'] == 'local_detour'
    state['manual'] = {'active': True}
    assert operation_summary(state)['phase'] == 'manual'
    state['safety'] = {'latched': True}
    assert operation_summary(state)['phase'] == 'estopped'
    state['manual'] = state['safety'] = {}
    state['failure_memory'] = {'available': True, 'state': 'OBSTACLE_HOLD'}
    assert operation_summary(state)['phase'] == 'obstacle_hold'
    state['failure_memory'] = {}
    state['mission']['state'] = 'PAUSED'
    assert operation_summary(state)['phase'] == 'paused'
