"""Failure memory is read-only UI state, scoped to one fresh navigation goal."""
from copy import deepcopy
from pathlib import Path
import time

import pytest

from smart_vision_amr.operation_status import (failure_memory_snapshot, operation_summary,
                                              OBSTACLE_HOLD_MESSAGE)
from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication


def payload(state='AVOIDING'):
    return {'state': state, 'reason': '같은 코너 접근 회피', 'goal_stamp': {'sec': 12, 'nanosec': 45},
            'sent_at_ns': 12000000045, 'count': 1, 'recovery_attempts': 2,
            'active': [{'id': 'failure-1', 'x': 1., 'y': 2., 'yaw': .4,
                        'reason': '선회 진입 실패', 'expires_in': 12., 'failures': 2}]}


def test_scope_expiration_and_ttl_do_not_modify_planner_payload():
    original = payload()
    result = failure_memory_snapshot(original, (12, 45), 'NAVIGATING', .5)
    assert result['available'] and result['count'] == 1
    assert result['active'][0]['expires_in'] == 11.5
    assert original['active'][0]['expires_in'] == 12.
    for current, state, age in [(None, 'NAVIGATING', 0.), ((13, 45), 'NAVIGATING', 0.),
                                 ((12, 45), 'COMPLETED', 0.), ((12, 45), 'IDLE', 0.),
                                 ((12, 45), 'NAVIGATING', 2.), ((12, 45), 'NAVIGATING', -.1)]:
        assert failure_memory_snapshot(original, current, state, age)['available'] is False


@pytest.mark.parametrize('change', [lambda p: p.update(state='UNKNOWN'),
    lambda p: p.update(goal_stamp={'sec': True, 'nanosec': 45}),
    lambda p: p.update(goal_stamp={'sec': 12, 'nanosec': 1000000000})])
def test_malformed_state_or_stamp_is_not_displayed(change):
    value = payload()
    change(value)
    assert not failure_memory_snapshot(value, (12, 45), 'NAVIGATING', 0.)['available']


def test_entries_are_bounded_finite_and_accept_approach_heading_alias():
    value = payload()
    first = value['active'][0]
    first['approach_heading'] = first.pop('yaw')
    value['active'] += [dict(first, x=float('nan')), dict(first, expires_in=.1), None]
    result = failure_memory_snapshot(value, [12, 45], 'NAVIGATING', .5)
    assert result['count'] == 1 and result['active'][0]['yaw'] == .4
    value['active'] = [first]*200
    assert failure_memory_snapshot(value, (12, 45), 'NAVIGATING', 0.)['count'] == 100


def test_sensor_hold_does_not_fabricate_a_failure_record():
    value = payload('SENSOR_HOLD')
    value['active'] = []
    result = failure_memory_snapshot(value, (12, 45), 'NAVIGATING', 0.)
    assert result['state'] == 'SENSOR_HOLD' and result['count'] == 0


def test_obstacle_hold_keeps_goal_and_valid_counters_and_uses_requested_message():
    value = payload('OBSTACLE_HOLD')
    value.update(active=[], retry_count=3, retry_limit=3,
                 blocked_obstacle_count=12, clear_wait_seconds=.5)
    result = failure_memory_snapshot(value, (12, 45), 'NAVIGATING', .1)
    assert result['available'] and result['state'] == 'OBSTACLE_HOLD'
    assert result['retry_count'] == result['retry_limit'] == 3
    assert result['blocked_obstacle_count'] == 12 and result['clear_wait_seconds'] == .5
    assert result['goal_stamp'] == {'sec': 12, 'nanosec': 45}
    state = {'connection': {'connected': True}, 'health': {'missing': []},
             'mission': {'state': 'NAVIGATING'}, 'failure_memory': result}
    operation = operation_summary(state)
    assert operation['phase'] == 'obstacle_hold' and operation['detail'] == OBSTACLE_HOLD_MESSAGE
    state['manual'] = {'active': True}
    assert operation_summary(state)['phase'] == 'manual'
    state['manual'] = {}
    result['state'] = 'SENSOR_HOLD'
    assert operation_summary(state)['phase'] == 'sensor_hold'
    assert operation_summary(state)['detail'] != OBSTACLE_HOLD_MESSAGE
    assert not failure_memory_snapshot(value, (12, 46), 'NAVIGATING', .1)['available']
    assert not failure_memory_snapshot(value, (12, 45), 'NAVIGATING', 2.)['available']


def test_bad_optional_obstacle_counters_are_not_exposed():
    value = payload('OBSTACLE_HOLD')
    value.update(retry_count=True, retry_limit=-1, blocked_obstacle_count='12',
                 clear_wait_seconds=float('nan'))
    result = failure_memory_snapshot(value, (12, 45), 'NAVIGATING', .1)
    assert result['state'] == 'OBSTACLE_HOLD'
    assert not {'retry_count', 'retry_limit', 'blocked_obstacle_count', 'clear_wait_seconds'} & result.keys()


def test_operator_priority_and_no_finished_mission_recovery_banner():
    state = {'connection': {'connected': True}, 'health': {'missing': []},
             'mission': {'state': 'NAVIGATING'},
             'failure_memory': failure_memory_snapshot(payload(), (12, 45), 'NAVIGATING', 0.)}
    assert operation_summary(state)['phase'] == 'failure_avoiding'
    for phase, result in [('RECOVERING', 'failure_reversing'), ('REJOINING', 'failure_rejoining'),
                           ('WAIT_ALTERNATIVE', 'failure_wait'), ('SENSOR_HOLD', 'sensor_hold')]:
        state['failure_memory']['state'] = phase
        assert operation_summary(state)['phase'] == result
    state['manual'] = {'active': True}
    assert operation_summary(state)['phase'] == 'manual'
    state['safety'] = {'latched': True}
    assert operation_summary(state)['phase'] == 'estopped'
    state['manual'] = state['safety'] = {}
    state['mission']['state'] = 'PAUSED'
    assert operation_summary(state)['phase'] == 'paused'
    state['mission']['state'] = 'COMPLETED'
    assert operation_summary(state)['phase'] == 'idle'


def test_application_snapshot_discards_previous_goal_and_stale_status(tmp_path):
    app = MissionApplication(DEFAULT_MARKERS, 216, False, Path(tmp_path)/'settings.json')
    app.mission.state = 'NAVIGATING'
    app.mission.goal_stamp = (12, 45)
    app.failure_memory_status = deepcopy(payload())
    app.failure_memory_at = time.monotonic()
    assert app.state()['failure_memory']['available']
    app.mission.goal_stamp = (12, 46)
    assert app.state()['failure_memory']['available'] is False
    app.mission.goal_stamp = (12, 45)
    app.failure_memory_at -= 3.
    assert app.state()['failure_memory']['available'] is False
