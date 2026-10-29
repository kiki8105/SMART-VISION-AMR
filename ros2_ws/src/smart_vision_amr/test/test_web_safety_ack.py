"""A UI reset request cannot clear a stop without the robot-side result."""

import time

import pytest

from smart_vision_amr.mission_logic import MissionError
from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication


def app_with_stop(tmp_path, acknowledged=True):
    app = MissionApplication(DEFAULT_MARKERS, 215, False, tmp_path/'settings.json')
    app.command('/api/emergency-stop', {})
    if acknowledged:
        app.manual_mux_at = time.monotonic()
        app.manual_mux_status = {'estop_latched': True, 'stop_acknowledged': True}
    return app


def test_reset_is_pending_until_robot_service_acknowledges_and_never_resumes(tmp_path):
    app = app_with_stop(tmp_path)
    state = app.command('/api/emergency-stop', {'reset': True})
    assert state['mission']['state'] == 'ESTOP'
    assert state['safety']['pending'] == 'reset'
    assert not any(kind == 'goal' for kind, _ in app.mission.commands)
    assert app.complete_safety_result('reset', True, 'guard confirmed')
    assert app.mission.state == 'CANCELED' and not app.mission.estop_latched
    assert not any(kind == 'goal' for kind, _ in app.mission.commands)


def test_guard_rejection_or_timeout_keeps_ui_and_mission_latched(tmp_path):
    app = app_with_stop(tmp_path)
    app.command('/api/emergency-stop', {'reset': True})
    assert not app.complete_safety_result('reset', False, 'EKF inputs unhealthy')
    assert app.mission.state == 'ESTOP' and app.mission.estop_latched
    assert app.safety_request is None
    assert app.state()['safety']['result'] == 'EKF inputs unhealthy'


def test_old_reset_result_cannot_undo_a_new_emergency_stop(tmp_path):
    app = app_with_stop(tmp_path)
    app.command('/api/emergency-stop', {'reset': True})
    app.command('/api/emergency-stop', {})
    assert not app.complete_safety_result('reset', True, 'late service result')
    assert app.mission.state == 'ESTOP' and app.mission.estop_latched
    assert app.safety_request == 'stop'


def test_unacknowledged_or_stale_stop_cannot_be_replaced_by_reset(tmp_path):
    app = app_with_stop(tmp_path, acknowledged=False)
    with pytest.raises(MissionError):
        app.command('/api/emergency-stop', {'reset': True})
    assert app.safety_request == 'stop'
    app.manual_mux_at = time.monotonic()-2
    app.manual_mux_status = {'estop_latched': True, 'stop_acknowledged': True}
    with pytest.raises(MissionError):
        app.command('/api/emergency-stop', {'reset': True})
