"""Current validated localization can outlive a failed prior-pose request."""

from copy import deepcopy
from pathlib import Path

import pytest

from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication


@pytest.fixture
def recovered_application(tmp_path, monkeypatch):
    now = 100.0
    monkeypatch.setattr('smart_vision_amr.web_mission_server.time.monotonic', lambda: now)
    app = MissionApplication(DEFAULT_MARKERS, 218, False, Path(tmp_path)/'settings.json',
                             access_mode='domain')
    app.connected = app.goal_available = app.planner_available = app.cancel_available = True
    app.map = {'width': 20, 'height': 20, 'resolution': .1, 'frame_id': 'map',
               'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    app.map_is_preview = False
    app.map_matches_configured = True
    app.pose = {'x': .5, 'y': .5, 'yaw': 0.}
    app.pose_at = app.scan_at = app.odom_at = app.camera_at = now
    app.pose_tf_stamp_ns = 100_000_000_000
    app.require_localization_health = True
    app.localization_health = {'level': 'ok', 'autonomous_allowed': True,
                               'stop_latched': False, 'reasons': [],
                               'sent_at_ns': 100_000_000_000}
    app.localization_health_at = now
    app.localization_invalidated = True
    app.manual_pose_state = 'FAILED'
    app.manual_pose_result = '이전 AMCL 초기 위치 요청 시간 초과'
    app.aruco_available = True
    app.mission.update_route([3])
    app.mission.emergency_stop()
    app.mission.drain_commands()
    app.manual_mux_at = now
    app.manual_mux_status = {'mode': 'estop', 'available': True,
                             'estop_latched': True, 'stop_acknowledged': True}
    return app


def test_recovered_current_pose_turns_prior_failure_into_warning_without_motion(recovered_application):
    app = recovered_application
    before = deepcopy((app.mission.estop_latched, app.mission.state,
                       app.manual_mux_status, app.safety_request, app.mission.commands,
                       app.manual_pose_state, app.manual_pose_result,
                       app.localization_invalidated))
    readiness = app.readiness()
    assert readiness['ready'] and not readiness['missing']
    assert any('이전 초기 위치 입력은 실패' in warning for warning in readiness['warnings'])
    assert before == (app.mission.estop_latched, app.mission.state,
                      app.manual_mux_status, app.safety_request, app.mission.commands,
                      app.manual_pose_state, app.manual_pose_result,
                      app.localization_invalidated)
    assert app.mission.estop_latched
    assert app.manual_pose_state == 'FAILED'  # No synthetic VERIFIED certificate.
    assert app.mission.commands == []


@pytest.mark.parametrize('change', [
    lambda app: setattr(app, 'localization_health_at', 98.5),
    lambda app: setattr(app, 'localization_health_at', 100.1),
    lambda app: app.localization_health.update(autonomous_allowed=False, level='error'),
    lambda app: setattr(app, 'localization_health', {}),
    lambda app: setattr(app, 'pose', None),
    lambda app: setattr(app, 'pose_at', 98.),
    lambda app: setattr(app, 'pose_at', 100.1),
    lambda app: setattr(app, 'pose_tf_stamp_ns', 0),
    lambda app: setattr(app, 'map', None),
    lambda app: setattr(app, 'map_is_preview', True),
    lambda app: setattr(app, 'map_matches_configured', False),
    lambda app: setattr(app, 'map_matches_configured', None),
    lambda app: setattr(app, 'scan_at', 98.),
    lambda app: setattr(app, 'odom_at', 98.),
    lambda app: setattr(app, 'connected', False),
    lambda app: setattr(app, 'require_localization_health', False),
    lambda app: setattr(app, 'manual_pose_request', {'pose': {'x': .5, 'y': .5}}),
])
def test_failed_pose_remains_blocked_without_current_recovery_proof(recovered_application, change):
    app = recovered_application
    change(app)
    readiness = app.readiness()
    assert not readiness['ready']
    assert '초기 위치 입력 검증 완료' in readiness['missing']
    assert not any('이전 초기 위치 입력은 실패' in warning for warning in readiness['warnings'])
    assert app.mission.estop_latched and app.safety_request is None
    assert app.manual_pose_state == 'FAILED' and not app.mission.commands


def test_pending_pose_still_blocks_even_with_fresh_healthy_localization(recovered_application):
    app = recovered_application
    app.manual_pose_state = 'WAITING'
    app.manual_pose_request = {'pose': {'x': .5, 'y': .5}}
    readiness = app.readiness()
    assert not readiness['ready']
    assert '초기 위치 입력 검증 완료' in readiness['missing']
    assert app.manual_pose_request is not None
    assert app.mission.estop_latched and not app.mission.commands


def test_recovery_does_not_remove_other_current_departure_conditions(recovered_application):
    app = recovered_application
    app.planner_available = False
    readiness = app.readiness()
    assert not readiness['ready'] and 'A* 계획 노드' in readiness['missing']
    assert app.mission.estop_latched and not app.mission.commands


def test_sessions_without_prior_failure_keep_existing_generic_readiness(recovered_application):
    app = recovered_application
    app.manual_pose_state = 'IDLE'
    app.localization_invalidated = False
    app.require_localization_health = False
    app.localization_health = {}
    assert app.readiness()['ready']
