"""ESTOP home seeding is a confirmed pose prior, never a motion/reset action."""

from copy import deepcopy
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from smart_vision_amr.mission_logic import MissionError
from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication


def application(tmp_path):
    app = MissionApplication(DEFAULT_MARKERS, 218, False, tmp_path/'settings.json', access_mode='domain')
    now = time.monotonic()
    app.connected = True
    metadata = {'width': 80, 'height': 80, 'resolution': .05, 'frame_id': 'map',
                'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    data = np.zeros((80, 80), dtype=np.int16)
    data[:, 60] = 100
    app._configured_map = (metadata, data.copy(), None)
    app.receive_map(metadata, data)
    app.delivery.registered_homepoint = {'x': .525, 'y': 2.025, 'yaw': 0.}
    app.mission.emergency_stop()
    app.manual_mux_at = now
    app.manual_mux_status = {'mode': 'estop', 'estop_latched': True, 'stop_acknowledged': True}
    app.scan_at = app.odom_at = app.localization_scan_at = app.localization_odom_at = now
    app.localization_services_ready = True
    app.localization_amcl_active_at = now
    app.speed = {'linear': 0., 'angular': 0.}
    app.home_seed_match = .9
    app.home_seed_match_at = now
    app.home_seed_match_key = app.home_seed_key()
    app.mission.drain_commands()
    return app


def test_confirmed_home_queues_only_pose_seed_and_preserves_estop_and_mission(tmp_path):
    app = application(tmp_path)
    before = deepcopy((app.mission.route, app.mission.current_index, app.mission.completed_ids))
    status = app.command('/api/localization/seed-home', {'confirm_actual_home': True})
    assert status['mission']['estop_latched']
    assert status['localization']['home_seed_pending']
    assert not app.localization.active and not app.localization_requested
    assert app.home_seed_request['home'] == app.delivery.registered_homepoint
    assert not app.mission.commands  # No goal, rotation, cancel or reset is queued.
    assert before == (app.mission.route, app.mission.current_index, app.mission.completed_ids)
    with pytest.raises(MissionError, match='입력이 끝난'):
        app.command('/api/emergency-stop', {'reset': True})


def test_home_confirmation_is_explicit_and_boolean(tmp_path):
    app = application(tmp_path)
    for payload in ({}, {'confirm_actual_home': False}, {'confirm_actual_home': 'true'}):
        with pytest.raises(MissionError, match='확인란'):
            app.command('/api/localization/seed-home', payload)
    assert app.home_seed_request is None and app.mission.estop_latched


@pytest.mark.parametrize('change', [
    lambda app: setattr(app, 'scan_at', 0.),
    lambda app: setattr(app, 'odom_at', 0.),
    lambda app: setattr(app, 'home_seed_match', .5),
    lambda app: setattr(app, 'home_seed_match_at', 0.),
    lambda app: setattr(app, 'map_matches_configured', False),
    lambda app: app.speed.update(linear=.02),
    lambda app: app.speed.update(angular=math.nan),
    lambda app: app.manual_mux_status.update(stop_acknowledged=False),
    lambda app: setattr(app, 'navigation_active', True),
    lambda app: app.delivery.registered_homepoint.update(x=.8),
])
def test_seed_rejects_stale_wrong_map_moving_unverified_or_changed_home(tmp_path, change):
    app = application(tmp_path)
    change(app)
    with pytest.raises(MissionError):
        app.command('/api/localization/seed-home', {'confirm_actual_home': True})
    assert app.home_seed_request is None and app.mission.estop_latched


def test_seed_ui_labels_nonmoving_action_and_requires_checkbox():
    package = Path(__file__).resolve().parents[1]
    html = (package/'web/index.html').read_text()
    script = (package/'web/app.js').read_text()
    assert '등록 홈 초기 위치만 입력' in html
    assert 'confirm-home-location' in html
    assert "api/localization/seed-home" in script
    assert "!$('#confirm-home-location').checked" in script


@pytest.mark.parametrize('expired', [False, True])
def test_ros_dispatch_revalidates_and_publishes_only_initialpose_while_latched(tmp_path, expired):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from smart_vision_amr.web_mission_server import create_ros_bridge
    app = application(tmp_path)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=219, signal_handler_options=SignalHandlerOptions.NO)
    node = create_ros_bridge(app, SimpleNamespace(camera_topic='/isolated_seed_camera',
        base_frame='isolated_seed_base'), context)
    poses, motion, goals = [], [], []
    node.initial_pose_pub = SimpleNamespace(publish=poses.append)
    node.motion_pub = SimpleNamespace(publish=motion.append)
    node.goal_pub = SimpleNamespace(publish=goals.append)
    node.get_subscriptions_info_by_topic = lambda _: [SimpleNamespace(node_name='amcl')]
    try:
        app.command('/api/localization/seed-home', {'confirm_actual_home': True})
        if expired:
            app.home_seed_request['at'] -= 3.
        node.tick_home_seed(time.monotonic())
        assert app.home_seed_request is None
        assert app.mission.estop_latched
        assert not app.localization.active
        assert not motion and not goals
        if expired:
            assert not poses and '거부' in app.home_seed_result
        else:
            assert len(poses) == 1
            assert poses[0].pose.pose.position.x == .525
            assert poses[0].pose.covariance[0] == .09
            assert app.localization_invalidated
    finally:
        node.destroy_node()
        context.shutdown()
