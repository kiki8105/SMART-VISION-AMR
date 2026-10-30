"""Explicit operator map pose must never start a mission or bypass stopped sensing."""

import math
import http.client
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from smart_vision_amr.mission_logic import MissionError
from smart_vision_amr.web_mission_server import (DEFAULT_MARKERS, MissionApplication,
                                               MissionHTTPServer)


def stopped_application(tmp_path):
    app = MissionApplication(DEFAULT_MARKERS, 218, False, Path(tmp_path)/'settings.json',
                             access_mode='domain')
    metadata = {'width': 80, 'height': 80, 'resolution': .05, 'frame_id': 'map',
                'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    data = np.zeros((80, 80), dtype=np.int16)
    data[:, 60] = 100
    app._configured_map = (metadata, data.copy(), None)
    app.receive_map(metadata, data)
    now = time.monotonic()
    app.connected = True
    app.mission.emergency_stop()
    app.mission.drain_commands()
    app.manual_mux_at = now
    app.manual_mux_status = {'mode': 'estop', 'estop_latched': True,
                             'stop_acknowledged': True}
    app.scan_at = app.odom_at = app.localization_scan_at = app.localization_odom_at = now
    app.localization_amcl_active_at = now
    app.manual_pose_service_ready = True
    app.speed = {'linear': 0., 'angular': 0.}
    return app


def submit(app, **overrides):
    payload = {'x': .525, 'y': 2.025, 'yaw_deg': 90., 'confirm': True}
    payload.update(overrides)
    return app.command('/api/localization/initial-pose', payload)


def test_manual_pose_is_only_a_pending_prior_with_estop_and_no_motion(tmp_path):
    app = stopped_application(tmp_path)
    before = (list(app.mission.route), app.mission.current_index,
              list(app.mission.completed_ids))
    state = submit(app)
    assert state['localization']['manual_pose']['state'] == 'WAITING'
    assert state['localization']['manual_pose']['requested_pose'] == {
        'x': .525, 'y': 2.025, 'yaw_deg': 90.}
    assert not state['localization']['manual_pose']['verified']
    assert state['mission']['estop_latched']
    assert app.mission.commands == []
    assert before == (list(app.mission.route), app.mission.current_index,
                      list(app.mission.completed_ids))
    with pytest.raises(MissionError, match='검증이 끝난'):
        app.command('/api/emergency-stop', {'reset': True})


@pytest.mark.parametrize('change', [
    lambda app: setattr(app, 'scan_at', 0.),
    lambda app: setattr(app, 'odom_at', 0.),
    lambda app: setattr(app, 'map_matches_configured', False),
    lambda app: setattr(app, 'map_is_preview', True),
    lambda app: setattr(app, 'manual_pose_service_ready', False),
    lambda app: setattr(app, 'localization_amcl_active_at', 0.),
    lambda app: app.manual_mux_status.update(stop_acknowledged=False),
    lambda app: app.speed.update(linear=.02),
    lambda app: setattr(app, 'navigation_active', True),
    lambda app: setattr(app, 'connected', False),
    lambda app: setattr(app, 'localization_service_pending', True),
    lambda app: setattr(app, 'localization_requested', True),
    lambda app: setattr(app, 'home_seed_request', {'home': {'x': 0.}, 'at': time.monotonic()}),
])
def test_manual_pose_rejects_missing_safety_map_sensor_or_amcl(tmp_path, change):
    app = stopped_application(tmp_path)
    change(app)
    with pytest.raises(MissionError):
        submit(app)
    assert app.manual_pose_request is None
    assert app.mission.estop_latched


@pytest.mark.parametrize('payload', [
    {'confirm': False}, {'confirm': 'true'},
    {'x': float('nan')}, {'y': float('inf')}, {'yaw_deg': True},
    {'yaw_deg': 181.}, {'x': 4.5}, {'x': 3.025}, {'x': 2.975},
])
def test_manual_pose_rejects_unconfirmed_invalid_or_unsafe_coordinates(tmp_path, payload):
    app = stopped_application(tmp_path)
    with pytest.raises(MissionError):
        submit(app, **payload)
    assert app.manual_pose_request is None


def test_manual_pose_can_use_nonhome_free_position_and_does_not_change_home(tmp_path):
    app = stopped_application(tmp_path)
    home = dict(app.delivery.registered_homepoint)
    submit(app, x=1.5, y=1.2, yaw_deg=-180.)
    assert app.manual_pose_request['pose'] == {'x': 1.5, 'y': 1.2, 'yaw': -math.pi}
    assert app.delivery.registered_homepoint == home


@pytest.mark.parametrize('cell_value', [-1, 100])
def test_unknown_or_occupied_neighbor_inside_footprint_is_rejected(tmp_path, cell_value):
    app = stopped_application(tmp_path)
    # Robot center is free at (row 40, column 10); cell 15 begins 22.5 cm
    # from it, inside the 22 cm body radius plus 3.5 cm half-cell margin.
    app._display_map_data[40, 15] = cell_value
    with pytest.raises(MissionError, match='차체'):
        submit(app)
    assert app.manual_pose_request is None


def test_footprint_boundary_is_rejected_but_distant_unknown_is_allowed(tmp_path):
    app = stopped_application(tmp_path)
    with pytest.raises(MissionError, match='지도 밖'):
        submit(app, x=.24)
    # This unknown cell starts 27.5 cm from the center, beyond the protected
    # 25.5 cm radius. It should not reject a genuinely open position.
    app._display_map_data[40, 16] = -1
    submit(app)
    assert app.manual_pose_request is not None


@pytest.mark.parametrize('change', [
    lambda app: setattr(app, 'map_matches_configured', False),
    lambda app: setattr(app, 'map_is_preview', True),
    lambda app: setattr(app, 'map_version', app.map_version+1),
    lambda app: setattr(app, 'localization_invalidated', True),
    lambda app: setattr(app, 'scan_at', 0.),
    lambda app: setattr(app, 'odom_at', 0.),
    lambda app: setattr(app, 'pose_at', 0.),
    lambda app: setattr(app, 'localization_amcl_active_at', 0.),
    lambda app: setattr(app, 'connected', False),
])
def test_verified_badge_requires_current_map_sensors_and_localization(tmp_path, change):
    app = stopped_application(tmp_path)
    app.manual_pose_state = 'VERIFIED'
    app.manual_pose_map_version = app.map_version
    app.pose = {'x': .525, 'y': 2.025, 'yaw': 0.}
    app.pose_at = time.monotonic()
    app.pose_tf_stamp_ns = 123
    assert app.state()['localization']['manual_pose']['verified']
    change(app)
    assert not app.state()['localization']['manual_pose']['verified']


def test_http_initial_pose_contract_and_explicit_confirmation(tmp_path):
    app = stopped_application(tmp_path)
    server = MissionHTTPServer(('127.0.0.1', 0), app, tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        for confirm, expected in ((False, 400), (True, 200)):
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
            connection.request('POST', '/api/localization/initial-pose',
                               json.dumps({'x': .525, 'y': 2.025, 'yaw_deg': 90.,
                                           'confirm': confirm}),
                               {'Content-Type': 'application/json'})
            response = connection.getresponse()
            data = json.loads(response.read())
            connection.close()
            assert response.status == expected
            if confirm:
                assert data['localization']['manual_pose']['state'] == 'WAITING'
            else:
                assert '확인란' in data['error']
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2.)


def test_ros_dispatch_waits_for_amcl_and_new_tf(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from smart_vision_amr.web_mission_server import create_ros_bridge

    class FakeFuture:
        def __init__(self):
            self.ready = False

        def done(self):
            return self.ready

        def result(self):
            return SimpleNamespace()

    class FakeService:
        def __init__(self):
            self.requests = []
            self.future = FakeFuture()

        def call_async(self, request):
            self.requests.append(request)
            return self.future

        def remove_pending_request(self, future):
            pass

    app = stopped_application(tmp_path)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=219,
               signal_handler_options=SignalHandlerOptions.NO)
    node = create_ros_bridge(app, SimpleNamespace(camera_topic='/isolated_manual_pose_camera',
                             base_frame='isolated_manual_pose_base'), context)
    service = node.initial_pose_client = FakeService()
    try:
        submit(app)
        node.tick_manual_pose(time.monotonic())
        assert len(service.requests) == 1
        request = service.requests[0].pose
        assert request.header.frame_id == 'map'
        assert request.pose.pose.position.x == .525
        assert abs(request.pose.pose.orientation.z-math.sin(math.pi/4)) < 1e-6
        assert request.pose.covariance[0] == .25**2
        assert app.mission.estop_latched
        assert app.manual_pose_state == 'WAITING'
        assert app.localization_invalidated
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'WAITING'
        service.future.ready = True
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'WAITING'  # Service receipt alone is not localization.
        app.manual_pose_amcl_at = time.monotonic()
        app.pose_at = time.monotonic()
        app.pose = {'x': .525, 'y': 2.025, 'yaw': math.pi/2}
        app.pose_tf_stamp_ns = node.manual_pose_ros_stamp_ns - 1
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'WAITING'  # Old TF cannot certify the new prior.
        app.pose_tf_stamp_ns = node.manual_pose_ros_stamp_ns + 1
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'VERIFIED'
        assert not app.localization_invalidated
        assert app.mission.estop_latched
        assert not app.mission.commands
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('change', [
    lambda app: setattr(app, 'scan_at', 0.),
    lambda app: app._display_map_data.__setitem__((40, 15), -1),
    lambda app: app._display_map_data.__setitem__((40, 15), 100),
])
def test_ros_dispatch_rechecks_sensors_and_footprint_before_sending(tmp_path, change):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from smart_vision_amr.web_mission_server import create_ros_bridge

    app = stopped_application(tmp_path)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=219,
               signal_handler_options=SignalHandlerOptions.NO)
    node = create_ros_bridge(app, SimpleNamespace(camera_topic='/isolated_manual_pose_camera2',
                             base_frame='isolated_manual_pose_base2'), context)
    node.initial_pose_client = SimpleNamespace(call_async=lambda _: pytest.fail('must not send'))
    try:
        submit(app)
        change(app)
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'FAILED'
        assert app.localization_invalidated
        assert app.mission.estop_latched
    finally:
        node.destroy_node()
        context.shutdown()


def test_ros_dispatch_rejects_competing_inflight_amcl_initialization(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from smart_vision_amr.web_mission_server import create_ros_bridge

    app = stopped_application(tmp_path)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=219,
               signal_handler_options=SignalHandlerOptions.NO)
    node = create_ros_bridge(app, SimpleNamespace(camera_topic='/isolated_competing_pose_camera',
                             base_frame='isolated_competing_pose_base'), context)
    node.initial_pose_client = SimpleNamespace(call_async=lambda _: pytest.fail('must not send'))
    try:
        submit(app)
        node.localization_future = SimpleNamespace()
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'FAILED'
        assert app.mission.estop_latched
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('change', [
    lambda app: setattr(app, 'scan_at', 0.),
    lambda app: setattr(app, 'localization_scan_at', 0.),
    lambda app: setattr(app, 'odom_at', 0.),
    lambda app: setattr(app, 'localization_odom_at', 0.),
    lambda app: app.manual_mux_status.update(stop_acknowledged=False),
    lambda app: setattr(app, 'manual_mux_at', 0.),
])
def test_ros_confirmation_fails_if_sensor_or_stop_ack_is_lost(tmp_path, change):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from smart_vision_amr.web_mission_server import create_ros_bridge

    class Future:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace()

    app = stopped_application(tmp_path)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=219,
               signal_handler_options=SignalHandlerOptions.NO)
    node = create_ros_bridge(app, SimpleNamespace(camera_topic='/isolated_lost_sensor_camera',
                             base_frame='isolated_lost_sensor_base'), context)
    node.initial_pose_client = SimpleNamespace(call_async=lambda _: Future())
    try:
        submit(app)
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'WAITING'
        app.manual_pose_amcl_at = time.monotonic()
        app.pose_at = time.monotonic()
        app.pose = {'x': .525, 'y': 2.025, 'yaw': math.pi/2}
        app.pose_tf_stamp_ns = node.manual_pose_ros_stamp_ns + 1
        change(app)
        node.tick_manual_pose(time.monotonic())
        assert app.manual_pose_state == 'FAILED'
        assert app.localization_invalidated
        assert app.mission.estop_latched
        assert not app.mission.commands
    finally:
        node.destroy_node()
        context.shutdown()
