"""Ten-sector request/pre-arm/Home contracts without ROS or robot commands."""

from copy import deepcopy
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
import pytest
import yaml

from smart_vision_amr.mission_logic import MissionError
from smart_vision_amr.web_mission_server import (
    MissionApplication, build_parser, create_ros_bridge, load_markers,
)


ROOT = Path(__file__).resolve().parents[4]


def warehouse_markers():
    return [{'id': i, 'name': '홈' if i == 0 else f'Sector {i}',
             'x': 2.8 if i == 0 else 3.+i*.2, 'y': 1.,
             'yaw': -math.pi/2, 'camera': 'rear'} for i in range(11)]


@pytest.mark.parametrize('topic', [None, '/smart_vision_amr/display_obstacles'])
def test_display_topic_selection_keeps_default_planning_observer(tmp_path, topic):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    options = build_parser().parse_args(
        [] if topic is None else ['--observed-obstacles-topic', topic])
    app = MissionApplication(warehouse_markers(), 222, True, tmp_path/'topic.json')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=222,
               signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = create_ros_bridge(app, options, context)
        assert node.observed_obstacle_sub.topic_name == (
            topic or '/smart_vision_amr/observed_obstacles')
        assert node.memory_sub.topic_name == '/smart_vision_amr/remembered_cells'
        assert app.mission.state == 'IDLE'
        assert not app.mission.commands
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_ten_sectors_request_prearm_dispatch_and_marker_zero_home(tmp_path):
    markers = warehouse_markers()
    app = MissionApplication(markers, 178, True, tmp_path/'warehouse.json', access_mode='domain')
    home = {key: markers[0][key] for key in ('x', 'y', 'yaw')}
    assert app.delivery.registered_homepoint == home
    assert app.delivery.homepoint == home
    for ident in range(1, 11):
        app.inventory_command('/api/inventory/requests',
            {'marker_id': ident, 'item': '부품', 'quantity': 1,
             'client_request_id': f'warehouse-{ident}'}, 7)
    assert [m['id'] for m in app.inventory_state(7)['markers']] == list(range(11))
    assert not app.readiness()['ready']
    with pytest.raises(MissionError):
        app.command('/api/dispatch/start', {'loaded': True})
    assert app.mission.state == 'IDLE'

    app.connected = app.goal_available = app.planner_available = app.cancel_available = True
    app.aruco_available = True
    app.map = {'width': 1, 'height': 1, 'resolution': .05}
    app.map_matches_configured = True
    app.pose = deepcopy(home)
    now = time.monotonic()
    app.pose_at = app.scan_at = app.odom_at = app.camera_at = now
    app.require_localization_health = True
    app.localization_health_at = now
    app.localization_health = {'level': 'healthy', 'autonomous_allowed': True, 'reasons': []}
    assert app.readiness()['ready']
    assert app.delivery.dispatch_missing() == []
    with pytest.raises(MissionError, match='적재'):
        app.command('/api/dispatch/start', {'loaded': False})
    app.command('/api/dispatch/start', {'loaded': True})
    mission = app.mission
    assert mission.route == list(range(1, 11))
    assert mission.home == home

    for ident in range(1, 11):
        assert mission.expected_id == ident
        mission.drain_commands()
        mission.goal_dispatched((100, ident))
        mission.on_navigation({'status': 'SUCCEEDED',
                               'goal_stamp': {'sec': 100, 'nanosec': ident}})
        target = {key: markers[ident][key] for key in ('x', 'y', 'yaw')}
        for frame in range(mission.required_confirmations):
            mission.observe(ident*10+frame, [ident], target, True,
                            time.monotonic()+1., alignment_verified=True)
            app.sync_delivery()
    assert mission.state == 'RETURNING_HOME'
    assert mission.commands[-1] == ('goal', home)
    assert all(item['status'] == 'DELIVERED' for item in app.delivery.queue.snapshot()['requests'])
    mission.drain_commands()
    mission.goal_dispatched((101, 0))
    mission.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': 101, 'nanosec': 0}},
                          pose=home, stationary=True)
    app.sync_delivery()
    assert mission.state == 'HOME_REACHED'
    assert app.delivery.queue.snapshot()['active_trip'] is None


def test_home_pose_seed_uses_warehouse_marker_zero_and_keeps_estop(tmp_path):
    markers = warehouse_markers()
    app = MissionApplication(markers, 178, True, tmp_path/'warehouse.json', access_mode='domain')
    metadata = {'width': 200, 'height': 120, 'resolution': .05, 'frame_id': 'map',
                'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    cells = np.zeros((120, 200), dtype=np.int16)
    app._configured_map = (metadata, cells.copy(), None)
    app.receive_map(metadata, cells)
    app.rotation_clearance_radius = .22
    app.connected = True
    app.mission.emergency_stop()
    now = time.monotonic()
    app.manual_mux_at = now
    app.manual_mux_status = {'available': True, 'mode': 'estop',
                             'estop_latched': True, 'stop_acknowledged': True}
    app.scan_at = app.odom_at = app.localization_scan_at = app.localization_odom_at = now
    app.localization_services_ready = True
    app.localization_amcl_active_at = now
    app.home_seed_match = .9
    app.home_seed_match_at = now
    app.home_seed_match_key = app.home_seed_key()
    app.mission.drain_commands()
    app.command('/api/localization/seed-home', {'confirm_actual_home': True})
    assert app.home_seed_request['home'] == {'x': 2.8, 'y': 1., 'yaw': -math.pi/2}
    assert app.mission.estop_latched
    assert not app.mission.commands


def test_marker_display_override_is_explicit_and_navigation_yaw_is_preserved(tmp_path):
    marker_file = tmp_path/'markers.yaml'
    values = warehouse_markers()
    marker_file.write_text(yaml.safe_dump({'markers': values}))
    assert 'screen_heading_deg' not in load_markers(marker_file)[3]
    values[3]['screen_heading_deg'] = 90
    marker_file.write_text(yaml.safe_dump({'markers': values}))
    assert load_markers(marker_file)[3]['screen_heading_deg'] == 90
    assert load_markers(marker_file)[3]['yaw'] == -math.pi/2
    for invalid in (True, '90', math.nan, math.inf):
        values[3]['screen_heading_deg'] = invalid
        marker_file.write_text(yaml.safe_dump({'markers': values}))
        with pytest.raises(ValueError, match='screen heading'):
            load_markers(marker_file)
    classroom = ROOT/'ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers_20261030.yaml'
    assert next(m for m in load_markers(classroom) if m['id'] == 3)['screen_heading_deg'] == 90


def test_warehouse_launcher_has_isolated_domain_map_home_and_rear_camera(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    launcher = scripts/'run_warehouse_web.sh'
    shutil.copy2(ROOT/'scripts/run_warehouse_web.sh', launcher)
    stub = scripts/'run_web_mission.sh'
    stub.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    stub.chmod(0o755)
    for relative in ('maps/warehouse_logistics.yaml', 'config/aruco_warehouse_markers.yaml',
                     'config/warehouse_floorplan.yaml'):
        path = tmp_path/'ros2_ws/src/smart_vision_amr'/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('fixture')
    env = {key: value for key, value in os.environ.items()
           if not key.startswith('SMART_VISION_WAREHOUSE_')}
    env['HOME'] = str(tmp_path/'home')
    result = subprocess.run(['bash', str(launcher)], env=env, capture_output=True, text=True, check=True)
    args = result.stdout.splitlines()
    value = lambda option: args[args.index(option)+1]
    assert value('--domain-id') == '178'
    assert value('--port') == '8793'
    assert value('--map-file') == value('--raw-map-file')
    assert value('--markers-file').endswith('/aruco_warehouse_markers.yaml')
    assert value('--config-file').endswith('/warehouse-sim-178.json')
    assert value('--front-camera-topic') == ''
    assert value('--camera-rotation') == '0'
    assert value('--rotation-clearance-radius') == '0.22'
    assert value('--sector-dwell-seconds') == '5'
    assert value('--observed-obstacles-topic') == '/smart_vision_amr/display_obstacles'
    assert '--require-localization-health' in args
    env['SMART_VISION_WAREHOUSE_DOMAIN'] = '16'
    blocked = subprocess.run(['bash', str(launcher)], env=env, capture_output=True, text=True)
    assert blocked.returncode == 2
