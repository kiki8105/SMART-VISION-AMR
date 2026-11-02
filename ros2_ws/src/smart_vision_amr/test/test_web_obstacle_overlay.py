"""Live obstacle artwork follows the received map; it never changes planning."""
from pathlib import Path
from tempfile import TemporaryDirectory
import time

import numpy as np

from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication


def setup_app(directory):
    app = MissionApplication(DEFAULT_MARKERS, 16, False, Path(directory) / 'web.json')
    metadata = {'width': 4, 'height': 3, 'resolution': .05,
                'origin': {'x': 0., 'y': 0., 'yaw': 0.}, 'frame_id': 'map'}
    # ROS OccupancyGrid indices are bottom-left row-major. Known-free additions
    # at 1 and 6 are visible; an old wall (2) and unknown (5) are not.
    grid = np.array([[0, 0, 100, 0], [0, -1, 0, 0], [0, 0, 0, 0]], dtype=np.int16)
    app.receive_map(metadata, grid)
    return app, metadata, grid


def test_only_known_free_live_obstacles_are_shown_and_empty_message_removes_them():
    with TemporaryDirectory() as directory:
        app, _, _ = setup_app(directory)
        assert app.receive_observed_obstacles([6, 2, 1, 5, 6])
        observed = app.state()['observed_obstacles']
        assert observed['cells'] == [1, 6]
        assert observed['map_version'] == app.map_version
        assert observed['age'] >= 0.
        assert app.receive_observed_obstacles([])
        assert app.state()['observed_obstacles']['cells'] == []


def test_bad_indices_cannot_replace_a_valid_overlay_and_stale_data_disappears():
    with TemporaryDirectory() as directory:
        app, _, _ = setup_app(directory)
        assert app.receive_observed_obstacles([1])
        for bad in ([-1], [12], [1.0], [True], ['1'], [1] * 13):
            assert not app.receive_observed_obstacles(bad)
            assert app.state()['observed_obstacles']['cells'] == [1]
        app.observed_obstacles_at -= 3.
        assert app.state()['observed_obstacles']['cells'] == []


def test_overlay_clears_on_map_change_or_preview_and_refuses_old_map_cells():
    with TemporaryDirectory() as directory:
        app, metadata, grid = setup_app(directory)
        assert app.receive_observed_obstacles([1])
        app.path = [{'x': 1., 'y': 1.}]
        app.path_stamp = {'sec': 23, 'nanosec': 4}
        assert app.state()['path_stamp'] == app.path_stamp
        changed = grid.copy()
        changed[0, 1] = 100
        app.receive_map(metadata, changed)
        assert app.state()['observed_obstacles']['cells'] == []
        assert app.state()['path'] == [], 'old-map route cannot overlay a new map'
        assert app.state()['path_stamp'] is None
        assert app.receive_observed_obstacles([1])
        assert app.state()['observed_obstacles']['cells'] == [], 'new static wall is not unexpected'
        app.restore_map_preview()
        assert not app.receive_observed_obstacles([6])
        assert app.state()['observed_obstacles']['cells'] == []
        assert app.path == []


def test_direct_ros_simulation_path_is_read_only_and_never_exposed_on_physical_domain():
    with TemporaryDirectory() as directory:
        app = MissionApplication(DEFAULT_MARKERS, 176, True, Path(directory) / 'sim.json')
        metadata = {'width': 4, 'height': 3, 'resolution': .05,
                    'origin': {'x': 0., 'y': 0., 'yaw': 0.}, 'frame_id': 'map'}
        app.receive_map(metadata, np.zeros((3, 4), dtype=np.int16))
        app.path = [{'x': .1, 'y': .2}]
        app.path_stamp = {'sec': 30, 'nanosec': 4}
        app.navigation['planner_status'] = 'READY'
        app.planner_available = True
        app.sim_clock_at = app.scan_at = app.odom_at = time.monotonic()
        app.planner_details = {'plan_kind': 'LOCAL_DETOUR', 'path_stamp': dict(app.path_stamp),
                               'goal': {'x': .1, 'y': .2, 'frame_id': 'map'}}
        state = app.state()
        assert state['mission']['state'] == 'IDLE'
        app.planner_details['goal_stamp'] = {'sec': 29, 'nanosec': 1}
        app.planner_details_at = time.monotonic() - 60.
        state = app.state()
        assert state['observed_route']['active'], 'a live publisher may keep a long-running READY path without status heartbeat'
        assert state['observed_route'] == {'monitoring': True, 'stale': False, 'reason': '', 'phase': 'READY',
                                           'active': True, 'plan_kind': 'LOCAL_DETOUR',
                                           'goal': {'x': .1, 'y': .2, 'frame_id': 'map'},
                                           'goal_stamp': {'sec': 29, 'nanosec': 1},
                                           'path_stamp': app.path_stamp}
        assert app.mission.commands == [], 'observing a ROS path must not dispatch goals'
        app.navigation['planner_status'] = 'WAIT_REPLAN'
        assert app.state()['observed_route']['monitoring']
        assert not app.state()['observed_route']['active']
        app.sim_clock_at -= 3.
        stale = app.state()
        assert stale['observed_route']['monitoring']
        assert stale['observed_route']['stale']
        assert stale['observed_route']['phase'] == 'STALE'
        assert stale['observed_route']['goal'] is None
        assert stale['path'] == []
        assert '/clock' in stale['observed_route']['reason']
        app.sim_clock_at = time.monotonic()
        app.scan_at -= 3.
        assert '/scan' in app.state()['observed_route']['reason']
        app.scan_at = time.monotonic()
        app.odom_at -= 3.
        assert '/odom' in app.state()['observed_route']['reason']
        app.odom_at = time.monotonic()
        app.planner_available = False
        assert 'A* 계획 노드' in app.state()['observed_route']['reason']
        app.planner_available = True
        app.sim_external_goal_seen = True
        app.navigation['planner_status'] = 'IDLE'
        finished = app.state()
        assert finished['observed_route']['monitoring']
        assert finished['observed_route']['stale']
        assert finished['observed_route']['phase'] == 'STALE'
        assert finished['observed_route']['goal'] is None
        assert finished['path'] == [], 'a 1-point fallback Path must not survive as an active route'
        assert '시험 종료' in finished['observed_route']['reason']
        app.domain_id = 16
        app.navigation['planner_status'] = 'READY'
        assert not app.state()['observed_route']['active']
        assert app.state()['observed_route']['goal'] is None


def test_simulation_goal_overlay_rejects_non_map_and_nonfinite_coordinates():
    with TemporaryDirectory() as directory:
        app, _, _ = setup_app(directory)
        app.domain_id = 176
        app.use_sim_time = True
        app.navigation['planner_status'] = 'PLANNING'
        for bad in ({'x': .1, 'y': .2, 'frame_id': 'odom'},
                    {'x': float('nan'), 'y': .2, 'frame_id': 'map'},
                    {'x': True, 'y': .2, 'frame_id': 'map'}):
            app.planner_details = {'goal': bad}
            assert app.state()['observed_route']['goal'] is None
