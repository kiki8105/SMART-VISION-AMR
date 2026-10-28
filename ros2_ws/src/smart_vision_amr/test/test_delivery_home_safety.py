"""Home target changes never relocate the robot or dispatch queued work.

All tests use in-memory map/pose feedback and temporary files, without ROS,
network access, a running server, or any actuator command publisher.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from smart_vision_amr.mission_logic import MissionError
from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication


class DeliveryHomeSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.config = Path(self.temporary.name)/'web.json'
        self.markers = deepcopy(DEFAULT_MARKERS)
        self.home = {'x': -.11, 'y': -.79, 'yaw': math.pi}
        self.markers[0].update(self.home)
        self.app = MissionApplication(self.markers, 218, False, self.config)
        self.app._show_map(
            {'width': 84, 'height': 136, 'resolution': .05,
             'origin': {'x': -1.53, 'y': -5.69, 'yaw': 0.}, 'frame_id': 'map'},
            np.zeros((136, 84), dtype=np.int16), preview=False, source='ros', match=True)
        self.app.connected = self.app.goal_available = self.app.planner_available = True
        self.app.cancel_available = self.app.aruco_available = True
        self.app.pose = deepcopy(self.home)
        self.app.pose_at = self.app.odom_at = self.app.scan_at = self.app.camera_at = time.monotonic()
        self.app.speed = {'linear': 0., 'angular': 0.}
        self.app.delivery.queue.submit(7, {'marker_id': 2, 'item': '볼트', 'quantity': 3,
                                            'client_request_id': 'home-check'})

    def tearDown(self):
        self.temporary.cleanup()

    def test_outside_pose_cannot_replace_home_and_preserves_requests(self):
        self.app.pose.update(x=2.7515, y=.8143, yaw=2.55)
        before = self.app.delivery.queue.snapshot()
        with self.assertRaisesRegex(MissionError, '지도 범위 밖'):
            self.app.command('/api/home/set', {})
        self.assertEqual(self.app.delivery.homepoint, self.home)
        self.assertEqual(self.app.mission.home, self.home)
        self.assertEqual(self.app.delivery.queue.snapshot(), before)
        self.assertEqual(self.app.mission.commands, [])
        self.assertFalse(self.config.exists())

    def test_unknown_and_occupied_home_cells_are_rejected(self):
        metadata = self.app._display_map_metadata
        column = math.floor((self.home['x']-metadata['origin']['x'])/.05)
        row = math.floor((self.home['y']-metadata['origin']['y'])/.05)
        for value, message in ((-1, '확인되지 않은'), (65, '장애물'), (100, '장애물')):
            self.app._display_map_data[row, column] = value
            with self.subTest(value=value), self.assertRaisesRegex(MissionError, message):
                self.app.command('/api/home/set', {})
            self.assertEqual(self.app.delivery.homepoint, self.home)
            self.assertEqual(self.app.mission.commands, [])

    def test_home_validation_respects_rotated_map_origin(self):
        self.app._show_map(
            {'width': 4, 'height': 3, 'resolution': .5,
             'origin': {'x': 10., 'y': 20., 'yaw': math.pi/2}, 'frame_id': 'map'},
            np.zeros((3, 4), dtype=np.int16), preview=False, source='ros', match=True)
        self.app.pose = {'x': 9.75, 'y': 20.25, 'yaw': .1}
        self.app.command('/api/home/set', {})
        self.assertEqual(self.app.delivery.homepoint, self.app.pose)
        self.app._display_map_data[0, 0] = 100
        with self.assertRaisesRegex(MissionError, '장애물'):
            self.app.command('/api/home/set', {})

    def test_partial_real_map_geometry_cannot_bypass_validation(self):
        self.app._display_map_data = None
        with self.assertRaisesRegex(MissionError, '지도 데이터'):
            self.app.command('/api/home/set', {})

    def test_registered_home_reset_keeps_bad_pose_and_queue_but_queues_no_motion(self):
        self.app.delivery.homepoint = {'x': 2.7515, 'y': .8143, 'yaw': 2.55}
        self.app.mission.home = deepcopy(self.app.delivery.homepoint)
        self.app.pose = deepcopy(self.app.delivery.homepoint)
        self.app.save()
        requests = self.app.delivery.queue.snapshot()
        original_pose = deepcopy(self.app.pose)
        state = self.app.command('/api/home/reset', {})
        self.assertEqual(state['delivery']['homepoint'], self.home)
        self.assertEqual(state['delivery']['registered_homepoint'], self.home)
        self.assertFalse(state['delivery']['homepoint_custom'])
        self.assertTrue(state['delivery']['homepoint_valid'])
        self.assertFalse(state['delivery']['home_ready'])
        self.assertFalse(state['delivery']['dispatch_ready'])
        self.assertTrue(any('홈까지' in message for message in state['delivery']['dispatch_missing']))
        self.assertEqual(self.app.mission.home, self.home)
        self.assertEqual(self.app.pose, original_pose)
        self.assertEqual(self.app.delivery.queue.snapshot(), requests)
        self.assertEqual(self.app.mission.commands, [])
        self.assertEqual(json.loads(self.config.read_text())['domains']['218']['homepoint'], self.home)

    def test_failed_registered_home_reset_rolls_back_memory_and_persistence(self):
        previous = {'x': .5, 'y': -.5, 'yaw': 0.}
        self.app.delivery.homepoint = deepcopy(previous)
        self.app.mission.home = deepcopy(previous)
        self.app.save()
        saved, stored = deepcopy(self.app.saved), self.config.read_bytes()
        with patch('smart_vision_amr.web_mission_server.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.app.command('/api/home/reset', {})
        self.assertEqual(self.app.delivery.homepoint, previous)
        self.assertEqual(self.app.mission.home, previous)
        self.assertEqual(self.app.saved, saved)
        self.assertEqual(self.config.read_bytes(), stored)
        self.assertEqual(self.app.mission.commands, [])

    def test_reset_is_rejected_during_any_motion_or_handover(self):
        for name, value in (('navigation_active', True), ('cancel_pending', True),
                            ('pending_connection', (219, False))):
            with self.subTest(name=name), patch.object(self.app, name, value):
                with self.assertRaises(MissionError):
                    self.app.command('/api/home/reset', {})
        for machine, state in ((self.app.manual, 'ACQUIRING'),
                               (self.app.localization, 'ROTATING')):
            with self.subTest(state=state), patch.object(machine, 'state', state):
                with self.assertRaises(MissionError):
                    self.app.command('/api/home/reset', {})
        self.app.speed['linear'] = .03
        with self.assertRaisesRegex(MissionError, '정지'):
            self.app.command('/api/home/reset', {})
        self.assertEqual(self.app.mission.commands, [])

    def test_departure_reasons_explain_distance_heading_and_never_mutate_queue(self):
        before = self.app.delivery.queue.snapshot()
        self.assertTrue(self.app.delivery.snapshot()['dispatch_ready'])
        self.app.pose.update(x=1.5, yaw=0.)
        state = self.app.delivery.snapshot()
        self.assertFalse(state['dispatch_ready'])
        self.assertTrue(any('홈까지' in message for message in state['dispatch_missing']))
        self.assertTrue(any('방향 정렬' in message for message in state['dispatch_missing']))
        self.app.scan_at = 0.
        self.assertIn('최신 /scan', self.app.delivery.snapshot()['dispatch_missing'])
        self.assertEqual(self.app.delivery.queue.snapshot(), before)
        self.assertEqual(self.app.mission.commands, [])

    def test_legacy_invalid_saved_home_cannot_dispatch_even_if_pose_matches_it(self):
        bad = {'x': 2.7515, 'y': .8143, 'yaw': 2.55}
        self.app.delivery.homepoint = deepcopy(bad)
        self.app.pose = deepcopy(bad)
        before = self.app.delivery.queue.snapshot()
        with self.assertRaisesRegex(MissionError, '지도 범위 밖'):
            self.app.command('/api/dispatch/start', {'loaded': True})
        state = self.app.delivery.snapshot()
        self.assertFalse(state['homepoint_valid'])
        self.assertIn('지도 범위 밖', state['homepoint_error'])
        self.assertEqual(self.app.delivery.queue.snapshot(), before)
        self.assertEqual(self.app.mission.commands, [])


if __name__ == '__main__':
    unittest.main()
