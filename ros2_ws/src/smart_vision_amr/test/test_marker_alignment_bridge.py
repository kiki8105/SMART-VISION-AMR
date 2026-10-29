"""Arrival/camera/rotation interlocks with fake outputs on isolated domain 219."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions

from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, create_ros_bridge, marker_measurements,
)


def corners(cx=50., cy=50., side=20.):
    d = side/2
    return [np.array([[[cx-d, cy-d], [cx+d, cy-d],
                       [cx+d, cy+d], [cx-d, cy+d]]], dtype=np.float32)]


class GeometryTests(unittest.TestCase):
    def test_center_is_normalized_in_the_upright_image(self):
        self.assertEqual(marker_measurements(corners(), [[3]], 100, 100),
                         [{'id': 3, 'center_x': 0., 'confidence': 1.}])
        self.assertAlmostEqual(marker_measurements(corners(70), [[3]], 100, 100)[0]['center_x'], .4)

    def test_an_id_without_safe_corner_geometry_cannot_confirm(self):
        for value in ([], corners(3), corners(side=3), corners(math.nan)):
            self.assertEqual(marker_measurements(value, [[3]], 100, 100), [])
        self.assertEqual(marker_measurements(corners(), None, 100, 100), [])


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.app = MissionApplication(DEFAULT_MARKERS, 219, False,
                                      Path(self.temp.name)/'settings.json')
        self.context = Context()
        rclpy.init(context=self.context, domain_id=219,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = create_ros_bridge(self.app, SimpleNamespace(
            camera_topic='/isolated_marker_camera', base_frame='isolated_marker_base'), self.context)
        # No executor is spun; replace every ROS output before calling methods.
        self.node.motion_pub = Publisher()
        self.node.goal_pub = Publisher()
        self.node.initial_pose_pub = Publisher()
        self.now = 100.
        self.clock = patch('smart_vision_amr.web_mission_server.time.monotonic',
                           side_effect=lambda: self.now)
        self.clock.start()
        self.app.mission.clock = lambda: self.now
        self.app.mission.update_route([3])
        self.app.mission.markers[3].update(x=.4, y=.4, yaw=0., camera='front')
        self.app.mission.state = 'CONFIRMING'
        self.app.mission.goal_stamp = (1, 0)
        self.app.mission.observation_since = self.now
        self.node.observation_started_ros_ns = 10
        self.frame = 10
        self.image = np.zeros((100, 100, 3), dtype=np.uint8)
        self.refresh()

    def refresh(self):
        self.app.pose = {'x': .4, 'y': .4, 'yaw': .6}
        self.app.odom_pose = {'x': .4, 'y': .4, 'yaw': .6}
        self.app.speed = {'linear': 0., 'angular': 0.}
        self.app.pose_at = self.app.odom_at = self.app.scan_at = self.now
        self.app.localization_odom_at = self.app.localization_scan_at = self.now
        self.app.front_camera_at = self.app.camera_at = self.now
        self.app.localization_clear = True
        self.app.localization_health = {'level': 'ok', 'autonomous_allowed': True}
        self.app.localization_health_at = self.now

    def tearDown(self):
        self.clock.stop()
        self.node.destroy_node()
        self.context.shutdown()
        self.temp.cleanup()

    def tick(self, delta=0.):
        self.now += delta
        self.refresh()
        self.node.tick_marker_alignment(self.now, True, True)

    def frame_at(self, cx=50., camera='front', ids=None, capture=None, fresh=True, capture_age=0.):
        self.frame += 1
        return self.node.process_marker_frame(camera, capture or self.frame, corners(cx),
            np.array([[3]] if ids is None else ids), self.image, self.now, fresh, capture_age)

    def test_fresh_blank_frame_enables_rotation_only_after_the_hold_ack(self):
        self.node.tick_marker_alignment(self.now, False, True)
        self.assertEqual(self.node.marker_alignment_angular, 0.)
        self.assertIsNone(self.node.marker_alignment_key)
        self.tick()
        self.frame_at(ids=[])
        self.tick(.1)
        self.assertEqual(self.node.marker_alignment_angular, .12)
        self.node.send_control('manual', 0., self.node.marker_alignment_angular,
                               operation='marker_alignment')
        packet = json.loads(self.node.motion_pub.messages[-1].data)
        self.assertEqual((packet['operation'], packet['mode'], packet['linear']),
                         ('marker_alignment', 'manual', 0.))
        self.assertEqual(self.node.goal_pub.messages, [])

    def test_centering_replaces_fixed_yaw_only_after_stop_and_new_stable_frames(self):
        self.tick()
        self.assertFalse(self.frame_at(75.))
        self.tick(.1)
        self.assertLess(self.node.marker_alignment_angular, 0.)
        verified = False
        for _ in range(10):
            verified = self.frame_at()
            self.app.mission.observe(self.frame, [3], self.app.pose, True, self.now,
                                     alignment_verified=verified)
            if self.app.mission.state == 'COMPLETED':
                break
            self.tick(.15)
        self.assertTrue(verified)
        self.assertEqual(self.app.mission.completed_ids, [3])
        self.assertEqual(self.app.mission.state, 'COMPLETED')
        self.assertGreater(abs(self.app.pose['yaw']), self.app.mission.yaw_tolerance)

    def test_wrong_camera_prearrival_and_stale_frames_never_count(self):
        self.tick()
        self.assertFalse(self.frame_at(camera='rear'))
        self.assertFalse(self.frame_at(capture=1))
        self.assertFalse(self.frame_at(fresh=False))
        self.assertFalse(self.frame_at(capture_age=.9))
        self.assertEqual(self.node.marker_alignment.frames, 0)

    def test_clearance_or_localization_failure_retains_goal_and_pauses(self):
        for bad_field in ('clearance', 'quality'):
            self.app.mission.state = 'CONFIRMING'
            self.node.marker_alignment_key = None
            self.tick()
            self.frame_at()
            if bad_field == 'clearance':
                self.app.localization_clear = False
            else:
                self.app.localization_health['autonomous_allowed'] = False
            self.node.tick_marker_alignment(self.now, True, True)
            self.assertEqual(self.node.marker_alignment_angular, 0.)
            self.assertEqual(self.app.mission.state, 'PAUSED')
            self.assertEqual(self.app.mission.expected_id, 3)
            self.assertEqual(self.app.mission.completed_ids, [])

    def test_confirmation_is_revoked_when_a_new_frame_moves_off_center(self):
        self.tick()
        for _ in range(8):
            self.frame_at()
            self.tick(.15)
        self.assertEqual(self.node.marker_alignment.state, 'CONFIRMED')
        self.assertFalse(self.frame_at(80.))
        self.assertEqual(self.node.marker_alignment.state, 'CENTERING')

    def test_replayed_frame_or_previous_goal_cannot_reuse_a_centering_approval(self):
        self.tick()
        for _ in range(8):
            self.frame_at()
            self.tick(.15)
        self.assertEqual(self.node.marker_alignment.state, 'CONFIRMED')
        self.assertFalse(self.frame_at(capture=self.frame))
        self.assertFalse(self.frame_at(capture=self.frame-2))
        self.app.mission.goal_stamp = (2, 0)
        self.assertFalse(self.frame_at())
        self.assertEqual(self.app.mission.completed_ids, [])

    def test_search_timeout_pauses_instead_of_claiming_delivery(self):
        self.tick()
        self.frame_at(ids=[])
        self.tick(45.1)
        self.assertEqual(self.node.marker_alignment_angular, 0.)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(self.app.mission.completed_ids, [])
