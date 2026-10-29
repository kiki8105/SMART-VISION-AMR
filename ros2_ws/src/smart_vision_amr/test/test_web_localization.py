"""Localization HTTP/bridge contracts without commanding any live robot.

Optional ROS cases create their own domain 218 context. All bridge publishers
and service clients are replaced before a timer callback is called; no executor
is spun, and no goal, motor command, or AMCL reset is sent through DDS.
"""
from concurrent.futures import Future
from copy import deepcopy
import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from uuid import uuid4

import numpy as np

from smart_vision_amr.localization_geometry import (
    occupied_distance, rotation_clearance, scan_map_agreement, scan_points,
)
from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, MissionHTTPServer,
    MissionHTTPRequestHandler, create_ros_bridge,
)

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from sensor_msgs.msg import Image
    from std_srvs.srv import Empty, Trigger
    from tf2_ros import TransformException
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


def ready_application(app, now):
    app.connected = app.goal_available = app.planner_available = True
    app.cancel_available = True
    app.map = {'width': 10, 'height': 10, 'resolution': .1,
               'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    app.map_is_preview = False
    app.map_matches_configured = True
    app.pose = {'x': .4, 'y': .4, 'yaw': 0.}
    app.pose_at = app.scan_at = app.odom_at = app.camera_at = now
    app.manual_mux_at = now
    app.manual_mux_status = {'mode': 'hold'}
    app.localization_services_ready = True
    app.localization_amcl_active_at = app.localization_scan_at = app.localization_odom_at = now
    app.localization_clear = True
    app.localization_clearance = 1.


def mission_bookkeeping(app):
    mission = app.mission
    return deepcopy((mission.route, mission.current_index, mission.completed_ids,
                     mission.home, mission.returning_home))


def start_mid_mission(app):
    mission = app.mission
    mission.update_route([1, 2, 3])
    mission.start(home={'x': .4, 'y': .4, 'yaw': 0.}, auto_return_home=True)
    mission.drain_commands()
    mission.current_index = 1
    mission.completed_ids = [1]
    mission._send_current_goal()
    mission.drain_commands()


class LocalizationHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.app = MissionApplication(DEFAULT_MARKERS, 218, False,
            Path(self.temp.name)/'settings.json', access_mode='domain')
        self.server = MissionHTTPServer(('127.0.0.1', 0), self.app, self.temp.name)
        self.thread = threading.Thread(target=lambda: self.server.serve_forever(.01), daemon=True)
        self.thread.start()
        self.remote = patch.object(MissionHTTPRequestHandler, 'local', property(lambda _: False))
        self.remote.start()
        ready_application(self.app, time.monotonic())
        self.cookie = 'amr_session='+self.app.pair('operator', '', domain_id=218)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.remote.stop()
        self.temp.cleanup()

    def request(self, endpoint, cookie=None, origin=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        headers = {'Content-Type': 'application/json'}
        if cookie:
            headers['Cookie'] = cookie
        if origin:
            headers['Origin'] = origin
        connection.request('POST', endpoint, '{}', headers)
        response = connection.getresponse()
        body, status = json.loads(response.read()), response.status
        connection.close()
        return status, body

    def test_start_and_stop_are_operator_only(self):
        manager = self.app.pair_inventory('manager', 4, '', domain_id=218)
        for endpoint in ('/api/localization/start', '/api/localization/stop', '/api/localization/seed-home'):
            for cookie in (None, 'amr_inventory_session='+manager):
                status, body = self.request(endpoint, cookie)
                self.assertEqual(status, 401, body)
        self.assertFalse(self.app.localization.active)

    def test_cross_origin_cannot_start_physical_rotation(self):
        status, body = self.request('/api/localization/start', self.cookie, 'https://foreign.example')
        self.assertEqual(status, 403, body)
        self.assertFalse(self.app.localization.active)

    def test_unknown_initial_pose_does_not_block_localization(self):
        self.app.pose = None
        self.app.pose_at = self.app.camera_at = 0.
        status, body = self.request('/api/localization/start', self.cookie)
        self.assertEqual(status, 200, body)
        self.assertEqual(self.app.localization.state, 'ACQUIRING')
        self.assertTrue(self.app.localization_requested)
        self.assertIsNone(body['localization']['estimate'])
        self.assertEqual(body['localization']['mode'], 'global')

    def install_known_free_map(self):
        self.app._show_map(self.app.map, np.zeros((10, 10), dtype=np.int16),
                           preview=False, source='ros', match=True)

    def test_known_free_current_pose_selects_local_refinement(self):
        self.install_known_free_map()
        before = deepcopy(self.app.pose)
        status, body = self.request('/api/localization/start', self.cookie)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['localization']['mode'], 'local')
        self.assertTrue(body['localization']['use_existing_pose'])
        self.assertEqual(self.app.localization_reference_pose, before)
        self.assertFalse(self.app.localization_invalidated)
        self.assertIn('현재 AMCL 위치를 유지', body['events'][-1]['message'])

    def test_startup_home_initialization_uses_registered_home_without_camera(self):
        self.install_known_free_map()
        self.app.delivery.registered_homepoint = {'x': .4, 'y': .4, 'yaw': 0.}
        self.app.pose = None
        self.app.camera_at = 0.
        status, body = self.request('/api/localization/start-home', self.cookie)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['localization']['mode'], 'home')
        self.assertEqual(self.app.localization_home_seed, {'x': .4, 'y': .4, 'yaw': 0.})
        self.assertTrue(self.app.localization.active)
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_startup_home_rejects_conflicting_marker_and_active_mission(self):
        self.install_known_free_map()
        self.app.delivery.registered_homepoint = {'x': .4, 'y': .4, 'yaw': 0.}
        self.app.detected_ids = [2]
        status, body = self.request('/api/localization/start-home', self.cookie)
        self.assertEqual(status, 400, body)
        self.assertIn('홈이 아닌', body['error'])
        self.app.detected_ids = [0]
        start_mid_mission(self.app)
        status, body = self.request('/api/localization/start-home', self.cookie)
        self.assertEqual(status, 400, body)
        self.assertFalse(self.app.localization.active)

    def test_startup_home_rejects_occupied_home_and_unclear_rotation(self):
        self.install_known_free_map()
        self.app.delivery.registered_homepoint = {'x': .4, 'y': .4, 'yaw': 0.}
        self.app._display_map_data[4, 4] = 100
        status, _ = self.request('/api/localization/start-home', self.cookie)
        self.assertEqual(status, 400)
        self.app._display_map_data[4, 4] = 0
        self.app.localization_clear = False
        status, _ = self.request('/api/localization/start-home', self.cookie)
        self.assertEqual(status, 400)

    def test_outside_unknown_occupied_or_invalidated_pose_requires_global_search(self):
        for kind in ('outside', 'unknown', 'occupied', 'invalidated', 'stale'):
            with self.subTest(kind=kind):
                ready_application(self.app, time.monotonic())
                self.install_known_free_map()
                self.app.localization_invalidated = False
                if kind == 'outside':
                    self.app.pose['x'] = 2.
                elif kind == 'unknown':
                    self.app._display_map_data[4, 4] = -1
                elif kind == 'occupied':
                    self.app._display_map_data[4, 4] = 100
                elif kind == 'invalidated':
                    self.app.localization_invalidated = True
                else:
                    self.app.pose_at = time.monotonic()-3.
                status, body = self.request('/api/localization/start', self.cookie)
                self.assertEqual(status, 200, body)
                self.assertEqual(body['localization']['mode'], 'global')
                self.assertFalse(body['localization']['use_existing_pose'])
                self.app.command('/api/localization/stop', {})

    def test_local_cancel_does_not_invalidate_existing_location(self):
        self.install_known_free_map()
        self.app.command('/api/localization/start', {})
        self.app.command('/api/localization/stop', {})
        self.assertFalse(self.app.localization_invalidated)
        self.assertNotIn('라이다 위치 확인 완료 또는 수동 초기 위치 지정',
                         self.app.readiness()['missing'])

    def test_readiness_rejects_stale_sensors_inactive_amcl_and_wrong_map(self):
        for field, value in (('scan_at', 0.), ('odom_at', 0.),
                             ('localization_scan_at', 0.), ('localization_clear', False),
                             ('localization_amcl_active_at', 0.),
                             ('localization_services_ready', False), ('map', None),
                             ('map_is_preview', True), ('map_matches_configured', False),
                             ('unmanaged_velocity_publishers', ['teleop'])):
            with self.subTest(field=field):
                ready_application(self.app, time.monotonic())
                self.app.unmanaged_velocity_publishers = []
                setattr(self.app, field, value)
                status, body = self.request('/api/localization/start', self.cookie)
                self.assertEqual(status, 400, body)
                self.assertFalse(self.app.localization.active)

    def test_start_preserves_route_home_completed_and_current_goal(self):
        start_mid_mission(self.app)
        before = mission_bookkeeping(self.app)
        status, body = self.request('/api/localization/start', self.cookie)
        self.assertEqual(status, 200, body)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_stop_remains_paused_without_restarting_the_mission(self):
        start_mid_mission(self.app)
        self.assertEqual(self.request('/api/localization/start', self.cookie)[0], 200)
        before = mission_bookkeeping(self.app)
        self.app.mission.drain_commands()
        status, body = self.request('/api/localization/stop', self.cookie)
        self.assertEqual(status, 200, body)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertTrue(self.app.localization_stop_requested)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_mission_start_resume_and_domain_change_cannot_bypass_localization(self):
        self.app.command('/api/localization/start', {})
        for endpoint in ('/api/mission/start', '/api/mission/resume',
                         '/api/mission/restart', '/api/connection', '/api/home/set'):
            with self.subTest(endpoint=endpoint):
                with self.assertRaises(ValueError):
                    self.app.command(endpoint, {'domain_id': 219})
        self.assertIsNone(self.app.pending_connection)
        self.assertEqual(self.app.domain_id, 218)

    def test_estop_cancels_localization_and_latches_safety(self):
        self.app.command('/api/localization/start', {})
        self.app.command('/api/emergency-stop', {})
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertTrue(self.app.mission.estop_latched)
        with self.assertRaises(ValueError):
            self.app.command('/api/localization/start', {})

    def test_joystick_preempts_localization_without_resuming_paused_mission(self):
        start_mid_mission(self.app)
        before = mission_bookkeeping(self.app)
        self.app.command('/api/localization/start', {})
        self.app.manual_command('operator', {'control_id': str(uuid4()), 'seq': 1,
            'active': True, 'linear': 0., 'angular': .2})
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertEqual(self.app.manual.state, 'ACQUIRING')
        self.assertFalse(self.app.manual.resume_mission)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)

    def test_pending_old_amcl_reset_cannot_start_a_new_search(self):
        self.app.localization_service_pending = True
        status, body = self.request('/api/localization/start', self.cookie)
        self.assertEqual(status, 400, body)
        self.assertEqual(self.app.localization.state, 'IDLE')


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


class _Client:
    def __init__(self, response_factory):
        self.available = True
        self.future = None
        self.requests = []
        self.response_factory = response_factory

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        self.future = Future()
        return self.future

    def acknowledge(self, **values):
        response = self.response_factory()
        for key, value in values.items():
            setattr(response, key, value)
        self.future.set_result(response)


@unittest.skipUnless(HAS_ROS, 'source ROS Humble to run isolated domain 218 tests')
class LocalizationBridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.app = MissionApplication(DEFAULT_MARKERS, 218, False,
            Path(self.temp.name)/'settings.json', access_mode='domain')
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=218,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = create_ros_bridge(self.app, SimpleNamespace(
            camera_topic='/isolated_localization_camera', base_frame='isolated_localization_base'),
            self.context)
        # These are the bridge's only output publishers. Replace before ticks.
        self.goals = self.node.goal_pub = _Publisher()
        self.seed_poses = self.node.initial_pose_pub = _Publisher()
        self.controls = self.node.motion_pub = _Publisher()
        self.cancel = self.node.cancel_client = _Client(Trigger.Response)
        self.global_reset = self.node.global_localization_client = _Client(Empty.Response)
        self.nomotion = self.node.nomotion_client = _Client(Empty.Response)
        self.amcl_state = self.node.amcl_state_client = _Client(GetState.Response)
        self.node.count_publishers = lambda topic: 1 if topic in {
            '/smart_vision_amr/planner_status', '/smart_vision_amr/navigation_cmd_vel'} else 0
        self.node.get_publishers_info_by_topic = lambda _: []
        self.node.get_subscriptions_info_by_topic = lambda topic: [
            SimpleNamespace(node_name=name) for name in (
                ('astar_planner', 'path_follower')
                if topic == '/smart_vision_amr/goal' else ('amcl',)
            )
        ]

        def no_transform(*_args, **_kwargs):
            raise TransformException('No live transforms are consulted')
        self.node.tf_buffer.lookup_transform = no_transform
        self.now = 100.
        self.monotonic = patch('smart_vision_amr.web_mission_server.time.monotonic',
                               side_effect=lambda: self.now)
        self.monotonic.start()
        self.app.localization.clock = self.app.manual.clock = lambda: self.now
        self.refresh()

    def tearDown(self):
        self.monotonic.stop()
        self.node.destroy_node()
        self.context.shutdown()
        self.temp.cleanup()

    def refresh(self):
        ready_application(self.app, self.now)
        self.app.manual_mux_status = {'mode': 'hold', 'source_id': self.node.motion_source}

    def tick(self, delta=0., refresh=True):
        self.now += delta
        if refresh:
            self.refresh()
        self.node.tick()

    def packets(self):
        return [json.loads(message.data) for message in self.controls.messages]

    def acquire(self):
        # These legacy tests intentionally exercise the destructive/global path.
        self.app.localization_invalidated = True
        self.app.command('/api/localization/start', {})
        self.tick()
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.tick(.9)
        self.assertEqual(self.app.localization.state, 'INITIALIZING')
        self.assertEqual(len(self.global_reset.requests), 1)

    def initialize(self):
        self.acquire()
        self.global_reset.acknowledge()
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.app.localization.on_odom(0., self.now)

    def acquire_local(self):
        self.app.localization_invalidated = False
        self.app._show_map(self.app.map, np.zeros((10, 10), dtype=np.int16),
                           preview=False, source='ros', match=True)
        self.app.command('/api/localization/start', {})
        self.tick()
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.tick(.9)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.global_reset.requests, [])

    def test_local_refinement_never_resets_amcl_and_requests_new_estimates(self):
        self.acquire_local()
        self.assertTrue(self.app.localization_use_existing_pose)
        self.assertFalse(self.app.localization_invalidated)
        self.assertFalse(self.app.localization_service_pending)
        self.assertIsNone(self.node.localization_future)
        self.assertGreater(self.node.localization_epoch_ns, 0)
        self.assertGreaterEqual(len(self.nomotion.requests), 1)
        self.assertEqual(self.app.localization.snapshot()['good_estimates'], 0)
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.app.localization.on_odom(0., self.now)
        self.tick(.1)
        self.assertEqual(self.packets()[-1]['angular'], .2)

    def test_startup_home_seeds_amcl_only_after_cancel_and_mux_hold(self):
        self.app.delivery.registered_homepoint = {'x': .4, 'y': .4, 'yaw': 0.}
        self.app._show_map(self.app.map, np.zeros((10, 10), dtype=np.int16),
                           preview=False, source='ros', match=True)
        self.app.pose = None
        self.app.camera_at = 0.
        self.app.command('/api/localization/start-home', {})
        self.tick()
        self.assertEqual(self.seed_poses.messages, [])
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.assertEqual(self.seed_poses.messages, [])
        self.tick(.9)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(len(self.seed_poses.messages), 1)
        self.assertEqual(self.global_reset.requests, [])
        seed = self.seed_poses.messages[0]
        self.assertEqual(seed.header.frame_id, 'map')
        self.assertEqual(seed.pose.pose.position.x, .4)
        self.assertEqual(seed.pose.covariance[0], .09)
        self.node.on_initial_pose(seed)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertIsNone(self.node.manual_initial_pose_ns)
        self.assertTrue(self.app.localization_invalidated)
        self.app.command('/api/localization/stop', {})
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_startup_home_refuses_seed_without_amcl_subscriber(self):
        self.app.delivery.registered_homepoint = {'x': .4, 'y': .4, 'yaw': 0.}
        self.app._show_map(self.app.map, np.zeros((10, 10), dtype=np.int16),
                           preview=False, source='ros', match=True)
        self.app.command('/api/localization/start-home', {})
        self.node.get_subscriptions_info_by_topic = lambda _: [SimpleNamespace(node_name='web_mission_server')]
        self.tick()
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.tick(.9)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(self.seed_poses.messages, [])
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_front_marker_counts_only_after_arrival_from_selected_camera(self):
        mission = self.app.mission
        mission.update_route([1])
        mission.markers[1]['camera'] = 'front'
        mission.markers[1].update(x=.4, y=.4, yaw=0.)
        mission.state = 'CONFIRMING'
        mission.observation_since = self.now-1.
        self.node.observation_started_ros_ns = 0
        self.node.detector = SimpleNamespace(detectMarkers=lambda _image:
            ([], np.asarray([[1]], dtype=np.int32), []))
        stamp = self.node.get_clock().now().nanoseconds
        message = Image()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        with patch('smart_vision_amr.web_mission_server.image_to_bgr',
                   return_value=np.zeros((16, 16, 3), dtype=np.uint8)), \
                patch('cv2.aruco.drawDetectedMarkers'), \
                patch.object(self.node, 'process_marker_frame', return_value=True):
            # This case isolates selected-camera routing. Alignment geometry,
            # motion/stability and safety are covered by alignment bridge tests.
            self.node.on_image(message)
            self.assertEqual(mission.confirmations, 0, 'rear detection cannot confirm a front goal')
            self.assertEqual(self.app.detected_ids, [1])
            self.node.on_front_image(message)
        self.assertEqual(mission.confirmations, 1)
        self.assertEqual(self.app.front_detected_ids, [1])
        self.assertIsNotNone(self.app.front_camera_jpeg)
        self.assertGreater(self.app.front_camera_at, 0.)

    def test_local_refinement_requires_new_epoch_and_same_strict_scan_quality(self):
        self.acquire_local()
        epoch = self.node.localization_epoch_ns
        self.node.source_fresh = lambda *_args, **_kwargs: True
        self.node.localization_scan = SimpleNamespace()
        self.node.localization_laser_pose = (0., 0., 0.)
        self.app.localization_distance = np.ones((10, 10))
        with patch('smart_vision_amr.web_mission_server.scan_points', return_value=np.ones((20, 2))), \
                patch('smart_vision_amr.web_mission_server.scan_map_agreement', side_effect=[1., .75, .90]):
            for stamp in (epoch-1, epoch+1, epoch+2):
                self.now += .1
                self.refresh()
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                message.pose.pose.position.x = message.pose.pose.position.y = .4
                message.pose.pose.orientation.w = 1.
                message.pose.covariance[0] = message.pose.covariance[7] = .01
                message.pose.covariance[35] = .01
                self.node.on_amcl(message)
                self.assertEqual(self.app.localization.snapshot()['good_estimates'],
                                 1 if stamp == epoch+2 else 0)
        self.assertEqual(self.global_reset.requests, [])

    def test_local_clearance_failure_preserves_pose_and_resume_readiness(self):
        start_mid_mission(self.app)
        before = mission_bookkeeping(self.app)
        self.acquire_local()
        position = deepcopy(self.app.pose)
        self.app.localization.on_odom(0., self.now)
        self.tick(.1)
        self.now += .1
        self.refresh()
        self.app.localization_clear = False
        self.node.tick()
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertFalse(self.app.localization_invalidated)
        self.assertEqual(self.app.pose, position)
        self.assertEqual(self.global_reset.requests, [])
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)
        self.assertTrue(self.app.readiness()['ready'])
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_local_explicit_cancel_preserves_location_without_global_reset(self):
        self.acquire_local()
        self.app.command('/api/localization/stop', {})
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertFalse(self.app.localization_invalidated)
        self.assertEqual(self.global_reset.requests, [])
        self.assertTrue(self.app.readiness()['ready'])

    def test_local_pose_becoming_invalid_during_handover_does_not_trigger_global_reset(self):
        self.app._show_map(self.app.map, np.zeros((10, 10), dtype=np.int16),
                           preview=False, source='ros', match=True)
        self.app.command('/api/localization/start', {})
        self.tick()
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.now += .9
        self.refresh()
        self.app.pose['x'] = 2.
        self.node.tick()
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertFalse(self.app.localization_invalidated)
        self.assertEqual(self.global_reset.requests, [])

    def test_odometry_pose_is_stored_separately_from_map_localization(self):
        self.node.source_fresh = lambda *_args, **_kwargs: True
        message = Odometry()
        message.pose.pose.position.x, message.pose.pose.position.y = 1.2, -.3
        message.pose.pose.orientation.z = math.sin(.4/2)
        message.pose.pose.orientation.w = math.cos(.4/2)
        before = deepcopy(self.app.pose)
        self.node.on_odom(message)
        self.assertEqual(self.app.odom_pose['x'], 1.2)
        self.assertEqual(self.app.odom_pose['y'], -.3)
        self.assertAlmostEqual(self.app.odom_pose['yaw'], .4)
        self.assertEqual(self.app.pose, before)

    def invalid_scan_tick(self, delta=.1, *, clearance=None, stale=False):
        self.now += delta
        self.refresh()
        self.app.localization.on_odom(0., self.now)
        self.app.localization_clear = False
        self.app.localization_clearance = clearance
        if stale:
            self.app.localization_scan_at = self.now-2.
        self.node.tick()

    def test_transient_invalid_scan_stops_then_valid_scan_resumes_local_rotation(self):
        self.acquire_local()
        self.app.localization.on_odom(0., self.now)
        self.tick(.1)
        self.assertEqual(self.packets()[-1]['angular'], .2)
        self.invalid_scan_tick()
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.assertIn('누락 복구', self.app.localization.reason)
        self.assertFalse(self.app.localization.succeeded)
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.packets()[-1]['angular'], .2)
        self.assertNotIn('누락 복구', self.app.localization.reason)
        self.assertFalse(self.app.localization_invalidated)

    def test_invalid_scan_more_than_one_second_fails_and_keeps_existing_pose(self):
        self.acquire_local()
        self.invalid_scan_tick()
        self.invalid_scan_tick(.6)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.invalid_scan_tick(.5)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertIn('1초 이상', self.app.localization.reason)
        self.assertEqual(self.packets()[-1]['mode'], 'hold')
        self.assertFalse(self.app.localization_invalidated)

    def test_measured_close_obstacle_never_gets_invalid_scan_grace(self):
        self.acquire_local()
        self.invalid_scan_tick(clearance=.20)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertIn('회전 공간', self.app.localization.reason)
        self.assertEqual(self.packets()[-1]['angular'], 0.)

    def test_stale_scan_pauses_immediately_then_fails_after_bounded_gap(self):
        self.acquire_local()
        self.invalid_scan_tick(stale=True)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.assertIn('수신 복구', self.app.localization.reason)
        self.invalid_scan_tick(1.01, stale=True)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(self.packets()[-1]['angular'], 0.)

    def test_short_stale_scan_gap_resumes_only_after_new_odom_baseline(self):
        self.acquire_local()
        self.app.localization.on_odom(0., self.now)
        self.tick(.1)
        self.assertEqual(self.packets()[-1]['angular'], .2)
        before_rotation = self.app.localization.rotation
        self.invalid_scan_tick(stale=True)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertIsNone(self.app.localization.last_odom_at)
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.now += .3
        self.refresh()
        self.app.localization.on_odom(.1, self.now)
        self.node.tick()
        self.assertEqual(self.app.localization.rotation, before_rotation)
        self.assertEqual(self.packets()[-1]['angular'], .2)

    def test_delayed_scan_receipt_does_not_refresh_rotation_clearance(self):
        message = LaserScan()
        stamp = self.node.get_clock().now().nanoseconds-1_200_000_000
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.range_min, message.range_max = .12, 3.5
        message.ranges = [1.]*360
        self.app.localization_scan_at = self.now-2.
        self.node.on_scan(message)
        self.assertEqual(self.app.scan_at, self.now)
        self.assertEqual(self.app.localization_scan_at, self.now-2.)
        self.assertFalse(self.app.localization_readiness()[0])

    def test_delayed_odometry_cannot_keep_autonomous_rotation_alive(self):
        self.initialize()
        message = Odometry()
        stamp = self.node.get_clock().now().nanoseconds-1_200_000_000
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.pose.pose.orientation.w = 1.
        self.app.localization_odom_at = self.now-2.
        self.node.on_odom(message)
        self.assertEqual(self.app.odom_at, self.now)
        self.assertEqual(self.app.localization_odom_at, self.now-2.)
        self.tick(.1, refresh=False)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.tick(1.01, refresh=False)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_reset_and_rotation_wait_for_real_cancel_acknowledgment(self):
        self.app.command('/api/localization/start', {})
        self.tick()
        self.tick(.9)
        self.assertEqual(self.app.localization.state, 'ACQUIRING')
        self.assertEqual(len(self.global_reset.requests), 0)
        self.assertTrue(all(packet['mode'] == 'hold' for packet in self.packets()))
        self.cancel.acknowledge(success=False)
        self.tick(.2)
        self.tick(1.1)
        self.assertEqual(len(self.global_reset.requests), 0)
        self.assertTrue(all(packet['angular'] == 0. for packet in self.packets()))

    def test_cancel_ack_without_exclusive_mux_hold_cannot_reset_or_move(self):
        self.app.command('/api/localization/start', {})
        self.tick()
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.now += 1.
        self.refresh()
        self.app.manual_mux_status = {'mode': 'hold', 'source_id': 'another-controller'}
        self.node.tick()
        self.assertEqual(self.app.localization.state, 'ACQUIRING')
        self.assertEqual(len(self.global_reset.requests), 0)
        self.assertTrue(all(packet['angular'] == 0. for packet in self.packets()))

    def test_only_after_reset_ack_and_new_odom_may_rotation_begin(self):
        self.acquire()
        self.tick(.2)
        self.assertTrue(all(packet['angular'] == 0. for packet in self.packets()))
        self.global_reset.acknowledge()
        self.tick(.1)
        self.assertEqual(self.packets()[-1]['angular'], 0.)
        self.app.localization.on_odom(0., self.now)
        self.tick(.1)
        self.assertEqual(self.packets()[-1]['mode'], 'manual')
        self.assertEqual(self.packets()[-1]['linear'], 0.)
        self.assertEqual(self.packets()[-1]['angular'], .2)

    def test_inactive_amcl_response_fails_before_global_reset(self):
        self.app.command('/api/localization/start', {})
        self.tick()
        response = GetState.Response()
        response.current_state.id = 2  # INACTIVE
        self.amcl_state.future.set_result(response)
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(len(self.global_reset.requests), 0)
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_blocked_clearance_stops_even_with_fresh_odometry(self):
        self.initialize()
        self.tick(.1)
        self.assertEqual(self.packets()[-1]['angular'], .2)
        self.now += .1
        self.refresh()
        self.app.localization_clear = False
        self.node.tick()
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(self.packets()[-1]['mode'], 'hold')
        self.assertEqual(self.packets()[-1]['angular'], 0.)

    def test_stale_scan_stops_instead_of_continuing_a_blind_sweep(self):
        self.initialize()
        self.tick(.1)
        self.now += .9
        self.refresh()
        self.app.scan_at = self.app.localization_scan_at = self.now-2.
        self.node.tick()
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_changed_live_map_stops_rotation_instead_of_matching_wrong_map(self):
        self.initialize()
        self.tick(.1)
        self.now += .1
        self.refresh()
        self.app.map_matches_configured = False
        self.node.tick()
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertEqual(self.packets()[-1]['mode'], 'hold')

    def test_cancelled_reset_future_cannot_restart_motion_when_late_ack_arrives(self):
        self.acquire()
        self.app.command('/api/localization/stop', {})
        self.global_reset.acknowledge()
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertFalse(self.app.localization_service_pending)
        self.assertTrue(all(packet['angular'] == 0. for packet in self.packets()))

    def test_emergency_stop_overrides_in_progress_rotation(self):
        self.initialize()
        self.tick(.1)
        self.app.command('/api/emergency-stop', {})
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertEqual(self.packets()[-1]['mode'], 'estop')
        self.assertEqual(self.packets()[-1]['angular'], 0.)

    def test_terminal_result_preserves_paused_route_and_sends_only_hold(self):
        start_mid_mission(self.app)
        before = mission_bookkeeping(self.app)
        self.initialize()
        goal_count = len(self.goals.messages)
        self.app.localization.fail('test no convergence', self.now)
        begin = len(self.controls.messages)
        for _ in range(5):
            self.tick(.1)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)
        self.assertEqual(len(self.goals.messages), goal_count)
        self.assertTrue(all(packet['mode'] == 'hold' for packet in self.packets()[begin:]))

    def test_success_leaves_the_saved_mission_paused_and_never_sends_auto(self):
        start_mid_mission(self.app)
        before = mission_bookkeeping(self.app)
        self.initialize()
        goal_count = len(self.goals.messages)
        yaw = 0.
        for _ in range(90):
            self.now += .1
            self.refresh()
            if self.app.localization.state == 'ROTATING':
                yaw += .02
            self.app.localization.on_odom(yaw, self.now)
            self.app.localization.on_estimate(.4, .4, yaw, .1, .1, now=self.now,
                                               quality_ok=True)
            self.node.tick()
            if self.app.localization.succeeded:
                break
        self.assertEqual(self.app.localization.state, 'SUCCEEDED')
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)
        self.assertEqual(len(self.goals.messages), goal_count)
        self.assertFalse(any(packet['mode'] == 'auto' for packet in self.packets()))
        self.assertEqual(self.packets()[-1]['mode'], 'hold')
        self.assertFalse(self.app.localization_invalidated)

    def test_joystick_cancels_rotation_and_must_acquire_its_own_acknowledged_lease(self):
        start_mid_mission(self.app)
        before = mission_bookkeeping(self.app)
        self.initialize()
        self.tick(.1)
        press = {'control_id': str(uuid4()), 'seq': 1, 'active': True,
                 'linear': .08, 'angular': -.2}
        self.app.manual_command('operator', press)
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertEqual(self.app.manual.state, 'ACQUIRING')
        self.assertEqual(self.packets()[-1]['mode'], 'hold')
        self.assertFalse(self.app.manual.resume_mission)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(mission_bookkeeping(self.app), before)

    def test_global_reset_service_exception_stops_without_guessing_pose(self):
        self.acquire()
        self.global_reset.future.set_exception(RuntimeError('AMCL unavailable'))
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'FAILED')
        self.assertIsNone(self.app.localization.estimate)
        self.assertEqual(self.packets()[-1]['mode'], 'hold')
        self.assertFalse(self.app.localization_service_pending)

    def home_during_acquiring(self, endpoint):
        start_mid_mission(self.app)
        self.app.command('/api/localization/start', {})
        self.tick()
        self.assertEqual(self.app.localization.state, 'ACQUIRING')
        self.assertEqual(len(self.global_reset.requests), 0)
        self.app.command(endpoint, {})
        home = deepcopy(self.app.delivery.homepoint)
        self.tick(.1)
        self.assertEqual(self.app.localization.state, 'CANCELED')
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        # Terminal localization cleanup must not discard a new user-directed
        # home goal queued after canceling this not-yet-initialized search.
        self.assertEqual(self.node.goal_waiting, home)
        self.assertEqual(len(self.global_reset.requests), 0)
        self.cancel.acknowledge(success=True)
        self.tick(.1)
        self.tick(.9)
        self.assertEqual(len(self.goals.messages), 1)
        self.assertAlmostEqual(self.goals.messages[0].pose.position.x, home['x'])
        self.assertAlmostEqual(self.goals.messages[0].pose.position.y, home['y'])
        self.assertTrue(self.app.mission.returning_home)

    def test_home_before_global_reset_retains_its_new_queued_goal(self):
        self.home_during_acquiring('/api/mission/home')

    def test_cancel_before_global_reset_retains_its_new_home_goal(self):
        self.home_during_acquiring('/api/mission/cancel')

    def test_low_covariance_wrong_pose_with_75_percent_match_never_converges(self):
        self.initialize()
        self.node.localization_epoch_ns = 1000
        self.node.source_fresh = lambda *_args, **_kwargs: True
        self.node.localization_scan = SimpleNamespace()
        self.node.localization_laser_pose = (0., 0., 0.)
        self.app.localization_distance = np.ones((10, 10))
        yaw = 0.
        with patch('smart_vision_amr.web_mission_server.scan_points', return_value=np.ones((20, 2))), \
                patch('smart_vision_amr.web_mission_server.scan_map_agreement', return_value=.75):
            for sequence in range(120):
                self.now += .1
                self.refresh()
                yaw += .02
                self.app.localization.on_odom(yaw, self.now)
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp.nanosec = 1001+sequence
                message.pose.pose.position.y = 1.32  # Previously observed false convergence.
                message.pose.pose.orientation.z = math.sin(yaw/2)
                message.pose.pose.orientation.w = math.cos(yaw/2)
                message.pose.covariance[0] = message.pose.covariance[7] = .0001
                message.pose.covariance[35] = .0001
                self.node.on_amcl(message)
                self.node.tick()
        self.assertGreater(self.app.localization.rotation, math.pi/2)
        self.assertEqual(self.app.localization.state, 'ROTATING')
        self.assertEqual(self.app.localization.snapshot()['good_estimates'], 0)
        self.assertFalse(self.app.localization.snapshot()['quality_ok'])
        self.assertEqual(self.app.localization_match, .75)
        self.assertFalse(self.app.localization.succeeded)

    def test_scan_match_must_reach_ninety_percent_before_counting_confidence(self):
        self.initialize()
        self.node.localization_epoch_ns = 1000
        self.node.source_fresh = lambda *_args, **_kwargs: True
        self.node.localization_scan = SimpleNamespace()
        self.node.localization_laser_pose = (0., 0., 0.)
        self.app.localization_distance = np.ones((10, 10))
        with patch('smart_vision_amr.web_mission_server.scan_points', return_value=np.ones((20, 2))), \
                patch('smart_vision_amr.web_mission_server.scan_map_agreement', side_effect=[.8999, .90]):
            for index in range(2):
                self.now += .1
                self.refresh()
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp.nanosec = 1001+index
                message.pose.pose.orientation.w = 1.
                message.pose.covariance[0] = message.pose.covariance[7] = .01
                message.pose.covariance[35] = .01
                self.node.on_amcl(message)
                self.assertEqual(self.app.localization.snapshot()['good_estimates'], index)

    def prepare_amcl_estimate(self, stamp=1001):
        self.node.localization_epoch_ns = 1000
        self.node.source_fresh = lambda *_args, **_kwargs: True
        self.node.localization_scan = SimpleNamespace()
        self.node.localization_laser_pose = (0., 0., 0.)
        self.app.localization_distance = np.ones((10, 10))
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp.nanosec = stamp
        message.pose.pose.orientation.w = 1.
        message.pose.covariance[0] = message.pose.covariance[7] = .04
        message.pose.covariance[35] = .01
        return message

    def test_correlated_xy_covariance_uses_largest_principal_uncertainty(self):
        self.initialize()
        message = self.prepare_amcl_estimate()
        message.pose.covariance[1] = message.pose.covariance[6] = .035
        self.now += .1
        with patch('smart_vision_amr.web_mission_server.scan_points', return_value=np.ones((20, 2))), \
                patch('smart_vision_amr.web_mission_server.scan_map_agreement', return_value=1.):
            self.node.on_amcl(message)
        # Each individual axis has std .20m, but correlated uncertainty along
        # the principal direction is sqrt(.075) > the .25m success threshold.
        self.assertAlmostEqual(self.app.localization.estimate['position_std'], math.sqrt(.075))
        self.assertGreater(self.app.localization.estimate['position_std'], .25)
        self.assertEqual(self.app.localization.snapshot()['good_estimates'], 0)

    def test_non_positive_semidefinite_xy_covariance_is_rejected(self):
        self.initialize()
        message = self.prepare_amcl_estimate()
        message.pose.covariance[1] = message.pose.covariance[6] = .1
        with patch('smart_vision_amr.web_mission_server.scan_map_agreement') as match:
            self.node.on_amcl(message)
        self.assertIsNone(self.app.localization.estimate)
        self.assertEqual(self.app.localization.snapshot()['good_estimates'], 0)
        match.assert_not_called()

    def test_nonfinite_xy_cross_covariance_is_rejected(self):
        self.initialize()
        message = self.prepare_amcl_estimate()
        message.pose.covariance[1] = math.nan
        with patch('smart_vision_amr.web_mission_server.scan_map_agreement') as match:
            self.node.on_amcl(message)
        self.assertIsNone(self.app.localization.estimate)
        self.assertEqual(self.app.localization.snapshot()['good_estimates'], 0)
        match.assert_not_called()

    def test_operator_initial_pose_fallback_uses_sixty_percent_not_global_threshold(self):
        self.app.localization_invalidated = True
        self.node.manual_initial_pose_ns = 1000
        with patch('smart_vision_amr.web_mission_server.scan_points', return_value=np.ones((20, 2))), \
                patch('smart_vision_amr.web_mission_server.scan_map_agreement', side_effect=[.5999, .60]):
            self.node.on_amcl(self.prepare_amcl_estimate(1001))
            self.assertTrue(self.app.localization_invalidated)
            self.assertEqual(self.node.manual_initial_pose_ns, 1000)
            self.node.on_amcl(self.prepare_amcl_estimate(1002))
            self.assertFalse(self.app.localization_invalidated)
            self.assertIsNone(self.node.manual_initial_pose_ns)
        self.assertEqual(self.app.localization.state, 'IDLE')
        self.assertFalse(self.app.localization.succeeded)

    def test_pre_reset_and_duplicate_amcl_packets_cannot_fake_confidence(self):
        self.initialize()
        self.node.localization_epoch_ns = 1000
        self.node.source_fresh = lambda *_args, **_kwargs: True
        self.node.localization_scan = SimpleNamespace()
        self.node.localization_laser_pose = (0., 0., 0.)
        self.app.localization_distance = np.ones((10, 10))

        def estimate(stamp):
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            message.header.stamp.nanosec = stamp
            message.pose.pose.orientation.w = 1.
            message.pose.covariance[0] = message.pose.covariance[7] = .01
            message.pose.covariance[35] = .01
            return message

        with patch('smart_vision_amr.web_mission_server.scan_points', return_value=np.ones((20, 2))), \
                patch('smart_vision_amr.web_mission_server.scan_map_agreement', return_value=1.):
            self.node.on_amcl(estimate(999))
            self.assertEqual(self.app.localization.snapshot()['good_estimates'], 0)
            self.now += .1
            self.node.on_amcl(estimate(1001))
            self.assertEqual(self.app.localization.snapshot()['good_estimates'], 1)
            self.now += .1
            for _ in range(20):
                self.node.on_amcl(estimate(1001))
            self.node.on_amcl(estimate(1000))
            self.assertEqual(self.app.localization.snapshot()['good_estimates'], 1)
            self.assertFalse(self.app.localization.succeeded)


class LocalizationGeometryTests(unittest.TestCase):
    def scan(self, values=None):
        return SimpleNamespace(ranges=values or [1.]*360, angle_min=-math.pi,
            angle_increment=2*math.pi/359, range_min=.12, range_max=3.5)

    def test_partial_view_cannot_authorize_a_stationary_rotation(self):
        message = self.scan()
        message.angle_increment = math.pi/359
        self.assertEqual(rotation_clearance(message, (0., 0., 0.)), (False, None))

    def test_sensor_offset_is_included_in_body_sweep_clearance(self):
        message = self.scan()
        message.ranges[0] = .4  # Backwards from sensor positioned 15cm forward.
        safe, distance = rotation_clearance(message, (.15, 0., 0.))
        self.assertFalse(safe)
        self.assertAlmostEqual(distance, .25)

    def test_zero_nan_and_all_infinite_scans_never_authorize_motion(self):
        for values in ([0.]*360, [math.nan]*360, [math.inf]*360):
            self.assertFalse(rotation_clearance(self.scan(values), (0., 0., 0.))[0])

    def test_scan_endpoint_transform_respects_laser_yaw(self):
        message = self.scan([1.])
        message.angle_min = 0.
        points = scan_points(message, (.1, .2, math.pi/2))
        np.testing.assert_allclose(points, [[.1, 1.2]], atol=1e-10)

    def test_outside_map_returns_do_not_count_as_matching_walls(self):
        metadata = {'width': 10, 'height': 10, 'resolution': .1,
                    'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        points = np.tile([5., 5.], (20, 1))
        self.assertEqual(scan_map_agreement(points, {'x': 0., 'y': 0., 'yaw': 0.},
                         metadata, np.zeros((10, 10))), 0.)

    def test_unknown_cells_adjacent_to_a_wall_never_count_as_matched_returns(self):
        metadata = {'width': 10, 'height': 10, 'resolution': .1,
                    'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        data = np.zeros((10, 10), dtype=np.int16)
        data[:, 5] = 100
        data[:, 4] = -1  # Only one grid cell from the wall, inside .15m tolerance.
        distance = occupied_distance(data, metadata['resolution'])
        self.assertTrue(np.all(np.isinf(distance[:, 4])))
        # Observe the wall from its mapped, free right-hand side. This isolates
        # endpoint handling from the separate line-of-sight rejection rule.
        pose = {'x': .8, 'y': .55, 'yaw': 0.}
        unknown_hits = np.tile([-.35, 0.], (20, 1))
        wall_hits = np.tile([-.25, 0.], (20, 1))
        self.assertEqual(scan_map_agreement(unknown_hits, pose, metadata, distance), 0.)
        self.assertEqual(scan_map_agreement(wall_hits, pose, metadata, distance), 1.)

    def test_endpoint_alias_behind_an_intervening_wall_is_not_a_scan_match(self):
        metadata = {'width': 30, 'height': 10, 'resolution': .1,
                    'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        data = np.zeros((10, 30), dtype=np.int16)
        data[:, 10] = data[:, 20] = 100
        distance = occupied_distance(data, metadata['resolution'])
        pose = {'x': .2, 'y': .5, 'yaw': 0.}
        first_wall = np.tile([.85, .05], (20, 1))
        hidden_wall = np.tile([1.85, .05], (20, 1))
        self.assertEqual(scan_map_agreement(first_wall, pose, metadata, distance), 1.)
        self.assertEqual(scan_map_agreement(hidden_wall, pose, metadata, distance), 0.)

    def test_wall_endpoint_across_unknown_space_is_not_a_scan_match(self):
        metadata = {'width': 30, 'height': 10, 'resolution': .1,
                    'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
        data = np.zeros((10, 30), dtype=np.int16)
        data[:, 10] = -1
        data[:, 20] = 100
        distance = occupied_distance(data, metadata['resolution'])
        pose = {'x': .2, 'y': .5, 'yaw': 0.}
        hidden_wall = np.tile([1.85, .05], (20, 1))
        self.assertEqual(scan_map_agreement(hidden_wall, pose, metadata, distance), 0.)


if __name__ == '__main__':
    unittest.main()
