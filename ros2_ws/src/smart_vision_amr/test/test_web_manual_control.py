"""Manual override contracts without a live vehicle or command publisher.

HTTP, mission and lease tests are pure Python. Optional bridge tests use their
own ROS context on domain 211 and replace all outputs before running a tick;
none of these tests publishes a vehicle velocity or navigation goal to DDS.
"""
from copy import deepcopy
from concurrent.futures import Future
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from uuid import uuid4

from smart_vision_amr.manual_control import ManualOverride
from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, MissionHTTPServer,
    MissionHTTPRequestHandler, create_ros_bridge,
)

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformException
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class ManualHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.app = MissionApplication(
            DEFAULT_MARKERS, 211, False, self.directory/'settings.json',
            access_mode='domain')
        self.server = MissionHTTPServer(('127.0.0.1', 0), self.app, self.directory)
        self.thread = threading.Thread(
            target=lambda: self.server.serve_forever(poll_interval=.01), daemon=True)
        self.thread.start()
        self.remote = patch.object(
            MissionHTTPRequestHandler, 'local', property(lambda _: False))
        self.remote.start()
        self.ready()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.remote.stop()
        self.temporary.cleanup()

    def ready(self):
        now = time.monotonic()
        self.app.connected = self.app.goal_available = True
        self.app.planner_available = self.app.cancel_available = True
        self.app.manual_mux_at = now
        self.app.manual_mux_status = {'mode': 'AUTO'}
        self.app.navigation_command_publishers = 0
        self.app.map = {'width': 1}
        self.app.pose = {'x': -.7, 'y': -1.2, 'yaw': .1}
        self.app.pose_at = self.app.scan_at = self.app.odom_at = self.app.camera_at = now

    def request(self, method, endpoint, payload=None, cookie=None, headers=None):
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.server.server_port, timeout=3)
        request_headers = {'Content-Type': 'application/json'}
        if cookie:
            request_headers['Cookie'] = cookie
        request_headers.update(headers or {})
        connection.request(method, endpoint,
                           None if payload is None else json.dumps(payload),
                           request_headers)
        response = connection.getresponse()
        body, status = response.read(), response.status
        returned_headers = dict(response.getheaders())
        connection.close()
        try:
            body = json.loads(body)
        except ValueError:
            pass
        return status, body, returned_headers

    def operator_cookie(self, address='operator'):
        token = self.app.pair(address, '', domain_id=211)
        return 'amr_session='+token

    def payload(self, **changes):
        value = {'control_id': str(uuid4()), 'seq': 1, 'active': True,
                 'linear': .08, 'angular': -.3}
        value.update(changes)
        return value

    def test_manual_endpoint_requires_operator_authentication(self):
        payload = self.payload()
        status, body, _ = self.request('POST', '/api/manual', payload)
        self.assertEqual(status, 401, body)
        manager = self.app.pair_inventory('manager', 7, '', domain_id=211)
        status, body, _ = self.request(
            'POST', '/api/manual', payload, 'amr_inventory_session='+manager)
        self.assertEqual(status, 401, body)
        self.assertFalse(self.app.manual.in_progress)
        self.assertEqual(self.app.mission.commands, [])

    def test_expired_operator_cannot_acquire_control(self):
        cookie = self.operator_cookie()
        token = cookie.split('=', 1)[1]
        self.app.sessions[token] = time.monotonic()-1
        status, body, _ = self.request('POST', '/api/manual', self.payload(), cookie)
        self.assertEqual(status, 401, body)
        self.assertFalse(self.app.manual.in_progress)

    def test_cross_origin_cannot_issue_manual_motion(self):
        status, body, _ = self.request(
            'POST', '/api/manual', self.payload(), self.operator_cookie(),
            {'Origin': 'https://untrusted.example'})
        self.assertEqual(status, 403, body)
        self.assertFalse(self.app.manual.in_progress)

    def test_missing_vehicle_readiness_rejects_acquisition(self):
        cookie = self.operator_cookie()
        for field, absent in (('connected', False), ('scan_at', 0),
                              ('odom_at', 0), ('manual_mux_at', 0)):
            with self.subTest(field=field):
                self.ready()
                setattr(self.app, field, absent)
                status, body, _ = self.request(
                    'POST', '/api/manual', self.payload(), cookie)
                self.assertEqual(status, 400, body)
                self.assertFalse(self.app.manual.in_progress)

    def test_other_operator_session_cannot_update_or_release_owner(self):
        owner = self.operator_cookie('first')
        other = self.operator_cookie('second')
        payload = self.payload()
        status, body, _ = self.request('POST', '/api/manual', payload, owner)
        self.assertEqual(status, 200, body)
        held_owner = self.app.manual.owner
        held_id = self.app.manual.control_id
        for value in (dict(payload, seq=2, angular=.2),
                      dict(payload, seq=3, active=False, linear=0, angular=0)):
            with self.subTest(active=value['active']):
                status, body, _ = self.request('POST', '/api/manual', value, other)
                # Foreign release may be an idempotent no-op, but must never
                # terminate the current operator's hold.
                self.assertIn(status, (200, 400, 403), body)
                self.assertEqual(self.app.manual.owner, held_owner)
                self.assertEqual(self.app.manual.control_id, held_id)
                self.assertTrue(self.app.manual.active)
        self.assertTrue(self.app.manual.in_progress)

    def test_estop_blocks_manual_acquisition_even_when_all_sensors_are_ready(self):
        self.app.mission.emergency_stop()
        status, body, _ = self.request(
            'POST', '/api/manual', self.payload(), self.operator_cookie())
        self.assertEqual(status, 400, body)
        self.assertEqual(self.app.mission.state, 'ESTOP')
        self.assertFalse(self.app.manual.in_progress)

    def test_invalid_manual_payloads_cannot_create_a_motion_lease(self):
        cookie = self.operator_cookie()
        invalid = [
            {'control_id': ''}, {'control_id': 17},
            {'seq': True}, {'seq': -1}, {'seq': '1'}, {'seq': 1.5},
            {'active': 1}, {'active': 'true'},
            {'linear': True}, {'linear': '0.1'}, {'linear': float('nan')},
            {'angular': None}, {'angular': float('inf')},
        ]
        for changes in invalid:
            with self.subTest(changes=changes):
                status, body, _ = self.request(
                    'POST', '/api/manual', self.payload(**changes), cookie)
                self.assertEqual(status, 400, body)
                self.assertFalse(self.app.manual.in_progress)

    def test_release_before_acquire_tombstones_late_inflight_press(self):
        cookie = self.operator_cookie()
        press = self.payload()
        release = dict(press, seq=2, active=False, linear=0, angular=0)
        status, body, _ = self.request('POST', '/api/manual', release, cookie)
        self.assertEqual(status, 200, body)
        status, body, _ = self.request('POST', '/api/manual', press, cookie)
        self.assertIn(status, (200, 400), body)
        self.assertFalse(self.app.manual.active,
                         'a delayed pointer-down request must not restart a released gesture')

    def test_late_update_after_owner_release_cannot_reacquire_same_gesture(self):
        cookie = self.operator_cookie()
        press = self.payload()
        self.assertEqual(self.request('POST', '/api/manual', press, cookie)[0], 200)
        release = dict(press, seq=3, active=False, linear=0, angular=0)
        self.assertEqual(self.request('POST', '/api/manual', release, cookie)[0], 200)
        status, body, _ = self.request(
            'POST', '/api/manual', dict(press, seq=2, linear=.12), cookie)
        self.assertIn(status, (200, 400), body)
        self.assertFalse(self.app.manual.active)

    def test_operator_status_distinguishes_control_owner_from_another_session(self):
        owner = self.operator_cookie('first')
        observer = self.operator_cookie('second')
        self.assertEqual(self.request('POST', '/api/manual', self.payload(), owner)[0], 200)
        own_state = self.request('GET', '/api/state', cookie=owner)[1]
        other_state = self.request('GET', '/api/state', cookie=observer)[1]
        self.assertTrue(own_state['manual']['owner'])
        self.assertFalse(other_state['manual']['owner'])


class ManualMissionBookkeepingTests(unittest.TestCase):
    """The joystick suspends execution, never route or delivery intent."""
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.app = MissionApplication(
            DEFAULT_MARKERS, 211, False, Path(self.temporary.name)/'settings.json')
        now = time.monotonic()
        self.app.connected = self.app.goal_available = True
        self.app.planner_available = self.app.cancel_available = True
        self.app.manual_mux_at = now
        self.app.manual_mux_status = {'mode': 'AUTO'}
        self.app.navigation_command_publishers = 0
        self.app.map = {'width': 1}
        self.app.pose = {'x': -.7, 'y': -1.2, 'yaw': .1}
        self.app.pose_at = self.app.scan_at = self.app.odom_at = self.app.camera_at = now
        self.press = {'control_id': str(uuid4()), 'seq': 1, 'active': True,
                      'linear': .1, 'angular': .3}

    def tearDown(self):
        self.temporary.cleanup()

    def start_mission(self):
        mission = self.app.mission
        mission.update_route([1, 2, 3])
        mission.start(home={'x': -.7, 'y': -1.2, 'yaw': .1}, auto_return_home=True)
        mission.current_index = 1
        mission.completed_ids = [1]
        mission.pending_route = [3, 1]
        mission.goal_dispatched({'sec': 4, 'nanosec': 7})
        return mission

    def bookkeeping(self):
        mission = self.app.mission
        return deepcopy({key: getattr(mission, key) for key in (
            'route', 'pending_route', 'current_index', 'completed_ids',
            'home', 'returning_home', 'auto_return_home', 'auto_returning_home')})

    def acquire(self):
        return self.app.manual_command('operator-session', self.press)

    def release(self):
        return self.app.manual_command('operator-session', dict(
            self.press, seq=2, active=False, linear=0, angular=0))

    def test_active_mission_is_held_without_resetting_waypoint_progress(self):
        mission = self.start_mission()
        previous = self.bookkeeping()
        self.acquire()
        self.assertEqual(mission.state, 'PAUSED')
        self.assertIsNone(mission.goal_stamp)
        self.assertTrue(self.app.manual.resume_mission)
        self.assertEqual(self.bookkeeping(), previous)
        self.assertTrue(any(kind == 'cancel' for kind, _ in mission.commands))
        self.release()
        self.assertEqual(self.bookkeeping(), previous)

    def test_idle_manual_control_does_not_create_an_autonomous_mission(self):
        self.acquire()
        self.assertEqual(self.app.mission.state, 'IDLE')
        self.assertFalse(self.app.manual.resume_mission)
        self.release()
        self.assertEqual(self.app.mission.state, 'IDLE')
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_previously_paused_mission_does_not_gain_automatic_resume_permission(self):
        mission = self.start_mission()
        mission.pause()
        previous = self.bookkeeping()
        self.acquire()
        self.assertFalse(self.app.manual.resume_mission)
        self.release()
        self.assertEqual(mission.state, 'PAUSED')
        self.assertEqual(self.bookkeeping(), previous)

    def test_old_arrival_and_camera_confirmation_cannot_advance_held_mission(self):
        mission = self.start_mission()
        stamp = mission.goal_stamp
        self.acquire()
        self.assertFalse(mission.on_navigation(
            {'status': 'SUCCEEDED', 'goal_stamp': stamp},
            pose=mission.markers[2], stationary=True))
        for frame in range(5):
            mission.observe(frame, [2], mission.markers[2], True, time.monotonic(), alignment_verified=True)
        self.assertEqual(mission.current_index, 1)
        self.assertEqual(mission.completed_ids, [1])
        self.assertEqual(mission.state, 'PAUSED')

    def test_new_delivery_waypoints_added_during_hold_are_not_rolled_back(self):
        mission = self.start_mission()
        self.acquire()
        mission.append_waypoints([1, 3])
        expected = self.bookkeeping()
        self.release()
        self.assertEqual(mission.route, [1, 2, 3, 1, 3])
        self.assertEqual(self.bookkeeping(), expected)

    def test_explicit_pause_disables_automatic_resume(self):
        self.start_mission()
        self.acquire()
        self.app.command('/api/mission/pause', {})
        self.assertFalse(self.app.manual.resume_mission)
        self.release()
        self.assertEqual(self.app.mission.state, 'PAUSED')

    def test_explicit_estop_overrides_manual_control_and_future_resume(self):
        self.start_mission()
        self.acquire()
        self.app.command('/api/emergency-stop', {})
        self.assertTrue(self.app.mission.estop_latched)
        self.assertFalse(self.app.manual.active)
        self.assertFalse(self.app.manual.resume_mission)
        with self.assertRaises(ValueError):
            self.app.manual_command('operator-session', dict(self.press, seq=2))

    def test_start_resume_restart_cannot_bypass_live_manual_override(self):
        self.start_mission()
        self.acquire()
        for endpoint in ('/api/mission/start', '/api/mission/resume', '/api/mission/restart'):
            with self.subTest(endpoint=endpoint):
                with self.assertRaises(ValueError):
                    self.app.command(endpoint, {})
                self.assertEqual(self.app.mission.state, 'PAUSED')
                self.assertTrue(self.app.manual.active)

    def test_user_home_request_replaces_resume_with_home_goal(self):
        mission = self.start_mission()
        self.acquire()
        self.app.command('/api/mission/home', {})
        self.assertFalse(self.app.manual.in_progress)
        self.assertFalse(self.app.manual.resume_mission)
        self.assertTrue(mission.returning_home)
        self.assertEqual(mission.state, 'RETURNING_HOME')
        self.assertEqual(mission.commands[-1], ('goal', self.app.delivery.homepoint))

    def test_domain_change_cannot_move_live_joystick_lease_to_another_robot(self):
        self.acquire()
        with self.assertRaises(ValueError):
            self.app.command('/api/connection', {'domain_id': 212})
        self.assertIsNone(self.app.pending_connection)
        self.assertEqual(self.app.domain_id, 211)

    def test_idle_teleop_needs_scan_and_odom_but_not_map_camera_or_localization(self):
        self.app.map = self.app.pose = None
        self.app.camera_at = self.app.pose_at = 0
        self.app.goal_available = self.app.planner_available = False
        self.app.cancel_available = False
        self.app.navigation_command_publishers = 0
        self.acquire()
        self.assertTrue(self.app.manual.active)
        self.assertEqual(self.app.mission.state, 'IDLE')

    def test_manual_rejects_unmanaged_cmd_vel_publishers(self):
        self.app.unmanaged_velocity_publishers = ['unmanaged_teleop']
        with self.assertRaises(ValueError):
            self.acquire()
        self.assertFalse(self.app.manual.in_progress)

    def test_existing_navigation_publishers_require_a_cancel_service(self):
        self.app.navigation_command_publishers = 1
        self.app.cancel_available = False
        with self.assertRaises(ValueError):
            self.acquire()
        self.assertFalse(self.app.manual.in_progress)

    def test_estop_mux_state_cannot_be_bypassed_by_an_idle_web_mission(self):
        self.app.manual_mux_status = {'mode': 'ESTOP'}
        with self.assertRaises(ValueError):
            self.acquire()
        self.assertFalse(self.app.manual.in_progress)

    def test_reconnection_cannot_accept_a_manual_lease_from_previous_domain(self):
        self.app.pending_connection = (212, False)
        with self.assertRaises(ValueError):
            self.acquire()
        self.assertFalse(self.app.manual.in_progress)


class ManualLeaseTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.control = ManualOverride(clock=lambda: self.now)
        self.payload = {'control_id': str(uuid4()), 'seq': 1, 'active': True,
                        'linear': .08, 'angular': -.3}

    def test_acquisition_exposes_zero_until_cancel_and_mux_acknowledged(self):
        self.assertEqual(self.control.request('owner', self.payload), 'acquire')
        before = self.control.snapshot('owner', available=True)
        self.assertEqual(before['state'], 'ACQUIRING')
        self.assertEqual(before['linear'], 0)
        self.assertEqual(before['angular'], 0)
        self.control.mark_acquired()
        after = self.control.snapshot('owner', available=True)
        self.assertEqual(after['state'], 'ACTIVE')
        self.assertEqual(after['linear'], .08)
        self.assertEqual(after['angular'], -.3)

    def test_commands_are_limited_in_both_directions(self):
        for linear, angular in ((100, -100), (-100, 100)):
            with self.subTest(linear=linear):
                control = ManualOverride(clock=lambda: self.now)
                control.request('owner', dict(self.payload, linear=linear, angular=angular))
                control.mark_acquired()
                state = control.snapshot('owner')
                self.assertEqual(abs(state['linear']), .12)
                self.assertEqual(abs(state['angular']), .60)
                self.assertEqual(state['linear'] > 0, linear > 0)
                self.assertEqual(state['angular'] > 0, angular > 0)

    def test_out_of_order_updates_do_not_change_speed_or_extend_deadline(self):
        self.control.request('owner', dict(self.payload, seq=5))
        deadline = self.control.deadline
        self.now += .2
        for seq in (1, 4, 5):
            with self.subTest(seq=seq):
                self.assertEqual(self.control.request('owner', dict(
                    self.payload, seq=seq, linear=-.12)), 'ignored')
                self.assertEqual(self.control.linear, .08)
                self.assertEqual(self.control.deadline, deadline)

    def test_release_is_zero_and_late_higher_sequence_cannot_reopen_a_gesture(self):
        self.control.request('owner', self.payload)
        self.control.mark_acquired()
        self.control.request('owner', dict(self.payload, seq=2, active=False))
        self.assertEqual(self.control.state, 'RELEASING')
        self.assertEqual(self.control.linear, 0)
        self.assertEqual(self.control.angular, 0)
        self.assertEqual(self.control.request('owner', dict(self.payload, seq=99)), 'ignored')
        self.assertFalse(self.control.active)

    def test_deadman_expires_at_half_second_and_cannot_resume_old_gesture(self):
        self.control.request('owner', self.payload)
        self.control.mark_acquired()
        self.now += .499
        self.assertFalse(self.control.expire())
        self.now += .002
        self.assertTrue(self.control.expire())
        self.assertFalse(self.control.active)
        self.assertEqual((self.control.linear, self.control.angular), (0, 0))
        self.assertEqual(self.control.request('owner', dict(self.payload, seq=2)), 'ignored')


    def test_late_heartbeat_cannot_renew_expired_lease_before_next_timer_tick(self):
        self.control.request('owner', self.payload)
        self.control.mark_acquired()
        self.now += .51
        self.assertEqual(self.control.request('owner', dict(self.payload, seq=2)), 'ignored')
        self.assertFalse(self.control.active)
        self.assertEqual((self.control.linear, self.control.angular), (0, 0))

    def test_same_operator_new_gesture_invalidates_delayed_old_gesture(self):
        self.control.request('owner', self.payload)
        new = dict(self.payload, control_id=str(uuid4()), linear=-.08)
        self.control.request('owner', new)
        self.assertEqual(self.control.request('owner', dict(self.payload, seq=3)), 'ignored')
        self.assertEqual(self.control.control_id, new['control_id'])
        self.assertEqual(self.control.linear, -.08)

    def test_different_operator_cannot_steal_acquiring_or_active_control(self):
        self.control.request('owner', self.payload)
        for state in ('ACQUIRING', 'ACTIVE'):
            with self.subTest(state=state):
                self.control.state = state
                with self.assertRaises(ValueError):
                    self.control.request('another', dict(self.payload, control_id=str(uuid4())))
                self.assertEqual(self.control.owner, 'owner')
                self.assertEqual(self.control.state, state)

    def test_released_control_does_not_lock_out_another_operator_while_replanning(self):
        self.control.request('owner', self.payload)
        self.control.resume_mission = True
        self.control.release()
        result = self.control.request('another', dict(self.payload, control_id=str(uuid4())))
        self.assertEqual(result, 'acquire')
        self.assertEqual(self.control.owner, 'another')
        self.assertTrue(self.control.resume_mission)

    def test_invalid_update_keeps_current_command_and_deadline_unchanged(self):
        self.control.request('owner', self.payload)
        deadline = self.control.deadline
        self.now += .1
        with self.assertRaises(ValueError):
            self.control.request('owner', dict(self.payload, seq=2, angular=float('nan')))
        self.assertEqual(self.control.angular, -.3)
        self.assertEqual(self.control.deadline, deadline)

    def test_snapshot_identifies_owner_without_leaking_session_or_control_id(self):
        self.control.request('session-secret', self.payload)
        self.assertTrue(self.control.snapshot('session-secret')['owner'])
        self.assertFalse(self.control.snapshot('someone-else')['owner'])
        self.assertFalse(self.control.snapshot()['owner'])
        encoded = json.dumps(self.control.snapshot('someone-else'))
        self.assertNotIn('session-secret', encoded)
        self.assertNotIn(self.payload['control_id'], encoded)

    def test_finish_clears_speeds_and_resume_intent_but_preserves_replay_barrier(self):
        self.control.request('owner', self.payload)
        self.control.resume_mission = True
        self.control.finish()
        self.assertEqual(self.control.state, 'IDLE')
        self.assertFalse(self.control.resume_mission)
        self.assertIsNone(self.control.owner)
        self.assertEqual((self.control.linear, self.control.angular), (0, 0))
        self.assertEqual(self.control.request('owner', dict(self.payload, seq=2)), 'ignored')


class _RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


class _CancelClient:
    def __init__(self):
        self.future = None
        self.available = True
        self.requests = 0

    def service_is_ready(self):
        return self.available

    def call_async(self, _request):
        self.future = Future()
        self.requests += 1
        return self.future

    def acknowledge(self, success=True):
        result = Trigger.Response()
        result.success = success
        self.future.set_result(result)


@unittest.skipUnless(HAS_ROS, 'source ROS Humble to run the isolated bridge adapter tests')
class ManualBridgeTests(unittest.TestCase):
    """Real bridge callbacks, fake outputs, explicit context on domain 211 only."""
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.app = MissionApplication(
            DEFAULT_MARKERS, 211, False, Path(self.temporary.name)/'settings.json')
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=211,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = create_ros_bridge(self.app, SimpleNamespace(
            camera_topic='/isolated_manual_test_camera', base_frame='isolated_manual_test_base'),
            self.context)
        # Replace every bridge output before any timer is spun or called.
        self.goals = self.node.goal_pub = _RecordingPublisher()
        self.controls = self.node.motion_pub = _RecordingPublisher()
        self.cancel = self.node.cancel_client = _CancelClient()
        self.node.count_publishers = lambda topic: 1 if topic in {
            '/smart_vision_amr/planner_status', '/smart_vision_amr/navigation_cmd_vel'} else 0
        self.node.get_publishers_info_by_topic = lambda _: []
        self.node.get_subscriptions_info_by_topic = lambda topic: [
            SimpleNamespace(node_name=name)
            for name in ('astar_planner', 'path_follower')
        ] if topic == '/smart_vision_amr/goal' else []

        def no_transform(*_args, **_kwargs):
            raise TransformException('No live transforms in this isolated test')
        self.node.tf_buffer.lookup_transform = no_transform
        self.now = 100.
        self.monotonic = patch('smart_vision_amr.web_mission_server.time.monotonic',
                               side_effect=lambda: self.now)
        self.monotonic.start()
        self.app.manual.clock = lambda: self.now
        self.sequence = 0
        self.last_auto_ack_seq = 0
        self.control_id = str(uuid4())
        self.refresh()

    def tearDown(self):
        self.monotonic.stop()
        self.node.destroy_node()
        self.context.shutdown()
        self.temporary.cleanup()

    def refresh(self):
        self.app.map = {'width': 1}
        self.app.pose = {'x': -.7, 'y': -1.2, 'yaw': .1}
        self.app.pose_at = self.app.scan_at = self.app.odom_at = self.app.camera_at = self.now
        self.app.manual_mux_at = self.now
        self.app.manual_mux_status = {
            'mode': 'hold', 'source_id': self.node.motion_source,
            'available': True, 'autonomous_allowed': True,
            'estop_latched': False,
        }

    def tick(self, delta=0, refresh=True):
        self.now += delta
        if refresh:
            self.refresh()
        self.node.tick()
        # Model the robot-side mux accepting an auto lease only after the
        # bridge has sent its zero-speed handover packet.
        for message in reversed(self.controls.messages):
            packet = json.loads(message.data)
            if packet['mode'] != 'auto' or packet['seq'] <= self.last_auto_ack_seq:
                continue
            self.last_auto_ack_seq = packet['seq']
            self.app.manual_mux_at = self.now
            self.app.manual_mux_status = {
                'mode': 'auto', 'source_id': packet['source_id'],
                'control_seq': packet['seq'], 'available': True,
                'autonomous_allowed': True, 'estop_latched': False,
                'last_control': {'source_id': packet['source_id'],
                                 'seq': packet['seq'], 'mode': 'auto',
                                 'origin': 'topic', 'accepted': True},
            }
            self.node.tick()
            break

    def command(self, active=True):
        self.sequence += 1
        return self.app.manual_command('test-operator', {
            'control_id': self.control_id, 'seq': self.sequence,
            'active': active, 'linear': .08 if active else 0,
            'angular': .3 if active else 0})

    def heartbeat(self, delta):
        self.now += delta
        self.refresh()
        self.command()
        self.node.tick()

    def modes(self, since=0):
        return [json.loads(message.data) for message in self.controls.messages[since:]]

    def start_mission(self):
        mission = self.app.mission
        mission.update_route([1, 2, 3])
        mission.start(home={'x': -.7, 'y': -1.2, 'yaw': .1}, auto_return_home=True)
        mission.drain_commands()
        mission.current_index = 1
        mission.completed_ids = [1]
        mission._send_current_goal()
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)

    def acquire(self):
        self.command()
        self.tick()
        self.cancel.acknowledge()
        self.tick(.1)
        self.heartbeat(.3)
        self.heartbeat(.3)
        self.heartbeat(.2)
        self.assertEqual(self.app.manual.state, 'ACTIVE')

    def navigation(self, status, goal):
        stamp = goal.header.stamp
        self.node.on_navigation(String(data=json.dumps({
            'status': status, 'goal_stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec}})))

    def test_held_manual_cannot_publish_nonzero_before_cancel_acknowledgement(self):
        self.start_mission()
        first = len(self.controls.messages)
        self.command()
        self.tick()
        for _ in range(4):
            self.heartbeat(.3)
        self.assertEqual(self.app.manual.state, 'ACQUIRING')
        self.assertTrue(self.node.cancel_required)
        self.assertTrue(all(message['mode'] == 'hold' for message in self.modes(first)))

    def test_rejected_cancel_ack_cannot_release_manual_motion(self):
        self.start_mission()
        first = len(self.controls.messages)
        self.command()
        self.tick()
        self.cancel.acknowledge(False)
        self.tick(.1)
        for _ in range(3):
            self.heartbeat(.3)
        self.assertEqual(self.app.manual.state, 'ACQUIRING')
        self.assertTrue(self.node.cancel_required)
        self.assertNotIn('manual', [message['mode'] for message in self.modes(first)])

    def test_release_keeps_goal_and_progress_waits_for_new_matching_moving_status(self):
        self.start_mission()
        old_goal = self.goals.messages[-1]
        previous = deepcopy((self.app.mission.route, self.app.mission.current_index,
                             self.app.mission.completed_ids, self.app.mission.home))
        self.acquire()
        self.assertEqual(self.modes()[-1]['mode'], 'manual')
        first = len(self.controls.messages)
        self.command(False)
        self.tick(.05)
        self.assertEqual(self.app.manual.state, 'RELEASING')
        self.assertEqual(len(self.goals.messages), 1)
        self.cancel.acknowledge()
        self.tick(.1)
        self.tick(.8)
        self.assertEqual(self.app.manual.state, 'WAIT_REPLAN')
        self.assertEqual(len(self.goals.messages), 2)
        new_goal = self.goals.messages[-1]
        self.assertEqual(new_goal.pose, old_goal.pose)
        self.assertNotEqual(new_goal.header.stamp, old_goal.header.stamp)
        self.assertEqual((self.app.mission.route, self.app.mission.current_index,
                          self.app.mission.completed_ids, self.app.mission.home), previous)
        self.navigation('MOVING', old_goal)
        self.assertEqual(self.app.manual.state, 'WAIT_REPLAN')
        self.navigation('PLANNED', new_goal)
        self.assertEqual(self.app.manual.state, 'WAIT_REPLAN')
        self.assertIn('auto', [message['mode'] for message in self.modes(first)])
        self.navigation('MOVING', new_goal)
        self.assertEqual(self.app.manual.state, 'IDLE')
        self.assertEqual(self.modes()[-1]['mode'], 'auto')

    def test_deadman_loss_stops_and_reconnects_same_mission_after_acknowledgement(self):
        self.start_mission()
        old_goal = self.goals.messages[-1]
        self.acquire()
        self.tick(.51)
        self.assertEqual(self.app.manual.state, 'RELEASING')
        self.assertEqual(self.modes()[-1]['mode'], 'hold')
        self.cancel.acknowledge()
        self.tick(.1)
        self.tick(.8)
        self.assertEqual(self.app.manual.state, 'WAIT_REPLAN')
        self.assertEqual(self.goals.messages[-1].pose, old_goal.pose)
        self.assertEqual(self.app.mission.current_index, 1)
        self.assertEqual(self.app.mission.completed_ids, [1])

    def test_manual_release_replans_with_camera_out_but_live_navigation_sensors(self):
        self.start_mission()
        old_goal = self.goals.messages[-1]
        self.acquire()
        self.tick(.51)
        self.cancel.acknowledge()
        self.tick(.1)
        self.app.camera_at = self.now-10
        self.tick(.8, refresh=False)
        self.assertEqual(self.app.manual.state, 'WAIT_REPLAN')
        self.assertEqual(self.app.mission.state, 'NAVIGATING')
        self.assertEqual(self.goals.messages[-1].pose, old_goal.pose)

    def test_sensor_loss_ends_manual_motion_and_does_not_resume_without_readiness(self):
        self.start_mission()
        self.acquire()
        self.app.odom_at = self.now-10
        self.tick(.1, refresh=False)
        self.assertEqual(self.app.manual.state, 'RELEASING')
        self.assertEqual(self.modes()[-1]['mode'], 'hold')
        self.cancel.acknowledge()
        self.tick(.9, refresh=False)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(self.app.manual.state, 'RELEASING')
        self.assertEqual(len(self.goals.messages), 1)



if __name__ == '__main__':
    unittest.main()
