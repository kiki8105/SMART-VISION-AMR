"""Inventory web roles and delivery lifecycle through an isolated HTTP server.

No ROS context, publisher, live server port, or real robot is used. Navigation
success and camera evidence are supplied directly to the same mission decisions
that the real adapters call; a published goal alone never completes a request.
"""
from copy import deepcopy
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, MissionHTTPServer,
    MissionHTTPRequestHandler,
)


class InventoryWebTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.config_file = self.directory/'settings.json'
        (self.directory/'index.html').write_text('<h1>Operator</h1>')
        (self.directory/'inventory.html').write_text('<h1>Inventory</h1>')
        self.app = MissionApplication(DEFAULT_MARKERS, 218, False, self.config_file)
        self.server = MissionHTTPServer(('127.0.0.1', 0), self.app, self.directory)
        self.thread = threading.Thread(
            target=lambda: self.server.serve_forever(poll_interval=.01), daemon=True)
        self.thread.start()
        self.remote = patch.object(MissionHTTPRequestHandler, 'local', property(lambda _: False))
        self.remote.start()
        self.sequence = 0
        self.operator = self.pair_operator()
        self.manager = self.pair_manager(7)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.remote.stop()
        self.temp.cleanup()

    def request(self, method, path, value=None, cookie=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        request_headers = {'Content-Type': 'application/json'}
        if cookie:
            request_headers['Cookie'] = cookie
        request_headers.update(headers or {})
        connection.request(method, path, None if value is None else json.dumps(value), request_headers)
        response = connection.getresponse()
        content, status, returned = response.read(), response.status, dict(response.getheaders())
        connection.close()
        try:
            content = json.loads(content)
        except ValueError:
            pass
        return status, content, returned

    def pair_operator(self):
        status, body, headers = self.request('POST', '/api/pair', {'code': self.app.pairing_code})
        self.assertEqual(status, 200, body)
        self.assertIn('HttpOnly', headers['Set-Cookie'])
        return headers['Set-Cookie'].split(';', 1)[0]

    def pair_manager(self, user_id):
        status, body, headers = self.request('POST', '/api/inventory/pair', {
            'user_id': user_id, 'code': self.app.inventory_join_code})
        self.assertEqual(status, 200, body)
        self.assertEqual(body['user_id'], user_id)
        self.assertIn('Path=/api/inventory', headers['Set-Cookie'])
        return headers['Set-Cookie'].split(';', 1)[0]

    def shortage(self, marker=2, cookie=None, **extra):
        self.sequence += 1
        value = {'marker_id': marker, 'item': '볼트', 'quantity': 3,
                 'client_request_id': f'test-{self.sequence}'}
        value.update(extra)
        status, body, _ = self.request('POST', '/api/inventory/requests', value,
                                      self.manager if cookie is None else cookie)
        self.assertEqual(status, 200, body)
        return body['requests'][-1]

    def ready(self, pose=None):
        self.app.connected = self.app.goal_available = self.app.planner_available = self.app.cancel_available = True
        self.app.aruco_available = True
        self.app.map = {'width': 1, 'height': 1, 'resolution': .05}
        self.app.pose = deepcopy(pose or self.app.delivery.homepoint)
        self.app.pose_at = self.app.scan_at = self.app.odom_at = self.app.camera_at = time.monotonic()
        self.app.speed = {'linear': 0., 'angular': 0.}

    def dispatch(self, **extra):
        payload = {'loaded': True}
        payload.update(extra)
        status, body, _ = self.request('POST', '/api/dispatch/start', payload, self.operator)
        self.assertEqual(status, 200, body)
        return body

    def records(self):
        return self.app.delivery.queue.snapshot()['requests']

    def verified_delivery(self, stamp=(10, 1)):
        mission = self.app.mission
        marker_id = mission.expected_id
        target = next(item for item in self.app.markers if item['id'] == marker_id)
        pose = {key: target[key] for key in ('x', 'y', 'yaw')}
        mission.drain_commands()
        mission.goal_dispatched(stamp)
        mission.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {
            'sec': stamp[0], 'nanosec': stamp[1]}})
        frame_time = time.monotonic()+.01
        for frame in range(mission.required_confirmations):
            mission.observe(frame+(stamp[1]*10), [marker_id], pose, True, frame_time, alignment_verified=True)
            self.app.sync_delivery()

    def verified_home(self, stamp=(20, 0)):
        self.app.mission.drain_commands()
        self.app.mission.goal_dispatched(stamp)
        self.app.mission.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {
            'sec': stamp[0], 'nanosec': stamp[1]}},
            pose=deepcopy(self.app.delivery.homepoint), stationary=True)
        self.app.sync_delivery()

    def test_public_inventory_page_but_api_requires_its_own_session(self):
        self.assertEqual(self.request('GET', '/inventory')[0], 200)
        self.assertEqual(self.request('GET', '/api/inventory/state')[0], 401)
        self.assertEqual(self.request('GET', '/api/inventory/state', cookie=self.operator)[0], 401)
        auth = self.request('GET', '/api/inventory/auth', cookie=self.manager)[1]
        self.assertTrue(auth['authenticated'])
        self.assertEqual(auth['user_id'], 7)
        self.assertFalse(self.request('GET', '/api/auth', cookie=self.manager)[1]['authenticated'])
        operator_auth = self.request('GET', '/api/auth', cookie=self.operator)[1]
        self.assertEqual(operator_auth['user_id'], 0)

    def test_role_codes_are_distinct_and_cannot_be_substituted(self):
        self.assertNotEqual(self.app.inventory_join_code, self.app.pairing_code)
        self.assertEqual(self.request('POST', '/api/pair', {
            'code': self.app.inventory_join_code})[0], 400)
        self.assertEqual(self.request('POST', '/api/inventory/pair', {
            'user_id': 8, 'code': self.app.pairing_code})[0], 400)
        for user_id in (0, 100, True, '7', 7.0):
            with self.subTest(user_id=user_id):
                self.assertEqual(self.request('POST', '/api/inventory/pair', {
                    'user_id': user_id, 'code': self.app.inventory_join_code})[0], 400)

    def test_manager_cannot_access_operator_controls_camera_or_codes(self):
        self.app.camera_jpeg, self.app.map_png = b'camera-private', b'map-private'
        for endpoint in ('/api/state', '/api/dispatch/state', '/api/camera.jpg',
                         '/api/map.png', '/api/health', '/api/markers'):
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.request('GET', endpoint, cookie=self.manager)[0], 401)
        for endpoint in ('/api/dispatch/start', '/api/home/set', '/api/home/reset', '/api/mission/home',
                         '/api/mission/start', '/api/emergency-stop', '/api/connection'):
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.request('POST', endpoint, {}, self.manager)[0], 401)
        state = self.request('GET', '/api/inventory/state', cookie=self.manager)[1]
        text = json.dumps(state)
        self.assertNotIn(self.app.pairing_code, text)
        self.assertNotIn(self.app.inventory_join_code, text)
        self.assertNotIn('camera', state)
        remote_operator = self.request('GET', '/api/state', cookie=self.operator)[1]
        self.assertNotIn('pairing_code', remote_operator['connection'])
        self.assertNotIn('inventory_join_code', remote_operator['delivery'])

    def test_manager_identity_comes_from_cookie_and_only_own_requests_return(self):
        second_manager = self.pair_manager(8)
        self.shortage(user_id=0)
        self.shortage(marker=3, cookie=second_manager, user_id=7)
        first = self.request('GET', '/api/inventory/state', cookie=self.manager)[1]
        second = self.request('GET', '/api/inventory/state', cookie=second_manager)[1]
        self.assertEqual([item['user_id'] for item in first['requests']], [7])
        self.assertEqual([item['user_id'] for item in second['requests']], [8])
        self.assertEqual([item['marker_id'] for item in first['requests']], [2])
        self.assertEqual([item['marker_id'] for item in second['requests']], [3])
        all_requests = self.request('GET', '/api/dispatch/state', cookie=self.operator)[1]['requests']
        self.assertEqual([item['user_id'] for item in all_requests], [7, 8])

    def test_http_idempotency_and_manager_requests_never_start_robot(self):
        self.ready()
        original = self.shortage(client_request_id='same-button')
        duplicate = self.shortage(client_request_id='same-button')
        self.assertEqual(original['id'], duplicate['id'])
        self.assertEqual(len(self.records()), 1)
        self.assertEqual(self.records()[0]['status'], 'PENDING')
        self.assertEqual(self.app.mission.state, 'IDLE')
        self.assertEqual(self.app.mission.commands, [])
        self.assertIsNone(self.app.delivery.queue.snapshot()['active_trip'])

    def test_manager_can_cancel_only_owned_pending_request(self):
        second_manager = self.pair_manager(8)
        mine, other = self.shortage(), self.shortage(cookie=second_manager)
        self.assertEqual(self.request('POST', '/api/inventory/requests/cancel', {
            'id': other['id']}, self.manager)[0], 403)
        self.assertEqual(self.request('POST', '/api/inventory/requests/cancel', {
            'id': mine['id']}, self.manager)[0], 200)
        self.ready()
        self.dispatch()
        self.assertEqual(self.request('POST', '/api/inventory/requests/cancel', {
            'id': other['id']}, second_manager)[0], 400)
        self.assertEqual([item['status'] for item in self.records()], ['CANCELED', 'IN_PROGRESS'])

    def test_row_action_cancels_pending_then_deletes_only_own_history(self):
        other_manager = self.pair_manager(8)
        mine = self.shortage()
        other = self.shortage(cookie=other_manager)
        self.assertEqual(self.request('POST', '/api/inventory/requests/delete',
                                      {'id': mine['id']}, self.manager)[0], 400)
        self.assertEqual(self.request('POST', '/api/inventory/requests/cancel',
                                      {'id': mine['id']}, self.manager)[0], 200)
        self.assertEqual(self.request('POST', '/api/inventory/requests/delete',
                                      {'id': mine['id']}, other_manager)[0], 403)
        status, body, _ = self.request('POST', '/api/inventory/requests/delete',
                                       {'id': mine['id']}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['deletion'], {'deleted': True, 'deferred': False})
        self.assertEqual([item['id'] for item in self.records()], [other['id']])
        self.assertEqual(self.app.mission.commands, [])
        self.assertEqual(self.request('POST', '/api/inventory/requests/delete',
                                      {'id': other['id']}, self.manager)[0], 403)
        self.assertEqual(self.request('POST', '/api/inventory/requests/delete',
                                      {'id': mine['id']}, self.operator)[0], 401)

    def test_row_delete_of_completed_active_trip_is_deferred_without_moving_robot(self):
        first, second = self.shortage(), self.shortage()
        self.app.delivery.queue.start_trip()
        self.app.delivery.queue.complete_request(first['id'])
        status, body, _ = self.request('POST', '/api/inventory/requests/delete',
                                       {'id': first['id']}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['deletion'], {'deleted': False, 'deferred': True})
        self.assertTrue(body['requests'][0]['deletion_pending'])
        self.assertEqual(self.app.delivery.queue.snapshot()['active_trip']['current_request_id'], second['id'])
        self.assertEqual(self.app.mission.commands, [])
        self.app.delivery.queue.cancel_trip()
        self.app.delivery.queue.close_trip()
        self.assertEqual(self.app.delivery.queue.clear_deferred_history_at_home(), 1)
        self.assertEqual([item['id'] for item in self.records()], [second['id']])

    def test_finished_history_reset_is_global_but_keeps_current_trip_and_pending(self):
        second_manager = self.pair_manager(8)
        mine = self.shortage()
        other = self.shortage(cookie=second_manager)
        current = self.shortage()
        pending = self.shortage(cookie=second_manager)
        self.assertEqual(self.request('POST', '/api/inventory/requests/cancel', {
            'id': mine['id']}, self.manager)[0], 200)
        self.assertEqual(self.request('POST', '/api/inventory/requests/cancel', {
            'id': other['id']}, second_manager)[0], 200)
        self.app.delivery.queue.start_trip([current['id']])
        self.app.delivery.queue.complete_request(current['id'])

        state = self.request('GET', '/api/inventory/state', cookie=self.manager)[1]
        self.assertEqual(state['service']['clearable_history_count'], 2)
        self.assertEqual(self.request('POST', '/api/inventory/history/clear', {})[0], 401)
        self.assertEqual(self.request('POST', '/api/inventory/history/clear', {}, self.operator)[0], 401)
        status, cleared, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, cleared)
        self.assertEqual(cleared['removed_count'], 2)
        self.assertEqual(cleared['service']['clearable_history_count'], 0)
        self.assertEqual(cleared['deferred_count'], 1)
        self.assertTrue(cleared['service']['clear_pending'])
        self.assertEqual([item['id'] for item in self.records()], [current['id'], pending['id']])
        self.assertEqual(self.app.delivery.queue.snapshot()['active_trip']['request_ids'], [current['id']])

        self.app.delivery.queue.close_trip()
        self.ready()
        self.app.sync_delivery()
        status, cleared, _ = self.request('POST', '/api/inventory/history/clear', {}, second_manager)
        self.assertEqual(status, 200, cleared)
        self.assertEqual(cleared['removed_count'], 0)
        self.assertEqual([item['id'] for item in self.records()], [pending['id']])
        self.assertEqual(self.request('POST', '/api/inventory/history/clear', {}, second_manager)[1]['removed_count'], 0)
        self.assertGreater(self.shortage()['id'], pending['id'])

    def test_history_reset_with_no_records_reports_already_home_without_goal(self):
        self.ready()
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['removed_count'], 0)
        self.assertEqual(body['home_return']['status'], 'already_home')
        self.assertEqual(body['home_return']['message'], '이미 홈포인트입니다.')
        self.assertEqual(self.app.mission.commands, [])

    def test_history_reset_away_requests_existing_home_return_and_requeues_trip(self):
        old = self.shortage()
        self.request('POST', '/api/inventory/requests/cancel', {'id': old['id']}, self.manager)
        current = self.shortage(marker=3)
        self.ready()
        self.dispatch()
        self.app.pose = dict(self.app.delivery.homepoint, x=1.0)
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['removed_count'], 1)
        self.assertEqual(body['home_return']['status'], 'requested')
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop', 'goal'])
        self.assertEqual(self.app.mission.commands[-1][1], self.app.delivery.homepoint)
        self.assertEqual([(item['id'], item['status']) for item in self.records()],
                         [(current['id'], 'PENDING')])
        self.assertEqual(self.app.delivery.queue.snapshot()['active_trip']['status'], 'RETURNING')

    def test_history_reset_at_home_cancels_active_goal_without_new_motion(self):
        current = self.shortage()
        self.ready()
        self.dispatch()
        self.app.navigation_active = True
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['removed_count'], 0)
        self.assertEqual(body['home_return']['status'], 'already_home')
        self.assertEqual(self.app.mission.state, 'HOME_REACHED')
        self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop'])
        self.assertEqual([(item['id'], item['status']) for item in self.records()],
                         [(current['id'], 'PENDING')])
        self.assertIsNone(self.app.delivery.queue.snapshot()['active_trip'])

    def test_history_reset_at_home_clears_delivered_record_after_closing_trip(self):
        request = self.shortage()
        self.ready()
        self.app.delivery.queue.start_trip([request['id']])
        self.app.delivery.queue.complete_request(request['id'])
        self.assertEqual(self.records()[0]['status'], 'DELIVERED')
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['home_return']['status'], 'already_home')
        self.assertEqual(body['removed_count'], 1)
        self.assertEqual(self.records(), [])
        self.assertIsNone(self.app.delivery.queue.snapshot()['active_trip'])

    def test_history_reset_during_delivery_defers_finished_until_verified_home(self):
        first, second = self.shortage(), self.shortage(marker=3)
        self.ready()
        self.dispatch()
        self.verified_delivery()
        self.assertEqual([item['status'] for item in self.records()], ['DELIVERED', 'IN_PROGRESS'])
        self.app.pose = dict(self.app.delivery.homepoint, x=1.0)
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['removed_count'], 0)
        self.assertEqual(body['deferred_count'], 1)
        self.assertTrue(body['service']['clear_pending'])
        self.assertEqual(body['home_return']['status'], 'requested')
        self.assertEqual([(item['id'], item['status']) for item in self.records()],
                         [(first['id'], 'DELIVERED'), (second['id'], 'PENDING')])
        self.app.pose = deepcopy(self.app.delivery.homepoint)
        self.verified_home()
        self.assertEqual([(item['id'], item['status']) for item in self.records()],
                         [(second['id'], 'PENDING')])
        self.assertEqual(self.app.delivery.queue.snapshot()['deferred_history_ids'], [])

    def test_deferred_history_waits_for_fresh_home_after_server_restart(self):
        first, second = self.shortage(), self.shortage(marker=3)
        self.ready()
        self.dispatch()
        self.verified_delivery()
        self.app.pose = dict(self.app.delivery.homepoint, x=1.0)
        result = self.request('POST', '/api/inventory/history/clear', {}, self.manager)[1]
        self.assertEqual(result['deferred_count'], 1)

        restarted = MissionApplication(DEFAULT_MARKERS, 218, False, self.config_file)
        self.assertIsNone(restarted.delivery.queue.snapshot()['active_trip'])
        self.assertEqual(restarted.delivery.queue.snapshot()['deferred_history_ids'], [first['id']])
        self.assertEqual([(item['id'], item['status'])
                          for item in restarted.delivery.queue.snapshot()['requests']],
                         [(first['id'], 'DELIVERED'), (second['id'], 'PENDING')])
        restarted.connected = True
        restarted.map = {'width': 1, 'height': 1, 'resolution': .05}
        restarted.pose = dict(restarted.delivery.homepoint, x=1.0)
        restarted.pose_at = restarted.odom_at = time.monotonic()
        restarted.sync_delivery()
        self.assertEqual(restarted.delivery.queue.snapshot()['deferred_history_ids'], [first['id']])
        restarted.pose = deepcopy(restarted.delivery.homepoint)
        restarted.sync_delivery()
        self.assertEqual(restarted.delivery.queue.snapshot()['deferred_history_ids'], [first['id']])
        restarted.scan_at = time.monotonic()
        restarted.localization_invalidated = True
        restarted.sync_delivery()
        self.assertEqual(restarted.delivery.queue.snapshot()['deferred_history_ids'], [first['id']])
        restarted.localization_invalidated = False
        restarted.sync_delivery()
        self.assertEqual([(item['id'], item['status'])
                          for item in restarted.delivery.queue.snapshot()['requests']],
                         [(second['id'], 'PENDING')])
        self.assertEqual(restarted.delivery.queue.snapshot()['deferred_history_ids'], [])

    def test_history_reset_offline_or_stale_pose_clears_records_without_home_goal(self):
        for outage in ('offline', 'stale_pose'):
            with self.subTest(outage=outage):
                request = self.shortage()
                self.request('POST', '/api/inventory/requests/cancel', {'id': request['id']}, self.manager)
                self.ready()
                if outage == 'offline':
                    self.app.connected = False
                else:
                    self.app.pose_at = time.monotonic()-3.
                status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
                self.assertEqual(status, 200, body)
                self.assertEqual(body['removed_count'], 1)
                self.assertEqual(body['home_return']['status'], 'unavailable')
                self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_history_reset_unready_while_moving_pauses_existing_goal(self):
        self.shortage()
        self.ready()
        self.dispatch()
        self.app.pose = dict(self.app.delivery.homepoint, x=1.0)
        self.app.scan_at = 0.
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['home_return']['status'], 'unavailable')
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop'])
        self.assertEqual(self.records()[0]['status'], 'IN_PROGRESS')

    def test_history_reset_unready_cancels_orphaned_navigation(self):
        self.ready()
        self.app.navigation_active = True
        self.app.scan_at = 0.
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['home_return']['status'], 'unavailable')
        self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop'])

    def test_history_reset_estop_does_not_command_motion(self):
        self.ready()
        self.app.mission.emergency_stop()
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['home_return']['status'], 'blocked')
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_history_reset_stops_localization_before_home_request(self):
        self.ready()
        self.app.localization.start()
        status, body, _ = self.request('POST', '/api/inventory/history/clear', {}, self.manager)
        self.assertEqual(status, 200, body)
        self.assertEqual(body['home_return']['status'], 'unavailable')
        self.assertFalse(self.app.localization.active)
        self.assertTrue(self.app.localization_stop_requested)
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_dispatch_requires_explicit_boolean_loading_acknowledgment(self):
        self.shortage()
        self.ready()
        for payload in ({}, {'loaded': False}, {'loaded': 1}, {'loaded': 'true'}):
            with self.subTest(payload=payload):
                self.assertEqual(self.request('POST', '/api/dispatch/start', payload, self.operator)[0], 400)
                self.assertEqual(self.records()[0]['status'], 'PENDING')
                self.assertEqual(self.app.mission.commands, [])
        body = self.dispatch()
        self.assertEqual(body['mission']['state'], 'NAVIGATING')
        self.assertEqual(body['mission']['home'], self.app.delivery.homepoint)
        self.assertEqual(self.records()[0]['status'], 'IN_PROGRESS')

    def test_dispatch_requires_fresh_home_pose_heading_and_stationary_feedback(self):
        self.shortage()
        mutations = [lambda: setattr(self.app, 'pose', None),
                     lambda: setattr(self.app, 'pose_at', time.monotonic()-3),
                     lambda: setattr(self.app, 'odom_at', time.monotonic()-3),
                     lambda: self.app.pose.update(x=self.app.pose['x']+.31),
                     lambda: self.app.pose.update(yaw=self.app.pose['yaw']+.36),
                     lambda: self.app.speed.update(linear=.03),
                     lambda: self.app.speed.update(angular=.07)]
        for mutate in mutations:
            self.ready()
            mutate()
            status, body, _ = self.request('POST', '/api/dispatch/start', {'loaded': True}, self.operator)
            self.assertEqual(status, 400, body)
            self.assertEqual(self.app.mission.commands, [])
            self.assertEqual(self.records()[0]['status'], 'PENDING')

    def test_dispatch_requires_sensors_and_idle_adapter_even_at_home(self):
        self.shortage()
        for field, value in (('camera_at', 0), ('scan_at', 0), ('connected', False),
                             ('aruco_available', False), ('cancel_available', False),
                             ('navigation_active', True), ('cancel_pending', True),
                             ('pending_connection', (219, False))):
            self.ready()
            self.app.navigation_active = self.app.cancel_pending = False
            self.app.pending_connection = None
            setattr(self.app, field, value)
            with self.subTest(field=field):
                self.assertEqual(self.request('POST', '/api/dispatch/start', {'loaded': True}, self.operator)[0], 400)
                self.assertEqual(self.records()[0]['status'], 'PENDING')
                self.assertEqual(self.app.mission.commands, [])

    def test_front_sector_dispatch_requires_front_camera_not_unrelated_rear(self):
        self.app.markers[2]['camera'] = 'front'
        self.app.mission.markers[2]['camera'] = 'front'
        self.shortage(marker=2)
        self.ready()
        missing = self.app.delivery.snapshot()['dispatch_missing']
        self.assertIn('최신 전방 카메라 영상', missing)
        status, body, _ = self.request('POST', '/api/dispatch/start', {'loaded': True}, self.operator)
        self.assertEqual(status, 400)
        self.assertIn('최신 전방 카메라 영상', body['error'])
        self.assertEqual(self.records()[0]['status'], 'PENDING')
        self.assertEqual(self.app.mission.commands, [])

        self.app.front_camera_at = time.monotonic()
        self.app.camera_at = 0.
        self.assertTrue(self.app.delivery.snapshot()['dispatch_ready'])
        body = self.dispatch()
        self.assertEqual(body['mission']['route'], [2])
        self.assertEqual(self.records()[0]['status'], 'IN_PROGRESS')

    def test_fixed_homepoint_persists_and_interrupted_queue_recovers_pending(self):
        self.ready({'x': 1.2, 'y': -.3, 'yaw': .4})
        status, body, _ = self.request('POST', '/api/home/set', {}, self.operator)
        self.assertEqual(status, 200, body)
        fixed = deepcopy(self.app.pose)
        self.app.pose['x'] = 99.
        self.assertEqual(self.app.delivery.homepoint, fixed)
        self.ready(fixed)
        self.shortage()
        self.shortage(marker=3)
        self.dispatch()
        restarted = MissionApplication(DEFAULT_MARKERS, 218, False, self.config_file)
        self.assertEqual(restarted.delivery.homepoint, fixed)
        self.assertEqual(restarted.mission.home, fixed)
        self.assertEqual([item['status'] for item in restarted.delivery.snapshot()['requests']], ['PENDING']*2)
        self.assertIsNone(restarted.delivery.snapshot()['active_trip'])
        self.assertEqual(restarted.mission.state, 'IDLE')
        self.assertEqual(restarted.mission.commands, [])

    def test_homepoint_persistence_error_does_not_change_live_home(self):
        self.ready({'x': 1.2, 'y': -.3, 'yaw': .4})
        original = deepcopy(self.app.delivery.homepoint)
        with patch.object(self.app, 'save', side_effect=OSError('read only')):
            status, _, _ = self.request('POST', '/api/home/set', {}, self.operator)
        self.assertEqual(status, 500)
        self.assertEqual(self.app.delivery.homepoint, original)
        self.assertEqual(self.app.mission.home, original)
        self.assertEqual(self.app.mission.commands, [])

    def test_failed_homepoint_save_does_not_resurrect_from_mutated_settings_cache(self):
        self.ready()
        self.app.save()
        original_home = deepcopy(self.app.delivery.homepoint)
        original_saved = deepcopy(self.app.saved)
        self.ready({'x': 1.2, 'y': -.3, 'yaw': .4})
        with patch('smart_vision_amr.web_mission_server.os.replace', side_effect=OSError('full disk')):
            status, _, _ = self.request('POST', '/api/home/set', {}, self.operator)
        self.assertEqual(status, 500)
        self.assertEqual(self.app.saved, original_saved)
        self.assertEqual(self.app.delivery.homepoint, original_home)
        self.app.save()
        restarted = MissionApplication(DEFAULT_MARKERS, 218, False, self.config_file)
        self.assertEqual(restarted.delivery.homepoint, original_home)
        self.assertEqual(restarted.mission.commands, [])

    def test_cancel_storage_failure_stops_current_motion_and_preserves_request_state(self):
        for endpoint in ('/api/mission/cancel', '/api/mission/home'):
            # Separate server-owned objects keep each scenario independent.
            self.app = MissionApplication(DEFAULT_MARKERS, 218, False, self.directory/f'{endpoint.rsplit("/", 1)[1]}.json')
            self.server.app = self.app
            self.operator = self.pair_operator()
            self.manager = self.pair_manager(7)
            self.shortage()
            self.ready()
            self.dispatch()
            before = self.app.delivery.queue.snapshot()
            with patch.object(self.app.delivery.queue, '_persist', side_effect=OSError('read only')):
                status, body, _ = self.request('POST', endpoint, {}, self.operator)
            with self.subTest(endpoint=endpoint):
                self.assertEqual(status, 500, body)
                self.assertEqual(self.app.delivery.queue.snapshot(), before)
                self.assertEqual(self.app.mission.state, 'PAUSED')
                self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop'])
                self.assertIn('오류', self.app.delivery.error)

    def test_verified_camera_delivery_drains_fifo_then_returns_home(self):
        first, second = self.shortage(), self.shortage(marker=3)
        self.ready()
        self.dispatch()
        original_goal = deepcopy(self.app.mission.commands[-1])
        third = self.shortage(marker=1)
        self.assertEqual(self.app.mission.route, [2, 3, 1])
        self.assertEqual(self.app.mission.commands[-1], original_goal)
        self.assertEqual([item['status'] for item in self.records()], ['IN_PROGRESS', 'QUEUED', 'QUEUED'])
        self.verified_delivery((10, 1))
        self.assertEqual([item['status'] for item in self.records()], ['DELIVERED', 'IN_PROGRESS', 'QUEUED'])
        self.verified_delivery((10, 2))
        self.verified_delivery((10, 3))
        self.assertEqual([item['status'] for item in self.records()], ['DELIVERED']*3)
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertEqual(self.app.mission.commands[-1], ('goal', self.app.delivery.homepoint))
        trip = self.app.delivery.queue.snapshot()['active_trip']
        self.assertEqual(trip['request_ids'], [first['id'], second['id'], third['id']])
        self.assertEqual(trip['status'], 'RETURNING')
        self.assertIsNotNone(trip)
        self.verified_home()
        self.assertEqual(self.app.mission.state, 'HOME_REACHED')
        self.assertIsNone(self.app.delivery.queue.snapshot()['active_trip'])
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_arrival_needs_correlated_navigation_and_post_arrival_camera_evidence(self):
        self.shortage()
        self.ready()
        self.dispatch()
        mission = self.app.mission
        pose = {key: mission.markers[2][key] for key in ('x', 'y', 'yaw')}
        for frame in range(5):
            mission.observe(frame, [2], pose, True, time.monotonic()+1., alignment_verified=True)
        self.app.sync_delivery()
        self.assertEqual(self.records()[0]['status'], 'IN_PROGRESS')
        mission.goal_dispatched((40, 1))
        mission.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': 40, 'nanosec': 0}})
        self.assertEqual(mission.state, 'NAVIGATING')
        mission.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': 40, 'nanosec': 1}})
        self.app.sync_delivery()
        self.assertEqual(self.records()[0]['status'], 'IN_PROGRESS')
        # Old frames, wrong markers, motion, and wrong position never deliver.
        for frame, marker, where, stationary, captured in (
                (10, 2, pose, True, mission.observation_since-1.),
                (11, 3, pose, True, time.monotonic()+1.),
                (12, 2, pose, False, time.monotonic()+1.),
                (13, 2, dict(pose, x=100.), True, time.monotonic()+1.)):
            mission.observe(frame, [marker], where, stationary, captured, alignment_verified=True)
            self.app.sync_delivery()
            self.assertEqual(self.records()[0]['status'], 'IN_PROGRESS')
        for frame in (20, 21, 22):
            mission.observe(frame, [2], pose, True, time.monotonic()+1., alignment_verified=True)
            self.app.sync_delivery()
        self.assertEqual(self.records()[0]['status'], 'DELIVERED')
        self.assertEqual(mission.state, 'RETURNING_HOME')

    def test_request_during_automatic_home_return_cancels_home_and_continues(self):
        self.shortage()
        self.ready()
        self.dispatch()
        self.verified_delivery()
        self.assertTrue(self.app.mission.auto_returning_home)
        new = self.shortage(marker=3)
        self.assertEqual(new['status'], 'IN_PROGRESS')
        self.assertEqual(self.app.mission.state, 'NAVIGATING')
        self.assertFalse(self.app.mission.returning_home)
        self.assertEqual(self.app.mission.route, [2, 3])
        commands = self.app.mission.commands
        self.assertEqual([kind for kind, _ in commands], ['cancel', 'stop', 'goal'])
        self.assertEqual(commands[-1][1]['id'], 3)
        self.assertEqual(self.app.delivery.queue.snapshot()['active_trip']['status'], 'ACTIVE')

    def test_new_shortage_while_paused_extends_queue_but_never_restarts_motion(self):
        self.shortage()
        self.ready()
        self.dispatch()
        self.assertEqual(self.request('POST', '/api/mission/pause', {}, self.operator)[0], 200)
        self.app.mission.drain_commands()
        self.shortage(marker=3)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(self.app.mission.route, [2, 3])
        self.assertEqual(self.app.mission.commands, [])
        self.assertEqual([item['status'] for item in self.records()], ['IN_PROGRESS', 'QUEUED'])
        self.assertEqual(self.request('POST', '/api/mission/resume', {}, self.operator)[0], 200)
        self.assertEqual(self.app.mission.state, 'NAVIGATING')

    def test_request_while_paused_automatic_return_requires_explicit_resume(self):
        self.shortage()
        self.ready()
        self.dispatch()
        self.verified_delivery()
        self.request('POST', '/api/mission/pause', {}, self.operator)
        self.app.mission.drain_commands()
        self.shortage(marker=3)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertFalse(self.app.mission.returning_home)
        self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop'])
        self.assertEqual(self.app.mission.expected_id, 3)

    def test_estop_requeues_unfulfilled_work_and_submissions_never_resume(self):
        self.shortage()
        self.ready()
        self.dispatch()
        status, body, _ = self.request('POST', '/api/emergency-stop', {}, self.operator)
        self.assertEqual(status, 200, body)
        self.shortage(marker=3)
        self.assertEqual([item['status'] for item in self.records()], ['PENDING']*2)
        self.assertEqual(self.app.mission.state, 'ESTOP')
        self.assertFalse(self.app.delivery.snapshot()['accepting_updates'])
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))
        self.app.manual_mux_at = time.monotonic()
        self.app.manual_mux_status = {'estop_latched': True, 'stop_acknowledged': True}
        status, _, _ = self.request('POST', '/api/emergency-stop', {'reset': True}, self.operator)
        self.assertEqual(status, 200)
        self.assertEqual(self.app.mission.state, 'ESTOP')
        self.app.complete_safety_result('reset', True, 'Robot safety reset acknowledged')
        self.app.sync_delivery()
        self.assertEqual(self.app.mission.state, 'CANCELED')
        self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_explicit_cancel_returns_home_and_requeues_only_unfulfilled_work(self):
        self.shortage()
        self.shortage(marker=3)
        self.ready()
        self.dispatch()
        self.verified_delivery()
        status, body, _ = self.request('POST', '/api/mission/cancel', {}, self.operator)
        self.assertEqual(status, 200, body)
        self.assertEqual([item['status'] for item in self.records()], ['DELIVERED', 'PENDING'])
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertFalse(self.app.mission.auto_return_home)
        self.shortage(marker=1)
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertEqual([item['status'] for item in self.records()], ['DELIVERED', 'PENDING', 'PENDING'])
        self.assertEqual(self.app.mission.commands[-1][1], self.app.delivery.homepoint)
        self.verified_home()
        self.assertIsNone(self.app.delivery.queue.snapshot()['active_trip'])
        self.ready()
        self.dispatch()
        self.assertEqual(self.app.mission.route, [3, 1])

    def test_logout_invalidates_only_inventory_session(self):
        status, body, headers = self.request('POST', '/api/inventory/logout', {}, self.manager)
        self.assertEqual(status, 200)
        self.assertFalse(body['authenticated'])
        self.assertIn('Max-Age=0', headers['Set-Cookie'])
        self.assertEqual(self.request('GET', '/api/inventory/state', cookie=self.manager)[0], 401)
        self.assertTrue(self.request('GET', '/api/auth', cookie=self.operator)[1]['authenticated'])


if __name__ == '__main__':
    unittest.main()
