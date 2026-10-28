"""Domain-based web access without a ROS context or a live robot connection."""
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


class WebDomainAccessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        (self.directory/'index.html').write_text('<h1>Operator</h1>')
        (self.directory/'inventory.html').write_text('<h1>Inventory</h1>')
        # This number is application data only: these tests never initialize ROS.
        self.domain_id = 217
        self.app = MissionApplication(
            DEFAULT_MARKERS, self.domain_id, False,
            self.directory/'settings.json', access_mode='domain')
        self.server = MissionHTTPServer(('127.0.0.1', 0), self.app, self.directory)
        self.thread = threading.Thread(
            target=lambda: self.server.serve_forever(poll_interval=.01), daemon=True)
        self.thread.start()
        self.remote = patch.object(
            MissionHTTPRequestHandler, 'local', property(lambda _: False))
        self.remote.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.remote.stop()
        self.temp.cleanup()

    def request(self, method, path, value=None, cookie=None, headers=None):
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.server.server_port, timeout=3)
        request_headers = {'Content-Type': 'application/json'}
        if cookie:
            request_headers['Cookie'] = cookie
        request_headers.update(headers or {})
        connection.request(method, path, None if value is None else json.dumps(value),
                           request_headers)
        response = connection.getresponse()
        content, status = response.read(), response.status
        returned_headers = dict(response.getheaders())
        connection.close()
        try:
            content = json.loads(content)
        except ValueError:
            pass
        return status, content, returned_headers

    def operator_cookie(self):
        status, body, headers = self.request(
            'POST', '/api/pair', {'domain_id': self.domain_id})
        self.assertEqual(status, 200, body)
        self.assertTrue(body['authenticated'])
        self.assertIn('HttpOnly', headers['Set-Cookie'])
        self.assertIn('SameSite=Strict', headers['Set-Cookie'])
        return headers['Set-Cookie'].split(';', 1)[0]

    def manager_cookie(self, user_id=7):
        status, body, headers = self.request(
            'POST', '/api/inventory/pair',
            {'user_id': user_id, 'domain_id': self.domain_id})
        self.assertEqual(status, 200, body)
        self.assertEqual(body['user_id'], user_id)
        self.assertIn('HttpOnly', headers['Set-Cookie'])
        self.assertIn('SameSite=Strict', headers['Set-Cookie'])
        self.assertIn('Path=/api/inventory', headers['Set-Cookie'])
        return headers['Set-Cookie'].split(';', 1)[0]

    def test_access_mode_is_public_without_exposing_domain_or_codes(self):
        status, body, _ = self.request('GET', '/api/access')
        self.assertEqual(status, 200)
        self.assertEqual(body, {'mode': 'domain'})
        self.assertEqual(self.request('GET', '/')[0], 200)
        self.assertEqual(self.request('GET', '/inventory')[0], 200)
        self.assertEqual(self.request('GET', '/api/state')[0], 401)
        self.assertEqual(self.request('GET', '/api/inventory/state')[0], 401)

    def test_matching_domain_pairs_operator_and_manager_without_terminal_codes(self):
        operator = self.operator_cookie()
        manager = self.manager_cookie()
        status, state, _ = self.request('GET', '/api/state', cookie=operator)
        self.assertEqual(status, 200)
        self.assertEqual(state['connection']['domain_id'], self.domain_id)
        self.assertNotIn('pairing_code', state['connection'])
        status, state, _ = self.request('GET', '/api/inventory/state', cookie=manager)
        self.assertEqual(status, 200)
        self.assertEqual(state['user_id'], 7)
        self.assertNotIn('inventory_join_code', state)

    def test_http_requires_domain_and_rejects_a_different_robot(self):
        for endpoint, base in (('/api/pair', {}),
                               ('/api/inventory/pair', {'user_id': 8})):
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.request('POST', endpoint, base)[0], 400)
                self.assertEqual(self.request(
                    'POST', endpoint, dict(base, domain_id=self.domain_id-1))[0], 400)
        self.assertFalse(self.app.sessions)
        self.assertFalse(self.app.inventory_sessions)

    def test_domain_is_a_strict_integer_in_range_for_both_roles(self):
        for index, value in enumerate((None, True, False, '217', 217.0, -1, 233,
                                       [], {}, self.domain_id-1)):
            with self.subTest(domain_id=value):
                with self.assertRaises(ValueError):
                    self.app.pair(f'operator-{index}', '', domain_id=value)
                with self.assertRaises(ValueError):
                    self.app.pair_inventory(
                        f'manager-{index}', 7, '', domain_id=value)
        self.assertFalse(self.app.sessions)
        self.assertFalse(self.app.inventory_sessions)

    def test_matching_domain_does_not_allow_invalid_manager_identity(self):
        for user_id in (None, 0, 100, True, '7', 7.0):
            with self.subTest(user_id=user_id):
                with self.assertRaises(ValueError):
                    self.app.pair_inventory(
                        'manager', user_id, '', domain_id=self.domain_id)
        self.assertFalse(self.app.inventory_sessions)

    def test_domain_mode_does_not_accept_old_code_instead_of_domain(self):
        with self.assertRaises(ValueError):
            self.app.pair('operator', self.app.pairing_code)
        with self.assertRaises(ValueError):
            self.app.pair_inventory('manager', 7, self.app.inventory_join_code)

    def test_role_separation_remains_after_domain_pairing(self):
        operator = self.operator_cookie()
        manager = self.manager_cookie()
        self.assertFalse(self.request('GET', '/api/auth', cookie=manager)[1]['authenticated'])
        self.assertEqual(self.request('GET', '/api/auth', cookie=operator)[1]['user_id'], 0)
        self.assertFalse(self.request(
            'GET', '/api/inventory/auth', cookie=operator)[1]['authenticated'])
        for endpoint in ('/api/state', '/api/dispatch/state', '/api/camera.jpg'):
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.request('GET', endpoint, cookie=manager)[0], 401)
        for endpoint in ('/api/mission/start', '/api/dispatch/start', '/api/connection'):
            with self.subTest(endpoint=endpoint):
                self.assertEqual(self.request('POST', endpoint, {}, cookie=manager)[0], 401)
        self.assertEqual(self.request('GET', '/api/inventory/state', cookie=operator)[0], 401)
        self.assertEqual(self.app.mission.commands, [])

    def test_domain_sessions_expire_for_both_roles(self):
        operator = self.app.pair('operator', '', domain_id=self.domain_id)
        manager = self.app.pair_inventory('manager', 7, '', domain_id=self.domain_id)
        self.app.sessions[operator] = time.monotonic()-1
        self.app.inventory_sessions[manager]['expires'] = time.monotonic()-1
        self.assertFalse(self.app.authenticated('amr_session='+operator))
        self.assertIsNone(self.app.inventory_identity('amr_inventory_session='+manager))

    def test_session_for_one_robot_cannot_authorize_another_domain(self):
        operator = self.operator_cookie()
        manager = self.manager_cookie()
        self.app.domain_id = self.domain_id-1
        self.assertEqual(self.request('GET', '/api/state', cookie=operator)[0], 401)
        self.assertEqual(self.request('GET', '/api/inventory/state', cookie=manager)[0], 401)
        self.assertEqual(self.request(
            'POST', '/api/mission/start', {}, cookie=operator)[0], 401)
        self.assertEqual(self.app.mission.commands, [])

    def test_domain_mode_does_not_advertise_unused_pin_codes_locally(self):
        state = self.app.state(local=True)
        self.assertNotIn('pairing_code', state['connection'])
        self.assertNotIn('inventory_join_code', state['delivery'])
        self.assertEqual(self.request('GET', '/api/auth')[1]['access_mode'], 'domain')
        self.assertEqual(self.request(
            'GET', '/api/inventory/auth')[1]['access_mode'], 'domain')

    def test_domain_failures_keep_existing_pair_rate_limits(self):
        for _ in range(5):
            with self.assertRaises(ValueError):
                self.app.pair('operator', '', domain_id=self.domain_id-1)
            with self.assertRaises(ValueError):
                self.app.pair_inventory('manager', 7, '', domain_id=self.domain_id-1)
        with self.assertRaisesRegex(ValueError, '1분'):
            self.app.pair('operator', '', domain_id=self.domain_id)
        with self.assertRaisesRegex(ValueError, '1분'):
            self.app.pair_inventory('manager', 7, '', domain_id=self.domain_id)

    def test_default_code_mode_still_requires_its_distinct_role_codes(self):
        code_app = MissionApplication(
            DEFAULT_MARKERS, self.domain_id, False, self.directory/'code-settings.json')
        self.server.app = code_app
        self.assertEqual(self.request('GET', '/api/access')[1], {'mode': 'code'})
        for endpoint, base in (('/api/pair', {}),
                               ('/api/inventory/pair', {'user_id': 8})):
            self.assertEqual(self.request(
                'POST', endpoint, dict(base, domain_id=self.domain_id))[0], 400)
        self.assertEqual(self.request('POST', '/api/pair', {
            'code': code_app.inventory_join_code})[0], 400)
        self.assertEqual(self.request('POST', '/api/inventory/pair', {
            'user_id': 8, 'code': code_app.pairing_code})[0], 400)
        self.assertEqual(self.request('POST', '/api/pair', {
            'code': code_app.pairing_code})[0], 200)
        self.assertEqual(self.request('POST', '/api/inventory/pair', {
            'user_id': 8, 'code': code_app.inventory_join_code})[0], 200)


if __name__ == '__main__':
    unittest.main()
