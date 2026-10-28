"""HTTP contracts, authentication, portable persistence, camera conversion."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from smart_vision_amr.web_mission_server import (DEFAULT_MARKERS, MissionApplication,
                                               MissionHTTPServer, MissionHTTPRequestHandler, image_to_bgr, load_markers)


class WebMissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        (self.directory/'index.html').write_text('<h1>Mission</h1>')
        self.app = MissionApplication(DEFAULT_MARKERS, 215, False, self.directory/'settings.json')
        self.server = MissionHTTPServer(('127.0.0.1', 0), self.app, self.directory)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.temp.cleanup()

    def request(self, method, path, value=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        payload = None if value is None else json.dumps(value)
        request_headers = {'Content-Type': 'application/json'}
        request_headers.update(headers or {})
        connection.request(method, path, payload, request_headers)
        response = connection.getresponse()
        content, status, returned_headers = response.read(), response.status, dict(response.getheaders())
        connection.close()
        try:
            content = json.loads(content)
        except ValueError:
            pass
        return status, content, returned_headers

    def ready(self, camera=True):
        self.app.connected = self.app.goal_available = self.app.planner_available = self.app.cancel_available = True
        self.app.map = {'width': 1}
        self.app.pose = {'x': -.7, 'y': -1.2, 'yaw': .1}
        self.app.pose_at = self.app.scan_at = self.app.odom_at = time.monotonic()
        self.app.camera_at = time.monotonic() if camera else 0

    def test_static_page_and_local_auth(self):
        self.assertEqual(self.request('GET', '/')[0], 200)
        auth = self.request('GET', '/api/auth')[1]
        self.assertTrue(auth['authenticated'])
        state = self.request('GET', '/api/state')[1]
        self.assertEqual(state['connection']['domain_id'], 215)
        self.assertEqual(state['connection']['pairing_code'], self.app.pairing_code)
        self.assertFalse(state['health']['ready'])

    def test_host_and_origin_rejection(self):
        self.assertEqual(self.request('GET', '/api/state', headers={'Host': 'evil.example:'+str(self.server.server_port)})[0], 403)
        self.assertEqual(self.request('POST', '/api/mission', {'route': [2]}, {'Origin': 'https://evil.example'})[0], 403)

    def test_start_rejects_missing_runtime_even_with_route(self):
        status, content, _ = self.request('POST', '/api/mission/start', {})
        self.assertEqual(status, 400)
        self.assertIn('/map', content['error'])
        self.assertEqual(self.app.mission.state, 'IDLE')

    def test_camera_outage_allows_existing_route_resume_but_not_new_departure(self):
        self.ready()
        self.assertEqual(self.request('POST', '/api/mission/start', {})[0], 200)
        self.assertEqual(self.request('POST', '/api/mission/pause', {})[0], 200)
        self.app.camera_at = 0
        health = self.request('GET', '/api/health')[1]
        self.assertTrue(health['ready'])
        self.assertTrue(health['warnings'])
        self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 200)
        self.assertEqual(self.app.mission.state, 'NAVIGATING')
        self.assertEqual(self.request('POST', '/api/mission/pause', {})[0], 200)
        self.assertEqual(self.request('POST', '/api/mission/restart', {})[0], 400)
        self.app.mission.state = 'HOME_REACHED'
        self.assertEqual(self.request('POST', '/api/mission/start', {})[0], 400)

    def test_front_marker_requires_front_video_to_start_but_can_resume_degraded(self):
        self.app.markers[2]['camera'] = 'front'
        self.app.mission.markers[2]['camera'] = 'front'
        self.assertEqual(self.request('POST', '/api/mission', {'route': [2]})[0], 200)
        self.ready(camera=False)
        status, body, _ = self.request('POST', '/api/mission/start', {})
        self.assertEqual(status, 400)
        self.assertIn('최신 전방 카메라 영상', body['error'])
        self.assertEqual(self.app.mission.commands, [])
        self.app.front_camera_at = time.monotonic()
        self.assertEqual(self.request('POST', '/api/mission/start', {})[0], 200,
                         'a front-only route must not require the rear camera')
        self.assertEqual(self.request('POST', '/api/mission/pause', {})[0], 200)
        self.app.front_camera_at = 0.
        health = self.request('GET', '/api/health')[1]
        self.assertTrue(health['ready'])
        self.assertTrue(any('전방 카메라' in warning for warning in health['warnings']))
        self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 200)

    def test_degraded_camera_cannot_override_scan_or_aruco_failure(self):
        self.ready()
        self.assertEqual(self.request('POST', '/api/mission/start', {})[0], 200)
        self.assertEqual(self.request('POST', '/api/mission/pause', {})[0], 200)
        self.app.camera_at = 0
        self.app.scan_at = 0
        self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 400)
        self.app.scan_at = time.monotonic()
        self.app.aruco_available = False
        self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 400)

    def test_route_validation_and_atomic_persistence(self):
        self.assertEqual(self.request('POST', '/api/mission', {'route': [3, 2, 3, 0]})[0], 200)
        stored = json.loads((self.directory/'settings.json').read_text())
        self.assertEqual(stored['domains']['215']['route'], [3, 2, 3, 0])
        restarted = MissionApplication(DEFAULT_MARKERS, 215, False, self.directory/'settings.json')
        self.assertEqual(restarted.mission.route, [3, 2, 3, 0])
        self.assertEqual(self.request('POST', '/api/mission', {'route': []})[0], 400)

    def test_emergency_reset_no_motion(self):
        status, body, _ = self.request('POST', '/api/emergency-stop', {})
        self.assertEqual(status, 200)
        self.assertEqual(body['mission']['state'], 'ESTOP')
        status, body, _ = self.request('POST', '/api/emergency-stop', {'reset': True})
        self.assertEqual(body['mission']['state'], 'CANCELED')
        self.assertFalse(body['mission']['estop_latched'])

    def test_connection_validation_and_busy_rejection(self):
        self.assertEqual(self.request('POST', '/api/connection', {'domain_id': '76'})[0], 400)
        self.assertEqual(self.request('POST', '/api/connection', {'domain_id': 233})[0], 400)
        self.assertEqual(self.request('POST', '/api/connection', {'domain_id': 214, 'use_sim_time': True})[0], 200)
        self.assertEqual(self.app.pending_connection, (214, True))
        self.assertEqual(self.request('POST', '/api/mission/home', {})[0], 400)
        self.assertEqual(self.app.mission.commands, [])
        self.app.mission.start()
        self.assertEqual(self.request('POST', '/api/connection', {'domain_id': 76})[0], 400)

    def test_colcon_symlink_installed_public_assets(self):
        with tempfile.TemporaryDirectory() as source_directory:
            source = Path(source_directory)/'source'
            build = Path(source_directory)/'build'
            source.mkdir()
            build.mkdir()
            for filename, content in {'index.html': '<h1>Installed mission</h1>',
                                      'app.js': 'window.installed = true;',
                                      'styles.css': 'body { color: black; }',
                                      'favicon.svg': '<svg></svg>'}.items():
                (source/filename).write_text(content)
                (build/filename).symlink_to(source/filename)
                installed = self.directory/filename
                installed.unlink(missing_ok=True)
                installed.symlink_to(build/filename)
                status, body, _ = self.request('GET', '/'+filename)
                self.assertEqual(status, 200)
                self.assertEqual(body, content.encode())
            self.assertEqual(self.request('GET', '/')[0], 200)
            self.assertEqual(self.request('GET', '/%2e%2e/source/index.html')[0], 404)
            self.assertEqual(self.request('GET', '/settings.json')[0], 404)
            self.assertEqual(self.request('GET', '/app.js/%2e%2e/settings.json')[0], 404)

    def test_no_private_path_traversal(self):
        outside = self.directory.parent/'outside-web-mission-secret'
        outside.write_text('secret')
        try:
            self.assertEqual(self.request('GET', '/%2e%2e/outside-web-mission-secret')[0], 404)
        finally:
            outside.unlink()

    def test_unknown_endpoints_content_type_and_big_payload(self):
        self.assertEqual(self.request('POST', '/api/mission', {'route': [2]}, {'Content-Type': 'text/plain'})[0], 415)
        self.assertEqual(self.request('POST', '/api/nope', {})[0], 404)
        self.assertEqual(self.request('GET', '/api/nope')[0], 404)
        self.assertEqual(self.request('POST', '/api/mission', {'junk': 'x'*66000})[0], 413)

    def test_remote_session_pairing_and_no_code_leak(self):
        self.assertFalse(self.app.authenticated(None))
        self.assertNotIn('pairing_code', self.app.state()['connection'])
        with self.assertRaises(ValueError):
            self.app.pair('192.0.2.4', 'bad')
        token = self.app.pair('192.0.2.4', self.app.pairing_code)
        self.assertTrue(self.app.authenticated('amr_session='+token))
        self.assertFalse(self.app.authenticated('amr_session=bad'))
        self.app.sessions[token] = time.monotonic()-1
        self.assertFalse(self.app.authenticated('amr_session='+token))

    def test_persistence_failure_never_queues_start_or_resume(self):
        now = time.monotonic()
        self.app.connected = self.app.goal_available = self.app.planner_available = self.app.cancel_available = True
        self.app.map = {'width': 1}
        self.app.pose = {'x': 0., 'y': 0., 'yaw': 0.}
        self.app.pose_at = self.app.scan_at = self.app.camera_at = self.app.odom_at = now
        with patch.object(self.app, 'save', side_effect=OSError('read-only configuration')):
            self.assertEqual(self.request('POST', '/api/mission/start', {})[0], 500)
            self.assertEqual(self.app.mission.state, 'IDLE')
            self.assertEqual(self.app.mission.commands, [])
            old_route = list(self.app.mission.route)
            self.assertEqual(self.request('POST', '/api/mission', {'route': [1, 0]})[0], 500)
            self.assertEqual(self.app.mission.route, old_route)
            self.app.mission.start()
            self.app.mission.pause()
            self.app.mission.drain_commands()
            self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 500)
            self.assertEqual(self.app.mission.state, 'PAUSED')
            self.assertEqual(self.app.mission.commands, [])

    def test_remote_http_requires_pair_and_hides_code(self):
        with patch.object(MissionHTTPRequestHandler, 'local', property(lambda _: False)):
            status, body, _ = self.request('GET', '/api/state')
            self.assertEqual(status, 401)
            self.assertTrue(body['pairing_required'])
            self.assertFalse(self.request('GET', '/api/auth')[1]['authenticated'])
            status, _, headers = self.request('POST', '/api/pair', {'code': self.app.pairing_code})
            self.assertEqual(status, 200)
            cookie = headers['Set-Cookie'].split(';', 1)[0]
            status, body, _ = self.request('GET', '/api/state', headers={'Cookie': cookie})
            self.assertEqual(status, 200)
            self.assertNotIn('pairing_code', body['connection'])
            self.assertNotIn('lan_urls', body['connection'])

    def test_pair_rate_limit(self):
        for _ in range(5):
            with self.assertRaises(ValueError):
                self.app.pair('192.0.2.5', 'bad')
        with self.assertRaisesRegex(ValueError, '1분'):
            self.app.pair('192.0.2.5', self.app.pairing_code)

    def test_pair_http_cookie_and_missing_images(self):
        status, _, headers = self.request('POST', '/api/pair', {'code': self.app.pairing_code})
        self.assertEqual(status, 200)
        self.assertIn('HttpOnly', headers['Set-Cookie'])
        self.assertIn('SameSite=Strict', headers['Set-Cookie'])
        self.assertEqual(self.request('GET', '/api/camera.jpg')[0], 503)
        self.assertEqual(self.request('GET', '/api/camera/front.jpg')[0], 503)
        self.assertEqual(self.request('GET', '/api/map.png')[0], 503)

    def test_image_conversion_respects_rgb_padding(self):
        msg = SimpleNamespace(encoding='rgb8', width=2, height=2, step=8,
                              data=bytes([255, 0, 0, 0, 255, 0, 99, 99, 0, 0, 255, 255, 255, 255, 99, 99]))
        image = image_to_bgr(msg)
        self.assertEqual(image.shape, (2, 2, 3))
        self.assertEqual(image[0, 0].tolist(), [0, 0, 255])
        self.assertEqual(image[1, 0].tolist(), [255, 0, 0])
        msg.step = 2
        with self.assertRaises(ValueError):
            image_to_bgr(msg)

    def test_marker_yaml_validation(self):
        file = self.directory/'markers.yaml'
        file.write_text('markers:\n  - {id: 2, x: 1, y: 2, yaw: 0}\n')
        self.assertEqual(load_markers(file)[0]['id'], 2)
        self.assertEqual(load_markers(file)[0]['camera'], 'rear')
        file.write_text('markers:\n  - {id: 2, x: 1, y: 2, yaw: 0, camera: front}\n')
        self.assertEqual(load_markers(file)[0]['camera'], 'front')
        file.write_text('markers:\n  - {id: 2, x: 1, y: 2, yaw: 0, camera: side}\n')
        with self.assertRaises(ValueError):
            load_markers(file)
        file.write_text('markers:\n  - {id: 2, x: .nan, y: 2, yaw: 0}\n')
        with self.assertRaises(ValueError):
            load_markers(file)

    def test_start_uses_fixed_home_cancel_and_home_share_return_contract(self):
        self.ready()
        status, body, _ = self.request('POST', '/api/mission/start', {})
        self.assertEqual(status, 200)
        home = dict(self.app.delivery.homepoint)
        self.assertEqual(body['mission']['home'], home)
        self.app.pose = dict(home, x=4.)
        self.app.camera_at = 0
        self.app.aruco_available = False
        status, body, _ = self.request('POST', '/api/mission/cancel', {})
        self.assertEqual(status, 200)
        self.assertEqual(body['mission']['state'], 'RETURNING_HOME')
        self.assertTrue(body['mission']['returning_home'])
        self.assertTrue(body['health']['ready'], 'return must not depend on camera or ArUco')
        self.assertEqual(self.app.mission.commands[-1], ('goal', home))
        self.assertFalse(body['mission']['can_return_home'])
        self.assertEqual(self.request('POST', '/api/connection', {'domain_id': 216})[0], 400)
        self.assertEqual(self.request('POST', '/api/mission/pause', {})[0], 200)
        self.assertEqual(self.request('POST', '/api/mission/home', {})[0], 200)
        self.assertEqual(self.app.mission.home, home)

    def test_unready_home_stops_old_mission_and_requires_explicit_resume(self):
        self.ready()
        self.assertEqual(self.request('POST', '/api/mission/start', {})[0], 200)
        self.app.scan_at = 0
        status, body, _ = self.request('POST', '/api/mission/home', {})
        self.assertEqual(status, 400)
        self.assertIn('/scan', body['error'])
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertTrue(self.app.mission.returning_home)
        self.assertEqual(self.app.mission.commands, [('cancel', None), ('stop', None)])
        self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 400)
        self.app.scan_at = time.monotonic()
        self.app.camera_at = 0
        self.app.aruco_available = False
        self.assertEqual(self.request('POST', '/api/mission/resume', {})[0], 200)
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')

    def test_fixed_home_available_before_first_trip_and_estop_never_returns(self):
        self.ready()
        self.assertEqual(self.request('POST', '/api/mission/home', {})[0], 200)
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertEqual(self.app.mission.home, self.app.delivery.homepoint)
        self.request('POST', '/api/emergency-stop', {})
        for endpoint in ('/api/mission/home', '/api/mission/cancel'):
            self.assertEqual(self.request('POST', endpoint, {})[0], 400)
            self.assertEqual(self.app.mission.state, 'ESTOP')
            self.assertFalse(any(kind == 'goal' for kind, _ in self.app.mission.commands))

    def test_restart_promotes_pending_route_and_keeps_original_home(self):
        self.ready()
        self.request('POST', '/api/mission/start', {})
        home = dict(self.app.delivery.homepoint)
        self.request('POST', '/api/mission', {'route': [1, 3]})
        self.app.pose = dict(home, y=5.)
        self.assertEqual(self.request('POST', '/api/mission/restart', {})[0], 400)
        self.request('POST', '/api/mission/pause', {})
        status, body, _ = self.request('POST', '/api/mission/restart', {})
        self.assertEqual(status, 200)
        self.assertEqual(body['mission']['route'], [1, 3])
        self.assertEqual(body['mission']['home'], home)
        self.assertEqual([kind for kind, _ in self.app.mission.commands], ['cancel', 'stop', 'goal'])

    def test_home_state_requires_fresh_tf_even_when_camera_ready(self):
        self.ready()
        self.request('POST', '/api/mission/start', {})
        self.app.pose_at = time.monotonic()-5
        status, _, _ = self.request('POST', '/api/mission/cancel', {})
        self.assertEqual(status, 400)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertTrue(self.app.mission.returning_home)

    def test_fixed_home_is_restored_from_disk_without_starting_navigation(self):
        self.ready()
        self.request('POST', '/api/mission/start', {})
        restarted = MissionApplication(DEFAULT_MARKERS, 215, False, self.directory/'settings.json')
        self.assertEqual(restarted.mission.home, self.app.delivery.homepoint)
        self.assertTrue(restarted.mission.snapshot()['can_return_home'])
        self.assertEqual(restarted.mission.state, 'IDLE')
        self.assertFalse(restarted.mission.commands)


if __name__ == '__main__':
    unittest.main()
