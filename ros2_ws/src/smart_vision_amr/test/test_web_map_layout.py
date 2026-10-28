"""Vector floorplan HTTP endpoint keeps the operator/map-image access boundary."""
import http.client
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

import numpy as np

from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, MissionHTTPServer, MissionHTTPRequestHandler,
)


class MapLayoutHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.app = MissionApplication(DEFAULT_MARKERS, 16, False, self.directory/'settings.json')
        self.server = MissionHTTPServer(('127.0.0.1', 0), self.app, self.directory)
        self.thread = threading.Thread(target=lambda: self.server.serve_forever(poll_interval=.01), daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)
        self.temporary.cleanup()

    def request(self, path, cookie=None):
        client = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        client.request('GET', path, headers={'Cookie': cookie} if cookie else {})
        response = client.getresponse()
        result = response.status, json.loads(response.read())
        client.close()
        return result

    def install_map(self):
        metadata = {'width': 2, 'height': 2, 'resolution': .05,
                    'origin': {'x': -1.53, 'y': -5.69, 'yaw': 0.}, 'frame_id': 'map'}
        self.app.receive_map(metadata, np.array([[0, -1], [100, 0]], np.int16))

    def test_missing_map_returns_503_then_versioned_layout_matches_state(self):
        self.assertEqual(self.request('/api/map-layout')[0], 503)
        self.install_map()
        status, layout = self.request('/api/map-layout')
        self.assertEqual(status, 200)
        self.assertEqual(layout['version'], self.request('/api/state')[1]['map']['version'])
        self.assertEqual((layout['width'], layout['height']), (2, 2))
        self.assertEqual(len(layout['free_paths']), 2)
        self.assertEqual(len(layout['occupied_paths']), 1)

    def test_remote_inventory_and_unauthenticated_clients_cannot_read_operator_layout(self):
        self.install_map()
        inventory = self.app.pair_inventory('fixture', 1, self.app.inventory_join_code)
        operator = self.app.pair('fixture', self.app.pairing_code)
        with patch.object(MissionHTTPRequestHandler, 'local', property(lambda _: False)):
            self.assertEqual(self.request('/api/map-layout')[0], 401)
            self.assertEqual(self.request('/api/map-layout', f'amr_inventory_session={inventory}')[0], 401)
            self.assertEqual(self.request('/api/map-layout', f'amr_session={operator}')[0], 200)


if __name__ == '__main__':
    unittest.main()
