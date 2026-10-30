"""The mapping controller waits for its own web server and cleans it up."""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import unittest


WRAPPER = Path(__file__).resolve().parents[1] / 'scripts/run_classroom_mapping_controller_16.sh'


def _free_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def _write_executable(path, content):
    path.write_text(content)
    path.chmod(0o755)


class MappingControllerAutostartTests(unittest.TestCase):
    def test_waits_for_web_readiness_then_stops_only_its_child(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / 'workspace'
            repo = workspace / 'src' / 'SMART-VISION-AMR'
            scripts = repo / 'scripts'
            tools = repo / 'tools'
            scripts.mkdir(parents=True)
            tools.mkdir()
            (workspace / 'install').mkdir()
            (workspace / 'install/local_setup.bash').write_text('# test overlay\n')
            shutil.copy2(WRAPPER, scripts / WRAPPER.name)
            (scripts / 'dds_env.bash').write_text('# test DDS profile\n')
            _write_executable(scripts / 'run_classroom_web_16.sh', '''#!/usr/bin/env bash
exec /usr/bin/python3 - "$SMART_VISION_WEB_PORT" <<'PY'
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import sys
import time
started = time.monotonic()
class Handler(BaseHTTPRequestHandler):
    server_version = 'SmartVisionMission/1.0'
    def do_GET(self):
        ready = time.monotonic() - started > .8
        body = json.dumps({'connection': {'domain_id': 16, 'use_sim_time': False,
             'access_mode': 'domain'}, 'manual': {}, 'safety': {}}).encode()
        self.send_response(200 if ready else 503)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(body)
    def log_message(self, *_):
        pass
HTTPServer(('127.0.0.1', int(sys.argv[1])), Handler).serve_forever()
PY
''')
            _write_executable(scripts / 'run_classroom_controller_16.sh',
                              '#!/usr/bin/env bash\nsleep .4\n')
            (tools / 'controller_manual_bridge.py').write_text(
                'import time\nwhile True:\n    time.sleep(.1)\n')
            port = _free_port()
            environment = os.environ.copy()
            environment.update(SMART_VISION_WEB_PORT=str(port),
                               SMART_VISION_ESTIMATION_LOCATION='robot')
            result = subprocess.run([str(scripts / WRAPPER.name)], env=environment,
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('web server started', result.stdout)
            with self.assertRaises(OSError):
                socket.create_connection(('127.0.0.1', port), timeout=.2)


if __name__ == '__main__':
    unittest.main()
