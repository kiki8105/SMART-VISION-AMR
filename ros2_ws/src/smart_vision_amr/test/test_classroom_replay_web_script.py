"""The isolated web launcher must not share mission settings across domains."""

import os
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory


SOURCE = Path(__file__).resolve().parents[4] / 'scripts' / 'run_classroom_replay_web.sh'


def test_replay_web_settings_default_tracks_simulation_domain():
    with TemporaryDirectory() as directory:
        root = Path(directory)
        scripts = root / 'scripts'
        scripts.mkdir()
        launcher = scripts / SOURCE.name
        shutil.copy2(SOURCE, launcher)
        stub = scripts / 'run_web_mission.sh'
        stub.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n', encoding='utf-8')
        stub.chmod(0o755)
        for relative in (
            'maps/classroom_20261030_cleaned.yaml',
            'maps/classroom_20261030_raw.yaml',
            'config/aruco_classroom_markers_20261030.yaml',
            'config/classroom_floorplan_20261030.yaml',
        ):
            path = root / 'ros2_ws/src/smart_vision_amr' / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture', encoding='utf-8')

        def command(domain, *, config=None):
            env = dict(os.environ, HOME=str(root / 'home'), SMART_VISION_CLASSROOM_SIM_DOMAIN=str(domain))
            env.pop('SMART_VISION_CLASSROOM_SIM_CONFIG', None)
            if config is not None:
                env['SMART_VISION_CLASSROOM_SIM_CONFIG'] = config
            result = subprocess.run(['bash', str(launcher)], env=env, capture_output=True,
                                    text=True, check=True)
            args = result.stdout.splitlines()
            return args[args.index('--config-file') + 1]

        assert command(76).endswith('/classroom-sim-76-20261030.json')
        assert command(176).endswith('/classroom-sim-176-20261030.json')
        assert command(176, config='/tmp/my-test-config.json') == '/tmp/my-test-config.json'
