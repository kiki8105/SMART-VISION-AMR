"""Exercise the real navigation wrapper's bounded lifecycle retry without ROS or motion."""

import json
import os
from pathlib import Path
import subprocess


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
WRAPPER = REPO / 'scripts/run_hardware_tuning.sh'


def run_retry(tmp_path, *, map_state, amcl_state, mode='success'):
    source = WRAPPER.read_text()
    retry_function = source.split('# BEGIN localization_startup_retry\n', 1)[1].split(
        '# END localization_startup_retry', 1)[0]
    assert 'localization_startup_retry 9>&- </dev/null' in source
    assert 'exec ros2 launch smart_vision_amr hardware.launch.py' in source

    state = tmp_path / 'state.json'
    calls = tmp_path / 'calls.txt'
    state.write_text(json.dumps({'map_server': map_state, 'amcl': amcl_state}))
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    ros2 = bin_dir / 'ros2'
    ros2.write_text('''#!/usr/bin/env python3
import json, os, pathlib, sys
args = sys.argv[1:]
state_path = pathlib.Path(os.environ['ROS2_STUB_STATE'])
data = json.loads(state_path.read_text())
if args[:2] == ['lifecycle', 'get']:
    print(data[args[2].lstrip('/')])
elif args[:2] == ['service', 'call']:
    with open(os.environ['ROS2_STUB_CALLS'], 'a') as record:
        record.write('manage_nodes\\n')
    if os.environ['ROS2_STUB_MODE'] == 'success':
        data.update(map_server='active [3]', amcl='active [3]')
        state_path.write_text(json.dumps(data))
        print('response: nav2_msgs.srv.ManageLifecycleNodes_Response(success=True)')
    else:
        print('response: nav2_msgs.srv.ManageLifecycleNodes_Response(success=False)')
else:
    raise SystemExit('unexpected ros2 call: ' + repr(args))
''')
    ros2.chmod(0o755)
    sleep = bin_dir / 'sleep'
    sleep.write_text('#!/usr/bin/env bash\nexit 0\n')
    sleep.chmod(0o755)
    env = os.environ.copy()
    env.update(PATH=f'{bin_dir}:{env["PATH"]}', ROS2_STUB_STATE=str(state),
               ROS2_STUB_CALLS=str(calls), ROS2_STUB_MODE=mode)
    result = subprocess.run(['bash', '-c', retry_function + '\nlocalization_startup_retry'],
                            env=env, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    return result, calls.read_text().splitlines() if calls.exists() else []


def test_already_active_never_calls_manage_nodes(tmp_path):
    result, calls = run_retry(tmp_path, map_state='active [3]', amcl_state='active [3]')
    assert calls == []
    assert 'STARTUP retry' not in result.stderr


def test_inactive_nodes_are_started_once_then_checked(tmp_path):
    result, calls = run_retry(tmp_path, map_state='inactive [2]', amcl_state='inactive [2]')
    assert calls == ['manage_nodes']
    assert 'STARTUP retry 1/2' in result.stderr
    assert 'active after retry' in result.stderr


def test_partial_transition_is_never_reset_or_restarted(tmp_path):
    result, calls = run_retry(tmp_path, map_state='active [3]', amcl_state='inactive [2]')
    assert calls == []
    assert 'without both nodes active' in result.stderr


def test_failed_startup_stops_after_two_attempts(tmp_path):
    result, calls = run_retry(tmp_path, map_state='unconfigured [1]',
                              amcl_state='unconfigured [1]', mode='reject')
    assert calls == ['manage_nodes', 'manage_nodes']
    assert 'without both nodes active' in result.stderr
