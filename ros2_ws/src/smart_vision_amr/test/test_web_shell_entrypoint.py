"""The physical web wrapper must start from a minimal built workspace."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[4]
SCRIPTS = REPO / 'scripts'


def _workspace(tmp_path):
    install = tmp_path / 'install'
    install.mkdir()
    binary = tmp_path / 'bin'
    binary.mkdir()
    ros2 = binary / 'ros2'
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1 $2" == "pkg prefix" ]]; then
  if [[ "$3" != smart_vision_amr && "${TEST_ALLOW_SIM:-false}" != true ]]; then
    printf 'unexpected simulator dependency %s\\n' "$3" >&2
    exit 4
  fi
  printf '%s\\n' "$TEST_INSTALL"
elif [[ "$1 $2 $3" == "run smart_vision_amr web_mission_server" ]]; then
  printf 'web-server-started\\n'
  printf 'server-arg:%s\\n' "${@:4}"
else
  exit 3
fi
''')
    ros2.chmod(0o755)
    (install / 'local_setup.bash').write_text(
        'export PATH="$TEST_BIN:$PATH"\n')
    executable = install / 'lib/smart_vision_amr/web_mission_server'
    executable.parent.mkdir(parents=True)
    executable.write_text('#!/bin/sh\nexit 0\n')
    executable.chmod(0o755)
    environment = dict(os.environ, TEST_INSTALL=str(install), TEST_BIN=str(binary))
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                '_SMART_VISION_DDS_OWN_PROFILE'):
        environment.pop(key, None)
    return environment


def test_sim_environment_uses_default_dds_profile_with_workspace_argument(tmp_path):
    environment = _workspace(tmp_path)
    environment['TEST_ALLOW_SIM'] = 'true'
    result = subprocess.run(
        ['bash', '-c', 'source "$1" "$2"', 'test',
         str(SCRIPTS / 'sim_env.bash'), str(tmp_path)],
        env=environment, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr


def test_physical_web_starts_without_gazebo_and_forwards_server_arguments(tmp_path):
    environment = _workspace(tmp_path)
    markers = REPO / 'ros2_ws/src/smart_vision_amr/config/aruco_mission_markers.yaml'
    result = subprocess.run(
        [str(SCRIPTS / 'run_web_mission.sh'), '--workspace', str(tmp_path),
         '--domain-id', '16', '--no-use-sim-time', '--markers-file', str(markers),
         '--', '--map-file', '/tmp/test-map.yaml'],
        env=environment, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert 'Web mission: domain=16 localhost_only=0 use_sim_time=false' in result.stdout
    assert 'web-server-started' in result.stdout
    assert 'server-arg:--map-file\nserver-arg:/tmp/test-map.yaml' in result.stdout


def test_physical_navigation_accepts_launch_arguments_without_dds_profile_confusion(tmp_path):
    workspace = tmp_path / 'Team2_workspace'
    scripts = workspace / 'src/SMART-VISION-AMR/scripts'
    scripts.mkdir(parents=True)
    for name in ('run_hardware_tuning.sh', 'dds_env.bash'):
        shutil.copy2(SCRIPTS / name, scripts / name)
    (scripts / 'check_map_publishers.py').write_text('pass\n')
    profile = workspace / 'src/SMART-VISION-AMR/ros2_ws/src/smart_vision_amr/config/dds_udp.xml'
    profile.parent.mkdir(parents=True)
    shutil.copy2(REPO / 'ros2_ws/src/smart_vision_amr/config/dds_udp.xml', profile)
    environment = _workspace(workspace)
    ros2 = Path(environment['TEST_BIN']) / 'ros2'
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1 $2" == "node list" ]]; then
  exit 0
elif [[ "$1 $2" == "launch smart_vision_amr" ]]; then
  printf 'navigation-launch-started\\n'
else
  exit 3
fi
''')
    ros2.chmod(0o755)
    map_file = tmp_path / 'map.yaml'
    params_file = tmp_path / 'params.yaml'
    map_file.write_text('image: map.pgm\n')
    params_file.write_text('{}\n')
    environment['TURTLEBOT3_TUNING_PARAMS_FILE'] = str(params_file)
    environment['TURTLEBOT3_TUNING_DOMAIN_ID'] = '76'
    result = subprocess.run(
        [str(scripts / 'run_hardware_tuning.sh'), str(map_file),
         'nodes_file:=test.yaml', 'localization_mode:=baseline',
         'estimation_location:=pc'],
        env=environment, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert 'navigation-launch-started' in result.stdout


@pytest.mark.parametrize('estimation,mode,timing', [
    ('robot', 'ekf_independent', 'false'),
    ('robot', 'ekf_independent', 'true'),
    ('pc', 'baseline', 'true'),
])
def test_robot_bringup_omits_unset_wheel_geometry_launch_arguments(tmp_path, estimation, mode, timing):
    environment = _workspace(tmp_path)
    stock = tmp_path / 'stock'
    (stock / 'install').mkdir(parents=True)
    (stock / 'install/setup.bash').write_text('export PATH="$TEST_BIN:$PATH"\n')
    prefix = tmp_path / 'runtime_prefix'
    (prefix / 'share/smart_vision_amr/launch').mkdir(parents=True)
    (prefix / 'share/smart_vision_amr/launch/hardware_robot_runtime.launch.py').touch()
    for name in ('motion_mux', 'localization_health'):
        executable = prefix / 'lib/smart_vision_amr' / name
        executable.parent.mkdir(parents=True, exist_ok=True)
        executable.write_text('#!/bin/sh\nexit 0\n')
        executable.chmod(0o755)
    device = tmp_path / 'video0'
    device.touch()
    log = tmp_path / 'launch-args'
    ros2 = Path(environment['TEST_BIN']) / 'ros2'
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1 $2" == "pkg prefix" ]]; then
  printf '%s\\n' "$TEST_RUNTIME_PREFIX"
elif [[ "$1" == "launch" ]]; then
  printf '%s\\n' "$@" > "$TEST_LAUNCH_LOG"
  for argument in "$@"; do
    if [[ "$argument" == *':=' ]]; then
      printf 'malformed launch argument %s\\n' "$argument" >&2
      exit 2
    fi
  done
  sleep 0.4
elif [[ "$1" == "run" ]]; then
  sleep 3
else
  exit 3
fi
''')
    ros2.chmod(0o755)
    environment.update({
        'WHEEL_RADIUS': '',
        'WHEEL_SEPARATION': '',
        'TURTLEBOT3_WORKSPACE': str(stock),
        'SMART_VISION_RUNTIME_WORKSPACE': str(tmp_path),
        'TEST_RUNTIME_PREFIX': str(prefix),
        'TEST_LAUNCH_LOG': str(log),
        'CAMERA_DEVICE': str(device),
        'OPENCR_DEVICE': str(device),
        'CAMERA_BACKEND': 'v4l2',
        'ENABLE_FRONT_CAMERA': 'false',
        'SMART_VISION_DDS_TRANSPORT': 'default',
        'SMART_VISION_ESTIMATION_LOCATION': estimation,
        'TURTLEBOT3_LOCALIZATION_MODE': mode,
        'SMART_VISION_TIMING_DIAGNOSTICS': timing,
    })
    result = subprocess.run(
        [str(SCRIPTS / 'robot_bringup_camera_16.sh')],
        env=environment, text=True, capture_output=True, timeout=8,
    )
    assert 'malformed launch argument' not in result.stderr
    assert result.returncode == 1  # The fake launch exits, so the wrapper stops its children.
    assert 'wheel_radius:=' not in log.read_text()
    assert 'wheel_separation:=' not in log.read_text()
    if estimation == 'robot':
        assert f'timing_diagnostics:={timing}' in log.read_text()
    else:
        assert 'timing_diagnostics:=' not in log.read_text()


def test_invalid_timing_flag_is_rejected_before_hardware_checks():
    result = subprocess.run(
        ['bash', str(SCRIPTS/'robot_bringup_camera_16.sh')],
        env={**os.environ, 'SMART_VISION_TIMING_DIAGNOSTICS': 'invalid'},
        text=True, capture_output=True, timeout=3)
    assert result.returncode == 2
    assert 'SMART_VISION_TIMING_DIAGNOSTICS must be true or false' in result.stderr


def test_static_deployment_checks_selected_overlay_without_live_nodes(tmp_path):
    environment = _workspace(tmp_path)
    prefix = tmp_path / 'install'
    for name in ('path_follower', 'astar_planner', 'command_stamper'):
        executable = prefix / 'lib/smart_vision_amr' / name
        executable.write_text('#!/bin/sh\nexit 0\n')
        executable.chmod(0o755)
    ros2 = Path(environment['TEST_BIN']) / 'ros2'
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1 $2" == "pkg prefix" ]]; then
  printf '%s\\n' "$TEST_INSTALL"
else
  printf 'Unexpected live ROS operation\\n' >&2
  exit 3
fi
''')
    ros2.chmod(0o755)
    result = subprocess.run(
        [str(SCRIPTS / 'check_classroom_deployment.sh'), '--workspace', str(tmp_path)],
        env=environment, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert 'ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0' in result.stdout
    assert 'Static checks passed' in result.stdout
    environment['TEST_INSTALL'] = str(tmp_path / 'foreign/install')
    result = subprocess.run(
        [str(SCRIPTS / 'check_classroom_deployment.sh'), '--workspace', str(tmp_path)],
        env=environment, text=True, capture_output=True, timeout=15,
    )
    assert result.returncode == 2
    assert 'Wrong smart_vision_amr overlay' in result.stderr
