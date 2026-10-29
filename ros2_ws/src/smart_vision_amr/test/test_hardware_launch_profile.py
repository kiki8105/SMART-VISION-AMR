"""Check the physical classroom's separate-process Nav2 launch wiring, without driving."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import PythonExpression
from launch.utilities import perform_substitutions


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
LAUNCH = PACKAGE/'launch/hardware.launch.py'


def description():
    spec = importlib.util.spec_from_file_location('hardware_launch_profile', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_generic_hardware_default_remains_composed():
    declaration = next(action for action in description().entities
                       if isinstance(action, DeclareLaunchArgument)
                       and action.name == 'use_composition')
    assert perform_substitutions(LaunchContext(), declaration.default_value) == 'true'


@pytest.mark.parametrize(('value', 'expected'), [
    ('true', 'True'), ('false', 'False'), ('True', 'True'), ('False', 'False'),
    ('1', 'True'), ('0', 'False'),
])
def test_composition_choice_reaches_the_nav2_include(value, expected):
    group = next(action for action in description().entities if isinstance(action, GroupAction))
    include = next(action for action in group.get_sub_entities()
                   if isinstance(action, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    context = LaunchContext()
    context.launch_configurations['use_composition'] = value
    normalized = arguments['use_composition'].perform(context)
    assert normalized == expected
    # This is the real Humble child-launch condition that fails for bare
    # lower-case "false". Checking --show-args alone does not execute it.
    assert PythonExpression(['not ', normalized]).perform(context) == str(expected == 'False')


def test_classroom_wrapper_defaults_to_ekf_and_preserves_overrides(tmp_path):
    # Exercise the real wrapper while replacing its downstream launcher with
    # an argument recorder. No ROS launch, service call, or publisher is started.
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    wrapper = scripts/'run_classroom_navigation_16.sh'
    shutil.copyfile(REPO/'scripts/run_classroom_navigation_16.sh', wrapper)
    downstream = scripts/'run_hardware_tuning.sh'
    downstream.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$TURTLEBOT3_TUNING_DOMAIN_ID" "$@"\n')
    downstream.chmod(0o755)
    result = subprocess.run(['bash', str(wrapper), 'use_rviz:=true'],
                            cwd=tmp_path, text=True, capture_output=True,
                            check=True, timeout=5)
    arguments = result.stdout.splitlines()
    assert arguments[0] == '16'
    assert 'use_composition:=false' in arguments
    assert 'use_motion_mux:=false' in arguments
    assert 'use_rviz:=true' in arguments
    assert arguments.count('localization_mode:=ekf_independent') == 1

    # Explicit rollback must not be overwritten by the new default. Passing
    # both modes to ros2 launch leaves the actual selected value ambiguous.
    rollback = subprocess.run(['bash', str(wrapper), 'localization_mode:=baseline',
                               'use_rviz:=true'],
                              cwd=tmp_path, text=True, capture_output=True,
                              check=True, timeout=5)
    rollback_arguments = rollback.stdout.splitlines()
    assert rollback_arguments.count('localization_mode:=baseline') == 1
    assert 'localization_mode:=ekf_independent' not in rollback_arguments
    assert 'use_rviz:=true' in rollback_arguments


def test_hardware_launch_exposes_composition_in_show_args():
    result = subprocess.run([sys.executable, '/opt/ros/humble/bin/ros2',
                             'launch', str(LAUNCH), '--show-args'],
                            text=True, capture_output=True, check=True, timeout=20)
    assert "'use_composition':" in result.stdout
    assert 'false uses separate processes' in result.stdout


def test_ekf_robot_wrapper_selects_independent_mode(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    shutil.copyfile(REPO/'scripts/robot_bringup_camera_16_ekf.sh',
                    scripts/'robot_bringup_camera_16_ekf.sh')
    baseline = scripts/'robot_bringup_camera_16.sh'
    baseline.write_text('#!/usr/bin/env bash\nprintf "%s\\n" '
                        '"$TURTLEBOT3_LOCALIZATION_MODE" "$CAMERA_BACKEND" "$@"\n')
    baseline.chmod(0o755)
    env = os.environ.copy()
    env.pop('CAMERA_BACKEND', None)
    result = subprocess.run(['bash', str(scripts/'robot_bringup_camera_16_ekf.sh'), 'test=1'],
                            text=True, capture_output=True, check=True, timeout=5, env=env)
    assert result.stdout.splitlines() == ['ekf_independent', 'rate_limited', 'test=1']
    env['CAMERA_BACKEND'] = 'v4l2'
    override = subprocess.run(['bash', str(scripts/'robot_bringup_camera_16_ekf.sh')],
                              text=True, capture_output=True, check=True, timeout=5, env=env)
    assert override.stdout.splitlines() == ['ekf_independent', 'v4l2']


def test_robot_bringup_defaults_to_local_runtime_and_legacy_pc_is_explicit(tmp_path):
    # Run only the initial validation path of the actual script. The copied
    # script deliberately lacks the EKF launch file and camera, so it cannot
    # start ROS or touch a physical device.
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    wrapper = scripts/'robot_bringup_camera_16.sh'
    shutil.copyfile(REPO/'scripts/robot_bringup_camera_16.sh', wrapper)
    env = os.environ.copy()
    env.pop('TURTLEBOT3_LOCALIZATION_MODE', None)
    env.pop('SMART_VISION_ESTIMATION_LOCATION', None)
    env['CAMERA_DEVICE'] = str(tmp_path/'missing_camera')
    default = subprocess.run(['bash', str(wrapper)], text=True,
                             capture_output=True, timeout=5, env=env)
    assert default.returncode == 2
    assert 'Camera device is missing' in default.stderr
    assert 'Independent EKF bringup is missing' not in default.stderr

    env['SMART_VISION_ESTIMATION_LOCATION'] = 'pc'
    env['SMART_VISION_DDS_TRANSPORT'] = 'default'
    legacy = subprocess.run(['bash', str(wrapper)], text=True,
                            capture_output=True, timeout=5, env=env)
    assert legacy.returncode == 2
    assert 'Independent EKF bringup is missing' in legacy.stderr

    env['TURTLEBOT3_LOCALIZATION_MODE'] = 'baseline'
    rollback = subprocess.run(['bash', str(wrapper)], text=True,
                              capture_output=True, timeout=5, env=env)
    assert rollback.returncode == 2
    assert 'Camera device is missing' in rollback.stderr
    assert 'Independent EKF bringup is missing' not in rollback.stderr


def test_explicit_robot_baseline_keeps_v4l2_camera_default(tmp_path):
    # A fake workspace setup puts a harmless ros2 stub first in PATH. The
    # baseline branch should request v4l2_camera, then stop before launch.
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    wrapper = scripts/'robot_bringup_camera_16.sh'
    shutil.copyfile(REPO/'scripts/robot_bringup_camera_16.sh', wrapper)
    workspace = tmp_path/'robot_ws'
    install = workspace/'install'
    install.mkdir(parents=True)
    bin_dir = tmp_path/'bin'
    bin_dir.mkdir()
    (install/'setup.bash').write_text(f'export PATH="{bin_dir}:$PATH"\n')
    ros2 = bin_dir/'ros2'
    ros2.write_text('#!/usr/bin/env bash\n'
                    'if [[ "$*" == "pkg prefix turtlebot3_bringup" ]]; then exit 0; fi\n'
                    'if [[ "$*" == "pkg prefix v4l2_camera" ]]; then exit 1; fi\n'
                    'exit 99\n')
    ros2.chmod(0o755)
    camera = tmp_path/'fake_camera'
    camera.touch()
    env = os.environ.copy()
    env['TURTLEBOT3_LOCALIZATION_MODE'] = 'baseline'
    env['SMART_VISION_ESTIMATION_LOCATION'] = 'pc'
    env['SMART_VISION_DDS_TRANSPORT'] = 'default'
    env.pop('CAMERA_BACKEND', None)
    env['TURTLEBOT3_WORKSPACE'] = str(workspace)
    env['CAMERA_DEVICE'] = str(camera)
    result = subprocess.run(['bash', str(wrapper)], text=True,
                            capture_output=True, timeout=10, env=env)
    assert result.returncode == 2
    assert 'Install the camera driver' in result.stderr
    assert 'Rate-limited camera node is missing' not in result.stderr


def test_baseline_robot_wrapper_forces_stock_mode(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    shutil.copyfile(REPO/'scripts/robot_bringup_camera_16_baseline.sh',
                    scripts/'robot_bringup_camera_16_baseline.sh')
    base = scripts/'robot_bringup_camera_16.sh'
    base.write_text('#!/usr/bin/env bash\nprintf "%s\\n" '
                    '"$TURTLEBOT3_LOCALIZATION_MODE" "$@"\n')
    base.chmod(0o755)
    env = os.environ.copy()
    env['TURTLEBOT3_LOCALIZATION_MODE'] = 'ekf_independent'
    result = subprocess.run(['bash', str(scripts/'robot_bringup_camera_16_baseline.sh'),
                             'test=1'], text=True, capture_output=True,
                            check=True, timeout=5, env=env)
    assert result.stdout.splitlines() == ['baseline', 'test=1']


def test_ekf_pc_wrapper_passes_localization_mode_and_user_options(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    shutil.copyfile(REPO/'scripts/run_classroom_navigation_16_ekf.sh',
                    scripts/'run_classroom_navigation_16_ekf.sh')
    baseline = scripts/'run_classroom_navigation_16.sh'
    baseline.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    baseline.chmod(0o755)
    result = subprocess.run(['bash', str(scripts/'run_classroom_navigation_16_ekf.sh'),
                             'use_rviz:=true'], text=True, capture_output=True,
                            check=True, timeout=5)
    assert result.stdout.splitlines() == ['use_rviz:=true',
                                          'localization_mode:=ekf_independent']


def test_baseline_pc_wrapper_forces_stock_mode(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    shutil.copyfile(REPO/'scripts/run_classroom_navigation_16_baseline.sh',
                    scripts/'run_classroom_navigation_16_baseline.sh')
    base = scripts/'run_classroom_navigation_16.sh'
    base.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    base.chmod(0o755)
    result = subprocess.run(['bash', str(scripts/'run_classroom_navigation_16_baseline.sh'),
                             'use_rviz:=true'], text=True, capture_output=True,
                            check=True, timeout=5)
    assert result.stdout.splitlines() == ['use_rviz:=true',
                                          'localization_mode:=baseline']
