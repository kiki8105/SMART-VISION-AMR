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


def test_classroom_wrapper_uses_separate_processes_and_preserves_overrides(tmp_path):
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
    assert arguments[-1] == 'use_rviz:=true'


def test_hardware_launch_exposes_composition_in_show_args():
    result = subprocess.run([sys.executable, '/opt/ros/humble/bin/ros2',
                             'launch', str(LAUNCH), '--show-args'],
                            text=True, capture_output=True, check=True, timeout=20)
    assert "'use_composition':" in result.stdout
    assert 'false uses separate processes' in result.stdout


def test_ekf_robot_wrapper_selects_independent_mode_without_replacing_baseline(tmp_path):
    # The Pi wrapper selects the EKF profile only for this process. The old
    # executable remains callable and defaults to its original baseline mode.
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
