"""PC setup shell regressions; apt, colcon and ROS commands are stubbed.

Only the installed Humble environment and Python dependency imports are read.
No ROS context, node, robot connection, package installation or build is made.
"""
import os
from pathlib import Path
import shutil
import subprocess


SOURCE = Path(__file__).resolve().parents[4]/'scripts/setup_pc_runtime.sh'


def setup_case(tmp_path):
    workspace = tmp_path/'default workspace'
    scripts = workspace/'src/SMART-VISION-AMR/scripts'
    scripts.mkdir(parents=True)
    script = scripts/'setup_pc_runtime.sh'
    shutil.copy2(SOURCE, script)
    log = tmp_path/'commands.log'
    functions = tmp_path/'stub-functions.bash'
    functions.write_text('''ros2() {
  printf 'ros2:%s %s %s;ament:%s;python:%s\n' "$1" "$2" "$3" "${AMENT_PREFIX_PATH:-}" "${PYTHONPATH:-}" >> "$TEST_COMMAND_LOG"
  if [[ "$1 $2" != 'pkg prefix' ]]; then
    printf 'Unexpected live ROS operation\n' >&2; return 90
  fi
  if [[ "$3" == "${TEST_MISSING_PACKAGE:-}" ]]; then return 1; fi
  case ":${AMENT_PREFIX_PATH:-}:" in
    *":$TEST_DEPENDENCY_ORIGIN:"*) printf '%s/%s\n' "$TEST_DEPENDENCY_ORIGIN" "$3" ;;
    *) return 1 ;;
  esac
}
sudo() { printf 'FORBIDDEN:sudo\n' >> "$TEST_COMMAND_LOG"; return 91; }
colcon() { printf 'FORBIDDEN:colcon\n' >> "$TEST_COMMAND_LOG"; return 92; }
''')
    environment = dict(os.environ, BASH_ENV=str(functions), TEST_COMMAND_LOG=str(log),
                       TEST_DEPENDENCY_ORIGIN=str(workspace/'install'))
    return script, workspace, log, environment


def overlay(workspace):
    install = workspace/'install'
    install.mkdir(parents=True)
    (install/'local_setup.bash').write_text(
        'export AMENT_PREFIX_PATH="'+str(install)+':${AMENT_PREFIX_PATH:-}"\n'
        'printf "sourced:%s\\n" "'+str(install)+'" >> "$TEST_COMMAND_LOG"\n')


def run(script, environment, *arguments):
    return subprocess.run([str(script), '--no-install', '--no-build', *map(str, arguments)],
                          env=environment, capture_output=True, text=True, timeout=15)


def assert_no_mutating_commands(log):
    content = log.read_text()
    assert 'FORBIDDEN:' not in content
    assert all('pkg prefix' in line for line in content.splitlines() if line.startswith('ros2:'))
    return content


def test_pc_setup_finds_dependencies_from_selected_default_overlay(tmp_path):
    script, workspace, log, environment = setup_case(tmp_path)
    overlay(workspace)
    result = run(script, environment)
    assert result.returncode == 0, result.stderr
    assert f'Physical PC workspace prepared: {workspace}' in result.stdout
    commands = assert_no_mutating_commands(log)
    assert f'sourced:{workspace}/install' in commands
    for name in ('nav2_bringup', 'turtlebot3_navigation2', 'robot_localization',
                 'compressed_image_transport'):
        assert 'ros2:pkg prefix '+name in commands


def test_pc_setup_reports_genuinely_missing_dependency_in_selected_workspace(tmp_path):
    script, workspace, log, environment = setup_case(tmp_path)
    overlay(workspace)
    environment['TEST_MISSING_PACKAGE'] = 'robot_localization'
    result = run(script, environment)
    assert result.returncode == 2
    assert 'robot_localization' in result.stderr
    assert str(workspace) in result.stderr
    assert 'Physical PC workspace prepared' not in result.stdout
    commands = assert_no_mutating_commands(log)
    assert 'ros2:pkg prefix turtlebot3_navigation2' in commands


def test_pc_setup_cannot_satisfy_dependencies_from_an_unrelated_inherited_overlay(tmp_path):
    script, workspace, log, environment = setup_case(tmp_path)
    foreign = tmp_path/'unrelated install'
    environment['TEST_DEPENDENCY_ORIGIN'] = str(foreign)
    for name in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                 'ROS_PACKAGE_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH'):
        environment[name] = str(foreign)
    result = run(script, environment)
    assert result.returncode == 2
    assert 'nav2_bringup' in result.stderr
    assert 'Physical PC workspace prepared' not in result.stdout
    commands = assert_no_mutating_commands(log)
    assert str(foreign) not in commands


def test_pc_setup_honors_and_canonicalizes_explicit_workspace(tmp_path):
    script, default_workspace, log, environment = setup_case(tmp_path)
    overlay(default_workspace)
    selected = tmp_path/'custom workspace'
    overlay(selected)
    environment['TEST_DEPENDENCY_ORIGIN'] = str(selected/'install')
    result = run(script, environment, selected/'..'/selected.name)
    assert result.returncode == 0, result.stderr
    assert f'Physical PC workspace prepared: {selected}' in result.stdout
    commands = assert_no_mutating_commands(log)
    assert f'sourced:{selected}/install' in commands
    assert f'sourced:{default_workspace}/install' not in commands
