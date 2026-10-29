"""Persistence checks independent of the Qt dashboard and ROS service transport."""

import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

import parameter_defaults as defaults
from parameter_defaults import (
    ParameterDefaultsError, export_parameters, is_persistent_parameter,
    save_parameter_defaults,
)


CONTROLLER = '''# Shared explanation must survive.
controller_server:
  ros__parameters:
    controller_frequency: 10.0 # Keep timing comment.
    FollowPath:
      max_vel_x: 0.22 # Speed limit.
      BaseObstacle.scale: 0.02 # Literal dotted key.
      critics: [RotateToGoal, BaseObstacle]
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.22
      inflation_layer:
        inflation_radius: 0.40
'''
NODES = '''# Simulation-only team settings.
/astar_planner:
  ros__parameters:
    occupied_threshold: 65
    obstacle_inflation_cells: 6 # Initial clearance.
/path_follower:
  ros__parameters:
    controller_id: FollowPath
'''


class ParameterDefaultsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.navigation = self.write('simulation.yaml', CONTROLLER)
        self.nodes = self.write('simulation_nodes.yaml', NODES)

    def tearDown(self):
        self.directory.cleanup()

    def write(self, name, text):
        path = self.root / name
        path.write_bytes(text.encode('utf-8'))
        return path

    def backups(self, path):
        return list(path.parent.glob(path.name + '.bak-*'))

    def test_save_both_launch_sources_preserves_unrelated_settings_and_comments(self):
        original = yaml.safe_load(CONTROLLER)
        result = save_parameter_defaults([self.navigation, self.nodes], {
            ('/controller_server', 'FollowPath.max_vel_x'): 0.13,
            ('/astar_planner', 'obstacle_inflation_cells'): 4,
            ('/local_costmap/local_costmap', 'inflation_layer.inflation_radius'): 0.31,
        })
        self.assertEqual(result, [str(self.navigation.resolve()), str(self.nodes.resolve())])
        nav = yaml.safe_load(self.navigation.read_text())
        self.assertEqual(nav['controller_server']['ros__parameters']['FollowPath']['max_vel_x'], 0.13)
        self.assertEqual(nav['controller_server']['ros__parameters']['controller_frequency'], 10.0)
        self.assertEqual(nav['controller_server']['ros__parameters']['FollowPath']['critics'],
                         original['controller_server']['ros__parameters']['FollowPath']['critics'])
        self.assertIn('# Shared explanation must survive.', self.navigation.read_text())
        self.assertIn('# Speed limit.', self.navigation.read_text())
        self.assertIn('# Initial clearance.', self.nodes.read_text())
        self.assertEqual(self.backups(self.navigation)[0].read_text(), CONTROLLER)
        self.assertEqual(self.backups(self.nodes)[0].read_text(), NODES)

    def test_existing_literal_dotted_alias_is_updated_in_place(self):
        save_parameter_defaults([self.navigation], {
            ('/controller_server', 'FollowPath.BaseObstacle.scale'): 0.15,
        })
        text = self.navigation.read_text()
        follow = yaml.safe_load(text)['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(follow['BaseObstacle.scale'], 0.15)
        self.assertNotIn('BaseObstacle', follow)
        self.assertIn('BaseObstacle.scale: 0.15 # Literal dotted key.', text)

    def test_fully_qualified_nested_and_slash_node_keys_match_exactly(self):
        path = self.write('robots.yaml', '''/robot1/controller_server:
  ros__parameters: {speed: 0.1}
robot2:
  controller_server:
    ros__parameters:
      speed: 0.2
''')
        save_parameter_defaults([path], {
            ('/robot1/controller_server', 'speed'): 0.11,
            ('/robot2/controller_server', 'speed'): 0.21,
        })
        data = yaml.safe_load(path.read_text())
        self.assertEqual(data['/robot1/controller_server']['ros__parameters']['speed'], 0.11)
        self.assertEqual(data['robot2']['controller_server']['ros__parameters']['speed'], 0.21)
        with self.assertRaisesRegex(ParameterDefaultsError, '정확히 하나'):
            save_parameter_defaults([path], {('/controller_server', 'speed'): 0.4})

    def test_missing_parameters_are_added_as_flat_names_without_losing_comments(self):
        path = self.write('minimal.yaml', 'n:\n  ros__parameters:\n    old: 1 # untouched')
        save_parameter_defaults([path], {('/n', 'planner.new_limit'): 1.5})
        text = path.read_text()
        data = yaml.safe_load(text)['n']['ros__parameters']
        self.assertEqual(data, {'old': 1, 'planner.new_limit': 1.5})
        self.assertIn('old: 1 # untouched', text)

    def test_missing_parameters_in_flow_and_empty_mappings(self):
        path = self.write('flow.yaml', 'n:\n  ros__parameters: {} # keep\nm:\n  ros__parameters: {old: 2}\n')
        save_parameter_defaults([path], {('/n', 'new'): ['true', 'a#b'], ('/m', 'new'): True})
        data = yaml.safe_load(path.read_text())
        self.assertEqual(data['n']['ros__parameters']['new'], ['true', 'a#b'])
        self.assertEqual(data['m']['ros__parameters'], {'old': 2, 'new': True})
        self.assertIn('# keep', path.read_text())

    def test_changing_block_array_keeps_item_and_adjacent_comments(self):
        path = self.write('array.yaml', '''n:
  ros__parameters:
    values:
      - "a#inside" # first explanation
      # List explanation.
      - old
    other: 9 # adjacent
''')
        save_parameter_defaults([path], {('/n', 'values'): ['new', 'next']})
        text = path.read_text()
        self.assertEqual(yaml.safe_load(text)['n']['ros__parameters'], {'values': ['new', 'next'], 'other': 9})
        self.assertIn('# first explanation', text)
        self.assertIn('# List explanation.', text)
        self.assertIn('other: 9 # adjacent', text)
        self.assertNotIn('a#inside', text)

    def test_multiline_string_becomes_quoted_without_damaging_next_parameter(self):
        path = self.write('string.yaml', 'n:\n  ros__parameters:\n    value: |\n      old\n      second\n    next: true\n')
        save_parameter_defaults([path], {('/n', 'value'): 'new\n# text inside string'})
        data = yaml.safe_load(path.read_text())['n']['ros__parameters']
        self.assertEqual(data, {'value': 'new\n# text inside string', 'next': True})

    def test_canonical_symlink_source_updates_real_file_and_keeps_links(self):
        installed = self.root / 'installed.yaml'
        build = self.root / 'build.yaml'
        build.symlink_to(self.navigation)
        installed.symlink_to(build)
        result = save_parameter_defaults([installed, self.navigation], {
            ('/controller_server', 'FollowPath.max_vel_x'): 0.12,
        })
        self.assertEqual(result, [str(self.navigation.resolve())])
        self.assertTrue(installed.is_symlink())
        self.assertTrue(build.is_symlink())
        self.assertIn('max_vel_x: 0.12', installed.read_text())
        self.assertEqual(len(self.backups(self.navigation)), 1)
        self.assertEqual(self.backups(installed), [])

    def test_runtime_or_launch_managed_parameters_are_rejected_before_writing(self):
        for name in ('drive_enabled', 'autostart', 'use_sim_time', 'yaml_filename',
                     'set_initial_pose', 'initial_pose.x', 'initial_pose.yaw',
                     'memory_enabled', 'memory_path'):
            with self.subTest(name=name):
                self.assertFalse(is_persistent_parameter(name))
                with self.assertRaisesRegex(ParameterDefaultsError, 'launch'):
                    save_parameter_defaults([self.navigation], {('/controller_server', name): True})
                self.assertEqual(self.navigation.read_text(), CONTROLLER)
                self.assertEqual(self.backups(self.navigation), [])
        self.assertTrue(is_persistent_parameter('FollowPath.max_vel_x'))

    def test_duplicate_dotted_aliases_are_rejected(self):
        path = self.write('duplicate.yaml', '''n:
  ros__parameters:
    group.value: 1
    group:
      value: 2
''')
        with self.assertRaisesRegex(ParameterDefaultsError, '중복 dotted'):
            save_parameter_defaults([path], {('/n', 'group.value'): 3})
        self.assertEqual(self.backups(path), [])

    def test_duplicate_yaml_keys_and_node_aliases_are_rejected(self):
        for text in ('n:\n  ros__parameters: {value: 1, value: 2}\n',
                     '/r/n:\n  ros__parameters: {v: 1}\nr:\n  n:\n    ros__parameters: {v: 2}\n'):
            with self.subTest(text=text):
                path = self.write('duplicate.yaml', text)
                with self.assertRaises(ParameterDefaultsError):
                    save_parameter_defaults([path], {('/n', 'v'): 3})
                self.assertEqual(path.read_text(), text)

    def test_ambiguous_source_files_and_missing_nodes_abort_entire_save(self):
        duplicate = self.write('duplicate.yaml', CONTROLLER)
        for paths, changes in (
                ([self.navigation, duplicate], {('/controller_server', 'FollowPath.max_vel_x'): 0.1}),
                ([self.navigation, self.nodes], {
                    ('/controller_server', 'FollowPath.max_vel_x'): 0.1,
                    ('/not_launched', 'speed'): 0.1,
                })):
            with self.subTest(paths=paths):
                with self.assertRaises(ParameterDefaultsError):
                    save_parameter_defaults(paths, changes)
                self.assertEqual(self.navigation.read_text(), CONTROLLER)
                self.assertEqual(self.nodes.read_text(), NODES)
                self.assertEqual(list(self.root.glob('*.bak-*')), [])

    def test_invalid_second_file_is_checked_before_any_backup_or_write(self):
        broken = self.write('invalid.yaml', 'n: [unclosed\n')
        with self.assertRaises(ParameterDefaultsError):
            save_parameter_defaults([self.navigation, broken], {
                ('/controller_server', 'FollowPath.max_vel_x'): 0.1,
            })
        self.assertEqual(self.navigation.read_text(), CONTROLLER)
        self.assertEqual(self.backups(self.navigation), [])

    def test_anchors_are_rejected_instead_of_changing_unrelated_aliased_values(self):
        path = self.write('anchors.yaml', 'n:\n  ros__parameters:\n    a: &shared 1\n    b: *shared\n')
        with self.assertRaisesRegex(ParameterDefaultsError, 'anchor/alias'):
            save_parameter_defaults([path], {('/n', 'a'): 2})

    def test_second_replace_failure_rolls_back_first_and_keeps_backups(self):
        original_replace = os.replace
        replacements = []

        def replace(source, target):
            replacements.append(Path(target))
            if len(replacements) == 2:
                raise OSError('injected second-file failure')
            return original_replace(source, target)

        with patch.object(defaults.os, 'replace', side_effect=replace):
            with self.assertRaisesRegex(ParameterDefaultsError, '원본 파일을 유지/복구'):
                save_parameter_defaults([self.navigation, self.nodes], {
                    ('/controller_server', 'FollowPath.max_vel_x'): 0.11,
                    ('/astar_planner', 'obstacle_inflation_cells'): 3,
                })
        self.assertEqual(self.navigation.read_text(), CONTROLLER)
        self.assertEqual(self.nodes.read_text(), NODES)
        self.assertEqual(replacements, [self.navigation, self.nodes, self.navigation])
        self.assertEqual(len(list(self.root.glob('*.bak-*'))), 2)
        self.assertEqual(list(self.root.glob('.*.tmp')), [])

    def test_backup_failure_happens_before_any_source_replacement(self):
        original_backup = defaults._backup

        def backup(config):
            if config.path == self.nodes:
                raise OSError('backup directory failure')
            return original_backup(config)

        with patch.object(defaults, '_backup', side_effect=backup):
            with self.assertRaises(ParameterDefaultsError):
                save_parameter_defaults([self.navigation, self.nodes], {
                    ('/controller_server', 'FollowPath.max_vel_x'): 0.11,
                    ('/astar_planner', 'obstacle_inflation_cells'): 3,
                })
        self.assertEqual(self.navigation.read_text(), CONTROLLER)
        self.assertEqual(self.nodes.read_text(), NODES)
        self.assertEqual(list(self.root.glob('.*.tmp')), [])

    def test_concurrent_source_change_is_not_overwritten(self):
        original_stage = defaults._stage

        def stage(path, raw, mode):
            staged = original_stage(path, raw, mode)
            path.write_text(CONTROLLER + '# changed elsewhere\n')
            return staged

        with patch.object(defaults, '_stage', side_effect=stage):
            with self.assertRaisesRegex(ParameterDefaultsError, '변경되었습니다'):
                save_parameter_defaults([self.navigation], {
                    ('/controller_server', 'FollowPath.max_vel_x'): 0.1,
                })
        self.assertEqual(self.navigation.read_text(), CONTROLLER + '# changed elsewhere\n')
        self.assertEqual(self.backups(self.navigation), [])

    def test_file_mode_crlf_and_repeated_unique_backups_are_preserved(self):
        path = self.write('windows.yaml', CONTROLLER.replace('\n', '\r\n'))
        path.chmod(0o640)
        for speed in (0.12, 0.14):
            save_parameter_defaults([path], {('/controller_server', 'FollowPath.max_vel_x'): speed})
        self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o640)
        self.assertEqual(len(self.backups(path)), 2)
        self.assertNotIn(b'\n', path.read_bytes().replace(b'\r\n', b''))

    def test_export_preserves_source_and_allows_explicit_runtime_settings(self):
        output = self.root / 'export.yaml'
        export_parameters(self.navigation, output, {
            ('/controller_server', 'FollowPath.max_vel_x'): 0.15,
            ('/controller_server', 'use_sim_time'): False,
        })
        self.assertEqual(self.navigation.read_text(), CONTROLLER)
        params = yaml.safe_load(output.read_text())['controller_server']['ros__parameters']
        self.assertEqual(params['FollowPath']['max_vel_x'], 0.15)
        self.assertIs(params['use_sim_time'], False)
        self.assertIn('# Literal dotted key.', output.read_text())

    def test_empty_changes_are_a_noop(self):
        self.assertEqual(save_parameter_defaults([], {}), [])
        self.assertEqual(self.backups(self.navigation), [])

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'source ROS Humble before running the ROS restart check')
    def test_fresh_ros_node_reads_saved_defaults_after_restart(self):
        # Each invocation creates and destroys a fresh ROS node using only the
        # launch-style params file. No GUI, executor, or client reapply is present.
        program = '''import json, sys, rclpy
rclpy.init(args=['--ros-args', '--params-file', sys.argv[1]])
node = rclpy.create_node('controller_server', automatically_declare_parameters_from_overrides=True)
print(json.dumps(node.get_parameter('FollowPath.max_vel_x').value))
node.destroy_node()
rclpy.shutdown()
'''
        env = dict(os.environ, ROS_DOMAIN_ID='229', ROS_LOCALHOST_ONLY='1')

        def fresh_value():
            result = subprocess.run([sys.executable, '-c', program, str(self.navigation)],
                                    check=True, capture_output=True, text=True, timeout=15, env=env)
            return json.loads(result.stdout.strip().splitlines()[-1])

        self.assertEqual(fresh_value(), 0.22)
        save_parameter_defaults([self.navigation], {('/controller_server', 'FollowPath.max_vel_x'): 0.13})
        self.assertEqual(fresh_value(), 0.13)


if __name__ == '__main__':
    unittest.main()
