"""GUI defaults persistence checks; only temporary YAML/INI files are written.

Run with the ROS 2 environment sourced and QT_QPA_PLATFORM=offscreen. The process
restart check creates two short-lived local ROS nodes in isolated domain 211.
"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtWidgets
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
import yaml

from navigation_panels import ParameterPanel


class ParameterPersistenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='parameter-persistence-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'launch defaults.yaml'
        self.source.write_text(
            '# Defaults consumed by the next controller_server process.\n'
            '/controller_server:\n'
            '  ros__parameters:\n'
            '    speed: 0.20\n'
            '    gain: 1.0\n'
            '    drive_enabled: false\n'
            '    use_sim_time: true\n', encoding='utf-8')
        self.original = self.source.read_bytes()
        self.settings = QtCore.QSettings(
            str(self.root / 'controller.ini'), QtCore.QSettings.IniFormat)
        self.panel, self.bridge = self.make_panel(parameter_configs=[str(self.source)])
        self.seed(self.panel)

    def make_panel(self, *, domain=211, namespace='', parameter_configs=None):
        bridge = SimpleNamespace(
            busy=False, maintenance=False, namespace=namespace,
            node=Mock(), call=Mock())
        panel = ParameterPanel(
            bridge, domain, Mock(), autoload=False,
            parameter_configs=parameter_configs, settings=self.settings)
        self.addCleanup(panel.deleteLater)
        self.addCleanup(panel.close)
        return panel, bridge

    def seed(self, panel):
        values = {'speed': .20, 'gain': 1.0,
                  'drive_enabled': False, 'use_sim_time': True}
        panel.records = {
            ('/controller_server', name): dict(
                value=value,
                type=Parameter(name, value=value).to_parameter_msg().value.type,
                descriptor=ParameterDescriptor(name=name))
            for name, value in values.items()
        }
        panel.rebuild_groups()

    def request_value(self, name, value):
        self.panel.show_record(('/controller_server', name))
        self.panel.value.setText(yaml.safe_dump(value).split('\n...')[0].strip())
        self.panel.apply_parameter()
        self.assertTrue(self.panel.write_pending)
        return self.bridge.call.call_args.args[3]

    def accept_value(self, name, value):
        done = self.request_value(name, value)
        done(SimpleNamespace(result=SimpleNamespace(successful=True, reason='')), None)
        self.assertFalse(self.panel.write_pending)

    def defaults(self):
        return yaml.safe_load(self.source.read_text())['/controller_server']['ros__parameters']

    def test_only_accepted_changes_are_saved_not_current_records_or_editor(self):
        self.accept_value('speed', .18)
        self.panel.records[('/controller_server', 'gain')]['value'] = 9.0
        self.panel.value.setText('0.99')  # Editing alone is not acceptance.

        self.panel.save_defaults()

        self.assertEqual(self.defaults()['speed'], .18)
        self.assertEqual(self.defaults()['gain'], 1.0)
        self.assertEqual(self.panel.saved_defaults,
                         {('/controller_server', 'speed'): .18})
        self.assertIn('저장 완료', self.panel.status.text())
        backups = list(self.root.glob(self.source.name + '.bak-*'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(backups[0].read_bytes(), self.original)

    def test_pending_rejected_and_service_error_changes_are_never_saved(self):
        for error in (None, 'parameter service unavailable'):
            with self.subTest(error=error):
                done = self.request_value('speed', .31)
                self.panel.save_defaults()
                self.assertEqual(self.source.read_bytes(), self.original)
                self.assertEqual(self.panel.changes, {})
                self.assertEqual(self.panel.saved_defaults, {})
                done(SimpleNamespace(result=SimpleNamespace(
                    successful=False, reason='outside allowed speed')), error)
                self.panel.save_defaults()
                self.assertEqual(self.source.read_bytes(), self.original)
                self.assertEqual(self.panel.changes, {})
                self.assertEqual(self.panel.saved_defaults, {})
        self.assertEqual(list(self.root.glob('*.bak-*')), [])

    def test_operational_values_are_excluded_with_visible_status(self):
        self.accept_value('speed', .16)
        self.accept_value('drive_enabled', True)
        self.accept_value('use_sim_time', False)

        self.panel.save_defaults()

        self.assertEqual(self.defaults()['speed'], .16)
        self.assertFalse(self.defaults()['drive_enabled'])
        self.assertTrue(self.defaults()['use_sim_time'])
        self.assertEqual(self.panel.saved_defaults,
                         {('/controller_server', 'speed'): .16})
        self.assertIn('제외', self.panel.status.text())
        self.assertIn('drive_enabled', self.panel.status.text())
        self.assertIn('use_sim_time', self.panel.status.text())

    def test_config_selection_is_canonical_and_scoped_by_domain_and_namespace(self):
        panel, _ = self.make_panel(domain=211, namespace='/robot1')
        alias = self.root / 'defaults-link.yaml'
        alias.symlink_to(self.source)
        with patch.object(QtWidgets.QFileDialog, 'getOpenFileNames',
                          return_value=([str(self.source), str(alias)], 'YAML')):
            self.assertTrue(panel.choose_parameter_configs())
        self.assertEqual(panel.parameter_configs, [str(self.source.resolve())])

        same, _ = self.make_panel(domain=211, namespace='/robot1')
        other_domain, _ = self.make_panel(domain=212, namespace='/robot1')
        other_namespace, _ = self.make_panel(domain=211, namespace='/robot2')
        root_namespace, _ = self.make_panel(domain=211, namespace='')
        self.assertEqual(same.parameter_configs, [str(self.source.resolve())])
        for other in (other_domain, other_namespace, root_namespace):
            self.assertEqual(other.parameter_configs, [])
        self.assertNotEqual(panel.parameter_config_key,
                            other_domain.parameter_config_key)
        self.assertNotEqual(panel.parameter_config_key,
                            other_namespace.parameter_config_key)

    def test_missing_source_reports_failure_without_claiming_saved(self):
        self.accept_value('speed', .17)
        self.source.unlink()

        self.panel.save_defaults()

        self.assertIn('저장 실패', self.panel.status.text())
        self.assertEqual(self.panel.saved_defaults, {})
        self.assertEqual(self.panel.changes,
                         {('/controller_server', 'speed'): .17})
        self.assertFalse(self.source.exists())

    def test_busy_write_refresh_and_reset_block_source_writes(self):
        self.accept_value('speed', .13)
        cases = ((self.bridge, 'busy'), (self.panel, 'write_pending'),
                 (self.panel, 'loading'), (self.panel, 'resetting'))
        for owner, attribute in cases:
            with self.subTest(state=attribute):
                setattr(owner, attribute, True)
                try:
                    with patch('navigation_panels.save_parameter_defaults') as save:
                        self.panel.save_defaults()
                    save.assert_not_called()
                    self.assertEqual(self.source.read_bytes(), self.original)
                    self.assertEqual(self.panel.saved_defaults, {})
                finally:
                    setattr(owner, attribute, False)

    def test_vehicle_scope_rejects_changes_from_another_namespace(self):
        self.bridge.namespace = '/robot1'
        self.panel.changes = {('/robot2/controller_server', 'speed'): .08}
        with patch('navigation_panels.save_parameter_defaults') as save:
            self.panel.save_defaults()
        save.assert_not_called()
        self.assertIn('저장 거부', self.panel.status.text())
        self.assertEqual(self.source.read_bytes(), self.original)

    def read_speed_in_new_ros_process(self):
        program = '''
import json
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node('controller_server')
try:
    node.declare_parameter('speed', 0.0)
    print('PARAMETER_DEFAULT=' + json.dumps(node.get_parameter('speed').value), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
'''
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID='211', ROS_LOCALHOST_ONLY='1',
                           ROS_LOG_DIR=str(self.root / 'ros-logs'))
        result = subprocess.run(
            [sys.executable, '-c', program, '--ros-args',
             '--params-file', str(self.source)],
            text=True, capture_output=True, env=environment,
            timeout=20, check=True)
        marker = 'PARAMETER_DEFAULT='
        values = [json.loads(line[len(marker):])
                  for line in result.stdout.splitlines() if line.startswith(marker)]
        self.assertEqual(len(values), 1, result.stdout + result.stderr)
        return values[0]

    def test_new_node_process_reads_gui_saved_default_after_restart(self):
        self.assertEqual(self.read_speed_in_new_ros_process(), .20)
        self.accept_value('speed', .14)

        self.panel.save_defaults()

        self.assertIn('저장 완료', self.panel.status.text())
        self.assertEqual(self.read_speed_in_new_ros_process(), .14)


if __name__ == '__main__':
    unittest.main()
