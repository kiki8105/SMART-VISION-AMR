"""PyQt stop shares the robot-side mux service without claiming /cmd_vel."""

import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtWidgets
from rclpy.task import Future

from cmd_vel_controller import Controller, LazyVelocityPublisher

# Keep the Python wrapper alive for the entire pytest process. QApplication
# destruction between unittest classes can invalidate later QWidget creation.
QT_TEST_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class ControllerSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QT_TEST_APP

    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.future = Future()
        self.client = Mock()
        self.client.service_is_ready.return_value = True
        self.client.call_async.return_value = self.future
        self.node = Mock()
        self.node.resolve_topic_name.side_effect = lambda topic: '/'+topic.lstrip('/')
        self.node.get_publishers_info_by_topic.return_value = [SimpleNamespace(node_name='motion_mux')]
        self.node.create_client.return_value = self.client
        self.publisher = LazyVelocityPublisher(self.node)
        settings = QtCore.QSettings(str(Path(self.temporary.name)/'controller.ini'),
                                    QtCore.QSettings.IniFormat)
        self.window = Controller(self.publisher, domain_id=76, settings=settings)

    def tearDown(self):
        self.window.timer.stop()
        self.window.video_timer.stop()
        self.window.deleteLater()
        self.temporary.cleanup()

    def test_red_stop_requests_robot_service_then_reports_acknowledgment(self):
        self.window.emergency_stop()
        self.assertEqual(self.node.create_client.call_args.args[1],
                         '/smart_vision_amr/emergency_stop')
        self.assertIn('응답 대기', self.window.status.text())
        self.node.create_publisher.assert_not_called()
        self.future.set_result(SimpleNamespace(success=True, message='zero published'))
        self.window.tick()
        self.assertIn('비상정지 확인', self.window.status.text())
        self.assertIsNone(self.publisher.publisher)

    def test_disconnected_service_never_claims_stop_was_acknowledged(self):
        self.client.service_is_ready.return_value = False
        self.window.emergency_stop()
        self.assertIn('미확인', self.window.status.text())
        self.client.call_async.assert_not_called()

    def test_reset_uses_shared_service_and_displays_guard_rejection(self):
        self.window.reset_emergency_stop()
        self.assertEqual(self.node.create_client.call_args.args[1],
                         '/smart_vision_amr/reset_emergency_stop')
        self.future.set_result(SimpleNamespace(success=False, message='sensor guard'))
        self.window.tick()
        self.assertIn('해제 거부', self.window.status.text())
        self.node.create_publisher.assert_not_called()

    def test_standalone_controller_keeps_zero_command_behavior(self):
        self.node.get_publishers_info_by_topic.return_value = []
        self.window.emergency_stop()
        self.node.create_client.assert_not_called()


if __name__ == '__main__':
    unittest.main()
