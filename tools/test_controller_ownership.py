"""Check controller output ownership without a ROS context or a robot."""
import copy
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtWidgets
from geometry_msgs.msg import Twist

from cmd_vel_controller import Controller, LazyVelocityPublisher

QT_TEST_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakePublisher:
    def __init__(self, topic):
        self.topic_name = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FakeNode:
    def __init__(self):
        self.endpoints = []
        self.created = []
        self.destroyed = []
        self.graph_error = False

    def resolve_topic_name(self, topic):
        return '/' + topic.lstrip('/')

    def get_publishers_info_by_topic(self, topic):
        if self.graph_error:
            raise RuntimeError('Graph unavailable')
        return self.endpoints

    def create_publisher(self, message_type, topic, depth):
        publisher = FakePublisher(self.resolve_topic_name(topic))
        self.created.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        self.destroyed.append(publisher)


class ControllerOwnershipTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QT_TEST_APP

    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.node = FakeNode()
        self.publisher = LazyVelocityPublisher(self.node)
        settings = QtCore.QSettings(str(Path(self.temporary.name) / 'controller.ini'),
                                    QtCore.QSettings.IniFormat)
        self.window = Controller(self.publisher, domain_id=76, settings=settings)

    def tearDown(self):
        self.window.timer.stop()
        self.window.video_timer.stop()
        self.window.deleteLater()
        self.publisher.close()
        self.temporary.cleanup()

    def test_open_idle_and_zero_commands_do_not_create_output_publisher(self):
        self.window.tick()
        self.window.stop()
        for _ in range(6):
            self.window.tick()
        self.assertEqual(self.node.created, [])

    def test_standalone_nonzero_manual_input_creates_publisher_and_stops(self):
        button = self.window.buttons['forward']
        button.setDown(True)
        self.window.start(button, (1, 0))
        self.assertEqual(len(self.node.created), 1)
        self.assertGreater(self.node.created[0].messages[-1].linear.x, 0.0)
        self.window.stop()
        self.assertEqual(self.node.created[0].messages[-1].linear.x, 0.0)

    def test_mux_owner_blocks_manual_without_creating_output_publisher(self):
        self.node.endpoints = [SimpleNamespace(node_name='motion_mux')]
        self.window.buttons['forward'].setDown(True)
        self.window.start(self.window.buttons['forward'], (1, 0))
        self.window.tick()
        self.assertEqual(self.node.created, [])
        self.assertIsNone(self.window.active)
        self.assertIn('웹 조이스틱', self.window.status.text())

    def test_mux_appearing_later_releases_existing_output_without_publishing(self):
        button = self.window.buttons['forward']
        button.setDown(True)
        self.window.start(button, (1, 0))
        previous_messages = len(self.node.created[0].messages)
        self.node.endpoints = [SimpleNamespace(node_name='motion_mux')]
        self.window.tick()
        self.assertEqual(self.node.destroyed, self.node.created)
        self.assertEqual(len(self.node.created[0].messages), previous_messages)
        self.assertIsNone(self.publisher.publisher)
        self.assertIsNone(self.window.active)

    def test_unavailable_graph_never_grants_manual_output(self):
        self.node.graph_error = True
        message = Twist()
        message.linear.x = 0.1
        self.assertFalse(self.publisher.publish(message))
        self.assertEqual(self.node.created, [])


if __name__ == '__main__':
    unittest.main()
