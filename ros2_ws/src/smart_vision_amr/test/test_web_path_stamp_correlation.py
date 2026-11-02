"""Planner status and Path arrive on independent topics; correlate exact stamps."""

import json
import os
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as ROSPath
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication, create_ros_bridge


@unittest.skipUnless(HAS_ROS, 'source ROS Humble for the isolated bridge test')
class PathStampBridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.environment = patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'})
        self.environment.start()
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=217,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.app = MissionApplication(DEFAULT_MARKERS, 217, False,
                                      Path(self.temp.name) / 'settings.json')
        self.app.mission.state = 'NAVIGATING'
        self.app.mission.goal_stamp = (15, 7)
        self.node = create_ros_bridge(self.app, SimpleNamespace(
            camera_topic='/isolated_path_stamp_camera', base_frame='isolated_base'), self.context)
        self.sent_at = self.node.get_clock().now().nanoseconds - 10_000_000

    def tearDown(self):
        self.node.destroy_node()
        self.context.shutdown()
        self.environment.stop()
        self.temp.cleanup()

    def path(self, nanosec, *, empty=False):
        message = ROSPath()
        message.header.frame_id = 'map'
        message.header.stamp.sec = 23
        message.header.stamp.nanosec = nanosec
        if not empty:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = 1.
            message.poses.append(pose)
        self.node.on_path(message)
        return self.app.state()

    def status(self, nanosec, *, kind='LOCAL_DETOUR', state='ACTIVE'):
        self.sent_at += 1_000_000
        packet = {'status': 'READY', 'sent_at_ns': self.sent_at,
                  'goal_stamp': {'sec': 15, 'nanosec': 7},
                  'goal': {'x': 1.2, 'y': .55, 'frame_id': 'map'},
                  'path_stamp': {'sec': 23, 'nanosec': nanosec},
                  'plan_kind': kind, 'local_detour_state': state}
        self.node.on_planner(String(data=json.dumps(packet)))
        return self.app.state()

    def test_status_first_then_path_exposes_exact_match_only(self):
        state = self.path(4)
        self.assertEqual(state['path_stamp'], {'sec': 23, 'nanosec': 4})
        state = self.status(5)
        self.assertEqual(state['local_detour']['path_stamp'], {'sec': 23, 'nanosec': 5})
        self.assertNotEqual(state['path_stamp'], state['local_detour']['path_stamp'])
        state = self.path(5)
        self.assertEqual(state['path_stamp'], state['local_detour']['path_stamp'])

    def test_path_first_then_status_and_empty_cancellation(self):
        state = self.path(7)
        self.assertFalse(state['local_detour']['available'])
        state = self.status(7)
        self.assertEqual(state['path_stamp'], state['local_detour']['path_stamp'])
        state = self.path(8, empty=True)
        self.assertEqual(state['path'], [])
        self.assertNotEqual(state['path_stamp'], state['local_detour']['path_stamp'])
        state = self.status(8, kind='GLOBAL', state='FAILED')
        self.assertEqual(state['path_stamp'], state['local_detour']['path_stamp'])
        self.assertNotEqual(state['local_detour']['plan_kind'], 'LOCAL_DETOUR')

    def test_delayed_status_still_matches_path_for_read_only_sim_monitoring(self):
        self.app.use_sim_time = True
        self.app.planner_available = True
        self.app.sim_clock_at = self.app.scan_at = self.app.odom_at = time.monotonic()
        self.sent_at = self.node.get_clock().now().nanoseconds - 3_000_000_000
        self.path(11)
        state = self.status(11, kind='GLOBAL', state='NONE')
        self.assertEqual(state['observed_route']['path_stamp'], {'sec': 23, 'nanosec': 11})
        self.assertTrue(state['observed_route']['active'])
        self.assertEqual(state['observed_route']['plan_kind'], 'GLOBAL')
        self.assertEqual(state['observed_route']['goal'], {'x': 1.2, 'y': .55, 'frame_id': 'map'})
        self.assertFalse(state['local_detour']['available'], 'delayed data must not become fresh control evidence')

    def test_out_of_order_status_does_not_replace_display_phase(self):
        self.path(12)
        self.status(12, kind='GLOBAL', state='NONE')
        late = {'status': 'WAIT_REPLAN', 'sent_at_ns': self.sent_at - 1,
                'goal_stamp': {'sec': 15, 'nanosec': 7},
                'path_stamp': {'sec': 23, 'nanosec': 3}}
        self.node.on_planner(String(data=json.dumps(late)))
        state = self.app.state()
        self.assertEqual(state['navigation']['planner_status'], 'READY')
        self.assertEqual(self.app.planner_details['path_stamp'], {'sec': 23, 'nanosec': 12})


if __name__ == '__main__':
    unittest.main()
