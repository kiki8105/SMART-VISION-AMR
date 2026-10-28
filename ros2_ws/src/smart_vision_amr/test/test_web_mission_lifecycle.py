"""ROS adapter cancellation races with controlled futures, isolated domain 215.

Publishers are replaced before any tick; this test never moves a robot. ROS is
optional for ordinary pure-Python test runs.
"""
from concurrent.futures import Future
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import json
import time
import unittest

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication, create_ros_bridge


class Publisher:
    def __init__(self):
        self.messages = []
    def publish(self, message):
        self.messages.append(message)
    def get_subscription_count(self):
        return 1


class ControlledCancelClient:
    def __init__(self):
        self.future = None
        self.available = True
        self.requests = 0
    def service_is_ready(self):
        return self.available
    def call_async(self, request):
        self.requests += 1
        self.future = Future()
        return self.future
    def ack(self, success=True):
        response = Trigger.Response()
        response.success = success
        self.future.set_result(response)


@unittest.skipUnless(HAS_ROS, 'source ROS Humble setup to run ROS adapter lifecycle tests')
class MissionLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.app = MissionApplication(DEFAULT_MARKERS, 215, False, Path(self.temporary.name)/'settings.json')
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=215, signal_handler_options=SignalHandlerOptions.NO)
        options = SimpleNamespace(camera_topic='/test_web_camera', base_frame='test_web_base')
        self.node = create_ros_bridge(self.app, options, self.context)
        self.goals, self.stops = Publisher(), Publisher()
        self.cancel = ControlledCancelClient()
        self.node.goal_pub, self.node.motion_pub, self.node.cancel_client = self.goals, self.stops, self.cancel

    def tearDown(self):
        self.node.destroy_node()
        self.context.shutdown()
        self.temporary.cleanup()

    def tick(self):
        self.app.pose_at = self.app.scan_at = self.app.odom_at = self.app.camera_at = time.monotonic()
        self.node.tick()

    def publish_first(self):
        self.app.mission.start(home={'x': -1., 'y': -2., 'yaw': .2})
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)

    def test_restart_waits_for_ack_and_stamps_after_it(self):
        self.publish_first()
        self.app.mission.pause()
        self.app.mission.restart()
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)
        time.sleep(.85)
        self.tick()
        self.assertEqual(len(self.goals.messages), 1, 'elapsed stop window alone cannot replace a cancel acknowledgement')
        stop_ns = self.node.get_clock().now().nanoseconds
        self.cancel.ack()
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)
        time.sleep(.25)
        self.tick()
        self.assertEqual(len(self.goals.messages), 2)
        stamp = self.goals.messages[-1].header.stamp
        self.assertGreater(stamp.sec*1000000000+stamp.nanosec, stop_ns)

    def test_estop_clears_held_goal_even_after_prior_cancel_ack(self):
        self.publish_first()
        self.app.mission.pause()
        self.app.mission.resume()
        self.tick()
        self.assertIsNotNone(self.node.goal_waiting)
        self.app.mission.emergency_stop()
        self.tick()
        self.assertIsNone(self.node.goal_waiting)
        self.cancel.ack()
        self.tick()
        time.sleep(.85)
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)
        self.assertEqual(self.app.mission.state, 'ESTOP')
        self.assertGreater(len(self.stops.messages), 0)

    def test_emergency_reset_before_tick_still_cancels_old_navigation(self):
        self.publish_first()
        self.app.mission.emergency_stop()
        self.app.mission.reset_emergency()
        self.tick()
        self.assertEqual(self.cancel.requests, 1)
        self.assertTrue(self.node.cancel_required)
        self.assertEqual(self.app.mission.state, 'CANCELED')
        self.assertEqual(len(self.goals.messages), 1)
        self.cancel.ack()
        self.tick()
        self.assertFalse(self.node.owns_navigation)

    def test_external_recovery_safety_estop_is_visible_and_needs_reset(self):
        from std_msgs.msg import String
        self.publish_first()
        message = String()
        message.data = json.dumps({'mode': 'estop', 'estop_latched': True,
                                   'reason': 'Reverse recovery safety stop',
                                   'source_id': self.node.motion_source})
        self.node.on_motion(message)
        self.assertTrue(self.app.mission.estop_latched)
        self.assertEqual(self.app.mission.state, 'ESTOP')
        self.tick()
        self.assertEqual(json.loads(self.stops.messages[-1].data)['mode'], 'estop')
        self.app.command('/api/emergency-stop', {'reset': True})
        self.tick()
        self.assertEqual(json.loads(self.stops.messages[-1].data)['mode'], 'reset')
        self.assertEqual(self.app.mission.state, 'CANCELED')

    def test_connection_change_rejected_until_cancel_ack(self):
        self.publish_first()
        self.app.mission.cancel()
        with self.assertRaises(ValueError):
            self.app.command('/api/connection', {'domain_id': 216})
        self.tick()
        with self.assertRaises(ValueError):
            self.app.command('/api/connection', {'domain_id': 216})
        self.cancel.ack()
        self.tick()
        with self.assertRaises(ValueError):
            self.app.command('/api/connection', {'domain_id': 216})
        self.app.mission.state = 'HOME_REACHED'
        self.app.mission.returning_home = False
        self.app.mission.commands.clear()
        self.app.command('/api/connection', {'domain_id': 216})
        self.assertEqual(self.app.pending_connection, (216, False))

    def test_old_odom_pauses_but_old_camera_keeps_route_active(self):
        self.publish_first()
        now = time.monotonic()
        self.app.pose_at = self.app.scan_at = self.app.camera_at = now
        self.app.odom_at = now-100
        self.node.tick()
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.app.odom_at = self.app.pose_at = self.app.scan_at = now
        self.app.camera_at = now-100
        self.app.mission.resume()
        self.node.tick()
        self.assertEqual(self.app.mission.state, 'NAVIGATING')
        self.assertEqual(self.app.mission.current_index, 0)

    def test_arrival_with_camera_outage_stays_stopped_until_fresh_marker(self):
        from std_msgs.msg import String
        self.publish_first()
        goal = self.goals.messages[-1]
        self.app.camera_at = time.monotonic()-100
        stamp = goal.header.stamp
        self.node.on_navigation(String(data=json.dumps({'status': 'SUCCEEDED', 'goal_stamp': {
            'sec': stamp.sec, 'nanosec': stamp.nanosec}})))
        self.node.tick()
        self.assertEqual(self.app.mission.state, 'CONFIRMING')
        self.assertEqual(self.app.mission.completed_ids, [])
        self.assertEqual(len(self.goals.messages), 1)
        self.assertTrue(self.node.motion_gate)
        self.assertEqual(json.loads(self.stops.messages[-1].data)['mode'], 'hold')

    def test_old_ros_source_sensor_headers_are_rejected(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        self.app.odom_at = self.app.scan_at = 0
        odom = Odometry()
        scan = LaserScan()
        old_ns = self.node.get_clock().now().nanoseconds-100*1000000000
        for message in (odom, scan):
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(old_ns, 1000000000)
        scan.range_min = .1
        scan.range_max = 10.
        scan.ranges = [5.]
        self.node.on_odom(odom)
        self.node.on_scan(scan)
        self.assertEqual(self.app.odom_at, 0)
        self.assertEqual(self.app.scan_at, 0)

    def test_failed_ack_or_unavailable_service_never_releases_new_goal(self):
        self.publish_first()
        self.cancel.available = False
        self.app.mission.pause()
        self.app.mission.restart()
        self.tick()
        time.sleep(.85)
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)

        self.assertTrue(self.node.cancel_required)
        self.cancel.available = True
        self.tick()
        self.cancel.ack(False)
        self.tick()
        self.assertTrue(self.node.cancel_required)
        time.sleep(.25)
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)

    def test_cancel_dispatches_home_only_after_ack_and_ignores_old_arrival(self):
        from std_msgs.msg import String
        self.publish_first()
        old_stamp = self.goals.messages[0].header.stamp
        self.app.mission.cancel()
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)
        time.sleep(.85)
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)
        self.cancel.ack()
        self.tick()
        time.sleep(.25)
        self.tick()
        self.assertEqual(len(self.goals.messages), 2)
        home_goal = self.goals.messages[-1]
        self.assertEqual(home_goal.pose.position.x, -1.)
        self.assertEqual(home_goal.pose.position.y, -2.)
        self.app.pose = {'x': -1., 'y': -2., 'yaw': .2}
        self.app.speed = {'linear': 0., 'angular': 0.}
        old_success = String(data=json.dumps({'status': 'SUCCEEDED', 'goal_stamp': {
            'sec': old_stamp.sec, 'nanosec': old_stamp.nanosec}}))
        self.node.on_navigation(old_success)
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertTrue(self.app.navigation_active)
        home_stamp = home_goal.header.stamp
        self.node.on_navigation(String(data=json.dumps({'status': 'SUCCEEDED', 'goal_stamp': {
            'sec': home_stamp.sec, 'nanosec': home_stamp.nanosec}})))
        self.assertEqual(self.app.mission.state, 'HOME_REACHED')
        self.assertFalse(self.app.navigation_active)
        self.assertFalse(self.app.mission.returning_home)

    def test_return_ignores_stale_camera_but_pauses_on_stale_scan(self):
        self.publish_first()
        self.app.mission.return_home()
        now = time.monotonic()
        self.app.pose_at = self.app.scan_at = self.app.odom_at = now
        self.app.camera_at = now-100
        self.node.tick()
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.app.scan_at = now-100
        self.node.tick()
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertTrue(self.app.mission.returning_home)
        self.assertIsNone(self.node.goal_waiting)

    def test_cancel_return_then_estop_cannot_release_held_home_goal(self):
        self.publish_first()
        self.app.mission.cancel()
        self.tick()
        self.assertIsNotNone(self.node.goal_waiting)
        self.app.mission.emergency_stop()
        self.tick()
        self.assertIsNone(self.node.goal_waiting)
        self.cancel.ack()
        self.tick()
        time.sleep(.85)
        self.tick()
        self.assertEqual(len(self.goals.messages), 1)
        self.assertEqual(self.app.mission.state, 'ESTOP')


if __name__ == '__main__':
    unittest.main()
