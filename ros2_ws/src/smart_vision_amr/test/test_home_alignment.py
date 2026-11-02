"""Regression for a terminal Nav2 Home success with residual yaw error."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from types import SimpleNamespace

from smart_vision_amr.home_alignment import HomeYawTrim
from smart_vision_amr.mission_logic import MarkerMission


HOME = {'x': -1., 'y': -2., 'yaw': .2}
MARKERS = [{'id': 2, 'name': 'Sector 2', 'x': 2., 'y': 0., 'yaw': 0.}]


def evidence(**overrides):
    values = dict(goal_stamp=(1, 2), yaw_error=-.385, xy_error=.07,
                  xy_tolerance=.30, odom_yaw=.58, sensors_fresh=True,
                  scan_clear=True, health_ready=True, control_ready=True,
                  stop_ack=True, stopped=True, operator_clear=True)
    values.update(overrides)
    return values


class HomeYawTrimTests(unittest.TestCase):
    def test_terminal_success_requires_bounded_sweep_then_stopped_heading(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        turn = trim.step(now=10., **evidence())
        self.assertEqual(turn.phase, 'TURNING')
        self.assertAlmostEqual(turn.angular, -.15)
        self.assertEqual(trim.step(now=11., **evidence(yaw_error=-.10, odom_yaw=.43,
                                                      stopped=False, stop_ack=False)).phase,
                         'SETTLING')
        self.assertEqual(trim.step(now=11.2, **evidence(yaw_error=-.10, odom_yaw=.43,
                                                       stopped=True, stop_ack=True)).phase,
                         'SETTLING')
        self.assertEqual(trim.step(now=11.51, **evidence(yaw_error=-.10, odom_yaw=.43,
                                                        stopped=True, stop_ack=True)).phase,
                         'DONE')

    def test_legacy_home_band_does_not_skip_strict_trim(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        result = trim.step(now=10., **evidence(yaw_error=-.34))
        self.assertEqual(result.phase, 'TURNING')
        self.assertLess(result.angular, 0.)

    def test_allowed_initial_error_can_reach_trim_target_within_rotation_limit(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        self.assertEqual(trim.step(now=10., **evidence(
            yaw_error=.80, odom_yaw=0.)).phase, 'TURNING')
        self.assertEqual(trim.step(now=14.4, **evidence(
            yaw_error=.14, odom_yaw=.66, stopped=False, stop_ack=False)).phase,
            'TURNING')
        self.assertEqual(trim.step(now=14.7, **evidence(
            yaw_error=.11, odom_yaw=.69, stopped=False, stop_ack=False)).phase,
            'SETTLING')
        self.assertEqual(trim.step(now=14.8, **evidence(
            yaw_error=.11, odom_yaw=.69)).phase, 'SETTLING')
        self.assertEqual(trim.step(now=15.11, **evidence(
            yaw_error=.11, odom_yaw=.69)).phase, 'DONE')

    def test_rotation_limit_still_applies_when_heading_reaches_target(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        self.assertEqual(trim.step(now=10., **evidence(
            yaw_error=.385, odom_yaw=.58)).phase, 'TURNING')
        overshoot = trim.step(now=11., **evidence(
            yaw_error=.10, odom_yaw=1.29, stopped=True, stop_ack=True))
        self.assertEqual(overshoot.phase, 'FAILED')
        self.assertIn('회전 각도', overshoot.reason)

    def test_stale_sensor_means_zero_motion_then_bounded_failure(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        self.assertEqual(trim.step(now=10., **evidence(sensors_fresh=False)).phase,
                         'WAIT_SAFE')
        self.assertEqual(trim.step(now=10., **evidence(sensors_fresh=False)).angular, 0.)
        self.assertEqual(trim.step(now=18.01, **evidence(sensors_fresh=False)).phase,
                         'FAILED')

    def test_scan_obstacle_manual_override_and_xy_drift_never_rotate(self):
        for changed in ({'scan_clear': False}, {'operator_clear': False},
                        {'xy_error': .31}, {'health_ready': False},
                        {'control_ready': False}, {'stop_ack': False},
                        {'stopped': False}):
            with self.subTest(changed=changed):
                trim = HomeYawTrim()
                trim.start((1, 2), 10.)
                result = trim.step(now=10., **evidence(**changed))
                self.assertEqual(result.angular, 0.)
                self.assertNotEqual(result.phase, 'TURNING')

    def test_retry_same_goal_does_not_restart_timeout(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        trim.start((1, 2), 17.)
        self.assertEqual(trim.step(now=18.01, **evidence()).phase, 'FAILED')

    def test_active_turn_stops_for_sensor_or_health_loss(self):
        for changed in ({'sensors_fresh': False}, {'health_ready': False},
                        {'scan_clear': False}):
            with self.subTest(changed=changed):
                trim = HomeYawTrim()
                trim.start((1, 2), 10.)
                self.assertEqual(trim.step(now=10., **evidence()).phase, 'TURNING')
                result = trim.step(now=10.2, **evidence(**changed))
                self.assertEqual(result.angular, 0.)
                self.assertIn(result.phase, {'WAIT_SAFE', 'FAILED'})

    def test_healthy_stopped_home_required_for_completion(self):
        trim = HomeYawTrim()
        trim.start((1, 2), 10.)
        self.assertEqual(trim.step(now=10., **evidence(yaw_error=.1,
                            health_ready=False)).phase, 'WAIT_SAFE')
        self.assertEqual(trim.step(now=18.01, **evidence(yaw_error=.1,
                            health_ready=False)).phase, 'FAILED')

    def test_mission_pauses_with_retained_home_and_no_false_completion(self):
        mission = MarkerMission(MARKERS, [2])
        mission.start(home=HOME, auto_return_home=True)
        mission.drain_commands()
        mission.return_home()
        mission.drain_commands()
        mission.goal_dispatched((1, 2))
        mission.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': 1, 'nanosec': 2}},
                              pose={'x': -1., 'y': -2., 'yaw': .585}, stationary=True)
        self.assertEqual(mission.state, 'RETURNING_HOME')
        self.assertFalse(mission.observe_home({'x': -1., 'y': -2., 'yaw': .585}, True))
        self.assertTrue(mission.pause_home_alignment('회전 여유 부족'))
        self.assertEqual(mission.state, 'PAUSED')
        self.assertEqual(mission.home, HOME)
        self.assertEqual(mission.drain_commands(), [('cancel', None), ('stop', None)])
        mission.resume()
        self.assertEqual(mission.state, 'RETURNING_HOME')


try:
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from smart_vision_amr.web_mission_server import MissionApplication, create_ros_bridge
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


@unittest.skipUnless(HAS_ROS, 'ROS Humble unavailable')
class HomeYawBridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory()
        self.app = MissionApplication(MARKERS, 215, False, Path(self.tmp.name)/'settings.json')
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=215,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = create_ros_bridge(self.app,
            SimpleNamespace(camera_topic='/unused_camera', base_frame='base_footprint'),
            self.context)
        self.pub = Publisher()
        self.node.motion_pub = self.pub
        now = time.monotonic()
        self.app.mission.start(home=HOME, auto_return_home=True)
        self.app.mission.drain_commands()
        self.app.mission.return_home()
        self.app.mission.drain_commands()
        self.app.mission.goal_dispatched((1, 2))
        # 0.34 rad is within MarkerMission's legacy 0.35 rad band, but must
        # still be corrected before reporting HOME_REACHED.
        self.app.pose = {'x': -.93, 'y': -2., 'yaw': .54}
        self.app.odom_pose = {'x': 0., 'y': 0., 'yaw': .54}
        self.app.speed = {'linear': 0., 'angular': 0.}
        self.app.pose_at = self.app.scan_at = self.app.odom_at = now
        self.app.localization_scan_at = self.app.localization_odom_at = now
        self.app.localization_health_at = self.app.manual_mux_at = now
        self.app.localization_clear = True
        self.app.localization_health = {'level': 'ok', 'autonomous_allowed': True}
        self.app.manual_mux_status = {'mode': 'hold', 'source_id': self.node.motion_source,
                                      'available': True, 'autonomous_allowed': True,
                                      'stop_acknowledged': True, 'estop_latched': False}
        self.app.map_matches_configured = True
        self.app.readiness = lambda **_kwargs: {'ready': True}
        self.node.on_navigation(String(data=json.dumps(
            {'status': 'SUCCEEDED', 'goal_stamp': {'sec': 1, 'nanosec': 2}})))
        self.app.manual_mux_status['control_seq'] = self.node.home_yaw_hold_seq

    def tearDown(self):
        self.node.destroy_node()
        self.context.shutdown()
        self.tmp.cleanup()

    def fresh(self, now):
        self.app.pose_at = self.app.scan_at = self.app.odom_at = now
        self.app.localization_scan_at = self.app.localization_odom_at = now
        self.app.localization_health_at = self.app.manual_mux_at = now

    def test_bridge_trims_home_and_completes_only_after_stopped(self):
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        now = time.monotonic()
        self.fresh(now)
        self.node.tick_home_yaw_trim(now, True, True)
        self.assertLess(self.node.home_yaw_angular, 0.)
        self.assertLessEqual(abs(self.node.home_yaw_angular), .15)
        self.app.pose['yaw'] = .29
        self.app.odom_pose['yaw'] = .29
        self.app.speed['angular'] = .12
        self.fresh(now+.2)
        self.node.tick_home_yaw_trim(now+.2, True, False)
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.app.speed['angular'] = 0.
        self.fresh(now+.3)
        self.node.tick_home_yaw_trim(now+.3, True, True)
        self.fresh(now+.65)
        self.node.tick_home_yaw_trim(now+.65, True, True)
        self.assertEqual(self.app.mission.state, 'HOME_REACHED')
        self.assertFalse(self.app.mission.returning_home)

    def test_bridge_blocks_unsafe_scan_and_pauses(self):
        self.app.localization_clear = False
        now = time.monotonic()
        self.fresh(now)
        self.node.tick_home_yaw_trim(now, True, True)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertIn('회전 반경', self.app.mission.reason)
        self.assertEqual(self.node.home_yaw_angular, 0.)
        self.assertEqual(json.loads(self.pub.messages[-1].data)['mode'], 'hold')

    def test_bridge_rejects_old_mux_hold_ack(self):
        self.app.manual_mux_status['control_seq'] = self.node.home_yaw_hold_seq-1
        now = time.monotonic()
        self.fresh(now)
        self.node.tick_home_yaw_trim(now, True, True)
        self.assertEqual(self.node.home_yaw_angular, 0.)
        self.assertEqual(self.node.home_yaw_trim.phase, 'WAIT_STOP')
        self.app.manual_mux_status['control_seq'] = self.node.home_yaw_hold_seq
        self.fresh(now+.1)
        self.node.tick_home_yaw_trim(now+.1, True, True)
        self.assertEqual(self.node.home_yaw_trim.phase, 'TURNING')

    def test_repeated_navigation_success_does_not_interrupt_trim(self):
        now = time.monotonic()
        self.fresh(now)
        self.node.tick_home_yaw_trim(now, True, True)
        self.assertEqual(self.node.home_yaw_trim.phase, 'TURNING')
        held_seq = self.node.home_yaw_hold_seq
        published = len(self.pub.messages)
        # The new pose is geometrically acceptable, but the trim has not
        # stopped and settled. A duplicate success must not complete Home.
        self.app.pose['yaw'] = .49
        self.node.on_navigation(String(data=json.dumps(
            {'status': 'SUCCEEDED', 'goal_stamp': {'sec': 1, 'nanosec': 2}})))
        self.assertEqual(self.node.home_yaw_trim.phase, 'TURNING')
        self.assertEqual(self.app.mission.state, 'RETURNING_HOME')
        self.assertEqual(self.node.home_yaw_hold_seq, held_seq)
        self.assertEqual(len(self.pub.messages), published)


if __name__ == '__main__':
    unittest.main()
