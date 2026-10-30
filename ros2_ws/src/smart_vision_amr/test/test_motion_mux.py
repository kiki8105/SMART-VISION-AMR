"""Manual authority, replay isolation, dead-man leases, and Nav2 topic wiring."""

import math
import unittest

from smart_vision_amr.motion_mux import MotionMuxCore


class MotionMuxTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.mux = MotionMuxCore(clock=lambda: self.now)
        self.seq = 0

    def control(self, mode, source='web-a', **values):
        self.seq += 1
        payload = {'source_id': source, 'seq': self.seq, 'mode': mode,
                   'sent_at_ns': 20_000_000_000, **values}
        return self.mux.receive_control(payload, ros_now_ns=20_000_000_000)

    def test_navigation_without_a_web_session_remains_usable(self):
        self.assertEqual(self.mux.output(), (0.0, 0.0))
        self.mux.receive_auto(.1, -.2)
        self.assertEqual(self.mux.output(), (.1, -.2))
        self.now += .5
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_manual_always_wins_over_concurrent_auto_commands(self):
        self.mux.receive_auto(.2, 0.0)
        self.assertTrue(self.control('manual', linear=-.1, angular=.3))
        for _ in range(5):
            self.assertFalse(self.mux.receive_auto(.2, -.8))
            self.assertEqual(self.mux.output(), (-.1, .3))

    def test_release_does_not_replay_an_old_navigation_command(self):
        self.mux.receive_auto(.2, 0.0)
        self.control('manual', linear=-.1)
        self.control('hold')
        self.mux.receive_auto(.25, 1.0)
        self.control('auto')
        self.assertEqual(self.mux.output(), (0.0, 0.0))
        self.mux.receive_auto(.1, .2)
        self.assertEqual(self.mux.output(), (.1, .2))

    def test_repeated_auto_heartbeats_do_not_clear_fresh_navigation(self):
        self.control('auto')
        self.mux.receive_auto(.1, 0.0)
        self.control('auto')
        self.assertEqual(self.mux.output(), (.1, 0.0))

    def test_manual_lease_expires_to_hold_even_with_fresh_auto_input(self):
        self.control('manual', linear=.1)
        self.now += .499
        self.assertEqual(self.mux.output(), (.1, 0.0))
        self.now += .0011
        self.mux.receive_auto(.25, .3)
        self.assertEqual(self.mux.output(), (0.0, 0.0))
        self.assertEqual(self.mux.status()['mode'], 'hold')
        self.now += 10
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_lease_is_renewed_by_manual_packets_only(self):
        self.control('manual', angular=.2)
        self.now += .3
        self.control('manual', angular=.3)
        self.now += .3
        self.assertEqual(self.mux.output(), (0.0, .3))
        self.now += .21
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_emergency_stop_is_latched_ahead_of_manual_and_auto(self):
        self.control('manual', linear=.1)
        self.control('estop')
        self.assertFalse(self.control('manual', linear=.2))
        self.assertFalse(self.control('auto'))
        self.mux.receive_auto(.2, .3)
        self.assertEqual(self.mux.output(), (0.0, 0.0))
        self.assertTrue(self.mux.status()['estop_latched'])
        self.assertTrue(self.control('reset'))
        self.assertEqual(self.mux.status()['mode'], 'hold')
        self.assertFalse(self.mux.status()['estop_latched'])
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_other_bridge_cannot_override_a_live_manual_lease(self):
        self.control('manual', linear=.1)
        self.assertFalse(self.control('manual', source='web-b', linear=-.2))
        self.assertFalse(self.control('auto', source='web-b'))
        self.assertEqual(self.mux.output(), (.1, 0.0))

    def test_restarted_bridge_retires_the_previous_source_permanently(self):
        self.control('manual', linear=.1)
        self.now += .6
        self.assertTrue(self.control('hold', source='web-b'))
        self.assertFalse(self.control('manual', source='web-a', linear=.2))
        self.assertFalse(self.control('estop', source='web-a'))
        self.assertEqual(self.mux.status()['source_id'], 'web-b')

    def test_emergency_stop_can_preempt_another_bridge_manual_lease(self):
        self.control('manual', linear=.1)
        self.assertTrue(self.control('estop', source='web-b'))
        self.assertEqual(self.mux.output(), (0.0, 0.0))
        self.assertFalse(self.control('reset', source='web-a'))
        self.assertTrue(self.control('reset', source='web-b'))

    def test_recovery_safety_stop_keeps_web_owner_for_explicit_reset(self):
        source = MotionMuxCore.RECOVERY_SAFETY_SOURCE
        self.control('auto', source='web-a')
        self.mux.receive_auto(.1, 0.0)
        self.assertTrue(self.control('estop', source=source))
        self.assertEqual(self.mux.output(), (0.0, 0.0))
        self.assertEqual(self.mux.status()['source_id'], 'web-a')
        self.assertFalse(self.control('reset', source=source))
        self.assertFalse(self.control('auto', source='web-a'))
        self.assertTrue(self.control('reset', source='web-a'))
        self.assertEqual(self.mux.status()['mode'], 'hold')
        self.assertFalse(self.mux.status()['estop_latched'])
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_unaccepted_source_does_not_steal_an_emergency_reset(self):
        self.control('estop')
        self.assertFalse(self.control('auto', source='web-b'))
        self.assertEqual(self.mux.status()['source_id'], 'web-a')
        self.assertTrue(self.control('reset'))

    def test_replayed_or_out_of_order_control_never_reactivates_motion(self):
        self.control('manual', linear=.1)
        self.control('hold')
        for sequence in (1, 2, 0):
            self.assertFalse(self.mux.receive_control(
                {'source_id': 'web-a', 'seq': sequence, 'mode': 'manual',
                 'linear': .2, 'sent_at_ns': 20_000_000_000},
                ros_now_ns=20_000_000_000))
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_stale_or_future_ros_stamps_are_rejected(self):
        for stamp in (18_999_999_999, 21_000_000_001):
            self.assertFalse(self.control('manual', sent_at_ns=stamp, linear=.1))
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_invalid_json_shapes_and_nonfinite_velocities_do_not_move(self):
        invalids = [None, [], {}, {'mode': []}, {'mode': {}},
                    {'source_id': 'x', 'seq': 1, 'mode': [], 'sent_at_ns': 0}]
        for payload in invalids:
            self.assertFalse(self.mux.receive_control(payload, 0))
        for value in (math.nan, math.inf, -math.inf, '0.1', True, 10**1000):
            self.assertFalse(self.control('manual', linear=value))
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_untrusted_velocities_are_bounded_at_the_final_output(self):
        self.control('manual', linear=100, angular=-100)
        self.assertEqual(self.mux.output(), (.25, -1.0))
        self.control('auto')
        self.mux.receive_auto(-100, 100)
        self.assertEqual(self.mux.output(), (-.25, 1.0))

    def test_invalid_auto_input_clears_previously_valid_velocity(self):
        self.mux.receive_auto(.1, .2)
        self.assertFalse(self.mux.receive_auto(math.nan, .2))
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_invalid_limits_fail_at_construction(self):
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                MotionMuxCore(manual_timeout=value)


class Nav2RemappingTest(unittest.TestCase):
    def test_scoped_final_output_remaps_keep_the_smoother_input_independent(self):
        # Resolve the exact rules through ROS itself, on an isolated localhost
        # domain. No commands are published and no robot nodes are launched.
        import os
        from unittest.mock import patch
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.context import Context
        from rclpy.node import Node

        context = Context()
        nodes = []
        with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
            rclpy.init(context=context, domain_id=229)
        scope = ['--ros-args',
                 '-r', 'velocity_smoother:cmd_vel_smoothed:=/smart_vision_amr/navigation_cmd_vel',
                 '-r', 'behavior_server:cmd_vel:=/smart_vision_amr/navigation_cmd_vel']
        try:
            smoother = Node('velocity_smoother', context=context, cli_args=scope + [
                '-r', 'cmd_vel:=cmd_vel_nav', '-r', 'cmd_vel_smoothed:=cmd_vel'])
            controller = Node('controller_server', context=context,
                              cli_args=scope + ['-r', 'cmd_vel:=cmd_vel_nav'])
            behavior = Node('behavior_server', context=context, cli_args=scope)
            nodes.extend([smoother, controller, behavior])
            output = smoother.create_publisher(Twist, 'cmd_vel_smoothed', 1)
            subscriber = smoother.create_subscription(Twist, 'cmd_vel', lambda _: None, 1)
            controller_output = controller.create_publisher(Twist, 'cmd_vel', 1)
            behavior_output = behavior.create_publisher(Twist, 'cmd_vel', 1)
            self.assertEqual(output.topic_name, '/smart_vision_amr/navigation_cmd_vel')
            self.assertEqual(behavior_output.topic_name, '/smart_vision_amr/navigation_cmd_vel')
            self.assertEqual(controller_output.topic_name, '/cmd_vel_nav')
            self.assertEqual(subscriber.topic_name, '/cmd_vel_nav')
        finally:
            for node in reversed(nodes):
                node.destroy_node()
            context.shutdown()

    def test_manual_watchdog_runs_while_simulation_clock_is_paused(self):
        import json
        import os
        import time
        from unittest.mock import patch
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from smart_vision_amr.motion_mux import MotionMuxNode

        context = Context()
        with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
            rclpy.init(context=context, domain_id=229)
        executor = SingleThreadedExecutor(context=context)
        mux = MotionMuxNode(context=context, use_sim_time=True)
        probe = Node('motion_mux_paused_clock_probe', context=context)
        outputs = []
        probe.create_subscription(Twist, '/cmd_vel',
                                  lambda msg: outputs.append(msg.linear.x), 1)
        executor.add_node(mux)
        executor.add_node(probe)
        try:
            deadline = time.monotonic()+3
            while mux.output_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertGreater(mux.output_pub.get_subscription_count(), 0)
            self.assertEqual(mux.get_clock().now().nanoseconds, 0)
            mux.core.manual_timeout = .15
            outputs.clear()
            control = String()
            control.data = json.dumps({'source_id': 'paused-clock-test', 'seq': 1,
                                       'mode': 'manual', 'linear': .1, 'angular': 0.0,
                                       'sent_at_ns': 0})
            mux.on_control(control)
            deadline = time.monotonic()+.4
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertIn(.1, outputs)
            self.assertEqual(outputs[-1], 0.0)
            self.assertEqual(mux.core.status()['mode'], 'hold')
            self.assertEqual(mux.get_clock().now().nanoseconds, 0)
        finally:
            executor.shutdown()
            probe.destroy_node()
            mux.destroy_node()
            context.shutdown()

    def test_duplicate_real_mux_endpoints_never_publish_a_handover_stop(self):
        import os
        import time
        from unittest.mock import patch
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.context import Context
        from rclpy.node import Node
        from smart_vision_amr.motion_mux import MotionMuxNode

        context = Context()
        with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
            rclpy.init(context=context, domain_id=229)
        mux = MotionMuxNode(context=context)
        duplicate = Node('motion_mux', context=context, enable_rosout=False)
        duplicate.create_publisher(Twist, '/cmd_vel', 1)
        try:
            deadline = time.monotonic()+3
            while (len(mux.get_publishers_info_by_topic('/cmd_vel')) < 2
                   and time.monotonic() < deadline):
                time.sleep(.01)
            self.assertEqual(len(mux.get_publishers_info_by_topic('/cmd_vel')), 2)
            mux._ownership_checked_at = -math.inf
            with patch.object(mux.output_pub, 'publish') as publish:
                mux.core.receive_auto(.1, .2)
                mux.publish_output()
                mux.core.mode = 'hold'
                mux.publish_output()
                publish.assert_not_called()
            self.assertFalse(mux.output_available)
            self.assertIn('Unmanaged /cmd_vel publisher:', mux.output_conflict)
        finally:
            duplicate.destroy_node()
            mux.destroy_node()
            context.shutdown()


class OutputOwnershipTest(unittest.TestCase):
    def harness(self, endpoints):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from smart_vision_amr.motion_mux import MotionMuxNode

        class Harness(MotionMuxNode):
            def __init__(self):
                self.core = MotionMuxCore()
                self.output_pub = Mock(topic_name='/cmd_vel')
                self.status_pub = Mock()
                self.output_available = False
                self.output_conflict = ''
                self.output_publishers = 0
                self._ownership_checked_at = -math.inf
                self.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
                self.get_publishers_info_by_topic = Mock(return_value=endpoints)

            def get_name(self):
                return 'motion_mux'

            def get_namespace(self):
                return '/'

        return Harness(), SimpleNamespace(node_name='motion_mux', node_namespace='/')

    def test_unmanaged_controller_disables_even_zero_velocity_publication(self):
        from types import SimpleNamespace
        node, own = self.harness([])
        foreign = SimpleNamespace(node_name='legacy_controller', node_namespace='/')
        node.get_publishers_info_by_topic.return_value = [own, foreign]
        node.publish_output()
        node.publish_status()
        node.output_pub.publish.assert_not_called()
        import json
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        self.assertFalse(status['available'])
        self.assertIn('Unmanaged /cmd_vel publisher:', status['reason'])
        self.assertIn('/legacy_controller', status['reason'])

    def test_a_second_identically_named_mux_also_disables_output(self):
        node, own = self.harness([])
        node.get_publishers_info_by_topic.return_value = [own, own]
        node.publish_output()
        self.assertFalse(node.output_available)
        node.output_pub.publish.assert_not_called()

    def test_missing_own_endpoint_or_graph_failure_never_grants_access(self):
        node, _ = self.harness([])
        node.publish_output()
        self.assertFalse(node.output_available)
        node._ownership_checked_at = -math.inf
        node.get_publishers_info_by_topic.side_effect = RuntimeError('graph unavailable')
        node.publish_output()
        self.assertFalse(node.output_available)
        node.output_pub.publish.assert_not_called()

    def test_only_one_local_endpoint_enables_the_output(self):
        node, own = self.harness([])
        node.get_publishers_info_by_topic.return_value = [own]
        node.core.receive_auto(.1, -.2)
        node.publish_output()
        message = node.output_pub.publish.call_args.args[0]
        self.assertTrue(node.output_available)
        self.assertEqual((message.linear.x, message.angular.z), (.1, -.2))

    def test_graph_inspection_is_cached_at_five_hertz(self):
        from unittest.mock import patch
        node, own = self.harness([])
        node.get_publishers_info_by_topic.return_value = [own]
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.0):
            node.publish_output()
            node.publish_status()
            node.publish_output()
        node.get_publishers_info_by_topic.assert_called_once_with('/cmd_vel')
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.21):
            node.publish_status()
        self.assertEqual(node.get_publishers_info_by_topic.call_count, 2)

    def test_reset_verification_forces_zero_even_if_a_delayed_command_reaches_core(self):
        node, own = self.harness([])
        node.get_publishers_info_by_topic.return_value = [own]
        node.pending_reset = {'applied': True}
        node.core.receive_auto(.2, .5)
        node.publish_output()
        message = node.output_pub.publish.call_args.args[0]
        self.assertEqual((message.linear.x, message.angular.z), (0.0, 0.0))
        node.publish_status()
        import json
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['output_owner'], 'safety_reset')
        self.assertEqual(status['output_linear'], 0.0)

    def test_duplicate_status_is_capped_but_transitions_and_service_ack_are_immediate(self):
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        node, own = self.harness([])
        node.get_publishers_info_by_topic.return_value = [own]
        node.service_source, node.service_seq = 'mux-service:test', 0
        node.pending_reset, node.control_pub = None, Mock()
        node.last_zero_published_at = None
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.0):
            self.assertTrue(node.publish_status())
            for _ in range(20):
                self.assertFalse(node.publish_status())
            self.assertEqual(node.status_pub.publish.call_count, 1)
            # State transitions bypass the cap even inside a 100 ms window.
            node.core.mode, node.core.estop_latched = 'estop', True
            self.assertTrue(node.publish_status())
            before = node.status_pub.publish.call_count
            self.assertTrue(node.on_emergency_stop(None, SimpleNamespace()).success)
            self.assertTrue(node.on_emergency_stop(None, SimpleNamespace()).success)
            self.assertEqual(node.status_pub.publish.call_count, before+2)


class LocalizationLeaseTest(unittest.TestCase):
    setUp = MotionMuxTest.setUp
    control = MotionMuxTest.control

    def ready(self, allowed=True):
        self.mux.receive_localization_health({'level': 'warning',
            'autonomous_allowed': False, 'allow_localization': allowed})

    def test_explicit_localization_is_yaw_only_capped_and_fails_closed(self):
        self.assertTrue(self.control('localization', angular=.8))
        self.assertEqual(self.mux.output(), (0., 0.))
        self.ready()
        self.assertEqual(self.mux.output(), (0., .25))
        self.assertFalse(self.control('localization', linear=.01, angular=.2))
        self.assertIn('yaw only', self.mux.status()['control_rejected_reason'])
        self.assertEqual(self.mux.output(), (0., .25))
        self.ready(False)
        self.assertEqual(self.mux.output(), (0., 0.))
        self.ready()
        self.now += .501
        self.assertEqual(self.mux.output(), (0., 0.))
        self.assertEqual(self.mux.status()['mode'], 'hold')

    def test_localization_health_expiry_blocks_rotation_even_during_live_lease(self):
        self.ready()
        self.control('localization', angular=.2)
        self.mux.localization_health_timeout = .1
        self.now += .11
        self.assertEqual(self.mux.output(), (0., 0.))
        self.assertFalse(self.mux.status()['allow_localization'])

    def test_manual_priority_and_explicit_hold_handoff(self):
        self.ready()
        self.control('manual', linear=.1)
        self.assertFalse(self.control('localization', angular=.2))
        self.assertFalse(self.control('localization', source='web-b', angular=.2))
        self.assertEqual(self.mux.output(), (.05, 0.))
        self.control('hold')
        self.assertTrue(self.control('localization', angular=.2))
        self.assertTrue(self.control('manual', source='web-b', linear=.1))
        self.assertEqual(self.mux.status()['source_id'], 'web-b')
        self.assertEqual(self.mux.output(), (.05, 0.))

    def test_localization_never_replays_auto_and_estop_always_wins(self):
        self.ready()
        self.control('localization', angular=.2)
        self.assertFalse(self.mux.receive_auto(.2, 0.))
        self.assertTrue(self.control('estop', source='other'))
        self.assertFalse(self.control('localization', angular=.2))
        self.assertEqual(self.mux.output(), (0., 0.))

    def test_receipts_are_once_per_packet_and_rejections_remain_visible(self):
        self.assertFalse(self.mux.receive_control([], 0))
        rejected = self.mux.status()['last_control_rejection']
        self.assertIn('JSON object', rejected['reason'])
        self.assertTrue(self.control('hold'))
        status = self.mux.status()
        self.assertEqual((status['control_received'], status['control_accepted'],
                          status['control_rejected']), (2, 1, 1))
        self.assertEqual(status['last_control_rejection'], rejected)
        self.assertEqual(status['control_seq'], self.seq)
        self.assertEqual(status['last_control']['origin'], 'core')
        packet = {'source_id': 'nav', 'seq': 1, 'sent_at_ns': 1,
                  'linear': .1, 'angular': 0.}
        self.control('auto')
        self.assertTrue(self.mux.receive_stamped_auto(packet, 1))
        self.assertEqual(self.mux.auto_received, 1)
        self.assertEqual(self.mux.last_auto['origin'], 'topic')
        self.mux.receive_auto(.1, 0.)
        self.assertEqual(self.mux.auto_received, 2)
        self.assertEqual(self.mux.last_auto['origin'], 'legacy')


class ControlDiagnosticsNodeTest(unittest.TestCase):
    harness = OutputOwnershipTest.harness

    def configured(self):
        from unittest.mock import Mock
        node, own = self.harness([])
        node.get_publishers_info_by_topic.return_value = [own]
        node.service_source = 'mux-service:test'
        node.service_seq = 0
        node.allow_topic_reset = False
        node.auto_started_ns = 0
        node.pending_reset = None
        node.control_pub = Mock()
        return node

    def test_malformed_precache_reset_and_service_loopback_receipts(self):
        import json
        from types import SimpleNamespace
        node = self.configured()
        node.on_control(SimpleNamespace(data='{'))
        node.on_control(SimpleNamespace(data=json.dumps({'source_id': 'web', 'seq': 1,
            'mode': 'reset', 'sent_at_ns': 0})))
        self.assertEqual(node.core.control_received, 2)
        self.assertEqual(node.core.last_control['origin'], 'topic')
        self.assertIn('service', node.core.last_control['reason'])
        node.on_control(SimpleNamespace(data=json.dumps({'source_id': node.service_source,
            'seq': 1, 'mode': 'reset', 'sent_at_ns': 0})))
        self.assertEqual(node.core.control_received, 2)
        node.on_emergency_stop(None, SimpleNamespace())
        self.assertEqual(node.core.control_received, 3)
        self.assertEqual(node.core.last_control['origin'], 'service')
        self.assertEqual(node.core.control_by_origin['topic'],
                         {'received': 2, 'accepted': 0, 'rejected': 2})
        self.assertEqual(node.core.control_by_origin['service'],
                         {'received': 1, 'accepted': 1, 'rejected': 0})
        node.on_navigation_control(SimpleNamespace(data='{'))
        self.assertEqual(node.core.auto_received, 1)
        self.assertIn('Malformed', node.core.last_auto['reason'])
        node.auto_started_ns = 2
        node.on_navigation_control(SimpleNamespace(data=json.dumps({'sent_at_ns': 1})))
        self.assertEqual(node.core.auto_received, 2)
        self.assertIn('startup', node.core.last_auto['reason'])

    def test_manual_heartbeat_counts_do_not_bypass_status_publish_cap(self):
        import json
        from types import SimpleNamespace
        from unittest.mock import patch
        node = self.configured()
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.):
            for seq in range(1, 101):
                node.on_control(SimpleNamespace(data=json.dumps({'source_id': 'web',
                    'seq': seq, 'mode': 'manual', 'linear': .1, 'sent_at_ns': 0})))
            self.assertEqual(node.status_pub.publish.call_count, 1)
            self.assertEqual(node.core.control_received, 100)
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.11):
            self.assertTrue(node.publish_status())
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['control_received'], 100)
        self.assertEqual(status['control_seq'], 100)

    def test_hold_ack_is_a_recent_zero_publication_not_physical_standstill(self):
        import json
        from unittest.mock import patch
        node = self.configured()
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.):
            node.core.receive_control({'source_id': 'web', 'seq': 1,
                'mode': 'hold', 'sent_at_ns': 0}, 0)
            node.publish_status()
            self.assertFalse(json.loads(node.status_pub.publish.call_args.args[0].data)['stop_acknowledged'])
            node.publish_output()
            node.publish_status(force=True)
            self.assertTrue(json.loads(node.status_pub.publish.call_args.args[0].data)['stop_acknowledged'])
        with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=10.21):
            node.publish_status()
            self.assertFalse(json.loads(node.status_pub.publish.call_args.args[0].data)['stop_acknowledged'])


if __name__ == '__main__':
    unittest.main()
