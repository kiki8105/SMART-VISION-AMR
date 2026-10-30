"""Battery telemetry status, web role visibility, and source freshness.

Pure state/HTTP tests need no ROS. Optional ROS adapter tests use localhost-only
domain 213, call callbacks directly, and never spin or publish a robot command.
"""
import http.client
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, MissionHTTPServer,
    MissionHTTPRequestHandler, build_parser, create_ros_bridge,
)

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import BatteryState
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class PowerStateTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.app = MissionApplication(DEFAULT_MARKERS, 213, False,
                                      Path(self.temporary.name)/'settings.json')

    def tearDown(self):
        self.temporary.cleanup()

    def test_no_telemetry_is_waiting_not_a_claim_that_power_is_off(self):
        self.assertEqual(self.app.power_state(), {
            'status': 'waiting', 'voltage': None, 'percentage': None,
            'age': None, 'source': '/battery_state',
        })

    def test_valid_voltage_reports_online_and_preserves_driver_percentage(self):
        with patch('smart_vision_amr.web_mission_server.time.monotonic', return_value=100.):
            self.app.update_battery(11.8, .72)
        power = self.app.power_state(now=101.25)
        self.assertEqual(power['status'], 'online')
        self.assertEqual(power['voltage'], 11.8)
        self.assertEqual(power['percentage'], .72)
        self.assertEqual(power['age'], 1.25)

    def test_unknown_or_invalid_percentage_is_not_guessed_from_voltage(self):
        for percentage in (None, float('nan'), float('inf'), -.1, 1.1):
            with self.subTest(percentage=percentage):
                self.app.update_battery(11.7, percentage)
                self.assertEqual(self.app.power_state()['status'], 'online')
                self.assertIsNone(self.app.power_state()['percentage'])
        for percentage in (0., 1.):
            with self.subTest(percentage=percentage):
                self.app.update_battery(11.7, percentage)
                self.assertEqual(self.app.power_state()['percentage'], percentage)

    def test_invalid_voltage_is_visible_and_strict_json_safe(self):
        for voltage in (float('nan'), float('inf'), -float('inf'), 0., -1.):
            with self.subTest(voltage=voltage):
                self.app.update_battery(voltage)
                power = self.app.power_state()
                self.assertEqual(power['status'], 'invalid')
                self.assertIsNone(power['voltage'])
                self.assertIsNotNone(power['age'])
                json.dumps(self.app.state(), allow_nan=False)
                json.dumps(self.app.inventory_state(7), allow_nan=False)

    def test_stale_status_retains_last_reading_and_fresh_packet_recovers(self):
        with patch('smart_vision_amr.web_mission_server.time.monotonic', return_value=100.):
            self.app.update_battery(11.8, .72)
        self.assertEqual(self.app.power_state(now=104.99)['status'], 'online')
        for now in (105., 200.):
            power = self.app.power_state(now=now)
            self.assertEqual(power['status'], 'stale')
            self.assertEqual(power['voltage'], 11.8)
            self.assertEqual(power['percentage'], .72)
            self.assertEqual(power['age'], now-100.)
        with patch('smart_vision_amr.web_mission_server.time.monotonic', return_value=200.):
            self.app.update_battery(11.6)
        self.assertEqual(self.app.power_state(now=200.)['status'], 'online')
        self.assertEqual(self.app.power_state(now=200.)['voltage'], 11.6)

    def test_reset_does_not_reuse_previous_domain_battery(self):
        self.app.update_battery(12.2, .95)
        self.app.reset_power()
        power = self.app.power_state()
        self.assertEqual(power['status'], 'waiting')
        self.assertIsNone(power['voltage'])
        self.assertIsNone(power['percentage'])
        self.assertIsNone(power['age'])

    def test_operator_and_inventory_states_share_read_only_power_status(self):
        with patch('smart_vision_amr.web_mission_server.time.monotonic', return_value=100.):
            self.app.update_battery(11.9, .8)
            operator = self.app.state()['robot']['power']
            manager = self.app.inventory_state(7)['service']['power']
        self.assertEqual(operator, manager)
        self.assertEqual(operator['status'], 'online')
        operator['voltage'] = 0.
        self.assertEqual(self.app.power_state(now=100.)['voltage'], 11.9)

    def test_http_exposes_power_to_each_role_without_expanding_operator_access(self):
        server = MissionHTTPServer(('127.0.0.1', 0), self.app,
                                   Path(self.temporary.name))
        thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=.01),
                                  daemon=True)

        def request(method, path, payload=None, cookie=None):
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port,
                                                    timeout=3)
            headers = {'Content-Type': 'application/json'}
            if cookie:
                headers['Cookie'] = cookie
            connection.request(method, path, None if payload is None else json.dumps(payload),
                               headers)
            response = connection.getresponse()
            status, data, returned = response.status, json.loads(response.read()), dict(response.getheaders())
            connection.close()
            return status, data, returned

        with patch.object(MissionHTTPRequestHandler, 'local', property(lambda _: False)):
            thread.start()
            try:
                status, _, headers = request('POST', '/api/pair', {'code': self.app.pairing_code})
                self.assertEqual(status, 200)
                operator_cookie = headers['Set-Cookie'].split(';', 1)[0]
                status, _, headers = request('POST', '/api/inventory/pair', {
                    'user_id': 7, 'code': self.app.inventory_join_code})
                self.assertEqual(status, 200)
                manager_cookie = headers['Set-Cookie'].split(';', 1)[0]
                self.app.update_battery(11.9, .8)
                status, operator, _ = request('GET', '/api/state', cookie=operator_cookie)
                self.assertEqual(status, 200)
                status, manager, _ = request('GET', '/api/inventory/state', cookie=manager_cookie)
                self.assertEqual(status, 200)
                for power in (operator['robot']['power'], manager['service']['power']):
                    self.assertEqual(power['status'], 'online')
                    self.assertEqual(power['voltage'], 11.9)
                self.assertEqual(request('GET', '/api/state', cookie=manager_cookie)[0], 401)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(2)

    def test_cli_uses_battery_state_by_default_and_accepts_custom_topic(self):
        self.assertEqual(build_parser().parse_args([]).battery_topic, '/battery_state')
        options = build_parser().parse_args(['--battery-topic', '/robot_2/battery_state'])
        self.assertEqual(options.battery_topic, '/robot_2/battery_state')

    def test_physical_departure_requires_fresh_valid_battery_when_enabled(self):
        self.app.require_battery_health = True
        self.app.minimum_battery_voltage = 10.8
        self.assertFalse(self.app.readiness()['ready'])
        self.assertTrue(any('/battery_state' in item for item in self.app.readiness()['missing']))
        self.app.update_battery(10.79)
        self.assertTrue(any('10.80V' in item for item in self.app.readiness()['missing']))
        self.app.update_battery(10.80000019)
        self.assertFalse(any('10.80V' in item for item in self.app.readiness()['missing']))
        self.app.update_battery(10.97)
        self.assertFalse(any('10.80V' in item for item in self.app.readiness()['missing']))
        self.app.update_battery(11.9)
        self.assertFalse(any('/battery_state' in item or '10.80V' in item
                             for item in self.app.readiness()['missing']))
        self.app.update_battery(float('nan'))
        self.assertTrue(any('/battery_state' in item for item in self.app.readiness()['missing']))
        self.app.update_battery(11.9)
        self.app.battery_at -= 6.0
        self.assertTrue(any('/battery_state' in item for item in self.app.readiness()['missing']))


@unittest.skipUnless(HAS_ROS, 'source ROS Humble setup for battery adapter tests')
class PowerROSAdapterTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.app = MissionApplication(DEFAULT_MARKERS, 213, False,
                                      Path(self.temporary.name)/'settings.json')
        self.environment = patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'})
        self.environment.start()
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=213,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.options = SimpleNamespace(camera_topic='/test_power_camera',
                                       base_frame='test_power_base')
        self.node = create_ros_bridge(self.app, self.options, self.context)

    def tearDown(self):
        self.node.destroy_node()
        self.context.shutdown()
        self.environment.stop()
        self.temporary.cleanup()

    def message(self, age=0., stamp=None, voltage=11.9, percentage=.8):
        message = BatteryState()
        if stamp is None:
            stamp = self.node.get_clock().now().nanoseconds-round(age*1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
        message.voltage = voltage
        message.percentage = percentage
        return message

    def test_subscription_uses_sensor_qos_and_legacy_options_default(self):
        subscriptions = [item for item in self.node.subscriptions if item.msg_type is BatteryState]
        self.assertEqual(len(subscriptions), 1)
        subscription = subscriptions[0]
        self.assertEqual(subscription.topic_name, '/battery_state')
        self.assertEqual(subscription.qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(subscription.qos_profile.durability, DurabilityPolicy.VOLATILE)

    def test_custom_topic_is_reflected_in_power_source(self):
        self.node.destroy_node()
        self.options.battery_topic = '/test_power/battery_state'
        self.node = create_ros_bridge(self.app, self.options, self.context)
        subscriptions = [item for item in self.node.subscriptions if item.msg_type is BatteryState]
        self.assertEqual(subscriptions[0].topic_name, self.options.battery_topic)
        self.assertEqual(self.app.power_state()['source'], self.options.battery_topic)

    def test_old_zero_or_far_future_headers_do_not_make_power_online(self):
        for message in (self.message(age=6.), self.message(age=-1.), self.message(stamp=0)):
            with self.subTest(stamp=message.header.stamp):
                self.node.on_battery(message)
                self.assertEqual(self.app.power_state()['status'], 'waiting')

    def test_replayed_or_out_of_order_packets_do_not_refresh_receipt_age(self):
        message = self.message()
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        with patch('smart_vision_amr.web_mission_server.time.monotonic', return_value=100.):
            self.node.on_battery(message)
        for duplicate in (message, self.message(stamp=stamp-10000000, voltage=12.2)):
            with patch('smart_vision_amr.web_mission_server.time.monotonic', return_value=104.):
                self.node.on_battery(duplicate)
        power = self.app.power_state(now=105.)
        self.assertEqual(power['status'], 'stale')
        self.assertEqual(power['voltage'], float(message.voltage))
        self.assertEqual(power['age'], 5.)

    def test_invalid_fresh_packet_then_new_valid_packet_recovers(self):
        message = self.message(voltage=float('nan'), percentage=float('nan'))
        self.node.on_battery(message)
        self.assertEqual(self.app.power_state()['status'], 'invalid')
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        self.node.on_battery(self.message(stamp=stamp+1, voltage=11.7, percentage=.7))
        self.assertEqual(self.app.power_state()['status'], 'online')
        self.assertAlmostEqual(self.app.power_state()['voltage'], 11.7, places=4)


if __name__ == '__main__':
    unittest.main()
