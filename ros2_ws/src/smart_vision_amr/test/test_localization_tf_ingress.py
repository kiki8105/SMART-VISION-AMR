"""Health-only TF reader QoS and ingress diagnostics, without a ROS context."""
from copy import deepcopy
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException

from smart_vision_amr.localization_health import HealthTransformListener, LocalizationHealth
from smart_vision_amr.localization_health_math import LocalizationHealthCore
from test_localization_health import healthy
from test_localization_tf_queue import node_with_queue, update


class SubscriptionRecorder:
    def __init__(self):
        self.subscriptions = {}

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        result = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        self.subscriptions[topic] = result
        return result

    def destroy_subscription(self, subscription):
        pass


def transform(parent, child, seconds):
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = parent, child
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(seconds*1e9), 1_000_000_000)
    message.transform.rotation.w = 1.
    return message


@pytest.mark.parametrize('best_effort', [True, False])
def test_dynamic_qos_can_switch_while_static_tf_remains_reliable_and_latched(best_effort):
    node, buffer = SubscriptionRecorder(), Mock()
    listener = HealthTransformListener(buffer, node, best_effort=best_effort)
    dynamic, static = node.subscriptions['/tf'].qos, node.subscriptions['/tf_static'].qos
    assert dynamic.reliability == (ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE)
    assert dynamic.depth == (20 if best_effort else 100)
    assert dynamic.durability == DurabilityPolicy.VOLATILE
    assert static.reliability == ReliabilityPolicy.RELIABLE
    assert static.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert static.depth == 100
    listener.unregister()


def test_dynamic_ingress_reports_receipt_and_header_ages_without_modifying_or_retiming_tf():
    clock = {'steady': 100.}
    buffer = Mock()
    listener = HealthTransformListener(buffer, SubscriptionRecorder(), clock=lambda: clock['steady'])
    message = TFMessage(transforms=[transform('map', 'odom', 11.),
                                     transform('odom', 'base_footprint', 9.98)])
    original = deepcopy(message)
    listener.callback(message)
    metrics = listener.diagnostics(100.25, 10_250_000_000)
    assert metrics['dynamic_tf_received_count'] == 1
    assert metrics['dynamic_tf_transform_count'] == 2
    assert metrics['dynamic_tf_map_odom_header_age_seconds'] == pytest.approx(-.75)
    assert metrics['dynamic_tf_odom_base_header_age_seconds'] == pytest.approx(.27)
    assert metrics['dynamic_tf_odom_base_receive_age_seconds'] == pytest.approx(.25)
    clock['steady'] = 100.3
    listener.callback(message)  # A new receipt does not make an old TF stamp new.
    metrics = listener.diagnostics(100.3, 10_300_000_000)
    assert metrics['dynamic_tf_odom_base_receive_age_seconds'] == 0.
    assert metrics['dynamic_tf_odom_base_header_age_seconds'] == pytest.approx(.32)
    assert metrics['dynamic_tf_odom_base_count'] == 2
    assert message == original
    assert buffer.set_transform.call_args_list[1].args[0] is message.transforms[1]
    assert buffer.set_transform.call_args_list[3].args[0] is message.transforms[1]
    listener.unregister()


def test_static_messages_do_not_fabricate_dynamic_ingress_evidence():
    buffer = Mock()
    listener = HealthTransformListener(buffer, SubscriptionRecorder(), clock=lambda: 100.)
    message = TFMessage(transforms=[transform('base_footprint', 'base_scan', 0.)])
    listener.static_callback(message)
    metrics = listener.diagnostics(100., 10_000_000_000)
    assert metrics['dynamic_tf_received_count'] == 0
    assert metrics['dynamic_tf_odom_base_header_age_seconds'] is None
    assert metrics['dynamic_tf_map_odom_receive_age_seconds'] is None
    buffer.set_transform.assert_not_called()
    buffer.set_transform_static.assert_called_once_with(message.transforms[0], 'default_authority')
    listener.unregister()


def test_ingress_tracks_only_the_configured_map_odom_and_base_links():
    listener = HealthTransformListener(Mock(), SubscriptionRecorder(), map_frame='room',
        odom_frame='wheel_odom', base_frame='chassis', clock=lambda: 100.)
    listener.callback(TFMessage(transforms=[transform('/room', '/wheel_odom', 10.),
        transform('wheel_odom', 'chassis', 10.), transform('odom', 'base_footprint', 9.)]))
    metrics = listener.diagnostics(100., 10_000_000_000)
    assert metrics['dynamic_tf_transform_count'] == 3
    assert metrics['dynamic_tf_map_odom_count'] == metrics['dynamic_tf_odom_base_count'] == 1
    assert metrics['dynamic_tf_odom_base_header_age_seconds'] == 0.
    listener.unregister()


def test_fresh_dynamic_receipts_never_substitute_for_missing_measurement_time_tf():
    listener = HealthTransformListener(Mock(), SubscriptionRecorder(), clock=lambda: 10.4)
    listener.callback(TFMessage(transforms=[transform('map', 'odom', 11.4),
                                           transform('odom', 'base_footprint', 10.4)]))
    node, _, _ = node_with_queue()
    wrong = transform('map', 'base_footprint', 8.)
    node._tf = Mock(return_value=wrong)
    update(node, 10.4)
    assert node.agreement is None
    assert node.agreement_tf_state == 'MEASUREMENT_TF_TIMEOUT'
    assert listener.diagnostics(10.4, 10_400_000_000)['dynamic_tf_odom_base_receive_age_seconds'] == 0.
    listener.unregister()


def test_fresh_ingress_cannot_release_a_stale_lookup_or_a_safety_latch():
    listener = HealthTransformListener(Mock(), SubscriptionRecorder(), clock=lambda: 100.)
    listener.callback(TFMessage(transforms=[transform('map', 'odom', 11.),
                                           transform('odom', 'base_footprint', 10.)]))
    metrics = healthy(map_tf_age_seconds=1.6, odom_tf_valid=False)
    metrics.update(listener.diagnostics(100., 10_000_000_000))
    core = LocalizationHealthCore()
    core.evaluate(0., healthy())
    assert core.evaluate(2.1, healthy())['autonomous_allowed']
    assert not core.evaluate(2.2, metrics)['autonomous_allowed']
    status = core.evaluate(2.8, metrics)
    assert status['stop_latched']
    assert {'map_tf_unavailable', 'odom_tf_unavailable'} <= set(status['reasons'])
    assert core.position_warning == .35 and core.tf_timeout == 1.
    listener.unregister()


def test_tick_reports_lookup_errors_separately_from_fresh_ingress_and_keeps_checks_closed():
    listener = HealthTransformListener(Mock(), SubscriptionRecorder(), clock=lambda: 10.15)
    listener.callback(TFMessage(transforms=[transform('map', 'odom', 11.),
                                           transform('odom', 'base_footprint', 10.)]))
    node, clock, _ = node_with_queue()
    node.odom_frame = 'odom'
    node.request_initial_covariance = node.update_agreement = Mock()
    node.tf_listener = listener
    node.scan_valid = node.odom_valid = True
    node.scan_at = node.odom_at = node.amcl_at = 10.1
    node.odom = SimpleNamespace(header=SimpleNamespace(stamp=node.scan.header.stamp),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=0.))))
    node.position_std = node.yaw_std = .1
    node.last_amcl_stamp_ns = 10_100_000_000
    node.jump_until = 0.
    node.no_motion_requests = node.no_motion_responses = node.no_motion_timeouts = node.no_motion_errors = 0
    node.stationary_refresh_requests = 0
    node.refresh_mode, node.refresh_state = 'idle', 'WAITING_FOR_INPUTS'
    node.agreement_tf_wait_ms = None
    node.received_intervals = node.header_intervals = {'scan': .1, 'odom': .1}
    node.last_received_at = {'scan': 10.1, 'odom': 10.1}
    node.last_received_stamp = {'scan': 10_100_000_000, 'odom': 10_100_000_000}
    node.rejected_count = {'scan': 0, 'odom': 0}
    node.rejection_reason = node.last_rejection_reason = {'scan': '', 'odom': ''}
    node.last_level = node.last_estop_at = None
    node.get_logger = lambda: Mock()
    node.publisher, node.publish_estop = Mock(), Mock()
    node._tf = Mock(side_effect=TransformException('listener buffer has no transform'))
    with patch('smart_vision_amr.localization_health.time.monotonic', return_value=10.15):
        LocalizationHealth.on_tick(node)
    status = json.loads(node.publisher.publish.call_args.args[0].data)
    metrics = status['metrics']
    assert metrics['dynamic_tf_received_count'] == 1
    assert metrics['map_tf_lookup_error'] == metrics['odom_tf_lookup_error'] == 'listener buffer has no transform'
    assert metrics['map_tf_age_seconds'] is None and not metrics['odom_tf_valid']
    assert not status['autonomous_allowed']
    listener.unregister()
