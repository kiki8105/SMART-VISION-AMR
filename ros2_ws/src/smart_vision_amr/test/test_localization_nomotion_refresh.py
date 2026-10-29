"""Real AMCL refresh requests; fake futures/TF only, no ROS context or actuator."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from smart_vision_amr.localization_health import LocalizationHealth
from smart_vision_amr.localization_health_math import LocalizationHealthCore


def setup_node():
    clock = {'ros': 10_000_000_000}
    scan = LaserScan()
    scan.header.stamp.sec = 10
    odom = Odometry()
    odom.header.stamp.sec = 10
    odom.pose.pose.orientation.w = 1.
    odom.twist.twist.linear.x = .03
    transform = TransformStamped()
    transform.header.stamp.sec = 10
    transform.transform.rotation.w = 1.
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = lambda _: Future()
    node = SimpleNamespace(
        no_motion_future=None, no_motion_started_at=None, last_no_motion_at=None,
        no_motion_requests=0, no_motion_responses=0, no_motion_timeouts=0, no_motion_errors=0,
        no_motion_client=client, core=LocalizationHealthCore(), position_std=.105, yaw_std=.08,
        amcl_at=8., last_amcl_stamp_ns=8_000_000_000, previous_amcl=None,
        scan=scan, scan_at=10., scan_valid=True, odom=odom, odom_at=10., odom_valid=True,
        map_distance=object(), map_metadata=object(), map_frame='map', odom_frame='odom',
        base_frame='base_footprint', _tf=Mock(return_value=transform),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock['ros'])))
    return node, clock


def request(node, at=10.):
    LocalizationHealth.request_initial_covariance(node, at)


def test_slow_measured_motion_requests_real_filter_update_without_forging_pose_epoch():
    node, _ = setup_node()
    epoch = (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std)
    request(node)
    assert node.no_motion_client.call_async.call_count == 1
    assert node.no_motion_requests == 1
    assert (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std) == epoch
    node.no_motion_future.set_result(object())
    request(node, 10.1)
    assert node.no_motion_responses == 1
    assert node.no_motion_client.call_async.call_count == 1  # Two second minimum.
    assert (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std) == epoch


@pytest.mark.parametrize('fault', ['idle', 'fresh_amcl', 'fresh_header', 'scan_stale', 'scan_invalid',
    'scan_old_header', 'odom_stale', 'odom_invalid', 'odom_old_header', 'no_map', 'no_tf',
    'old_tf', 'invalid_tf', 'service_missing', 'bad_twist'])
def test_periodic_refresh_needs_actual_motion_and_fresh_independent_evidence(fault):
    node, _ = setup_node()
    if fault == 'idle': node.odom.twist.twist.linear.x = 0.
    if fault == 'fresh_amcl': node.amcl_at = 9.5
    if fault == 'fresh_header': node.last_amcl_stamp_ns = 9_500_000_000
    if fault == 'scan_stale': node.scan_at = 9.
    if fault == 'scan_invalid': node.scan_valid = False
    if fault == 'scan_old_header': node.scan.header.stamp.sec = 9
    if fault == 'odom_stale': node.odom_at = 9.
    if fault == 'odom_invalid': node.odom_valid = False
    if fault == 'odom_old_header': node.odom.header.stamp.sec = 9
    if fault == 'no_map': node.map_distance = None
    if fault == 'no_tf': node._tf.side_effect = TransformException('no TF')
    if fault == 'old_tf': node._tf.return_value.header.stamp.sec = 8
    if fault == 'invalid_tf': node._tf.return_value.transform.rotation.w = 0.
    if fault == 'service_missing': node.no_motion_client.service_is_ready.return_value = False
    if fault == 'bad_twist': node.odom.twist.twist.linear.x = float('nan')
    request(node)
    node.no_motion_client.call_async.assert_not_called()


def test_one_pending_request_has_one_second_deadline_and_does_not_stall_future_retries():
    node, clock = setup_node()
    request(node)
    pending = node.no_motion_future
    node.scan_at = node.odom_at = 10.5
    request(node, 10.5)
    assert node.no_motion_future is pending
    assert node.no_motion_client.call_async.call_count == 1
    node.scan_at = node.odom_at = 11.
    request(node, 11.)
    assert pending.cancelled() and node.no_motion_future is None
    node.no_motion_client.remove_pending_request.assert_called_once_with(pending)
    assert node.no_motion_timeouts == 1
    clock['ros'] = 12_000_000_000
    node.scan.header.stamp.sec = node.odom.header.stamp.sec = 12
    node._tf.return_value.header.stamp.sec = 12
    node.scan_at = node.odom_at = 12.
    request(node, 12.)
    assert node.no_motion_client.call_async.call_count == 2
    assert node.amcl_at == 8. and node.last_amcl_stamp_ns == 8_000_000_000


def test_service_exception_is_diagnostic_not_a_guard_epoch_refresh():
    node, _ = setup_node()
    node.no_motion_client.call_async.side_effect = RuntimeError('server unavailable')
    request(node)
    assert node.no_motion_future is None and node.no_motion_errors == 1
    assert node.amcl_at == 8.


def test_only_a_real_new_amcl_message_advances_epoch_after_service_response(monkeypatch):
    node, _ = setup_node()
    request(node)
    node.no_motion_future.set_result(object())
    request(node, 10.1)
    assert node.amcl_at == 8.
    # A replay remains a replay even after the no-motion service responded.
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 8
    msg.pose.pose.orientation.w = 1.
    LocalizationHealth.on_amcl(node, msg)
    assert node.amcl_at == 8.
    node.odom = None
    msg.header.stamp.sec = 10
    monkeypatch.setattr('smart_vision_amr.localization_health.time.monotonic', lambda: 10.2)
    LocalizationHealth.on_amcl(node, msg)
    assert node.amcl_at == 10.2 and node.last_amcl_stamp_ns == 10_000_000_000


def test_bootstrap_request_still_works_without_motion_pose_or_tf():
    node, _ = setup_node()
    node.position_std = node.amcl_at = node.last_amcl_stamp_ns = None
    node.odom = None
    node._tf.side_effect = TransformException('not localized yet')
    request(node)
    assert node.no_motion_client.call_async.call_count == 1


def test_filter_request_or_response_never_resets_motion_budget_or_safety_latch():
    node, _ = setup_node()
    node.core.amcl_motion_without_update_seconds = 4.2
    node.core.stop_latched = True
    request(node)
    node.no_motion_future.set_result(object())
    request(node, 10.1)
    assert node.core.amcl_motion_without_update_seconds == 4.2
    assert node.core.stop_latched
    assert node.amcl_at == 8. and node.last_amcl_stamp_ns == 8_000_000_000


def test_runtime_parameters_cannot_flood_or_leave_a_request_unbounded():
    node, _ = setup_node()
    values = {'nomotion_update_interval': .05, 'nomotion_update_trigger_age': 1.,
              'nomotion_request_timeout': 100.}
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    request(node)
    pending = node.no_motion_future
    node.scan_at = node.odom_at = 11.
    request(node, 11.)
    assert pending.cancelled() and node.no_motion_future is None
    assert node.no_motion_client.call_async.call_count == 1
    # The configured small interval cannot create another request before 2 s.
    node.scan_at = node.odom_at = 11.1
    request(node, 11.1)
    assert node.no_motion_client.call_async.call_count == 1


def test_configured_trigger_age_can_delay_periodic_refresh_without_fabricating_update():
    node, _ = setup_node()
    values = {'nomotion_update_interval': 2., 'nomotion_update_trigger_age': 3.,
              'nomotion_request_timeout': 1.}
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    request(node)
    node.no_motion_client.call_async.assert_not_called()
    assert node.amcl_at == 8.
