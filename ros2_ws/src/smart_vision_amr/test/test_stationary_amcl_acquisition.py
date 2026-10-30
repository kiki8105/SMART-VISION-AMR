"""Stationary reacquisition requests real measurements, never confidence or motion."""
from collections import deque
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException

from smart_vision_amr.localization_health import LocalizationHealth
from test_localization_nomotion_refresh import setup_node, request


def uncertain():
    node, clock = setup_node()
    node.core.acquired = False
    node.position_std = .446
    node.odom.twist.twist.linear.x = node.odom.twist.twist.angular.z = 0.
    node.scan.header.frame_id = 'base_scan'
    return node, clock


def fresh(node, clock, seconds):
    clock['ros'] = round(seconds*1e9)
    node.scan.header.stamp.sec, node.scan.header.stamp.nanosec = divmod(clock['ros'], 1_000_000_000)
    node.odom.header.stamp = node.scan.header.stamp
    node._tf.return_value.header.stamp = node.scan.header.stamp
    node.scan_at = node.odom_at = seconds


def test_high_existing_covariance_can_refresh_without_turning_or_faking_a_new_pose():
    node, _ = uncertain()
    epoch = (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std)
    node.core.stop_latched = True
    node.core.amcl_motion_without_update_seconds = 4.1
    request(node)
    assert node.refresh_mode == 'stationary' and node.stationary_refresh_requests == 1
    node.no_motion_future.set_result(object())
    request(node, 10.1)
    assert node.no_motion_responses == 1
    assert epoch == (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std)
    assert node.core.stop_latched and node.core.amcl_motion_without_update_seconds == 4.1


@pytest.mark.parametrize('fault', ['scan_stale', 'scan_invalid', 'odom_stale', 'odom_invalid',
    'clock_future', 'no_odom_tf', 'no_scan_tf', 'stamped_tf_skew', 'nan_twist', 'moving'])
def test_uncertain_idle_refresh_needs_fresh_physical_evidence(fault):
    node, _ = uncertain()
    if fault == 'scan_stale': node.scan_at = 9.
    if fault == 'scan_invalid': node.scan_valid = False
    if fault == 'odom_stale': node.odom_at = 9.
    if fault == 'odom_invalid': node.odom_valid = False
    if fault == 'clock_future': node.scan.header.stamp.sec = 11
    if fault == 'no_odom_tf': node._tf.side_effect = TransformException('odom absent')
    if fault == 'no_scan_tf':
        original = node._tf.return_value
        node._tf.side_effect = lambda target, source, *stamp: (original if target == 'odom' else
            (_ for _ in ()).throw(TransformException('measurement transform absent')))
    if fault == 'stamped_tf_skew': node._tf.return_value.header.stamp.nanosec = 30_000_000
    if fault == 'nan_twist': node.odom.twist.twist.angular.z = float('nan')
    if fault == 'moving':
        node.odom.twist.twist.linear.x = .1
        node.amcl_at = 9.8  # Fresh filter update; neither stationary nor stale-motion request.
    request(node)
    node.no_motion_client.call_async.assert_not_called()


def test_healthy_idle_tracking_is_not_forced_to_update():
    node, _ = uncertain()
    node.core.acquired, node.position_std, node.yaw_std = True, .1, .1
    request(node)
    node.no_motion_client.call_async.assert_not_called()


def test_stationary_refresh_has_count_time_and_fresh_measurement_bounds():
    node, clock = uncertain()
    values = {'stationary_refresh_max_requests': 2, 'stationary_refresh_timeout': 30.}
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    request(node)
    node.no_motion_future.set_result(object())
    # A new receipt time does not make the same scan a new observation.
    node.scan_at = node.odom_at = 12.
    request(node, 12.)
    assert node.refresh_state == 'WAITING_FOR_NEW_SCAN'
    fresh(node, clock, 12.)
    request(node, 12.)
    node.no_motion_future.set_result(object())
    fresh(node, clock, 14.)
    request(node, 14.)
    assert node.no_motion_client.call_async.call_count == 2
    assert node.refresh_state == 'STATIONARY_LIMIT_REACHED'
    node.stationary_refresh_requests = 0
    node.stationary_refresh_started = 0.
    fresh(node, clock, 40.)
    request(node, 40.)
    assert node.no_motion_client.call_async.call_count == 2
    assert node.refresh_state == 'STATIONARY_LIMIT_REACHED'


def lagging_measurement_tf():
    node, clock = uncertain()
    older = deepcopy(node.scan)
    node.scan.header.stamp.nanosec = 100_000_000
    node.scan_at = 10.1
    node.odom_at = 10.15
    node.odom.header.stamp.nanosec = 150_000_000
    clock['ros'] = 10_150_000_000
    node.agreement = node.agreement_at = node.agreement_stamp = None
    node.agreement_pending = deque([(10_000_000_000, 10., older),
                                     (10_100_000_000, 10.1, node.scan)])
    settings = {'agreement_tolerance': .18}
    node.get_parameter = lambda name: SimpleNamespace(value=settings[name])

    def transform(target, source, stamp=None):
        result = TransformStamped()
        result.transform.rotation.w = 1.
        if stamp is None:
            result.header.stamp = node.odom.header.stamp
        else:
            if stamp.nanosec >= 100_000_000:
                raise TransformException('newest scan ahead of TF')
            result.header.stamp = stamp
        return result

    node._tf = transform
    with patch('smart_vision_amr.localization_health.scan_points', return_value=[]), \
            patch('smart_vision_amr.localization_health.scan_map_agreement', return_value=.9):
        LocalizationHealth.update_agreement(node, 10.15)
    return node, clock


def test_stationary_refresh_uses_fresh_tf_ready_measurement_when_newest_scan_outruns_tf():
    node, _ = lagging_measurement_tf()
    epoch = (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std)
    assert node.agreement_stamp == 10_000_000_000
    request(node, 10.15)
    assert node.no_motion_client.call_async.call_count == 1
    assert node.refresh_mode == 'stationary'
    assert node.last_refresh_scan_stamp == 10_000_000_000
    assert node.last_refresh_scan_stamp != 10_100_000_000
    assert epoch == (node.amcl_at, node.last_amcl_stamp_ns, node.position_std, node.yaw_std)


@pytest.mark.parametrize('fault', ['receipt_stale', 'header_stale', 'scan_invalid', 'odom_invalid'])
def test_tf_ready_fallback_never_revives_stale_or_invalid_physical_inputs(fault):
    node, _ = lagging_measurement_tf()
    if fault == 'receipt_stale': node.agreement_scan_at = 9.
    if fault == 'header_stale': node.agreement_scan.header.stamp.sec = 9
    if fault == 'scan_invalid': node.scan_valid = False
    if fault == 'odom_invalid': node.odom_valid = False
    request(node, 10.15)
    node.no_motion_client.call_async.assert_not_called()


@pytest.mark.parametrize('reset', ['initialpose', 'map', 'invalid_map'])
def test_map_or_initialpose_discards_tf_ready_refresh_proof(reset):
    node, _ = lagging_measurement_tf()
    if reset == 'initialpose':
        LocalizationHealth.on_initialpose(node, object())
    else:
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = message.info.height = 1
        message.info.resolution = .05
        message.info.origin.orientation.w = 1.
        message.data = [0] if reset == 'map' else []
        LocalizationHealth.on_map(node, message)
    assert node.agreement_scan is None and node.agreement_scan_at is None
    assert node.stationary_refresh_scan_stamp is None
