"""Delayed measurement TF may catch up; newest scans must not starve older ones."""
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from smart_vision_amr.localization_health import LocalizationHealth
from smart_vision_amr.localization_health_math import LocalizationHealthCore
from test_localization_health import healthy


def scan(stamp):
    msg = LaserScan()
    msg.header.frame_id = 'base_scan'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(stamp*1e9), 1_000_000_000)
    msg.angle_increment, msg.range_min, msg.range_max = .01, .1, 5.
    msg.ranges = [1.]*40
    return msg


def node_with_queue():
    clock = {'ros': 10_150_000_000, 'ready': 10_100_000_000}
    old, newest = scan(10.0), scan(10.1)
    node = SimpleNamespace(scan=newest, scan_at=10.1, map_distance=object(), map_metadata=object(),
        base_frame='base_footprint', map_frame='map', core=LocalizationHealthCore(),
        agreement=None, agreement_at=None, agreement_stamp=None,
        agreement_pending=deque([(10_000_000_000, 10., old), (10_100_000_000, 10.1, newest)]),
        agreement_tf_drop_count=0, agreement_tf_state='WAITING_FOR_SCAN', agreement_tf_error='',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock['ros'])),
        get_parameter=lambda name: SimpleNamespace(value=.18 if name == 'agreement_tolerance' else .2))
    calls = []
    def transform(target, source, stamp):
        assert stamp is not None  # Never substitute latest TF for a measurement.
        ns = stamp.sec*1_000_000_000+stamp.nanosec
        calls.append(ns)
        if ns >= clock['ready']:
            raise TransformException('scan ahead of TF')
        result = TransformStamped()
        result.header.stamp = stamp
        result.transform.rotation.w = 1.
        return result
    node._tf = transform
    return node, clock, calls


def update(node, now):
    with patch('smart_vision_amr.localization_health.scan_points', return_value=[]), \
         patch('smart_vision_amr.localization_health.scan_map_agreement', return_value=.9):
        LocalizationHealth.update_agreement(node, now)


def test_latest_scan_ahead_of_tf_does_not_starve_an_older_valid_measurement():
    node, _, calls = node_with_queue()
    update(node, 10.15)
    assert node.agreement_stamp == 10_000_000_000
    assert node.agreement == .9 and node.agreement_at == 10.15
    assert len(node.agreement_pending) == 1
    assert calls and set(calls) == {10_000_000_000, 10_100_000_000}


def test_bounded_retry_applies_newest_exact_stamp_when_tf_arrives():
    node, clock, _ = node_with_queue()
    update(node, 10.15)
    clock['ready'] = 10_200_000_000
    update(node, 10.18)
    assert node.agreement_stamp == 10_100_000_000
    assert node.agreement_tf_wait_ms == pytest.approx(80.)
    assert not node.agreement_pending
    assert node.agreement_tf_state == 'READY'


def test_timeout_or_tf_stamp_mismatch_does_not_refresh_old_agreement():
    node, _, _ = node_with_queue()
    node._tf = Mock(side_effect=TransformException('none'))
    node.agreement, node.agreement_at, node.agreement_stamp = .8, 9.8, 9_800_000_000
    update(node, 10.4)
    assert not node.agreement_pending and node.agreement_tf_drop_count == 2
    assert (node.agreement, node.agreement_at, node.agreement_stamp) == (.8, 9.8, 9_800_000_000)
    assert node.agreement_tf_state == 'MEASUREMENT_TF_TIMEOUT'
    node, _, _ = node_with_queue()
    wrong = TransformStamped()
    wrong.header.stamp.sec = 8
    wrong.transform.rotation.w = 1.
    node._tf = Mock(return_value=wrong)
    update(node, 10.4)
    assert node.agreement is None and not node.agreement_pending


def test_clock_future_or_stale_scan_is_never_retimed_to_local_now():
    node, clock, calls = node_with_queue()
    clock['ros'] = 12_000_000_000
    update(node, 10.2)
    assert node.agreement is None and not calls
    assert node.agreement_tf_state == 'SCAN_STALE_OR_CLOCK_SKEW'
    node, clock, calls = node_with_queue()
    clock['ros'] = 9_000_000_000
    update(node, 10.2)
    assert node.agreement is None and not calls


def test_agreement_receipt_cannot_hide_an_old_measurement_stamp():
    core = LocalizationHealthCore(stable_seconds=1.)
    core.evaluate(1., healthy())
    assert core.evaluate(2.1, healthy())['autonomous_allowed']
    status = core.evaluate(2.2, healthy(agreement_header_age_seconds=1.1))
    assert not status['autonomous_allowed']
    assert 'scan_map_check_unavailable' in status['reasons']
