"""Latest scan admission is cheap; only the output timer proves motion."""

import math
from unittest.mock import Mock

import numpy as np

from smart_vision_amr.motion_mux import MotionMuxNode
from test_near_scan_outlier_guard import _MuxHarness, scan


def admit(node, message):
    # Bypass the geometry fixture's explicit following output tick.
    MotionMuxNode.on_scan_guard(node, message)


def test_clear_scan_admission_waits_for_timer_and_prepares_cloud_once():
    node = _MuxHarness()
    node._refresh_scan_guard_locked = Mock(wraps=node._refresh_scan_guard_locked)
    node.core.scan_guard_hold = True
    admit(node, scan([.9]*101))
    assert node._scan_guard_pending is not None
    assert node._guard_proof is None
    assert node.core.scan_guard_hold
    node._refresh_scan_guard_locked.assert_not_called()
    node.output_pub.publish.assert_not_called()
    node.publish_output(refresh_ownership=False)
    node._refresh_scan_guard_locked.assert_called_once()
    assert node.last_published_output == (.1, 0.)
    assert isinstance(node._scan_guard_checks[0][0], np.ndarray)


def test_new_body_hit_stops_immediately_without_repeating_full_proof():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    node._refresh_scan_guard_locked = Mock(wraps=node._refresh_scan_guard_locked)
    node._ros_now += 100_000_000
    values = [math.inf]*101
    values[50] = .21
    admit(node, scan(values, node._ros_now))
    node._refresh_scan_guard_locked.assert_not_called()
    assert node.core.scan_guard_code == 'BODY_INTERSECTION'
    assert node.core.scan_guard_support == 'unconfirmed'
    assert node.last_published_output == (0., 0.)
    node.publish_output(refresh_ownership=False)
    assert node.last_published_output == (0., 0.)


def test_new_invalid_scan_stops_immediately_and_clear_scan_cannot_skip_proof():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    node._ros_now += 100_000_000
    admit(node, scan([math.nan]*101, node._ros_now))
    assert node.core.scan_guard_code == 'INVALID_SCAN'
    assert node.last_published_output == (0., 0.)
    admit(node, scan([.9]*101, node._ros_now))
    assert node.core.scan_guard_hold
    assert node.last_published_output == (0., 0.)
    node.publish_output(refresh_ownership=False)
    assert not node.core.scan_guard_hold
    assert node.last_published_output == (.1, 0.)


def test_new_sweep_hit_is_checked_before_timer_can_emit_another_command():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    node._ros_now += 100_000_000
    values = [math.inf]*101
    values[50] = .25  # Clear current body; obstruct the next forward curve.
    admit(node, scan(values, node._ros_now))
    assert node._guard_proof is None
    publications = node.output_pub.publish.call_count
    node.publish_output(refresh_ownership=False)
    assert node.core.scan_guard_code == 'SWEPT_INTERSECTION'
    assert node.last_published_output == (0., 0.)
    assert node.output_pub.publish.call_count == publications+1


def test_timer_output_can_skip_graph_query_without_relaxing_ownership_expiry():
    node = _MuxHarness()
    node._scan_guard_enabled = False
    node.check_output_ownership = Mock(side_effect=AssertionError('graph lookup on output path'))
    node.publish_output(refresh_ownership=False)
    assert node.last_published_output == (.1, 0.)
    node._ownership_completed_at -= 1.
    publications = node.output_pub.publish.call_count
    node.publish_output(refresh_ownership=False)
    assert not node.output_available
    assert node.output_pub.publish.call_count == publications
