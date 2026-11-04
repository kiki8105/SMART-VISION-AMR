"""A coherent planning proof never bypasses the independent newest-raw stop.

The raster/prefix geometry comes from the recorded start/cancel incident. Scan
messages, callback receipts and TF timing are controlled synthetic evidence;
no ROS Node, executor, publisher or action server is constructed. In particular,
the TF buffer really interpolates a latest transform 100 ms ahead of /clock.
"""
from copy import deepcopy
import math
from types import MethodType

import pytest
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer

from smart_vision_amr.motion_mux import MotionMuxNode
from test_dynamic_escape_certificate_ownership import incident  # noqa: F401
from test_near_scan_outlier_guard import _MuxHarness, scan


def _stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _future_tf_mux(now_ns, frame):
    """Use production TF lookup/interpolation with fake output publishers only."""
    mux = _MuxHarness()
    mux._ros_now = now_ns
    mux._scan_guard_base_frame = frame
    mux.scan_guard_tf_buffer = Buffer()
    mux._scan_guard_pose = MethodType(MotionMuxNode._scan_guard_pose, mux)
    for offset_ns in (-100_000_000, 100_000_000):
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = frame
        stamp_ns = now_ns + offset_ns
        transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000)
        transform.transform.translation.x = .1 * offset_ns / 1.e9
        transform.transform.rotation.w = 1.
        mux.scan_guard_tf_buffer.set_transform(transform, 'unit_recorded_odom')
    return mux


@pytest.mark.parametrize('owned_prefix', [False, True], ids=['new-prefix', 'owned-prefix'])
@pytest.mark.parametrize('raw_ahead_ns', [0, 150_000_000],
                         ids=['new-raw-tf-ready', 'new-raw-tf-pending'])
def test_unprojected_close_raw_stops_final_command_despite_fresh_paired_prefix(
        incident, owned_prefix, raw_ahead_ns):
    planner, record = incident
    old_free = planner.latest_scan
    old_stamp_ns = _stamp_ns(old_free)
    frame = planner.params['base_frame']
    assert planner.params['robot_radius'] == .22
    assert planner.params['start_egress_clearance_margin'] == 0.
    assert frame == old_free.header.frame_id
    assert dict(planner.start_visibility_history)[old_stamp_ns] == 100.

    if owned_prefix:
        planner.plan_to(planner.last_goal)
        assert planner.active_start_certificate
        original_path = list(planner.planned_world_points)

    # The actual incident had bounded future latest TF. Preserve that 100 ms
    # timing witness without inventing a newer projected obstacle frame.
    acquired_tf = deepcopy(planner.tf_buffer.lookup_transform.return_value)
    planner.clock.update(steady=100.1, ros=100.1)
    now_ns = planner.get_clock().now().nanoseconds
    latest_tf = deepcopy(acquired_tf)
    latest_tf.header.stamp.sec, latest_tf.header.stamp.nanosec = divmod(
        now_ns + 100_000_000, 1_000_000_000)

    def lookup(_target, _source, at):
        if at.nanoseconds == 0:
            return latest_tf
        assert at.nanoseconds == old_stamp_ns
        return acquired_tf

    planner.tf_buffer.lookup_transform.side_effect = lookup
    values = [math.inf] * 101
    values[50] = .21  # Actual point inside the unchanged 22 cm body.
    new_raw = scan(values, now_ns + raw_ahead_ns)
    new_raw.header.frame_id = frame
    old_before, new_before = deepcopy(old_free), deepcopy(new_raw)
    planner.on_scan(new_raw)
    assert planner.latest_scan is new_raw
    assert _stamp_ns(new_raw) not in dict(planner.start_visibility_history)
    assert planner._fresh_motion_pose() == pytest.approx(record['pose_world'])
    assert planner.plan_metrics['motion_pose_diagnostics']['tf_source_age_s'] == -.1

    if owned_prefix:
        assert not planner._live_route_blockers(planner.observed)
        assert planner.planned_world_points == original_path
    else:
        planner.plan_to(planner.last_goal)
    assert planner.planned_world_points and planner.active_start_certificate
    assert planner.active_start_certificate['edge'] == ((31, 96), (30, 96))
    selected = planner.plan_metrics['start_scan_selection']
    assert selected['reason'] == 'coherent_event_ready'
    assert selected['selected_stamp_ns'] == old_stamp_ns
    assert selected['newest_raw_scan_stamp_ns'] == _stamp_ns(new_raw)
    assert selected['source_age_s'] == pytest.approx(.1)
    assert selected['raw_first_receipt_age_s'] == pytest.approx(.1)
    assert selected['visibility_first_receipt_age_s'] == pytest.approx(.1)

    # Exercise the actual final cmd_vel guard on the very same raw objects.
    # It cannot mistake the planner's older paired proof for the newest range.
    mux = _future_tf_mux(now_ns, frame)
    assert mux._scan_guard_radius == planner.params['robot_radius'] == .22
    mux.on_scan_guard(old_free)
    assert mux.last_published_output == (.1, 0.)
    assert mux.core.scan_guard_metrics['pose_source'] == 'interpolated'
    assert mux.core.scan_guard_metrics['future_lead_seconds'] == .1
    mux.on_scan_guard(new_raw)
    assert (mux._scan_guard_pending is not None) is bool(raw_ahead_ns)
    assert mux.core.scan_guard_code == 'BODY_INTERSECTION'
    assert mux.core.scan_guard_support == 'unconfirmed'
    assert mux.core.output() == (0., 0.)
    assert mux.last_published_output == (0., 0.)
    published = mux.output_pub.publish.call_args.args[0]
    assert published.linear.x == published.angular.z == 0.
    assert old_free == old_before and new_raw == new_before
