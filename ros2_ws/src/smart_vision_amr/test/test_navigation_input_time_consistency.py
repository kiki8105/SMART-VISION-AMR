"""Source/receipt time geometry regressions; no ROS context or DDS is created.

These callbacks deliberately exercise asynchronous odometry and Nav2 polygon
updates. A moving robot's current pose is not the pose of an older polygon.
"""

from copy import deepcopy
from math import cos, pi, sin
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped

from test_navigation_input_preflight import healthy_follower, polygon, start
from test_path_follower_cancel import goal_for, make_path


def ros_now(node, seconds):
    nanoseconds = round(seconds*1_000_000_000)
    sec, nanosec = divmod(nanoseconds, 1_000_000_000)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=nanoseconds,
                                    to_msg=lambda: Time(sec=sec, nanosec=nanosec)))


def odom(node, seconds, x, *, yaw=0., frame='odom'):
    message = deepcopy(node.latest_odom)
    nanoseconds = round(seconds*1_000_000_000)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
    message.header.frame_id = frame
    message.pose.pose.position.x = x
    message.pose.pose.orientation.z = sin(yaw/2.)
    message.pose.pose.orientation.w = cos(yaw/2.)
    message.twist.twist.linear.x = .15
    node.on_odom(message)


def circle(node, center_x, *, stamp=4.2, frame='odom'):
    message = PolygonStamped()
    message.header.frame_id = frame
    for index in range(16):
        point = Point32()
        point.x, point.y = center_x+.22*cos(index*pi/8), .22*sin(index*pi/8)
        message.polygon.points.append(point)
    nanoseconds = round(stamp*1_000_000_000)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        nanoseconds, 1_000_000_000)
    node.on_footprint(message)


def delayed_polygon(*, center_x=0.):
    node = healthy_follower()
    # A valid .15m/s motion: the footprint is .8s behind the newest odometry,
    # but both header and receipt remain inside the existing freshness bounds.
    odom(node, 4.1, -.015)
    odom(node, 4.3, .015)
    for tenth in range(44, 51):
        odom(node, tenth/10., .15*(tenth/10.-4.2))
    circle(node, center_x)
    return node


def test_delayed_fresh_polygon_uses_interpolated_source_time_odometry():
    node = delayed_polygon()
    assert node._navigation_input_error() is None


def test_forged_fresh_polygon_cannot_match_newer_pose_instead_of_its_source_time():
    # .20 is only .08m from the newest pose, but .20m from its stamped pose.
    # The unchanged .10m geometry bound must reject that false agreement.
    node = delayed_polygon(center_x=.20)
    assert node._navigation_input_error() is not None


def test_fresh_polygon_transform_uses_its_acquisition_time_not_latest_tf():
    node = delayed_polygon()
    circle(node, 0., frame='moving_grid')

    def transform(target, source, stamp, *args, **kwargs):
        message = TransformStamped()
        message.header.stamp.sec = 4
        message.header.stamp.nanosec = 200_000_000
        message.transform.rotation.w = 1.
        # The source frame subsequently moved by a metre. Only its stamped
        # transform maps the old polygon into the matching odometry frame.
        message.transform.translation.x = 0. if stamp.nanoseconds == 4_200_000_000 else 1.
        return message

    node.tf_buffer.lookup_transform.side_effect = transform
    assert node._navigation_input_error() is None
    assert any(call.args[:2] == ('odom', 'moving_grid')
               and call.args[2].nanoseconds == 4_200_000_000
               for call in node.tf_buffer.lookup_transform.call_args_list)


@pytest.mark.parametrize('fault', ['gap', 'frame_change', 'jump'])
def test_source_time_interpolation_rejects_unproven_or_discontinuous_motion(fault):
    node = healthy_follower()
    odom(node, 4.1, -.015)
    odom(node, 4.5 if fault == 'gap' else 4.3,
         1. if fault == 'jump' else .015,
         frame='other_odom' if fault == 'frame_change' else 'odom')
    for tenth in range(46 if fault == 'gap' else 44, 51):
        odom(node, tenth/10., .15*(tenth/10.-4.2))
    circle(node, 0.)
    assert node._navigation_input_error() is not None


def test_source_time_history_cannot_hide_stale_current_odometry_receipt():
    node = delayed_polygon()
    node.latest_odom_at -= 1.
    assert 'receipt' in node._navigation_input_error()


def test_active_follow_path_survives_short_sensor_gap_and_fresh_recovery():
    # New goals still fail preflight at 0.75 s. During an active goal the
    # robot-side localization guard has already held velocity, so retaining
    # the action briefly avoids an unnecessary cancel/replan on one late scan.
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        clock.return_value = 100.85
        ros_now(node, 5.85)
        assert 'Stale scan' in node._navigation_input_error()
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_not_called()
        assert not node.path_invalidated

        # New scan and odometry arrive before the bounded continuity deadline.
        clock.return_value = 100.90
        ros_now(node, 5.90)
        scan = deepcopy(node.latest_scan)
        scan.header.stamp.sec, scan.header.stamp.nanosec = 5, 900_000_000
        node.on_scan(scan)
        odometry = deepcopy(node.latest_odom)
        odometry.header.stamp.sec, odometry.header.stamp.nanosec = 5, 900_000_000
        node.on_odom(odometry)
        # The delayed odometry begins a new source-time history epoch. Nav2's
        # accompanying footprint must represent that same fresh epoch.
        circle(node, 0., stamp=5.9)
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_not_called()
        assert node.navigation_input_fault is None


@pytest.mark.parametrize('source', ['scan', 'odom'])
def test_active_follow_path_cancels_after_one_second_sensor_loss(source):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        clock.return_value = 101.05
        ros_now(node, 6.05)
        if source == 'scan':
            odometry = deepcopy(node.latest_odom)
            odometry.header.stamp.sec, odometry.header.stamp.nanosec = 6, 50_000_000
            node.on_odom(odometry)
        else:
            scan = deepcopy(node.latest_scan)
            scan.header.stamp.sec, scan.header.stamp.nanosec = 6, 50_000_000
            node.on_scan(scan)
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_called_once()
        assert node.navigation_input_fault.startswith('Stale '+source)


@pytest.mark.parametrize('fault', ['missing_scan', 'bad_scan', 'bad_odom'])
def test_continuity_window_does_not_accept_missing_or_invalid_input(fault):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        clock.return_value = 100.85
        ros_now(node, 5.85)
        if fault == 'missing_scan':
            node.latest_scan = None
        elif fault == 'bad_scan':
            node.latest_scan.ranges = [float('nan')]*360
        else:
            node.latest_odom.twist.twist.linear.x = float('nan')
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_called_once()
        assert node.navigation_input_fault is not None


def test_old_header_current_geometry_still_has_receipt_time_pose_proof():
    node = delayed_polygon()
    circle(node, .12, stamp=1.)
    assert node._navigation_input_error() is None


def test_old_header_does_not_allow_stale_coordinates_with_a_new_receipt():
    node = delayed_polygon()
    circle(node, 0., stamp=1.)
    assert node._navigation_input_error() is not None


@pytest.mark.parametrize('stamp, allowed', [(1., False), (4.2, True)])
def test_only_valid_source_stamp_can_compensate_more_than_ten_cm_of_later_motion(stamp, allowed):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        ros_now(node, 4.2)
        odom(node, 4.2, 0.)
        circle(node, 0., stamp=stamp)
        # Same physical geometry/motion in both cases. Only the truthful source
        # stamp lets downstream consumers reconstruct the historical body pose.
        for tenth in range(43, 51):
            clock.return_value = 100.+tenth/10.-4.2
            ros_now(node, tenth/10.)
            odom(node, tenth/10., .15*(tenth/10.-4.2))
        for source in ('scan', 'costmap'):
            getattr(node, 'latest_'+source).header.stamp.sec = 5
            setattr(node, 'latest_'+source+'_at', 100.8)
        issue = node._navigation_input_error()
        assert (issue is None) is allowed
        if not allowed:
            assert 'Old footprint position' in issue


def test_old_asymmetric_polygon_still_requires_current_body_orientation_agreement():
    node = healthy_follower()
    rectangle = [(-.3, -.15), (.3, -.15), (.3, .15), (-.3, .15)]
    polygon(node, rectangle, stamp=5)
    assert node._navigation_input_error() is None
    polygon(node, rectangle, stamp=1)
    ros_now(node, 5.2)
    odom(node, 5.2, 0., yaw=.2)
    assert 'orientation' in node._navigation_input_error()


def test_future_footprint_waits_for_bracketing_odom_without_canceling_valid_action():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        previous = node.latest_footprint
        circle(node, .0075, stamp=5.05)
        candidate = node.navigation_pending_footprint['message']
        assert node.latest_footprint is previous
        assert node._navigation_input_error() is None
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_not_called()

        clock.return_value = 100.05
        ros_now(node, 5.1)
        odom(node, 5.1, .015)
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint is candidate
        assert node.latest_footprint_at == 100.
        assert node.navigation_footprint_receipt_ros_ns == 5_000_000_000
        assert node._navigation_odom_at(5_050_000_000, 'odom', 'base_footprint')[0] == pytest.approx(.0075)
        assert node._navigation_input_error() is None


@pytest.mark.parametrize('odom_age, future_offset', [
    (.267210175, .099940742),
    (.293100768, .050851997),
    (.334336756, .097553969),
    (.266740099, .049410057),
    (.313634870, .052500631),
])
@pytest.mark.parametrize('receipt_delayed', [False, True])
def test_actual_delayed_odom_cases_defer_future_polygon_without_legacy_receipt_proof(
        odom_age, future_offset, receipt_delayed):
    """Five live failures were inside .75s odom freshness, outside .25s proof.

    Reproduce their source-time skew without ROS/DDS. Neither callback receipt
    order nor a current pose may substitute for bracketing source odometry.
    """
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        previous = node.latest_footprint
        clock.return_value = 100.+odom_age
        ros_now(node, 5.+odom_age)
        if not receipt_delayed:
            node.latest_odom_at = clock.return_value-.02
        circle(node, .15*future_offset, stamp=5.+future_offset)
        pending = node.navigation_pending_footprint
        assert pending is not None
        assert pending['receipt_pose'] is None  # The legacy .25s proof stays strict.
        candidate = pending['message']
        assert node.latest_footprint is previous
        assert node._navigation_input_error() is None
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_not_called()
        with pytest.raises(ValueError, match='No continuous odometry'):
            node._navigation_odom_at(pending['stamp_ns'], 'odom', 'base_footprint')

        clock.return_value += .05
        ros_now(node, 5.+odom_age+.05)
        odom(node, 5.+future_offset+.05, .15*(future_offset+.05))
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint is candidate
        assert node.latest_footprint_at == pending['received_at']
        assert node.navigation_footprint_receipt_ros_ns == pending['received_ros_ns']
        assert node.navigation_footprint_receipt_pose is None
        assert node._navigation_odom_at(pending['stamp_ns'], 'odom', 'base_footprint')[0] == pytest.approx(.15*future_offset)
        assert node._navigation_input_error() is None
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize('fault', ['source_deadline', 'receipt_deadline', 'nonfinite_receipt'])
def test_future_deferral_cannot_hide_navigation_odom_deadline_or_nonfinite_receipt(fault):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.):
        node = healthy_follower()
        _, handle = start(node)
        ros_now(node, 5.75 if fault == 'source_deadline' else 5.3)
        if fault == 'source_deadline':
            node.latest_scan.header.stamp.nanosec = 750_000_000
        node.latest_odom_at = (float('nan') if fault == 'nonfinite_receipt' else
                               99.25 if fault == 'receipt_deadline' else 100.)
        circle(node, .0075, stamp=5.05)
        assert node.navigation_pending_footprint is None
        assert 'odom' in node._navigation_input_error()
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_called_once()


@pytest.mark.parametrize('fault', ['epoch', 'expiry'])
def test_delayed_odom_pending_candidate_still_fails_closed_on_epoch_or_expiry(fault):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        previous = node.latest_footprint
        ros_now(node, 5.3)
        circle(node, .0075, stamp=5.05)
        assert node.navigation_pending_footprint is not None
        clock.return_value = 100.25 if fault == 'expiry' else 100.05
        odom(node, 5.1, 1. if fault == 'epoch' else .015)
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint is previous
        assert ('Odometry changed' if fault == 'epoch' else 'Timed out') in node._navigation_input_error()
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_called_once()


def test_delayed_odom_does_not_relax_old_header_receipt_time_proof():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.):
        node = healthy_follower()
        ros_now(node, 5.3)
        circle(node, 0., stamp=1.)
        assert node.navigation_pending_footprint is None
        assert node.navigation_footprint_receipt_pose is None
        assert 'Old footprint has no fresh receipt-time odometry proof' in node._navigation_input_error()


def test_first_future_footprint_cannot_dispatch_before_exact_odom_alignment():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        node.latest_footprint = node.latest_footprint_at = None
        circle(node, 0., stamp=5.05)
        path = make_path(stamp=5)
        node.on_goal(goal_for(path, stamp=4))
        node.on_path(path)
        node.client.send_goal_async.assert_not_called()
        assert node._navigation_input_error() == 'Missing footprint'
        clock.return_value = 100.05
        ros_now(node, 5.05)
        odom(node, 5.05, 0.)
        assert node._navigation_input_error() is None
        node.on_path(make_path(stamp=6))
        node.client.send_goal_async.assert_called_once()


@pytest.mark.parametrize('elapsed, allowed', [(-.001, False), (.249, True), (.250, False)])
def test_pending_deadline_is_steady_and_late_odom_cannot_resurrect_expired_footprint(elapsed, allowed):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        previous = node.latest_footprint
        circle(node, 0., stamp=5.05)
        candidate = node.navigation_pending_footprint['message']
        clock.return_value = 100.+elapsed  # ROS clock intentionally stays at 5.
        odom(node, 5.1, 0.)
        assert node.navigation_pending_footprint is None
        assert (node.latest_footprint is candidate) is allowed
        if allowed:
            assert node._navigation_input_error() is None
            node._watch_navigation_inputs()
            handle.cancel_goal_async.assert_not_called()
        else:
            assert node.latest_footprint is previous
            assert 'Timed out' in node._navigation_input_error()
            node._watch_navigation_inputs()
            handle.cancel_goal_async.assert_called_once()
            clock.return_value = 100.3
            odom(node, 5.15, 0.)
            assert node.latest_footprint is previous
            assert 'Timed out' in node._navigation_input_error()


def test_newer_pending_footprints_cannot_slide_original_deadline():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        circle(node, 0., stamp=5.05)
        clock.return_value = 100.2
        circle(node, 0., stamp=5.1)
        assert node.navigation_pending_footprint['wait_started_at'] == 100.
        assert node.navigation_pending_footprint['stamp_ns'] == 5_050_000_000
        clock.return_value = 100.25
        assert 'Timed out' in node._navigation_input_error()


@pytest.mark.parametrize('fault', ['gap', 'jump', 'frame', 'base', 'backward', 'malformed'])
def test_pending_candidate_is_disqualified_by_odometry_epoch_change(fault):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        previous = node.latest_footprint
        circle(node, 0., stamp=5.05)
        message = deepcopy(node.latest_odom)
        message.header.stamp.sec = 5
        message.header.stamp.nanosec = 300_000_000 if fault == 'gap' else 100_000_000
        if fault == 'jump': message.pose.pose.position.x = 1.
        if fault == 'frame': message.header.frame_id = 'other_odom'
        if fault == 'base': message.child_frame_id = 'other_base'
        if fault == 'backward': message.header.stamp.sec = 4
        if fault == 'malformed': message.pose.pose.orientation.w = 0.
        clock.return_value = 100.05
        node.on_odom(message)
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint is previous
        assert 'Odometry changed' in node._navigation_input_error()


@pytest.mark.parametrize('fault', ['scan', 'odom', 'costmap', 'footprint', 'costmap_geometry'])
def test_pending_alignment_does_not_hide_other_input_faults_or_extend_previous_deadlines(fault):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.):
        node = healthy_follower()
        _, handle = start(node)
        circle(node, 0., stamp=5.05)
        if fault == 'costmap_geometry':
            node.latest_costmap.data = []
        else:
            setattr(node, 'latest_'+fault+'_at', 98.)
        assert node._navigation_input_error() is not None
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_called_once()


@pytest.mark.parametrize('fault', ['empty_frame', 'changed_frame', 'nan', 'crossed', 'size'])
def test_malformed_future_footprint_is_not_hidden_behind_previous_good_geometry(fault):
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.):
        node = healthy_follower()
        circle(node, 0., stamp=5.02)
        assert node.navigation_pending_footprint is not None
        message = deepcopy(node.latest_footprint)
        message.header.stamp.nanosec = 50_000_000
        if fault == 'empty_frame': message.header.frame_id = ''
        if fault == 'changed_frame': message.header.frame_id = 'unconnected_grid'
        if fault == 'nan': message.polygon.points[0].x = float('nan')
        if fault == 'crossed':
            a, b = message.polygon.points[0], message.polygon.points[8]
            message.polygon.points[0], message.polygon.points[8] = b, a
        if fault == 'size':
            for point in message.polygon.points:
                point.x *= 10.
                point.y *= 10.
        node.on_footprint(message)
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint is message
        assert node._navigation_input_error() is not None


def test_aligned_pending_polygon_still_must_match_actual_acquisition_position():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        circle(node, .2, stamp=5.05)
        clock.return_value = 100.05
        odom(node, 5.1, 0.)
        assert 'Footprint disagrees' in node._navigation_input_error()


def test_missing_acquisition_history_warning_reports_bounded_source_context():
    node = healthy_follower()
    circle(node, 0., stamp=4.9)  # Fresh header predates the only retained odom sample.
    assert node._navigation_input_error() is not None
    message = node.logger.warning.call_args.args[0]
    assert 'No continuous odometry at footprint acquisition time' in message
    for value in ('requested_ns=4900000000', 'first_ns=5000000000',
                  'last_ns=5000000000', 'count=1', 'epoch=0',
                  "requested_frames=('odom', 'base_footprint')",
                  "first_frames=('odom', 'base_footprint')",
                  "last_frames=('odom', 'base_footprint')"):
        assert value in message


def test_high_rate_future_footprints_do_not_starve_earliest_acquisition_alignment():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.) as clock:
        node = healthy_follower()
        _, handle = start(node)
        ros_now(node, 5.1)
        circle(node, 0., stamp=5.02)
        first = node.navigation_pending_footprint['message']
        for index, stamp in enumerate((5.04, 5.06, 5.08), 1):
            clock.return_value = 100.+index*.01
            circle(node, 0., stamp=stamp)
            assert node.navigation_pending_footprint['message'] is first
        clock.return_value = 100.05
        odom(node, 5.05, 0.)  # Brackets the first, not the most recent polygon.
        assert node.latest_footprint is first
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint_at == 100.
        assert node._navigation_input_error() is None
        node._watch_navigation_inputs()
        handle.cancel_goal_async.assert_not_called()
        with pytest.raises(ValueError, match='No continuous odometry'):
            node._navigation_odom_at(5_080_000_000, 'odom', 'base_footprint')


def test_new_genuinely_aligned_footprint_replaces_pending_candidate_normally():
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=100.):
        node = healthy_follower()
        circle(node, 0., stamp=5.05)
        assert node.navigation_pending_footprint is not None
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        node.tf_buffer.lookup_transform.side_effect = None
        node.tf_buffer.lookup_transform.return_value = transform
        circle(node, 0., stamp=5., frame='connected_grid')
        assert node.navigation_pending_footprint is None
        assert node.latest_footprint.header.frame_id == 'connected_grid'
        assert node.latest_footprint.header.stamp.nanosec == 0
        assert node._navigation_input_error() is None
