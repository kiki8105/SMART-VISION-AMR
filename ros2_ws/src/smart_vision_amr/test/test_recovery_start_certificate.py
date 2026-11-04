"""Planner/follower reverse handover contracts, using real messages and no ROS nodes."""
from copy import deepcopy
import json
from math import cos, sin

import pytest
from std_msgs.msg import String

from test_planner_failure_memory import prepared


@pytest.fixture
def backup():
    from test_path_follower_backup import FollowerBackupTest
    case = FollowerBackupTest()
    case.setUp()
    case.feed_sensors()
    return case


def packet(case, certificate):
    return String(data=json.dumps({'goal_id': '10:0', 'distance': .2, 'speed': .04,
                                   'certified_start': certificate}))


def moved(case, *, x=0., yaw=0., frame=None, base=None, stamp_ns=12_050_000_000):
    message = deepcopy(case.node.latest_odom)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    if frame is not None: message.header.frame_id = frame
    if base is not None: message.child_frame_id = base
    message.pose.pose.position.x = x
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = sin(yaw/2), cos(yaw/2)
    case.node.on_odom(message)


def advance_stationary_odom(case, final_second):
    """Keep the original certified sample in a continuous live odom history."""
    for stamp_ns in range(12_050_000_000, final_second * 1_000_000_000 + 1,
                          50_000_000):
        moved(case, stamp_ns=stamp_ns)
    case.node.clock_seconds = final_second
    case.feed_sensors()


@pytest.mark.parametrize('x,yaw', [(0., 0.), (.0029, .0099)])
def test_handover_inside_reserved_start_envelope_dispatches_only_after_follow_terminal(backup, x, yaw):
    backup.request()
    backup.node.backup_client.send_goal_async.assert_not_called()
    moved(backup, x=x, yaw=yaw)
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_called_once()


@pytest.mark.parametrize('fault', ['missing', 'nan', 'boolean_stamp', 'wide_position',
                                 'wide_yaw', 'insufficient_reserve', 'stale_issue',
                                 'future_issue', 'stale_source'])
def test_missing_invalid_or_expired_start_certificate_never_starts_backup(backup, fault):
    cert = backup.certificate()
    if fault == 'missing': cert = None
    if fault == 'nan': cert['x'] = float('nan')
    if fault == 'boolean_stamp': cert['stamp_ns'] = True
    if fault == 'wide_position': cert['max_position_drift_m'] = .004
    if fault == 'wide_yaw': cert['max_yaw_drift_rad'] = .011
    if fault == 'insufficient_reserve': cert['sweep_reserve_m'] = .009
    if fault == 'stale_issue': cert['certified_at_ns'] -= 1_000_000_000
    if fault == 'future_issue': cert['certified_at_ns'] += 260_000_000
    if fault == 'stale_source': cert['stamp_ns'] -= 800_000_000
    backup.node.on_recovery_request(packet(backup, cert))
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert 'RECOVERY_FAILED' in backup.statuses()


@pytest.mark.parametrize('fault', ['position', 'yaw', 'frame', 'base', 'epoch',
                                 'baseline_forgery', 'unknown_baseline'])
def test_dispatch_rechecks_actual_start_after_follow_path_cancel(backup, fault):
    cert = backup.certificate()
    if fault == 'baseline_forgery': cert['x'] = .001
    if fault == 'unknown_baseline': cert['stamp_ns'] -= 50_000_000
    backup.node.on_recovery_request(packet(backup, cert))
    assert backup.node.recovery_request is not None
    if fault == 'position': moved(backup, x=.0031)
    if fault == 'yaw': moved(backup, yaw=.0101)
    if fault == 'frame': moved(backup, frame='different_odom')
    if fault == 'base': moved(backup, base='different_base')
    if fault == 'epoch': moved(backup, stamp_ns=11_900_000_000)
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert backup.node.recovery_request is None
    assert 'RECOVERY_FAILED' in backup.statuses()


def test_certificate_age_is_rechecked_at_dispatch_after_cancellation_delay(backup):
    backup.request()
    advance_stationary_odom(backup, 15)
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert 'RECOVERY_FAILED' in backup.statuses()


def test_stable_cancellation_delay_keeps_checked_reverse_available(backup):
    backup.request()
    advance_stationary_odom(backup, 14)
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_called_once()


def test_future_source_ordering_waits_for_exact_odom_without_extrapolation(backup):
    cert = backup.certificate()
    cert['stamp_ns'] += 50_000_000
    backup.node.on_recovery_request(packet(backup, cert))
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert backup.node.recovery_request is not None
    moved(backup)
    backup.node._monitor_recovery()
    backup.node.backup_client.send_goal_async.assert_called_once()


def test_three_second_goal_buffer_does_not_refresh_old_certificate(backup):
    payload = json.loads(packet(backup, backup.certificate()).data)
    payload['goal_id'] = '13:0'
    backup.node.on_recovery_request(String(data=json.dumps(payload)))
    assert backup.node.pending_recovery_request is not None
    backup.node.clock_seconds = 14
    from test_path_follower_cancel import goal_for, make_path
    backup.node.on_goal(goal_for(make_path(stamp=14), stamp=13))
    assert backup.node.recovery_request is None
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert 'RECOVERY_FAILED' in backup.statuses()


def test_start_certificate_does_not_reject_intended_motion_after_backup_dispatch(backup):
    backup.request()
    backup.finish_follow_cancel()
    handle, _ = backup.accept_backup()
    moved(backup, x=-.05)
    backup.node._monitor_recovery()
    assert backup.node.recovery_request is not None
    handle.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize('fault', ['position', 'yaw', 'stale', 'wrong_frame', 'wrong_base'])
def test_map_pose_is_rechecked_even_with_identical_odometry(backup, fault):
    backup.request()
    transform = backup.node.tf_buffer.lookup_transform.return_value
    if fault == 'position': transform.transform.translation.x = .0031
    if fault == 'yaw':
        transform.transform.rotation.z, transform.transform.rotation.w = sin(.0101/2), cos(.0101/2)
    if fault == 'stale': transform.header.stamp.sec = 11
    if fault == 'wrong_frame': transform.header.frame_id = 'other_map'
    if fault == 'wrong_base': transform.child_frame_id = 'other_base'
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert backup.node.recovery_request is None


def test_temporarily_missing_map_transform_waits_but_never_dispatches_blind(backup):
    from tf2_ros import TransformException
    backup.request()
    backup.node.tf_buffer.lookup_transform.side_effect = TransformException('not received yet')
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert backup.node.recovery_request is not None
    backup.node.tf_buffer.lookup_transform.side_effect = None
    backup.node._monitor_recovery()
    backup.node.backup_client.send_goal_async.assert_called_once()


def test_missing_map_transform_cannot_outlive_original_certificate(backup):
    from tf2_ros import TransformException
    backup.request()
    backup.node.tf_buffer.lookup_transform.side_effect = TransformException('not received yet')
    backup.finish_follow_cancel()
    backup.node.clock_seconds = 15
    backup.node._monitor_recovery()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert backup.node.recovery_request is None


@pytest.mark.parametrize('fault', ['motion', 'map_correction', 'expired'])
def test_waiting_for_action_server_cannot_make_start_certificate_stale(backup, fault):
    backup.request()
    def server_ready(**_):
        if fault == 'motion': moved(backup, x=.004)
        if fault == 'map_correction':
            backup.node.tf_buffer.lookup_transform.return_value.transform.translation.x = .004
        if fault == 'expired': backup.node.clock_seconds = 15
        return True
    backup.node.backup_client.wait_for_server.side_effect = server_ready
    backup.finish_follow_cancel()
    backup.node.backup_client.send_goal_async.assert_not_called()
    assert backup.node.recovery_request is None


@pytest.mark.parametrize('scale,yaw', [(1., 0.), (1.05, .01)])
def test_actual_planner_request_interoperates_with_exact_follower_handover(prepared, backup, scale, yaw):
    """Also runs with the staged older adapter and staged follower module loaded."""
    from test_planner_start_reverse_evidence import reverse_required_scenario
    planner = prepared
    goal = reverse_required_scenario(planner)
    if scale != 1.:
        from smart_vision_amr.reverse_recovery import Breadcrumb
        from nav_msgs.msg import Odometry
        measured = Odometry()
        measured.header.frame_id, measured.child_frame_id = 'odom', 'base_link'
        measured.header.stamp.sec = 100
        measured.pose.pose.position.x, measured.pose.pose.position.y = 1.3001, .4001
        measured.pose.pose.orientation.z, measured.pose.pose.orientation.w = scale*sin(yaw/2), scale*cos(yaw/2)
        planner.on_odom(measured)
        transform = planner.tf_buffer.lookup_transform.return_value
        transform.transform.rotation.z, transform.transform.rotation.w = scale*sin(yaw/2), scale*cos(yaw/2)
        planner.breadcrumbs = type(planner.breadcrumbs)(
            (Breadcrumb(point.stamp, point.x, point.y, yaw) for point in planner.breadcrumbs), maxlen=300)
        planner.breadcrumb_odom_anchor = planner.latest_odom
        planner.odom_samples = type(planner.odom_samples)(
            ((99.75+i*.05, 99_750_000_000+i*50_000_000,
              planner.latest_odom[2], planner.latest_odom[3], yaw, planner.latest_odom[5])
             for i in range(6)), maxlen=100)
    planner.plan_to(goal)
    message = planner.recovery_pub.publish.call_args.args[0]
    payload = json.loads(message.data)
    cert = payload['certified_start']
    assert cert['yaw'] == pytest.approx(yaw)
    assert cert['map_yaw'] == pytest.approx(yaw)
    follower = backup.node
    follower.clock_seconds = 100
    backup.feed_sensors()
    baseline = deepcopy(follower.latest_odom)
    baseline.child_frame_id = cert['child_frame_id']
    baseline.pose.pose.position.x, baseline.pose.pose.position.y = cert['x'], cert['y']
    baseline.pose.pose.orientation.z = scale*sin(yaw/2)
    baseline.pose.pose.orientation.w = scale*cos(yaw/2)
    follower.on_odom(baseline)
    transform = follower.tf_buffer.lookup_transform.return_value
    transform.header.frame_id, transform.child_frame_id = cert['map_frame'], cert['map_base_frame']
    transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(cert['map_stamp_ns'], 1_000_000_000)
    transform.transform.translation.x, transform.transform.translation.y = cert['map_x'], cert['map_y']
    transform.transform.rotation.z, transform.transform.rotation.w = scale*sin(yaw/2), scale*cos(yaw/2)
    follower.on_goal(goal)
    follower.on_recovery_request(message)
    assert follower.recovery_request is not None
    follower.backup_client.send_goal_async.assert_not_called()
    backup.finish_follow_cancel()
    follower.backup_client.send_goal_async.assert_called_once()
    assert follower.backup_client.send_goal_async.call_args.args[0].target.x == pytest.approx(-payload['distance'])
