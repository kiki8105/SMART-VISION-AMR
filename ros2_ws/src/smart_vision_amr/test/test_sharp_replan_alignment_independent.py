"""Independent motion-ownership checks for a material replacement route.

Actual map geometry/pose comes from accepted-route recordings. Local scan,
footprint, free costmap, source-stamped stationary odometry and action results
are controlled test evidence, not a claim that the historical robot stopped.
No ROS context/node/action server or drive publisher is started.
"""
from copy import deepcopy
import json
from math import cos, degrees, pi, sin
from pathlib import Path as FilePath
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from test_navigation_input_preflight import healthy_follower
from test_path_follower_cancel import goal_for, make_path, result_future


FIXTURE = (FilePath(__file__).resolve().parents[4] / 'benchmarks' /
           'driving_parameter_tuning_20261104' / 'accepted_route_tracking_20261105' /
           'actual_entry_fixture.json')


def path_message(record):
    path = Path()
    path.header.frame_id = record['frame_id']
    ns = record['stamp_ns']
    path.header.stamp.sec, path.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    for x, y in record['points']:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    return path


@pytest.fixture
def rig(monkeypatch):
    clock = {'source': 5., 'receipt': 100.}
    monkeypatch.setattr('smart_vision_amr.path_follower.time.monotonic',
                        lambda: clock['receipt'])
    node = healthy_follower()
    node.parameters.update({'initial_heading_alignment_enabled': True,
                            'initial_heading_alignment_prune_distance': .4})
    def now():
        stamp = PoseStamped().header.stamp
        ns = round(clock['source'] * 1_000_000_000)
        stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp)
    node.get_clock = lambda: SimpleNamespace(now=now)
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    node.tf_buffer.lookup_transform.side_effect = None
    node.tf_buffer.lookup_transform.return_value = transform
    def sample(delta=.05, *, linear=0., yaw_rate=0., refresh_scan=True,
               mode='auto', estop=False, reset=False):
        clock['source'] += delta
        clock['receipt'] += delta
        ns = round(clock['source']*1_000_000_000)
        # Real source timestamps, never a receipt-only refresh of old odometry.
        message = deepcopy(node.latest_odom)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        message.twist.twist.linear.x = float(linear)
        message.twist.twist.angular.z = float(yaw_rate)
        for source in ('scan', 'costmap', 'footprint'):
            current = deepcopy(getattr(node, 'latest_'+source))
            if source != 'scan' or refresh_scan:
                current.header.stamp = deepcopy(message.header.stamp)
            getattr(node, 'on_'+source)(current)
        transform.header.stamp = deepcopy(message.header.stamp)
        node.motion_status = {'mode': mode, 'available': True,
                              'autonomous_allowed': mode == 'auto', 'estop_latched': estop,
                              'reset_pending': reset}
        node.motion_status_at = clock['receipt']
        node.on_odom(message)
        return message
    def pose(x, y, yaw):
        transform.transform.translation.x, transform.transform.translation.y = float(x), float(y)
        transform.transform.rotation.z, transform.transform.rotation.w = sin(yaw/2), cos(yaw/2)
        transform.header.stamp = now().to_msg()
    sample(.01)
    return node, clock, pose, sample


def accepted_old_route(node, path):
    """Model a previously dispatched route; run real acceptance/cancel callbacks."""
    node.on_goal(goal_for(path, stamp=1))
    node._dispatch_follow_path(path)
    owned = node.request_context
    handle = Mock(accepted=True)
    node.on_goal_response(Mock(result=Mock(return_value=handle)), owned)
    assert node.active_goal is handle
    assert node._route_alignment_reference['generation'] == node.goal_generation
    return handle, owned


def confirm_stop(node, sample):
    for _ in range(6):
        sample()
    node._monitor_initial_alignment()


def finish_spin(node, pose, path):
    state = node.initial_alignment
    assert state is not None
    handle = Mock(accepted=True)
    node._on_initial_alignment_response(Mock(result=Mock(return_value=handle)), state)
    error = node._initial_heading_error(path)
    tf = node.tf_buffer.lookup_transform.return_value
    from math import atan2
    q = tf.transform.rotation
    yaw = atan2(2*q.w*q.z, 1-2*q.z*q.z)
    pose(tf.transform.translation.x, tf.transform.translation.y, yaw+error)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_SUCCEEDED), state)
    return state, handle


@pytest.mark.parametrize('case_name', ['v2', 'local025'])
def test_actual_large_replacement_cannot_drive_before_owned_terminal_and_stop(rig, case_name):
    node, _, pose, sample = rig
    case = json.loads(FIXTURE.read_text())['cases'][case_name]
    old, replacement = path_message(case['old_path']), path_message(case['replacement_path'])
    current = case['estimated_pose']
    pose(current['x'], current['y'], current['yaw'])
    handle, context = accepted_old_route(node, old)
    assert degrees(node._initial_heading_error(replacement)) == pytest.approx(
        case['recorded_error_deg'], abs=.01)
    assert node._navigation_input_error() is None
    node.on_path(replacement)
    handle.cancel_goal_async.assert_called_once()
    node._on_follow_cancel_response(
        Mock(result=Mock(return_value=SimpleNamespace(goals_canceling=[object()]))), handle)
    confirm_stop(node, sample)
    assert node.follow_cancel_acknowledged
    assert node.client.send_goal_async.call_count == 1
    node.spin_client.send_goal_async.assert_not_called()
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    # Source evidence newer than the actual terminal is required; a terminal
    # callback by itself does not prove the physical robot has settled.
    assert node.client.send_goal_async.call_count == 1
    node.spin_client.send_goal_async.assert_not_called()
    confirm_stop(node, sample)
    assert node.initial_alignment['kind'] == 'material_replan'
    node.spin_client.send_goal_async.assert_called_once()
    assert node.client.send_goal_async.call_count == 1
    state, _ = finish_spin(node, pose, replacement)
    assert state['context']['path_stamp'] == {
        'sec': replacement.header.stamp.sec, 'nanosec': replacement.header.stamp.nanosec}
    assert node.client.send_goal_async.call_count == 2
    assert node.client.send_goal_async.call_args.args[0].path is replacement
    assert node.initial_alignment_total_starts == 0


def test_small_changed_prefix_uses_follow_path_without_extra_spin(rig):
    node, _, pose, _ = rig
    old = make_path(stamp=6)
    handle, context = accepted_old_route(node, old)
    changed = make_path(stamp=7)
    # Material route change with a mere15-degree first heading error.
    changed.poses[1].pose.position.y = .27
    pose(0., 0., 0.)
    node.on_path(changed)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    assert node.client.send_goal_async.call_count == 2
    node.spin_client.send_goal_async.assert_not_called()


def test_equivalent_suffix_alias_does_not_align_or_consume_replan_budget(rig):
    node, _, pose, _ = rig
    path = make_path(stamp=6)
    handle, context = accepted_old_route(node, path)
    pose(.5, 0., pi/2)  # Large yaw alone is not proof of a changed prefix.
    alias = deepcopy(path)
    alias.header.stamp.sec = 7
    alias.poses = alias.poses[1:]
    node.on_path(alias)
    handle.cancel_goal_async.assert_not_called()
    node.spin_client.send_goal_async.assert_not_called()
    assert node.client.send_goal_async.call_count == 1
    assert context['path_stamp'] == {'sec': 7, 'nanosec': 0}
    assert getattr(node, 'replan_alignment_total_starts', 0) == 0


def test_angular_motion_prevents_material_replan_alignment_even_with_zero_translation(rig):
    node, _, pose, sample = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    path = make_path(stamp=7)
    path.poses[1].pose.position.y = 1.
    pose(0., 0., 0.)
    node.on_path(path)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    for _ in range(8):
        sample(yaw_rate=.10)
    node._monitor_initial_alignment()
    node.spin_client.send_goal_async.assert_not_called()
    assert node.client.send_goal_async.call_count == 1
    confirm_stop(node, sample)
    node.spin_client.send_goal_async.assert_called_once()


def test_stale_scan_cannot_start_material_alignment_after_stop(rig):
    node, clock, pose, sample = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    replacement = make_path(stamp=7)
    replacement.poses[1].pose.position.y = 1.
    pose(0., 0., 0.)
    node.on_path(replacement)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    # Keep all other valid proof; source scan remains older than its bound
    # throughout the odom callbacks that can trigger pending alignment.
    node.latest_scan.header.stamp.sec = 1
    node.latest_scan.header.stamp.nanosec = 0
    for _ in range(5):
        sample(refresh_scan=False)
    node.latest_scan_at = clock['receipt']
    node._monitor_initial_alignment()
    node.spin_client.send_goal_async.assert_not_called()
    assert node.client.send_goal_async.call_count == 1


def test_old_material_spin_terminal_cannot_dispatch_a_new_goal(rig):
    node, _, pose, sample = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    changed = make_path(stamp=7)
    changed.poses[1].pose.position.y = 1.
    pose(0., 0., 0.)
    node.on_path(changed)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    confirm_stop(node, sample)
    state = node.initial_alignment
    spin_handle = Mock(accepted=True)
    node._on_initial_alignment_response(Mock(result=Mock(return_value=spin_handle)), state)
    new_goal_path = make_path(y=1., stamp=9)
    new_goal_path.poses[0].pose.position.y = 0.
    node.on_goal(goal_for(new_goal_path, stamp=8))
    node.on_path(new_goal_path)
    spin_handle.cancel_goal_async.assert_called_once()
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_SUCCEEDED), state)
    # The stale success cannot treat the canceled route's alignment as proof
    # that this different goal can be driven.
    assert node.client.send_goal_async.call_count == 1
    assert node.initial_alignment is not None
    assert node.initial_alignment is not state
    assert node.initial_alignment['context']['generation'] == node.goal_generation
    reference = node._route_alignment_reference
    assert reference is None or reference['generation'] != node.goal_generation


def attempt_material_spin(node, path, sample):
    node.on_path(path)
    confirm_stop(node, sample)
    state = node.initial_alignment
    if state is not None:
        spin_handle = Mock(accepted=True)
        node._on_initial_alignment_response(Mock(result=Mock(return_value=spin_handle)), state)
        node._on_initial_alignment_result(result_future(GoalStatus.STATUS_ABORTED), state)
    return state


def test_repeated_same_material_route_cannot_refund_its_two_physical_starts(rig):
    node, _, pose, sample = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    replacement = make_path(stamp=7)
    replacement.poses[1].pose.position.y = 1.
    pose(0., 0., 0.)
    node.on_path(replacement)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    # The first failed Spin and a genuinely new path stamp consume the same
    # geometry's two starts, rather than giving a new physical budget per DDS packet.
    confirm_stop(node, sample)
    first = node.initial_alignment
    assert first is not None
    spin_handle = Mock(accepted=True)
    node._on_initial_alignment_response(Mock(result=Mock(return_value=spin_handle)), first)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_ABORTED), first)
    second = deepcopy(replacement)
    second.header.stamp.sec = 8
    assert attempt_material_spin(node, second, sample) is not None
    third = deepcopy(replacement)
    third.header.stamp.sec = 9
    assert attempt_material_spin(node, third, sample) is None
    assert node.navigation_suspended
    assert node.client.send_goal_async.call_count == 1
    assert node.spin_client.send_goal_async.call_count == 2
    assert node.initial_alignment_total_starts == 0
    packet = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert packet['status'] == 'CONTROLLER_HOLD'
    assert node.requested_goal is not None  # Explicit mission target remains owned.


def test_material_route_churn_cannot_bypass_six_start_explicit_goal_budget(rig):
    node, _, pose, sample = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    pose(0., 0., 0.)
    # Distinct>3cm route prefixes all requiring heading setup. A new geometry
    # may get its own2 starts but cannot refund the explicit goal's6 total.
    paths = []
    for index, angle in enumerate((.7, .9, 1.1, 1.3, -.7, -.9, -1.1)):
        path = make_path(stamp=7+index)
        path.poses[1].pose.position.x = .6*cos(angle)
        path.poses[1].pose.position.y = .6*sin(angle)
        paths.append(path)
    node.on_path(paths[0])
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    confirm_stop(node, sample)
    first = node.initial_alignment
    assert first is not None
    spin_handle = Mock(accepted=True)
    node._on_initial_alignment_response(Mock(result=Mock(return_value=spin_handle)), first)
    node._on_initial_alignment_result(result_future(GoalStatus.STATUS_ABORTED), first)
    for path in paths[1:6]:
        assert attempt_material_spin(node, path, sample) is not None
    assert attempt_material_spin(node, paths[6], sample) is None
    assert node.navigation_suspended
    assert node.client.send_goal_async.call_count == 1
    assert node.spin_client.send_goal_async.call_count == 6
    assert node.initial_alignment_total_starts == 0
    packet = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert packet['status'] == 'CONTROLLER_HOLD'


def test_future_stamped_preterminal_odom_is_not_a_postterminal_stop_window(rig):
    node, clock, pose, _ = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    replacement = make_path(stamp=7)
    replacement.poses[1].pose.position.y = 1.
    pose(0., 0., 0.)
    node.on_path(replacement)
    # Source /clock can lag TF/odom briefly. All these genuine odom callbacks
    # were received BEFORE the owned FollowPath terminal. Their headers lead
    # the node clock by up to the existing250ms tolerance, not a new bound.
    node_ns = round(clock['source']*1_000_000_000)
    for lead_ns in (10_000_000, 50_000_000, 90_000_000, 130_000_000,
                    170_000_000, 210_000_000, 249_000_000):
        clock['receipt'] += .04
        odom = deepcopy(node.latest_odom)
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(node_ns+lead_ns, 1_000_000_000)
        node.on_odom(odom)
    assert node._navigation_input_error() is None
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    node._monitor_initial_alignment()
    assert node.client.send_goal_async.call_count == 1
    node.spin_client.send_goal_async.assert_not_called()


def test_changed_distant_suffix_does_not_spin_to_unchanged_local_prefix(rig):
    node, _, pose, _ = rig
    path = make_path(stamp=6)
    handle, context = accepted_old_route(node, path)
    changed = deepcopy(path)
    changed.header.stamp.sec = 7
    changed.poses[-1].pose.position.y = 1.
    # Keep goal matching out of this geometry unit case: the recorded actual
    # route tests above independently exercise same-goal matching.
    node.require_goal_match = False
    pose(0., 0., pi/2)
    node.on_path(changed)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    assert node.client.send_goal_async.call_count == 2
    node.spin_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('authority', [dict(mode='manual'), dict(estop=True), dict(reset=True)])
def test_material_alignment_waits_for_auto_authority_even_when_stopped(rig, authority):
    node, _, pose, sample = rig
    handle, context = accepted_old_route(node, make_path(stamp=6))
    replacement = make_path(stamp=7)
    replacement.poses[1].pose.position.y = 1.
    pose(0., 0., 0.)
    node.on_path(replacement)
    node.on_result(result_future(GoalStatus.STATUS_CANCELED), context, handle=handle)
    for _ in range(6):
        sample(**authority)
    node._monitor_initial_alignment()
    assert node.client.send_goal_async.call_count == 1
    node.spin_client.send_goal_async.assert_not_called()
