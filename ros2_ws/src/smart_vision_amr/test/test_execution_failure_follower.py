"""Failure events are spatial evidence and must follow a confirmed action stop."""
import json
import time
from unittest.mock import Mock, patch
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from smart_vision_amr.execution_progress import ExecutionProgress
from test_path_follower_cancel import FollowerHarness, make_path, goal_for, result_future
from test_follow_path_watchdog import accepted_follow


def prepared():
    node = FollowerHarness()
    node.parameters.update({'execution_failure_enabled': True, 'execution_progress_window': 6.,
                            'execution_min_progress': .05, 'execution_heading_progress': .15,
                            'execution_evidence_timeout': 1., 'curve_base_frame': 'base_footprint'})
    node.execution_progress = ExecutionProgress()
    node.execution_reference = node.execution_route_id = None
    node.pending_execution_failure = None
    node.execution_last_ros_ns = node.execution_clock_advanced_at = None
    node.execution_failure_pub = Mock()
    node.latest_navigation_command = Twist()
    node.latest_navigation_command.linear.x = .1
    node.tf_buffer = Mock()
    node.requested_goal = goal_for(make_path(), stamp=1)
    refresh(node)
    handle = accepted_follow(node)
    refresh(node)
    node.test_base = time.monotonic()
    return node, handle


def refresh(node):
    now = time.monotonic()
    stamp = node.get_clock().now().to_msg()
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.angle_increment = .1
    scan.range_min, scan.range_max = .05, 4.
    scan.ranges = [2.]*60
    odom = Odometry()
    odom.header.stamp = stamp
    odom.pose.pose.orientation.w = 1.
    node.latest_scan, node.latest_odom = scan, odom
    node.latest_scan_at = node.latest_odom_at = now
    node.latest_navigation_command_at = now
    node.localization_health = {'level': 'ok', 'autonomous_allowed': True,
                                'sent_at_ns': node.get_clock().now().nanoseconds}
    node.localization_health_at = now
    node.motion_status = {'mode': 'auto', 'available': True, 'estop_latched': False,
                          'autonomous_allowed': True, 'output_linear': .1, 'output_angular': 0.,
                          'sent_at_ns': node.get_clock().now().nanoseconds}
    node.motion_status_at = now
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.transform.rotation.w = 1.
    node.tf_buffer.lookup_transform.return_value = tf


def detect(node):
    for step in range(71):
        with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+step*.1):
            node.clock_seconds = 10+step
            refresh(node)
            node._monitor_execution_progress()
            if node.pending_execution_failure is not None:
                return
    raise AssertionError('No stuck execution was detected')


def test_failure_event_waits_for_cancel_terminal_then_emits_once():
    node, handle = prepared()
    detect(node)
    assert node.cancel_requested and node.active_goal is handle
    handle.cancel_goal_async.assert_called_once()
    node.execution_failure_pub.publish.assert_not_called()
    # Merely receiving a cancel acknowledgment cannot start the planner's recovery.
    node._on_follow_cancel_response(Mock(result=Mock(return_value=Mock(goals_canceling=[1]))), handle)
    node.execution_failure_pub.publish.assert_not_called()
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+6.1):
        refresh(node)
        node.on_result(result_future(), handle=handle)
    node.execution_failure_pub.publish.assert_called_once()
    event = json.loads(node.execution_failure_pub.publish.call_args.args[0].data)
    assert event['terminal_confirmed'] is True
    assert event['reason'] == 'execution_no_progress'
    assert event['goal_stamp'] == {'sec': 1, 'nanosec': 0}
    assert len(event['event_id']) == 32 and len(event['path_fingerprint']) == 24
    assert abs(event['segment']['end']['x']-.22) < 1e-6
    assert node.active_goal is None and node.pending_path is None
    last = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert last['execution_failure_id'] == event['event_id'] and last['route_failure']


def test_new_route_cannot_start_until_failure_terminal_is_confirmed():
    node, handle = prepared()
    detect(node)
    node.on_path(make_path(1.))
    assert node.client.send_goal_async.call_count == 1
    assert node.pending_path is None


def test_actual_controller_abort_can_report_spatial_failure():
    node, handle = prepared()
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2., speed=.1)))
    node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    event = json.loads(node.execution_failure_pub.publish.call_args.args[0].data)
    assert event['reason'] == 'controller_aborted' and event['terminal_confirmed']


@pytest.mark.parametrize('fault', ['scan', 'odom', 'health', 'manual', 'estop', 'clock', 'tf',
                                  'scan_numeric', 'scan_angle', 'odom_numeric', 'output_zero'])
def test_bad_evidence_or_non_auto_control_cannot_accumulate_failure(fault):
    node, _ = prepared()
    for step in range(71):
        with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+step*.1):
            node.clock_seconds = 10+step
            refresh(node)
            if fault == 'scan': node.latest_scan_at = 0.
            if fault == 'odom': node.latest_odom.header.stamp.sec = 0
            if fault == 'health': node.localization_health['level'] = 'critical'
            if fault == 'manual': node.motion_status['mode'] = 'manual'
            if fault == 'estop': node.motion_status['estop_latched'] = True
            if fault == 'clock': node.execution_last_ros_ns = node.clock_seconds*1_000_000_000+1
            if fault == 'tf': node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 0
            if fault == 'scan_numeric': node.latest_scan.ranges = [float('nan')]*60
            if fault == 'scan_angle': node.latest_scan.angle_min = float('nan')
            if fault == 'odom_numeric': node.latest_odom.pose.pose.position.x = float('nan')
            if fault == 'output_zero': node.motion_status['output_linear'] = 0.
            node._monitor_execution_progress()
    assert node.pending_execution_failure is None
    node.execution_failure_pub.publish.assert_not_called()


def test_watchdog_abort_is_never_route_failure():
    node, handle = prepared()
    node.follow_watchdog_fault = 'Feedback timed out'
    node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()


def test_explicit_cancel_does_not_record_failure():
    node, handle = prepared()
    detect(node)
    node.cancel_navigation(Trigger.Request(), Trigger.Response())
    node.on_result(result_future(), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()


def test_sensor_disconnect_while_canceling_discards_spatial_diagnosis():
    node, handle = prepared()
    detect(node)
    node.latest_scan_at = 0.
    node.on_result(result_future(), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    assert node.pending_execution_failure is None


def test_equivalent_replans_do_not_reset_the_execution_window():
    node, _ = prepared()
    route_id = node.execution_route_id
    node.execution_progress.observe(
        __import__('smart_vision_amr.execution_progress', fromlist=['ProgressSample']).ProgressSample(
            1., 1_000_000_000, 0., 0., .8, 0., .1, 0.), route_id)
    node._select_execution_reference(make_path())
    assert node.execution_route_id == route_id
    assert len(node.execution_progress.samples) == 1


def test_runtime_detection_limits_replace_window_and_drop_old_evidence():
    node, _ = prepared()
    old_detector = node.execution_progress
    node.parameters['execution_progress_window'] = 9.
    node._monitor_execution_progress()
    assert node.execution_progress is not old_detector
    assert node.execution_progress.window == 9.
    assert len(node.execution_progress.samples) == 1


def test_invalid_runtime_detection_limits_do_not_poison_a_route():
    node, _ = prepared()
    node.parameters['execution_min_progress'] = float('nan')
    node._monitor_execution_progress()
    assert node.pending_execution_failure is None
    node.execution_failure_pub.publish.assert_not_called()


def test_goal_success_racing_cancellation_discards_diagnosis_and_preserves_success():
    node, handle = prepared()
    detect(node)
    node.on_result(result_future(GoalStatus.STATUS_SUCCEEDED), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    assert node.pending_execution_failure is None
    assert node.active_goal is None
    messages = [json.loads(call.args[0].data) for call in node.status_pub.publish.call_args_list]
    assert messages[-1]['status'] == 'SUCCEEDED'
    assert not any(message.get('route_failure') for message in messages)


@pytest.mark.parametrize('missing', ['raw_command', 'raw_zero', 'mux_zero', 'stale_command',
                                    'before_action', 'no_feedback'])
def test_abort_without_actual_current_execution_intent_does_not_record_route_failure(missing):
    node, handle = prepared()
    if missing != 'no_feedback':
        node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2., speed=.1)))
    if missing == 'raw_command': node.latest_navigation_command = None
    if missing == 'raw_zero': node.latest_navigation_command.linear.x = 0.
    if missing == 'mux_zero': node.motion_status['output_linear'] = 0.
    if missing == 'stale_command': node.latest_navigation_command_at = time.monotonic()-2.
    if missing == 'before_action': node.latest_navigation_command_at = node.follow_requested_at-.1
    node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['status'] == 'FAILED' and not status.get('route_failure', False)


def test_post_terminal_failure_barrier_rejects_delayed_route_until_correlated_new_stamp():
    node, handle = prepared()
    detect(node)
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+6.1):
        refresh(node)
        node.on_result(result_future(), handle=handle)
    event = json.loads(node.execution_failure_pub.publish.call_args.args[0].data)
    assert node.execution_replan_cutoff_ns == event['created_at_ns']
    equal = make_path(stamp=event['created_at_ns']//1_000_000_000)
    equal.header.stamp.nanosec = event['created_at_ns']%1_000_000_000
    node.on_path(equal)
    node.on_path(make_path(stamp=1))
    assert node.client.send_goal_async.call_count == 1
    new = make_path(stamp=equal.header.stamp.sec)
    new.header.stamp.nanosec = equal.header.stamp.nanosec+1
    node.on_path(new)
    assert node.client.send_goal_async.call_count == 2
    assert node.execution_replan_cutoff_ns is None


def test_new_explicit_goal_and_manual_stop_clear_post_failure_barrier():
    node, _ = prepared()
    node.execution_replan_cutoff_ns = 5_000_000_000
    node.on_goal(goal_for(make_path(), stamp=6))
    assert node.execution_replan_cutoff_ns is None
    node.execution_replan_cutoff_ns = 7_000_000_000
    node.cancel_navigation(Trigger.Request(), Trigger.Response())
    assert node.execution_replan_cutoff_ns is None


def test_real_command_then_zero_before_abort_keeps_bounded_executed_intent_evidence():
    node, handle = prepared()
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2., speed=.1)))
    node._monitor_execution_progress()
    assert node.last_executed_intent is not None
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+.5):
        refresh(node)
        node.latest_navigation_command.linear.x = 0.
        node.motion_status['output_linear'] = 0.
        node._monitor_execution_progress()
        node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    event = json.loads(node.execution_failure_pub.publish.call_args.args[0].data)
    assert event['reason'] == 'controller_aborted'


@pytest.mark.parametrize('fault', ['expired', 'different_action', 'health_fault', 'command_transport_fault'])
def test_cached_intent_cannot_survive_fault_or_unrelated_action(fault):
    node, handle = prepared()
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2., speed=.1)))
    node._monitor_execution_progress()
    with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+(14. if fault == 'expired' else .5)):
        refresh(node)
        node.latest_navigation_command.linear.x = 0.
        node.motion_status['output_linear'] = 0.
        if fault == 'different_action': node.follow_request_seq += 1
        if fault == 'health_fault':
            node.localization_health['level'] = 'critical'
            node._monitor_execution_progress()
            node.localization_health['level'] = 'ok'
        if fault == 'command_transport_fault': node.latest_navigation_command_at -= 3.
        node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()


def test_follower_detects_alternating_dwb_turns_that_cross_zero_briefly():
    node, _ = prepared()
    commands = [.2, .2, 0., -.2, -.2, 0.]
    for step in range(71):
        with patch('smart_vision_amr.path_follower.time.monotonic', return_value=node.test_base+step*.1):
            node.clock_seconds = 10+step
            refresh(node)
            node.latest_navigation_command.linear.x = 0.
            node.latest_navigation_command.angular.z = commands[step%6]
            node.motion_status['output_linear'] = 0.
            node.motion_status['output_angular'] = commands[step%6]
            node._monitor_execution_progress()
            if node.pending_execution_failure:
                break
    assert node.pending_execution_failure['reason'] == 'execution_oscillation'


@pytest.mark.parametrize('interruption', ['manual', 'health'])
def test_brief_interruption_before_terminal_discards_pending_diagnosis_even_after_recovery(interruption):
    node, handle = prepared()
    detect(node)
    if interruption == 'manual':
        bad = dict(node.motion_status, mode='manual')
        node.on_motion_status(String(data=json.dumps(bad)))
        node.on_motion_status(String(data=json.dumps(dict(bad, mode='auto'))))
    else:
        bad = dict(node.localization_health, level='critical', autonomous_allowed=False)
        node.on_localization_health(String(data=json.dumps(bad)))
        node.on_localization_health(String(data=json.dumps(dict(bad, level='ok', autonomous_allowed=True))))
    assert node.pending_execution_failure is None
    assert node.execution_discarded_reason is not None
    assert node.active_goal is handle and node.cancel_requested
    node.on_path(make_path(stamp=node.clock_seconds+1))
    assert node.client.send_goal_async.call_count == 1
    node.on_result(result_future(), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    assert node.pending_path is None
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['status'] == 'WAIT_REPLAN' and status['route_failure'] is False


@pytest.mark.parametrize('interruption', ['manual', 'health'])
def test_brief_interruption_invalidates_prior_intent_before_abort(interruption):
    node, handle = prepared()
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2., speed=.1)))
    node._monitor_execution_progress()
    assert node.last_executed_intent is not None
    if interruption == 'manual':
        node.on_motion_status(String(data=json.dumps(dict(node.motion_status, mode='manual'))))
    else:
        node.on_localization_health(String(data=json.dumps(dict(node.localization_health, level='warning'))))
    refresh(node)
    node.latest_navigation_command.linear.x = 0.
    node.motion_status['output_linear'] = 0.
    node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()


def test_disabling_detection_suppresses_spontaneous_abort_memory_as_well_as_progress_detection():
    node, handle = prepared()
    node.on_feedback(SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2., speed=.1)))
    node._monitor_execution_progress()
    node.parameters['execution_failure_enabled'] = False
    node.on_result(result_future(GoalStatus.STATUS_ABORTED), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    assert node.last_executed_intent is None


@pytest.mark.parametrize('monitor_runs', [False, True])
def test_disabling_pending_diagnosis_keeps_stop_barrier_but_never_publishes_failure_memory(monitor_runs):
    node, handle = prepared()
    detect(node)
    node.parameters['execution_failure_enabled'] = False
    if monitor_runs:
        node._monitor_execution_progress()
    node.on_path(make_path(stamp=node.clock_seconds+1))
    assert node.client.send_goal_async.call_count == 1 and node.active_goal is handle
    node.on_result(result_future(), handle=handle)
    node.execution_failure_pub.publish.assert_not_called()
    assert node.pending_execution_failure is None
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['status'] == 'WAIT_REPLAN' and status['route_failure'] is False
