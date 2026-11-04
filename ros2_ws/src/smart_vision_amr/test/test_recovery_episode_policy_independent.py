"""Independent scenario tests for one retained-goal recovery episode.

All motion evidence uses stamped messages and the production freshness/stop
gates. Only local-search availability and the final global-search result are
controlled for deterministic scenarios. No ROS node, executor or drive starts.
"""
from copy import deepcopy
import json
from math import sin, cos
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from test_local_detour_planner import planner_factory, hits, terminal_event
from test_planner_recovery import status
from smart_vision_amr.recovery_episode import RecoveryPhase


def evidence(node, at, *, x=.525, y=1.275, yaw=0., mode='auto', orientation_scale=1.):
    """Real independent stationary odometry, source stamps and fresh inputs."""
    node.clock.update(steady=float(at), ros=float(at))
    stamp=node.get_clock().now().to_msg()
    tf=TransformStamped()
    tf.header.frame_id='map';tf.child_frame_id=node.params['base_frame']
    tf.header.stamp=stamp
    tf.transform.translation.x=x;tf.transform.translation.y=y
    tf.transform.rotation.z=orientation_scale*sin(yaw/2)
    tf.transform.rotation.w=orientation_scale*cos(yaw/2)
    node.tf_buffer.lookup_transform.return_value=tf
    odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=stamp
    odom.child_frame_id=node.params['base_frame']
    odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
    odom.pose.pose.orientation.z=orientation_scale*sin(yaw/2)
    odom.pose.pose.orientation.w=orientation_scale*cos(yaw/2)
    node.on_odom(odom)
    scan=LaserScan();scan.header.stamp=stamp
    scan.header.frame_id=node.params['base_frame']
    scan.angle_min=-3.141592653589793;scan.angle_increment=.1
    scan.range_min=.05;scan.range_max=5.;scan.ranges=[float('inf')]*64
    node.on_scan(scan)
    node.on_localization_health(String(data=json.dumps({
        'level':'ok','autonomous_allowed':True,
        'sent_at_ns':node.get_clock().now().nanoseconds})))
    node.on_motion_status(String(data=json.dumps({
        'mode':mode,'available':True,'autonomous_allowed':True,
        'estop_latched':False,'reset_pending':False,
        'sent_at_ns':node.get_clock().now().nanoseconds})))
    node.last_observation_time=at


def stopped(node, *, at=100.3):
    for index in range(6):
        evidence(node, at-.25+.05*index)
    assert node._fresh_motion_pose() is not None
    assert node._reverse_start_stopped()
    # No invented traversed trail: one current verified breadcrumb is enough
    # to prove stop, but it cannot authorize the actual reverse selector.
    assert node._record_breadcrumb()


def probes(node, *, no_global_route=False):
    events=[]
    node._try_local_detour=Mock(side_effect=lambda *a,**k:events.append('local') or False)
    real_reverse=node._maybe_request_reverse
    def reverse(*args,**kwargs):
        events.append('reverse')
        return real_reverse(*args,**kwargs)
    node._maybe_request_reverse=Mock(side_effect=reverse)
    real_global=node.plan_to
    def global_plan(*args,**kwargs):
        events.append('global')
        return real_global(*args,**kwargs)
    node.plan_to=Mock(side_effect=global_plan)
    if no_global_route:
        node.planner.plan=Mock(return_value=[])
        # Both graph and the new continuous connector represent controlled
        # planning availability here; no stop/collision/authority gate is
        # approved by this rejection-only coordinator fixture.
        node._try_metric_connector=Mock(return_value=False)
    return events


def obstacle(node):
    node.on_observed(hits(node,{(17,25)}))


def test_global_no_path_cannot_attempt_reverse_twice_in_one_episode(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    assert events==['local','reverse','global']
    assert not node.recovery_active
    assert not node.path_pub.publish.call_args.args[0].poses
    assert node.recovery_coordinator.current.phase is RecoveryPhase.WAIT_OBSTACLE


def test_identical_fresh_input_timer_ticks_do_not_repeat_completed_global_search(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    completed=list(events)
    for at in (101.,103.,106.):
        evidence(node,at)
        node.on_observed(hits(node,{(17,25)}))
        node.retry_transform()
    assert events==completed==['local','reverse','global']


def test_old_obstacle_retry_latch_cannot_preempt_new_local_reverse_cycle(planner_factory):
    node=planner_factory();stopped(node)
    node.obstacle_hold.attempts=3;node.obstacle_hold.holding=True
    node.obstacle_hold.blocked_cells={(17,25)}
    node.observed={(17,25)}
    events=probes(node,no_global_route=True)
    node.on_execution_failure(terminal_event(node))
    assert events[:3]==['local','reverse','global']


def test_previous_reverse_counter_cannot_skip_new_episode_physical_verdict(planner_factory):
    node=planner_factory();stopped(node)
    node.recovery_attempts=3
    node.recovery_attempted_cells={(10,25)}
    events=probes(node,no_global_route=True)
    obstacle(node)
    assert events==['local','reverse','global']
    assert node.last_reverse_rejection_code not in {'START_ALREADY_ATTEMPTED','ATTEMPT_LIMIT'}


@pytest.mark.parametrize('late_status',['SUCCEEDED','RECOVERY_STOPPED'])
def test_old_terminal_status_cannot_steal_pending_stop_episode(planner_factory,late_status):
    node=planner_factory();evidence(node,100.05)
    assert not node._reverse_start_stopped()
    assert node._record_breadcrumb()
    original_path_stamp=node.current_path_stamp_ns
    events=probes(node)
    obstacle(node)
    before=list(node.local_reference_points)
    trail=tuple(node.breadcrumbs)
    node.on_navigation_status(status(node,late_status,
        episode_id='previous-episode',path_stamp={
            'sec':original_path_stamp//1_000_000_000,
            'nanosec':original_path_stamp%1_000_000_000}))
    assert not node.goal_completed
    assert node.local_reference_points==before
    assert tuple(node.breadcrumbs)==trail
    assert events==['local']
    assert not node._maybe_request_reverse.called and not node.plan_to.called


def test_sensor_delay_preserves_stop_phase_until_real_stationary_window_recovers(planner_factory):
    node=planner_factory();evidence(node,100.05)
    events=probes(node,no_global_route=True)
    obstacle(node)
    episode=node.recovery_coordinator.current
    assert episode.phase is RecoveryPhase.STOP_PROOF
    node.latest_scan_at=95.
    node.retry_transform()
    assert node.recovery_coordinator.current is episode
    assert episode.phase is RecoveryPhase.STOP_PROOF
    assert events==['local']
    # The old sample preceding the sensor gap must age out of the production
    # .50 s stop window; six genuinely dense new samples prove stillness.
    stopped(node,at=100.8)
    node.retry_transform()
    assert events==['local','reverse','global']
    assert episode.phase is RecoveryPhase.WAIT_OBSTACLE


@pytest.mark.parametrize('change',['manual','goal','map'])
def test_prior_episode_ownership_is_revoked_by_explicit_context_change(planner_factory,change):
    node=planner_factory();evidence(node,100.05)
    events=probes(node)
    obstacle(node)
    old=node.recovery_coordinator.current
    assert old.phase is RecoveryPhase.STOP_PROOF
    if change=='manual':
        node.on_navigation_status(String(data=json.dumps({
            'manual_stop':True,'stop_stamp':{'sec':101,'nanosec':0}})))
    elif change=='goal':
        goal=deepcopy(node.last_goal);goal.header.stamp.sec=101
        node.on_goal(goal)
    else:
        message=deepcopy(node.map[0]);message.info.map_load_time.sec+=1
        node.on_map(message)
    assert node.recovery_coordinator.current is not old
    assert not node._maybe_request_reverse.called
    assert node.pending_reverse_probe is None


def test_operator_fresh_same_target_goal_can_restart_after_wait(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    old=node.recovery_coordinator.current
    assert old.phase is RecoveryPhase.WAIT_OBSTACLE
    events.clear()
    goal=deepcopy(node.last_goal);goal.header.stamp.sec=101
    node.on_goal(goal)
    assert events[0]=='global'
    assert node.recovery_coordinator.current is not old


def test_positive_free_evidence_can_open_one_new_local_first_cycle(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    old=node.recovery_coordinator.current
    assert old.phase is RecoveryPhase.WAIT_OBSTACLE
    assert not node._resume_recovery_wait()
    assert events==['local','reverse','global']
    assert node._resume_recovery_wait(positive_free=True)
    assert node.recovery_coordinator.current is not old
    assert events==['local','reverse','global','local','reverse','global']


def test_confirmed_moving_blocker_can_open_one_new_local_first_cycle(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    old=node.recovery_coordinator.current
    assert old.phase is RecoveryPhase.WAIT_OBSTACLE
    node.observed={(25,25)}
    assert node._resume_recovery_wait(moving_obstacle_confirmed=True)
    assert node.recovery_coordinator.current is not old
    assert events==['local','reverse','global','local','reverse','global']


def test_measured_removal_reuses_safe_suffix_and_commits_follow_ownership(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.WAIT_OBSTACLE
    node.observed.clear()
    # The synthetic projector has delivered the original scan event.
    node._record_start_visibility(node.get_clock().now().nanoseconds,node.clock['steady'])
    assert node._resume_recovery_wait(positive_free=True)
    episode=node.recovery_coordinator.current
    assert episode.phase is RecoveryPhase.FOLLOW
    assert episode.path_stamp_ns==node.current_path_stamp_ns
    assert node.planned_world_points
    assert events==['local','reverse','global']


def test_overlay_expiration_is_not_positive_free_or_permission_to_replan(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    old=node.recovery_coordinator.current
    for at in (101.,103.,106.):
        evidence(node,at)
        node.on_observed(hits(node,set()))
        node.retry_transform()
    assert node.recovery_coordinator.current is old
    assert old.phase is RecoveryPhase.WAIT_OBSTACLE
    assert events==['local','reverse','global']


def test_real_stamped_free_visibility_interval_restores_existing_safe_route(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.WAIT_OBSTACLE
    tracked=set(node.obstacle_hold.blocked_cells)
    assert tracked
    for at in (100.35,100.85,101.35,101.85,102.4):
        evidence(node,at)
        free=deepcopy(node.map[0]);free.header.stamp=node.get_clock().now().to_msg()
        free.data=[-1]*(free.info.width*free.info.height)
        for x,y in tracked:
            free.data[y*free.info.width+x]=0
        node.on_visibility(free)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.FOLLOW
    assert node.planned_world_points
    assert events==['local','reverse','global']


def test_same_positive_removal_proof_cannot_reopen_failed_cycle_repeatedly(planner_factory):
    node=planner_factory();stopped(node)
    events=probes(node,no_global_route=True)
    obstacle(node)
    node.observed.clear()
    # Isolate a legitimate removal followed by another unavailable route.
    # This controls local-path availability, not any body/stop safety gate.
    node._reuse_clear_forward_suffix=Mock(return_value=False)
    assert node._resume_recovery_wait(positive_free=True)
    first=list(events)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.WAIT_OBSTACLE
    assert not node._resume_recovery_wait(positive_free=True)
    assert events==first==['local','reverse','global','local','reverse','global']
