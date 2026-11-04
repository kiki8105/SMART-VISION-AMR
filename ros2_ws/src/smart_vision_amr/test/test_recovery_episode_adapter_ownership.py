"""Ownership-only callbacks: no executor, action server or physical motion."""
from copy import deepcopy
from unittest.mock import Mock

from smart_vision_amr.recovery_episode import RecoveryPhase
from test_local_detour_planner import planner_factory, patch, terminal_event
from test_planner_recovery import status
from test_planner_failure_memory import count
from recovery_test_evidence import stationary_evidence


def reverse_context(node):
    """Seed an already accepted action context, not a fabricated clearance proof."""
    coordinator=node.recovery_coordinator
    assert coordinator.begin('owned-reverse',node._local_route_key(),{(17,25)},(),())
    assert coordinator.local_result(False) and coordinator.stop_proved()
    assert coordinator.reverse_verdict(True)
    node.active_recovery_episode_id='owned-reverse'
    node.recovery_active=True
    node.recovery_requested_at=node.clock['steady']
    return coordinator.current


def test_identical_map_republish_preserves_active_local_action(planner_factory):
    node=planner_factory();patch(node)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.FOLLOW
    stamp=node.current_path_stamp_ns;path=tuple(node.planned_world_points)
    node.plan_to=Mock(wraps=node.plan_to)
    for at in (103.,106.):
        stationary_evidence(node,at,x=.525,y=1.275)
        message=deepcopy(node.map[0]);message.header.stamp=node.get_clock().now().to_msg()
        node.on_map(message)
    node.plan_to.assert_not_called()
    assert node.current_path_stamp_ns==stamp and tuple(node.planned_world_points)==path


def test_old_execution_path_cannot_poison_current_route(planner_factory):
    node=planner_factory()
    stale=node.current_path_stamp_ns
    node.plan_to(node.last_goal)
    assert node.current_path_stamp_ns>stale
    node.on_execution_failure(terminal_event(node,path_stamp={
        'sec':stale//1_000_000_000,'nanosec':stale%1_000_000_000}))
    assert count(node)==0 and not node.controller_pending_retry_events
    assert node.planned_world_points


def test_old_success_cannot_finish_replacement_even_same_goal(planner_factory):
    node=planner_factory();old=node.current_path_stamp_ns
    node.plan_to(node.last_goal)
    node.on_navigation_status(status(node,'SUCCEEDED',path_stamp={
        'sec':old//1_000_000_000,'nanosec':old%1_000_000_000}))
    assert not node.goal_completed
    node.on_navigation_status(status(node,'SUCCEEDED'))
    assert node.goal_completed


def test_unowned_reverse_terminal_preserves_current_episode_and_breadcrumbs(planner_factory):
    node=planner_factory();stationary_evidence(node,100.3,x=.525,y=1.275)
    node._record_breadcrumb();episode=reverse_context(node)
    trail=tuple(node.breadcrumbs)
    node.plan_to=Mock(wraps=node.plan_to)
    for terminal in ('RECOVERY_STOPPED','RECOVERY_SUCCEEDED','RECOVERY_FAILED'):
        node.on_navigation_status(status(node,terminal,episode_id='old-reverse'))
    node.plan_to.assert_not_called()
    assert node.recovery_active and node.recovery_coordinator.current is episode
    assert tuple(node.breadcrumbs)==trail and episode.phase is RecoveryPhase.REVERSING


def test_owned_reverse_terminal_advances_once_to_fresh_global(planner_factory):
    node=planner_factory();stationary_evidence(node,100.3,x=.525,y=1.275)
    reverse_context(node)
    node._invalidate_path('Owned reverse is active')
    node.plan_to=Mock(wraps=node.plan_to)
    node.on_navigation_status(status(node,'RECOVERY_SUCCEEDED',episode_id='owned-reverse'))
    node.plan_to.assert_called_once_with(node.last_goal)
    assert not node.recovery_active
    assert node.recovery_coordinator.current.phase is RecoveryPhase.FOLLOW
    assert node.recovery_coordinator.current.path_stamp_ns==node.current_path_stamp_ns
    node.on_navigation_status(status(node,'RECOVERY_SUCCEEDED',episode_id='owned-reverse'))
    node.plan_to.assert_called_once()


def test_missing_reverse_terminal_never_grants_another_motion_action(planner_factory):
    node=planner_factory();reverse_context(node)
    node.plan_to=Mock(wraps=node.plan_to)
    for at in (126.,127.):
        stationary_evidence(node,at,x=.525,y=1.275)
        node.retry_transform()
    node.plan_to.assert_not_called()
    assert node.recovery_active
    assert node.recovery_coordinator.current.phase is RecoveryPhase.REVERSING
    assert node.recovery_pub.publish.call_count==1
