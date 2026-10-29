"""Bound real execution retries, retain the goal, and require positive removal proof."""
import copy
import json

import pytest
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from test_planner_failure_memory import (
    prepared, refresh, failure_event, path_points, count,
)
from test_planner_recovery import status


PHYSICAL_HITS = {(10, 15), (10, 16)}


def ready(node, now, **motion_changes):
    node.clock.update(steady=float(now), ros=float(now))
    refresh(node)
    motion = {'sent_at_ns': node.get_clock().now().nanoseconds,
              'mode': 'auto', 'available': True, 'autonomous_allowed': True,
              'estop_latched': False, 'reset_pending': False}
    motion.update(motion_changes)
    node.on_motion_status(String(data=json.dumps(motion)))


def arm(node):
    original = path_points(node)
    node.observed.update(PHYSICAL_HITS)
    for i in range(3):
        ready(node, 100.+i*.1)
        node.on_execution_failure(failure_event(node, event_id=f'{i+1:032x}'))
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 3
    assert set(node.obstacle_hold.blocked_cells) == PHYSICAL_HITS
    assert not path_points(node)
    return original


def visibility(node, *, free=(), occupied=(), unknown=True):
    msg = OccupancyGrid()
    msg.header.frame_id = node.map[0].header.frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.info = copy.deepcopy(node.map[0].info)
    msg.data = [-1 if unknown else 0] * (msg.info.width*msg.info.height)
    for x, y in free:
        msg.data[y*msg.info.width+x] = 0
    for x, y in occupied:
        msg.data[y*msg.info.width+x] = 100
    return msg


def prove_free(node, start=101., duration=2.):
    for i in range(int(duration/.25)+1):
        ready(node, start+i*.25)
        node.on_visibility(visibility(node, free=PHYSICAL_HITS))


def test_three_confirmed_failures_hold_instead_of_replanning_forever(prepared):
    node = prepared
    goal = node.last_goal
    arm(node)
    publications = node.path_pub.publish.call_count
    for now in (101., 103., 105.):
        ready(node, now)
        node.retry_transform()
        node.plan_to(goal)
    assert node.path_pub.publish.call_count == publications
    assert node.last_goal is goal
    assert node.obstacle_hold.holding
    node.publish_failure_status()
    packet = json.loads(node.failure_status_pub.publish.call_args.args[0].data)
    assert packet['state'] == 'OBSTACLE_HOLD'
    assert packet['retry_count'] == packet['retry_limit'] == 3


def test_actual_removal_confirmation_releases_associated_memory_and_same_goal(prepared):
    node = prepared
    goal = node.last_goal
    map_data = list(node.map[0].data)
    original = arm(node)
    assert count(node) == 1
    prove_free(node)
    assert not node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 0
    assert not node.obstacle_episode_failure_ids
    assert count(node) == 0
    assert node.last_goal is goal
    assert path_points(node) == original
    assert list(node.map[0].data) == map_data


def test_missing_hits_or_expired_failure_ttl_never_prove_obstacle_removed(prepared):
    node = prepared
    arm(node)
    ready(node, 221.)
    node.observed.clear()  # Emulate transient overlay expiry; no positive ray proof.
    node.retry_transform()
    node.on_visibility(visibility(node))  # Unknown cells remain unknown.
    assert count(node) == 0
    assert node.obstacle_hold.holding
    assert not path_points(node)


@pytest.mark.parametrize('fault', ['stale', 'replayed', 'frame', 'origin', 'shape',
                                   'partial_free', 'occupied', 'nearby_hit', 'scan_fault'])
def test_bad_visibility_or_sensor_evidence_breaks_clear_confirmation(prepared, fault):
    node = prepared
    arm(node)
    ready(node, 101.)
    initial = visibility(node, free=PHYSICAL_HITS)
    node.on_visibility(initial)
    assert node.obstacle_hold.clear_since == 101.
    ready(node, 101.25)
    msg = visibility(node, free=PHYSICAL_HITS)
    if fault == 'stale': msg.header.stamp.sec = 98
    if fault == 'replayed': msg = initial
    if fault == 'frame': msg.header.frame_id = 'other_map'
    if fault == 'origin': msg.info.origin.position.x += .05
    if fault == 'shape': msg.data = msg.data[:-1]
    if fault == 'partial_free': msg.data[16*msg.info.width+10] = -1
    if fault == 'occupied': msg.data[16*msg.info.width+10] = 100
    if fault == 'nearby_hit': msg.data[15*msg.info.width+11] = 100
    if fault == 'scan_fault': node.latest_scan_at = 98.
    node.on_visibility(msg)
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.clear_since is None
    prove_free(node, start=102., duration=1.75)
    assert node.obstacle_hold.holding
    ready(node, 104.)
    node.on_visibility(visibility(node, free=PHYSICAL_HITS))
    assert not node.obstacle_hold.holding


@pytest.mark.parametrize('motion_fault', [
    {'mode': 'manual'}, {'estop_latched': True}, {'reset_pending': True},
    {'available': False}, {'autonomous_allowed': False}, {'sent_at_ns': 98_000_000_000},
])
def test_manual_estop_or_unavailable_motion_arbiter_never_auto_resumes(prepared, motion_fault):
    node = prepared
    arm(node)
    for i in range(9):
        ready(node, 101.+i*.25, **motion_fault)
        node.on_visibility(visibility(node, free=PHYSICAL_HITS))
    assert node.obstacle_hold.holding
    assert not path_points(node)


def test_lost_visibility_stream_breaks_interval_even_when_sensors_later_return(prepared):
    node = prepared
    arm(node)
    ready(node, 101.)
    node.on_visibility(visibility(node, free=PHYSICAL_HITS))
    ready(node, 101.6)
    node.retry_transform()
    assert node.obstacle_hold.clear_since is None
    ready(node, 103.)
    node.on_visibility(visibility(node, free=PHYSICAL_HITS))
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.clear_since == 103.


def test_manual_cancel_cancels_auto_resume_after_obstacle_removal(prepared):
    node = prepared
    arm(node)
    node.on_navigation_status(status(node, 'CANCELED', manual_stop=True,
                                    stop_stamp={'sec': 101, 'nanosec': 0}))
    assert node.navigation_suspended
    assert not node.obstacle_hold.holding
    publications = node.path_pub.publish.call_count
    prove_free(node)
    assert node.path_pub.publish.call_count == publications


def test_same_goal_resend_preserves_hold_while_new_target_gets_new_budget(prepared):
    node = prepared
    arm(node)
    same = copy.deepcopy(node.last_goal)
    same.header.stamp.sec = 101
    ready(node, 101.)
    node.on_goal(same)
    assert node.obstacle_hold.holding and node.obstacle_hold.attempts == 3
    assert not path_points(node)
    other = copy.deepcopy(same)
    other.header.stamp.sec = 102
    other.pose.position.x = 2.45
    ready(node, 102.)
    node.on_goal(other)
    assert not node.obstacle_hold.holding and node.obstacle_hold.attempts == 0
    assert node.last_goal is other
    assert path_points(node)


def test_mapped_wall_hits_do_not_arm_a_remove_the_wall_wait(prepared):
    node = prepared
    changed = copy.deepcopy(node.map[0])
    changed.data[15*changed.info.width+10] = 100
    node.on_map(changed)
    node.observed = {(10, 15)}  # Only the mapped occupied cell is a wall echo.
    for now in (101., 102., 103., 104.):
        ready(node, now)
        assert not node._register_obstacle_failure(.55, 1.55)
    assert not node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 0


def test_partial_rejoin_progress_cannot_reset_the_same_obstacle_retry_budget(prepared):
    from math import pi
    node = prepared
    node.observed.update(PHYSICAL_HITS)
    for i in range(2):
        ready(node, 100.+i*.1)
        node.on_execution_failure(failure_event(node, event_id=f'{i+1:032x}'))
    assert node.obstacle_hold.attempts == 2
    episode_ids = set(node.obstacle_episode_failure_ids)
    assert episode_ids
    # Verified departure along a new northbound leg is useful progress, but it
    # has not completed the target or proven this physical object was removed.
    ready(node, 101.)
    refresh(node, yaw=pi/2)
    node.rejoin_anchor = {'points': [(.55, 1.55), (.55, 2.55)],
                          'progress': 0., 'odom': node.latest_odom,
                          'pose': (.55, 1.55, pi/2)}
    node.rejoin_verified = False
    node.recovery_attempts = 2
    ready(node, 106.)
    refresh(node, x=.55, y=2.05, yaw=pi/2)
    node.on_navigation_status(status(node, 'MOVING'))
    node._update_rejoin()
    assert node.rejoin_verified
    assert node.recovery_attempts == 0  # The short reverse budget may recover.
    assert node.obstacle_hold.attempts == 2
    assert node.obstacle_episode_failure_ids == episode_ids
    node.on_execution_failure(failure_event(node, event_id=f'{3:032x}',
        pose={'x': .55, 'y': 2.05, 'yaw': pi/2},
        segment={'start': {'x': .55, 'y': 2.05},
                 'end': {'x': .55, 'y': 2.27}, 'heading': pi/2}))
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 3
    assert not path_points(node)



def hit_message(node, cells):
    from std_msgs.msg import Int32MultiArray
    width = node.map[0].info.width
    return Int32MultiArray(data=[y*width+x for x, y in cells])


def newly_blocking_cell(node):
    # Select a genuine still-ahead cell on this particular published approach,
    # rather than repeatedly reporting one hit while no approach is active.
    actual_remaining = sorted(node._remaining_path_cells(),
                              key=lambda cell: (cell[0], abs(cell[1]-15)))
    return next(cell for cell in actual_remaining
                if 8 <= cell[0] <= 15 and cell not in node.observed)


@pytest.mark.parametrize('layer', ['observed', 'memory'])
def test_three_distinct_published_approaches_blocked_by_new_hits_enter_hold(prepared, layer):
    node = prepared
    on_hits = node.on_observed if layer == 'observed' else node.on_memory
    observed = set()
    for index in range(3):
        ready(node, 100.+index*2.1)
        if index:
            node.retry_transform()
        assert path_points(node)
        observed.add(newly_blocking_cell(node))
        on_hits(hit_message(node, observed))
        assert node.obstacle_hold.attempts == index+1
        assert not path_points(node)
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 3


def test_unchanged_scans_or_hit_churn_without_a_new_path_do_not_spend_retries(prepared):
    node = prepared
    ready(node, 100.)
    blocking = {newly_blocking_cell(node)}
    node.on_observed(hit_message(node, blocking))
    assert node.obstacle_hold.attempts == 1
    assert not node.planned_cells
    for index in range(1, 8):
        ready(node, 100.+index*.1)
        node.on_observed(hit_message(node, blocking))
        node.on_observed(hit_message(node, blocking | {(28, 28)}))
    assert node.obstacle_hold.attempts == 1
    assert not node.obstacle_hold.holding


@pytest.mark.parametrize('kind', ['unrelated', 'passed'])
def test_unrelated_or_already_passed_hits_do_not_spend_obstacle_retries(prepared, kind):
    node = prepared
    ready(node, 100.1)
    if kind == 'passed':
        refresh(node, x=1.55)
        hit = (6, 15)
    else:
        hit = (10, 25)
    node.path_pub.publish.reset_mock()
    node.on_observed(hit_message(node, {hit}))
    assert node.obstacle_hold.attempts == 0
    node.path_pub.publish.assert_not_called()


@pytest.mark.parametrize('kind', ['manual', 'estop', 'sensor_fault'])
def test_manual_or_unhealthy_evidence_does_not_count_a_blocked_approach(prepared, kind):
    node = prepared
    ready(node, 100.1, mode='manual' if kind == 'manual' else 'auto',
          estop_latched=kind == 'estop')
    if kind == 'sensor_fault':
        node.latest_scan_at = 98.
    node.on_observed(hit_message(node, {newly_blocking_cell(node)}))
    assert node.obstacle_hold.attempts == 0
    assert not node.obstacle_hold.holding


def test_ten_cm_neighbor_veto_survives_float32_map_resolution_rounding(prepared):
    node = prepared
    changed = OccupancyGrid()
    changed.header.frame_id = 'map'
    changed.info.width = changed.info.height = 60
    import struct
    # Emulate a received ROS float32 after wire serialization/deserialization.
    changed.info.resolution = struct.unpack('<f', struct.pack('<f', .05))[0]
    changed.info.origin.orientation.w = 1.
    changed.data = [0]*3600
    node.on_map(changed)
    assert changed.info.resolution > .05
    tracked = {(20, 30)}
    node.observed = set(tracked)
    for _ in range(3):
        node._register_obstacle_failure(.55, 1.55)
    assert node.obstacle_hold.holding
    for i in range(9):
        ready(node, 101.+i*.25)
        node.on_visibility(visibility(node, free=tracked, occupied={(22, 30)}))
    # Two cells are ~10 cm apart. A shifted hit there still vetoes removal;
    # truncating int(.10/resolution) would silently reduce the guard to 5 cm.
    assert node.obstacle_hold.holding
    assert node.obstacle_hold.clear_since is None



def test_new_blocking_object_preserves_goal_budget_but_releases_only_its_memory(prepared):
    node = prepared
    goal = node.last_goal
    object_a, object_b = (10, 15), (19, 15)
    node.observed = {object_a}
    ready(node, 100.)
    node.on_execution_failure(failure_event(node, event_id='a'*32))
    failure_a = node.last_failure_id
    assert node.obstacle_hold.attempts == 1
    assert node.obstacle_hold.blocked_cells == {object_a}
    # Independent map/odom data place the robot past A and near a distinct B.
    node.observed = {object_a, object_b}
    for i in range(2):
        ready(node, 110.+i)
        refresh(node, x=1.55)
        node.on_execution_failure(failure_event(node, event_id=f'{i+10:032x}',
            pose={'x': 1.55, 'y': 1.55, 'yaw': 0.},
            segment={'start': {'x': 1.55, 'y': 1.55},
                     'end': {'x': 1.77, 'y': 1.55}, 'heading': 0.}))
    failure_b = node.last_failure_id
    assert failure_b != failure_a
    assert node.obstacle_hold.holding and node.obstacle_hold.attempts == 3
    assert node.obstacle_hold.blocked_cells == {object_b}
    assert node.obstacle_episode_failure_ids == {failure_b}
    assert count(node) == 2
    # A remains physically occupied behind the robot. It must not prevent B's
    # verified removal, nor may removing B erase A's still-valid failed approach.
    for i in range(9):
        ready(node, 112.+i*.25)
        refresh(node, x=1.55)
        node.on_visibility(visibility(node, free={object_b}, occupied={object_a}))
    assert not node.obstacle_hold.holding
    assert node.obstacle_hold.attempts == 0
    assert node.last_goal is goal
    assert path_points(node)
    snapshot = node.failure_memory.snapshot(now=node.clock['steady'])
    assert [entry['id'] for entry in snapshot['entries']] == [failure_a]
    assert object_a in node.observed and object_b not in node.observed
