"""Real planner integration of failure evidence; no process or robot is started."""

import copy
import json
from collections import deque
from math import cos, sin
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from smart_vision_amr.failure_memory import FailureMemory
from test_planner_recovery import PlannerHarness, make_goal, status


class FailureHarness(PlannerHarness):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.failure_memory = FailureMemory()
        self.failure_state = 'TRACKING'
        self.failure_reason = ''
        self.last_failure_id = None
        self.pending_failure_id = None
        self.seen_failure_ids = deque(maxlen=256)
        self.localization_health = None
        self.latest_health_at = None
        self.latest_scan = self.latest_scan_at = None
        self.planned_world_points = []
        self.rejoin_anchor = None
        self.rejoin_verified = False
        self.failure_status_pub = Mock()
        # Turn-footprint selection has separate tests; isolate evidence/planning here.
        self._turn_ready = Mock(return_value=True)

    def get_clock(self):
        sec = int(self.clock['ros'])
        nanosec = int(round((self.clock['ros'] - sec) * 1_000_000_000))
        stamp = Time(sec=sec, nanosec=nanosec)
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=sec * 1_000_000_000 + nanosec, to_msg=lambda: stamp))


def refresh(node, x=.55, y=1.55, yaw=0., *, odom_x=None, odom_y=None):
    stamp = node.get_clock().now().to_msg()
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'base_link'
    tf.header.stamp = stamp
    tf.transform.translation.x, tf.transform.translation.y = x, y
    tf.transform.rotation.z, tf.transform.rotation.w = sin(yaw / 2), cos(yaw / 2)
    node.tf_buffer.lookup_transform.return_value = tf
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.header.stamp = stamp
    odom.pose.pose.position.x = x if odom_x is None else odom_x
    odom.pose.pose.position.y = y if odom_y is None else odom_y
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = sin(yaw / 2), cos(yaw / 2)
    node.on_odom(odom)
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.angle_increment = .1
    scan.range_min, scan.range_max = .05, 5.
    scan.ranges = [3.] * 60
    node.latest_scan, node.latest_scan_at = scan, node.clock['steady']
    node.localization_health = {'level': 'ok', 'autonomous_allowed': True,
                                'sent_at_ns': node.get_clock().now().nanoseconds}
    node.latest_health_at = node.clock['steady']


def failure_event(node, **overrides):
    goal = node.last_goal
    payload = {
        'schema_version': 1, 'event_id': '1' * 32,
        'created_at_ns': node.get_clock().now().nanoseconds,
        'detected_at_ns': node.get_clock().now().nanoseconds,
        'goal': {'x': goal.pose.position.x, 'y': goal.pose.position.y,
                 'frame_id': goal.header.frame_id},
        'goal_stamp': {'sec': goal.header.stamp.sec, 'nanosec': goal.header.stamp.nanosec},
        'pose': {'x': .55, 'y': 1.55, 'yaw': 0.},
        'segment': {'start': {'x': .55, 'y': 1.55},
                    'end': {'x': .77, 'y': 1.55}, 'heading': 0.},
        'reason': 'execution_no_progress', 'path_fingerprint': 'f' * 24,
        'evidence': {'duration_sec': 6., 'progress_m': 0.},
        'terminal_confirmed': True,
    }
    payload.update(overrides)
    return String(data=json.dumps(payload))


def count(node):
    return node.failure_memory.snapshot(now=node.clock['steady'])['count']


def path_points(node):
    path = node.path_pub.publish.call_args.args[0]
    return [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]


@pytest.fixture
def prepared(monkeypatch):
    clock = {'steady': 100., 'ros': 100.}
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: clock['steady'])
    node = FailureHarness(clock)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 30
    msg.info.resolution = .1
    msg.info.origin.orientation.w = 1.
    msg.data = [0] * 900
    node.on_map(msg)
    refresh(node)
    goal = make_goal(stamp=100)
    goal.pose.position.x, goal.pose.position.y = 2.55, 1.55
    node.on_goal(goal)
    assert path_points(node)
    return node


def test_execution_failure_retains_goal_and_avoids_same_approach_even_with_smoothing(prepared):
    node = prepared
    goal = node.last_goal
    before = path_points(node)
    source = list(node.map[0].data)
    node.on_execution_failure(failure_event(node))
    assert count(node) == 1
    assert node.last_goal is goal
    replacement_stamp = node.path_pub.publish.call_args.args[0].header.stamp
    assert replacement_stamp.sec * 1_000_000_000 + replacement_stamp.nanosec > 100_000_000_000
    node.clock.update(steady=102.1, ros=102.1)
    refresh(node)
    node.retry_transform()
    after = path_points(node)
    assert after and after != before
    assert list(node.map[0].data) == source
    scopes = node._memory_scopes()
    assert not any(node.failure_memory.segment_blocked(
        a, b, now=node.clock['steady'], map_scope=scopes[0], goal_scope=scopes[1])
        for a, b in zip(after, after[1:]))


@pytest.mark.parametrize('override', [
    {'terminal_confirmed': False}, {'terminal_confirmed': 'yes'},
    {'schema_version': 99}, {'schema_version': True}, {'reason': 'scan_timeout'}, {'reason': 'tf_timeout'},
    {'reason': 'controller_watchdog'}, {'reason': 'cancelled'},
    {'goal': {'x': 8., 'y': 9., 'frame_id': 'map'}},
    {'goal_stamp': {'sec': 99, 'nanosec': 0}},
    {'created_at_ns': 96_000_000_000}, {'created_at_ns': 101_000_000_000},
    {'pose': {'x': float('nan'), 'y': 1.55, 'yaw': 0.}},
])
def test_invalid_nonmotion_or_unmatched_event_cannot_poison_planning(prepared, override):
    node = prepared
    before = path_points(node)
    node.on_execution_failure(failure_event(node, **override))
    assert count(node) == 0
    assert path_points(node) == before


def test_replayed_event_does_not_reinvalidate_or_extend_memory(prepared):
    node = prepared
    event = failure_event(node)
    node.on_execution_failure(event)
    before = node.failure_memory.snapshot(now=100.)
    published = node.path_pub.publish.call_count
    node.clock.update(steady=100.5, ros=100.5)
    refresh(node)
    node.on_execution_failure(event)
    after = node.failure_memory.snapshot(now=100.5)
    assert count(node) == 1
    assert after['entries'][0]['last_seen'] == before['entries'][0]['last_seen']
    assert node.path_pub.publish.call_count == published


@pytest.mark.parametrize('fault', ['health_stale', 'health_critical', 'health_denied',
                                   'odom_stale', 'tf_stale', 'scan_stale',
                                   'health_source_stale', 'odom_source_stale', 'scan_source_stale'])
def test_bad_independent_motion_evidence_does_not_become_route_failure(prepared, fault):
    node = prepared
    if fault == 'health_stale': node.latest_health_at = 98.
    if fault == 'health_critical': node.localization_health['level'] = 'critical'
    if fault == 'health_denied': node.localization_health['autonomous_allowed'] = False
    if fault == 'odom_stale': node.latest_odom = (98., *node.latest_odom[1:])
    if fault == 'tf_stale': node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 98
    if fault == 'scan_stale': node.latest_scan_at = 98.
    if fault == 'health_source_stale': node.localization_health['sent_at_ns'] = 98_000_000_000
    if fault == 'odom_source_stale': node.latest_odom = (node.latest_odom[0], 98_000_000_000, *node.latest_odom[2:])
    if fault == 'scan_source_stale': node.latest_scan.header.stamp.sec = 98
    node.on_execution_failure(failure_event(node))
    assert count(node) == 0


def test_same_goal_resend_and_new_occupancy_content_preserve_failure_scope(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    scopes = node._memory_scopes()
    replacement = copy.deepcopy(node.last_goal)
    replacement.header.stamp.sec = 101
    node.clock.update(steady=101., ros=101.)
    refresh(node)
    node.on_goal(replacement)
    assert node._memory_scopes() == scopes
    assert count(node) == 1
    changed = copy.deepcopy(node.map[0])
    changed.data[0] = 100
    changed.info.map_load_time.sec = 101
    node.on_map(changed)
    assert node._memory_scopes() == scopes
    assert count(node) == 1


def test_expired_failure_restores_route_and_never_changes_saved_map(prepared):
    node = prepared
    initial = path_points(node)
    original_data = list(node.map[0].data)
    node.on_execution_failure(failure_event(node))
    assert count(node) == 1
    node.clock.update(steady=221., ros=221.)
    refresh(node)
    node.plan_to(node.last_goal)
    assert count(node) == 0
    assert path_points(node) == initial
    assert list(node.map[0].data) == original_data


def test_map_tf_only_progress_cannot_verify_rejoin(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    node.rejoin_anchor = {'points': [(.55, 1.55), (2.55, 1.55)],
                          'progress': 0., 'odom': node.latest_odom, 'pose': (.55, 1.55, 0.)}
    node.rejoin_verified = False
    node.clock.update(steady=105., ros=105.)
    # AMCL/TF moved .5m while independent wheel odometry stayed stationary.
    refresh(node, x=1.05, odom_x=.55)
    node.on_navigation_status(status(node, "MOVING"))
    node._update_rejoin()
    assert node.rejoin_verified is False
    assert count(node) == 1


def test_actual_odom_and_path_progress_verify_rejoin_without_forgetting_failure(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    node.rejoin_anchor = {'points': [(.55, 1.55), (2.55, 1.55)],
                          'progress': 0., 'odom': node.latest_odom, 'pose': (.55, 1.55, 0.)}
    node.rejoin_verified = False
    node.recovery_attempts = 2
    node.recovery_attempted_cells = {(5, 15)}
    node.clock.update(steady=105., ros=105.)
    refresh(node, x=1.05)
    node.on_navigation_status(status(node, "MOVING"))
    node._update_rejoin()
    assert node.rejoin_verified is True
    assert node.recovery_attempts == 0
    assert not node.recovery_attempted_cells
    # Advancing on an alternative does not prove that the failed approach works.
    assert count(node) == 1


@pytest.mark.parametrize('fault', ['off_path', 'different_odom_frame', 'stale_evidence',
                                   'odom_jump', 'odom_stamp', 'not_moving'])
def test_rejoin_requires_same_frame_fresh_data_and_small_cross_track(prepared, fault):
    node = prepared
    node.on_execution_failure(failure_event(node))
    node.rejoin_anchor = {'points': [(.55, 1.55), (2.55, 1.55)],
                          'progress': 0., 'odom': node.latest_odom, 'pose': (.55, 1.55, 0.)}
    node.rejoin_verified = False
    node.clock.update(steady=105., ros=105.)
    refresh(node, x=1.05, y=2.05 if fault == 'off_path' else 1.55)
    if fault == 'different_odom_frame':
        node.latest_odom = (*node.latest_odom[:5], 'other_odom')
    if fault == 'stale_evidence':
        node.latest_scan_at = 98.
    if fault == 'odom_jump':
        node.latest_odom = (*node.latest_odom[:2], 2.05, *node.latest_odom[3:])
    if fault == 'odom_stamp':
        old = node.rejoin_anchor['odom']
        node.rejoin_anchor['odom'] = (old[0], node.latest_odom[1], *old[2:])
    node.on_navigation_status(status(node, "MOVING"))
    if fault == 'not_moving':
        node.navigation_progress_at = None
    node._update_rejoin()
    assert node.rejoin_verified is False


@pytest.mark.parametrize('state', ['suspended', 'completed'])
def test_inactive_goal_never_accepts_delayed_failure(prepared, state):
    node = prepared
    event = failure_event(node)
    if state == 'suspended': node.navigation_suspended = True
    if state == 'completed': node.goal_completed = True
    before = node.path_pub.publish.call_count
    node.on_execution_failure(event)
    assert count(node) == 0
    assert node.path_pub.publish.call_count == before


@pytest.mark.parametrize('payload', ['not-json', 'null', '[]', '{}'])
def test_malformed_packet_cannot_change_route_or_memory(prepared, payload):
    node = prepared
    before = path_points(node)
    node.on_execution_failure(String(data=payload))
    assert count(node) == 0
    assert path_points(node) == before


def test_failure_for_robot_pose_far_from_current_tf_is_ignored(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node, pose={'x': 2., 'y': 2., 'yaw': 0.}))
    assert count(node) == 0


def test_failure_event_before_failed_status_cannot_cancel_its_replacement(prepared):
    node = prepared
    event = failure_event(node)
    event_id = json.loads(event.data)['event_id']
    node.on_execution_failure(event)
    alternate = path_points(node)
    assert alternate and count(node) == 1
    publications = node.path_pub.publish.call_count
    node.on_navigation_status(status(node, 'FAILED', route_failure=True,
                                      execution_failure_id=event_id))
    assert node.path_pub.publish.call_count == publications
    assert path_points(node) == alternate
    assert node.pending_failure_id is None
    assert not node.waiting_for_replan


def test_failed_status_before_failure_event_holds_old_route_until_correlated(prepared):
    node = prepared
    event = failure_event(node)
    event_id = json.loads(event.data)['event_id']
    original = path_points(node)
    node.on_navigation_status(status(node, 'FAILED', route_failure=True,
                                      execution_failure_id=event_id))
    assert not path_points(node)
    assert node.pending_failure_id == event_id
    assert count(node) == 0
    publications = node.path_pub.publish.call_count
    node.clock.update(steady=100.2, ros=100.2)
    refresh(node)
    node.retry_transform()
    assert node.path_pub.publish.call_count == publications
    node.on_execution_failure(event)
    assert node.pending_failure_id is None
    assert count(node) == 1
    assert path_points(node) and path_points(node) != original


def test_lost_failure_event_waits_three_seconds_and_fresh_evidence_before_retry(prepared):
    node = prepared
    original = path_points(node)
    node.on_navigation_status(status(node, 'FAILED', route_failure=True,
                                      execution_failure_id='lost-event'))
    publications = node.path_pub.publish.call_count
    node.clock.update(steady=102.9, ros=102.9)
    refresh(node)
    node.retry_transform()
    assert node.pending_failure_id == 'lost-event'
    assert node.path_pub.publish.call_count == publications
    node.clock.update(steady=103.1, ros=103.1)
    refresh(node)
    node.latest_scan_at = 100.
    node.retry_transform()
    assert node.pending_failure_id == 'lost-event'
    assert node.path_pub.publish.call_count == publications
    node.clock.update(steady=103.2, ros=103.2)
    refresh(node)
    node.retry_transform()
    assert node.pending_failure_id is None
    assert path_points(node) == original
    assert count(node) == 0  # Lost evidence cannot invent a spatial restriction.


def test_sensor_loss_during_cancellation_retries_retained_goal_after_fresh_recovery(prepared):
    node = prepared
    goal = node.last_goal
    original = path_points(node)
    node.latest_scan_at = 98.
    node.on_navigation_status(status(node, 'WAIT_REPLAN', route_failure=False,
        message='Execution diagnosis discarded: fresh sensor/TF evidence unavailable'))
    assert not path_points(node)
    assert node.waiting_for_replan
    publications = node.path_pub.publish.call_count
    node.clock.update(steady=102.1, ros=102.1)
    refresh(node)
    node.latest_scan_at = 98.
    node.retry_transform()
    assert node.path_pub.publish.call_count == publications
    node.clock.update(steady=102.2, ros=102.2)
    refresh(node)
    node.retry_transform()
    assert node.last_goal is goal
    assert path_points(node) == original
    assert count(node) == 0


def test_runtime_disabled_memory_is_clear_in_status_without_destroying_records(prepared):
    node = prepared
    node.on_execution_failure(failure_event(node))
    assert count(node) == 1
    node.params['failure_memory_enabled'] = False
    node.publish_failure_status()
    packet = json.loads(node.failure_status_pub.publish.call_args.args[0].data)
    assert packet['state'] == 'CLEAR'
    assert packet['count'] == 0 and packet['active'] == []
    assert count(node) == 1
