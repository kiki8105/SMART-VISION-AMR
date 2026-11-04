"""Near-range beam-cluster ghosts stop motion but cannot poison A* evidence."""

import math
import time
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
from unittest.mock import patch

from sensor_msgs.msg import LaserScan

from smart_vision_amr.motion_mux import MotionMuxCore, MotionMuxNode
from smart_vision_amr.scan_outlier_guard import NearScanOutlierGuard
from test_live_obstacles import harness


def scan(ranges, stamp_ns=100_000_000_000):
    message = LaserScan()
    message.header.frame_id = 'base_scan'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.angle_min, message.angle_increment = -.5, .01
    message.range_min, message.range_max = .12, 4.
    message.ranges = list(ranges)
    return message


def one_ghost():
    values = [math.inf] * 101
    values[63] = values[65] = .85  # The two adjacent real ranges differ sharply.
    values[64] = .124  # 14 degrees, physically dubious and isolated.
    return values


def four_beam_ghost():
    values = [math.inf] * 101
    values[64:68] = [.12285, .15279, .13440, .12728]
    return values


def test_one_scan_ghost_is_unknown_then_clear_without_occupancy_evidence():
    guard = NearScanOutlierGuard()
    first = guard.classify(scan(one_ghost()), 100_000_000_000)
    assert first.unconfirmed == {64}
    assert not first.confirmed
    assert not guard.classify(scan(one_ghost()), 100_000_000_000).accepted
    clear = guard.classify(scan([.85] * 101), 100_100_000_000)
    assert not clear.unconfirmed and not clear.confirmed


def test_real_one_beam_obstacle_is_confirmed_on_next_scan():
    guard = NearScanOutlierGuard()
    guard.classify(scan(one_ghost()), 100_000_000_000)
    repeated = one_ghost()
    repeated[64] = .14
    confirmed = guard.classify(scan(repeated), 100_100_000_000)
    assert confirmed.confirmed == {64}
    assert not confirmed.unconfirmed


def test_adjacent_beams_confirm_real_object_on_second_scan():
    guard = NearScanOutlierGuard()
    nearby = one_ghost()
    nearby[65] = .17
    first = guard.classify(scan(nearby), 100_000_000_000)
    assert first.unconfirmed == {64, 65}
    assert not first.confirmed
    second = guard.classify(scan(nearby), 100_200_000_000)
    assert second.confirmed == {64, 65}
    assert not second.unconfirmed


def test_four_adjacent_minimum_range_beams_in_one_scan_are_not_confirmation():
    """A captured Gazebo edge echo covered four beams then vanished at 5 Hz."""
    guard = NearScanOutlierGuard()
    clear = [1.] * 101
    ghost = clear.copy()
    ghost[64:68] = [.12285, .15279, .13440, .12728]
    guard.classify(scan(clear), 100_000_000_000)
    first = guard.classify(scan(ghost), 100_200_000_000)
    assert first.unconfirmed == {64, 65, 66, 67}
    assert not first.confirmed
    next_clear = guard.classify(scan(clear), 100_400_000_000)
    assert not next_clear.unconfirmed and not next_clear.confirmed


def test_slow_or_different_return_cannot_confirm_old_single_beam():
    guard = NearScanOutlierGuard()
    guard.classify(scan(one_ghost()), 100_000_000_000)
    assert guard.classify(scan(one_ghost()), 101_000_000_000).unconfirmed == {64}
    changed = one_ghost()
    changed[64] = .28  # No longer in the near-minimum guard band.
    result = guard.classify(scan(changed), 101_100_000_000)
    assert not result.confirmed and not result.unconfirmed


def _project(node, values, dt=.1):
    node.ros_ns += round(dt * 1e9)
    message = scan(values, node.ros_ns)
    node.transform.header.stamp = deepcopy(message.header.stamp)
    node.on_scan(message)
    return message


def test_one_scan_ghost_does_not_enter_observed_or_persistent_map_memory():
    with harness() as node:
        node.params['memory_enabled'] = True
        node.params['save_every_scans'] = 100
        node.scans_since_save = 0
        node.visibility_pub = Mock()
        _project(node, one_ghost())
        assert 6 not in node.observed()
        assert (0, 1) not in node.memory.cells
        assert node.visibility_pub.publish.call_args.args[0].data[6] == -1
        _project(node, [math.inf] * 101)
        assert node.observed() == set()
        assert (0, 1) not in node.memory.cells
        assert node.visibility_pub.publish.call_args.args[0].data[6] == 0


def test_four_beam_ghost_does_not_enter_observed_or_persistent_map_memory():
    with harness() as node:
        node.params['memory_enabled'] = True
        node.params['save_every_scans'] = 100
        node.scans_since_save = 0
        _project(node, four_beam_ghost())
        assert node.observed() == set()
        assert not node.memory.cells
        _project(node, [math.inf] * 101)
        assert node.observed() == set()
        assert not node.memory.cells


def test_persistent_and_spatially_supported_near_hits_enter_real_obstacle_layer():
    with harness() as node:
        node.params['memory_enabled'] = True
        node.params['save_every_scans'] = 100
        node.scans_since_save = 0
        _project(node, one_ghost())
        _project(node, one_ghost())
        assert 6 in node.observed()  # Sensor and near hit are in map cell (0, 1).
        assert node.memory.cells[(0, 1)].occupied_observations == 1
        _project(node, [math.inf] * 101)
        assert node.observed() == set()
    with harness() as node:
        near = one_ghost()
        near[65] = .17
        _project(node, near)
        assert 6 not in node.observed()
        _project(node, near)
        assert 6 in node.observed()


def test_one_scan_near_ghost_keeps_existing_astar_path_and_failure_budget():
    from test_planner_recovery import PlannerHarness, make_goal

    with harness() as sensor:
        planner = PlannerHarness()
        planner.params['live_obstacle_layer_required'] = True
        planner.on_map(sensor.map['message'])
        with patch('smart_vision_amr.astar_planner.time.monotonic', return_value=100.):
            sensor.scan(math.inf)
            planner.on_observed(sensor.observation_pub.publish.call_args.args[0])
            planner.on_goal(make_goal())
            original = list(planner.planned_cells)
            publications = planner.path_pub.publish.call_count
            assert original

            only_one = [math.inf] * 101
            only_one[64] = .124
            _project(sensor, only_one)
            planner.on_observed(sensor.observation_pub.publish.call_args.args[0])
            assert planner.planned_cells == original
            assert planner.path_pub.publish.call_count == publications
            assert planner.obstacle_hold.attempts == 0
            assert planner.failure_memory.snapshot(now=100.)['count'] == 0

            _project(sensor, [math.inf] * 101)
            planner.on_observed(sensor.observation_pub.publish.call_args.args[0])
            assert planner.planned_cells == original


def test_four_beam_ghost_keeps_existing_astar_path_and_failure_budget():
    from test_planner_recovery import PlannerHarness, make_goal

    with harness() as sensor:
        planner = PlannerHarness()
        planner.params['live_obstacle_layer_required'] = True
        planner.on_map(sensor.map['message'])
        with patch('smart_vision_amr.astar_planner.time.monotonic', return_value=100.):
            sensor.scan(math.inf)
            planner.on_observed(sensor.observation_pub.publish.call_args.args[0])
            planner.on_goal(make_goal())
            original = list(planner.planned_cells)
            publications = planner.path_pub.publish.call_count
            assert original

            _project(sensor, four_beam_ghost())
            planner.on_observed(sensor.observation_pub.publish.call_args.args[0])
            assert planner.planned_cells == original
            assert planner.path_pub.publish.call_count == publications
            assert planner.obstacle_hold.attempts == 0
            assert planner.failure_memory.snapshot(now=100.)['count'] == 0


class _MuxHarness(MotionMuxNode):
    def __init__(self):
        self.core = MotionMuxCore(clock=lambda: 10.)
        self.core.receive_auto(.1, .0)
        self.near_scan_guard = NearScanOutlierGuard()
        self.near_scan_guard_last_stamp_ns = -1
        self._scan_guard_enabled = True
        self._scan_guard_radius = .22
        self._scan_guard_base_frame = 'base_footprint'
        self._scan_guard_horizon = .5
        self._scan_guard_sensor_timeout = 1.
        self._scan_guard_tf_cache = {'base_scan': (0., 0., 0.)}
        self._scan_guard_sample = None
        self._scan_guard_pending = None
        self._scan_guard_pose_timeout = .25
        self._scan_guard_fixed_frame = 'odom'
        self.last_published_output = (0., 0.)
        self.output_available = True
        self.output_pub = Mock()
        self._ownership_completed_at = time.monotonic()
        self._ros_now = 100_000_000_000

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self._ros_now))

    def _scan_guard_pose(self, stamp_ns=0):
        return (0., 0., 0.), stamp_ns or self._ros_now


def test_mux_stops_first_ghost_and_releases_only_on_fresh_clear_scan():
    node = _MuxHarness()
    assert node.core.output() == (.1, .0)
    node.on_scan_guard(scan(one_ghost()))
    assert node.core.output() == (0., 0.)
    assert node.output_pub.publish.call_args.args[0].linear.x == 0.
    assert node.core.status()['scan_guard_hold']
    # A replay or invalid all-NaN packet cannot falsely release the stop.
    node.on_scan_guard(scan([math.inf] * 101))
    node._ros_now += 100_000_000
    node.on_scan_guard(scan([math.nan] * 101, node._ros_now))
    assert node.core.output() == (0., 0.)
    node.on_scan_guard(scan([math.inf] * 101, node._ros_now))
    assert node.core.output() == (.1, .0)
    assert node.output_pub.publish.call_args.args[0].linear.x == .1


def test_mux_keeps_real_close_obstacle_stopped_even_after_second_scan():
    node = _MuxHarness()
    node.on_scan_guard(scan(one_ghost()))
    node._ros_now += 100_000_000
    node.on_scan_guard(scan(one_ghost(), node._ros_now))
    assert node.core.output() == (0., 0.)
    assert 'Confirmed' in node.core.status()['scan_guard_reason']
    node._ros_now += 100_000_000
    node.on_scan_guard(scan([.85] * 101, node._ros_now))
    assert node.core.output() == (.1, .0)


def test_mux_rechecks_direction_change_without_a_new_scan_or_stuck_zero_output():
    node = _MuxHarness()
    obstacle = [math.inf]*101
    obstacle[50] = .25  # Outside the 22 cm body but in the forward sweep.
    node.on_scan_guard(scan(obstacle))
    assert node.core.scan_guard_code == 'SWEPT_INTERSECTION'
    assert node.core.output() == (0., 0.)
    node.core.receive_auto(-.1, 0.)
    node._publish_output_locked()
    assert not node.core.scan_guard_hold
    assert node.core.output() == (-.1, 0.)


def test_mux_missing_tf_cannot_release_even_a_fresh_clear_scan():
    node = _MuxHarness()
    node._scan_guard_tf_cache = {}
    node.scan_guard_tf_buffer = Mock()
    node.scan_guard_tf_buffer.lookup_transform.side_effect = RuntimeError('unknown frame')
    node.on_scan_guard(scan([math.inf]*101))
    assert node.core.scan_guard_hold
    assert node.core.scan_guard_code == 'TF_UNAVAILABLE'
    node._scan_guard_tf_cache['base_scan'] = (-.064, 0., 0.)
    node._ros_now += 100_000_000
    node.on_scan_guard(scan([math.inf]*101, node._ros_now))
    assert not node.core.scan_guard_hold


def test_mux_fresh_mounting_offset_clears_parallel_near_return_without_mutating_scan():
    node = _MuxHarness()
    node._scan_guard_tf_cache['base_scan'] = (-.064, 0., 0.)
    message = scan([.22])
    message.angle_min = math.pi/2
    before = deepcopy(message)
    node.on_scan_guard(message)
    assert not node.core.scan_guard_hold
    assert node.core.scan_guard_code == 'CLEAR'
    assert message == before


def test_mux_guard_is_fail_closed_on_scan_freshness_and_pending_unknown_tf():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    assert not node.core.scan_guard_hold
    node._ros_now += 1_100_000_000
    node._publish_output_locked()
    assert node.core.scan_guard_code == 'STALE_SCAN'
    assert node.core.output() == (0., 0.)


def test_mux_old_actual_velocity_is_checked_until_stop_before_away_command():
    node = _MuxHarness()
    node.last_published_output = (.1, 0.)
    node.core.receive_auto(-.1, 0.)
    values = [math.inf]*101
    values[50] = .25
    node.on_scan_guard(scan(values))
    assert node.core.output() == (0., 0.)
    assert node.last_published_output == (0., 0.)
    node._publish_output_locked()
    assert node.core.output() == (-.1, 0.)


def test_mux_rejects_inverted_scan_plane_and_caches_only_valid_mounting():
    node = _MuxHarness()
    node._scan_guard_tf_cache = {}
    node.scan_guard_tf_buffer = Mock()
    rotation = SimpleNamespace(x=1., y=0., z=0., w=0.)
    node.scan_guard_tf_buffer.lookup_transform.return_value = SimpleNamespace(
        transform=SimpleNamespace(translation=SimpleNamespace(x=-.064, y=0., z=.12),
                                  rotation=rotation))
    assert node._scan_guard_extrinsic('base_scan') is None
    assert not node._scan_guard_tf_cache
    rotation.x, rotation.w = 0., 1.
    assert node._scan_guard_extrinsic('base_scan') == (-.064, 0., 0.)
    node.scan_guard_tf_buffer.lookup_transform.reset_mock()
    assert node._scan_guard_extrinsic('base_scan') == (-.064, 0., 0.)
    node.scan_guard_tf_buffer.lookup_transform.assert_not_called()


def test_delayed_scan_is_rebased_by_odometry_before_collision_decision():
    from smart_vision_amr.scan_outlier_guard import scan_points_in_base, swept_body_decision
    node = _MuxHarness()
    node.core.receive_auto(.22, 0.)
    stamp = node._ros_now-200_000_000
    message = scan([.36], stamp)
    message.angle_min = 0.
    # Uncompensated 36 cm looks clear; ego motion of 4.4 cm consumes the gap.
    raw = scan_points_in_base(message, (0., 0., 0.))
    assert not swept_body_decision(raw, .22, 0.).swept
    node._scan_guard_pose = lambda at=0: (((0., 0., 0.) if at else (.044, 0., 0.)),
                                         at or node._ros_now)
    node.on_scan_guard(message)
    assert node.core.scan_guard_code == 'SWEPT_INTERSECTION'
    assert node.core.scan_guard_metrics['scan_age_seconds'] == .2
    assert node.core.output() == (0., 0.)


def test_two_hundred_ms_scan_delay_does_not_inflate_parallel_side_wall():
    node = _MuxHarness()
    node.core.receive_auto(.22, 0.)
    message = scan([.24], node._ros_now-200_000_000)
    message.angle_min = math.pi/2
    node._scan_guard_pose = lambda at=0: (((0., 0., 0.) if at else (.044, 0., 0.)),
                                         at or node._ros_now)
    node.on_scan_guard(message)
    assert node.core.scan_guard_code == 'CLEAR'
    assert node.core.output() == (.22, 0.)


def test_scan_a_few_ms_ahead_of_tf_reuses_valid_sample_without_zero_or_ramp_reset():
    node = _MuxHarness()
    tf_latest = [node._ros_now]
    def pose(at=0):
        return None if at > tf_latest[0] else ((0., 0., 0.), at or tf_latest[0])
    node._scan_guard_pose = pose
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-50_000_000))
    for _ in range(5):
        node._ros_now += 10_000_000
        node.on_scan_guard(scan([math.inf]*101, node._ros_now))
        assert node._scan_guard_pending is not None
        assert not node.core.scan_guard_hold
        assert node.last_published_output == (.1, 0.)
        tf_latest[0] = node._ros_now+40_000_000
        node._ros_now = tf_latest[0]
        node._publish_output_locked()
        assert node._scan_guard_pending is None
        assert node.last_published_output == (.1, 0.)
        tf_latest[0] += 50_000_000
        node._ros_now = tf_latest[0]
    assert all(msg.args[0].linear.x == .1 for msg in node.output_pub.publish.call_args_list)


def test_pending_scan_with_imminent_body_hit_still_stops_on_first_return():
    node = _MuxHarness()
    tf_latest = node._ros_now
    node._scan_guard_pose = lambda at=0: (None if at > tf_latest else
                                         ((0., 0., 0.), at or tf_latest))
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-50_000_000))
    node._ros_now += 10_000_000
    node.on_scan_guard(scan(one_ghost(), node._ros_now))
    assert node._scan_guard_pending is not None
    assert node.core.scan_guard_code == 'BODY_INTERSECTION'
    assert node.core.scan_guard_support == 'unconfirmed'
    assert node.core.output() == (0., 0.)


def test_missing_or_stale_current_pose_does_not_authorize_motion():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    node._scan_guard_pose = lambda at=0: None
    node._publish_output_locked()
    assert not node.core.scan_guard_hold
    assert node.core.scan_guard_metrics['pose_source'] == 'cached'
    # Reuse does not renew the cached pose's timestamp or its age budget.
    node._ros_now += 250_000_000
    node._publish_output_locked()
    assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'
    node._scan_guard_pose = lambda at=0: ((0., 0., 0.), node._ros_now-300_000_000)
    node._publish_output_locked()
    assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'
    assert node.core.scan_guard_metrics['pose_source'] == 'stale'


def test_real_tf_buffer_future_scan_waits_for_interpolation_without_zero_pulses():
    from types import MethodType
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer

    node = _MuxHarness()
    node.scan_guard_tf_buffer = Buffer()
    node._scan_guard_pose = MethodType(MotionMuxNode._scan_guard_pose, node)

    def publish_pose(stamp_ns):
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        transform.transform.translation.x = .22*(stamp_ns-100_000_000_000)/1.e9
        transform.transform.rotation.w = 1.
        node.scan_guard_tf_buffer.set_transform(transform, 'test_odom')

    publish_pose(node._ros_now-100_000_000)
    publish_pose(node._ros_now)
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-50_000_000))
    assert node.last_published_output == (.1, 0.)
    for _ in range(5):
        node._ros_now += 10_000_000
        node.on_scan_guard(scan([math.inf]*101, node._ros_now))
        assert node._scan_guard_pending is not None
        assert node.last_published_output == (.1, 0.)
        node._ros_now += 40_000_000
        publish_pose(node._ros_now)
        node._publish_output_locked()
        assert node._scan_guard_pending is None
        assert node.last_published_output == (.1, 0.)
        node._ros_now += 50_000_000
        publish_pose(node._ros_now)
    assert all(call.args[0].linear.x == .1 for call in node.output_pub.publish.call_args_list)


def test_real_tf_buffer_interpolates_95_and_58_ms_future_latest_pose():
    from types import MethodType
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer

    for future_ms in (95, 58):
        node = _MuxHarness()
        node.scan_guard_tf_buffer = Buffer()
        node._scan_guard_pose = MethodType(MotionMuxNode._scan_guard_pose, node)
        for offset in (-100, future_ms):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
            stamp = node._ros_now+offset*1_000_000
            transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            transform.transform.translation.x = .22*offset/1.e3
            transform.transform.rotation.w = 1.
            node.scan_guard_tf_buffer.set_transform(transform, 'test_odom')
        node.on_scan_guard(scan([math.inf]*101, node._ros_now-50_000_000))
        assert node.core.scan_guard_code == 'CLEAR'
        assert node.core.scan_guard_metrics['pose_source'] == 'interpolated'
        assert node.core.scan_guard_metrics['pose_age_seconds'] == 0.
        metrics = node.core.scan_guard_metrics
        assert metrics['future_lead_seconds'] == future_ms/1000.
        assert metrics['pose_uncertainty_seconds'] >= future_ms/1000.
        assert metrics['longitudinal_uncertainty_m'] >= node.core.max_linear_velocity*future_ms/1000.
        assert node._scan_guard_pose_cache[0][1] == node._ros_now
        assert abs(node._scan_guard_pose_cache[0][0][0]) < 1.e-12
        assert node.last_published_output == (.1, 0.)


def test_lookup_gap_reuses_original_pose_without_renewing_stamp_or_motion_bound():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    initial_cache = node._scan_guard_pose_cache
    def missing(at=0):
        node._scan_guard_pose_lookup = {'failure_kind': 'lookup', 'lookup_error': 'LookupException: temporary'}
        return None
    node._scan_guard_pose = missing
    node._ros_now += 100_000_000
    node._publish_output_locked()
    assert node.core.scan_guard_code == 'CLEAR'
    metrics = node.core.scan_guard_metrics
    assert metrics['pose_source'] == 'cached'
    assert metrics['pose_age_seconds'] == .1
    assert metrics['lookup_error'] == 'LookupException: temporary'
    assert metrics['longitudinal_uncertainty_m'] >= node.core.max_linear_velocity*.1
    assert node._scan_guard_pose_cache == initial_cache
    node._ros_now += 150_000_000
    node._publish_output_locked()
    assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'
    assert node.core.scan_guard_metrics['lookup_error'] == 'LookupException: temporary'


def test_invalid_stale_and_unbounded_future_pose_cannot_fall_back_to_cache():
    for fault in ('geometry', 'stale', 'future'):
        node = _MuxHarness()
        node.on_scan_guard(scan([math.inf]*101))
        assert node._scan_guard_pose_cache is not None
        def invalid(at=0):
            node._scan_guard_pose_lookup = {'failure_kind': 'geometry', 'lookup_error': 'Invalid quaternion'}
            return None
        node._scan_guard_pose = (invalid if fault == 'geometry' else
            lambda at=0: ((0., 0., 0.), node._ros_now+(-300_000_000 if fault == 'stale' else 300_000_000)))
        node._publish_output_locked()
        assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'
        assert node._scan_guard_pose_cache is None
        assert node.core.scan_guard_metrics['pose_source'] == {
            'geometry': 'invalid_geometry', 'stale': 'stale', 'future': 'future_out_of_bounds'}[fault]


def test_pose_geometry_validation_forbids_verified_pose_cache_fallback():
    from types import MethodType
    node = _MuxHarness()
    node.on_scan_guard(scan([math.inf]*101))
    node._scan_guard_pose = MethodType(MotionMuxNode._scan_guard_pose, node)
    node.scan_guard_tf_buffer = Mock()
    node.scan_guard_tf_buffer.lookup_transform.return_value = SimpleNamespace(
        transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0.),
                                  rotation=SimpleNamespace(x=0., y=0., z=0., w=0.)))
    node._publish_output_locked()
    assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'
    assert node.core.scan_guard_metrics['pose_source'] == 'invalid_geometry'
    assert 'Invalid odometry transform geometry' in node.core.scan_guard_metrics['lookup_error']
    assert node._scan_guard_pose_cache is None


def test_all_guard_faults_include_scan_and_pose_lookup_diagnostics():
    node = _MuxHarness()
    node.on_scan_guard(scan([math.nan]*101))
    assert node.core.scan_guard_code == 'INVALID_SCAN'
    assert set(('scan_age_seconds', 'pose_age_seconds', 'lookup_error', 'pose_source')) <= node.core.scan_guard_metrics.keys()


def test_cached_interpolated_pose_preserves_future_lead_plus_receipt_elapsed():
    node = _MuxHarness()
    initial_ns = node._ros_now
    def future(at=0):
        return ((0., 0., 0.), at or initial_ns+95_000_000)
    node._scan_guard_pose = future
    with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=100.):
        node.on_scan_guard(scan([math.inf]*101, initial_ns-50_000_000))
        initial_cache = node._scan_guard_pose_cache
        assert node.core.scan_guard_metrics['future_lead_seconds'] == .095
        assert node.core.scan_guard_metrics['pose_uncertainty_seconds'] == .095
    node._scan_guard_pose = lambda at=0: None
    node._ros_now += 100_000_000
    with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=100.1):
        node._publish_output_locked()
        metrics = node.core.scan_guard_metrics
        assert metrics['pose_source'] == 'cached'
        assert metrics['pose_age_seconds'] == .1
        assert metrics['future_lead_seconds'] == .095
        assert abs(metrics['pose_uncertainty_seconds']-.195) < 1.e-12
        assert abs(metrics['longitudinal_uncertainty_m']-
                   node.core.max_linear_velocity*(.195+node.GUARD_COMPUTE_BUDGET)) < 1.e-12
        assert node._scan_guard_pose_cache == initial_cache
    node._ros_now += 150_000_000
    with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=100.25):
        node._publish_output_locked()
        assert node.core.scan_guard_code == 'POSE_UNAVAILABLE'


def test_repeated_same_future_tf_does_not_refresh_its_evidence_receipt():
    node = _MuxHarness()
    initial_ns = node._ros_now
    node._scan_guard_pose = lambda at=0: ((0., 0., 0.), at or initial_ns+95_000_000)
    with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=100.):
        node.on_scan_guard(scan([math.inf]*101, initial_ns-50_000_000))
    node._ros_now += 60_000_000
    with patch('smart_vision_amr.motion_mux.time.monotonic', return_value=100.06):
        node._publish_output_locked()
        metrics = node.core.scan_guard_metrics
        assert metrics['future_lead_seconds'] == .095
        assert abs(metrics['pose_uncertainty_seconds']-.155) < 1.e-12
        assert node._scan_guard_pose_cache[1] == 100.
