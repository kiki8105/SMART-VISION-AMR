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
        self.output_available = True
        self.output_pub = Mock()
        self._ownership_completed_at = time.monotonic()
        self._ros_now = 100_000_000_000

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self._ros_now))


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
