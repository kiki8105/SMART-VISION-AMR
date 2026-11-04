"""Offline planner integration for static starts and obstacle-focused retreat."""
from collections import deque
import json
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from smart_vision_amr.map_utils import occupancy_to_grid
from smart_vision_amr.reverse_recovery import Breadcrumb
from test_planner_failure_memory import prepared, refresh


def scenario(node):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = .1
    msg.info.width, msg.info.height = 30, 20
    msg.info.origin.orientation.w = 1.
    msg.data = [100 if y == 0 else 0 for y in range(20) for x in range(30)]
    node.map = msg, occupancy_to_grid(msg, 65, 2)
    node.raw_static_grid = occupancy_to_grid(msg, 65, 0)
    node.params.update(obstacle_inflation_cells=2, robot_radius=.22)
    node.map_signature = (65, 2, 0)
    node.observed = {(16, 3)}
    node.remembered = set()
    node.last_observation_time = node.clock['steady']
    # The actual pose is .2999 m from the hit square. With continuous-pose
    # start certification, this baseline may now route behind the hit without
    # invoking the separate reverse action.
    refresh(node, x=1.3001, y=.38, yaw=0.)
    node.breadcrumbs = deque((Breadcrumb(97.+i, 1.0001+.10*i, .38, 0.)
                             for i in range(4)), maxlen=300)
    node.breadcrumb_odom_anchor = node.latest_odom
    node.odom_samples = deque((
        (99.75+i*.05, 99_750_000_000+i*50_000_000,
         node.latest_odom[2], node.latest_odom[3], node.latest_odom[4], node.latest_odom[5])
        for i in range(6)), maxlen=100)
    node.latest_odom_child_frame = 'base_link'
    motion = {'sent_at_ns': node.get_clock().now().nanoseconds,
              'mode': 'auto', 'available': True, 'autonomous_allowed': True,
              'estop_latched': False, 'reset_pending': False}
    node.on_motion_status(String(data=json.dumps(motion)))
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = 2.55, .38
    goal.pose.orientation.w = 1.
    node.last_goal = goal
    node.recovery_pub.reset_mock()
    return goal


def reverse_required_scenario(node):
    """A clear rear trail exists, but no certified A* first edge exists.

    A diagonal scan hit leaves the *measured pose* clear of the obstacle and
    wall. The raster start and its immediate neighbors are inside the hit's
    complete footprint inflation. A 20 cm reverse reaches a free cell from
    which the retained goal is reachable around the hit.
    """
    goal = scenario(node)
    node.params['robot_radius'] = .24
    node.observed = {(15, 6)}
    refresh(node, x=1.3001, y=.4001, yaw=0.)
    node.latest_odom_child_frame = 'base_link'
    node.breadcrumbs = deque((Breadcrumb(97.+i, 1.0001+.10*i, .4001, 0.)
                             for i in range(4)), maxlen=300)
    node.breadcrumb_odom_anchor = node.latest_odom
    node.odom_samples = deque((
        (99.75+i*.05, 99_750_000_000+i*50_000_000,
         node.latest_odom[2], node.latest_odom[3], node.latest_odom[4], node.latest_odom[5])
        for i in range(6)), maxlen=100)
    goal.pose.position.y = .4001
    return goal


def test_live_hit_at_inflated_start_requests_checked_retreat_parallel_to_wall(prepared):
    node = prepared
    goal = reverse_required_scenario(node)
    source = list(node.map[0].data)
    node.plan_to(goal)
    request = json.loads(node.recovery_pub.publish.call_args.args[0].data)
    assert request['distance'] == pytest.approx(.20)
    assert request['start_cell'] == [13, 4]
    assert request['exit_cell'] == [11, 4]
    assert request['blocking_cells'] == [[15, 6]]
    cert = request['certified_start']
    assert cert == {
        'frame_id': 'odom', 'child_frame_id': 'base_link',
        'stamp_ns': 100_000_000_000, 'x': 1.3001, 'y': .4001, 'yaw': 0.,
        'certified_at_ns': 100_000_000_000, 'max_position_drift_m': .003,
        'max_yaw_drift_rad': .01, 'sweep_reserve_m': .01,
        'map_frame': 'map', 'map_base_frame': 'base_link', 'map_x': 1.3001, 'map_y': .4001, 'map_yaw': 0.,
        'map_stamp_ns': 100_000_000_000}
    assert node.last_goal is goal and node.recovery_active
    assert list(node.map[0].data) == source
    assert not node.path_pub.publish.call_args.args[0].poses
    node.plan_to(goal)
    assert node.recovery_pub.publish.call_count == 1


def test_static_inflated_start_requests_checked_wall_parallel_retreat(prepared):
    node = prepared
    goal = scenario(node)
    msg = node.map[0]
    msg.info.resolution = .05
    msg.info.width, msg.info.height = 80, 40
    msg.data = [100 if y == 0 or (x, y) == (33, 10) else 0
                for y in range(40) for x in range(80)]
    raw = occupancy_to_grid(msg, 65, 0)
    grid = occupancy_to_grid(msg, 65, 5)
    # Uniform 5-cell inflation covers the current cell because of the
    # diagonal obstacle, although the side-wall clearance stays .28 m.
    refresh(node, x=1.425, y=.33, yaw=0.)
    node.latest_odom_child_frame = 'base_link'
    node.breadcrumbs = deque((Breadcrumb(97.+i, 1.125+.10*i, .33, 0.)
                              for i in range(4)), maxlen=300)
    node.breadcrumb_odom_anchor = node.latest_odom
    node.odom_samples = deque((
        (99.75+i*.05, 99_750_000_000+i*50_000_000,
         node.latest_odom[2], node.latest_odom[3], node.latest_odom[4], node.latest_odom[5])
        for i in range(6)), maxlen=100)
    node.map = msg, grid
    node.raw_static_grid = raw
    node.observed.clear()
    assert grid.is_static_obstacle((28, 6))
    assert not raw.is_static_obstacle((28, 6))
    assert node._maybe_request_reverse((28, 6), (51, 7), msg, grid, raw,
                                       set(), set(), {}, True), node.last_reverse_rejection_code
    request = json.loads(node.recovery_pub.publish.call_args.args[0].data)
    assert request['distance'] == pytest.approx(.10)
    assert request['exit_cell'] == [26, 6]
    assert request['blocking_cells'] == []
    assert node.last_goal is goal and node.recovery_active


@pytest.mark.parametrize('fault', [
    'rear_hit', 'missing_history', 'stale_scan', 'stale_pose', 'health_denied',
    'manual', 'estop', 'reset_pending', 'attempt_budget', 'same_cell',
    'raw_start', 'stale_observed', 'invalid_radius', 'moving', 'missing_child_frame', 'sparse_odom',
])
def test_retreat_never_bypasses_independent_gates_or_collision(prepared, fault):
    node = prepared
    goal = reverse_required_scenario(node)
    if fault == 'rear_hit': node.observed.add((10, 3))
    if fault == 'missing_history': node._clear_breadcrumbs()
    if fault == 'stale_scan': node.latest_scan_at = 98.
    if fault == 'stale_pose': node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 98
    if fault == 'health_denied': node.localization_health['autonomous_allowed'] = False
    if fault == 'manual': node.latest_motion_status['mode'] = 'manual'
    if fault == 'estop': node.latest_motion_status['estop_latched'] = True
    if fault == 'reset_pending': node.latest_motion_status['reset_pending'] = True
    if fault == 'attempt_budget': node.recovery_attempts = 3
    if fault == 'same_cell': node.recovery_attempted_cells.add((13, 4))
    if fault == 'raw_start': node.observed.add((13, 4))
    if fault == 'stale_observed': node.last_observation_time = 98.
    if fault == 'invalid_radius': node.params['robot_radius'] = float('nan')
    if fault == 'moving':
        old = node.odom_samples[0]
        node.odom_samples[0] = (old[0], old[1], old[2]-.01, old[3], old[4], old[5])
    if fault == 'missing_child_frame': node.latest_odom_child_frame = ''
    if fault == 'sparse_odom':
        node.odom_samples = deque([node.odom_samples[0], node.odom_samples[-1]], maxlen=100)
    node.plan_to(goal)
    node.recovery_pub.publish.assert_not_called()
    assert node.last_goal is goal


def test_static_failure_without_live_cause_cannot_use_obstacle_focused_mode(prepared):
    node = prepared
    goal = scenario(node)
    # The retained failed edges already disconnect the start without this hit.
    edges = {((13, 3), (13+dx, 3+dy)) for dx in (-1, 0, 1)
             for dy in (-1, 0, 1) if dx or dy}
    node._failure_constraints = Mock(return_value=(edges, {}))
    node._safe_failure_edge_fallback = Mock(return_value=[])
    node.plan_to(goal)
    node.recovery_pub.publish.assert_not_called()


def static_scenario(node):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = .05
    msg.info.width = msg.info.height = 40
    msg.info.origin.orientation.w = 1.
    msg.data = [100 if (x, y) == (24, 18) else 0
                for y in range(40) for x in range(40)]
    node.map = msg, occupancy_to_grid(msg, 65, 6)
    node.raw_static_grid = occupancy_to_grid(msg, 65, 0)
    node.params.update(obstacle_inflation_cells=6, robot_radius=.22)
    node.map_signature = (65, 6, 0)
    node.observed = set()
    node.remembered = set()
    node.last_observation_time = node.clock['steady']
    refresh(node, x=.925, y=.925, yaw=0.)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = .525, .925
    goal.pose.orientation.w = 1.
    node.last_goal = goal
    node.recovery_pub.reset_mock()
    return goal


def test_static_quantization_uses_private_single_cell_view_with_full_inflation_tail(prepared):
    node = prepared
    goal = static_scenario(node)
    source = list(node.map[0].data)
    original_grid = node.map[1]
    node.plan_to(goal)
    assert node.planned_cells[:2] == [(18, 18), (17, 18)]
    assert original_grid.is_static_obstacle((18, 18))
    assert all(not original_grid.is_static_obstacle(cell) for cell in node.planned_cells[1:])
    assert list(node.map[0].data) == source
    node.recovery_pub.publish.assert_not_called()


def test_static_start_certificate_uses_deployment_radius_without_stock_floor(prepared, monkeypatch):
    from smart_vision_amr import astar_planner as planner_module
    node = prepared
    goal = static_scenario(node)
    node.params['robot_radius'] = .17
    certified_radii = []
    original = planner_module.certified_start_only_edges

    def capture_radius(*args, **kwargs):
        certified_radii.append(args[6] * node.map[0].info.resolution)
        return original(*args, **kwargs)

    monkeypatch.setattr(planner_module, 'certified_start_only_edges', capture_radius)
    node.plan_to(goal)
    assert certified_radii == pytest.approx([.17])


@pytest.mark.parametrize('fault', ['scan_stale', 'health_denied', 'raw_start', 'raw_hit', 'goal_inflated'])
def test_static_certificate_preserves_freshness_raw_layers_and_goal_inflation(prepared, fault):
    node = prepared
    goal = static_scenario(node)
    if fault == 'scan_stale': node.latest_scan_at = 98.
    if fault == 'health_denied': node.localization_health['autonomous_allowed'] = False
    if fault == 'raw_start':
        node.map[0].data[18*40+18] = -1
        node.raw_static_grid = occupancy_to_grid(node.map[0], 65, 0)
    if fault == 'raw_hit': node.observed.add((18, 18))
    if fault == 'goal_inflated': goal.pose.position.x = 1.125
    node.plan_to(goal)
    assert not node.planned_cells
    node.recovery_pub.publish.assert_not_called()


def test_private_start_view_cannot_enable_later_diagonal_across_original_start_corner(prepared):
    from smart_vision_amr.grid import Grid
    node = prepared
    goal = static_scenario(node)
    node.map[0].data = [0]*1600
    node.raw_static_grid = Grid(tuple(tuple(False for x in range(40)) for y in range(40)))
    original = Grid(tuple(tuple((x, y) == (18, 18) for x in range(40)) for y in range(40)))
    node.map = node.map[0], original
    goal.pose.position.x, goal.pose.position.y = .925, .875
    start, east, north = (18, 18), (19, 18), (18, 17)
    # Force the first step east. Clearing S must not then permit E->N across
    # the original inflated S side cell, even though the first E edge is safe.
    constraints = {(start, (18+dx, 18+dy)) for dx in (-1, 0, 1)
                   for dy in (-1, 0, 1) if (dx or dy) and (18+dx, 18+dy) != east}
    node._failure_constraints = Mock(return_value=(constraints, {}))
    node.plan_to(goal)
    assert node.planned_cells[:2] == [start, east]
    assert node.planned_cells[-1] == north
    assert (east, north) not in set(zip(node.planned_cells, node.planned_cells[1:]))
    assert original.is_static_obstacle(start)


def test_manual_stop_during_recovery_clears_trail_and_ignores_late_success(prepared):
    from test_planner_recovery import status
    node = prepared
    goal = reverse_required_scenario(node)
    node.plan_to(goal)
    assert node.recovery_active
    publications = node.recovery_pub.publish.call_count
    node.on_navigation_status(status(node, 'CANCELED', manual_stop=True,
        stop_stamp={'sec': 100, 'nanosec': 0}))
    assert not node.recovery_active and not node.breadcrumbs
    assert node.navigation_suspended
    node.on_navigation_status(status(node, 'RECOVERY_SUCCEEDED'))
    node.retry_transform()
    assert node.last_goal is goal
    assert node.recovery_pub.publish.call_count == publications


@pytest.mark.parametrize('delay,allowed', [(.4, True), (.8, False)])
def test_expensive_candidate_work_cannot_refresh_measured_certificate_age(prepared, monkeypatch, delay, allowed):
    from smart_vision_amr import astar_planner as planner_module
    node = prepared
    goal = reverse_required_scenario(node)
    original = planner_module.select_reverse_egress
    def slow_selector(*args, **kwargs):
        result = original(*args, **kwargs)
        node.clock.update(steady=100.+delay, ros=100.+delay)
        return result
    monkeypatch.setattr(planner_module, 'select_reverse_egress', slow_selector)
    node.plan_to(goal)
    if allowed:
        cert = json.loads(node.recovery_pub.publish.call_args.args[0].data)['certified_start']
        assert cert['certified_at_ns'] == 100_000_000_000
        assert cert['stamp_ns'] == 100_000_000_000
    else:
        node.recovery_pub.publish.assert_not_called()


def test_changed_odom_child_frame_discards_reverse_trail_and_stationary_window(prepared):
    from nav_msgs.msg import Odometry
    node = prepared
    scenario(node)
    assert node.breadcrumbs and len(node.odom_samples) >= 4
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'different_base'
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 100_000_000
    msg.pose.pose.position.x, msg.pose.pose.position.y = 1.3001, .38
    msg.pose.pose.orientation.w = 1.
    node.on_odom(msg)
    assert not node.breadcrumbs
    assert len(node.odom_samples) == 1
    assert not node._reverse_start_stopped()


@pytest.mark.parametrize('times,position_error,yaw_error,expected', [
    ((99.62, 99.74, 99.87, 100.0), 0., 0., True),
    ((99.62, 99.74, 99.95, 100.0), 0., 0., False),
    ((99.62, 99.74, 99.87, 100.0), .003, 0., False),
    ((99.62, 99.74, 99.87, 100.0), 0., .006, False),
])
def test_stationary_reverse_evidence_allows_bounded_10hz_jitter_but_not_motion(
        prepared, times, position_error, yaw_error, expected):
    node = prepared
    scenario(node)
    latest = node.latest_odom
    node.odom_samples = deque((
        (received, round(received * 1_000_000_000),
         latest[2] + (position_error if index == 0 else 0.),
         latest[3], latest[4] + (yaw_error if index == 0 else 0.), latest[5])
        for index, received in enumerate(times)), maxlen=100)
    assert node._reverse_start_stopped() is expected


def test_actual_five_cell_static_inflation_corner_has_full_stock_radius_exit(prepared):
    node = prepared
    goal = static_scenario(node)
    msg = node.map[0]
    msg.data = [100 if (x, y) == (23, 23) else 0
                for y in range(40) for x in range(40)]
    node.map = msg, occupancy_to_grid(msg, 65, 5)
    node.raw_static_grid = occupancy_to_grid(msg, 65, 0)
    node.params['obstacle_inflation_cells'] = 5
    node.map_signature = (65, 5, 0)
    original = node.map[1]
    assert original.is_static_obstacle((18, 18))
    node.plan_to(goal)
    assert node.planned_cells[:2] == [(18, 18), (17, 18)]
    assert all(not original.is_static_obstacle(cell) for cell in node.planned_cells[1:])
    assert original.is_static_obstacle((18, 18))
    node.recovery_pub.publish.assert_not_called()


@pytest.mark.parametrize('scale', [.95, 1.05])
def test_chassis_odom_fresh_map_pose_and_breadcrumb_use_same_normalized_yaw(prepared, scale):
    from math import sin, cos
    from nav_msgs.msg import Odometry
    node = prepared
    scenario(node)
    tf = node.tf_buffer.lookup_transform.return_value
    tf.transform.rotation.z = scale*sin(.1/2)
    tf.transform.rotation.w = scale*cos(.1/2)
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = 1.3001, .38
    msg.pose.pose.orientation.z = scale*sin(.1/2)
    msg.pose.pose.orientation.w = scale*cos(.1/2)
    node.odom_samples.clear()
    node.on_odom(msg)
    node._clear_breadcrumbs()
    assert node.latest_odom[4] == pytest.approx(.1, abs=1e-12)
    assert node._fresh_motion_pose()[2] == pytest.approx(.1, abs=1e-12)
    assert node._record_breadcrumb(transform=tf)
    assert node.breadcrumbs[-1].yaw == pytest.approx(.1, abs=1e-12)


def test_invalid_zero_quaternion_cannot_fabricate_reverse_odometry_heading(prepared):
    from nav_msgs.msg import Odometry
    node = prepared
    scenario(node)
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.orientation.x = msg.pose.pose.orientation.y = 0.
    msg.pose.pose.orientation.z = msg.pose.pose.orientation.w = 0.
    node.on_odom(msg)
    assert node.latest_odom is None
    assert not node.breadcrumbs
    assert not node.odom_samples
