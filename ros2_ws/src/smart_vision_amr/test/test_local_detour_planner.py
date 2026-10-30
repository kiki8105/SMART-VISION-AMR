"""Real planner callbacks and ROS messages with fake outputs, no DDS context.

The harness does not construct a ROS Node or spin an executor. Every publisher,
clock and transform is fake; geometry, failure memory and route repair are real.
"""
from copy import deepcopy
import json
from math import cos, hypot, sin
from unittest.mock import Mock

import pytest
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformException

from smart_vision_amr.path_geometry import route_cells
from test_planner_failure_memory import FailureHarness, refresh
from test_planner_recovery import make_goal, status


def ready(node, at=100., *, x=.525, y=1.275, yaw=0., odom_x=None, motion=None):
    node.clock.update(steady=float(at), ros=float(at))
    refresh(node, x=x, y=y, yaw=yaw, odom_x=odom_x)
    payload = {'sent_at_ns': node.get_clock().now().nanoseconds, 'mode': 'auto',
               'available': True, 'autonomous_allowed': True,
               'estop_latched': False, 'reset_pending': False}
    payload.update(motion or {})
    node.on_motion_status(String(data=json.dumps(payload)))


@pytest.fixture
def planner_factory(monkeypatch):
    clock = {'steady': 100., 'ros': 100.}
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: clock['steady'])

    def build(*, occupied=(), unknown=(), clearance_penalty_gain=1.5, route_yaw=0.):
        node = FailureHarness(clock)
        node.params.update(local_detour_enabled=True, local_detour_max_planning_time=.1,
                           obstacle_inflation_cells=1, clearance_penalty_gain=clearance_penalty_gain)
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width, message.info.height = 80, 60
        message.info.resolution = .05
        message.info.origin.orientation.z = sin(route_yaw/2)
        message.info.origin.orientation.w = cos(route_yaw/2)
        message.data = [0]*(80*60)
        for x, y in occupied:
            message.data[y*80+x] = 100
        for x, y in unknown:
            message.data[y*80+x] = -1
        node.on_map(message)
        def world(x, y):
            return (cos(route_yaw)*x-sin(route_yaw)*y,
                    sin(route_yaw)*x+cos(route_yaw)*y)
        start_x, start_y = world(.525, 1.275)
        ready(node, x=start_x, y=start_y, yaw=route_yaw)
        goal = make_goal(stamp=100)
        goal.pose.position.x, goal.pose.position.y = world(3.275, 1.275)
        goal.pose.orientation.z, goal.pose.orientation.w = sin((route_yaw+.75)/2), cos((route_yaw+.75)/2)
        node.on_goal(goal)
        assert node.planned_world_points
        node.path_pub.publish.reset_mock()
        node.status_pub.publish.reset_mock()
        return node

    return build


def hits(node, cells):
    return Int32MultiArray(data=[y*node.map[0].info.width+x for x, y in cells])


def paths(node):
    return [call.args[0] for call in node.path_pub.publish.call_args_list]


def packets(node):
    return [json.loads(call.args[0].data) for call in node.status_pub.publish.call_args_list]


def segment_box_clearance(a, b, box):
    """Independent exact line/rectangle distance for the circular footprint.

    This checks continuous published segments against physical ground truth,
    rather than consulting the planner's inflated cells or waypoint centers.
    """
    left, bottom, right, top = box
    dx, dy = b[0]-a[0], b[1]-a[1]
    entry, leave = 0., 1.
    for value, movement, low, high in ((a[0], dx, left, right),
                                       (a[1], dy, bottom, top)):
        if movement == 0:
            if not low <= value <= high:
                break
        else:
            first, last = sorted(((low-value)/movement, (high-value)/movement))
            entry, leave = max(entry, first), min(leave, last)
            if entry > leave:
                break
    else:
        return 0.

    def point_box(point):
        return hypot(max(left-point[0], point[0]-right, 0.),
                     max(bottom-point[1], point[1]-top, 0.))

    distances = [point_box(a), point_box(b)]
    square = dx*dx+dy*dy
    for x, y in ((left, bottom), (left, top), (right, bottom), (right, top)):
        position = 0. if square == 0 else max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/square))
        distances.append(hypot(x-a[0]-position*dx, y-a[1]-position*dy))
    return min(distances)


def path_box_clearance(points, box):
    return min(segment_box_clearance(a, b, box) for a, b in zip(points, points[1:]))


def patch(node, cells={(24, 25)}, *, at=100.1, layer='observed'):
    ready(node, at)
    getattr(node, 'on_'+layer)(hits(node, cells))


def test_local_patch_is_immediate_nonempty_and_keeps_goal_yaw_and_exact_suffix(planner_factory):
    node = planner_factory()
    goal, original = deepcopy(node.last_goal), list(node.planned_world_points)
    source_map = list(node.map[0].data)
    global_count = node.global_plan_count
    node.plan_to = Mock(wraps=node.plan_to)
    patch(node)
    assert paths(node) and all(path.poses for path in paths(node)), 'do not cancel with an empty global path first'
    assert node.global_plan_count == global_count
    node.plan_to.assert_not_called()
    assert node.last_goal == goal
    assert list(node.map[0].data) == source_map
    output = paths(node)[-1]
    endpoint = output.poses[-1].pose.position
    assert (endpoint.x, endpoint.y) == original[-1]
    assert (endpoint.x, endpoint.y) == pytest.approx((goal.pose.position.x, goal.pose.position.y))
    assert output.poses[-1].pose.orientation == goal.pose.orientation
    rejoin = node.plan_metrics['rejoin_point']
    rejoin_index = original.index((rejoin['x'], rejoin['y']))
    suffix = original[rejoin_index:]
    assert node.planned_world_points[-len(suffix):] == suffix
    assert rejoin['x'] > 1.225
    assert any(value.get('plan_kind') == 'LOCAL_DETOUR' for value in packets(node))
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'


@pytest.mark.parametrize('kind', ['static', 'unknown'])
def test_adapter_inflates_real_footprint_for_map_and_live_hits_even_with_small_global_inflation(planner_factory, kind):
    static = {(x, 33) for x in range(17, 36)}
    node = planner_factory(clearance_penalty_gain=0.,
                           **{'occupied' if kind == 'static' else 'unknown': static})
    patch(node)
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    required = node.params['robot_radius']+node.params['local_detour_clearance_margin']
    for px, py in node.planned_world_points:
        for x, y in static | {(24, 25)}:
            dx, dy = max(x*.05-px, px-(x+1)*.05, 0.), max(y*.05-py, py-(y+1)*.05, 0.)
            assert hypot(dx, dy) >= required-1e-8
    assert min(y for _, y in node.planned_world_points) < 1.1, 'the upper wall forces the lower bypass'


def test_complete_corridor_blockage_falls_back_and_never_publishes_partial_local_path(planner_factory):
    node = planner_factory()
    patch(node, {(24, y) for y in range(60)})
    assert paths(node) and not paths(node)[-1].poses
    assert node.waiting_for_replan
    assert node.plan_metrics['local_detour_state'] == 'FAILED'
    assert node.local_detour_attempts == 1
    assert node.plan_metrics['expansions'] <= node.params['local_detour_max_expansions']


def test_new_obstacle_on_active_detour_invalidates_immediately_instead_of_chaining_patches(planner_factory):
    node = planner_factory()
    patch(node)
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    new_cell = next((int(x/.05), int(y/.05)) for x, y in node.planned_world_points
                    if .85 < x < 1.35 and abs(y-1.275) > .15)
    attempt = node.local_attempt_id
    node.path_pub.publish.reset_mock()
    patch(node, {(24, 25), new_cell}, at=100.2)
    assert not paths(node)[-1].poses
    assert node.waiting_for_replan
    assert node.local_attempt_id == attempt
    assert node.plan_metrics['detour_reason'] == 'active_detour_obstructed'


def test_partial_box_front_face_reserves_a_complete_bypass_before_rejoining(planner_factory):
    # Translated SITL geometry: a .45 x .50 m box is initially visible only as
    # its near LiDAR face. The hidden depth is test ground truth, not sensor
    # input. A temporary bounded occlusion reserve must prevent an early turn
    # through the unseen box body; it must not fabricate observations/map cells.
    node = planner_factory()
    front_face = {(24, y) for y in range(20, 31)}
    whole_box = {(x, y) for x in range(24, 33) for y in range(20, 31)}
    actual_box = (1.20, 1.025, 1.65, 1.525)
    original_map, goal = list(node.map[0].data), deepcopy(node.last_goal)
    global_count = node.global_plan_count
    patch(node, front_face)
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    required = node.params['robot_radius']+node.params['local_detour_clearance_margin']
    assert path_box_clearance(node.planned_world_points, actual_box) >= required-1e-8
    assert node.plan_metrics['rejoin_point']['x'] > actual_box[2]+required
    assert node.observed == front_face, 'occlusion reserve is not a measured obstacle'
    assert not node.remembered, 'occlusion reserve must not poison obstacle memory'
    assert list(node.map[0].data) == original_map
    assert node.last_goal == goal
    assert node.global_plan_count == global_count
    assert all(path.poses for path in paths(node))
    attempt, replacement = node.local_attempt_id, list(node.planned_world_points)
    node.path_pub.publish.reset_mock()

    # Moving alongside the obstacle reveals its side/far face. This supported
    # box stays clear of the first prefix, so there is no cancel/global-retry
    # loop and no replenishing or consuming the retained-goal attempt budget.
    ready(node, 100.2, x=.975, y=.725)
    node.on_observed(hits(node, whole_box))
    assert not paths(node)
    assert node.planned_world_points == replacement
    assert node.local_attempt_id == attempt and node.local_detour_attempts == 1
    assert node.global_plan_count == global_count
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    assert not node.waiting_for_replan


@pytest.mark.parametrize('observation_margin,retained', [(0., False), (.05, True)])
def test_one_cell_face_refinement_uses_reserved_observation_margin_without_chaining(
        planner_factory, observation_margin, retained):
    node = planner_factory(clearance_penalty_gain=0.)
    # The six-cell physical profile exposes conservative diagonal contacts.
    # Keeping a zero soft-cost gain proves this is a hard planning reserve,
    # not a lucky route preference that disappears with a different cost field.
    node.params.update(obstacle_inflation_cells=6,
                       local_detour_observation_margin=observation_margin)
    front = {(24, y) for y in range(20, 31)}
    patch(node, front)
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    original = list(node.planned_world_points)
    attempt, global_count = node.local_attempt_id, node.global_plan_count
    node.path_pub.publish.reset_mock()
    # Translation of actual07's near face extending by one newly visible row.
    # The new hit remains ahead of the robot, so no behind-progress filtering
    # or suppression of real collision checks may explain retaining the patch.
    patch(node, front | {(24, 19)}, at=100.2)
    assert node.observed == front | {(24, 19)}
    assert node.local_attempt_id == attempt and node.local_detour_attempts == 1
    assert node.global_plan_count == global_count
    if not retained:
        assert paths(node) and not paths(node)[-1].poses
        assert node.plan_metrics['detour_reason'] == 'active_detour_obstructed'
        return
    assert not paths(node)
    assert node.planned_world_points == original
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    assert not node.waiting_for_replan
    assert path_box_clearance(original, (1.20, .95, 1.25, 1.0)) >= .24-1e-8
    # The reserve only plans farther from uncertain returns. It must not make
    # a larger newly observed obstruction legal, or chain an active repair.
    new_hit = next((int(x/.05), int(y/.05)) for x, y in original
                   if 1.0 < x < 1.6 and y < .9)
    patch(node, front | {(24, 19), new_hit}, at=100.3)
    assert paths(node) and not paths(node)[-1].poses
    assert node.plan_metrics['detour_reason'] == 'active_detour_obstructed'
    assert node.local_attempt_id == attempt and node.global_plan_count == global_count


@pytest.mark.parametrize('route_yaw', [1.5707963267948966, 3.141592653589793, -.7])
def test_partial_box_occlusion_reserve_follows_reference_heading_on_rotated_maps(planner_factory, route_yaw):
    node = planner_factory(route_yaw=route_yaw)
    ready(node, 100.1, x=cos(route_yaw)*.525-sin(route_yaw)*1.275,
          y=sin(route_yaw)*.525+cos(route_yaw)*1.275, yaw=route_yaw)
    node.on_observed(hits(node, {(24, y) for y in range(20, 31)}))
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    # Undo only the test map rotation for the independent world-footprint proof.
    local_points = [(cos(route_yaw)*x+sin(route_yaw)*y,
                     -sin(route_yaw)*x+cos(route_yaw)*y) for x, y in node.planned_world_points]
    required = node.params['robot_radius']+node.params['local_detour_clearance_margin']
    assert path_box_clearance(local_points, (1.20, 1.025, 1.65, 1.525)) >= required-1e-8
    assert len(node.observed) == 11 and not node.remembered
    assert node.global_plan_count == 1 and node.local_detour_attempts == 1


def test_new_live_obstacle_rebuilds_clearance_preference_after_the_global_plan(planner_factory):
    front_face = {(24, y) for y in range(20, 31)}
    actual_box = (1.20, 1.025, 1.65, 1.525)
    clearances = []
    for gain in (0., 1.5):
        node = planner_factory(clearance_penalty_gain=gain)
        # This area was free when the global route's cached costs were made.
        # Reusing those costs would give the same edge-hugging local route.
        assert node.local_cell_penalty.get((24, 17), 0.) == 0.
        patch(node, front_face)
        assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
        assert node.global_plan_count == 1
        clearances.append(path_box_clearance(node.planned_world_points, actual_box))
    assert clearances[0] >= .24-1e-8, 'soft preference never replaces hard footprint clearance'
    assert clearances[1] > clearances[0]+.05, 'new live geometry must affect local soft costs'


def test_partial_face_with_no_footprint_clear_corridor_falls_back_without_a_partial_patch(planner_factory):
    # The initial route fits between two mapped walls. Once the box is seen,
    # neither side has enough room for the circular footprint and reserve.
    walls = {(x, y) for x in range(80) for y in (16, 34)}
    node = planner_factory(occupied=walls)
    patch(node, {(24, y) for y in range(20, 31)})
    assert paths(node) and all(not path.poses for path in paths(node))
    assert node.waiting_for_replan and node.plan_metrics['local_detour_state'] == 'FAILED'
    assert node.local_detour_attempts == 1
    assert node.params['obstacle_retry_limit'] == 3


def test_box_revealed_larger_than_bounded_reserve_still_invalidates_active_patch(planner_factory):
    node = planner_factory()
    patch(node, {(24, y) for y in range(20, 31)})
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    attempt, global_count = node.local_attempt_id, node.global_plan_count
    node.path_pub.publish.reset_mock()
    # A much longer actual obstacle exceeds the supported .50 m reserve.
    # Newly measured cells on the route must still revoke it immediately;
    # the reserve cannot excuse a collision or chain another active prefix.
    ready(node, 100.2, x=.975, y=.525)
    node.on_observed(hits(node, {(x, y) for x in range(24, 45) for y in range(20, 31)}))
    assert paths(node) and not paths(node)[-1].poses
    assert node.waiting_for_replan
    assert node.plan_metrics['detour_reason'] == 'active_detour_obstructed'
    assert node.local_attempt_id == attempt and node.local_detour_attempts == 1
    assert node.global_plan_count == global_count


@pytest.mark.parametrize('fault', ['manual', 'estop', 'hold', 'mux_stale', 'mux_unavailable',
                                   'health_stale', 'health_denied', 'scan_stale', 'odom_stale', 'no_tf'])
def test_unsafe_or_stale_evidence_never_produces_a_local_patch(planner_factory, fault):
    node = planner_factory()
    ready(node, 100.1)
    if fault in {'manual', 'hold'}: node.latest_motion_status['mode'] = fault
    if fault == 'estop': node.latest_motion_status['estop_latched'] = True
    if fault == 'mux_stale': node.latest_motion_status_at = 98.
    if fault == 'mux_unavailable': node.latest_motion_status['available'] = False
    if fault == 'health_stale': node.latest_health_at = 98.
    if fault == 'health_denied': node.localization_health['autonomous_allowed'] = False
    if fault == 'scan_stale': node.latest_scan_at = 98.
    if fault == 'odom_stale': node.latest_odom = (98.,)+node.latest_odom[1:]
    if fault == 'no_tf': node.tf_buffer.lookup_transform.side_effect = TransformException('no map pose')
    node.on_observed(hits(node, {(24, 25)}))
    assert not paths(node)[-1].poses
    assert node.local_attempt_id == 0
    assert not any(value.get('plan_kind') == 'LOCAL_DETOUR' for value in packets(node))


def test_explicit_manual_cancel_clears_reference_and_new_scans_cannot_restart_mission(planner_factory):
    node = planner_factory()
    patch(node)
    node.on_navigation_status(status(node, 'CANCELED', manual_stop=True,
                                    stop_stamp={'sec': 101, 'nanosec': 0}))
    assert node.navigation_suspended
    assert not node.local_reference_points
    assert node.local_rejoin_anchor is None
    node.path_pub.publish.reset_mock()
    patch(node, {(24, 25), (30, 25)}, at=102.)
    assert not paths(node)
    assert node.navigation_suspended


def test_new_goal_and_replaced_map_discard_the_previous_patch_and_reference(planner_factory):
    node = planner_factory()
    patch(node)
    old_key, old_attempt = node.local_reference_key, node.local_attempt_id
    ready(node, 101.)
    goal = deepcopy(node.last_goal)
    goal.header.stamp.sec = 101
    goal.pose.position.x = 3.525
    node.on_goal(goal)
    assert node.local_reference_key != old_key
    assert node.local_rejoin_anchor is None
    assert node.local_detour_attempts == 0
    assert node.last_goal is goal
    assert node.local_attempt_id == old_attempt
    changed = deepcopy(node.map[0])
    changed.info.origin.position.x = .05
    node.on_map(changed)
    assert not node.local_reference_points
    assert node.local_rejoin_anchor is None


def test_global_replanning_alone_cannot_reset_the_retained_goal_local_attempt_budget(planner_factory):
    node = planner_factory()
    node.local_detour_attempts = node.params['local_detour_max_attempts']
    ready(node, 101.)
    node.plan_to(node.last_goal)
    assert node.local_detour_attempts == node.params['local_detour_max_attempts']
    node.path_pub.publish.reset_mock()
    patch(node, at=101.1)
    assert node.plan_metrics['detour_reason'] == 'attempt_budget'
    assert not paths(node)[-1].poses


def test_failed_approach_memory_is_retained_and_not_crossed_by_local_prefix(planner_factory):
    node = planner_factory()
    scopes = node._memory_scopes()
    node.failure_memory.record_failure(x=1.025, y=1.275, approach_heading=0.,
        reason='execution_no_progress', now=100., map_scope=scopes[0], goal_scope=scopes[1], failure_id='f'*32)
    patch(node)
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    assert node.failure_memory.snapshot(now=100.1)['count'] == 1
    assert not any(node.failure_memory.segment_blocked(a, b, now=100.1,
        map_scope=scopes[0], goal_scope=scopes[1])
        for a, b in zip(node.planned_world_points, node.planned_world_points[1:]))


def test_attempt_budget_resets_only_after_forward_rejoin_verified_by_independent_odom(planner_factory):
    node = planner_factory()
    patch(node)
    rejoin_x = node.plan_metrics['rejoin_point']['x']+.15
    ready(node, 101., x=rejoin_x, odom_x=.525)
    node.on_navigation_status(status(node, 'MOVING'))
    node._update_local_rejoin()
    assert node.local_detour_attempts == 1, 'an AMCL correction alone cannot replenish attempts'
    ready(node, 101.1, x=rejoin_x, odom_x=rejoin_x)
    node.on_navigation_status(status(node, 'MOVING'))
    node._update_local_rejoin()
    assert node.local_detour_attempts == 0
    assert node.plan_metrics['local_detour_state'] == 'REJOINED'


@pytest.mark.parametrize('parameter,value', [
    ('local_detour_max_planning_time', 0.),
    ('local_detour_max_planning_time', float('nan')),
    ('local_detour_max_planning_time', .2),
    ('local_detour_max_expansions', 0),
    ('local_detour_clearance_margin', -.01),
    ('local_detour_clearance_margin', float('inf')),
    ('local_detour_max_candidates', 0),
    ('local_detour_occlusion_extension', -.01),
    ('local_detour_occlusion_extension', float('inf')),
    ('local_detour_rejoin_buffer', -.01),
    ('local_detour_rejoin_buffer', 1.01),
    ('local_detour_observation_margin', -.01),
    ('local_detour_observation_margin', float('nan')),
])
def test_bad_runtime_configuration_disables_only_local_repair_and_preserves_global_fallback(
        planner_factory, parameter, value):
    node = planner_factory()
    goal = deepcopy(node.last_goal)
    global_count = node.global_plan_count
    node.params[parameter] = value
    node.local_grid_cache = None
    patch(node)
    assert paths(node) and not paths(node)[-1].poses
    assert node.local_attempt_id == 0
    assert node.plan_metrics['detour_reason'] == 'invalid_configuration'
    assert parameter in node.plan_metrics['detour_configuration_error']
    assert not any(item.get('plan_kind') == 'LOCAL_DETOUR' for item in packets(node))
    # The global planner's optional cache warmup must also tolerate the bad
    # setting. A local feature typo must not crash or disable ordinary routing.
    ready(node, 102.2)
    node.plan_to(node.last_goal)
    assert paths(node)[-1].poses
    assert node.global_plan_count == global_count+1
    assert node.plan_metrics['plan_kind'] == 'GLOBAL'
    assert node.plan_metrics['detour_reason'] == 'invalid_configuration'
    assert node.local_grid_cache is None
    assert node.last_goal == goal
