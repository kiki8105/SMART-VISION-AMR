"""Independent adversarial checks of one published start-only egress edge.

These exercise production planner geometry/callbacks with captured classroom
inputs and fake clocks/publishers. No ROS executor or simulated robot runs.
"""

from copy import deepcopy

import pytest
from std_msgs.msg import Int32MultiArray

from smart_vision_amr.map_utils import cell_to_world, world_to_cell
from test_local_detour_planner import ready as harness_ready, segment_box_clearance
from test_occlusion_reserve_incident import FIXTURE, load_incident


def fresh_ready(node, *, projected=True, **kwargs):
    harness_ready(node, scan_frame=node.params['base_frame'], **kwargs)
    # This harness returns map<-base for TF calls. The independent raw scan
    # certificate correctly requires an explicit acquisition sensor frame.
    if projected:
        # Unless the test sends an explicit visibility callback below, its
        # fake projector has delivered this exact original scan event.
        stamp = node.latest_scan.header.stamp
        node.last_visibility_stamp_ns = stamp.sec*1_000_000_000+stamp.nanosec
        node._record_start_visibility(node.last_visibility_stamp_ns, node.clock['steady'])


@pytest.fixture
def certified_route(monkeypatch):
    node, record = load_incident(FIXTURE.parent/'occlusion_reserve_certified_start_20261104.json')
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    node.params['local_detour_occlusion_extension'] = record['recorded_extension_m']
    node.params['live_route_confirmation_scans'] = 1
    fresh_ready(node, x=record['pose_world'][0], y=record['pose_world'][1], yaw=record['pose_world'][2])
    node.last_observation_time = node.clock['steady']
    node.plan_to(node.last_goal)
    assert node.planned_world_points
    assert node.active_start_certificate
    assert node.plan_metrics['start_certificate_diagnostics']['allowed_edges'] == [[[33, 91], [32, 91]]]
    node.path_pub.publish.reset_mock()
    return node, record


def observe(node, cells, *, at=100.1, position=None, visibility=False):
    before = node.tf_buffer.lookup_transform.return_value.transform
    x, y = position or (before.translation.x, before.translation.y)
    fresh_ready(node, at=at, x=x, y=y, yaw=-1.438, projected=not visibility)
    width = node.map[0].info.width
    if not visibility:
        node.on_observed(Int32MultiArray(data=[y*width+x for x, y in cells]))
        return
    node.params['live_route_confirmation_scans'] = 2
    message = deepcopy(node.map[0])
    message.header.stamp = node.get_clock().now().to_msg()
    message.data = [0]*(message.info.width*message.info.height)
    for x, y in cells:
        message.data[y*width+x] = 100
    node.on_visibility(message)


def raw_route_intersection(node, observed):
    return set(node._remaining_path_cells()) & node._live_obstacle_blocked(observed)


def exemption(node):
    return node._validated_start_prefix_exemption(set(node._remaining_path_cells()))


def test_same_fresh_start_only_grid_overlap_does_not_cancel_selected_edge(certified_route):
    node, _ = certified_route
    start = (33, 91)
    assert raw_route_intersection(node, node.observed) == {start}
    assert exemption(node) == {start}
    assert not node._live_route_blockers(node.observed)
    assert node.active_start_certificate
    assert node.planned_world_points


@pytest.mark.parametrize('visibility', [False, True])
def test_real_observation_callbacks_keep_unchanged_certified_route(certified_route, visibility):
    node, _ = certified_route
    # A distant new hit makes the overlay callback take its changed-data path.
    observed = node.observed | {(75, 120)}
    assert raw_route_intersection(node, observed) == {(33, 91)}
    path_before = list(node.planned_world_points)
    observe(node, observed, visibility=visibility)
    assert node.planned_world_points == path_before
    assert not node.waiting_for_replan
    assert not node.path_pub.publish.called


def test_five_cm_egress_then_old_start_backbuffer_is_not_a_second_collision(certified_route):
    node, _ = certified_route
    destination = node.planned_world_points[1]
    assert world_to_cell(node.map[0].info, *destination) == (32, 91)
    fresh_ready(node, at=100.2, x=destination[0], y=destination[1], yaw=-1.438)
    node.last_observation_time = node.clock['steady']
    # The normal 27 cm back-buffer still contains the old start after this
    # first 5 cm edge. Its completed-prefix treatment must not be reentry.
    assert (33, 91) in node._remaining_path_cells()
    assert exemption(node) == {(33, 91)}
    assert not node.active_start_certificate
    assert node.completed_start_prefix
    assert not node._live_route_blockers(node.observed)
    observe(node, node.observed | {(75, 120)}, at=100.3, position=destination)
    assert node.planned_world_points and not node.waiting_for_replan


@pytest.mark.parametrize('visibility', [False, True])
@pytest.mark.parametrize('collision', ['body', 'first_sweep', 'later_route'])
def test_new_raw_collision_is_never_hidden_by_start_prefix(certified_route, collision, visibility):
    node, record = certified_route
    if collision == 'body':
        new_hit = (33, 91)
    elif collision == 'first_sweep':
        new_hit = (28, 88)
        info = node.map[0].info
        x, y = cell_to_world(info, new_hit)
        half = info.resolution/2
        square = x-half, y-half, x+half, y+half
        pose = record['pose_world'][:2]
        assert segment_box_clearance(pose, pose, square) > .22
        assert segment_box_clearance(node.planned_world_points[0], node.planned_world_points[1], square) < .22
    else:
        new_hit = world_to_cell(node.map[0].info, *node.planned_world_points[12])
        assert new_hit != (33, 91)
    observed = node.observed | {new_hit}
    # Isolate this safety/cancel boundary; alternative-route generation has
    # separate production-adapter tests and cannot hide a missing rejection.
    node.params['local_detour_enabled'] = False
    observe(node, observed, visibility=visibility)
    if visibility and not node.path_pub.publish.called:
        observe(node, observed, at=100.3, visibility=True)
    assert not node.active_start_certificate
    assert not node.completed_start_prefix
    assert node.waiting_for_replan
    assert node.path_pub.publish.called
    assert not node.path_pub.publish.call_args.args[0].poses


def test_completed_prefix_cannot_hide_new_body_collision(certified_route):
    node, _ = certified_route
    destination = node.planned_world_points[1]
    fresh_ready(node, at=100.2, x=destination[0], y=destination[1], yaw=-1.438)
    node.last_observation_time = node.clock['steady']
    assert exemption(node) == {(33, 91)}
    observe(node, node.observed | {(32, 91)}, at=100.3, position=destination)
    assert not node.active_start_certificate and not node.completed_start_prefix
    assert not node.path_pub.publish.call_args.args[0].poses


@pytest.mark.parametrize('completed', [False, True])
def test_fresh_raw_scan_collision_revokes_exemption_before_overlay_arrives(certified_route, completed):
    node, _ = certified_route
    if completed:
        destination = node.planned_world_points[1]
        fresh_ready(node, at=100.2, x=destination[0], y=destination[1], yaw=-1.438)
        node.last_observation_time = node.clock['steady']
        assert exemption(node) == {(33, 91)}
        assert node.completed_start_prefix
    # Current acquisition-time laser return is inside the actual circular
    # body. The obstacle overlay deliberately remains the previous frame.
    previous_observed = set(node.observed)
    node.latest_scan.ranges[0] = .10
    assert not exemption(node)
    assert not node.active_start_certificate and not node.completed_start_prefix
    assert node.observed == previous_observed
    assert node._live_route_blockers(node.observed)


def test_completed_prefix_does_not_authorize_reentry(certified_route):
    node, record = certified_route
    destination = node.planned_world_points[1]
    fresh_ready(node, at=100.2, x=destination[0], y=destination[1], yaw=-1.438)
    node.last_observation_time = node.clock['steady']
    assert exemption(node) == {(33, 91)}
    fresh_ready(node, at=100.3, x=record['pose_world'][0], y=record['pose_world'][1], yaw=-1.438)
    node.last_observation_time = node.clock['steady']
    assert not exemption(node)
    assert not node.active_start_certificate and not node.completed_start_prefix
    assert node._live_route_blockers(node.observed)


def test_actual_departure_through_an_unselected_exit_cannot_borrow_the_edge(certified_route):
    node, _ = certified_route
    assert node.active_start_certificate['edge'] == ((33, 91), (32, 91))
    other_exit = cell_to_world(node.map[0].info, (33, 90))
    fresh_ready(node, at=100.2, x=other_exit[0], y=other_exit[1], yaw=-1.438)
    node.last_observation_time = node.clock['steady']
    assert not exemption(node)
    assert not node.active_start_certificate and not node.completed_start_prefix


def test_amcl_jump_without_independent_odom_motion_does_not_complete_the_edge(certified_route):
    node, record = certified_route
    destination = node.planned_world_points[1]
    fresh_ready(node, at=100.2, x=destination[0], y=destination[1], yaw=-1.438,
                odom_x=record['pose_world'][0])
    node.last_observation_time = node.clock['steady']
    assert not exemption(node)
    assert not node.active_start_certificate and not node.completed_start_prefix


@pytest.mark.parametrize('change', ['manual', 'goal', 'path_stamp', 'path_geometry',
                                   'map', 'radius', 'clock_rewind', 'scan_stale', 'observed_stale'])
def test_active_certificate_is_not_reusable_under_changed_context(certified_route, change):
    node, _ = certified_route
    if change == 'manual':
        node.latest_motion_status['mode'] = 'manual'
    elif change == 'goal':
        node.last_goal = deepcopy(node.last_goal)
        node.last_goal.header.stamp.nanosec += 1
    elif change == 'path_stamp':
        node.current_path_stamp_ns += 1
    elif change == 'path_geometry':
        node.planned_world_points[1] = node.planned_world_points[2]
    elif change == 'map':
        node.raw_static_grid = deepcopy(node.raw_static_grid)
    elif change == 'radius':
        node.params['robot_radius'] = .23
    elif change == 'clock_rewind':
        fresh_ready(node, at=99., x=.007734634241251749, y=-.6735987818507905, yaw=-1.438)
        node.last_observation_time = node.clock['steady']
    elif change == 'scan_stale':
        node.latest_scan_at -= 2.
    elif change == 'observed_stale':
        node.last_observation_time -= 2.
    assert not exemption(node)
    assert not node.active_start_certificate and not node.completed_start_prefix


def test_confirmed_memory_start_stays_hard_even_with_an_existing_certificate(certified_route):
    node, record = certified_route
    node.remembered = {tuple(cell) for cell in record['later_memory_block_snapshot']['remembered_cells']}
    assert not exemption(node)
    assert not node.active_start_certificate and not node.completed_start_prefix


def test_fresh_recurring_geometric_proof_is_not_replaced_by_a_fixed_wait_timer(certified_route):
    node, record = certified_route
    # A ten-second jump without received odometry is a real source epoch gap,
    # not recurring fresh evidence. Preserve the original test's intent with
    # actual continuous stamped stationary callbacks throughout that interval.
    for index in range(1,201):
        fresh_ready(node, at=100.+index*.05, x=record['pose_world'][0], y=record['pose_world'][1], yaw=-1.438)
    node.last_observation_time = node.clock['steady']
    assert exemption(node) == {(33, 91)}
    assert node.active_start_certificate


def test_explicit_invalidation_revokes_selected_edge_without_regrant(certified_route):
    node, _ = certified_route
    node._invalidate_path('Operator stop')
    assert not node.active_start_certificate and not node.completed_start_prefix
    assert not exemption(node)
