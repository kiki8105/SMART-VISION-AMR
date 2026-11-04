"""Replay a captured false aisle closure, without starting ROS or Gazebo."""

from hashlib import sha256
import json
from pathlib import Path
from unittest.mock import Mock, patch

from nav_msgs.msg import OccupancyGrid
import pytest
import yaml

from smart_vision_amr.clearance_cost import clearance_penalties
from smart_vision_amr.local_detour import LocalDetourResult
from smart_vision_amr.map_utils import cell_to_world, inflate_cells
from smart_vision_amr.path_geometry import collision_free, smooth_grid_path
from smart_vision_amr.web_mission_server import load_saved_map
from body_geometry_oracle import continuous_raw_body_gap, full_cell_body_mask
from test_local_detour_planner import path_box_clearance, ready
from test_planner_failure_memory import FailureHarness
from test_planner_recovery import make_goal


PACKAGE = Path(__file__).resolve().parents[1]
FIXTURE = Path(__file__).parent / 'fixtures/occlusion_reserve_inflation022_20261104.json'


def load_incident(path=FIXTURE):
    recorded = json.loads(path.read_text())
    map_path = PACKAGE / 'maps' / recorded['map']['yaml']
    assert sha256(map_path.with_suffix('.pgm').read_bytes()).hexdigest() == recorded['map']['image_sha256']
    metadata, pixels = load_saved_map(map_path)
    node = FailureHarness({'steady': 100., 'ros': 100.})
    profile = yaml.safe_load((PACKAGE / 'config/classroom_hardware_nodes.yaml').read_text())[
        '/astar_planner']['ros__parameters']
    node.params.update(profile)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height = metadata['width'], metadata['height']
    msg.info.resolution = metadata['resolution']
    msg.info.origin.position.x, msg.info.origin.position.y = recorded['map']['origin'][:2]
    msg.info.origin.orientation.w = 1.
    msg.data = pixels.flatten().tolist()
    node.on_map(msg)
    node.last_goal = make_goal(stamp=100)
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = recorded['reference_world'][-1]
    node.local_reference_points = [tuple(point) for point in recorded['reference_world']]
    node.local_reference_key = node._local_route_key()
    node.observed = {tuple(cell) for cell in recorded['observed_cells']}
    node.remembered = {tuple(cell) for cell in recorded['remembered_cells']}
    assert node.params['robot_radius'] == recorded['robot_radius_m'] == .22
    assert node.params['obstacle_inflation_cells'] == recorded['astar_inflation_cells'] == 5
    return node, recorded


@pytest.fixture
def incident():
    return load_incident()


def replay(node, recorded, extension):
    """Use production local/global reserve and the unchanged body masks."""
    node.params['local_detour_occlusion_extension'] = extension
    start, goal = tuple(recorded['start_cell']), tuple(recorded['goal_cell'])
    info, grid = node.map[0].info, node.map[1]
    cfg = node._local_configuration()
    local_grid, inflation = node._local_footprint_grid(cfg)
    assert inflation == 5 and local_grid.rows == grid.rows
    reference = [((x-info.origin.position.x)/info.resolution,
                  (y-info.origin.position.y)/info.resolution)
                 for x, y in node.local_reference_points]
    local = node._local_occlusion_reserve(node.observed, reference, start, inflation, cfg)
    global_hits = node._global_occlusion_reserve(start, node.observed)
    assert local == global_hits, 'local repair and global fallback must read the same depth setting'
    assert node.observed <= global_hits
    assert not node.remembered, 'the recorded first closure preceded memory confirmation'
    blocked = node._live_obstacle_blocked(global_hits)
    costs = clearance_penalties(node.raw_static_grid, global_hits, info.resolution,
                               node.params['preferred_clearance'], node.params['clearance_penalty_gain'])
    route = node.planner.plan(grid, start, goal, blocked, cell_penalty=costs)
    return route, global_hits, blocked, costs


@pytest.mark.parametrize('extension,expected_cells', [
    (0., 75), (.05, 76), (.10, 76), (.15, 76), (.20, 76), (.25, 0),
])
def test_recorded_extension_sensitivity_preserves_all_measured_hits(incident, extension, expected_cells):
    node, recorded = incident
    original = set(node.observed)
    route, _, blocked, _ = replay(node, recorded, extension)
    assert len(route) == expected_cells
    assert node.observed == original
    assert not set(route) & blocked
    assert all(not node.map[1].is_static_obstacle(cell) for cell in route)


def test_current_profile_has_continuous_22cm_body_clearance_to_actual_hidden_box_rear(incident):
    node, recorded = incident
    assert node.params['local_detour_occlusion_extension'] == recorded['current_extension_m'] == .20
    route, reserved, blocked, costs = replay(node, recorded, .20)
    assert reserved > node.observed, 'retain unseen depth rather than disabling the reserve'
    grid = node.map[1]
    points, _ = smooth_grid_path(
        route, lambda cell: grid.is_static_obstacle(cell) or cell in blocked,
        spacing=node.params['path_resample_spacing']/.05,
        shortcut_distance=node.params['path_smoothing_shortcut_distance']/.05,
        iterations=node.params['path_smoothing_iterations'], cell_penalty=costs)
    assert collision_free(points, lambda cell: grid.is_static_obstacle(cell) or cell in blocked)
    world = [(x*.05-1.65, y*.05-5.25) for x, y in points]
    # Independent exact segment-vs-physical rectangle distance, not the same
    # inflated grid used by planning. This includes the box's unobserved rear.
    gap = path_box_clearance(world, recorded['actor']['bounds']) - .22
    assert gap == pytest.approx(.13875015703854907, abs=1e-6)
    assert gap > .13


def test_old_extension_single_measured_face_pixel_creates_false_connection_cut(incident):
    node, recorded = incident
    node.observed = {tuple(recorded['critical_hit_cell'])}
    route, reserved, _, _ = replay(node, recorded, .25)
    assert not route
    assert reserved == {tuple(cell) for cell in recorded['old_critical_reserve_cells']}
    old_blocked = node._live_obstacle_blocked(reserved)
    assert {tuple(cell) for cell in recorded['reserve_only_blocked_connection_cells']} <= old_blocked
    route, _, blocked, _ = replay(node, recorded, .20)
    assert route and not set(route) & blocked


def test_measured_hit_stays_hard_when_it_replaces_a_virtual_reserved_cell(incident):
    node, recorded = incident
    node.observed.add((43, 87))
    route, _, blocked, _ = replay(node, recorded, .20)
    # A new measured face is never softened merely to keep the historical
    # bypass feasible. Fresh physical observations still close it when needed.
    assert (43, 87) in blocked
    assert not route


@pytest.fixture(params=[
    'occlusion_reserve_inflation022_20261104.json',
    'occlusion_reserve_second_first_20261104.json',
    'occlusion_reserve_second_memory_20261104.json',
])
def frozen_scene(request, monkeypatch):
    node, recorded = load_incident(FIXTURE.parent/request.param)
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic', lambda: node.clock['steady'])
    node.params['local_detour_occlusion_extension'] = recorded.get('recorded_extension_m', .25)
    x, y = cell_to_world(node.map[0].info, recorded['start_cell'])
    ready(node, x=x, y=y)
    node.last_observation_time = node.clock['steady']
    node.planned_world_points = list(node.local_reference_points)
    return node, recorded


def assert_physical_path(node, recorded):
    info, grid = node.map[0].info, node.map[1]
    actual_blocked = full_cell_body_mask(node.observed | node.remembered, info, .22)
    points = [((x-info.origin.position.x)/info.resolution,
               (y-info.origin.position.y)/info.resolution) for x, y in node.planned_world_points]
    assert collision_free(points, lambda c:grid.is_static_obstacle(c) or c in actual_blocked)
    assert path_box_clearance(node.planned_world_points, recorded['actor']['bounds'])-.22 > .02
    assert continuous_raw_body_gap(node, node.planned_world_points) >= -1e-10


def test_frozen_scenes_retry_local_with_soft_virtual_depth_before_global_and_one_attempt(frozen_scene):
    node, recorded = frozen_scene
    before_observed, before_memory, before_map = set(node.observed), set(node.remembered), list(node.map[0].data)
    goal, reference = node.last_goal, list(node.local_reference_points)
    node.plan_to = Mock(wraps=node.plan_to)
    assert node._try_local_detour(node.observed | node.remembered)
    assert node.plan_metrics['occlusion_fallback'] is True
    assert node.plan_metrics['occlusion_constraint_mode'] == 'SOFT_VIRTUAL'
    assert node.plan_metrics['local_detour_state'] == 'ACTIVE'
    assert node.local_detour_attempts == node.local_attempt_id == 1
    assert node.plan_metrics['expansions'] <= node.params['local_detour_max_expansions']
    assert node.plan_metrics['candidates_examined'] <= node.params['local_detour_max_candidates']
    assert node.plan_metrics['search_duration_ms'] < 1000*node.params['local_detour_max_planning_time'] + 5
    node.plan_to.assert_not_called()
    assert node.last_goal is goal and node.local_reference_points == reference
    assert node.observed == before_observed and node.remembered == before_memory
    assert list(node.map[0].data) == before_map
    assert_physical_path(node, recorded)


def test_frozen_scenes_global_fallback_keeps_hard_observations_memory_and_box_clearance(frozen_scene):
    node, recorded = frozen_scene
    goal = node.last_goal
    observed, remembered = set(node.observed), set(node.remembered)
    node.plan_to(goal)
    assert node.planned_world_points and not node.waiting_for_replan
    assert node.plan_metrics['occlusion_fallback'] is True
    assert node.plan_metrics['occlusion_constraint_mode'] == 'SOFT_VIRTUAL'
    assert node.observed == observed and node.remembered == remembered
    assert node.last_goal is goal
    assert_physical_path(node, recorded)


@pytest.mark.parametrize('layer', ['observed', 'remembered'])
def test_real_measured_wall_cannot_be_softened_to_recover_a_virtual_reserve(frozen_scene, layer):
    node, _ = frozen_scene
    # Close every possible left-to-right crossing in the map with real cells.
    getattr(node, layer).update((x, 85) for x in range(node.map[0].info.width))
    assert not node._try_local_detour(node.observed | node.remembered)
    node.plan_to(node.last_goal)
    assert not node.planned_world_points
    assert not node.plan_metrics['occlusion_fallback']


@pytest.mark.parametrize('fault', ['scan', 'odom', 'tf', 'manual', 'reference'])
def test_global_virtual_fallback_requires_fresh_independent_inputs_and_matching_reference(frozen_scene, fault):
    node, recorded = frozen_scene
    if fault == 'scan': node.latest_scan_at -= 2.
    if fault == 'odom': node.latest_odom = None
    if fault == 'tf': node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 0
    if fault == 'manual': node.latest_motion_status['mode'] = 'manual'
    if fault == 'reference': node.local_reference_key = ('unrelated',)
    assert not node._occlusion_fallback_ready(tuple(recorded['start_cell']))
    node.plan_to(node.last_goal)
    if fault != 'reference':
        assert not node.planned_world_points
    assert not node.plan_metrics['occlusion_fallback']


def test_local_conservative_and_soft_search_share_one_expansion_candidate_attempt_budget(frozen_scene):
    node, _ = frozen_scene
    calls = []
    def exhausted(*args, **kwargs):
        calls.append(kwargs)
        return LocalDetourResult(None, [], None, kwargs['max_expansions'],
                                 'expansion_budget', 0., candidates_examined=kwargs['max_candidates'])
    with patch('smart_vision_amr.astar_planner.plan_local_detour', side_effect=exhausted):
        assert not node._try_local_detour(node.observed | node.remembered)
    assert len(calls) == 2
    assert sum(call['max_expansions'] for call in calls) == node.params['local_detour_max_expansions']
    assert sum(call['max_candidates'] for call in calls) == node.params['local_detour_max_candidates']
    assert node.plan_metrics['expansions'] == node.params['local_detour_max_expansions']
    assert node.plan_metrics['candidates_examined'] == node.params['local_detour_max_candidates']
    assert node.local_detour_attempts == node.local_attempt_id == 1


@pytest.mark.parametrize('budget', ['time', 'expansions', 'candidates'])
def test_no_soft_search_after_entire_local_event_budget_is_spent(frozen_scene, budget):
    node, _ = frozen_scene
    clock = [100.]
    if budget == 'expansions': node.params['local_detour_max_expansions'] = 1
    if budget == 'candidates': node.params['local_detour_max_candidates'] = 1
    def exhausted(*args, **kwargs):
        if budget == 'time': clock[0] += node.params['local_detour_max_planning_time']+.001
        return LocalDetourResult(None, [], None, kwargs['max_expansions'],
                                 'time_budget' if budget == 'time' else 'expansion_budget', 0.,
                                 candidates_examined=kwargs['max_candidates'])
    with patch('smart_vision_amr.astar_planner.time.perf_counter', side_effect=lambda: clock[0]), \
         patch('smart_vision_amr.astar_planner.plan_local_detour', side_effect=exhausted) as search:
        assert not node._try_local_detour(node.observed | node.remembered)
    assert search.call_count == 1
    assert node.local_detour_attempts == 1
    assert not node.plan_metrics['occlusion_fallback_attempted']


def test_fallback_fresh_tf_and_cost_preparation_cannot_extend_the_shared_deadline(frozen_scene):
    node, _ = frozen_scene
    clock = [100.]
    def slow_fresh_check(start):
        clock[0] += node.params['local_detour_max_planning_time']+.001
        return True
    result = LocalDetourResult(None, [], None, 0, 'no_local_route', 0.)
    with patch('smart_vision_amr.astar_planner.time.perf_counter', side_effect=lambda: clock[0]), \
         patch.object(node, '_occlusion_fallback_ready', side_effect=slow_fresh_check), \
         patch('smart_vision_amr.astar_planner.plan_local_detour', return_value=result) as search:
        assert not node._try_local_detour(node.observed | node.remembered)
    assert search.call_count == 1
    assert not node.plan_metrics['occlusion_fallback_attempted']
