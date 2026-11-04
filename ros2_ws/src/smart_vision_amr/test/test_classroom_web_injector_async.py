import threading
import time
import math
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))

import inject_classroom_web_obstacle as injector

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from inject_classroom_web_obstacle import (
    CandidateSearch, candidate_still_ahead, departed_home, first_home_truth,
    matching_home_path, route_readiness,
    current_observed_bypass_route, newly_observed_route_clear,
    select_feasible_web_obstacle,
)
from smart_vision_amr.map_utils import cell_to_world, world_to_cell


def test_exact_square_geometry_rejects_prior_off_path_physical_candidate():
    # The failed staged actor put the box 43.6 cm from the centerline. An
    # enclosing box-circle and stale 24 cm body radius incorrectly counted it
    # as a blocker; the selected physical 20 cm body has 11 cm clear space.
    path = [(.658, -.3), (.658, -1.8)]
    center = (1.0936, -1.1613)
    assert math.isclose(injector.path_clearance_from_box(path, center, .20), .1106)
    candidate = {'x': center[0], 'y': center[1], 'path_tangent': [0., -1.]}
    assert not candidate_still_ahead(candidate, (.658, -.5), path, .20)


def test_box_must_overlap_selected_body_route_by_more_than_one_centimeter():
    path = [(-1., 0.), (1., 0.)]
    candidate = {'x': 0., 'y': .32, 'path_tangent': [1., 0.]}
    assert math.isclose(injector.path_clearance_from_box(path, (0., .32), .20), -.005)
    assert not candidate_still_ahead(candidate, (-.7, 0.), path, .20)
    candidate['y'] = .31
    assert candidate_still_ahead(candidate, (-.7, 0.), path, .20)
    # A simulated larger body legitimately needs another geometric decision.
    candidate['y'] = .35
    assert not candidate_still_ahead(candidate, (-.7, 0.), path, .20)
    assert candidate_still_ahead(candidate, (-.7, 0.), path, .24)


def test_segment_square_distance_checks_between_samples_and_square_corners():
    assert injector.segment_box_distance((-1., -1.), (1., 1.), (0., 0.)) == 0.
    assert math.isclose(injector.segment_box_distance((.3, .3), (.3, .3), (.5, .5)),
                        math.sqrt(2)*.075)
    assert math.isclose(injector.segment_box_distance((0., 0.), (1., 0.), (.5, .45)), .325)


def test_owned_box_move_cannot_reproduce_recorded_fourth_step_near_robot():
    # Captured stage 25 step four teleported the box toward a stationary robot
    # and reduced its actual clearance from 3.25 cm to 1.45 cm.
    before = (.47001884441225683, -.6518634459562301)
    after = (.45205379086809205, -.6692489816441315)
    robot = (.11255, -.60512)
    ready, evidence = injector.movement_step_readiness(before, after, robot, .05,
                                                       robot_radius=.20)
    assert not ready
    assert evidence['reason'] == 'robot_near_box_sweep'
    assert evidence['swept_clearance_m'] < .02
    assert evidence['required_clearance_m'] > .064
    # Waiting for the same real robot to pass permits this exact owned step;
    # no goal/velocity injection or weaker QA threshold is necessary.
    ready, evidence = injector.movement_step_readiness(before, after, (.2, -1.4), .05,
                                                       robot_radius=.20)
    assert ready
    assert evidence['swept_clearance_m'] >= evidence['required_clearance_m']


def test_owned_box_move_checks_entire_sweep_and_fresh_truth():
    # Clear endpoints do not license a swept box crossing the robot.
    ready, evidence = injector.movement_step_readiness((-.5, 0.), (.5, 0.), (0., 0.),
                                                       0., robot_radius=.20)
    assert not ready
    assert evidence['swept_clearance_m'] == -.20
    assert not injector.movement_step_readiness((1., 0.), (1.025, 0.), (0., 0.),
                                                .21, robot_radius=.20)[0]
    assert not injector.movement_step_readiness(None, (1.025, 0.), (0., 0.),
                                                .01, robot_radius=.20)[0]
    assert injector.movement_step_readiness((1., 0.), (1.025, 0.), (0., 0.),
                                            .01, robot_radius=.20)[0]


def test_geometry_comes_from_generated_nodes_yaml_and_explicit_override(tmp_path):
    import pytest
    yaml = tmp_path/'nodes.yaml'
    yaml.write_text('/astar_planner:\n  ros__parameters:\n    robot_radius: 0.20\n    obstacle_inflation_cells: 4\n')
    assert injector.geometry_parameters(yaml) == {'robot_radius': .20, 'obstacle_inflation_cells': 4}
    assert injector.geometry_parameters(yaml, .24, 5) == {'robot_radius': .24, 'obstacle_inflation_cells': 5}
    with pytest.raises(ValueError):
        injector.geometry_parameters(yaml, 0., 4)
    with pytest.raises(ValueError):
        injector.geometry_parameters(yaml, .20, -1)


def test_physical_candidate_selection_proves_actual_obstruction_and_all_moved_bypasses():
    width = height = 80
    info = SimpleNamespace(width=width, height=height, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    grid = SimpleNamespace(info=info, data=[0]*(width*height))
    path = [(.5+.05*index, 2.) for index in range(45)]
    candidate = select_feasible_web_obstacle(path, (.5, 2.), (2.7, 2.), grid,
        world_to_cell, cell_to_world, robot_radius=.20, obstacle_inflation_cells=4)
    assert candidate is not None
    assert candidate['robot_radius'] == .20
    assert candidate['initial_path_clearance_m'] < -.01
    assert len(candidate['preflight']['route_cells_by_step']) == 5
    assert candidate['preflight']['obstacle_inflation_cells'] == 4
    assert current_observed_bypass_route(grid, (.5, 2.), (2.7, 2.),
        (candidate['x'], candidate['y']), (), world_to_cell, cell_to_world,
        robot_radius=.20, obstacle_inflation_cells=4)


def classroom_grid():
    pgm = ROOT/'ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.pgm'
    with pgm.open('rb') as stream:
        assert stream.readline() == b'P5\n'
        width, height = map(int, stream.readline().split())
        assert stream.readline() == b'255\n'
        pixels = stream.read()
    assert len(pixels) == width*height
    occupancy = []
    for y in range(height):
        for x in range(width):
            gray = pixels[(height-1-y)*width+x]
            occupancy.append(100 if gray == 0 else 0 if gray == 254 else -1)
    info = SimpleNamespace(width=width, height=height, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=-1.65, y=-5.25),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    return SimpleNamespace(info=info, data=occupancy)


def test_home_departure_is_required_after_precomputed_search():
    assert not departed_home(None, (0., 0.))
    assert not departed_home((0., 0.), None)
    assert not departed_home((.099, 0.), (0., 0.))
    assert departed_home((.101, 0.), (0., 0.))


def test_empty_or_superseded_path_cannot_survive_injector_cache():
    goal = (-.188431, -.220497)
    valid = Path()
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y = goal
    valid.poses.append(pose)
    assert matching_home_path(valid, goal) == [goal]
    assert matching_home_path(Path(), goal) == []
    other = Path()
    different = PoseStamped()
    different.pose.position.x, different.pose.position.y = (1., 1.)
    other.poses.append(different)
    assert matching_home_path(other, goal) == []


def test_active_route_needs_fresh_matching_navigation_and_observations():
    expected = (-.188431, -.220497)
    kwargs = dict(goal_active=True, navigation_status='MOVING',
                  navigation_goal_xy=expected, navigation_at=99.5,
                  path=[(0., 0.), expected], path_at=76.,
                  observed_at=99.5, now=100., expected_goal_xy=expected,
                  navigation_goal_stamp=(216, 600000000),
                  expected_goal_stamp=(216, 600000000))
    assert route_readiness(**kwargs) is None  # Valid 24-second path gap.
    assert route_readiness(**{**kwargs, 'path': []}) == 'path_missing_or_stale'
    assert route_readiness(**{**kwargs, 'path_at': 69.9}) == 'path_missing_or_stale'
    assert route_readiness(**{**kwargs, 'navigation_at': 97.}) == 'navigation_status_stale'
    assert route_readiness(**{**kwargs, 'observed_at': 97.}) == 'observations_stale'
    assert route_readiness(**{**kwargs, 'goal_active': False}) == 'goal_inactive'
    assert route_readiness(**{**kwargs, 'navigation_goal_xy': (1., 1.)}) == 'navigation_goal_mismatch'
    assert route_readiness(**{**kwargs, 'navigation_goal_stamp': (216, 700000000)}) == 'navigation_goal_stamp_mismatch'
    assert route_readiness(**{**kwargs, 'navigation_status': 'FAILED'}) == 'navigation_not_moving'


def test_goal_arriving_before_first_truth_acquires_only_subsequent_truth():
    assert first_home_truth(True, None, (.2, -.5)) == (.2, -.5)
    assert first_home_truth(False, None, (.2, -.5)) is None
    assert first_home_truth(True, (0., 0.), (.2, -.5)) == (0., 0.)


def test_final_bypass_proof_includes_current_observed_cells():
    width = height = 40
    info = SimpleNamespace(width=width, height=height, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    grid = SimpleNamespace(info=info, data=[0] * (width * height))
    route = current_observed_bypass_route(
        grid, (.25, 1.), (1.7, 1.), (1., 1.), (),
        world_to_cell, cell_to_world)
    assert route is not None
    # A new full-width observed barrier closes the former bypass.
    barrier = [y * width + 17 for y in range(height)]
    assert current_observed_bypass_route(
        grid, (.25, 1.), (1.7, 1.), (1., 1.), barrier,
        world_to_cell, cell_to_world) is None
    assert not newly_observed_route_clear(route, barrier, info)
    assert newly_observed_route_clear(route, (), info)


def test_box_candidate_rejected_when_prior_sector_memory_seals_bypass():
    # Two gaps cross a wall. The test box seals the upper gap, while one
    # remembered cell seals the lower gap only after its five-cell inflation.
    width = height = 80
    info = SimpleNamespace(width=width, height=height, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    data = [0] * (width*height)
    for y in (*range(0, 15), *range(36, 45), *range(66, 80)):
        data[y*width+40] = 100
    grid = SimpleNamespace(info=info, data=data)
    start, goal = cell_to_world(info, (20, 40)), cell_to_world(info, (60, 40))
    box = cell_to_world(info, (40, 25))
    previous_memory = (55*width+40,)
    assert current_observed_bypass_route(
        grid, start, goal, box, (), world_to_cell, cell_to_world)
    assert current_observed_bypass_route(
        grid, start, goal, box, (), world_to_cell, cell_to_world,
        remembered_indices=previous_memory) is None


def test_candidate_reserves_one_cell_lidar_overshoot_seen_in_sector_three_trial():
    """A box-only route vanished after its first real Gazebo LiDAR scan."""
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.map_utils import occupancy_to_grid
    from smart_vision_amr.path_geometry import smooth_grid_path

    grid = classroom_grid()
    info = grid.info
    robot = (-.18826165215479984, -.22050049362091373)
    goal = (-.25, -1.98)
    static = occupancy_to_grid(grid, 65, 5)
    cells = AStarPlanner().plan(static, world_to_cell(info, *robot),
                                world_to_cell(info, *goal))
    points, _ = smooth_grid_path(cells, static.is_static_obstacle,
                                 spacing=1, shortcut_distance=16, iterations=2)
    path = [cell_to_world(info, (x-.5, y-.5)) for x, y in points]

    # Both Gazebo trials first selected this center. The detected lower face
    # occupied y=90, one cell beyond the box-only raster's y=89 boundary.
    optimistic = (-.08713200790640302, -.8871319661831518)
    start = cell_to_world(info, (32, 99))
    assert current_observed_bypass_route(
        grid, start, goal, optimistic, (), world_to_cell, cell_to_world) is None

    diagnostics = {}
    candidate = select_feasible_web_obstacle(
        path, robot, goal, grid, world_to_cell, cell_to_world,
        diagnostics=diagnostics)
    assert candidate is not None
    assert diagnostics['reason'] == 'FEASIBLE_BYPASS'
    # Selection now requires actual body/square overlap and can choose another
    # corridor. The scan overshoot must be proved for that selected box, not
    # translated from an unrelated historical wall/beam arrangement.
    assert candidate['initial_path_clearance_m'] < -.01
    assert len(candidate['preflight']['route_cells_by_step']) == 5
    center = (candidate['x'], candidate['y'])
    raw = occupancy_to_grid(grid, 65, 0)
    projected = injector.projected_lidar_box_cells(raw, info, center,
                                                  world_to_cell, cell_to_world)
    geometric = injector.box_cells_on_raw_map(raw, info, center,
                                             world_to_cell, cell_to_world)
    assert projected > geometric
    projected_indices = [y*info.width+x for x, y in projected]
    assert current_observed_bypass_route(
        grid, robot, goal, center, projected_indices,
        world_to_cell, cell_to_world) is not None


def test_candidate_search_skips_snapshot_observed_closed_box(monkeypatch):
    width = height = 80
    info = SimpleNamespace(width=width, height=height, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    grid = SimpleNamespace(info=info, data=[0] * (width * height))
    path = [(.25 + .05 * index, 2.) for index in range(36)]
    robot, goal = (.25, 2.), (2., 2.)
    checked = []

    def snapshot_bypass(*args, **kwargs):
        checked.append(args[3])
        return None if len(checked) == 1 else [(0, 0), (1, 0)]

    def static_five_pose(*args, **kwargs):
        return {'route_cells_by_step': [2] * 5, 'movement_side': 1.}

    monkeypatch.setattr(injector, 'current_observed_bypass_route', snapshot_bypass)
    monkeypatch.setattr(injector, 'preflight_box_route', static_five_pose)
    diagnostics = {}
    candidate = select_feasible_web_obstacle(
        path, robot, goal, grid, world_to_cell, cell_to_world,
        observed_indices=(21,), diagnostics=diagnostics)
    assert candidate is not None
    assert len(checked) >= 2
    assert (candidate['x'], candidate['y']) == checked[1]
    assert diagnostics['snapshot_observed_rejected'] == 1
    assert diagnostics['reason'] == 'FEASIBLE_BYPASS'


def test_spawn_ahead_bound_remains_sixty_centimeters():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    path = [(-1., 0.), (0., 0.), (1., 0.)]
    assert not candidate_still_ahead(candidate, (-.486, 0.), path)
    assert candidate_still_ahead(candidate, (-.601, 0.), path)


def test_spawn_overlap_gate_allows_nearby_wall_only_with_current_bypass_proof():
    info = SimpleNamespace(width=80, height=80, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    grid = SimpleNamespace(info=info, data=[0]*(info.width*info.height))
    box = (1.3, 2.2)
    nearby_wall = {48*info.width+26}  # Square y=[2.40, 2.45], outside box.
    # A return within the evidence association radius is not a physical box
    # overlap. The same observed cells still participate in the global proof.
    assert injector.near_obstacle_indices(
        nearby_wall, (), grid, box, cell_to_world)
    assert not injector.box_overlapping_obstacle_indices(
        nearby_wall, grid, box, cell_to_world)
    assert current_observed_bypass_route(
        grid, (.5, 2.), (3.5, 2.), box, nearby_wall,
        world_to_cell, cell_to_world,
        robot_radius=.22, obstacle_inflation_cells=5)
    barrier = {y*info.width+20 for y in range(info.height)}
    assert current_observed_bypass_route(
        grid, (.5, 2.), (3.5, 2.), box, nearby_wall | barrier,
        world_to_cell, cell_to_world,
        robot_radius=.22, obstacle_inflation_cells=5) is None


def test_spawn_overlap_uses_full_cells_not_just_centers_and_rejects_touching():
    info = SimpleNamespace(width=80, height=80, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    grid = SimpleNamespace(info=info)
    index = 40*info.width+43  # Cell x=[2.15, 2.20], center outside both boxes.
    assert injector.box_overlapping_obstacle_indices(
        (index,), grid, (2.035, 2.025), cell_to_world) == [index]
    assert injector.box_overlapping_obstacle_indices(
        (index,), grid, (2.025, 2.025), cell_to_world) == [index]
    assert injector.box_overlapping_obstacle_indices(
        (index,), grid, (2.024, 2.025), cell_to_world) == []


def test_spawn_overlap_handles_rotated_cell_corners_without_aabb_false_positive():
    yaw = math.pi/4.
    orientation = SimpleNamespace(x=0., y=0., z=math.sin(yaw/2.), w=math.cos(yaw/2.))

    def rotated_grid(center):
        return SimpleNamespace(info=SimpleNamespace(width=1, height=1, resolution=.1,
            origin=SimpleNamespace(position=SimpleNamespace(
                x=center[0], y=center[1]-.1/math.sqrt(2.)), orientation=orientation)))

    # Both world-axis bounding boxes overlap, but the rotated cell diamond
    # misses the physical box corner. Moving it 1 cm closer truly intersects.
    assert injector.near_obstacle_indices(
        (0,), (), rotated_grid((.17, .17)), (0., 0.), cell_to_world) == [0]
    assert injector.box_overlapping_obstacle_indices(
        (0,), rotated_grid((.17, .17)), (0., 0.), cell_to_world) == []
    assert injector.box_overlapping_obstacle_indices(
        (0,), rotated_grid((.16, .16)), (0., 0.), cell_to_world) == [0]


def test_callback_refresh_ends_on_both_post_check_streams_before_budget():
    now = [10.]
    observer = SimpleNamespace(robot_at=10., observed_at=10.)
    callbacks = []

    def spin_once(node, *, timeout_sec):
        now[0] += timeout_sec
        callbacks.append(now[0])
        node.robot_at = now[0]
        if now[0] >= 10.02-1e-10:
            node.observed_at = now[0]

    assert injector.refresh_placement_callbacks(
        observer, 10., .30, spin_once=spin_once, clock=lambda: now[0])
    assert len(callbacks) == 2
    assert math.isclose(now[0]-10., .02, abs_tol=1e-10)


def test_callback_refresh_missing_stream_expires_and_respects_interruption():
    now = [10.]
    observer = SimpleNamespace(robot_at=9.9, observed_at=9.9)

    def spin_once(node, *, timeout_sec):
        now[0] += timeout_sec
        node.robot_at = now[0]

    assert not injector.refresh_placement_callbacks(
        observer, 10., .15, spin_once=spin_once, clock=lambda: now[0])
    assert math.isclose(now[0], 10.15, abs_tol=1e-10)
    assert observer.observed_at == 9.9  # A timeout never invents fresh data.
    started = now[0]
    assert not injector.refresh_placement_callbacks(
        observer, started, .30, spin_once=spin_once, clock=lambda: now[0],
        active=lambda: now[0] < started+.02-1e-10)
    assert now[0]-started < .03


def test_event_refresh_preserves_short_spawn_window_without_relaxing_ahead_gate():
    now = [10.]
    observer = SimpleNamespace(robot_at=9.99, observed_at=9.99)
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    path = [(-1., 0.), (1., 0.)]

    def spin_once(node, *, timeout_sec):
        now[0] += timeout_sec
        node.robot_at = node.observed_at = now[0]

    for budget in (.30, .15):
        assert injector.refresh_placement_callbacks(
            observer, now[0], budget, spin_once=spin_once, clock=lambda: now[0])
    current_robot = (-.665+.22*(now[0]-10.), 0.)
    assert candidate_still_ahead(candidate, current_robot, path, .22)
    assert not candidate_still_ahead(candidate, (-.665+.22*.45, 0.), path, .22)


def test_candidate_search_keeps_ros_thread_free_and_uses_snapshot():
    entered = threading.Event()
    release = threading.Event()
    original_path = [[0., 0.], [1., 0.]]
    original_observed = {13}
    original_remembered = {17}
    seen = {}

    def slow_selector(path, robot, goal, grid, world_to_cell,
                      cell_to_world, observed, diagnostics, *, remembered_indices,
                      robot_radius, obstacle_inflation_cells):
        entered.set()
        assert release.wait(2.)
        seen.update(path=path, robot=robot, goal=goal,
                    observed=observed, remembered=remembered_indices, grid=grid,
                    robot_radius=robot_radius, inflation=obstacle_inflation_cells)
        diagnostics['reason'] = 'FEASIBLE_BYPASS'
        return {'x': .7, 'y': .1}

    started = time.monotonic()
    job = CandidateSearch(original_path, (0., 0.), (1., 0.), 'grid',
                          original_observed, selector=slow_selector,
                          remembered_indices=original_remembered,
                          robot_radius=.20, obstacle_inflation_cells=4)
    assert time.monotonic() - started < .2
    assert entered.wait(1.)
    # Simulated ROS callbacks can continue changing the live route while the
    # pure topology search is still running on the frozen snapshot.
    original_path[1][0] = 99.
    original_observed.add(42)
    original_remembered.add(99)
    callbacks_serviced = 0
    for _ in range(5):
        assert not job.done.is_set()
        callbacks_serviced += 1
    assert callbacks_serviced == 5
    release.set()
    assert job.done.wait(2.)
    assert job.error is None
    assert job.candidate == {'x': .7, 'y': .1}
    assert seen == {'path': ((0., 0.), (1., 0.)),
                    'robot': (0., 0.), 'goal': (1., 0.),
                    'observed': (13,), 'remembered': (17,), 'grid': 'grid',
                    'robot_radius': .20, 'inflation': 4}
    assert job.diagnostics['reason'] == 'FEASIBLE_BYPASS'


def test_candidate_search_reports_worker_failure_without_hanging():
    def broken_selector(*args, **kwargs):
        raise ValueError('bad map')

    job = CandidateSearch([], (0., 0.), (1., 0.), 'grid', (),
                          selector=broken_selector)
    assert job.done.wait(2.)
    assert job.candidate is None
    assert job.error == "ValueError('bad map')"


def test_fixed_scenario_preserves_exact_center_direction_and_all_five_poses(tmp_path):
    import json
    source = ROOT/'benchmarks/driving_parameter_tuning_20261104/scenarios/baseline_sector3_box.json'
    result = injector.obstacle_scenario_manifest(source)
    assert result['mode'] == 'FIXED_NO_RELOCATION'
    assert result['scenario'] == json.loads(source.read_text())
    positions = result['actor_positions_by_step']
    assert len(positions) == 5
    assert positions[0] == [.5239140050447512, -.599706838892526]
    assert math.isclose(math.hypot(positions[-1][0]-positions[0][0],
                                  positions[-1][1]-positions[0][1]), .10)
    assert positions[-1][0] < positions[0][0] and positions[-1][1] < positions[0][1]
    assert len(result['sha256']) == 64


def test_fixed_scenario_rejects_invalid_coordinates_tangent_and_side(tmp_path):
    import json
    import pytest
    original = {'x': 1., 'y': 2., 'path_tangent': [1., 0.], 'movement_side': -1}
    for changes in ({'x': float('nan')}, {'y': float('inf')}, {'x': True},
                    {'path_tangent': [0., 0.]}, {'path_tangent': [2., 0.]},
                    {'path_tangent': [True, 0.]}, {'path_tangent': [1.]},
                    {'movement_side': 0}, {'movement_side': True}, {'extra': 1}):
        source = tmp_path/'scene.json'
        source.write_text(json.dumps(dict(original, **changes)))
        with pytest.raises(ValueError):
            injector.obstacle_scenario_manifest(source)


def fixed_scene_context():
    info = SimpleNamespace(width=80, height=80, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    return (SimpleNamespace(info=info, data=[0]*6400),
            [(.5+.05*index, 2.) for index in range(45)],
            {'x': 1.35, 'y': 2.20, 'path_tangent': [1., 0.], 'movement_side': -1})


def test_fixed_scene_reuses_full_geometry_and_does_not_choose_opposite_side(monkeypatch):
    grid, path, scene = fixed_scene_context()
    calls = []
    original = injector.preflight_box_route
    def preflight(*args, **kwargs):
        calls.append((args[3], args[7], kwargs))
        return original(*args, **kwargs)
    monkeypatch.setattr(injector, 'preflight_box_route', preflight)
    candidate = injector.select_fixed_web_obstacle(path, (.5, 2.), (2.7, 2.), grid,
        world_to_cell, cell_to_world, scenario=scene, robot_radius=.22,
        obstacle_inflation_cells=5)
    assert candidate is not None
    assert (candidate['x'], candidate['y']) == (scene['x'], scene['y'])
    assert candidate['path_tangent'] == scene['path_tangent']
    assert candidate['preflight']['movement_side'] == -1
    assert len(candidate['preflight']['route_cells_by_step']) == 5
    assert candidate['preflight']['robot_radius'] == .22
    assert candidate['preflight']['obstacle_inflation_cells'] == 5
    assert calls[0][1] == ([1., 0.], -1)
    assert calls[0][2]['require_path_obstruction'] is True
    # A rejected fixed direction cannot silently select the opposite side.
    calls.clear()
    def reject(*args, **kwargs):
        calls.append(args[7])
        return None
    monkeypatch.setattr(injector, 'preflight_box_route', reject)
    diagnostics = {}
    assert injector.select_fixed_web_obstacle(path, (.5, 2.), (2.7, 2.), grid,
        world_to_cell, cell_to_world, diagnostics=diagnostics, scenario=scene,
        robot_radius=.22, obstacle_inflation_cells=5) is None
    assert calls == [([1., 0.], -1)]
    assert diagnostics['reason'] == 'FIXED_SCENE_MOVED_BYPASS_UNPROVED'
    assert diagnostics['input_status'] == 'INVALID_INPUT'


def test_fixed_scene_does_not_relocate_when_new_route_avoids_box(monkeypatch):
    grid, path, scene = fixed_scene_context()
    diagnostics = {}
    monkeypatch.setattr(injector, 'select_feasible_web_obstacle',
                        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError('adaptive relocation')))
    assert injector.select_fixed_web_obstacle([(x, 1.5) for x, _ in path], (.5, 1.5),
        (2.7, 1.5), grid, world_to_cell, cell_to_world, diagnostics=diagnostics,
        scenario=scene, robot_radius=.22, obstacle_inflation_cells=5) is None
    assert diagnostics['reason'] == 'FIXED_SCENE_DOES_NOT_BLOCK_CURRENT_PATH'
    assert diagnostics['input_status'] == 'INVALID_INPUT'


def test_fixed_scene_keeps_observed_memory_and_initial_ahead_gates(monkeypatch):
    grid, path, scene = fixed_scene_context()
    for robot, observed, memory, expected in (
            ((.80, 2.), (), (), 'FIXED_SCENE_NOT_AHEAD_OR_OUT_OF_RANGE'),
            ((.5, 2.), (44*80+27,), (), 'FIXED_SCENE_TOO_CLOSE_TO_OBSERVED_CELL'),
            ((.5, 2.), (), (40*80+10,), 'FIXED_SCENE_CURRENT_BYPASS_CLOSED')):
        diagnostics = {}
        assert injector.select_fixed_web_obstacle(path, robot, (2.7, 2.), grid,
            world_to_cell, cell_to_world, observed, diagnostics, scenario=scene,
            remembered_indices=memory, robot_radius=.22, obstacle_inflation_cells=5) is None
        assert diagnostics['reason'] == expected
