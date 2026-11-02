import threading
import time
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


def test_candidate_search_keeps_ros_thread_free_and_uses_snapshot():
    entered = threading.Event()
    release = threading.Event()
    original_path = [[0., 0.], [1., 0.]]
    original_observed = {13}
    seen = {}

    def slow_selector(path, robot, goal, grid, world_to_cell,
                      cell_to_world, observed, diagnostics):
        entered.set()
        assert release.wait(2.)
        seen.update(path=path, robot=robot, goal=goal,
                    observed=observed, grid=grid)
        diagnostics['reason'] = 'FEASIBLE_BYPASS'
        return {'x': .7, 'y': .1}

    started = time.monotonic()
    job = CandidateSearch(original_path, (0., 0.), (1., 0.), 'grid',
                          original_observed, selector=slow_selector)
    assert time.monotonic() - started < .2
    assert entered.wait(1.)
    # Simulated ROS callbacks can continue changing the live route while the
    # pure topology search is still running on the frozen snapshot.
    original_path[1][0] = 99.
    original_observed.add(42)
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
                    'observed': (13,), 'grid': 'grid'}
    assert job.diagnostics['reason'] == 'FEASIBLE_BYPASS'


def test_candidate_search_reports_worker_failure_without_hanging():
    def broken_selector(*args):
        raise ValueError('bad map')

    job = CandidateSearch([], (0., 0.), (1., 0.), 'grid', (),
                          selector=broken_selector)
    assert job.done.wait(2.)
    assert job.candidate is None
    assert job.error == "ValueError('bad map')"
