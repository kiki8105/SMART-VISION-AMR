"""Selected fixed scenes use fresh physical clearance at spawn, not search range."""
from pathlib import Path
import math
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
import inject_classroom_web_obstacle as injector


CANDIDATE = {'x': .44, 'y': -.60,
             'path_tangent': [.6954214275160555, -.7186021417665909]}
# The unchanged recorded route crossed the box's expanded physical footprint.
RECORDED_PATH = [(-.225, -.275), (-.0875, -.4125), (.05, -.55),
                 (.18, -.68), (.30, -.80), (.4477, -.9477)]


def ready(candidate, robot, path, age=.05, **context):
    values = dict(route_reason=None, current_goal_stamp=(9, 320000000),
                  expected_goal_stamp=(9, 320000000))
    values.update(context)
    return injector.spawn_placement_readiness(
        candidate, robot, path, age, .22, **values)


def test_recorded_point_616_to_590_keeps_same_safe_fixed_scene():
    before, after = (-.10724, -.31660), (-.08884, -.33850)
    assert injector.candidate_still_ahead(CANDIDATE, before, RECORDED_PATH, .22)
    assert not injector.candidate_still_ahead(CANDIDATE, after, RECORDED_PATH, .22)
    assert math.isclose(math.dist(before, (.44, -.6)), .616269, abs_tol=1e-6)
    assert math.isclose(math.dist(after, (.44, -.6)), .589961, abs_tol=1e-6)
    for robot, initial_range in ((before, True), (after, False)):
        accepted, metrics = ready(CANDIDATE, robot, RECORDED_PATH)
        assert accepted
        assert metrics['initial_selection_distance_window'] is initial_range
        assert metrics['swept_clearance_m'] >= metrics['required_clearance_m']
        assert metrics['current_path_clearance_m'] < -.01
        assert metrics['approach_along_path_m'] <= -.25
    assert (CANDIDATE['x'], CANDIDATE['y']) == (.44, -.60)


def test_initial_selection_range_is_unchanged():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    path = [(-2., 0.), (2., 0.)]
    assert injector.candidate_still_ahead(candidate, (-.60, 0.), path, .22)
    assert injector.candidate_still_ahead(candidate, (-1., 0.), path, .22)
    assert not injector.candidate_still_ahead(candidate, (-.59, 0.), path, .22)
    assert not injector.candidate_still_ahead(candidate, (-1.01, 0.), path, .22)


def test_actual_overlap_and_reaction_margin_still_reject_spawn():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    path = [(-1., 0.), (1., 0.)]
    for robot in ((-.30, 0.), (-.40, 0.)):
        accepted, metrics = ready(candidate, robot, path)
        assert not accepted
        assert metrics['reason'] == 'robot_near_box_sweep'
        assert metrics['swept_clearance_m'] < metrics['required_clearance_m']
    # The second pose is physically outside the body but lacks reaction room.
    assert injector.path_clearance_from_box([(-.40, 0.)], (0., 0.), .22) > .02


def test_truth_age_distance_is_not_ignored():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    path = [(-1., 0.), (1., 0.)]
    assert ready(candidate, (-.45, 0.), path, .10)[0]
    accepted, metrics = ready(candidate, (-.45, 0.), path, .20)
    assert not accepted
    assert math.isclose(metrics['required_clearance_m'], .02 + .22 * (.20 + .20))


@pytest.mark.parametrize('age', [None, -.001, .20001, float('nan')])
def test_unavailable_or_stale_truth_rejects_spawn(age):
    accepted, metrics = ready(CANDIDATE, (-.10724, -.31660), RECORDED_PATH, age)
    assert not accepted
    assert metrics['reason'] == 'truth_unavailable_or_stale'


def test_actually_passed_scene_rejects_even_if_body_clear():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    accepted, metrics = ready(candidate, (.6, 0.), [(-1., 0.), (1., 0.)])
    assert not accepted
    assert metrics['reason'] == 'candidate_actually_passed_or_not_ahead'


def test_latest_path_must_still_really_be_blocked():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    accepted, metrics = ready(candidate, (-.7, 0.), [(-1., .6), (1., .6)])
    assert not accepted
    assert metrics['reason'] == 'current_route_no_longer_blocked'
    # A half-centimeter contact does not meet the retained >1cm obstruction.
    accepted, metrics = ready(candidate, (-.7, 0.), [(-1., .34), (1., .34)])
    assert not accepted
    assert metrics['reason'] == 'current_route_no_longer_blocked'


@pytest.mark.parametrize('reason', ['navigation_not_moving', 'path_missing_or_stale',
                                    'observations_stale', 'goal_inactive'])
def test_existing_route_freshness_and_moving_checks_remain_required(reason):
    assert not ready(CANDIDATE, (-.10724, -.31660), RECORDED_PATH,
                     route_reason=reason)[0]


def test_same_coordinates_with_new_goal_generation_cannot_reuse_old_proof():
    accepted, metrics = ready(CANDIDATE, (-.10724, -.31660), RECORDED_PATH,
                              current_goal_stamp=(10, 0))
    assert not accepted
    assert metrics['reason'] == 'goal_rearmed_during_preflight'


def test_route_lease_is_rechecked_after_expensive_bypass_search():
    def route_reason(now):
        return injector.route_readiness(
            True, 'MOVING', (-.25, -1.98), 10., RECORDED_PATH, 10., 10.,
            now, (-.25, -1.98), (9, 320000000), (9, 320000000))
    assert route_reason(10.1) is None
    # Truth may be freshly received, but an old MOVING heartbeat cannot
    # authorize a spawn after the synchronous memory-bypass proof took time.
    reason = route_reason(12.01)
    assert reason == 'navigation_status_stale'
    accepted, metrics = ready(CANDIDATE, (-.08884, -.33850), RECORDED_PATH,
                              age=.01, route_reason=reason)
    assert not accepted
    assert metrics['reason'] == 'navigation_status_stale'
