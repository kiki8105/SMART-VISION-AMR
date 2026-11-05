"""Raw-scan interlock follows base-frame body geometry and requested motion."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from smart_vision_amr.scan_outlier_guard import (rebase_scan_points, scan_points_in_base,
    swept_body_decision)


def check(points, v=.1, w=0., radius=.22):
    return swept_body_decision([(i, *p) for i, p in enumerate(points)], v, w,
                               radius=radius, horizon=.5)


def test_parallel_side_wall_outside_body_does_not_stop_forward_or_rotation():
    points = [(-.08, .235), (0., .235), (.10, .235), (.25, .235)]
    assert not check(points).swept
    assert not check(points, v=0., w=.15).swept
    assert check(points, v=.2, w=1.).swept  # Turning toward the side can intersect it.


def test_current_body_contact_stops_every_motion_including_away_and_reverse():
    for v, w in [(.1, 0), (-.1, 0), (0, .15), (0, -.15)]:
        result = check([(0., .219)], v, w)
        assert result.inside == result.swept == {0}


def test_forward_reverse_and_turn_direction_check_full_swept_circle():
    assert check([(.25, 0.)]).swept == {0}
    assert not check([(.25, 0.)], v=-.1).swept
    assert check([(-.25, 0.)], v=-.1).swept == {0}
    assert not check([(-.25, 0.)]).swept
    assert check([(.12, .22)], v=.2, w=1.).swept
    assert not check([(.12, .22)], v=.2, w=-1.).swept


def test_laser_origin_offset_and_rotation_are_not_hidden_body_padding():
    scan = SimpleNamespace(range_min=.12, range_max=4., angle_min=math.pi/2,
                           angle_increment=.01, ranges=[.22, math.inf, math.nan])
    points = scan_points_in_base(scan, (-.064, 0., 0.))
    assert len(points) == 1
    assert math.hypot(points[0][1], points[0][2]) > .22
    assert not swept_body_decision(points, .1, 0.).swept
    rotated = scan_points_in_base(scan, (0., 0., -math.pi/2))
    assert rotated[0][1] == pytest.approx(.22)
    assert rotated[0][2] == pytest.approx(0.)
    assert swept_body_decision(rotated, .1, 0.).inside == {0}
    assert scan.ranges[1] == math.inf and math.isnan(scan.ranges[2])


def test_arc_distance_matches_dense_independent_center_sampling():
    points = [(x/20, y/20) for x in range(-8, 9) for y in range(-8, 9)]
    for v, w in [(.22, .5), (.22, -.5), (-.1, .8), (-.1, -.8)]:
        actual = check(points, v, w)
        centers = [(v/w*math.sin(w*t/4000), v/w*(1-math.cos(w*t/4000)))
                   for t in range(2001)]
        numerical = {i for i, (x, y) in enumerate(points)
                     if min(math.hypot(x-a, y-b) for a, b in centers) <= .22}
        assert actual.swept == numerical


def historical_fixture(name):
    data = json.loads((Path(__file__).parent/'fixtures'/name).read_text())
    raw = data['scan']
    raw['ranges'] = [v if v is not None else math.inf for v in raw['ranges']]
    return data, SimpleNamespace(**raw)


def test_recorded_parallel_returns_allow_historical_twenty_centimeter_motion():
    data, scan = historical_fixture('post_obstacle_scan_guard_20261104.json')
    points = scan_points_in_base(scan, data['extrinsic'])
    assert not swept_body_decision(points, *data['requested_twist'], radius=.20).swept
    assert not swept_body_decision(points, *data['requested_twist'], radius=.22).swept


def test_historical_later_scan_inside_new_twenty_two_body_remains_a_hold():
    data, scan = historical_fixture('post_obstacle_scan_guard_late_20261104.json')
    points = scan_points_in_base(scan, data['extrinsic'])
    old = swept_body_decision(points, *data['requested_twist'], radius=.20)
    new = swept_body_decision(points, *data['requested_twist'], radius=.22)
    assert not old.inside
    assert new.inside  # Enlarging the model cannot label these points clear.


@pytest.mark.parametrize('kwargs', [{'radius':0.}, {'horizon':0.}, {'horizon':3.}])
def test_invalid_geometry_fails_closed(kwargs):
    with pytest.raises(ValueError):
        swept_body_decision([], .1, .1, **kwargs)


def test_acquisition_pose_to_current_base_compensates_translation_and_rotation():
    points = [(5, .36, 0.)]
    current = rebase_scan_points(points, (1., 2., 0.), (1.044, 2., math.pi/2))
    assert current[0][0] == 5
    assert current[0][1] == pytest.approx(0., abs=1.e-12)
    assert current[0][2] == pytest.approx(-.316)


def test_latest_pose_motion_bound_is_directional_not_full_radial_scan_age_padding():
    # A short pose age adds forward/backward uncertainty; a parallel wall
    # outside the body does not inherit a fictitious 5 cm radial expansion.
    result = swept_body_decision([(0, 0., .24)], .22, 0., radius=.22,
        longitudinal_uncertainty=.05, lateral_uncertainty=.001, yaw_uncertainty=.01)
    assert not result.swept
    assert swept_body_decision([(0, .25, 0.)], 0., 0., radius=.22,
        longitudinal_uncertainty=.05).inside == {0}


def test_horizontal_uncertainty_interval_arc_distance_is_conservative():
    points = [(i, x/20, y/20) for i, (x, y) in enumerate(
        ( (x,y) for x in range(-7,8) for y in range(-7,8) ))]
    for v, w in [(.22, .7), (-.1, .7), (.22, -.7)]:
        result = swept_body_decision(points, v, w, radius=.22,
            longitudinal_uncertainty=.025, lateral_uncertainty=.002, yaw_uncertainty=.03)
        # Independently sample actual displacement and heading error inside
        # those bounds. A possible collision must never be labelled clear.
        numerical = set()
        for step in range(51):
            t = step*.5/50
            cx, cy = v/w*math.sin(w*t), v/w*(1-math.cos(w*t))
            for angle in [-.03, 0., .03]:
                for dx in [-.025, 0., .025]:
                    for dy in [-.002, .002]:
                        a = dx+math.cos(angle)*cx-math.sin(angle)*cy
                        b = dy+math.sin(angle)*cx+math.cos(angle)*cy
                        numerical.update(i for i,x,y in points if math.hypot(x-a,y-b) <= .22)
        assert numerical <= result.swept


@pytest.mark.parametrize('v,w,horizon', [
    (0., 0., .5), (0., .5, .5), (.22, 0., .5), (-.10, 0., .5),
    (.15, .35, .5), (-.10, .35, .5), (.10, -.35, .5), (-.15, -.35, .5),
    (.10, 1.e-7, .5), (-.10, -1.e-7, .5), (.10, 4., 2.), (-.10, -4., 2.)])
def test_vector_raw_sweep_matches_frozen_scalar_including_uncertainty(v, w, horizon):
    from scalar_sweep_reference import scalar_swept_body_decision
    points = [(i, x/40, y/40) for i, (x, y) in enumerate(
        ( (x, y) for x in range(-20, 21) for y in range(-20, 21) ))]
    bounds = dict(radius=.22, horizon=horizon, longitudinal_uncertainty=.025,
                  lateral_uncertainty=.002, yaw_uncertainty=.03)
    actual = swept_body_decision(points, v, w, **bounds)
    scalar = scalar_swept_body_decision(points, v, w, **bounds)
    assert actual.inside == scalar.inside
    assert actual.swept == scalar.swept
    assert actual.minimum_body_clearance == pytest.approx(scalar.minimum_body_clearance, abs=1.e-12)
    assert actual.minimum_swept_clearance == pytest.approx(scalar.minimum_swept_clearance, abs=1.e-12)
