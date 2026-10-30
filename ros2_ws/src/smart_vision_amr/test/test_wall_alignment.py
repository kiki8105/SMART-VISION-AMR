"""Marker-selected surface geometry, ambiguity rejection and mounting frames."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from smart_vision_amr.wall_alignment import estimate_camera_axis_wall, estimate_marker_wall


def wall_scan(normal_yaw=0., distance=1.5, laser=(0., 0., 0.), noise=0., corner=False):
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    rng = np.random.default_rng(31)
    normal = np.array([math.cos(normal_yaw), math.sin(normal_yaw)])
    origin = np.array(laser[:2])
    ranges = []
    for angle in angles+laser[2]:
        ray = np.array([math.cos(angle), math.sin(angle)])
        denominator = float(normal @ ray)
        value = (distance-float(normal @ origin))/denominator if denominator > .01 else math.inf
        if corner:
            other = np.array([math.cos(normal_yaw+math.pi/2), math.sin(normal_yaw+math.pi/2)])
            divisor = float(other @ ray)
            if divisor > .01:
                value = min(value, (distance-float(other @ origin))/divisor)
        ranges.append(value+rng.normal(0., noise) if math.isfinite(value) else value)
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                           angle_increment=2*math.pi/720, range_min=.12, range_max=8.)


def estimate(scan, *, laser=(0., 0., 0.), camera=(0., 0., 0.), center=0., **kwargs):
    return estimate_marker_wall(scan, laser, camera_pose=camera, center_x=center,
                                horizontal_fov=math.radians(62), **kwargs)


@pytest.mark.parametrize('normal,camera', [(0., 0.), (.12, 0.), (-.18, 0.),
                                         (math.pi, math.pi), (math.pi-.15, math.pi),
                                         (1.3, 1.2)])
def test_front_rear_and_arbitrary_camera_yaw_use_explicit_mount(normal, camera):
    result = estimate(wall_scan(normal, noise=.004), camera=(.05, -.02, camera))
    assert result['reliable'], result
    expected = math.atan2(math.sin(normal-camera), math.cos(normal-camera))
    assert result['heading_error_rad'] == pytest.approx(expected, abs=.015)
    assert result['distance_m'] == pytest.approx(1.5, abs=.02)
    assert result['support_count'] >= 12
    assert result['confidence'] > .8


def test_laser_translation_and_yaw_are_applied_before_fitting():
    laser = (-.064, .025, .17)
    result = estimate(wall_scan(.2, laser=laser, noise=.006), laser=laser,
                      camera=(.07, -.01, 0.))
    assert result['reliable'], result
    assert result['normal_yaw_base'] == pytest.approx(.2, abs=.015)
    assert result['distance_m'] == pytest.approx(1.5, abs=.02)


def test_marker_pixel_bearing_selects_surface_instead_of_nearest_side():
    # The observed marker is toward pixel-right; its ray selects a wall there.
    scan = wall_scan(-.3, noise=.002)
    result = estimate(scan, center=.6)
    assert result['reliable'], result
    assert result['marker_bearing_base'] < 0
    assert result['heading_error_rad'] == pytest.approx(-.3, abs=.01)


def test_sparse_returns_and_missing_center_ray_cannot_prove_alignment():
    scan = wall_scan()
    scan.ranges = [value if index % 10 == 0 else math.inf for index, value in enumerate(scan.ranges)]
    result = estimate(scan)
    assert not result['reliable']
    scan = wall_scan()
    scan.ranges[350:371] = [math.inf]*21
    result = estimate(scan)
    assert not result['reliable']
    assert result['reason'] == 'no_single_surface_spans_marker_bearing'


def test_corner_spanning_marker_bearing_is_rejected():
    result = estimate(wall_scan(corner=True), camera=(0., 0., math.pi/4))
    assert not result['reliable'], result
    assert result['reason'] in ('mixed_surfaces_or_corner', 'nonplanar_or_short_surface')


def test_isolated_noise_is_robust_but_large_mixed_surface_is_rejected():
    scan = wall_scan(noise=.005)
    for index in (340, 352, 370, 388):
        scan.ranges[index] += .055
    result = estimate(scan)
    assert result['reliable'], result
    assert result['heading_error_rad'] == pytest.approx(0., abs=.02)
    scan = wall_scan(noise=.06)
    assert not estimate(scan)['reliable']


@pytest.mark.parametrize('distance', [.18, 4.])
def test_too_near_or_far_wall_rejected(distance):
    assert not estimate(wall_scan(distance=distance))['reliable']


@pytest.mark.parametrize('kwargs', [{'camera': (0., 0., math.nan)}, {'center': True},
                                  {'center': 2.}, {'min_support': 1}, {'min_span': 0.},
                                  {'cone_half_angle': 2.}, {'max_residual': math.inf}])
def test_invalid_inputs_fail_closed(kwargs):
    result = estimate(wall_scan(), **kwargs)
    assert not result['reliable']
    assert result['heading_error_rad'] is None


def test_short_wall_cannot_be_confirmed_even_with_many_samples():
    result = estimate(wall_scan(distance=.3), cone_half_angle=math.radians(12))
    assert not result['reliable'], result


def test_malformed_scan_and_unknown_mount_fail_without_throwing():
    assert not estimate(SimpleNamespace())['reliable']
    assert not estimate(wall_scan(), laser=None)['reliable']


@pytest.mark.parametrize('camera_yaw', [0., math.pi, -.7])
def test_axis_wall_does_not_require_or_invent_camera_fov(camera_yaw):
    result = estimate_camera_axis_wall(wall_scan(camera_yaw+.1), (0., 0., 0.),
                                       camera_pose=(.04, -.01, camera_yaw))
    assert result['reliable'], result
    assert result['selection'] == 'centered_camera_axis'
    assert result['heading_error_rad'] == pytest.approx(.1, abs=.01)


@pytest.mark.parametrize('center', [-.1, -.051, .051, .3])
def test_uncalibrated_fov_rejects_marker_outside_strict_axis_center(center):
    result = estimate_marker_wall(wall_scan(), (0., 0., 0.), camera_pose=(0., 0., 0.),
                                  center_x=center)
    assert not result['reliable']


def test_axis_wall_requires_known_camera_mount():
    result = estimate_camera_axis_wall(wall_scan(), (0., 0., 0.), camera_pose=None)
    assert not result['reliable']


@pytest.mark.parametrize('base_wall_distance', [.425, .45])
def test_close_flat_wall_with_real_rear_camera_and_laser_offsets_has_enough_support(base_wall_distance):
    laser = (-.064, 0., math.pi)
    camera = (-.174, 0., math.pi)
    scan = wall_scan(math.pi, distance=base_wall_distance, laser=laser)
    narrow = estimate_camera_axis_wall(scan, laser, camera_pose=camera,
                                       cone_half_angle=math.radians(20))
    assert not narrow['reliable'], narrow
    assert narrow['span_m'] < .25
    result = estimate_camera_axis_wall(scan, laser, camera_pose=camera)
    assert result['reliable'], result
    assert result['span_m'] >= .25 and result['support_count'] >= 12
    assert result['distance_m'] == pytest.approx(base_wall_distance, abs=.001)
    assert result['heading_error_rad'] == pytest.approx(0., abs=.001)


@pytest.mark.parametrize('camera_y', [-.04, 0., .04])
def test_close_rear_wall_corner_still_cannot_prove_a_single_wall(camera_y):
    # Two perpendicular walls meet just behind the camera axis, including a
    # small lateral camera/marker offset. The wider cone must reject the mix.
    laser = (-.064, 0., math.pi)
    scan = wall_scan(3*math.pi/4, distance=.425/math.sqrt(2), laser=laser, corner=True)
    result = estimate_camera_axis_wall(scan, laser, camera_pose=(-.174, camera_y, math.pi))
    assert not result['reliable'], result


def test_close_rear_board_below_minimum_camera_distance_remains_rejected():
    laser = (-.064, 0., math.pi)
    scan = wall_scan(math.pi, distance=.415, laser=laser)
    result = estimate_camera_axis_wall(scan, laser, camera_pose=(-.174, 0., math.pi))
    assert not result['reliable'], result
