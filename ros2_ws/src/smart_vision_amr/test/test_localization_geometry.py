"""Independent wall/line-of-sight checks for confident but incorrect AMCL poses."""
import math

import numpy as np

from smart_vision_amr.localization_geometry import occupied_distance, scan_map_agreement


def scene():
    data = np.zeros((80, 80), dtype=np.int16)
    data[:, 60] = 100  # Measured wall at x=3.025 m.
    metadata = {'width': 80, 'height': 80, 'resolution': .05,
                'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    pose = {'x': .525, 'y': 2.025, 'yaw': 0.}
    points = np.column_stack((np.full(20, 2.5), np.linspace(-.7, .7, 20)))
    return data, metadata, pose, points


def test_clean_rays_to_known_wall_pass():
    data, metadata, pose, points = scene()
    assert scan_map_agreement(points, pose, metadata, occupied_distance(data, .05)) == 1.


def test_matching_endpoints_behind_an_intervening_wall_are_rejected():
    data, metadata, pose, points = scene()
    data[:, 30] = 100  # Endpoint-only matching would incorrectly return 1.0.
    assert scan_map_agreement(points, pose, metadata, occupied_distance(data, .05)) == 0.


def test_unknown_ray_interior_cannot_confirm_position_even_if_endpoint_matches():
    data, metadata, pose, points = scene()
    data[:, 30] = -1
    assert scan_map_agreement(points, pose, metadata, occupied_distance(data, .05)) == 0.


def test_unknown_endpoint_stays_invalid():
    data, metadata, pose, points = scene()
    data[:, 60] = -1
    assert scan_map_agreement(points, pose, metadata, occupied_distance(data, .05)) == 0.


def test_ray_check_excludes_robot_body_and_wall_surface_margins():
    data, metadata, pose, points = scene()
    data[:, 11] = 100  # 5 cm from center: inside the body exclusion zone.
    data[:, 58] = 100  # 10 cm before endpoints: within the wall surface margin.
    assert scan_map_agreement(points, pose, metadata, occupied_distance(data, .05)) == 1.


def test_rotated_map_origin_and_robot_orientation_preserve_ray_geometry():
    data, metadata, pose, points = scene()
    metadata['origin'] = {'x': -3., 'y': 4., 'yaw': math.pi/2}
    pose = {'x': -5.025, 'y': 4.525, 'yaw': math.pi/2}
    assert scan_map_agreement(points, pose, metadata, occupied_distance(data, .05)) == 1.


def test_subsampling_is_bounded_without_ignoring_last_part_of_scan():
    data, metadata, pose, points = scene()
    dense = np.repeat(points, 16, axis=0)[:319]
    dense[-100:, 0] = 1.  # These endpoints fail the measured-wall match.
    score = scan_map_agreement(dense, pose, metadata, occupied_distance(data, .05))
    assert .65 < score < .75
