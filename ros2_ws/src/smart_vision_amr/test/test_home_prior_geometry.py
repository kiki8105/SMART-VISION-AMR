"""Repeated, bounded Home evidence cannot stand in for localization confidence."""
import math

import numpy as np
import pytest

from smart_vision_amr.home_prior_geometry import home_neighborhood_evidence
from smart_vision_amr.localization_geometry import occupied_distance, scan_map_agreement


def scene():
    data = np.zeros((80, 80), dtype=np.int16)
    data[:, 60] = 100
    metadata = {'width': 80, 'height': 80, 'resolution': .05,
                'origin': {'x': 0., 'y': 0., 'yaw': 0.}}
    home = {'x': .525, 'y': 2.025, 'yaw': 0.}
    # The robot was placed 20 cm farther along x than the registered Home.
    points = np.column_stack((np.full(80, 2.3), np.linspace(-.7, .7, 80)))
    observations = [(101, points), (102, points + [0., .001]),
                    (103, points + [0., -.001])]
    return observations, home, metadata, occupied_distance(data, .05), data


def test_nearby_home_support_requires_repeated_common_candidates_without_returning_pose():
    observations, home, metadata, distance, data = scene()
    assert scan_map_agreement(observations[0][1], home, metadata, distance) < .65
    result = home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert result['supported']
    assert result['reason'] == 'home_neighborhood_supported'
    assert result['distinct_scans'] == 3
    assert result['consistent_candidates'] >= 3
    assert result['support_clusters'] >= 1
    assert result['best_min_agreement'] >= .65
    assert result['checked_candidates'] <= result['shortlisted_candidates']
    assert not any('pose' in key or 'position' in key for key in result)
    assert home == {'x': .525, 'y': 2.025, 'yaw': 0.}


def test_identical_stamp_does_not_become_repeated_sensor_evidence():
    observations, *rest = scene()
    result = home_neighborhood_evidence([observations[0]] * 3, *rest)
    assert not result['supported']
    assert result['distinct_scans'] == 1
    assert result['reason'] == 'insufficient_distinct_scans'


def test_two_scans_do_not_enable_home_prior():
    observations, *rest = scene()
    result = home_neighborhood_evidence(observations[:2], *rest)
    assert not result['supported']
    assert result['reason'] == 'insufficient_distinct_scans'


def test_each_observation_must_support_the_same_candidate_neighborhood():
    observations, *rest = scene()
    # Scan 3 would require moving the candidate 80 cm to hit the measured wall.
    observations[2] = (103, observations[2][1] + [.8, 0.])
    result = home_neighborhood_evidence(observations, *rest)
    assert not result['supported']
    assert result['reason'] == 'no_repeated_endpoint_match'


def test_endpoint_matches_behind_intervening_wall_do_not_enable_prior():
    observations, home, metadata, _, data = scene()
    data[:, 30] = 100
    distance = occupied_distance(data, .05)
    result = home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert result['shortlisted_candidates'] > 0
    assert not result['supported']
    assert result['reason'] == 'no_repeated_line_of_sight_match'


def test_unknown_ray_interior_is_not_treated_as_clear_space():
    observations, home, metadata, _, data = scene()
    data[:, 30] = -1
    distance = occupied_distance(data, .05)
    result = home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert not result['supported']
    assert result['reason'] == 'no_repeated_line_of_sight_match'


@pytest.mark.parametrize('cell_value', [-1, 100])
def test_unknown_or_occupied_footprint_invalidates_all_nearby_candidates(cell_value):
    observations, home, metadata, _, data = scene()
    data[20:65, :24] = cell_value
    distance = occupied_distance(data, .05)
    result = home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert not result['supported']
    assert result['footprint_candidates'] == 0
    assert result['reason'] == 'no_free_footprint_near_home'


def test_footprint_outside_map_is_never_validated():
    observations, home, metadata, distance, data = scene()
    home['x'] = -.4
    result = home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert not result['supported']
    assert result['reason'] == 'no_free_footprint_near_home'


def test_rotated_map_origin_preserves_the_same_near_home_evidence():
    observations, home, metadata, distance, data = scene()
    metadata['origin'] = {'x': -3., 'y': 4., 'yaw': math.pi / 2}
    home = {'x': -5.025, 'y': 4.525, 'yaw': math.pi / 2}
    result = home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert result['supported']


@pytest.mark.parametrize('options', [
    {'translation_radius': .3001}, {'yaw_limit': math.radians(15.1)},
    {'min_agreement': .6499}, {'footprint_radius': .219},
    {'translation_step': .001}, {'yaw_step': math.radians(.1)},
    {'max_shortlist': 10000}, {'max_shortlist': True},
    {'translation_radius': float('nan')},
])
def test_configuration_cannot_enlarge_search_or_weaken_required_evidence(options):
    with pytest.raises(ValueError):
        home_neighborhood_evidence(*scene(), **options)


@pytest.mark.parametrize('bad', [np.zeros((11, 2)), np.zeros((30, 3)),
                                 np.full((20, 2), np.nan)])
def test_invalid_scan_geometry_fails_closed(bad):
    observations, *rest = scene()
    observations[1] = (102, bad)
    result = home_neighborhood_evidence(observations, *rest)
    assert not result['supported']
    assert result['reason'] == 'invalid_scan_geometry'


def test_mismatched_map_dimensions_fail_closed():
    observations, home, metadata, distance, data = scene()
    result = home_neighborhood_evidence(observations, home, metadata, distance[:-1], data)
    assert not result['supported']
    assert result['reason'] == 'invalid_map_geometry'


def test_initial_zero_or_boolean_stamp_is_not_evidence():
    observations, *rest = scene()
    for stamp in (0, True):
        observations[0] = (stamp, observations[0][1])
        result = home_neighborhood_evidence(observations, *rest)
        assert not result['supported']
        assert result['reason'] == 'invalid_scan_identity'


def test_every_searched_candidate_stays_in_circular_hard_bounds(monkeypatch):
    from smart_vision_amr import home_prior_geometry as geometry
    observations, home, metadata, distance, data = scene()
    searched = []

    def record_endpoint_bound(points, candidates, *_):
        searched.append(candidates.copy())
        return np.ones(len(candidates))

    monkeypatch.setattr(geometry, '_endpoint_scores', record_endpoint_bound)
    monkeypatch.setattr(geometry, 'scan_map_agreement', lambda *args: 1.)
    assert geometry.home_neighborhood_evidence(observations, home, metadata,
                                              distance, data)['supported']
    for candidates in searched:
        offset = candidates[:, :2]-[home['x'], home['y']]
        assert np.all(np.linalg.norm(offset, axis=1) <= .30+1e-12)
        assert np.all(np.abs(candidates[:, 2]-home['yaw']) <= math.radians(15.)+1e-12)


def test_one_isolated_repeated_candidate_does_not_establish_cluster_support(monkeypatch):
    from smart_vision_amr import home_prior_geometry as geometry
    observations, home, metadata, distance, data = scene()
    first = []

    def record_endpoint_bound(points, candidates, *_):
        if not first:
            first.append(candidates[0].copy())
        return np.ones(len(candidates))

    def only_one_candidate(points, pose, *_):
        candidate = np.array([pose['x'], pose['y'], pose['yaw']])
        return float(np.allclose(candidate, first[0], atol=1e-12))

    monkeypatch.setattr(geometry, '_endpoint_scores', record_endpoint_bound)
    monkeypatch.setattr(geometry, 'scan_map_agreement', only_one_candidate)
    result = geometry.home_neighborhood_evidence(observations, home, metadata, distance, data)
    assert not result['supported']
    assert result['consistent_candidates'] == 1
    assert result['reason'] == 'no_consistent_candidate_cluster'
