"""Offline DWB evidence preserves recorded input and full critic attribution."""

import gzip
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


SCRIPT = Path(__file__).resolve().parents[4] / 'scripts/replay_incident_controller.py'
spec = importlib.util.spec_from_file_location('incident_controller_replay', SCRIPT)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def event(topic, data, seconds):
    return {'topic': topic, 'data': data, 'ros_time_ns': int(seconds * 1.e9)}


def test_snapshot_uses_received_time_without_looking_ahead(tmp_path):
    incident = tmp_path / 'events.jsonl.gz'
    events = [event('/scan', {'ranges': [.25, 1.2]}, 10.),
              event('/odom', {'pose': {'pose': {'position': {'x': .4}}}}, 10.),
              event('/smart_vision_amr/path', {'poses': [{'original': True}]}, 10.),
              event('/tf', {'transforms': [
                  {'header': {'frame_id': 'map', 'stamp': {'sec': 12}},
                   'child_frame_id': 'odom', 'transform': {'x': 1}}]}, 10.),
              event('/scan', {'ranges': [.30, 1.3]}, 11.),
              event('/scan', {'ranges': [.01, .01]}, 12.),
              event('/smart_vision_amr/path', {'poses': []}, 12.)]
    with gzip.open(incident, 'wt') as stream:
        stream.write('\n'.join(json.dumps(entry) for entry in events))
    snapshot = replay.load_snapshot(incident, 11.)
    assert snapshot['/scan']['ranges'] == [.30, 1.3]
    assert snapshot['/smart_vision_amr/path']['poses'] == [{'original': True}]
    # AMCL future-dates map->odom by transform_tolerance. Selection must use
    # reception time rather than discard the transform's future header.
    assert snapshot['transforms'][0]['header']['stamp']['sec'] == 12
    with pytest.raises(ValueError, match='active route'):
        replay.load_snapshot(incident, 12.)


def test_snapshot_rejects_missing_map_transform(tmp_path):
    incident = tmp_path / 'events.jsonl'
    incident.write_text('\n'.join(json.dumps(event(topic, data, 1.)) for topic, data in (
        ('/scan', {}), ('/odom', {}), ('/smart_vision_amr/path', {'poses': [{}]}))))
    with pytest.raises(ValueError, match='map -> odom'):
        replay.load_snapshot(incident, 1.)


def test_diagnostic_profile_preserves_controller_behavior_except_explicit_override(tmp_path):
    source = tmp_path / 'source.yaml'
    original = {
        'controller_server': {'ros__parameters': {
            'controller_frequency': 10., 'FollowPath': {
                'forward_prune_distance': .75, 'sim_time': 2., 'GoalAlign.scale': 24.}}},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'robot_radius': .20, 'inflation_layer': {'inflation_radius': .20}}}},
        'map_server': {'ros__parameters': {'yaml_filename': 'ignored.yaml'}},
    }
    source.write_text(yaml.safe_dump(original))
    diagnostics = replay.build_parameters(source, ['FollowPath.forward_prune_distance=0.45'])
    controller = diagnostics['controller_server']['ros__parameters']
    assert controller['FollowPath']['forward_prune_distance'] == .45
    assert controller['FollowPath']['sim_time'] == 2.
    assert controller['FollowPath']['GoalAlign.scale'] == 24.
    assert controller['FollowPath']['publish_evaluation'] is True
    assert controller['FollowPath']['short_circuit_trajectory_evaluation'] is False
    assert diagnostics['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] == .20
    assert 'map_server' not in diagnostics
    with pytest.raises(ValueError, match='overrides use'):
        replay.build_parameters(source, ['robot_radius=0.01'])


def trajectory(x, theta, total, scores):
    return {'traj': {'velocity': {'x': x, 'y': 0., 'theta': theta},
                     'poses': [{'x': 1., 'y': 2., 'theta': theta}]},
            'total': total, 'scores': scores}


def test_winner_and_forward_critic_scores_distinguish_cost_preference_from_collision():
    evaluation = {'best_index': 0, 'twists': [
        trajectory(0., -.27, 20.6, [{'name': 'GoalDist', 'raw_score': 15., 'scale': .6}]),
        trajectory(.05, -.27, 23., [{'name': 'GoalDist', 'raw_score': 17., 'scale': .6}]),
        trajectory(.12, 0., -1., [{'name': 'BaseObstacle', 'raw_score': -1., 'scale': .02}]),
        trajectory(.10, -.10, 24., [{'name': 'GoalDist', 'raw_score': 20., 'scale': .6}]),
    ]}
    summary = replay.summarize_evaluation(evaluation)
    assert summary['best']['velocity']['x'] == 0.
    assert summary['best_forward']['velocity']['x'] == .05
    assert summary['best_forward']['scores'][0]['raw_score'] == 17.
    assert summary['valid_count'] == 3
    assert summary['forward_valid_count'] == 2
    assert summary['invalid_by_critic'] == {'BaseObstacle': 1}


def test_no_valid_forward_trajectory_is_reported_explicitly():
    summary = replay.summarize_evaluation({'best_index': 0, 'twists': [
        trajectory(0., 0., 0., []),
        trajectory(.1, 0., -1., [{'name': 'Oscillation', 'raw_score': -1., 'scale': 1.}])]})
    assert summary['best_forward'] is None
    assert summary['forward_valid_count'] == 0
    assert summary['invalid_by_critic'] == {'Oscillation': 1}


def test_footprint_regression_accepts_nonlethal_inflation_cost_but_requires_body_rejection():
    spec = importlib.util.spec_from_file_location(
        'controller_regression', SCRIPT.with_name('run_controller_regression.py'))
    suite = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(suite)
    evaluation = {'invalid_by_critic': {'ObstacleFootprint': 8, 'BaseObstacle': 8},
                  'best': {'scores': [{'name': 'BaseObstacle', 'raw_score': 240.}]}}
    observations = [
        {'kind': 'command', 'data': {'linear': {'x': .1}, 'angular': {'z': 0.}}},
        {'kind': 'evaluation', 'data': evaluation}]
    assert suite.assess('footprint_obstacle', observations)['passed']
    evaluation['invalid_by_critic']['ObstacleFootprint'] = 0
    assert not suite.assess('footprint_obstacle', observations)['passed']
