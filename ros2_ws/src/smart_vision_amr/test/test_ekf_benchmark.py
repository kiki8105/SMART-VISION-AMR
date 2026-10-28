"""Guard the coordinate alignment and mode-selection rules used by the trial."""

import importlib.util
import math
from pathlib import Path

from smart_vision_amr.ekf_benchmark_math import (
    Pose2, align_pose, errors, interpolate_pose, summarize_error_rows, wrap_angle,
)


REPO = Path(__file__).resolve().parents[4]


def load_script(name):
    path = REPO / 'scripts' / name
    spec = importlib.util.spec_from_file_location(name.removesuffix('.py'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_align_odom_frame_with_different_origin_and_heading():
    initial_truth = Pose2(-3, -2, math.pi / 2)
    initial_odom = Pose2(10, 20, 0)
    # +1 m in odom x points along world +y after alignment.
    moved = align_pose(Pose2(11, 20, .2), initial_odom, initial_truth)
    assert math.isclose(moved.x, -3, abs_tol=1e-9)
    assert math.isclose(moved.y, -1, abs_tol=1e-9)
    assert math.isclose(moved.yaw, math.pi / 2 + .2, abs_tol=1e-9)
    assert errors(moved, Pose2(-3, -1, math.pi / 2 + .2)) == (0.0, 0.0)


def test_angle_wrap_and_rmse_use_shortest_heading_difference():
    assert math.isclose(wrap_angle(-math.pi + .1 - (math.pi - .1)), .2)
    position, heading = errors(Pose2(1, 0, -math.pi + .1),
                               Pose2(0, 0, math.pi - .1))
    assert math.isclose(position, 1.0)
    assert math.isclose(heading, .2)
    metrics = summarize_error_rows([
        {'position_error_m': 0, 'heading_error_rad': 0},
        {'position_error_m': 1, 'heading_error_rad': math.pi / 2},
    ])
    assert math.isclose(metrics['position_rmse_m'], 1 / math.sqrt(2))
    assert math.isclose(metrics['heading_rmse_deg'], 90 / math.sqrt(2))


def test_interpolation_uses_stamps_and_never_extrapolates():
    samples = [(1_000_000_000, Pose2(0, 0, math.pi - .1)),
               (1_100_000_000, Pose2(1, 0, -math.pi + .1))]
    result = interpolate_pose(samples, 1_050_000_000)
    assert result is not None
    pose, side_ns, span_ns = result
    assert math.isclose(pose.x, .5)
    assert math.isclose(abs(pose.yaw), math.pi)
    assert side_ns == 50_000_000 and span_ns == 100_000_000
    assert interpolate_pose(samples, 900_000_000) is None
    assert interpolate_pose(samples, 1_200_000_000) is None


def trial(mode, position_rmse=.1, heading_rmse=5, status='COMPLETE', tf_gap=.1):
    return {
        'mode': mode, 'trial_status': status,
        'route_completion_fraction': 1 if status == 'COMPLETE' else .5,
        'position': {'sample_count': 300, 'position_rmse_m': position_rmse,
                     'heading_rmse_deg': heading_rmse},
        'max_interpolation_side_s': .02, 'max_interpolation_span_s': .04,
        'tf_odom_base_count': 300, 'tf_max_gap_s': tf_gap,
        'tf_odom_base_rate_hz': 30, 'graph_authority_ok': True,
        'tf_conflict_count': 0, 'tf_stale_sample_count': 0,
        'matched_sample_rate_hz': 8, 'matched_sample_fraction': .8,
        'last_matched_age_s': .1, 'truth_distance_m': 2.2,
        'sim_duration_s': 38, 'wall_duration_s': 42,
        'sim_realtime_factor': .9, 'estimate_header_lag_p95_s': .08,
        'failure': None if status == 'COMPLETE' else 'route timed out',
    }


def test_incomplete_trials_never_win_and_overlay_is_diagnostic():
    compare = load_script('compare_ekf_benchmarks.py')
    records = ([trial('baseline') for _ in range(3)]
               + [trial('ekf_overlay', .01, 1) for _ in range(3)]
               + [trial('ekf_independent', .04, 5, 'FAILED') for _ in range(3)])
    summary = compare.summarize(records)
    assert summary['simulation_recommendation'] == 'inconclusive'
    assert summary['modes']['ekf_independent']['valid_trial_count'] == 0


def test_independent_wins_only_with_real_improvement_and_sound_tf():
    compare = load_script('compare_ekf_benchmarks.py')
    records = ([trial('baseline', .1, 5) for _ in range(3)]
               + [trial('ekf_overlay', .01, 1) for _ in range(3)]
               + [trial('ekf_independent', .07, 5.2) for _ in range(3)])
    assert compare.summarize(records)['simulation_recommendation'] == 'ekf_independent'
    records[-1]['tf_max_gap_s'] = .8
    assert compare.summarize(records)['simulation_recommendation'] == 'inconclusive'


def test_sparse_matching_cannot_count_as_valid_completed_route():
    compare = load_script('compare_ekf_benchmarks.py')
    item = trial('ekf_independent')
    item['position']['sample_count'] = 4
    item['matched_sample_rate_hz'] = .1
    item['matched_sample_fraction'] = .02
    assert not compare.valid_trial(item)


def test_ground_truth_plugin_only_in_generated_test_world(tmp_path):
    make = load_script('make_ekf_benchmark_world.py')
    source = REPO / 'ros2_ws/src/smart_vision_amr/worlds/simulation_room.world'
    output = make.make_world(source)
    assert 'libgazebo_ros_state.so' in output
    assert '<update_rate>20.0</update_rate>' in output
    assert output.count('</world>') == 1
    assert 'libgazebo_ros_state.so' not in source.read_text()
