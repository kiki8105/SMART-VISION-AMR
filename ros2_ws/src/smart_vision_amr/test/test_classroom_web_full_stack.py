"""The web-owned acceptance stages cannot pass on navigation-only evidence."""

from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))

import run_classroom_staged_validation as staged
from inject_classroom_web_obstacle import (
    candidate_still_ahead, near_obstacle_indices, path_clearance_from_box,
    preflight_box_route,
    select_feasible_web_obstacle)
from verify_classroom_web_dwell import assess_dwell
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.map_utils import cell_to_world, occupancy_to_grid, world_to_cell


def web_evidence(stage):
    route = [int(stop.removeprefix('Sector ')) for stop in stage.route
             if stop.startswith('Sector ')]
    return {
        'passed': True, 'domain': 176, 'localhost_only': True,
        'physical_robot_touched': False, 'route': route, 'final_route': route,
        'mission_accepted_via_web': True, 'final_state': 'HOME_REACHED',
        'completed_ids': route, 'startup_reset_requested_via_web': True,
        'startup_reset_completed': True,
        'home_stationary_verified': True, 'home_stationary_duration_s': 1.3,
        'home_pose_drift_m': .002,
        'home_independent_settle_check': {'passed': True},
        'mission_continuity': {'route_preserved': True,
                               'completed_prefix_valid': True,
                               'start_event_count': 1, 'cancel_event_count': 0,
                               'marker_confirmations': {str(i): 1 for i in route}},
        'web_runtime_manifest': {'mode': 'ekf_independent', 'record_incidents': True},
        'web_world_variant': 'stress' if '--stress' in stage.options else 'replay',
        'stress_crate_assessment': {'passed': True},
        'rear_camera_frames': {str(i): {'bytes': 1200, 'expected_visible': True,
                                      'center_error': .12, 'unannotated': True,
                                      'raw_camera_age_s': .1} for i in route},
        'front_camera_frames': {str(i): {'bytes': 1200, 'camera_age_s': .1,
                                       'width': 640, 'height': 480} for i in route},
        'dwell': {str(i): {'event_duration_s': 5.1, 'marker_confirmed': True,
                          'alignment_at_dwell': {'state': 'CONFIRMED',
                            'soft_center_result': 'centered', 'center_error': .12,
                            'rotation_degrees': 12.}}
                  for i in route},
        'web_ground_truth_checks': {key: {'passed': True} for key in
                                    (*map(str, route), 'home')},
        'web_independent_dwell_checks': {str(i): {'passed': True} for i in route},
    }


def test_new_web_stages_are_real_web_owned_route_targets():
    solo = staged.BY_KEY['03_web_sector3_clean']
    sector = staged.BY_KEY['06_web_sector3_moving_obstacle']
    clean = staged.BY_KEY['12_web_integrated_clean']
    obstacle = staged.BY_KEY['12_web_integrated_home_obstacle']
    stress_clean = staged.BY_KEY['12_web_integrated_stress_clean']
    stress_obstacle = staged.BY_KEY['12_web_integrated_stress_home_obstacle']
    assert sector.kind == obstacle.kind == 'web_moving_obstacle'
    assert solo.kind == clean.kind == 'web_dwell'
    assert staged.web_stage_route(solo) == '3'
    assert staged.web_stage_route(sector) == '3'
    assert staged.web_stage_route(clean) == '1,2,3'
    assert staged.web_stage_route(obstacle) == '1,2,3'
    assert stress_clean.options == stress_obstacle.options == ('--stress',)
    assert staged.web_stage_route(stress_clean) == '1,2,3'
    assert staged.web_stage_route(stress_obstacle) == '1,2,3'


def test_stress_web_stage_requires_actual_stress_fixture_manifest():
    stage = staged.BY_KEY['12_web_integrated_stress_clean']
    evidence = web_evidence(stage)
    assert staged.assess(stage, evidence)['passed']
    evidence['web_world_variant'] = 'replay'
    assert 'stress_world_runtime_unverified' in staged.assess(stage, evidence)['reasons']
    evidence['web_world_variant'] = 'stress'
    evidence['stress_crate_assessment']['passed'] = False
    assert 'unmapped_stress_crate_bypass_unverified' in staged.assess(stage, evidence)['reasons']


def test_web_clean_requires_ekf_reset_marker_center_and_camera_frame():
    stage = staged.BY_KEY['12_web_integrated_clean']
    evidence = web_evidence(stage)
    assert staged.assess(stage, evidence)['passed']
    evidence['startup_reset_completed'] = False
    evidence['web_runtime_manifest']['mode'] = 'baseline'
    evidence['dwell']['3']['alignment_at_dwell']['center_error'] = .6
    evidence['rear_camera_frames']['2']['bytes'] = 0
    evidence['front_camera_frames']['1']['camera_age_s'] = 2.
    evidence['dwell']['2']['alignment_at_dwell']['rotation_degrees'] = 168.
    reasons = staged.assess(stage, evidence)['reasons']
    assert 'physical_style_web_startup_reset_unverified' in reasons
    assert 'independent_ekf_runtime_unverified' in reasons
    assert 'sector_3_rear_camera_center_unverified' in reasons
    assert 'sector_2_rear_camera_frame_missing' in reasons
    assert 'sector_1_fresh_front_camera_unverified' in reasons
    assert 'sector_2_excessive_marker_search_rotation' in reasons


def test_home_obstacle_must_follow_sector3_dwell_and_home_ros_goal():
    stage = staged.BY_KEY['12_web_integrated_home_obstacle']
    evidence = web_evidence(stage)
    evidence['web_obstacle'] = {
        'passed': True, 'spawned': True, 'truth_seen': True, 'removed': True,
        'observed_while_live': True, 'observed_cleared': True,
        'path_reacted_before_removal': True, 'bypassed_while_live': True,
        'moving_steps': 4, 'minimum_clearance_m': .1,
        'events': [
            {'event': 'web_goal_observed', 'at': 11.},
            {'event': 'obstacle_spawned', 'at': 13.},
            {'event': 'obstacle_removed', 'at': 16.}],
    }
    evidence['dwell']['3']['end_event_at'] = 10.
    evidence['final_observed_at'] = 20.
    evidence['web_obstacle_overlay_checks'] = {
        'red_cells_while_live': True, 'red_cells_cleared_after_removal': True}
    evidence['home_obstacle_temporal_check'] = staged.check_home_obstacle_timing(evidence)
    assert evidence['home_obstacle_temporal_check']['passed']
    assert staged.assess(stage, evidence)['passed']
    evidence['web_obstacle']['events'][0]['at'] = 9.
    evidence['home_obstacle_temporal_check'] = staged.check_home_obstacle_timing(evidence)
    assert not evidence['home_obstacle_temporal_check']['passed']
    assert 'home_return_obstacle_timing_unverified' in staged.assess(stage, evidence)['reasons']


def test_web_obstacle_requires_injector_and_red_overlay_lifecycle():
    stage = staged.BY_KEY['06_web_sector3_moving_obstacle']
    evidence = web_evidence(stage)
    evidence['web_obstacle'] = {
        'passed': True, 'spawned': True, 'truth_seen': True, 'removed': True,
        'observed_while_live': True, 'observed_cleared': True,
        'path_reacted_before_removal': True, 'bypassed_while_live': True,
        'moving_steps': 4, 'minimum_clearance_m': .1}
    evidence['web_obstacle_overlay_checks'] = {
        'red_cells_while_live': True, 'red_cells_cleared_after_removal': True}
    assert staged.assess(stage, evidence)['passed']
    evidence['web_obstacle']['passed'] = False
    evidence['web_obstacle']['bypassed_while_live'] = False
    evidence['web_obstacle_overlay_checks']['red_cells_cleared_after_removal'] = False
    reasons = staged.assess(stage, evidence)['reasons']
    assert 'web_obstacle_injector_failed' in reasons
    assert 'web_obstacle_bypassed_while_live_unverified' in reasons
    assert 'web_red_obstacle_clearance_unverified' in reasons


def test_web_overlay_must_correlate_owned_lidar_cells_and_removal():
    evidence = {
        'web_obstacle': {'observed_indices_live': [10], 'events': [
            {'event': 'obstacle_spawned', 'at': 2.},
            {'event': 'obstacle_removed', 'at': 16.}]},
        'observed_obstacle_overlay_trace': [
            {'at': 1., 'cells': [], 'renderable': True},
            {'at': 3., 'cells': [10], 'renderable': True},
            {'at': 19., 'cells': [], 'renderable': True}]}
    checks = staged.check_web_obstacle_overlay(evidence)
    assert checks['red_cells_while_live']
    assert checks['red_cells_cleared_after_removal']
    evidence['observed_obstacle_overlay_trace'][1]['cells'] = [99]
    assert not staged.check_web_obstacle_overlay(evidence)['red_cells_while_live']
    evidence['observed_obstacle_overlay_trace'][1]['cells'] = [10]
    evidence['observed_obstacle_overlay_trace'][1]['renderable'] = False
    assert not staged.check_web_obstacle_overlay(evidence)['red_cells_while_live']


def test_near_obstacle_cells_excludes_preexisting_wall_echoes():
    class Info:
        width = 10
        height = 10
    class Grid:
        info = Info()
    convert = lambda info, cell: (cell[0]*.05, cell[1]*.05)
    assert near_obstacle_indices([0, 1, 50], [0], Grid(), (0., 0.), convert) == [1, 50]


def test_changed_path_needs_conservative_clearance_from_box():
    assert path_clearance_from_box([(0., 0.), (1., 0.)], (.5, 0.)) < 0.
    assert path_clearance_from_box([(0., 1.), (1., 1.)], (.5, 0.)) > .02


def test_moving_box_spawn_rejects_robot_that_reached_selected_candidate():
    candidate = {'x': 0., 'y': 0., 'path_tangent': [1., 0.]}
    path = [(-1., 0.), (0., 0.), (1., 0.)]
    assert candidate_still_ahead(candidate, (-.75, 0.), path)
    assert not candidate_still_ahead(candidate, (-.08, .10), path)
    assert not candidate_still_ahead(candidate, (.10, 0.), path)
    assert not candidate_still_ahead(candidate, (-.75, 0.), [(-1., 1.), (1., 1.)])


def test_classroom_moving_box_preserves_inflated_route_at_all_five_poses():
    image = cv2.imread(str(ROOT/'ros2_ws/src/smart_vision_amr/maps/'
                            'classroom_20261030_cleaned.pgm'), 0)
    height, width = image.shape
    info = SimpleNamespace(width=width, height=height, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=-1.65, y=-5.25),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    data = np.flipud(np.where(image == 0, 100,
                              np.where(image == 254, 0, -1))).ravel().tolist()
    grid = SimpleNamespace(info=info, data=data)
    robot = (-.071, -.265)
    goal = (-.25, -1.98)
    cells = AStarPlanner().plan(occupancy_to_grid(grid, 65, 5),
                                world_to_cell(info, -.175, -.225),
                                world_to_cell(info, *goal))
    path = [cell_to_world(info, cell) for cell in cells]
    assert preflight_box_route(grid, robot, goal, (.375, -.875), path,
                               world_to_cell, cell_to_world) is None
    candidate = select_feasible_web_obstacle(path, robot, goal, grid,
                                              world_to_cell, cell_to_world)
    assert candidate is not None
    assert candidate['preflight']['route_cells_by_step'] == [
        length for length in candidate['preflight']['route_cells_by_step'] if length > 0]
    assert len(candidate['preflight']['route_cells_by_step']) == 5
    assert candidate['preflight']['initial_centerline_clearance_m'] <= .05
    assert candidate['static_wall_distance_m'] > .16


def test_dwell_verifier_rejects_off_center_recognition():
    stage = staged.BY_KEY['06_web_sector3_moving_obstacle']
    evidence = web_evidence(stage)
    evidence.update(configured_dwell_seconds=5., final_home_distance_m=.02,
                    final_speed=[0., 0.], path_points_per_sector={'3': 4},
                    home_path_points=10)
    evidence['dwell']['3'].update(observed_dwelling=True, sample_count=25,
        next_goal_after_dwell=True, max_linear_mps=0., max_angular_rps=0.,
        stale_odom_samples=0, max_pose_delta_m=0.)
    assert assess_dwell(evidence) == []
    evidence['dwell']['3']['alignment_at_dwell']['soft_center_result'] = 'out_of_band'
    assert 'sector_3_camera_centering_unverified' in assess_dwell(evidence)
