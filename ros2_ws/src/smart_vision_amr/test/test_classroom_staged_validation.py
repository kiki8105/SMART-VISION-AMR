"""Staged acceptance must not turn an unobserved route or unsafe stop green."""

import hashlib
import sys
import subprocess
from math import pi
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import assess_classroom_safety_timeline as timeline
import capture_classroom_safety_timeline as capture
import run_classroom_staged_validation as staged


def test_final_physical_value_parity_allows_only_sim_clock_override(tmp_path, monkeypatch):
    monkeypatch.setattr(staged, 'REPO', tmp_path)
    config = tmp_path/'ros2_ws/src/smart_vision_amr/config'
    config.mkdir(parents=True)
    nav = config/'curve_tuning_waffle_pi.yaml'
    nodes = config/'classroom_hardware_nodes.yaml'
    ekf = config/'ekf_hardware.yaml'
    generated = tmp_path/'classroom_nav_sim_time.yaml'
    nav.write_text('controller_server:\n  ros__parameters:\n'
                   '    use_sim_time: false\n    max_vel_x: 0.12\n')
    generated.write_text('controller_server:\n  ros__parameters:\n'
                         '    use_sim_time: true\n    max_vel_x: 0.12\n')
    nodes.write_text('safe_distance: 0.3\n')
    ekf.write_text('frequency: 30\n')
    manifest = {
        'nav_profile_source': str(nav),
        'config_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (nav, nodes, ekf, generated)},
    }
    assert staged.check_physical_source_value_parity(tmp_path, manifest)['passed']
    generated.write_text(generated.read_text().replace('0.12', '0.13'))
    checks = staged.check_physical_source_value_parity(tmp_path, manifest)['checks']
    assert not checks['nav_values_equal_except_sim_clock']
    assert not checks['generated_nav_hash_matches_runtime']


def test_final_physical_value_stage_is_web_owned_and_stress_world():
    stage = staged.BY_KEY['17_web_hardware_value_parity']
    assert stage.key in staged.WEB_FULL_STACK_STAGES
    assert stage.kind == 'web_dwell' and '--stress' in stage.options
    assert stage.route == ('Sector 1', 'Sector 2', 'Sector 3', 'Home')


def test_memory_stages_keep_filter_off_and_require_runtime_memory():
    for key, route, kind in (
            ('21_web_memory_sector2', ('Sector 2', 'Home'), 'web_dwell'),
            ('22_web_memory_integrated_clean',
             ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
            ('23_web_memory_integrated_obstacle',
             ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_moving_obstacle'),
            ('24_web_memory_sector1_to_sector2_removal',
             ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell')):
        stage = staged.BY_KEY[key]
        assert stage.route == route and stage.kind == kind
        assert stage.options == ('--memory-session',)
        assert stage.key in staged.WEB_FULL_STACK_STAGES
    hashes = staged.web_runtime_source_hashes(
        staged.BY_KEY['24_web_memory_sector1_to_sector2_removal'])
    assert 'scripts/inject_classroom_memory_transition.py' in hashes


def test_sector1_to_sector2_memory_stage_rejects_missing_transition():
    stage = staged.BY_KEY['24_web_memory_sector1_to_sector2_removal']
    evidence = {
        'passed': True, 'domain': 176, 'localhost_only': True,
        'physical_robot_touched': False,
        'web_runtime_manifest': {
            'mode': 'ekf_independent', 'record_incidents': True,
            'memory_session': True, 'laser_filter_for_change_memory': False},
    }
    assert 'sector1_to_sector2_memory_transition_failed' in staged.assess(
        stage, evidence)['reasons']
    evidence['memory_transition'] = {
        'passed': True, 'minimum_clearance_m': .04,
        'after_removal': 'CONTINUED_EXISTING_PATH',
        **{key: True for key in (
            'spawned', 'truth_seen', 'observed_on_sector1',
            'confirmed_on_sector1', 'sector2_planned_with_memory',
            'sector2_initial_plan_without_live_hit', 'removed_on_sector2',
            'free_ray_evidence_after_removal', 'memory_cleared')},
    }
    assert not any('memory_transition' in reason for reason in staged.assess(
        stage, evidence)['reasons'])
    evidence['web_runtime_manifest']['memory_session'] = False
    assert 'runtime_memory_not_enabled' in staged.assess(stage, evidence)['reasons']


def test_full_web_trial_fingerprints_loaded_runtime_sources():
    hashes = staged.web_runtime_source_hashes(
        staged.BY_KEY['12_web_integrated_stress_clean'])
    required = {
        'scripts/run_classroom_staged_validation.py',
        'scripts/verify_classroom_web_dwell.py',
        'ros2_ws/src/smart_vision_amr/smart_vision_amr/continuous_start_egress.py',
        'ros2_ws/src/smart_vision_amr/smart_vision_amr/home_alignment.py',
        'ros2_ws/src/smart_vision_amr/smart_vision_amr/mission_logic.py',
        'ros2_ws/src/smart_vision_amr/web/app.js',
        'ros2_ws/src/smart_vision_amr/web/index.html',
        'ros2_ws/src/smart_vision_amr/config/aruco_classroom_markers_20261030.yaml',
        'ros2_ws/src/smart_vision_amr/config/localization_health.yaml',
        'ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.pgm',
        'ros2_ws/src/smart_vision_amr/worlds/classroom_20261030_stress.world',
        'gazebo_models/smart_vision_aruco_4x4_50_id_3/meshes/marker.obj',
    }
    assert required <= hashes.keys()
    assert not any('/test/' in path or path.startswith('docs/') for path in hashes)
    assert staged.web_runtime_source_hashes(staged.BY_KEY['01_sector_1']) == {}


def test_runtime_fingerprint_detects_content_addition_and_deletion(tmp_path, monkeypatch):
    monkeypatch.setattr(staged, 'REPO', tmp_path)
    package = tmp_path/'ros2_ws/src/smart_vision_amr/smart_vision_amr'
    package.mkdir(parents=True)
    first = package/'astar_planner.py'
    first.write_text('old')
    stage = staged.BY_KEY['12_web_integrated_stress_clean']
    baseline = staged.web_runtime_source_hashes(stage)
    first.write_text('changed')
    assert staged.web_runtime_source_hashes(stage) != baseline
    first.write_text('old')
    added = package/'continuous_start_egress.py'
    added.write_text('new')
    assert staged.web_runtime_source_hashes(stage) != baseline
    added.unlink()
    first.unlink()
    assert staged.web_runtime_source_hashes(stage) != baseline


def route_evidence(stage):
    legs = []
    for stop in stage.route:
        legs.append({'stop': stop, 'terminal': 'SUCCEEDED', 'passed': True,
                     'position_error_m': .03, 'heading_error_rad': .12,
                     'paths_published': 1, 'motion_commands': 20,
                     'marker_alignment': {'marker_visible': True}})
    return {'passed': True, 'domain': 176, 'localhost_only': True,
            'physical_robot_touched': False,
            'fault_delivery': {'passed': True},
            'route': list(stage.route), 'legs': legs}


def base_timeline():
    return [
        {'t': 0., 'kind': 'planner', 'status': 'READY'},
        {'t': .01, 'kind': 'path', 'count': 5},
        {'t': .02, 'kind': 'scan', 'front_min_m': .5,
         'front_core_min_m': .5, 'front_core_angle_deg': 0.},
        {'t': .03, 'kind': 'truth', 'x': 0., 'y': 0., 'yaw': 0.},
        {'t': .04, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.},
    ]


def test_route_requires_each_real_path_and_marker():
    stage = staged.BY_KEY['04_sectors_1_2']
    evidence = route_evidence(stage)
    assert staged.assess(stage, evidence)['passed']
    evidence['legs'][1]['paths_published'] = 0
    evidence['legs'][1]['marker_alignment']['marker_visible'] = False
    result = staged.assess(stage, evidence)
    assert not result['passed']
    assert 'Sector 2:no_nonempty_path' in result['reasons']
    assert 'Sector 2:rear_marker_unverified' in result['reasons']


def test_integrated_clean_route_requires_home_and_no_obstacle_fixture():
    stage = staged.BY_KEY['12_integrated_clean']
    assert stage.options == ('--mission-route',)
    evidence = route_evidence(stage)
    evidence.update(home_obstacle_requested=False, route_obstacle_requested=False,
                    home_obstacle=None, route_obstacle=None)
    assert staged.assess(stage, evidence)['passed']

    evidence['home_obstacle_requested'] = True
    result = staged.assess(stage, evidence)
    assert not result['passed']
    assert 'clean_route_obstacle_fixture_present' in result['reasons']

    evidence['home_obstacle_requested'] = False
    evidence['legs'][3]['terminal'] = 'PERSISTENT_OBSTACLE_HOLD'
    assert not staged.assess(stage, evidence)['passed']


def test_obstacle_hold_timeout_reaches_route_runner_and_rejects_bad_input(tmp_path):
    stage = staged.BY_KEY['12_integrated_clean']
    command = staged.stage_runner_command(stage, tmp_path/'trial', 600, 300)
    assert command[command.index('--obstacle-hold-timeout') + 1] == '300'
    assert '--mission-route' in command
    assert '--home-obstacle' not in command

    invalid = subprocess.run([str(staged.RUNNER), '--obstacle-hold-timeout',
                              'not-a-number'], capture_output=True, text=True)
    assert invalid.returncode == 2
    assert '--obstacle-hold-timeout must be an integer' in invalid.stderr


def test_sensor_input_hold_is_not_reported_as_obstacle_hold():
    stage = staged.BY_KEY['03_sector_3']
    evidence = route_evidence(stage)
    evidence['passed'] = False
    leg = evidence['legs'][0]
    leg['passed'] = False
    leg['terminal'] = 'PERSISTENT_OBSTACLE_HOLD'
    leg['failure_memory_status'] = {'state': 'SENSOR_HOLD'}
    leg['last_navigation'] = {'input_hold': True}
    outcome = staged.assess(stage, evidence)
    assert not outcome['passed']
    assert 'Sector3:SENSOR_HOLD' in outcome['reasons']
    assert outcome['legs'][0]['raw_terminal'] == 'PERSISTENT_OBSTACLE_HOLD'
    assert outcome['legs'][0]['terminal'] == 'SENSOR_HOLD'


def test_single_obstacle_cannot_pass_without_reaction():
    stage = staged.BY_KEY['06_single_unmapped_obstacle']
    evidence = {'passed': True, 'domain': 176, 'localhost_only': True,
                'physical_robot_touched': False,
                'fault_delivery': {'passed': True},
                'paths': [{'count': 10}],
                'commands': [{'linear': .1, 'angular': 0.}],
                'metrics': {'terminal': 'SUCCEEDED', 'actual_goal_within_35cm': True,
                            'obstacle_observed': False,
                            'route_reacted_after_obstacle': False,
                            'obstacle_observation_cleared': False,
                            'obstacle_removed': True,
                            'obstacle_motion_steps': 4,
                            'minimum_obstacle_clearance_m': .1}}
    result = staged.assess(stage, evidence)
    assert not result['passed']
    assert 'obstacle_observed_not_proven' in result['reasons']


def test_sustained_sensor_loss_is_safe_hold_not_delivery_success():
    stage = staged.BY_KEY['10_sensor_dds_outage']
    evidence = {'passed': True, 'domain': 176, 'localhost_only': True,
                'physical_robot_touched': False,
                'fault_delivery': {'passed': True, 'counts': {
                    f'{name}_dropped_outage': 10 for name in ('scan', 'odom', 'tf')}},
                'paths': [{'count': 15}], 'commands': [{'linear': .08}],
                'goal': [-.25, -1.98],
                'health': [{'autonomous_allowed': False}],
                'metrics': {'terminal': 'SAFE_HOLD', 'safe_hold_proven': True,
                            'safety_block_events': 1,
                            'outage_health_stop_latched': True,
                            'outage_mux_estop_latched': True,
                            'outage_goal_retained': True,
                            'outage_manual_reset_required': True,
                            'final_planner': {'goal': {'x': -.25, 'y': -1.98}}}}
    outcome = staged.assess(stage, evidence)
    assert outcome['passed'] and outcome['outcome'] == 'SAFE_HOLD'
    evidence['metrics']['outage_goal_retained'] = False
    assert not staged.assess(stage, evidence)['passed']


def test_wan_stage_needs_proof_of_no_default_route_and_loopback():
    stage = staged.BY_KEY['09_wan_absent']
    evidence = route_evidence(stage)
    assert 'wan_isolation_not_proven' in staged.assess(stage, evidence)['reasons']
    evidence['staged_network_namespace'] = {'no_default_route': True,
                                             'loopback_up': True}
    assert staged.assess(stage, evidence)['passed']


def test_wait_replan_allows_bounded_inertia_and_logs_stopping_margin():
    records = base_timeline()+[
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .11, 'kind': 'path', 'count': 0},
        {'t': .15, 'kind': 'cmd_vel', 'linear': .10, 'angular': 0.},
        {'t': .20, 'kind': 'truth', 'x': .012, 'y': 0., 'yaw': 0.},
        {'t': .24, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .36, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .40, 'kind': 'truth', 'x': .03, 'y': 0., 'yaw': 0.},
        {'t': .50, 'kind': 'planner', 'status': 'READY'},
    ]
    result = timeline.assess(records)
    assert result['passed'], result['violations']
    interval = result['wait_replan_intervals'][0]
    assert interval['last_nonzero_command_after_s'] < .25
    assert interval['estimated_front_body_clearance_m'] > .02


def test_wait_replan_near_wall_fails_even_when_command_stops_quickly():
    records = base_timeline()
    records[2]['front_core_min_m'] = .134
    records += [
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .15, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.},
        {'t': .20, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': 0.},
        {'t': .24, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .4, 'kind': 'truth', 'x': .03, 'y': 0., 'yaw': 0.},
        {'t': .5, 'kind': 'planner', 'status': 'READY'},
    ]
    result = timeline.assess(records)
    assert not result['passed']
    assert any(v['kind'] == 'insufficient_front_stopping_clearance'
               for v in result['violations'])


def test_planning_with_prior_path_and_clear_front_is_not_a_stop_violation():
    records = base_timeline()+[
        {'t': .10, 'kind': 'planner', 'status': 'PLANNING'},
        {'t': .40, 'kind': 'scan', 'front_min_m': .45,
         'front_core_min_m': .45, 'front_core_angle_deg': 0.},
        {'t': .45, 'kind': 'cmd_vel', 'linear': .08, 'angular': .02},
        {'t': .50, 'kind': 'planner', 'status': 'READY'},
    ]
    result = timeline.assess(records)
    assert result['passed'], result['violations']
    assert result['counts']['planning_active_front_clear'] == 1


def test_missing_timeline_is_not_an_automatic_pass():
    result = timeline.assess([])
    assert not result['passed']
    assert 'nonempty_path' in result['missing_evidence']


def test_wait_replan_requires_zero_command_by_deadline():
    records = base_timeline()+[
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .18, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.},
        {'t': .28, 'kind': 'truth', 'x': .015, 'y': 0., 'yaw': 0.},
        {'t': .38, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .40, 'kind': 'planner', 'status': 'READY'},
    ]
    result = timeline.assess(records)
    assert not result['passed']
    assert any(v['kind'] == 'zero_command_not_observed_in_time'
               for v in result['violations'])


def test_recovery_does_not_erase_unsafe_preceding_hold():
    records = base_timeline()+[
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .20, 'kind': 'cmd_vel', 'linear': .1, 'angular': 0.},
        {'t': .30, 'kind': 'truth', 'x': .06, 'y': 0., 'yaw': 0.},
        {'t': .32, 'kind': 'navigation', 'status': 'RECOVERING'},
        {'t': .35, 'kind': 'cmd_vel', 'linear': -.04, 'angular': 0.},
        {'t': .42, 'kind': 'planner', 'status': 'RECOVERING'},
    ]
    result = timeline.assess(records)
    assert not result['passed']
    assert result['recovery_transitions'][0]['source'] == 'navigation'
    assert any(v['kind'] == 'excessive_ground_truth_stopping_distance'
               for v in result['violations'])


def test_web_disconnect_requires_actual_http_mission_and_motion():
    stage = staged.BY_KEY['08_browser_client_disconnect']
    evidence = {'passed': True, 'domain': 176, 'localhost_only': True,
                'physical_robot_touched': False, 'web_client_disconnected': True,
                'mission_accepted_via_web': False, 'mission_route_after_disconnect': [3],
                'state_after_disconnect': 'NAVIGATING', 'displacement_m': .1}
    assert 'web_mission_not_accepted' in staged.assess(stage, evidence)['reasons']
    evidence['mission_accepted_via_web'] = True
    evidence['web_disconnect_truth'] = {'passed': True, 'displacement_m': .1}
    assert staged.assess(stage, evidence)['passed']


def test_lidar_records_the_bearing_of_the_closest_front_beam():
    scan = SimpleNamespace(ranges=[.20, .13, .18, .55, .70],
                           angle_min=-pi/6, angle_increment=pi/12,
                           range_min=.02, range_max=3.5)
    value, bearing = capture.finite_sector_min(scan, 0., pi/6)
    assert value == .13
    assert abs(bearing+15.) < 1e-9
    core, core_bearing = capture.finite_sector_min(scan, 0., pi/12)
    assert core == .13
    assert abs(core_bearing+15.) < 1e-9


def test_near_lidar_snapshot_preserves_adjacent_invalid_and_valid_beams():
    values = [float('inf'), .88, .124, .126, float('nan'), .91]
    snapshot = capture.scan_neighborhood(values, 2)
    assert [item['index'] for item in snapshot] == [0, 1, 2, 3, 4, 5]
    assert [item['range_m'] for item in snapshot] == [
        'inf', .88, .124, .126, 'nan', .91]


def test_known_static_world_proves_isolated_min_range_return_is_not_a_collision():
    # The local Gazebo fixture is fully known; the 0.13 m frame is absent
    # from both adjacent scans and impossible against its collision box.
    records = base_timeline()+[
        {'t': .08, 'kind': 'scan', 'front_min_m': .13,
         'front_core_min_m': .13, 'front_core_angle_deg': 0.},
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .15, 'kind': 'cmd_vel', 'linear': .10, 'angular': .5},
        {'t': .20, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': .08},
        {'t': .24, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .27, 'kind': 'scan', 'front_min_m': .8,
         'front_core_min_m': .8, 'front_core_angle_deg': 0.},
        {'t': .40, 'kind': 'truth', 'x': .03, 'y': 0., 'yaw': .2},
        {'t': .50, 'kind': 'planner', 'status': 'READY'},
    ]
    far_box = [('known_wall', [(1.2, -.5), (1.25, -.5),
                               (1.25, .5), (1.2, .5)])]
    raw = timeline.assess(records)
    assert not raw['passed']
    verified = timeline.assess(records, static_world_boxes=far_box)
    assert verified['passed'], verified['violations']
    interval = verified['wait_replan_intervals'][0]
    assert interval['static_world_swept_clearance_m'] > .05
    assert interval['rotation_verified_by_static_world_sweep']
    assert interval['isolated_static_world_scan_outlier']['observed_m'] == .13
    assert interval['isolated_static_world_scan_outlier'][
        'static_world_expected_m'] > 1.


def test_repeated_close_returns_stay_unsafe_even_in_static_fixture():
    records = base_timeline()+[
        {'t': .08, 'kind': 'scan', 'front_min_m': .13,
         'front_core_min_m': .13, 'front_core_angle_deg': 0.},
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .15, 'kind': 'cmd_vel', 'linear': .10, 'angular': 0.},
        {'t': .20, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': 0.},
        {'t': .27, 'kind': 'scan', 'front_min_m': .14,
         'front_core_min_m': .14, 'front_core_angle_deg': 0.},
        {'t': .28, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .40, 'kind': 'truth', 'x': .03, 'y': 0., 'yaw': 0.},
        {'t': .50, 'kind': 'planner', 'status': 'READY'},
    ]
    far_box = [('known_wall', [(1.2, -.5), (1.25, -.5),
                               (1.25, .5), (1.2, .5)])]
    result = timeline.assess(records, static_world_boxes=far_box)
    assert not result['passed']
    assert any(item['kind'] == 'insufficient_front_stopping_clearance'
               for item in result['violations'])


def test_static_sweep_rejects_actual_nearby_collision_even_if_scan_outlier():
    records = base_timeline()+[
        {'t': .08, 'kind': 'scan', 'front_min_m': .13,
         'front_core_min_m': .13, 'front_core_angle_deg': 0.},
        {'t': .10, 'kind': 'planner', 'status': 'WAIT_REPLAN'},
        {'t': .16, 'kind': 'cmd_vel', 'linear': 0., 'angular': 0.},
        {'t': .20, 'kind': 'truth', 'x': .02, 'y': 0., 'yaw': 0.},
        {'t': .28, 'kind': 'scan', 'front_min_m': .8,
         'front_core_min_m': .8, 'front_core_angle_deg': 0.},
        {'t': .40, 'kind': 'truth', 'x': .03, 'y': 0., 'yaw': 0.},
        {'t': .50, 'kind': 'planner', 'status': 'READY'},
    ]
    near_box = [('near_wall', [(.28, -.2), (.33, -.2),
                              (.33, .2), (.28, .2)])]
    result = timeline.assess(records, static_world_boxes=near_box)
    assert not result['passed']
    assert any(item['kind'] == 'insufficient_static_world_swept_clearance'
               for item in result['violations'])


def test_world_loader_uses_collision_boxes_not_visual_geometry(tmp_path):
    world = tmp_path/'world.sdf'
    world.write_text('''<sdf version="1.6"><world name="test">
      <model name="wall"><static>true</static>
        <link name="body"><pose>1 0 .3 0 0 0</pose>
          <collision name="solid"><geometry><box>
            <size>.1 .4 .6</size></box></geometry></collision>
          <visual name="paint"><geometry><box>
            <size>3 3 .1</size></box></geometry></visual>
        </link></model></world></sdf>''')
    boxes = timeline.load_static_world_boxes(world)
    assert len(boxes) == 1
    assert boxes[0][0] == 'wall/body'
    assert timeline.static_sweep_clearance(
        [{'x': 0., 'y': 0.}], boxes) > .7


def dwell_fixture():
    records = []
    for first, last in ((10., 15.1), (25., 30.1)):
        for index in range(52):
            at = round(first+index*.1, 3)
            records.append({'kind': 'truth', 'wall_time': at,
                            'x': 1., 'y': 2., 'yaw': 0.})
            records.append({'kind': 'cmd_vel', 'wall_time': at,
                            'linear': 0., 'angular': 0.})
    records.extend([
        {'kind': 'goal', 'wall_time': 15.2, 'x': 2., 'y': 3., 'yaw': .5},
        {'kind': 'goal', 'wall_time': 30.2, 'x': 0., 'y': 0., 'yaw': 0.},
    ])
    evidence = {'route': [1, 2],
                'dwell': {'1': {'start_event_at': 10., 'end_event_at': 15.1},
                          '2': {'start_event_at': 25., 'end_event_at': 30.1}},
                'marker_targets': {'2': [2., 3., .5]},
                'home_target': {'x': 0., 'y': 0., 'yaw': 0.}}
    return records, evidence


def test_web_dwell_requires_continuous_gazebo_stillness_and_later_ros_goal():
    records, evidence = dwell_fixture()
    checks = staged.check_web_dwell_independent(records, evidence)
    assert checks['1']['passed'], checks['1']
    assert checks['2']['passed'], checks['2']
    assert checks['1']['truth_samples'] > 20
    assert checks['1']['next_ros_goal_delay_s'] > 0.


def test_web_dwell_fails_if_robot_drifts_or_goal_publishes_early():
    records, evidence = dwell_fixture()
    drifting = [dict(item) for item in records]
    for item in drifting:
        if item['kind'] == 'truth' and 12. <= item['wall_time'] <= 12.5:
            item['x'] += .06
    checks = staged.check_web_dwell_independent(drifting, evidence)
    assert not checks['1']['passed']
    assert checks['1']['reason'] == 'robot_or_command_moved_during_dwell'
    premature = [dict(item) for item in records]
    premature.append({'kind': 'goal', 'wall_time': 14.9,
                      'x': 2., 'y': 3., 'yaw': .5})
    checks = staged.check_web_dwell_independent(premature, evidence)
    assert checks['1']['reason'] == 'next_goal_published_before_dwell_end'


def test_web_dwell_fails_when_independent_sampling_has_a_gap():
    records, evidence = dwell_fixture()
    records = [item for item in records if not
               (11. <= item['wall_time'] <= 12.)]
    checks = staged.check_web_dwell_independent(records, evidence)
    assert not checks['1']['passed']
    assert checks['1']['reason'] == 'continuous_observation_missing'


def test_web_transient_needs_real_relay_fault_and_same_web_goal(tmp_path):
    records = [{'kind': 'goal', 'monotonic': 95., 'wall_time': 10.,
                'x': 1., 'y': 1., 'yaw': 0.}]
    for index in range(60):
        at = 100.1+index*.1
        held = 101.8 <= at < 102.4
        distance = (.1+(at-100.1)*.08 if at < 101.8 else
                    .236 if held else .236+(at-102.4)*.08)
        records.extend([
            {'kind': 'truth', 'monotonic': at, 'x': distance,
             'y': 0., 'yaw': 0.},
            {'kind': 'cmd_vel', 'monotonic': at,
             'linear': 0. if held else .08, 'angular': 0.},
            {'kind': 'health', 'monotonic': at,
             'autonomous_allowed': not held, 'stop_latched': False},
        ])
    records.extend([
        {'kind': 'navigation', 'monotonic': 101.9,
         'status': 'WAIT_REPLAN', 'input_hold': True,
         'goal_stamp': {'sec': 3, 'nanosec': 8}},
        {'kind': 'navigation', 'monotonic': 102.5,
         'status': 'CANCELED', 'manual_stop': False,
         'goal_stamp': {'sec': 3, 'nanosec': 8}},
        {'kind': 'navigation', 'monotonic': 102.7,
         'status': 'MOVING', 'goal_stamp': {'sec': 3, 'nanosec': 8}},
    ])
    relay = tmp_path/'sensor_faults.jsonl'
    relay.write_text('\n'.join(map(__import__('json').dumps, [
        {'event': 'sample', 'monotonic': 99., 'counts': {},
         'max_delivery_delay_s': {'scan': .01, 'odom': .01, 'tf': .01}},
        {'event': 'profile', 'name': 'transient', 'monotonic': 100.05},
        {'event': 'sample', 'monotonic': 105.,
         'counts': {'scan_dropped_transient': 4,
                    'odom_dropped_transient': 24,
                    'tf_dropped_transient': 24},
         'max_delivery_delay_s': {'scan': .35, 'odom': .36, 'tf': .34}},
    ]))+'\n')
    evidence = {'transient_injected_mid_route': True,
                'transient_profile_request': {
                    'profile_requested': 'transient',
                    'requested_monotonic': 100.},
                'marker_targets': {'1': [1., 1., 0.]},
                'dwell': {'1': {'start_event_at': 20.}},
                'mission_continuity': {'samples': 60,
                 'route_preserved': True, 'completed_prefix_valid': True,
                 'start_event_count': 1, 'cancel_event_count': 0,
                 'marker_confirmations': {'1': 1, '2': 1}},
                'final_state': 'HOME_REACHED'}
    result = staged.check_web_transient(records, relay, evidence)
    assert result['passed'], result['reasons']
    assert result['follower_cancel_overlap_seen']
    assert result['health_hold_seen'] and result['health_auto_recovered']
    records.append({'kind': 'navigation', 'monotonic': 102.,
                    'status': 'CANCELED', 'manual_stop': True})
    result = staged.check_web_transient(records, relay, evidence)
    assert 'manual_or_latched_navigation_stop_during_transient' in result['reasons']


def test_web_transient_rejects_missing_hold_or_duplicate_delivery(tmp_path):
    records = [{'kind': 'goal', 'monotonic': 95., 'wall_time': 10.,
                'x': 1., 'y': 1., 'yaw': 0.}]
    for index in range(40):
        at = 100.1+index*.1
        records.extend([
            {'kind': 'truth', 'monotonic': at, 'x': index*.01,
             'y': 0., 'yaw': 0.},
            {'kind': 'cmd_vel', 'monotonic': at, 'linear': .08, 'angular': 0.},
            {'kind': 'health', 'monotonic': at,
             'autonomous_allowed': True, 'stop_latched': False},
        ])
    relay = tmp_path/'relay.jsonl'
    relay.write_text('\n'.join(map(__import__('json').dumps, [
        {'event': 'profile', 'name': 'transient', 'monotonic': 100.05},
        {'event': 'sample', 'monotonic': 104.,
         'counts': {f'{name}_dropped_transient': 1
                    for name in ('scan', 'odom', 'tf')},
         'max_delivery_delay_s': {}},
    ]))+'\n')
    evidence = {'transient_injected_mid_route': True,
                'transient_profile_request': {'profile_requested': 'transient',
                                              'requested_monotonic': 100.},
                'marker_targets': {'1': [1., 1., 0.]},
                'dwell': {'1': {'start_event_at': 20.}},
                'mission_continuity': {'samples': 40,
                 'route_preserved': True, 'completed_prefix_valid': True,
                 'start_event_count': 1, 'cancel_event_count': 0,
                 'marker_confirmations': {'1': 2, '2': 1}},
                'final_state': 'HOME_REACHED'}
    result = staged.check_web_transient(records, relay, evidence)
    assert 'bounded_localization_hold_and_recovery_not_proven' in result['reasons']
    assert 'duplicate_or_missing_sector_delivery' in result['reasons']
