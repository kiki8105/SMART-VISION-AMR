"""Warehouse actor motion/geometry tests without a ROS or Gazebo process."""
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

from warehouse_dynamic_obstacles import (
    Actor, Motion, Pose, angle, load_routes, main, movement_allowed,
    propose, segment_box_distance,
)


def data():
    return {'frame_id': 'world', 'update_rate_hz': 10.,
            'fixed_obstacles': [{'name': 'wall', 'x_min': -2., 'x_max': 2.,
                                 'y_min': 1., 'y_max': 1.2}],
            'actors': [{'model_name': 'warehouse_cart_1', 'collision_radius_m': .375,
                        'speed_mps': .12, 'dwell_seconds': .2, 'loop': 'ping_pong',
                        'waypoints': [{'x': 0., 'y': 0., 'z': 0.},
                                      {'x': .24, 'y': 0., 'z': 0.}]}]}


def write(tmp_path, value):
    path = tmp_path/'routes.yaml'
    path.write_text(yaml.safe_dump(value))
    return path


def actor():
    return Actor('warehouse_cart_1', (Pose(0., 0.), Pose(.24, 0.)), .12, .375, .2)


def test_valid_route_keeps_world_coordinates_radius_and_original_source_hash(tmp_path):
    path = write(tmp_path, data())
    result = load_routes(path)
    assert result['actors'][0].collision_radius_m == .375
    assert result['actors'][0].waypoints[0].z == 0.
    assert result['robot_radius'] == .22
    assert result['sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()


def test_full_segment_wall_collision_is_rejected_even_when_both_waypoints_are_free(tmp_path):
    scene = data()
    scene['fixed_obstacles'] = [{'x_min': -.1, 'x_max': .1, 'y_min': -.1, 'y_max': .1}]
    scene['actors'][0]['waypoints'] = [{'x': -1., 'y': 0.}, {'x': 1., 'y': 0.}]
    assert math.hypot(-1., 0.) > .375
    with pytest.raises(ValueError, match='intersects'):
        load_routes(write(tmp_path, scene))


def test_loop_closing_segment_is_checked_not_just_waypoint_pairs(tmp_path):
    scene = data()
    scene['fixed_obstacles'] = [{'x_min': -.1, 'x_max': .1, 'y_min': -.1, 'y_max': .1}]
    scene['actors'][0]['waypoints'] = [{'x': -1., 'y': -1.}, {'x': -1., 'y': 1.}, {'x': 1., 'y': 1.}]
    scene['actors'][0]['loop'] = 'once'
    assert load_routes(write(tmp_path, scene))
    scene['actors'][0]['loop'] = 'loop'
    with pytest.raises(ValueError, match='intersects'):
        load_routes(write(tmp_path, scene))


@pytest.mark.parametrize('field,value', [('speed_mps', .0), ('speed_mps', math.nan),
    ('collision_radius_m', -.1), ('model_name', 'waffle_pi'), ('model_name', 'robot'),
    ('loop', 'teleport'), ('waypoints', [False, False])])
def test_invalid_or_robot_target_routes_are_rejected(tmp_path, field, value):
    scene = data()
    scene['actors'][0][field] = value
    with pytest.raises(ValueError):
        load_routes(write(tmp_path, scene))


def test_fixed_wall_bounds_and_vertical_teleport_are_rejected(tmp_path):
    scene = data()
    scene['actors'][0]['waypoints'][1]['z'] = .1
    with pytest.raises(ValueError, match='fixed ground height'):
        load_routes(write(tmp_path, scene))
    scene = data()
    scene['fixed_obstacles'][0]['x_min'] = 3.
    with pytest.raises(ValueError, match='bounds'):
        load_routes(write(tmp_path, scene))


def test_cart_circumscribed_circle_is_used_for_fixed_wall_clearance(tmp_path):
    scene = data()
    scene['fixed_obstacles'][0]['y_min'] = .39
    with pytest.raises(ValueError, match='intersects'):
        load_routes(write(tmp_path, scene))  # .375 body + .02 retained gap.
    scene['fixed_obstacles'][0]['y_min'] = .40
    assert load_routes(write(tmp_path, scene))


@pytest.mark.parametrize('mode', ['ping_pong', 'loop', 'once'])
def test_motion_is_speed_and_yaw_bounded_with_dwell_and_no_endpoint_teleport(mode):
    model = replace(actor(), loop=mode)
    state = Motion(model.waypoints[0], dwell_remaining=.2)
    visits, stopped = set(), 0
    for _ in range(1000):
        old = state
        state = propose(model, state, .1)
        distance = math.hypot(state.pose.x-old.pose.x, state.pose.y-old.pose.y)
        assert distance <= model.speed_mps*.1+1.e-12
        assert abs(angle(state.pose.yaw-old.pose.yaw)) <= model.max_yaw_rate_rps*.1+1.e-12
        assert -.000001 <= state.pose.x <= .240001
        assert abs(state.pose.y) < 1.e-9
        visits.add(round(state.pose.x, 3))
        stopped += distance == 0.
    assert {0., .24} <= visits and stopped > 10
    assert state.finished == (mode == 'once')


def test_turning_at_waypoint_does_not_cut_a_corner_or_strafe():
    model = replace(actor(), waypoints=(Pose(0., 0.), Pose(0., 1.)))
    state = Motion(model.waypoints[0])
    rotated = propose(model, state, .1)
    assert rotated.pose.x == rotated.pose.y == 0.
    assert rotated.pose.yaw == pytest.approx(.1)
    for _ in range(20):
        state = propose(model, state, .1)
    assert state.pose.x == 0. and state.pose.y > 0.


def test_pause_or_tiny_step_budget_never_advances_or_jumps_actor():
    state = Motion(Pose(0., 0.))
    assert propose(actor(), state, 0.) is state
    step = propose(actor(), state, 10., max_step=.01)
    assert math.hypot(step.pose.x-state.pose.x, step.pose.y-state.pose.y) <= .01
    model = replace(actor(), max_step_m=.005)
    assert propose(model, state, 10.).pose.x == .005


def test_exact_rectangle_distance_includes_corner_tangency_and_crossing():
    square = (-.1, -.1, .1, .1)
    assert segment_box_distance((-1., 0.), (1., 0.), square) == 0.
    assert segment_box_distance((-.2, .2), (.2, .2), square) == pytest.approx(.1)
    assert segment_box_distance((.2, .2), (.3, .3), square) == pytest.approx(math.sqrt(.02))


def test_robot_guard_checks_entire_actor_sweep_and_measured_circle():
    model = actor()
    # Both endpoints are far away; translating through the robot is forbidden.
    allowed, reason, gap = movement_allowed(model, Pose(-1., 0.), Pose(1., 0.), Pose(0., 0.), 0.)
    assert not allowed and reason == 'yield_to_robot'
    assert gap == pytest.approx(-.375-.22)
    assert movement_allowed(model, Pose(0., 0.), Pose(.01, 0.), Pose(2., 0.), 0.)[0]


@pytest.mark.parametrize('age', [None, -.001, .5, 1., math.inf, math.nan])
def test_missing_or_stale_robot_truth_never_allows_actor_motion(age):
    assert not movement_allowed(actor(), Pose(0., 0.), Pose(.01, 0.), Pose(2., 0.), age)[0]


def test_truth_delay_and_reaction_margin_cause_yield_without_changing_route_progress():
    model = actor()
    state = Motion(Pose(0., 0.))
    step = propose(model, state, .1)
    assert movement_allowed(model, state.pose, step.pose, Pose(.82, 0.), 0.)[0]
    allowed, reason, _ = movement_allowed(model, state.pose, step.pose, Pose(.82, 0.), .2)
    assert not allowed and reason == 'yield_to_robot'
    # No commit on yielding: the next safe tick proposes the SAME short step.
    assert propose(model, state, .1) == step
    assert state.pose == Pose(0., 0.)


def test_other_actor_collision_is_not_avoided_by_teleporting_or_ignoring_it():
    allowed, reason, _ = movement_allowed(actor(), Pose(0., 0.), Pose(.01, 0.), Pose(3., 0.), 0.,
        others=[(Pose(.5, 0.), .375, .12)])
    assert not allowed and reason == 'yield_to_actor'


def test_physical_domain_is_rejected_before_ros_or_file_access(tmp_path):
    with pytest.raises(SystemExit) as result:
        main(['--routes', str(tmp_path/'does_not_exist.yaml'), '--domain', '16'])
    assert result.value.code == 2


def test_geometry_only_cli_does_not_need_any_ros_process(tmp_path, capsys):
    assert main(['--routes', str(write(tmp_path, data())), '--domain', '178', '--check-only']) == 0
    assert '"valid": true' in capsys.readouterr().out


def test_actual_warehouse_routes_match_collidable_world_models_and_fixed_routes():
    root = Path(__file__).resolve().parents[4]/'ros2_ws/src/smart_vision_amr'
    config = load_routes(root/'config/warehouse_dynamic_routes.yaml')
    world = ET.parse(root/'worlds/warehouse_logistics.world').getroot().find('world')
    assert any(p.get('filename') == 'libgazebo_ros_state.so' and
               p.findtext('ros/namespace') == '/gazebo' for p in world.findall('plugin'))
    for actor in config['actors']:
        model = world.find(f"model[@name='{actor.model_name}']")
        assert model is not None and model.findtext('static') == 'false'
        position = list(map(float, model.findtext('pose').split()))
        start = actor.waypoints[0]
        assert position[:3] == pytest.approx([start.x, start.y, start.z], abs=1.e-8)
        assert angle(position[5]-start.yaw) == pytest.approx(0., abs=1.e-8)
        collisions = model.findall('link/collision')
        assert collisions
        for link in model.findall('link'):
            assert link.findtext('kinematic') == 'true' and link.findtext('gravity') == 'false'
        for collision in collisions:
            xyz = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
            box = collision.find('geometry/box/size')
            cylinder = collision.find('geometry/cylinder/radius')
            if box is not None:
                sx, sy, _ = map(float, box.text.split())
                assert math.hypot(xyz[0], xyz[1])+math.hypot(sx/2, sy/2) <= actor.collision_radius_m+1.e-9
            elif cylinder is not None:
                assert math.hypot(xyz[0], xyz[1])+float(cylinder.text) <= actor.collision_radius_m+1.e-9
            else:
                pytest.fail('An unbounded actor collision shape was introduced')


def test_custom_ros_context_runs_without_initializing_a_global_context(tmp_path):
    pytest.importorskip('rclpy')
    root = Path(__file__).resolve().parents[4]
    output = tmp_path/'actor.json'
    # Real ROS event loop in an isolated child, no Gazebo/clock/services.
    # A clean readiness timeout exercises custom-context timers and shutdown;
    # the old global spin_once crashed first and never wrote this report.
    result = subprocess.run(['/usr/bin/python3', str(root/'scripts/warehouse_dynamic_obstacles.py'),
        '--routes', str(write(tmp_path, data())), '--domain', '179',
        '--startup-timeout', '1', '--output', str(output)],
        env={**os.environ, 'ROS_LOCALHOST_ONLY': '1'}, capture_output=True, text=True, timeout=8.)
    assert result.returncode == 1, result.stderr
    assert 'Traceback' not in result.stderr
    report = json.loads(output.read_text())
    assert report['status'] == 'FAILED'
    assert report['fault'] == 'clock_service_or_model_truth_not_ready'
    assert report['domain'] == 179 and report['robot_commands_published'] == 0
    assert not report['inflight_models']
