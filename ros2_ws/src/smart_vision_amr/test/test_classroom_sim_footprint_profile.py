"""Gazebo must not silently reuse the smaller measured physical footprint."""

from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import yaml


REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO / 'ros2_ws/src/smart_vision_amr/config'
BUILDER = REPO / 'scripts/build_classroom_replay_nav_profile.py'


def test_classroom_sim_profile_derives_stock_sdf_geometry_only(tmp_path):
    nav_source = CONFIG / 'curve_tuning_waffle_pi.yaml'
    nodes_source = CONFIG / 'classroom_hardware_nodes.yaml'
    nav_before, nodes_before = nav_source.read_bytes(), nodes_source.read_bytes()
    nav_output = tmp_path / 'classroom_nav_sim_time.yaml'
    nodes_output = tmp_path / 'classroom_nodes_sim.yaml'

    result = subprocess.run(
        [sys.executable, str(BUILDER), '--source', str(nav_source),
         '--nodes-source', str(nodes_source), '--nodes-destination', str(nodes_output),
         str(nav_output)], capture_output=True, text=True, check=True)
    assert str(nav_output) in result.stdout and str(nodes_output) in result.stdout
    assert nav_source.read_bytes() == nav_before
    assert nodes_source.read_bytes() == nodes_before

    physical_nav, sim_nav = (yaml.safe_load(data) for data in
                             (nav_before, nav_output.read_bytes()))
    physical_nodes, sim_nodes = (yaml.safe_load(data) for data in
                                 (nodes_before, nodes_output.read_bytes()))

    # Gazebo's stock collision box reaches about 23.7 cm from rotation center.
    assert physical_nodes['/astar_planner']['ros__parameters']['robot_radius'] == .22
    for name in ('local_costmap', 'global_costmap'):
        physical_costmap = physical_nav[name][name]['ros__parameters']
        sim_costmap = sim_nav[name][name]['ros__parameters']
        assert physical_costmap['robot_radius'] == .22
        assert sim_costmap['robot_radius'] == .24
        assert sim_costmap['inflation_layer']['inflation_radius'] == .26
        assert sim_costmap['robot_radius'] >= .23699
        assert sim_costmap['inflation_layer']['inflation_radius'] > sim_costmap['robot_radius']

    planner = sim_nodes['/astar_planner']['ros__parameters']
    follower = sim_nodes['/path_follower']['ros__parameters']
    assert planner['robot_radius'] == follower['recovery_robot_radius'] == .24
    assert planner['obstacle_inflation_cells'] == 5
    assert planner['preferred_clearance'] == physical_nodes['/astar_planner']['ros__parameters']['preferred_clearance'] == .45

    # Only the declared footprint geometry and simulation clock may differ.
    def enable_existing_sim_clock(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'use_sim_time':
                    value[key] = True
                else:
                    enable_existing_sim_clock(child)
        elif isinstance(value, list):
            for child in value:
                enable_existing_sim_clock(child)

    expected_nav = deepcopy(physical_nav)
    enable_existing_sim_clock(expected_nav)
    for name in ('local_costmap', 'global_costmap'):
        parameters = expected_nav[name][name]['ros__parameters']
        parameters['robot_radius'] = .24
        parameters['inflation_layer']['inflation_radius'] = .26
    assert sim_nav == expected_nav

    expected_nodes = deepcopy(physical_nodes)
    expected_planner = expected_nodes['/astar_planner']['ros__parameters']
    expected_planner.update(robot_radius=.24, obstacle_inflation_cells=5)
    expected_nodes['/path_follower']['ros__parameters']['recovery_robot_radius'] = .24
    assert sim_nodes == expected_nodes


def test_classroom_launchers_feed_generated_nodes_profile_to_gazebo():
    for launcher in ('run_classroom_fault_sim.sh', 'run_classroom_replay_sim.sh'):
        text = (REPO/'scripts'/launcher).read_text()
        assert 'classroom_nodes_sim.yaml' in text
        assert 'nodes_params_file:=$NODES_PARAMS' in text or 'nodes_params_file:=$nodes_profile' in text


def test_physical_footprint_profile_changes_only_clock_and_preserves_center_preference(tmp_path):
    nav_source = CONFIG/'classroom_demo_operator_20261104.yaml'
    nodes_source = CONFIG/'classroom_hardware_nodes.yaml'
    nav_output, nodes_output = tmp_path/'nav.yaml', tmp_path/'nodes.yaml'
    subprocess.run([sys.executable, str(BUILDER), '--physical-footprint',
        '--source', str(nav_source), '--nodes-source', str(nodes_source),
        '--nodes-destination', str(nodes_output), str(nav_output)], check=True,
        capture_output=True, text=True)
    def sim_time(value):
        if isinstance(value, dict):
            return {key: True if key == 'use_sim_time' else sim_time(child)
                    for key, child in value.items()}
        if isinstance(value, list):
            return [sim_time(child) for child in value]
        return value
    assert yaml.safe_load(nav_output.read_text()) == sim_time(yaml.safe_load(nav_source.read_text()))
    nodes = yaml.safe_load(nodes_output.read_text())
    assert nodes == sim_time(yaml.safe_load(nodes_source.read_text()))
    planner = nodes['/astar_planner']['ros__parameters']
    assert planner['robot_radius'] == .22
    assert planner['obstacle_inflation_cells'] == 5
    assert planner['preferred_clearance'] == .45
