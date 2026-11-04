"""Fixture isolation must not alter the tracked map, world or model."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[4]
MODULE = ROOT/'scripts/prepare_classroom_fault_fixture.py'
spec = importlib.util.spec_from_file_location('prepare_classroom_fault_fixture', MODULE)
fixture = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = fixture
spec.loader.exec_module(fixture)


def test_copy_only_remaps_actual_sensor_publishers(tmp_path):
    repo = ROOT
    robot = repo.parent/'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    world = repo/'ros2_ws/src/smart_vision_amr/worlds/simulation_room.world'
    map_yaml = repo/'ros2_ws/src/smart_vision_amr/maps/simulation_room.yaml'
    before = {path: fixture.digest(path) for path in (robot, world, map_yaml)}
    output = tmp_path/'isolated'
    manifest = fixture.prepare(robot, world, map_yaml, output)
    assert before == {path: fixture.digest(path) for path in before}
    model = (output/'waffle_pi_fault.sdf').read_text()
    assert '/classroom_fault/raw_scan' in model
    assert '/classroom_fault/raw_odom' in model
    assert '/classroom_fault/raw_tf' in model
    assert model.count('<range>') == robot.read_text().count('<range>')
    assert (output/'classroom_fault.pgm').is_file()
    assert manifest['world_geometry_unchanged']


def test_independent_ekf_fixture_only_remaps_scan(tmp_path):
    robot = ROOT.parent/'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    world = ROOT/'ros2_ws/src/smart_vision_amr/worlds/simulation_room.world'
    map_yaml = ROOT/'ros2_ws/src/smart_vision_amr/maps/simulation_room.yaml'
    output = tmp_path/'independent'
    fixture.prepare(robot, world, map_yaml, output, 'ekf_independent')
    model = (output/'waffle_pi_fault.sdf').read_text()
    assert '/classroom_fault/raw_scan' in model
    assert '/classroom_fault/raw_odom' not in model
    assert '/classroom_fault/raw_tf' not in model


def test_unknown_model_layout_fails_closed(tmp_path):
    model = tmp_path/'robot.sdf'
    model.write_text('<sdf><model name="x"/></sdf>')
    for name in ('world.world', 'map.yaml'):
        (tmp_path/name).write_text('unused')
    import pytest
    with pytest.raises(ValueError, match='sensor plugin'):
        fixture.prepare(model, tmp_path/'world.world', tmp_path/'map.yaml',
                        tmp_path/'out')


def test_physical_replay_uses_copied_22cm_collision_instead_of_stock_24cm(tmp_path):
    import xml.etree.ElementTree as ET
    robot = ROOT.parent/'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    world = ROOT/'ros2_ws/src/smart_vision_amr/worlds/simulation_room.world'
    map_yaml = ROOT/'ros2_ws/src/smart_vision_amr/maps/simulation_room.yaml'
    before = robot.read_bytes()
    manifest = fixture.prepare(robot, world, map_yaml, tmp_path/'physical',
                               'ekf_independent', physical_footprint=True)
    assert robot.read_bytes() == before
    assert manifest['stock_max_collision_radius_m'] > .23699
    assert manifest['proxy_max_collision_radius_m'] == .22
    assert all(value <= .22 for value in manifest['proxy_collision_bounds_m'].values())
    original = ET.parse(robot)
    copied = ET.parse(tmp_path/'physical/waffle_pi_fault.sdf')
    assert len(original.findall('.//collision')) == len(copied.findall('.//collision'))
    assert ET.tostring(original.find('.//inertial')) == ET.tostring(copied.find('.//inertial'))
    assert len(original.findall('.//sensor')) == len(copied.findall('.//sensor'))
    base = copied.find("./model/link[@name='base_link']/collision[@name='base_collision']")
    assert base.findtext('geometry/cylinder/radius') == '0.22'
    assert base.findtext('pose') == '0 0 0.048 0 0 0'
