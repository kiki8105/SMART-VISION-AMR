"""Hardware stress changes real copied producers/contact, never stock assets."""

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

from smart_vision_amr.gazebo_model_variant import (
    add_classroom_dual_cameras, configure_physical_rear_camera,
    make_independent_ekf_model, set_camera_update_rate,
)


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT/'scripts/prepare_classroom_fault_fixture.py'
SPEC = importlib.util.spec_from_file_location('hardware_profile_fixture', SCRIPT)
fixture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fixture)
ROBOT = ROOT.parent/'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
WORLD = ROOT/'ros2_ws/src/smart_vision_amr/worlds/simulation_room.world'
MAP = ROOT/'ros2_ws/src/smart_vision_amr/maps/simulation_room.yaml'

RATE_PATHS = {
    "./model/link[@name='base_scan']/sensor[@name='hls_lfcd_lds']/update_rate": 10.,
    "./model/link[@name='imu_link']/sensor[@name='tb3_imu']/update_rate": 20.,
    "./model/plugin[@name='turtlebot3_joint_state']/update_rate": 20.,
}
LEFT = "./model/link[@name='wheel_left_link']/collision[@name='wheel_left_collision']/geometry/cylinder/radius"
RIGHT = "./model/link[@name='wheel_right_link']/collision[@name='wheel_right_collision']/geometry/cylinder/radius"
ACCELERATION = "./model/plugin[@name='turtlebot3_diff_drive']/max_wheel_acceleration"


def prepared(tmp_path, profile='nominal', *, mode='ekf_independent'):
    output = tmp_path/profile
    manifest = fixture.prepare(ROBOT, WORLD, MAP, output, mode,
                               physical_footprint=True, hardware_profile=profile)
    return output, manifest, ET.parse(output/'waffle_pi_fault.sdf')


def assignments(profile):
    expected = {} if profile == 'nominal' else dict(RATE_PATHS)
    if profile == 'wheel_asymmetry':
        expected.update({LEFT: .03267, RIGHT: .03333})
    if profile == 'actuator_lag':
        expected[ACCELERATION] = .15
    return expected


@pytest.mark.parametrize('profile', fixture.HARDWARE_PROFILES)
@pytest.mark.parametrize('mode', ('baseline', 'ekf_independent'))
def test_only_selected_physical_fields_change_and_stock_stays_immutable(tmp_path, profile, mode):
    # Hash the image too: a new model must not silently change mapping inputs.
    image = MAP.parent/next(line.split(':', 1)[1].strip() for line in MAP.read_text().splitlines()
                            if line.startswith('image:'))
    before = {path: fixture.digest(path) for path in (ROBOT, WORLD, MAP, image)}
    _, _, nominal = prepared(tmp_path/'baseline', mode=mode)
    output, manifest, actual = prepared(tmp_path/'selected', profile, mode=mode)
    expected = copy.deepcopy(nominal)
    for path, value in assignments(profile).items():
        expected.find(path).text = format(value, '.12g')
    # Entire XML equality checks visual, inertial, friction, plugin nominal
    # kinematics, topics and every unselected field, not just expected values.
    assert ET.tostring(actual.getroot()) == ET.tostring(expected.getroot())
    assert before == {path: fixture.digest(path) for path in before}
    assert manifest['source'] == {str(path): value for path, value in before.items()}
    assert manifest['fixture'] == {str(path): fixture.digest(path) for path in
                                   (output/'waffle_pi_fault.sdf', output/'classroom_fault.world',
                                    output/'classroom_fault.yaml', output/'classroom_fault.pgm')}
    assert manifest['hardware_profile'] == profile
    assert manifest['odometry_pipeline_unchanged'] is True
    assert manifest['nominal_drive_kinematics_unchanged'] is True
    assert manifest['proxy_max_collision_radius_m'] == .22
    assert all(bound <= .22 for bound in manifest['proxy_collision_bounds_m'].values())
    original = ET.parse(ROBOT)
    assert {change['sdf_path'] for change in manifest['hardware_profile_edits']} == set(assignments(profile))
    for change in manifest['hardware_profile_edits']:
        assert change['before'] == float(original.findtext(change['sdf_path']))
        assert change['after'] == float(actual.findtext(change['sdf_path']))
        assert change['changed'] == (change['before'] != change['after'])
    assert json.loads((output/'fixture_manifest.json').read_text()) == manifest


def test_default_and_explicit_nominal_produce_identical_model(tmp_path):
    default = tmp_path/'default'
    explicit, manifest, _ = prepared(tmp_path, 'nominal')
    fixture.prepare(ROBOT, WORLD, MAP, default, 'ekf_independent', physical_footprint=True)
    assert (default/'waffle_pi_fault.sdf').read_bytes() == (explicit/'waffle_pi_fault.sdf').read_bytes()
    assert manifest['hardware_profile_edits'] == []
    assert manifest['hardware_profile_calibration'] == 'stock_model'


@pytest.mark.parametrize('mode', ('baseline', 'ekf_independent'))
@pytest.mark.parametrize('physical', (False, True))
def test_nominal_preserves_exact_pre_profile_model_edits(tmp_path, mode, physical):
    expected = ET.parse(ROBOT)
    if physical:
        fixture.apply_physical_footprint_proxy(expected)
    expected.find(".//plugin[@name='turtlebot3_laserscan']/ros/remapping").text = '~/out:=/classroom_fault/raw_scan'
    if mode == 'baseline':
        ros = expected.find(".//plugin[@name='turtlebot3_diff_drive']/ros")
        ET.SubElement(ros, 'remapping').text = 'odom:=/classroom_fault/raw_odom'
        ET.SubElement(ros, 'remapping').text = '/tf:=/classroom_fault/raw_tf'
    output = tmp_path/'nominal'
    fixture.prepare(ROBOT, WORLD, MAP, output, mode, physical_footprint=physical)
    actual = ET.parse(output/'waffle_pi_fault.sdf')
    assert ET.tostring(actual.getroot()) == ET.tostring(expected.getroot())


@pytest.mark.parametrize('profile', fixture.HARDWARE_PROFILES[1:])
def test_actual_launch_model_transforms_preserve_stress_and_independent_odom(tmp_path, profile):
    output, _, copied = prepared(tmp_path, profile)
    source = output/'waffle_pi_fault.sdf'
    transformed = tmp_path/'spawned.sdf'
    # Follow simulation.launch.py's real _spawn_robot order, without ROS/Gazebo.
    set_camera_update_rate(source, transformed, 5.)
    add_classroom_dual_cameras(transformed, transformed)
    configure_physical_rear_camera(transformed, transformed)
    make_independent_ekf_model(transformed, transformed)
    spawned = ET.parse(transformed)
    for path, value in assignments(profile).items():
        assert float(spawned.findtext(path)) == value
    drive = spawned.find("./model/plugin[@name='turtlebot3_diff_drive']")
    assert drive.findtext('publish_odom_tf') == 'false'
    assert drive.findtext('odometry_topic') == 'gazebo/odom_raw'
    assert 'odom:=/gazebo/odom_raw' in [item.text for item in drive.findall('ros/remapping')]
    assert drive.findtext('wheel_diameter') == copied.findtext("./model/plugin[@name='turtlebot3_diff_drive']/wheel_diameter")
    assert drive.findtext('wheel_separation') == '0.287'
    assert spawned.findtext(".//plugin[@name='turtlebot3_joint_state']/ros/remapping") == '~/out:=joint_states'
    assert spawned.findtext(".//plugin[@name='turtlebot3_imu']/ros/remapping") == '~/out:=imu'
    assert spawned.findtext(".//plugin[@name='turtlebot3_laserscan']/ros/remapping") == '~/out:=/classroom_fault/raw_scan'
    cameras = spawned.findall(".//sensor[@type='camera']")
    assert len(cameras) == 1 and cameras[0].get('name') == 'rear_camera'
    assert cameras[0].findtext('update_rate') == '5.0'
    assert cameras[0].findtext('camera/image/width') == '320'
    assert fixture.collision_radius_bounds(spawned) == fixture.collision_radius_bounds(copied)


@pytest.mark.parametrize('fault', ('missing_imu_rate', 'invalid_joint_rate', 'wrong_wheel_radius',
                                  'wrong_nominal_diameter', 'invalid_nominal_diameter'))
def test_unknown_stress_layout_fails_before_mutation(fault):
    tree = ET.parse(ROBOT)
    profile = 'wheel_asymmetry'
    if fault == 'missing_imu_rate':
        sensor = tree.find("./model/link[@name='imu_link']/sensor[@name='tb3_imu']")
        sensor.remove(sensor.find('update_rate'))
    elif fault == 'invalid_joint_rate':
        tree.find("./model/plugin[@name='turtlebot3_joint_state']/update_rate").text = 'nan'
    elif fault == 'wrong_wheel_radius':
        tree.find(LEFT).text = '.034'
    elif fault == 'wrong_nominal_diameter':
        tree.find("./model/plugin[@name='turtlebot3_diff_drive']/wheel_diameter").text = '.067'
    elif fault == 'invalid_nominal_diameter':
        tree.find("./model/plugin[@name='turtlebot3_diff_drive']/wheel_diameter").text = 'nan'
    before = ET.tostring(tree.getroot())
    with pytest.raises(ValueError):
        fixture.apply_hardware_profile(tree, profile)
    assert ET.tostring(tree.getroot()) == before


def test_unknown_profile_rejected_without_output(tmp_path):
    output = tmp_path/'out'
    with pytest.raises(ValueError, match='unknown hardware profile'):
        fixture.prepare(ROBOT, WORLD, MAP, output, hardware_profile='idealized')
    assert not output.exists()


def test_cli_profile_reaches_contact_geometry_and_exact_manifest(tmp_path):
    output = tmp_path/'cli'
    result = subprocess.run([sys.executable, str(SCRIPT), '--source-model', str(ROBOT),
                             '--world', str(WORLD), '--map', str(MAP), '--output', str(output),
                             '--mode', 'ekf_independent', '--physical-footprint',
                             '--hardware-profile', 'wheel_asymmetry'],
                            check=True, capture_output=True, text=True)
    assert result.stdout.strip() == str(output/'fixture_manifest.json')
    manifest = json.loads((output/'fixture_manifest.json').read_text())
    model = ET.parse(output/'waffle_pi_fault.sdf')
    assert manifest['hardware_profile'] == 'wheel_asymmetry'
    assert float(model.findtext(LEFT)) == .03267
    assert float(model.findtext(RIGHT)) == .03333
    assert manifest['fixture'][str(output/'waffle_pi_fault.sdf')] == fixture.digest(output/'waffle_pi_fault.sdf')
