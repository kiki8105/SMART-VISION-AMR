"""The EKF trial must keep sensor and TF ownership unambiguous."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from smart_vision_amr.gazebo_model_variant import make_independent_ekf_model
from smart_vision_amr.wheel_joint_odometry import WheelIntegrator


def test_ekf_config_has_single_tf_owner_and_nonoverlapping_inputs():
    config = Path(__file__).resolve().parents[1] / 'config'
    overlay = yaml.safe_load((config / 'ekf_overlay.yaml').read_text())['ekf_overlay']['ros__parameters']
    independent = yaml.safe_load((config / 'ekf_independent.yaml').read_text())['ekf_independent']['ros__parameters']
    assert overlay['publish_tf'] is False
    assert independent['publish_tf'] is True
    assert overlay['odom0'] == '/odom'
    assert independent['odom0'] == '/wheel/odom'
    assert independent['imu0'] == '/imu'
    assert len(independent['odom0_config']) == len(independent['imu0_config']) == 15
    assert [i for i, enabled in enumerate(independent['odom0_config']) if enabled] == [6, 7]
    assert [i for i, enabled in enumerate(independent['imu0_config']) if enabled] == [11]


def test_independent_model_disables_gazebo_odom_tf(tmp_path):
    try:
        source = (Path(get_package_share_directory('turtlebot3_gazebo')) /
                  'models/turtlebot3_waffle_pi/model.sdf')
    except PackageNotFoundError:
        source = tmp_path / 'original.sdf'
    if not source.is_file():
        # Keep the transformation test runnable on a fresh checkout before
        # the upstream simulation package has been built.
        source = tmp_path / 'original.sdf'
        source.write_text('''<sdf><model><plugin name="turtlebot3_diff_drive">
          <publish_odom_tf>true</publish_odom_tf><odometry_topic>odom</odometry_topic>
          </plugin></model></sdf>''')
    original = source.read_bytes()
    destination = tmp_path / 'variant.sdf'
    make_independent_ekf_model(source, destination)
    assert source.read_bytes() == original
    assert not destination.read_text().startswith('<?xml')
    plugin = ET.parse(destination).find(".//plugin[@name='turtlebot3_diff_drive']")
    assert plugin.findtext('publish_odom_tf') == 'false'
    assert plugin.findtext('odometry_topic') == 'gazebo/odom_raw'
    assert 'odom:=/gazebo/odom_raw' in [
        element.text for element in plugin.findall('ros/remapping')]


def test_wheel_integrator_straight_and_turn():
    wheels = WheelIntegrator(radius=0.033, separation=0.287)
    assert wheels.update(1_000_000_000, 0., 0.) is None
    straight = wheels.update(1_100_000_000, 1., 1.)
    assert math.isclose(straight[0], 0.033)
    assert math.isclose(straight[1], 0.)
    assert math.isclose(straight[2], 0.)
    assert math.isclose(straight[3], 0.33)
    turn = wheels.update(1_200_000_000, 0., 2.)
    assert math.isclose(turn[0], straight[0])
    assert math.isclose(turn[1], straight[1])
    assert turn[2] > 0.
    assert math.isclose(turn[4], 0.66 / 0.287)


def test_wheel_integrator_rejects_time_and_angle_jumps():
    wheels = WheelIntegrator()
    wheels.update(1_000_000_000, 0., 0.)
    assert wheels.update(1_900_000_000, 10., 10.) is None
    assert wheels.update(2_000_000_000, 10., 10.) is not None
    assert wheels.update(2_100_000_000, 100., 100.) is None
    assert wheels.update(2_200_000_000, 100., 100.) is not None
    assert math.isclose(wheels.x, 0.)
