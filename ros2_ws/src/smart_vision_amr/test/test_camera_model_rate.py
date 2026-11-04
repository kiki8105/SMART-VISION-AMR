import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
from smart_vision_amr.gazebo_model_variant import (
    add_classroom_dual_cameras, configure_physical_rear_camera, set_camera_update_rate,
)


def test_camera_rate_preserves_source_and_other_sensors(tmp_path):
    source, target = tmp_path/'source.sdf', tmp_path/'variant.sdf'
    text = '<sdf><model><link><sensor type="camera"><update_rate>30</update_rate><camera><image><width>640</width></image></camera></sensor><sensor type="imu"><update_rate>200</update_rate></sensor></link></model></sdf>'
    source.write_text(text)
    set_camera_update_rate(source, target, 8.)
    tree=ET.parse(target)
    assert source.read_text() == text
    assert tree.find(".//sensor[@type='camera']/update_rate").text == '8.0'
    assert tree.find(".//sensor[@type='imu']/update_rate").text == '200'
    assert tree.find('.//width').text == '640'


@pytest.mark.parametrize('rate', [0., -1., float('nan'), float('inf'), 61.])
def test_invalid_camera_rates_rejected(tmp_path, rate):
    with pytest.raises(ValueError):
        set_camera_update_rate(tmp_path/'unused', tmp_path/'out', rate)


def test_physical_camera_replay_has_only_rear320x240_at5hz_and_preserves_stock(tmp_path):
    repo = Path(__file__).parents[4]
    source = repo.parent/'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    original = source.read_bytes()
    model = tmp_path/'rear_only.sdf'
    add_classroom_dual_cameras(source, model)
    configure_physical_rear_camera(model, model)
    tree = ET.parse(model)
    sensors = tree.findall(".//sensor[@type='camera']")
    assert len(sensors) == 1 and sensors[0].get('name') == 'rear_camera'
    assert sensors[0].findtext('update_rate') == '5.0'
    assert sensors[0].findtext('camera/image/width') == '320'
    assert sensors[0].findtext('camera/image/height') == '240'
    assert '/camera/front/' not in model.read_text()
    assert len(tree.findall(".//plugin[@filename='libgazebo_ros_camera.so']")) == 1
    assert tree.findtext(".//sensor[@type='imu']/update_rate") == '200'
    assert source.read_bytes() == original


def test_rear_only_configuration_requires_known_rear_optical_pose(tmp_path):
    source = tmp_path/'model.sdf'
    source.write_text('<sdf><model/></sdf>')
    with pytest.raises(ValueError, match='rear optical pose'):
        configure_physical_rear_camera(source, tmp_path/'out.sdf')
