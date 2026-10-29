import xml.etree.ElementTree as ET
import pytest
from smart_vision_amr.gazebo_model_variant import set_camera_update_rate


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
