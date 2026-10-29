"""Preview load is limited before decoding without modifying ROS image data."""

import os
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtGui, QtWidgets
import pytest

from cmd_vel_controller import CameraFeed, Controller

QT_TEST_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_excess_camera_frames_are_skipped_before_decode():
    now = [0.0]
    feed = CameraFeed(start_thread=False, clock=lambda: now[0])
    feed.connect('/camera/image_raw', False)
    incoming = SimpleNamespace(data=bytearray(b'original'))
    frame = Mock(width=lambda: 640, height=lambda: 480)
    with patch('cmd_vel_controller.decode_camera_image', return_value=frame) as decode:
        assert feed.receive(incoming, feed.request)
        for moment in (.01, .04, .08, .12):
            now[0] = moment
            assert not feed.receive(incoming, feed.request)
        now[0] = .125
        assert feed.receive(incoming, feed.request)
        assert decode.call_count == 2
        assert decode.call_args.args[0] is incoming
    assert incoming.data == bytearray(b'original')
    assert feed.skipped_frames == 4
    assert feed.latest == (frame, .125)
    feed.close()


def test_reconnected_old_topic_and_repeated_decode_errors_do_not_waste_work():
    now = [0.0]
    feed = CameraFeed(max_fps=100, start_thread=False, clock=lambda: now[0])
    feed.connect('/old', True)
    old = feed.request
    feed.connect('/new', True)
    with patch('cmd_vel_controller.decode_camera_image', side_effect=ValueError('bad JPEG')) as decode:
        assert not feed.receive(object(), old)
        decode.assert_not_called()
        assert not feed.receive(object(), feed.request)
        now[0] = .02
        assert not feed.receive(object(), feed.request)
        assert decode.call_count == 1
        assert feed.frame_interval == .1  # Hard limit of 10 preview decodes/sec.
    feed.close()


def test_unchanged_decoded_frame_is_not_scaled_repeatedly(tmp_path):
    frame = QtGui.QImage(8, 8, QtGui.QImage.Format_RGB888)
    camera = SimpleNamespace(latest=(frame, time.monotonic()), status='8 × 8', connect=Mock())
    settings = QtCore.QSettings(str(tmp_path/'preview.ini'), QtCore.QSettings.IniFormat)
    window = Controller(Mock(topic_name='/cmd_vel'), domain_id=76, settings=settings, camera=camera)
    try:
        with patch.object(window.video, 'setPixmap') as render:
            window.render_camera()
            window.render_camera()
            assert render.call_count == 1
            camera.latest = (frame, time.monotonic()+.01)
            window.render_camera()
            assert render.call_count == 2
        assert window.video_timer.interval() == 125
    finally:
        window.timer.stop()
        window.video_timer.stop()
        window.deleteLater()


@pytest.mark.parametrize('fps', [0., -1., float('nan'), float('inf')])
def test_invalid_preview_rates_are_rejected(fps):
    with pytest.raises(ValueError):
        CameraFeed(max_fps=fps, start_thread=False)
