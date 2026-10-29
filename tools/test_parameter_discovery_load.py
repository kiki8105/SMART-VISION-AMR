"""Opening a passive GUI during driving must not start a parameter traffic burst."""

import os
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtWidgets
from navigation_panels import ParameterPanel

QT_TEST_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_automatic_discovery_waits_for_idle_but_explicit_refresh_is_available():
    bridge = SimpleNamespace(busy=True, maintenance=False, namespace='', node=Mock(), call=Mock())
    bridge.node.get_service_names_and_types.return_value = []
    panel = ParameterPanel(bridge, 76, Mock(), autoload=False)
    panel.autoload_pending = True
    try:
        with patch.object(QtCore.QTimer, 'singleShot') as later:
            panel.autoload_parameters()
            bridge.node.get_service_names_and_types.assert_not_called()
            assert '주행 중' in panel.status.text()
            assert later.call_args.args[0] == 1000
        # A user's deliberate live-tuning request is still honored; this does
        # not invoke any blocking service wait or alter the driving node.
        panel.refresh_all()
        bridge.node.get_service_names_and_types.assert_called_once()
        assert not panel.autoload_pending and not panel.loading
    finally:
        panel.groups_refresh_timer.stop()
        panel.deleteLater()


def test_automatic_scan_pauses_remaining_requests_when_a_mission_starts():
    bridge = SimpleNamespace(busy=False, maintenance=False, namespace='', node=Mock(), call=Mock())
    bridge.node.get_service_names_and_types.return_value = [
        ('/controller_server/list_parameters', ['rcl_interfaces/srv/ListParameters'])]
    panel = ParameterPanel(bridge, 76, Mock(), autoload=False)
    try:
        panel.refresh_all(automatic=True)
        listed = bridge.call.call_args.args[3]
        bridge.busy = True
        with patch.object(QtCore.QTimer, 'singleShot') as later:
            listed(SimpleNamespace(result=SimpleNamespace(names=['speed'])), None)
            assert bridge.call.call_count == 1  # No Get/Describe burst while moving.
            resume = later.call_args.args[1]
        bridge.busy = False
        resume()
        assert bridge.call.call_count == 3
    finally:
        panel.groups_refresh_timer.stop()
        panel.deleteLater()
