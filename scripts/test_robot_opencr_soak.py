"""Offline safety regressions for the read-only robot soak probe."""

import importlib.util
from pathlib import Path

import pytest


PATH = Path(__file__).with_name('soak_robot_opencr_16.py')
SPEC = importlib.util.spec_from_file_location('soak_robot_opencr_16', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def healthy(monitor, now):
    for topic in MODULE.STREAM_MAX_GAP:
        monitor.record(topic, int((100 + now) * 1e9), now,
                       int((100 + now) * 1e9), voltage=12.0)
    monitor.tick(now, int((100 + now) * 1e9),
                 nodes=[MODULE.DRIVER_NODE],
                 publisher_counts={topic: 1 for topic in MODULE.STREAM_MAX_GAP})


def test_healthy_window_needs_full_duration_after_all_streams_advance():
    monitor = MODULE.SoakWindow(duration=3.0, startup_grace=2.0)
    healthy(monitor, 0.0)
    assert monitor.ready_since is None
    healthy(monitor, 0.1)
    assert monitor.ready_since == 0.1
    healthy(monitor, 3.0)
    assert not monitor.complete
    healthy(monitor, 3.2)
    assert monitor.complete
    assert monitor.report(3.2)['status'] == 'passed'


def test_driver_or_sensor_stream_loss_fails_after_ready():
    monitor = MODULE.SoakWindow(duration=300.0)
    healthy(monitor, 0.0)
    healthy(monitor, 0.1)
    monitor.tick(0.2, int(100.2e9), nodes=[],
                 publisher_counts={topic: 1 for topic in MODULE.STREAM_MAX_GAP})
    assert '/turtlebot3_node' in monitor.failure
    assert not monitor.complete

    monitor = MODULE.SoakWindow(duration=300.0)
    healthy(monitor, 0.0)
    healthy(monitor, 0.1)
    monitor.tick(2.2, int(102.2e9), nodes=[MODULE.DRIVER_NODE],
                 publisher_counts={topic: 1 for topic in MODULE.STREAM_MAX_GAP})
    assert '/scan' in monitor.failure


def test_missing_stream_fails_after_startup_grace():
    monitor = MODULE.SoakWindow(duration=300.0, startup_grace=2.0)
    monitor.tick(2.1, int(102.1e9), nodes=[MODULE.DRIVER_NODE],
                 publisher_counts={topic: 0 for topic in MODULE.STREAM_MAX_GAP})
    assert '/wheel/odom_raw' in monitor.failure


def test_voltage_floor_and_serial_error_fail_immediately():
    monitor = MODULE.SoakWindow()
    monitor.record('/battery_state', int(100e9), 0.0, int(100e9), voltage=10.97)
    assert 'below trial floor' in monitor.failure
    monitor = MODULE.SoakWindow(min_voltage=11.2)
    monitor.record('/battery_state', int(100e9), 0.0, int(100e9),
                   voltage=11.19999981)
    assert monitor.failure is None  # BatteryState.voltage is float32.
    monitor.record('/battery_state', int(100.1e9), 0.1, int(100.1e9),
                   voltage=11.198)
    assert 'below trial floor' in monitor.failure
    monitor = MODULE.SoakWindow()
    monitor.record_log('DynamixelSDKWrapper', 40,
                       'Failed to read[[TxRxResult] There is no status packet!]')
    assert 'OpenCR/Dynamixel' in monitor.failure
    monitor = MODULE.SoakWindow()
    monitor.record_log('DynamixelSDKWrapper', 40,
                       'Failed transmit instruction packet!')
    assert 'OpenCR/Dynamixel' in monitor.failure
    monitor = MODULE.SoakWindow()
    monitor.record_log('DynamixelSDKWrapper', 30, 'transient warning')
    assert monitor.failure is None


def test_stale_or_regressed_stamps_fail_and_skew_is_reported():
    monitor = MODULE.SoakWindow()
    monitor.record('/scan', int(100e9), 0.0, int(100.2e9))
    monitor.record('/scan', int(99e9), 0.1, int(100.3e9))
    assert 'moved backwards' in monitor.failure
    monitor = MODULE.SoakWindow()
    monitor.record('/imu', int(100e9), 0.0, int(100.2e9))
    assert monitor.report(0.1)['streams']['/imu']['last_stamp_offset_sec'] == pytest.approx(0.2)
    monitor.record('/imu', int(100e9), 0.1, int(102.2e9))
    assert 'sensor stamp offset' in monitor.failure


def test_battery_stamp_uses_five_second_deadline_while_motion_keeps_two_seconds():
    monitor = MODULE.SoakWindow()
    monitor.record('/battery_state', int(100e9), 0.0, int(104.9e9),
                   voltage=12.0)
    assert monitor.failure is None
    monitor.record('/battery_state', int(100.1e9), 0.1, int(105.2e9),
                   voltage=12.0)
    assert 'sensor stamp offset' in monitor.failure
    monitor = MODULE.SoakWindow()
    monitor.record('/scan', int(100e9), 0.0, int(102.1e9))
    assert 'sensor stamp offset' in monitor.failure

    monitor = MODULE.SoakWindow(duration=300.0)
    healthy(monitor, 0.0)
    healthy(monitor, 0.1)
    for topic in MODULE.STREAM_MAX_GAP:
        if topic != '/battery_state':
            monitor.record(topic, int(104.8e9), 4.8, int(104.8e9))
    counts = {topic: 1 for topic in MODULE.STREAM_MAX_GAP}
    monitor.tick(4.8, int(104.8e9), nodes=[MODULE.DRIVER_NODE],
                 publisher_counts=counts)
    assert monitor.failure is None
    monitor.tick(5.2, int(105.2e9), nodes=[MODULE.DRIVER_NODE],
                 publisher_counts=counts)
    assert '/battery_state' in monitor.failure


def test_rosout_subscription_matches_reliable_volatile_publisher():
    qos = pytest.importorskip('rclpy.qos')
    profile = MODULE.rosout_qos_profile()
    assert profile.reliability == qos.ReliabilityPolicy.RELIABLE
    assert profile.durability == qos.DurabilityPolicy.VOLATILE
    assert profile.depth >= 20


def test_invalid_configuration_and_report_do_not_contain_secrets(tmp_path):
    with pytest.raises(ValueError):
        MODULE.SoakWindow(duration=0)
    with pytest.raises(ValueError):
        MODULE.SoakWindow(min_voltage=float('nan'))
    monitor = MODULE.SoakWindow()
    report = monitor.report(1.0)
    path = tmp_path/'diag.json'
    MODULE._save_report(path, report)
    content = path.read_text()
    assert '"status": "interrupted"' in content
    assert 'password' not in content
