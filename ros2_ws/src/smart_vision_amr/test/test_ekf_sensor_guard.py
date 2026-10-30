"""EKF stream-loss safety behavior without driving a robot."""

import json
import time
from types import SimpleNamespace

from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from smart_vision_amr.ekf_sensor_guard import (
    EkfSensorGuard, EkfSensorGuardCore, ROS_ERROR_LEVEL, battery_voltage_issues,
    dynamixel_log_failure, publisher_issues,
)
from smart_vision_amr.motion_mux import MotionMuxCore


def feed(core, name, now):
    stamp_ns = int(now * 1e9)
    assert core.observe(name, stamp_ns, stamp_ns, now)


def feed_all(core, now):
    for name in core.REQUIRED:
        feed(core, name, now)
    core.update_graph([], now)


def test_startup_remains_estopped_until_all_streams_stable_and_operator_resets():
    core = EkfSensorGuardCore()
    assert core.tick(10.0) is None
    assert core.tick(100.0) is None
    assert core.latched and not core.armed
    feed(core, 'wheel', 101.0)
    feed(core, 'imu', 101.0)
    assert core.tick(101.0) is None
    assert core.tick(102.0) is None  # no scan, so never armed
    assert not core.operator_reset(102.0)
    for now in (103.0, 104.0, 105.1):
        feed_all(core, now)
        core.tick(now)
    assert core.latched and not core.armed
    assert core.operator_reset(105.1)
    assert not core.latched and core.armed


def test_stale_input_latches_estop_and_only_operator_reset_rearms():
    core = EkfSensorGuardCore()
    for now in (10.0, 11.0, 12.1):
        feed_all(core, now)
        assert core.tick(now) is None
    assert core.operator_reset(12.1)
    assert core.armed

    feed(core, 'wheel', 13.2)
    feed(core, 'scan', 13.2)
    assert 'imu' in core.tick(13.2)
    assert core.latched
    assert core.tick(13.3) is None
    assert not core.operator_reset(13.3)

    for now in (13.3, 14.3, 15.4):
        feed_all(core, now)
        core.tick(now)
    assert core.latched  # healthy data alone cannot clear the stop
    assert core.operator_reset(15.4)
    assert core.armed and not core.latched
    assert 'imu' in core.tick(16.5)


def test_invalid_header_does_not_keep_stream_alive():
    core = EkfSensorGuardCore(arm_seconds=0.2)
    for now in (10.0, 10.3):
        feed_all(core, now)
        core.tick(now)
    assert core.operator_reset(10.3)
    assert core.armed
    assert not core.observe('imu', 10_300_000_000, 10_500_000_000, 10.5)
    assert not core.observe('imu', 9_000_000_000, 10_500_000_000, 10.5)
    feed(core, 'wheel', 11.4)
    feed(core, 'scan', 11.4)
    assert 'imu' in core.tick(11.4)


def test_runtime_odom_duplicate_trips_even_with_fresh_sensors():
    core = EkfSensorGuardCore(arm_seconds=0.2)
    for now in (10.0, 10.3):
        feed_all(core, now)
        core.tick(now)
    assert core.operator_reset(10.3)
    for name in core.REQUIRED:
        feed(core, name, 10.5)
    core.update_graph(['/odom publishers=2 (expected 1)'], 10.5)
    assert '/odom publishers=2' in core.tick(10.5)
    assert core.latched
    for now in (10.6, 10.9):
        for name in core.REQUIRED:
            feed(core, name, now)
        core.update_graph(['/odom publishers=2 (expected 1)'], now)
        core.tick(now)
    assert not core.operator_reset(10.9)


def test_filter_publisher_without_fresh_odom_or_tf_trips_the_guard():
    core = EkfSensorGuardCore(arm_seconds=0.2)
    for now in (10.0, 10.3):
        feed_all(core, now)
        core.tick(now)
    assert core.operator_reset(10.3)
    for name in ('wheel', 'imu', 'scan'):
        feed(core, name, 11.4)
    core.update_graph([], 11.4)  # Graph alone is healthy but filter is stalled.
    reason = core.tick(11.4)
    assert 'odom' in reason and 'odom_tf' in reason and core.latched


def test_battery_gate_is_opt_in_and_rejects_bad_or_stale_readings_when_enabled():
    core = EkfSensorGuardCore(arm_seconds=0.2)
    for now in (10.0, 10.3):
        feed_all(core, now)
        core.tick(now)
    # The zero default preserves existing launches without /battery_state.
    assert 'battery' not in core.required
    assert core.operator_reset(10.3)
    core.set_battery_limits(10.8, 0.2)
    assert 'battery' in core.tick(10.4)
    assert core.latched
    assert core.observe_battery(10.7, 10_500_000_000, 10_500_000_000, 10.5) is False
    assert '10.80 V' in ' '.join(core.missing(10.5))
    assert not core.observe_battery(10.9, 10_550_000_000, 10_550_000_000, 10.55)
    assert '11.00 V' in ' '.join(core.missing(10.55))
    for now in (10.6, 10.9):
        feed_all(core, now)
        stamp = int(now*1e9)
        assert core.observe_battery(11.0, stamp, stamp, now)
        core.tick(now)
    assert core.operator_reset(10.9)
    feed_all(core, 11.0)
    assert not core.observe_battery(10.7, 11_000_000_000, 11_000_000_000, 11.0)
    assert 'battery voltage' in core.trip_reason
    assert core.latched and not core.armed
    feed_all(core, 11.1)
    assert not core.observe_battery(float('nan'), 11_100_000_000,
                                    11_100_000_000, 11.1)
    assert 'invalid battery voltage' in ' '.join(core.missing(11.1))
    feed_all(core, 11.2)
    assert not core.observe_battery(10.9, 11_200_000_000, 11_200_000_000, 11.2)
    assert not core.operator_reset(11.2)
    for now in (11.3, 11.6):
        feed_all(core, now)
        stamp = int(now*1e9)
        assert core.observe_battery(11.0, stamp, stamp, now)
        core.tick(now)
    assert core.operator_reset(11.6)
    feed_all(core, 16.7)
    core.update_graph([], 16.7)
    assert 'battery' in core.tick(16.7)  # No new battery header for >5 s.


def test_battery_has_separate_five_second_receive_and_header_freshness():
    core = EkfSensorGuardCore(minimum_battery_voltage=10.8,
                              battery_stale_seconds=5.0)
    feed_all(core, 10.0)
    assert core.observe_battery(11.5, 10_000_000_000, 10_000_000_000, 10.0)
    feed_all(core, 11.5)
    assert 'battery' not in core.missing(11.5)  # Wheel/IMU use the shorter 1 s limit.
    feed_all(core, 14.9)
    assert 'battery' not in core.missing(14.9)
    feed_all(core, 15.0)
    assert 'battery' in core.missing(15.0)
    # A recent arrival with an old header cannot refresh the stream.
    assert not core.observe_battery(11.5, 11_000_000_000, 17_000_000_000, 17.0)
    core.set_battery_stale_seconds(6.0)
    assert core.stamps['battery'].max_age_ns == 6_000_000_000


def test_initial_battery_above_trip_limit_can_arm_without_recovery_margin():
    core = EkfSensorGuardCore(arm_seconds=0.2, minimum_battery_voltage=10.8,
                              battery_rearm_margin_voltage=0.2)
    for now in (10.0, 10.3):
        feed_all(core, now)
        stamp = int(now*1e9)
        assert core.observe_battery(10.9, stamp, stamp, now)
        core.tick(now)
    assert not core.battery_recovery_required
    assert core.operator_reset(10.3)
    feed_all(core, 10.4)
    assert not core.observe_battery(10.7, 10_400_000_000, 10_400_000_000, 10.4)
    assert core.latched and core.battery_recovery_required
    feed_all(core, 10.5)
    assert not core.observe_battery(10.9, 10_500_000_000, 10_500_000_000, 10.5)
    assert '11.00 V' in ' '.join(core.missing(10.5))


def test_one_bad_battery_sample_latches_and_publishes_before_timer_runs(monkeypatch):
    core = EkfSensorGuardCore(arm_seconds=0.2, minimum_battery_voltage=10.8)
    for now in (10.0, 10.3):
        feed_all(core, now)
        stamp = int(now*1e9)
        assert core.observe_battery(11.5, stamp, stamp, now)
        core.tick(now)
    assert core.operator_reset(10.3)

    class FakeGuard:
        def __init__(self):
            self.core = core
            self.estops = 0
            self.errors = []

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000))

        def get_logger(self):
            return SimpleNamespace(error=self.errors.append)

        def _publish_estop(self, _now):
            self.estops += 1

    monkeypatch.setattr(time, 'monotonic', lambda: 11.0)
    guard = FakeGuard()
    message = BatteryState()
    message.header.stamp.sec = 11
    message.voltage = 10.7
    EkfSensorGuard.on_battery(guard, message)
    assert core.latched and guard.estops == 1
    assert 'battery voltage 10.70 V' in core.trip_reason
    message.header.stamp.nanosec = 20_000_000
    message.voltage = 11.5
    EkfSensorGuard.on_battery(guard, message)
    assert core.latched and guard.estops == 1
    assert not core.operator_reset(11.0)


def test_battery_gate_rejects_invalid_limits_and_voltage():
    assert battery_voltage_issues(-1., 10.8) == ['invalid battery voltage']
    assert battery_voltage_issues(float('inf'), 10.8) == ['invalid battery voltage']
    assert battery_voltage_issues(10.8, 10.8) == []
    encoded = BatteryState()
    encoded.voltage = 11.0
    assert battery_voltage_issues(encoded.voltage, 11.0) == []
    core = EkfSensorGuardCore()
    for limit in (-1., float('nan'), float('inf'), True):
        try:
            core.set_battery_limits(limit, 0.2)
        except ValueError:
            pass
        else:
            assert False, f'accepted invalid voltage limit {limit}'


def test_opencr_error_log_latches_stop_even_when_streams_look_fresh():
    core = EkfSensorGuardCore(arm_seconds=0.2)
    for now in (10.0, 10.3):
        feed_all(core, now)
        core.tick(now)
    assert core.operator_reset(10.3)

    class FakeGuard:
        def __init__(self):
            self.core = core
            self.estops = 0
            self.errors = []

        def get_logger(self):
            return SimpleNamespace(error=self.errors.append)

        def _publish_estop(self, _now):
            self.estops += 1

    guard = FakeGuard()
    message = Log()
    message.level = ROS_ERROR_LEVEL
    message.name = 'some_other_node'
    message.msg = 'Failed to read[[TxRxResult] There is no status packet!]'
    assert not dynamixel_log_failure(message)
    EkfSensorGuard.on_rosout(guard, message)
    assert not core.latched and guard.estops == 0
    message.name = 'DynamixelSDKWrapper'
    assert dynamixel_log_failure(message)
    EkfSensorGuard.on_rosout(guard, message)
    assert core.latched and not core.armed and guard.estops == 1
    assert 'OpenCR transaction failed' in core.trip_reason
    EkfSensorGuard.on_rosout(guard, message)
    assert guard.estops == 1 and len(guard.errors) == 1
    feed_all(core, 10.4)  # Restamped data cannot undo the hardware fault.
    core.tick(10.4)
    assert core.latched


def test_live_graph_requires_one_expected_publisher_per_topic():
    expected = EkfSensorGuard.EXPECTED_PUBLISHERS
    graph = {topic: [SimpleNamespace(node_name=name or 'robot_node',
                                     node_namespace='/')]
             for topic, name in expected.items()}
    assert publisher_issues(lambda topic: graph[topic], expected) == []
    graph['/odom'].append(SimpleNamespace(node_name='turtlebot3_node',
                                          node_namespace='/'))
    assert '/odom publishers=2' in ' '.join(
        publisher_issues(lambda topic: graph[topic], expected))
    graph['/odom'] = [SimpleNamespace(node_name='turtlebot3_node',
                                     node_namespace='/')]
    assert '/odom publisher is not /ekf_hardware' in publisher_issues(
        lambda topic: graph[topic], expected)
    graph['/wheel/odom_raw'] = []
    assert '/wheel/odom_raw publishers=0' in ' '.join(
        publisher_issues(lambda topic: graph[topic], expected))


def test_mux_restart_cannot_clear_guard_without_explicit_reset_packet():
    core = EkfSensorGuardCore(arm_seconds=0.02)
    now = time.monotonic()
    for now in (now - 0.1, now - 0.01):
        feed_all(core, now)
        core.tick(now)
    assert core.latched

    class FakeGuard:
        def __init__(self):
            self.core = core
            self.acknowledged = True
            self.pending_reset = None
            self.last_reset_seq = {}
            self.estop_count = 0

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000))

        def get_logger(self):
            return SimpleNamespace(info=lambda *_: None, error=lambda *_: None)

        def _publish_estop(self, _now):
            self.estop_count += 1

    guard = FakeGuard()
    status = String()
    status.data = json.dumps({'estop_latched': False, 'source_id': 'web-new'})
    # New motion mux process reports unlocked by default, but no operator
    # reset was sent. The guard must reassert its startup latch.
    EkfSensorGuard.on_motion_status(guard, status)
    assert core.latched and guard.estop_count == 1

    reset = String()
    reset.data = json.dumps({'mode': 'reset', 'source_id': 'web-new',
                             'seq': 1, 'sent_at_ns': 20_000_000_000})
    EkfSensorGuard.on_control(guard, reset)
    assert guard.pending_reset is not None
    guard.acknowledged = True
    EkfSensorGuard.on_motion_status(guard, status)
    assert not core.latched and core.armed


def test_latched_guard_reasserts_estop_after_ack_if_mux_restart_status_is_lost():
    now = time.monotonic()

    class FakeGuard:
        core = EkfSensorGuardCore()
        acknowledged = True
        last_estop_at = now - 1.1
        last_owner_check_at = now
        estop_count = 0

        def get_logger(self):
            return SimpleNamespace(error=lambda *_: None)

        def _publish_estop(self, _now):
            self.estop_count += 1

    guard = FakeGuard()
    EkfSensorGuard.on_tick(guard)
    assert guard.core.latched and guard.estop_count == 1


def test_safety_estop_preempts_manual_without_stealing_reset_owner():
    mux = MotionMuxCore(clock=lambda: 10.0)
    web = {'source_id': 'web', 'seq': 1, 'sent_at_ns': 20_000_000_000,
           'mode': 'manual', 'linear': 0.1}
    assert mux.receive_control(web, 20_000_000_000)
    safety = {'source_id': MotionMuxCore.EKF_SENSOR_SAFETY_SOURCE, 'seq': 2,
              'sent_at_ns': 20_000_000_000, 'mode': 'estop'}
    assert mux.receive_control(safety, 20_000_000_000)
    assert mux.status()['source_id'] == 'web'
    assert mux.output() == (0.0, 0.0)
    assert not mux.receive_control({**safety, 'seq': 3, 'mode': 'reset'},
                                   20_000_000_000)
    assert mux.receive_control({**web, 'seq': 4, 'mode': 'reset'},
                               20_000_000_000)
    assert mux.status()['mode'] == 'hold'
