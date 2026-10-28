"""EKF stream-loss safety behavior without driving a robot."""

import json
import time
from types import SimpleNamespace

from std_msgs.msg import String

from smart_vision_amr.ekf_sensor_guard import (
    EkfSensorGuard, EkfSensorGuardCore, publisher_issues,
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
