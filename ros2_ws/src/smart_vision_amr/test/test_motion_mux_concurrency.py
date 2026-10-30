"""Executor isolation and stop ordering on an isolated ROS domain."""

import json
import os
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from smart_vision_amr.motion_mux import MotionMuxCore, MotionMuxNode


def _until(predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return predicate()


def test_topic_estop_is_processed_while_status_publication_is_blocked():
    context = Context()
    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        rclpy.init(context=context, domain_id=230)
    mux = MotionMuxNode(context=context)
    probe = Node('mux_callback_contention_probe', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(mux)
    executor.add_node(probe)
    control = probe.create_publisher(String, '/smart_vision_amr/manual_control', 10)
    outputs = []
    probe.create_subscription(Twist, '/cmd_vel',
                              lambda msg: outputs.append(msg.linear.x), 10)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    entered, release = threading.Event(), threading.Event()

    def packet(seq, mode, linear=0.):
        return String(data=json.dumps({
            'source_id': 'contention-test', 'seq': seq, 'mode': mode,
            'linear': linear, 'angular': 0.,
            'sent_at_ns': probe.get_clock().now().nanoseconds}))

    try:
        assert _until(lambda: control.get_subscription_count() > 0
                      and mux.output_available)
        control.publish(packet(1, 'manual', .1))
        assert _until(lambda: mux.core.mode == 'manual' and .1 in outputs)

        original_publish = mux.status_pub.publish

        def slow_status(message):
            entered.set()
            assert release.wait(2.), 'status publication did not get released'
            return original_publish(message)

        with patch.object(mux.status_pub, 'publish', side_effect=slow_status):
            assert entered.wait(2.), 'status timer never entered its blocked write'
            before = len(outputs)
            control.publish(packet(2, 'estop'))
            assert _until(lambda: mux.core.estop_latched
                          and mux.core.last_control['seq'] == 2, timeout=.5)
            assert not release.is_set()
            assert _until(lambda: 0. in outputs[before:], timeout=.5)
        release.set()
        assert _until(lambda: len(outputs) > before+1)
        first_zero = outputs.index(0., before)
        assert all(value == 0. for value in outputs[first_zero:])
    finally:
        release.set()
        executor.shutdown()
        spinner.join(timeout=3.)
        probe.destroy_node()
        mux.destroy_node()
        context.shutdown()


def test_service_stop_serializes_with_inflight_manual_publish_and_shutdown():
    class Harness(MotionMuxNode):
        def __init__(self):
            self._state_lock = threading.RLock()
            self._status_write_lock = threading.Lock()
            self._closing = False
            self.core = MotionMuxCore()
            self.output_pub = Mock(topic_name='/cmd_vel')
            self.status_pub = Mock()
            self.control_pub = Mock()
            self.output_available = True
            self.output_conflict = ''
            self.output_publishers = 1
            self.pending_reset = None
            self.reset_timer = Mock()
            self.service_source = 'mux-service:concurrency-test'
            self.service_seq = 0
            self.allow_topic_reset = True
            self.last_zero_published_at = None
            self.get_clock = lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=0))

    mux = Harness()
    entered, release = threading.Event(), threading.Event()
    written = []

    def write(message):
        if message.linear.x != 0.:
            entered.set()
            assert release.wait(2.)
        written.append(message.linear.x)

    mux.output_pub.publish.side_effect = write

    def control(seq, mode, linear=0.):
        mux.on_control(String(data=json.dumps({
            'source_id': 'web', 'seq': seq, 'mode': mode,
            'linear': linear, 'sent_at_ns': 0})))

    manual = threading.Thread(target=control, args=(1, 'manual', .1))
    manual.start()
    assert entered.wait(2.)
    stopped = threading.Event()

    def stop():
        assert mux.on_emergency_stop(None, SimpleNamespace()).success
        stopped.set()

    service = threading.Thread(target=stop)
    service.start()
    assert not stopped.wait(.05)
    release.set()
    manual.join(timeout=2.)
    service.join(timeout=2.)
    assert stopped.is_set()
    assert written == [.1, 0.]
    assert mux.core.estop_latched
    control(2, 'manual', .1)
    assert mux.core.last_control['accepted'] is False
    mux.begin_shutdown()
    control(3, 'manual', .1)
    mux.publish_output(refresh_ownership=False)
    assert written == [.1, 0., 0., 0.]


def test_stalled_ownership_query_cannot_keep_or_restore_motion_permission():
    first_started, release_first = threading.Event(), threading.Event()
    own = SimpleNamespace(node_name='motion_mux', node_namespace='/')
    foreign = SimpleNamespace(node_name='another_controller', node_namespace='/')
    calls = [0]

    def lookup(_topic):
        calls[0] += 1
        if calls[0] == 1:
            first_started.set()
            assert release_first.wait(2.)
            return [own]
        return [own, foreign]

    class Harness(MotionMuxNode):
        def __init__(self):
            self._state_lock = threading.RLock()
            self.core = MotionMuxCore()
            self.core.output = Mock(return_value=(.1, 0.))
            self.output_pub = Mock(topic_name='/cmd_vel')
            self.output_available = True
            self.output_conflict = ''
            self.output_publishers = 1
            self._ownership_checked_at = float('-inf')
            self._ownership_completed_at = time.monotonic()
            self._ownership_query_inflight = False
            self.get_publishers_info_by_topic = Mock(side_effect=lookup)

        def get_name(self):
            return 'motion_mux'

        def get_namespace(self):
            return '/'

    mux = Harness()
    older = threading.Thread(target=mux.check_output_ownership)
    older.start()
    assert first_started.wait(2.)
    # The status and output timers can both request a graph check, but only
    # one lookup may run at once. Otherwise completion order can undo a stop.
    mux.check_output_ownership()
    assert calls[0] == 1
    with mux._state_lock:
        mux._ownership_completed_at = time.monotonic()-1.
    mux.publish_output(refresh_ownership=False)
    assert not mux.output_available
    assert 'stale' in mux.output_conflict
    mux.output_pub.publish.assert_not_called()
    # A query that finishes after the freshness limit is too old to restore
    # motion permission, even if it reports only this node's publisher.
    time.sleep(mux.OWNERSHIP_MAX_AGE+.02)
    release_first.set()
    older.join(timeout=2.)
    assert not older.is_alive()
    assert not mux.output_available
    assert 'timed out' in mux.output_conflict
    mux.check_output_ownership()
    assert calls[0] == 2
    assert not mux.output_available
    assert mux.output_publishers == 2
    assert 'another_controller' in mux.output_conflict
    mux.publish_output(refresh_ownership=False)
    mux.output_pub.publish.assert_not_called()


def test_service_stop_cancels_reset_while_unlatched_status_write_is_blocked():
    class Harness(MotionMuxNode):
        def __init__(self):
            self._state_lock = threading.RLock()
            self._status_write_lock = threading.Lock()
            self.core = MotionMuxCore(startup_estop=True)
            self.output_pub = Mock(topic_name='/cmd_vel')
            self.status_pub = Mock()
            self.control_pub = Mock()
            self.output_available = True
            self.output_conflict = ''
            self.output_publishers = 1
            self.reset_timer = Mock()
            self.service_source = 'mux-service:reset-contention'
            self.service_seq = 1
            self.last_zero_published_at = None
            self.get_clock = lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=0))
            self.pending_reset = {
                'future': Future(), 'at': time.monotonic()-.11,
                'applied': False,
                'packet': {'source_id': self.service_source, 'seq': 1,
                           'mode': 'reset', 'sent_at_ns': 0}}

    mux = Harness()
    reset_future = mux.pending_reset['future']
    entered, release = threading.Event(), threading.Event()

    def slow_status(message):
        if not json.loads(message.data)['estop_latched']:
            entered.set()
            assert release.wait(2.)

    mux.status_pub.publish.side_effect = slow_status
    reset_tick = threading.Thread(target=mux.advance_reset)
    reset_tick.start()
    assert entered.wait(2.)
    assert mux.core.estop_latched is False

    stop = threading.Thread(target=mux.on_emergency_stop,
                            args=(None, SimpleNamespace()))
    stop.start()
    assert _until(lambda: reset_future.done(), timeout=.5)
    assert reset_future.result() is False
    assert mux.core.estop_latched
    assert mux.core.output() == (0., 0.)
    assert all(call.args[0].linear.x == 0. for call in mux.output_pub.publish.call_args_list)
    release.set()
    reset_tick.join(timeout=2.)
    stop.join(timeout=2.)
    assert not reset_tick.is_alive() and not stop.is_alive()


def test_topic_stop_before_reset_tick_cancels_it_but_ekf_heartbeat_can_be_vetoed():
    class Harness(MotionMuxNode):
        def __init__(self):
            self._state_lock = threading.RLock()
            self._status_write_lock = threading.Lock()
            self.core = MotionMuxCore(startup_estop=True)
            self.output_pub = Mock(topic_name='/cmd_vel')
            self.status_pub = Mock()
            self.output_available = True
            self.output_conflict = ''
            self.output_publishers = 1
            self.reset_timer = Mock()
            self.service_source = 'mux-service:pending-stop'
            self.allow_topic_reset = False
            self.last_zero_published_at = None
            self.get_clock = lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=0))

        def begin_reset(self, seq):
            future = Future()
            self.pending_reset = {
                'future': future, 'at': time.monotonic()-.09,
                'applied': False,
                'packet': {'source_id': self.service_source, 'seq': seq,
                           'mode': 'reset', 'sent_at_ns': 0}}
            return future

    mux = Harness()

    def stop(source, seq):
        mux.on_control(String(data=json.dumps({
            'source_id': source, 'seq': seq, 'mode': 'estop',
            'sent_at_ns': 0})))

    canceled = mux.begin_reset(1)
    stop('web', 1)  # Inside the reset service's 100 ms intent grace.
    assert canceled.done() and canceled.result() is False
    assert mux.pending_reset is None and mux.core.estop_latched
    mux.advance_reset()
    assert mux.core.estop_latched and mux.core.output() == (0., 0.)
    assert all(call.args[0].linear.x == 0. for call in mux.output_pub.publish.call_args_list)

    vetoed = mux.begin_reset(2)
    stop(MotionMuxCore.EKF_SENSOR_SAFETY_SOURCE, 1)
    assert mux.pending_reset is not None  # Pre-apply heartbeat is expected.
    mux.pending_reset['at'] = time.monotonic()-.11
    mux.advance_reset()
    assert not mux.core.estop_latched and mux.pending_reset['applied']
    stop(MotionMuxCore.EKF_SENSOR_SAFETY_SOURCE, 2)
    assert vetoed.done() and vetoed.result() is False
    assert mux.pending_reset is None and mux.core.estop_latched
    assert all(call.args[0].linear.x == 0. for call in mux.output_pub.publish.call_args_list)
