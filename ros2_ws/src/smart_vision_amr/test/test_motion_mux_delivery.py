"""Real DDS/executor contention and complete output-envelope timing.

These are isolated loopback ROS nodes, never a Gazebo/physical driving claim.
The publisher supplies original stamped odom, TF, scan and control separately.
"""
from contextlib import contextmanager
import json
import math
import os
import threading
import time
from unittest.mock import patch

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from smart_vision_amr.motion_mux import MotionMuxNode
from test_braking_envelope import brake_mux, odom
from test_near_scan_outlier_guard import scan


def until(predicate, timeout=3.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.005)
    return predicate()


@contextmanager
def streams(beams=360, *, measured=(0., 0.), approaching=True):
    context = Context()
    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        rclpy.init(context=context, domain_id=227)
    mux = MotionMuxNode(context=context, use_sim_time=True)
    mux.core.max_linear_velocity, mux.core.max_angular_velocity = .22, .5
    probe = Node('mux_delivery_producer', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(mux)
    executor.add_node(probe)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    topics = [(Clock, '/clock'), (Odometry, '/odom'), (LaserScan, '/scan'),
              (TFMessage, '/tf'), (String, '/smart_vision_amr/navigation_control'),
              (String, '/smart_vision_amr/localization_health'),
              (String, '/smart_vision_amr/manual_control')]
    pubs = {topic: probe.create_publisher(kind, topic, 10 if topic == '/tf' else qos)
            for kind, topic in topics}
    outputs, errors = [], []
    probe.create_subscription(Twist, '/cmd_vel', lambda msg:
                              outputs.append((time.monotonic(), msg.linear.x, msg.angular.z)), 100)
    stop, auto = threading.Event(), threading.Event()
    auto.set()
    first = time.monotonic()

    def feed():
        k = 0
        while not stop.is_set():
            try:
                stamp_ns = 100_000_000_000+round((time.monotonic()-first)*1.e9)
                stamp = Clock().clock
                stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                pubs['/clock'].publish(Clock(clock=stamp))
                if k % 5 == 0:
                    tf = TransformStamped()
                    tf.header.stamp, tf.header.frame_id = stamp, 'odom'
                    tf.child_frame_id, tf.transform.rotation.w = 'base_footprint', 1.
                    pubs['/tf'].publish(TFMessage(transforms=[tf]))
                    message = Odometry()
                    message.header.stamp, message.header.frame_id = stamp, 'odom'
                    message.child_frame_id = 'base_footprint'
                    message.twist.twist.linear.x, message.twist.twist.angular.z = measured
                    pubs['/odom'].publish(message)
                    pubs['/smart_vision_amr/navigation_control'].publish(String(data=json.dumps({
                        'source_id': 'delivery-nav', 'seq': k, 'sent_at_ns': stamp_ns,
                        'linear': .22, 'angular': .35})))
                    if k % 10 == 0:
                        message = LaserScan()
                        message.header.stamp, message.header.frame_id = stamp, 'base_footprint'
                        message.angle_min, message.angle_increment = -math.pi, 2*math.pi/beams
                        message.range_min, message.range_max = .12, 4.
                        message.ranges = [2.]*beams
                        # A fresh approaching object causes a genuine brake cap
                        # search, not just the cheap empty-point shortcut.
                        if approaching:
                            message.ranges[beams//2] = .40
                        pubs['/scan'].publish(message)
                        pubs['/smart_vision_amr/localization_health'].publish(String(data=json.dumps({
                            'sent_at_ns': stamp_ns, 'level': 'healthy', 'autonomous_allowed': True})))
                        if auto.is_set():
                            pubs['/smart_vision_amr/manual_control'].publish(String(data=json.dumps({
                                'source_id': 'delivery-web', 'seq': k, 'sent_at_ns': stamp_ns, 'mode': 'auto'})))
                k += 1
                stop.wait(.01)
            except BaseException as exc:
                errors.append(exc)
                return

    spinner = threading.Thread(target=executor.spin, daemon=True)
    producer = threading.Thread(target=feed, daemon=True)
    spinner.start()
    producer.start()
    try:
        assert until(lambda: mux.output_available and any(v > .01 for _, v, _ in outputs)), (
            mux.core.scan_guard_code, mux.core.scan_guard_metrics, errors)
        yield mux, pubs, outputs, auto
        assert not errors
    finally:
        stop.set()
        producer.join(timeout=2.)
        executor.shutdown(timeout_sec=3.)
        spinner.join(timeout=3.)
        probe.destroy_node()
        mux.destroy_node()
        context.shutdown()


def test_busy_geometry_does_not_queue_clock_odom_or_stop_behind_four_workers():
    with streams() as (mux, pubs, outputs, auto):
        entered, release = threading.Event(), threading.Event()
        original = mux._refresh_scan_guard_locked

        def busy(*args, **kwargs):
            if not entered.is_set():
                entered.set()
                assert release.wait(1.)
            return original(*args, **kwargs)

        try:
            with patch.object(mux, '_refresh_scan_guard_locked', side_effect=busy):
                assert entered.wait(2.)
                start_ros = mux.get_clock().now().nanoseconds
                old_feedback = mux._braking_feedback
                # Real 20Hz nav/output/odom/TF and 10Hz scan/health/control all
                # compete while geometry owns the shared application lock.
                assert until(lambda: mux._braking_feedback is not None and
                    mux._braking_feedback['stamp_ns'] > old_feedback['stamp_ns']+80_000_000
                    and mux.get_clock().now().nanoseconds > start_ros+80_000_000, .4)
                feedback = mux._braking_feedback
                assert time.monotonic()-feedback['received_at'] < .10
                auto.clear()
                before = len(outputs)
                pubs['/smart_vision_amr/manual_control'].publish(String(data=json.dumps({
                    'source_id': 'delivery-web', 'seq': 1_000_000, 'mode': 'hold',
                    'sent_at_ns': mux.get_clock().now().nanoseconds})))
                # Zero precedes completion of the blocked geometry, while
                # actual control admission still waits for its original lock.
                assert until(lambda: any(v == w == 0. for _, v, w in outputs[before:]), .2)
                assert not release.is_set()
                assert mux._urgent_stop_event.is_set()
                release.set()
                assert until(lambda: mux.core.mode == 'hold')
                time.sleep(.15)
                first_zero = next(i for i in range(before, len(outputs)) if outputs[i][1:] == (0., 0.))
                assert all(item[1:] == (0., 0.) for item in outputs[first_zero:])
        finally:
            release.set()


@pytest.mark.parametrize('later_stop', [False, True])
def test_explicit_resume_is_preserved_during_contention_but_later_stop_wins(later_stop):
    with streams() as (mux, pubs, outputs, auto):
        auto.clear()
        control = pubs['/smart_vision_amr/manual_control']

        def packet(seq, mode):
            return String(data=json.dumps(dict(source_id='delivery-web', seq=seq, mode=mode,
                sent_at_ns=mux.get_clock().now().nanoseconds)))

        control.publish(packet(1_000_000, 'hold'))
        assert until(lambda: mux.core.mode == 'hold')
        entered, release = threading.Event(), threading.Event()
        timer = None

        def busy_callback():
            timer.cancel()
            with mux._state_guard():
                entered.set()
                assert release.wait(1.)

        timer = mux.create_timer(.02, busy_callback, clock=mux.steady_clock,
                                 callback_group=mux.output_group)
        try:
            assert entered.wait(2.)
            resume = packet(2_000_000, 'auto')
            control.publish(resume)
            assert until(lambda: mux._deferred_auto_control is not None, .3)
            original = mux._deferred_auto_control
            assert original[0]['sent_at_ns'] == json.loads(resume.data)['sent_at_ns']
            assert mux.core.mode == 'hold'
            control.publish(resume)  # Replay must not renew the first receipt.
            time.sleep(.03)
            assert mux._deferred_auto_control is original
            before = len(outputs)
            if later_stop:
                control.publish(packet(2_000_001, 'estop'))
                assert until(lambda: mux._urgent_stop_event.is_set(), .3)
            release.set()
            if later_stop:
                assert until(lambda: mux.core.estop_latched)
                time.sleep(.1)
                assert mux._deferred_auto_control is None
                assert all(v == w == 0. for _, v, w in outputs[before:])
            else:
                assert until(lambda: mux.core.mode == 'auto' and
                    mux.core.last_control['seq'] == 2_000_000 and
                    any(v > .01 for _, v, _ in outputs[before:]))
                assert mux._deferred_auto_control is None
        finally:
            release.set()
            # Executor tasks may still reference this cancelled timer. Node
            # teardown destroys it only after all workers have stopped.
            timer.cancel()


@pytest.mark.parametrize('beams', [360, 720])
@pytest.mark.parametrize('moving', [False, True])
def test_complete_mux_curved_braking_meets_compute_budget_without_feedback_starvation(beams, moving, record_property):
    with streams(beams, measured=(.22, .35) if moving else (0., 0.), approaching=not moving) as (mux, pubs, outputs, auto):
        times, codes = [], []
        original = mux._refresh_scan_guard_locked

        def measured(*args, **kwargs):
            start = time.perf_counter()
            result = original(*args, **kwargs)
            times.append(time.perf_counter()-start)
            codes.append(mux.core.scan_guard_code)
            return result

        start = time.monotonic()
        with patch.object(mux, '_refresh_scan_guard_locked', side_effect=measured):
            time.sleep(1.5)
        selected = [item for item in outputs if item[0] >= start+.1]
        gaps = sorted(b[0]-a[0] for a, b in zip(selected, selected[1:]))
        durations = sorted(times)
        record_property('full_guard_max_seconds', max(durations))
        record_property('scenario', 'moving_feedback_far_clear' if moving else 'stationary_feedback_near_cap')
        record_property('nonzero_output_fraction', sum(v > .01 for _, v, _ in selected)/len(selected))
        record_property('output_gap_p95_seconds', gaps[int(.95*(len(gaps)-1))])
        assert len(selected) >= 25, (len(selected), codes)
        assert durations[int(.95*(len(durations)-1))] < .05
        assert gaps[int(.95*(len(gaps)-1))] < .10
        assert codes.count('BRAKING_FEEDBACK_UNAVAILABLE') == 0, codes
        assert 'CONTROL_DEADLINE' not in [mux.core.scan_guard_code], mux.core.scan_guard_metrics
        assert sum(v > .01 for _, v, _ in selected) >= len(selected)*.9


@pytest.mark.parametrize('change', ['wall_overrun', 'clock_jump', 'clock_rewind',
                                   'invalid_feedback', 'feedback_jump', 'feedback_in_bounds'])
def test_final_publish_checks_compute_bound_and_new_feedback_without_renewing_evidence(monkeypatch, change):
    node = brake_mux(monkeypatch)
    mono = [100.]
    monkeypatch.setattr('smart_vision_amr.motion_mux.time.monotonic', lambda: mono[0])
    node.on_braking_odom(odom(node))
    node.on_scan_guard(scan([math.inf]*101))
    first = node._braking_feedback
    original = node._final_output_guard_locked

    def during_final(v, w):
        original(v, w)
        if change == 'wall_overrun':
            mono[0] += .051
        elif change == 'clock_jump':
            node._ros_now += 60_000_000
        elif change == 'clock_rewind':
            node._ros_now -= 1
        elif change == 'invalid_feedback':
            node.on_braking_odom(odom(node, frame='other'))
        else:
            node._ros_now += 1_000_000
            node.on_braking_odom(odom(node, .10 if change == 'feedback_jump' else .001))

    node._final_output_guard_locked = during_final
    node._publish_output_locked()
    assert first['received_at'] == 100.
    if change == 'feedback_in_bounds':
        assert node.last_published_output[0] > 0.
        assert not node.core.scan_guard_hold
    else:
        assert node.last_published_output == (0., 0.)
        assert node.core.scan_guard_code == ('CONTROL_DEADLINE' if change in {
            'wall_overrun', 'clock_jump', 'clock_rewind'} else 'BRAKING_FEEDBACK_UNAVAILABLE')


def test_compute_budget_is_included_in_pose_pending_and_velocity_geometry(monkeypatch):
    node = brake_mux(monkeypatch)
    node.on_braking_odom(odom(node))
    node.on_scan_guard(scan([math.inf]*101, node._ros_now-100_000_000))
    pending_stamp = node._ros_now
    node._scan_guard_pose = lambda stamp=0: (None if stamp == pending_stamp else
        ((0., 0., 0.), stamp or node._ros_now))
    node.on_scan_guard(scan([math.inf]*101))
    metrics = node.core.scan_guard_metrics
    assert metrics['longitudinal_uncertainty_m'] >= node.core.max_linear_velocity*.05
    assert metrics['braking_pending_motion_bound_seconds'] >= .05
    assert metrics['braking_linear_velocity_interval'][1] >= .15*.05


def test_feedback_first_receipt_precedes_lock_wait_and_duplicates_cannot_renew_it(monkeypatch):
    node = brake_mux(monkeypatch)
    node._braking_feedback_lock = threading.Lock()
    first_receipt = threading.Event()
    mono = [100.]

    def now():
        value = mono[0]
        first_receipt.set()
        return value

    monkeypatch.setattr('smart_vision_amr.motion_mux.time.monotonic', now)
    with node._braking_feedback_lock:
        worker = threading.Thread(target=node.on_braking_odom, args=(odom(node),))
        worker.start()
        assert first_receipt.wait(1.)
        mono[0] = 100.2
    worker.join(timeout=1.)
    assert not worker.is_alive()
    assert node._braking_feedback['received_at'] == 100.
    node.on_braking_odom(odom(node))
    assert node._braking_feedback['received_at'] == 100.


@pytest.mark.parametrize('fault', ['old', 'future', 'duplicate', 'retired', 'nan', 'safety_old'])
def test_fast_stop_admission_preserves_rejection_and_safety_watchdog_exception(monkeypatch, fault):
    node = brake_mux(monkeypatch)
    node._output_publish_lock = threading.Lock()
    node._urgent_stop_event = threading.Event()
    node._urgent_stop_generation = 0
    node._ownership_completed_at = 100.
    node.service_source = 'mux-service:test-only'
    node.pending_reset, node.allow_topic_reset = None, True
    packet = dict(source_id='web', seq=1, mode='estop', sent_at_ns=node._ros_now)
    if fault in {'old', 'safety_old'}:
        packet['sent_at_ns'] -= 2_000_000_000
    if fault == 'future':
        packet['sent_at_ns'] += 2_000_000_000
    if fault == 'duplicate':
        node.core.sequences['web'] = 1
    if fault == 'retired':
        node.core.retired_sources.add('web')
    if fault == 'nan':
        packet['linear'] = math.nan
    if fault == 'safety_old':
        packet['source_id'] = node.core.EKF_SENSOR_SAFETY_SOURCE
    node.on_control(String(data=json.dumps(packet)))
    assert not node._urgent_stop_event.is_set()
    assert node.core.estop_latched == (fault == 'safety_old')
    assert node._urgent_stop_generation == (1 if fault == 'safety_old' else 0)
    if fault == 'safety_old':
        assert node.last_published_output == (0., 0.)
    else:
        node.output_pub.publish.assert_not_called()


def test_newer_auto_owner_cannot_be_overwritten_by_an_older_deferred_source(monkeypatch):
    node = brake_mux(monkeypatch)
    node.core.mode, node.core.source_id = 'hold', 'old-owner'
    node.service_source, node.pending_reset, node.allow_topic_reset = 'mux-service:test', None, True
    node._deferred_auto_control = (dict(source_id='queued-A', seq=1, mode='auto',
        sent_at_ns=node._ros_now-10_000_000), 99.99)
    node.on_control(String(data=json.dumps(dict(source_id='newer-B', seq=1,
        mode='auto', sent_at_ns=node._ros_now))))
    assert node.core.source_id == 'newer-B'
    assert node.core.last_control['source_id'] == 'newer-B'
    assert 'newer-B' not in node.core.retired_sources
    assert node._deferred_auto_control is None


def test_queued_resume_does_not_renew_original_age_while_clock_is_paused(monkeypatch):
    node = brake_mux(monkeypatch)
    node.core.mode = 'hold'
    node._deferred_auto_control = (dict(source_id='web', seq=1, mode='auto',
        sent_at_ns=node._ros_now), 98.99)
    node._publish_output_locked()
    assert node.core.mode == 'hold'
    assert not node.core.last_control['accepted']
    assert node.last_published_output == (0., 0.)
    assert node._deferred_auto_control is None
