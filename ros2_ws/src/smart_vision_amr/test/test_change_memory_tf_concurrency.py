"""A slow scan must not starve the TF callbacks needed by later scans."""

import os
import threading
import time
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from smart_vision_amr import change_memory


def _until(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return predicate()


def test_main_drains_workers_before_persisting_memory(monkeypatch):
    calls = []
    context = object()

    class MemoryNode:
        def __init__(self):
            self.context = context

        def persist(self):
            calls.append('persist')

        def destroy_node(self):
            calls.append('destroy')

    class Executor:
        def __init__(self, *, num_threads, context):
            assert num_threads == 2
            assert context is node.context
            calls.append('executor')

        def add_node(self, added):
            assert added is node
            calls.append('add')

        def spin(self):
            calls.append('spin')
            raise KeyboardInterrupt

        def shutdown(self):
            calls.append('shutdown')

    node = MemoryNode()
    monkeypatch.setattr(change_memory, 'ChangeMemoryNode', lambda: node)
    monkeypatch.setattr(change_memory, 'MultiThreadedExecutor', Executor)
    monkeypatch.setattr(change_memory.rclpy, 'init', lambda **_: calls.append('init'))
    monkeypatch.setattr(change_memory.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(change_memory.rclpy, 'shutdown', lambda: calls.append('ros_shutdown'))

    change_memory.main(args=[])
    assert calls == ['init', 'executor', 'add', 'spin', 'shutdown',
                     'persist', 'destroy', 'ros_shutdown']


def test_tf_arrives_while_scan_callback_is_busy():
    entered, release = threading.Event(), threading.Event()

    class BusyScanNode(change_memory.ChangeMemoryNode):
        def on_scan(self, _scan):
            entered.set()
            assert release.wait(3.), 'scan callback was never released'

    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        rclpy.init(domain_id=231)
    context = rclpy.get_default_context()
    node = BusyScanNode()
    probe = Node('change_memory_tf_concurrency_probe', context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    executor.add_node(probe)
    scan_pub = probe.create_publisher(LaserScan, '/scan', 10)
    tf_pub = probe.create_publisher(TFMessage, '/tf', 10)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    try:
        assert _until(lambda: scan_pub.get_subscription_count() > 0
                      and tf_pub.get_subscription_count() > 0)
        scan_pub.publish(LaserScan())
        assert entered.wait(2.), 'scan callback was not entered'

        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.header.stamp = probe.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        tf_pub.publish(TFMessage(transforms=[transform]))

        assert _until(lambda: node.tf_buffer.can_transform('map', 'odom', Time()),
                      timeout=1.), 'TF was starved by the scan callback'
        assert not release.is_set()
    finally:
        release.set()
        executor.shutdown(timeout_sec=3.)
        spinner.join(timeout=3.)
        probe.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_scan_projection_does_not_freeze_sim_clock_or_allow_concurrent_state_callbacks():
    """The real /clock callback must run while one application callback is busy."""
    entered, release = threading.Event(), threading.Event()
    received_stamps = []

    class BusyScanNode(change_memory.ChangeMemoryNode):
        def on_scan(self, _scan):
            received_stamps.append(_scan.header.stamp.nanosec)
            entered.set()
            if len(received_stamps) == 1:
                assert release.wait(3.), 'scan callback was never released'

    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        rclpy.init(domain_id=232)
    context = rclpy.get_default_context()
    node = BusyScanNode()
    node.set_parameters([Parameter('use_sim_time', value=True)])
    probe = Node('change_memory_clock_concurrency_probe', context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    executor.add_node(probe)
    scan_pub = probe.create_publisher(LaserScan, '/scan', 10)
    clock_pub = probe.create_publisher(Clock, '/clock', 10)
    tf_pub = probe.create_publisher(TFMessage, '/tf', 10)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        # State-owning map/scan/initial-pose, services and retry timer remain
        # mutually exclusive. Only TF and /clock may run alongside them.
        state_topics = {'/map', '/scan', '/initialpose'}
        state_subscriptions = [sub for sub in node.subscriptions if sub.topic_name in state_topics]
        assert len(state_subscriptions) == 3
        assert all(sub.callback_group is node.application_callbacks for sub in state_subscriptions)
        assert all(service.callback_group is node.application_callbacks for service in node.services
                   if service.srv_name in {'/smart_vision_amr/save_memory',
                                           '/smart_vision_amr/clear_memory'})
        assert all(timer.callback_group is node.application_callbacks for timer in node.timers)
        assert _until(lambda: scan_pub.get_subscription_count() > 0
                      and clock_pub.get_subscription_count() > 0
                      and tf_pub.get_subscription_count() > 0)
        first_clock = Clock()
        first_clock.clock.sec = 10
        clock_pub.publish(first_clock)
        assert _until(lambda: node.get_clock().now().nanoseconds == 10_000_000_000)
        first_scan = LaserScan()
        first_scan.header.stamp.nanosec = 1
        scan_pub.publish(first_scan)
        assert entered.wait(2.)
        for stamp in range(2, 22):
            pending = LaserScan()
            pending.header.stamp.nanosec = stamp
            scan_pub.publish(pending)

        next_clock = Clock()
        next_clock.clock.sec, next_clock.clock.nanosec = 10, 120_000_000
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
        transform.header.stamp = next_clock.clock
        transform.transform.rotation.w = 1.
        clock_pub.publish(next_clock)
        tf_pub.publish(TFMessage(transforms=[transform]))
        assert _until(lambda: node.get_clock().now().nanoseconds == 10_120_000_000,
                      timeout=1.), '/clock was blocked behind projection'
        assert _until(lambda: node.tf_buffer.can_transform('map', 'odom', Time()),
                      timeout=1.), 'TF was blocked behind projection'
        assert not release.is_set()
        # Allow the transport to deliver the burst while application work is
        # blocked. Once released it must not process five old acquisitions.
        time.sleep(.05)
        release.set()
        assert _until(lambda: received_stamps[-1] == 21, timeout=1.)
        assert received_stamps == [1, 21]
    finally:
        release.set()
        executor.shutdown(timeout_sec=3.)
        spinner.join(timeout=3.)
        probe.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
