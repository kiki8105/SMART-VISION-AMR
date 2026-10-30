"""Real ROS service stop acknowledgment and sensor-guard reset veto."""

import json
import os
import time
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from smart_vision_amr.motion_mux import MotionMuxNode


def test_robot_side_services_acknowledge_zero_and_allow_a_guard_to_veto_reset():
    context = Context()
    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        rclpy.init(context=context, domain_id=228)
    mux = MotionMuxNode(context=context)
    probe = Node('safety_protocol_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(mux)
    executor.add_node(probe)
    outputs, intents = [], []
    probe.create_subscription(Twist, '/cmd_vel',
                              lambda message: outputs.append(message.linear.x), 10)
    guard = probe.create_publisher(String, '/smart_vision_amr/manual_control', 10)
    veto = [False]

    def on_control(message):
        packet = json.loads(message.data)
        if packet.get('mode') == 'reset':
            intents.append(packet)

    def on_status(message):
        status = json.loads(message.data)
        if veto[0] and status.get('reset_pending') and not status['estop_latched']:
            guard.publish(String(data=json.dumps({
                'source_id': 'ekf_sensor_safety', 'seq': time.monotonic_ns(),
                'sent_at_ns': probe.get_clock().now().nanoseconds,
                'mode': 'estop'})))

    probe.create_subscription(String, '/smart_vision_amr/manual_control', on_control, 10)
    probe.create_subscription(String, '/smart_vision_amr/motion_status', on_status, 10)
    stop = probe.create_client(Trigger, '/smart_vision_amr/emergency_stop')
    reset = probe.create_client(Trigger, '/smart_vision_amr/reset_emergency_stop')

    def until(predicate, timeout=3.0):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate()

    def call(client):
        future = client.call_async(Trigger.Request())
        if client is reset:
            until(lambda: mux.pending_reset is not None)
            assert not mux.reset_timer.is_canceled()
        until(future.done)
        if client is reset:
            assert mux.reset_timer.is_canceled()
        return future.result()

    try:
        until(lambda: stop.service_is_ready() and reset.service_is_ready()
              and mux.output_available and bool(outputs))
        assert mux.reset_timer.is_canceled()
        mux.core.receive_auto(.15, .0)
        until(lambda: .15 in outputs)
        assert call(stop).success
        until(lambda: outputs[-1] == 0.0)
        assert mux.core.estop_latched
        # This service result is command acknowledgment, not a claim that a
        # physical motor has stopped (there is no physical robot in this test).
        assert call(reset).success
        assert intents and intents[-1]['source_id'] == mux.core.source_id
        assert mux.core.mode == 'hold' and not mux.core.estop_latched
        assert call(stop).success
        veto[0] = True
        assert not call(reset).success
        assert mux.core.estop_latched
        assert mux.core.output() == (0.0, 0.0)
        veto[0] = False
        future = reset.call_async(Trigger.Request())
        until(lambda: mux.pending_reset is not None)
        assert not mux.reset_timer.is_canceled()
        assert call(stop).success
        until(future.done)
        assert not future.result().success
        assert mux.reset_timer.is_canceled()
    finally:
        executor.shutdown()
        probe.destroy_node()
        mux.destroy_node()
        context.shutdown()
