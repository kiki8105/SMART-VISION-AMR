"""The production mux loop retains subscriptions and drains before its final stop."""

import signal
import os
import subprocess
import sys
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace

import rclpy.executors

from smart_vision_amr import motion_mux


def test_main_uses_bounded_persistent_executor_and_stops_after_draining(monkeypatch):
    events = []
    old_handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    handlers = dict(old_handlers)
    context = object()
    core = SimpleNamespace(mode='manual', estop_latched=False)
    def publish_output(refresh_ownership=True):
        events.append(('stop', core.mode, core.estop_latched, refresh_ownership))

    def begin_shutdown():
        core.mode, core.estop_latched = 'hold', True
        publish_output(refresh_ownership=False)

    node = SimpleNamespace(
        context=context,
        core=core,
        begin_shutdown=begin_shutdown,
        publish_output=publish_output,
        _state_guard=nullcontext,
        _reset_services_active=0,
        destroy_node=lambda: events.append('destroy_node'),
    )

    def set_signal(signum, handler):
        previous = handlers[signum]
        handlers[signum] = handler
        return previous

    class Executor:
        def add_node(self, added):
            assert added is node
            events.append('add_node')

        def spin_once(self, timeout_sec):
            assert timeout_sec == .05
            events.append('spin_once')
            handlers[signal.SIGTERM](signal.SIGTERM, None)

        def shutdown(self, *, timeout_sec):
            assert timeout_sec == 2.0
            events.append('executor_shutdown')

    def make_executor(*, num_threads, context):
        assert context is node.context
        assert num_threads == 4
        events.append('make_executor')
        return Executor()

    monkeypatch.setattr(signal, 'signal', set_signal)
    monkeypatch.setattr(rclpy.executors, 'MultiThreadedExecutor', make_executor)
    monkeypatch.setattr(motion_mux, 'MotionMuxNode', lambda: node)
    monkeypatch.setattr(motion_mux.rclpy, 'init', lambda **_: events.append('init'))
    monkeypatch.setattr(motion_mux.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(motion_mux.rclpy, 'shutdown',
                        lambda: events.append('context_shutdown'))
    monkeypatch.setattr(motion_mux.rclpy, 'spin_once',
                        lambda *_, **__: (_ for _ in ()).throw(
                            AssertionError('rclpy.spin_once must not re-register the node')))
    monkeypatch.setattr(motion_mux.time, 'sleep',
                        lambda duration: events.append(('flush', duration)))

    motion_mux.main(args=[])

    assert events == [
        'init', 'make_executor', 'add_node', 'spin_once',
        ('stop', 'hold', True, False), ('flush', .05),
        'executor_shutdown', ('stop', 'hold', True, False), ('flush', .05),
        'destroy_node', 'context_shutdown',
    ]
    assert handlers == old_handlers


def test_pending_async_reset_drains_before_real_executor_shutdown():
    # Keep the regression in a subprocess: a broken Humble executor shutdown
    # leaves an outstanding service task and can otherwise hang pytest itself.
    script = '''
import os
import signal
import threading
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from smart_vision_amr import motion_mux

original_node = motion_mux.MotionMuxNode
state = {'observed': False, 'error': None}

class ObservedMux(original_node):
    def __init__(self):
        super().__init__()
        state['mux'] = self

motion_mux.MotionMuxNode = ObservedMux

def request_then_stop():
    probe = None
    try:
        deadline = time.monotonic()+5.
        while 'mux' not in state and time.monotonic() < deadline:
            time.sleep(.01)
        assert 'mux' in state
        mux = state['mux']
        probe = Node('mux_shutdown_reset_probe', context=mux.context)
        client = probe.create_client(Trigger, '/smart_vision_amr/reset_emergency_stop')
        while not client.service_is_ready() and time.monotonic() < deadline:
            time.sleep(.01)
        assert client.service_is_ready()
        client.call_async(Trigger.Request())
        while time.monotonic() < deadline:
            with mux._state_guard():
                if mux.pending_reset is not None and mux._reset_services_active:
                    state['observed'] = True
                    break
            time.sleep(.01)
        assert state['observed'], 'reset service never suspended'
    except Exception as exc:
        state['error'] = repr(exc)
    finally:
        os.kill(os.getpid(), signal.SIGTERM)
        if probe is not None:
            probe.destroy_node()

worker = threading.Thread(target=request_then_stop)
worker.start()
motion_mux.main(args=[])
worker.join(timeout=2.)
assert not worker.is_alive()
assert state['error'] is None, state['error']
assert state['observed']
'''
    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['ROS_DOMAIN_ID'] = '231'
    package = str(Path(__file__).resolve().parents[1])
    env['PYTHONPATH'] = package+os.pathsep+env.get('PYTHONPATH', '')
    completed = subprocess.run([sys.executable, '-c', script], env=env,
                               capture_output=True, text=True, timeout=12.)
    assert completed.returncode == 0, completed.stdout+completed.stderr
