"""The production mux loop must retain subscriptions and publish a final stop."""

import signal
from types import SimpleNamespace

import rclpy.executors

from smart_vision_amr import motion_mux


def test_main_uses_one_persistent_executor_and_stops_before_shutdown(monkeypatch):
    events = []
    old_handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    handlers = dict(old_handlers)
    context = object()
    core = SimpleNamespace(mode='manual', estop_latched=False)
    node = SimpleNamespace(
        context=context,
        core=core,
        publish_output=lambda: events.append(('stop', core.mode, core.estop_latched)),
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

        def shutdown(self):
            events.append('executor_shutdown')

    def make_executor(*, context):
        assert context is node.context
        events.append('make_executor')
        return Executor()

    monkeypatch.setattr(signal, 'signal', set_signal)
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', make_executor)
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
        ('stop', 'hold', True), ('flush', .05),
        'executor_shutdown', 'destroy_node', 'context_shutdown',
    ]
    assert handlers == old_handlers
