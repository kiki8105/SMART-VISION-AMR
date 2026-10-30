"""Delay attribution uses fake clocks/publishers, with no ROS node or robot."""

import json
from types import SimpleNamespace

import pytest

from smart_vision_amr.runtime_timing import (
    GUARD_METRICS, MUX_METRICS, RuntimeTiming, guard_timing_report, timed_call,
)


class FakeClock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Publisher:
    topic_name = '/cmd_vel'

    def __init__(self, clock, delay=0.):
        self.clock, self.delay, self.messages = clock, delay, []

    def publish(self, message):
        self.messages.append(message)
        self.clock.advance(self.delay)


def test_timing_is_bounded_and_slow_warnings_are_throttled():
    clock, warnings = FakeClock(), []
    timing = RuntimeTiming({'write', 'gap'}, clock=clock, warning=warnings.append)
    timing.gap('gap', .1)
    timed_call(timing, 'write', clock.advance, .3)
    timing.gap('gap', .1)
    timed_call(timing, 'write', clock.advance, .2)
    for index in range(1000):
        timing.observe(f'foreign-{index}', 100.)
        timing.gap(f'foreign-{index}', .1)
    assert len(warnings) == 1
    assert set(timing.metrics) == {'write', 'gap'}
    assert set(timing.previous) == {'gap'}
    assert timing.snapshot()['write']['count'] == 2
    assert timing.snapshot()['write']['max_seconds'] == pytest.approx(.3)
    assert timing.snapshot()['gap']['max_seconds'] == pytest.approx(.3)
    clock.advance(1.)
    timing.observe('write', .2)
    assert len(warnings) == 2


def test_instrumentation_preserves_results_and_exceptions():
    clock = FakeClock()
    timing = RuntimeTiming({'write'}, clock=clock)

    def fail():
        clock.advance(.25)
        raise RuntimeError('original publish failure')

    with pytest.raises(RuntimeError, match='original publish failure'):
        timed_call(timing, 'write', fail)
    assert timing.snapshot()['write']['last_seconds'] == pytest.approx(.25)
    assert timed_call(None, 'write', lambda: 'unchanged') == 'unchanged'


def mux_harness(clock, timing=True):
    from smart_vision_amr.motion_mux import MotionMuxCore, MotionMuxNode

    class Harness(MotionMuxNode):
        def __init__(self):
            self._timing = RuntimeTiming(MUX_METRICS, clock=clock) if timing else None
            self._guard_timing = None
            self.core = MotionMuxCore(clock=clock)
            self.output_pub, self.status_pub = Publisher(clock), Publisher(clock)
            self.output_available = False
            self.output_conflict = ''
            self.output_publishers = 0
            self._ownership_checked_at = float('-inf')
            self.service_source = 'mux-service:test'
            self.allow_topic_reset = False
            self.pending_reset = None
            self.last_zero_published_at = None

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock()*1e9)))

        def get_publishers_info_by_topic(self, _topic):
            clock.advance(.025)
            return [SimpleNamespace(node_name='motion_mux', node_namespace='/')]

        def get_name(self):
            return 'motion_mux'

        def get_namespace(self):
            return '/'

    return Harness()


def test_slow_status_publish_is_separate_from_control_arrival_age(monkeypatch):
    import smart_vision_amr.motion_mux as module
    clock = FakeClock()
    monkeypatch.setattr(module.time, 'monotonic', clock)
    mux = mux_harness(clock)
    mux.status_pub.delay = .35
    mux.output_pub.delay = .02
    payload = {'source_id': 'web', 'seq': 1, 'mode': 'hold',
               'sent_at_ns': int((clock()-.15)*1e9)}
    mux.on_control(SimpleNamespace(data=json.dumps(payload)))
    metrics = mux._timing.snapshot()
    assert metrics['control_receipt_age']['last_seconds'] == pytest.approx(.15)
    assert metrics['ownership_query']['max_seconds'] == pytest.approx(.025)
    assert metrics['cmd_vel_publish']['max_seconds'] == pytest.approx(.02)
    assert metrics['status_publish']['max_seconds'] == pytest.approx(.35)
    assert metrics['control_callback']['max_seconds'] == pytest.approx(.395)
    assert mux.core.last_control['accepted'] is True
    # Completed status-write time appears in the NEXT snapshot, without
    # rewriting the preceding control's original stamp or receive observation.
    mux.publish_status(force=True)
    status = json.loads(mux.status_pub.messages[-1].data)
    assert status['timing_diagnostics']['mux']['status_publish']['max_seconds'] == pytest.approx(.35)
    assert metrics['control_receipt_age']['max_seconds'] == pytest.approx(.15)
    assert mux.core.mode == 'hold'


def test_slow_guard_publish_is_reported_on_next_heartbeat_without_restamping(monkeypatch):
    import smart_vision_amr.ekf_sensor_guard as module
    clock = FakeClock()
    monkeypatch.setattr(module.time, 'monotonic', clock)
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: int(clock()*1e9))
    publisher = Publisher(clock, delay=1.2)
    guard = SimpleNamespace(
        SAFETY_SOURCE='ekf_sensor_safety', safety_seq=0, control_pub=publisher,
        _timing=RuntimeTiming(GUARD_METRICS, clock=clock),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock()*1e9))))
    module.EkfSensorGuard._publish_estop(guard, clock())
    first = json.loads(publisher.messages[0].data)
    assert first['sent_at_ns'] == 100_000_000_000
    assert first['timing_diagnostics']['metrics'] == {}
    assert clock()-first['sent_at_ns']/1e9 == pytest.approx(1.2)
    publisher.delay = 0.
    module.EkfSensorGuard._publish_estop(guard, clock())
    second = json.loads(publisher.messages[1].data)
    assert second['timing_diagnostics']['metrics']['control_publish']['max_seconds'] == pytest.approx(1.2)
    mux = mux_harness(clock)
    # The delayed, sequence-fresh safety stop still latches. The next
    # heartbeat reports the preceding source write without restamping it.
    mux.on_control(SimpleNamespace(data=json.dumps(first)))
    assert mux.core.control_accepted == 1
    assert mux.core.estop_latched
    mux.on_control(SimpleNamespace(data=json.dumps(second)))
    assert mux.core.control_accepted == 2
    assert mux.core.estop_latched
    assert mux._guard_timing['metrics']['control_publish']['max_seconds'] == pytest.approx(1.2)


def test_disabled_mux_omits_diagnostics_and_does_not_accept_old_commands():
    clock = FakeClock()
    mux = mux_harness(clock, timing=False)
    mux.on_control(SimpleNamespace(data=json.dumps({
        'source_id': 'web', 'seq': 1, 'mode': 'manual', 'linear': .1,
        'sent_at_ns': int((clock()-2.)*1e9)})))
    assert mux.core.control_rejected == 1
    assert mux.core.output() == (0., 0.)
    assert 'timing_diagnostics' not in json.loads(mux.status_pub.messages[-1].data)


def test_guard_report_rejects_unbounded_or_nonfinite_diagnostic_fields():
    metrics = {
        'control_publish': {'count': 1, 'last_seconds': .3, 'max_seconds': .4},
        'tick_gap': {'count': 2, 'last_seconds': float('nan'), 'max_seconds': .4},
        'arbitrary-key': {'count': 1, 'last_seconds': 100., 'max_seconds': 100.},
    }
    report = guard_timing_report({'timing_diagnostics': {'metrics': metrics}})
    assert set(report) == {'control_publish'}
    metrics['control_publish']['count'] = 2**100
    assert guard_timing_report({'timing_diagnostics': {'metrics': metrics}}) is None
