"""Bounded real-value incident capture, time coverage and atomic artifacts."""
import gzip
import json
from pathlib import Path

import pytest

from smart_vision_amr.flight_recorder import (FlightRecorder, IncidentBuffer,
    diagnostic_value, write_incident)


def add(buffer, now, **data):
    return buffer.add('/odom', 'nav_msgs/msg/Odometry', data, now=now,
                      ros_time_ns=round(now*1e9), wall_time_ns=round((1000+now)*1e9))


def trigger(buffer, now):
    return buffer.trigger('test failure', now=now, ros_time_ns=round(now*1e9),
                          wall_time_ns=round((1000+now)*1e9))


def test_prebuffer_keeps_only_configured_window_and_records_actual_values():
    buffer = IncidentBuffer(pre_seconds=30., post_seconds=5.)
    for now in range(101):
        add(buffer, float(now), position_x=now/10., yaw=.123)
    assert len(buffer.samples) == 31
    assert json.loads(buffer.samples[0].line)['data']['position_x'] == 7.
    assert trigger(buffer, 100.)
    for now in range(101, 107):
        add(buffer, float(now), position_x=now/10.)
    manifest, samples = buffer.finish(now=106.)
    assert manifest['actual_pre_seconds'] == 30.
    assert manifest['actual_post_seconds'] == 5.
    assert manifest['configured_pre_seconds'] == 30.
    assert manifest['configured_post_seconds'] == 5.
    assert json.loads(samples[-1].line)['data']['position_x'] == 10.5
    assert not manifest['finished_early']
    assert buffer.capture is None


def test_post_capture_never_finishes_before_deadline_and_shutdown_reports_short_coverage():
    buffer = IncidentBuffer(pre_seconds=30., post_seconds=5.)
    add(buffer, 8., speed=.04)
    trigger(buffer, 10.)
    add(buffer, 11., speed=0.)
    assert buffer.finish(now=11.) is None
    manifest, _ = buffer.finish(now=12., force=True)
    assert manifest['finished_early']
    assert manifest['actual_pre_seconds'] == 2.
    assert manifest['actual_post_seconds'] == 1.


def test_sample_count_and_bytes_bound_both_ring_and_active_capture():
    buffer = IncidentBuffer(max_samples=3, max_bytes=1200, max_message_bytes=500)
    add(buffer, 1., value='before')
    trigger(buffer, 1.)
    for index in range(2, 10):
        add(buffer, index/10.+1., value='x'*100)
    assert len(buffer.samples) <= 3
    assert len(buffer.capture['samples']) <= 3
    assert buffer.buffer_bytes <= 1200 and buffer.capture['bytes'] <= 1200
    manifest, samples = buffer.finish(now=7.)
    assert manifest['capture_overflow_samples'] > 0
    assert len(samples) <= 3
    assert manifest['actual_pre_seconds'] == 0.  # Lost prebuffer is never advertised as 30 s.


def test_oversize_message_is_counted_without_unbounded_payload_or_fake_sample():
    buffer = IncidentBuffer(max_message_bytes=300)
    trigger(buffer, 10.)
    assert not add(buffer, 11., image='x'*1000)
    assert buffer.oversize_messages == 1
    manifest, samples = buffer.finish(now=16.)
    assert not samples
    assert manifest['oversize_messages_during_capture'] == 1


def test_repeated_triggers_coalesce_with_bounded_reasons_without_extending_deadline():
    buffer = IncidentBuffer(post_seconds=5.)
    trigger(buffer, 10.)
    for now in range(11, 61):
        assert not trigger(buffer, float(now))
    assert len(buffer.capture['triggers']) == 20
    assert buffer.capture['deadline'] == 15.
    assert buffer.finish(now=16.) is not None


def test_secret_fields_are_redacted_recursively_and_nonfinite_values_explicit():
    value = diagnostic_value({'token': 'do-not-write', 'status': {'admin_code': '123456'},
                              'scan': [1., float('inf'), float('nan')]})
    assert value['token'] == '[REDACTED]'
    assert value['status']['admin_code'] == '[REDACTED]'
    assert value['scan'] == [1., 'Infinity', 'NaN']
    assert 'do-not-write' not in json.dumps(value, allow_nan=False)


def test_atomic_gzip_artifact_has_manifest_and_actual_fields_and_retains_only_own_files(tmp_path):
    unrelated = tmp_path/'incident_keep_this'
    unrelated.mkdir()
    (unrelated/'personal.txt').write_text('untouched')
    for index in range(3):
        buffer = IncidentBuffer(post_seconds=0.)
        add(buffer, 10.+index, position={'x': 1.23, 'y': -4.56}, angular_velocity=.17)
        trigger(buffer, 10.+index)
        incident = buffer.finish(now=11.+index)
        output = Path(write_incident(tmp_path, incident, max_incidents=2,
                                    metadata={'ros_domain_id': '16'}))
        with gzip.open(output/'events.jsonl.gz', 'rt') as stream:
            payload = json.loads(stream.readline())
        assert payload['data']['position'] == {'x': 1.23, 'y': -4.56}
        manifest = json.loads((output/'manifest.json').read_text())
        assert manifest['ros_domain_id'] == '16'
        assert manifest['format'].endswith('not rosbag')
    owned = list(tmp_path.glob('incident_*/manifest.json'))
    assert len(owned) == 2
    assert (unrelated/'personal.txt').read_text() == 'untouched'
    assert not list(tmp_path.glob('.*.tmp'))


def test_failure_health_and_estop_triggers_are_distinguished_from_normal_status():
    assert FlightRecorder.trigger_signature('/smart_vision_amr/navigation_status',
        {'status': 'FAILED', 'message': 'path blocked'}) == 'FAILED:path blocked'
    assert FlightRecorder.trigger_signature('/smart_vision_amr/localization_health',
        {'level': 'CRITICAL', 'reasons': ['stale_odom']}).startswith('CRITICAL:')
    assert FlightRecorder.trigger_signature('/smart_vision_amr/motion_status',
        {'mode': 'estop', 'estop_latched': True}) == 'ESTOP'
    assert FlightRecorder.trigger_signature('/smart_vision_amr/navigation_status',
        {'status': 'SUCCEEDED'}) is None
    assert FlightRecorder.trigger_signature('/smart_vision_amr/localization_health',
        {'level': 'WARNING'}) is None


def test_execution_failure_capture_waits_for_confirmed_terminal_result():
    topic = '/smart_vision_amr/execution_failure'
    event = {'event_id': 'unique-execution', 'reason': 'execution_no_progress',
             'terminal_confirmed': False}
    assert FlightRecorder.trigger_signature(topic, event) is None
    event['terminal_confirmed'] = True
    assert FlightRecorder.trigger_signature(topic, event) == 'EXECUTION_FAILURE:unique-execution'
    assert FlightRecorder.trigger_signature('/smart_vision_amr/failure_memory_status',
        {'state': 'REJOINING', 'count': 1}) is None


def test_invalid_bounds_are_rejected():
    with pytest.raises(ValueError):
        IncidentBuffer(pre_seconds=float('nan'))
    with pytest.raises(ValueError):
        IncidentBuffer(max_samples=0)


def test_sensor_throttle_skips_expensive_conversion_but_keeps_distinct_tf_publishers():
    from types import SimpleNamespace
    from unittest.mock import Mock, patch
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    class Harness(FlightRecorder):
        def __init__(self):
            self.periods = {'/tf': .2}
            self.sampled_at, self.trigger_signatures = {}, {}
            self.buffer = IncidentBuffer()
            self.last_trigger = float('-inf')
            self.static_transform_snapshot = None
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
        def get_parameter(self, name):
            return SimpleNamespace(value=10.)
    node = Harness()
    def transform(child):
        item = TransformStamped()
        item.header.frame_id = 'map' if child == 'odom' else 'odom'
        item.child_frame_id = child
        item.transform.rotation.w = 1.
        return TFMessage(transforms=[item])
    with patch('smart_vision_amr.flight_recorder.time.monotonic', return_value=10.):
        node.receive('/tf', transform('wheel_left_link'))
        node.receive('/tf', transform('base_footprint'))
        node.receive('/tf', transform('odom'))
        assert len(node.buffer.samples) == 3
        with patch('smart_vision_amr.flight_recorder.message_to_ordereddict', side_effect=AssertionError('discarded TF should not be converted')):
            node.receive('/tf', transform('odom'))
    frames = [json.loads(sample.line)['data']['transforms'][0]['child_frame_id']
              for sample in node.buffer.samples]
    assert frames == ['wheel_left_link', 'base_footprint', 'odom']
