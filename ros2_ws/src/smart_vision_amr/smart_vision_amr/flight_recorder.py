"""Bounded incident diagnostics with real ROS fields, saved as gzip JSONL.

This is intentionally a sampled diagnostic capture, not a rosbag. The manifest
reports actual pre/post coverage, dropped samples, and per-topic sampling.
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
import gzip
import json
import math
import os
from pathlib import Path as FilePath
import shutil
import time
from uuid import uuid4

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import SpeedLimit
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


_SENSITIVE_KEYS = {'password', 'passwd', 'token', 'secret', 'authorization',
                   'pairing_code', 'access_code', 'admin_code', 'credentials'}


def diagnostic_value(value):
    """Preserve numeric fields, encode nonfinite values explicitly, omit secrets."""
    if isinstance(value, dict):
        return {str(key): ('[REDACTED]' if str(key).lower() in _SENSITIVE_KEYS
                           else diagnostic_value(item)) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [diagnostic_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0 else '-Infinity')
    return value


@dataclass(frozen=True)
class RecordedSample:
    monotonic_sec: float
    line: bytes


class IncidentBuffer:
    """ROS-independent bounded prebuffer and a single bounded post capture."""
    def __init__(self, pre_seconds=30., post_seconds=5., max_samples=4000,
                 max_bytes=8*1024*1024, max_message_bytes=512*1024):
        if (not all(math.isfinite(value) and value >= 0 for value in (pre_seconds, post_seconds))
                or max_samples < 1 or max_bytes < 128 or max_message_bytes < 128):
            raise ValueError('Invalid incident buffer limits')
        self.pre_seconds, self.post_seconds = pre_seconds, post_seconds
        self.max_samples, self.max_bytes = int(max_samples), int(max_bytes)
        self.max_message_bytes = min(int(max_message_bytes), self.max_bytes)
        self.samples = deque()
        self.buffer_bytes = 0
        self.capture = None
        self.oversize_messages = 0
        self.overflow_samples = 0

    def _trim(self, samples, size, *, cutoff=None):
        removed = 0
        while samples and (len(samples) > self.max_samples or size > self.max_bytes
                           or (cutoff is not None and samples[0].monotonic_sec < cutoff)):
            size -= len(samples.popleft().line)
            removed += 1
        return size, removed

    def add(self, topic, message_type, data, *, now, ros_time_ns, wall_time_ns):
        if not math.isfinite(now):
            raise ValueError('Invalid monotonic sample time')
        payload = {'topic': topic, 'message_type': message_type, 'monotonic_sec': now,
                   'ros_time_ns': int(ros_time_ns), 'received_wall_time_ns': int(wall_time_ns),
                   'data': diagnostic_value(data)}
        line = (json.dumps(payload, ensure_ascii=False, separators=(',', ':'),
                           allow_nan=False)+'\n').encode('utf-8')
        if len(line) > self.max_message_bytes:
            self.oversize_messages += 1
            return False
        sample = RecordedSample(now, line)
        self.samples.append(sample)
        self.buffer_bytes += len(line)
        self.buffer_bytes, removed = self._trim(self.samples, self.buffer_bytes,
                                               cutoff=now-self.pre_seconds)
        self.overflow_samples += removed
        capture = self.capture
        if capture is not None and now <= capture['deadline']:
            capture['samples'].append(sample)
            capture['bytes'] += len(line)
            capture['bytes'], removed = self._trim(capture['samples'], capture['bytes'])
            capture['dropped_samples'] += removed
        return True

    def trigger(self, reason, *, now, ros_time_ns, wall_time_ns):
        if not math.isfinite(now):
            raise ValueError('Invalid incident trigger time')
        event = {'reason': str(reason)[:500], 'monotonic_sec': now,
                 'ros_time_ns': int(ros_time_ns), 'wall_time_ns': int(wall_time_ns)}
        if self.capture is not None:
            if len(self.capture['triggers']) < 20:
                self.capture['triggers'].append(event)
            return False
        self.buffer_bytes, _ = self._trim(self.samples, self.buffer_bytes,
                                         cutoff=now-self.pre_seconds)
        self.capture = {'trigger': event, 'triggers': [event], 'deadline': now+self.post_seconds,
                        'samples': deque(self.samples), 'bytes': self.buffer_bytes,
                        'dropped_samples': 0,
                        'oversize_at_start': self.oversize_messages}
        return True

    def finish(self, *, now, force=False):
        capture = self.capture
        if capture is None or (not force and now < capture['deadline']):
            return None
        self.capture = None
        samples = tuple(capture['samples'])
        trigger_time = capture['trigger']['monotonic_sec']
        first = samples[0].monotonic_sec if samples else trigger_time
        last = samples[-1].monotonic_sec if samples else trigger_time
        manifest = {'schema': 1, 'generated_by': 'smart_vision_amr.flight_recorder',
                    'format': 'sampled ROS messages as gzip JSONL; not rosbag',
                    'trigger': capture['trigger'], 'triggers': capture['triggers'],
                    'configured_pre_seconds': self.pre_seconds,
                    'configured_post_seconds': self.post_seconds,
                    'actual_pre_seconds': max(0., trigger_time-first),
                    'actual_post_seconds': max(0., last-trigger_time),
                    'finished_early': now < capture['deadline'],
                    'sample_count': len(samples), 'uncompressed_bytes': capture['bytes'],
                    'capture_overflow_samples': capture['dropped_samples'],
                    'oversize_messages_during_capture': self.oversize_messages-capture['oversize_at_start'],
                    'prebuffer_oversize_messages_total': self.oversize_messages,
                    'nonfinite_numeric_encoding': 'NaN / Infinity / -Infinity strings',
                    'sensitive_fields': 'redacted recursively; no HTTP credentials/config captured'}
        return manifest, samples


def write_incident(directory, incident, *, max_incidents=50, metadata=None):
    """Write all files in a staging directory, then atomically expose capture."""
    directory = FilePath(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    manifest, samples = incident
    manifest = dict(manifest)
    if metadata:
        manifest.update(metadata)
    stamp = datetime.fromtimestamp(manifest['trigger']['wall_time_ns']/1e9,
                                   timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    name = f'incident_{stamp}_{uuid4().hex[:8]}'
    staging, final = directory/('.'+name+'.tmp'), directory/name
    staging.mkdir(mode=0o700)
    try:
        with gzip.open(staging/'events.jsonl.gz', 'wb', compresslevel=3) as stream:
            for sample in samples:
                stream.write(sample.line)
        (staging/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False,
                                                        allow_nan=False)+'\n', encoding='utf-8')
        os.replace(staging, final)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    # Delete only prior captures owned by this recorder; never arbitrary paths.
    candidates = []
    for item in directory.glob('incident_*'):
        if not item.is_dir() or item.is_symlink():
            continue
        try:
            old = json.loads((item/'manifest.json').read_text(encoding='utf-8'))
            if old.get('generated_by') == 'smart_vision_amr.flight_recorder':
                candidates.append(item)
        except (ValueError, OSError):
            pass
    candidates.sort(key=lambda item: item.name, reverse=True)
    for expired in candidates[max(1, int(max_incidents)):]:
        shutil.rmtree(expired)
    return str(final)


class FlightRecorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')
        for name, default in (
            ('output_directory', '~/.ros/smart_vision_amr/incidents'),
            ('pre_buffer_seconds', 30.), ('post_buffer_seconds', 5.),
            ('max_samples', 4000), ('max_buffer_bytes', 8*1024*1024),
            ('max_message_bytes', 512*1024), ('max_incidents', 50),
            ('sensor_sample_period', .2), ('scan_sample_period', .5),
            ('tf_sample_period', .2), ('path_sample_period', 1.),
            ('status_sample_period', .2), ('minimum_incident_interval', 10.)):
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.buffer = IncidentBuffer(float(value('pre_buffer_seconds')),
            float(value('post_buffer_seconds')), int(value('max_samples')),
            int(value('max_buffer_bytes')), int(value('max_message_bytes')))
        self.output_directory = str(value('output_directory'))
        self.max_incidents = max(1, int(value('max_incidents')))
        self.sampled_at, self.trigger_signatures = {}, {}
        self.last_trigger = float('-inf')
        self.writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix='incident-writer')
        self.pending_writes = deque(maxlen=2)
        self.write_future = None
        self.last_written_directory = None
        self.static_transform_snapshot = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/smart_vision_amr/incident_status', latched)
        self.create_service(Trigger, '/smart_vision_amr/capture_incident', self.capture_service)
        sensors = [('/odom', Odometry), ('/imu', Imu), ('/cmd_vel', Twist),
                   ('/smart_vision_amr/navigation_cmd_vel', Twist), ('/speed_limit', SpeedLimit)]
        self.periods = {topic: max(.02, float(value('sensor_sample_period'))) for topic, _ in sensors}
        self.periods.update({'/scan': max(.02, float(value('scan_sample_period'))),
                            '/tf': max(.02, float(value('tf_sample_period'))),
                            '/tf_static': 0., '/smart_vision_amr/goal': 0.,
                            '/smart_vision_amr/path': max(.02, float(value('path_sample_period')))})
        for topic, kind in sensors + [('/scan', LaserScan), ('/tf', TFMessage)]:
            self.create_subscription(kind, topic, lambda msg, source=topic: self.receive(source, msg),
                                     qos_profile_sensor_data)
        for topic, kind in [('/tf_static', TFMessage), ('/smart_vision_amr/goal', PoseStamped),
                            ('/smart_vision_amr/path', Path)]:
            self.create_subscription(kind, topic, lambda msg, source=topic: self.receive(source, msg),
                                     latched if topic == '/tf_static' else 10)
        for topic in ('/smart_vision_amr/planner_status', '/smart_vision_amr/navigation_status',
                      '/smart_vision_amr/motion_status', '/smart_vision_amr/localization_health',
                      '/smart_vision_amr/scan_projection_status',
                      '/smart_vision_amr/execution_failure',
                      '/smart_vision_amr/failure_memory_status',
                      '/smart_vision_amr/navigation_control'):
            self.periods[topic] = max(.02, float(value('status_sample_period')))
            qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
                   if topic == '/smart_vision_amr/navigation_control' else 10)
            self.create_subscription(String, topic,
                lambda msg, source=topic: self.receive(source, msg), qos)
        from rclpy.clock import Clock, ClockType
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status('BUFFERING', 'Waiting for incident triggers')

    @staticmethod
    def trigger_signature(topic, message):
        if not isinstance(message, dict):
            return None
        if (topic.endswith('/execution_failure')
                and message.get('terminal_confirmed') is True):
            return 'EXECUTION_FAILURE:'+str(message.get('event_id', ''))[:128]
        status = str(message.get('status', '')).upper()
        level = str(message.get('level', message.get('severity', ''))).upper()
        if topic.endswith('/motion_status') and message.get('estop_latched') is True:
            return 'ESTOP'
        if topic.endswith('/localization_health') and level in ('CRITICAL', 'STOP', 'FAULT'):
            return level+':'+str(message.get('reasons', ''))[:300]
        if status in ('FAILED', 'BLOCKED', 'RECOVERY_FAILED', 'SAFETY_STOP'):
            return status+':'+str(message.get('message', ''))[:300]
        return None

    def receive(self, topic, message):
        now = time.monotonic()
        sample_key = topic
        if isinstance(message, TFMessage):
            # /tf multiplexes robot joints, odom and map publishers. A single
            # global throttle can keep only the high-rate wheel batch and
            # erase every map->odom sample needed to diagnose a TF outage.
            frames = tuple((item.header.frame_id, item.child_frame_id)
                           for item in message.transforms)
            sample_key = (topic, frames)
            if sample_key not in self.sampled_at and len(self.sampled_at) >= 256:
                oldest = min(self.sampled_at, key=self.sampled_at.get)
                self.sampled_at.pop(oldest)
        # Reject unsampled sensor packets before recursive ROS conversion.
        # TF can arrive at 100 Hz; serializing 95 discarded packets defeats
        # the recorder's intended CPU isolation even with a bounded ring.
        if (not isinstance(message, String)
                and now-self.sampled_at.get(sample_key, float('-inf')) < self.periods[topic]):
            return
        data = None
        signature = None
        if isinstance(message, String):
            try:
                decoded = json.loads(message.data)
                signature = self.trigger_signature(topic, decoded)
                # Decode JSON status text so nested credentials can be redacted.
                data = {'data': diagnostic_value(decoded), 'string_encoding': 'decoded_json'}
            except (TypeError, ValueError):
                data = {'data': '[non-JSON status omitted]'}
        else:
            data = message_to_ordereddict(message)
        previous_signature = self.trigger_signatures.get(topic)
        self.trigger_signatures[topic] = signature
        changed_event = signature is not None and signature != previous_signature
        if not changed_event and now-self.sampled_at.get(sample_key, float('-inf')) < self.periods[topic]:
            return
        self.sampled_at[sample_key] = now
        module = type(message).__module__.split('.')[0]
        kind = module+'/msg/'+type(message).__name__
        self.buffer.add(topic, kind, data, now=now,
                        ros_time_ns=self.get_clock().now().nanoseconds, wall_time_ns=time.time_ns())
        if topic == '/tf_static':
            safe_data = diagnostic_value(data)
            encoded = json.dumps(safe_data, allow_nan=False)
            if len(encoded.encode('utf-8')) <= self.buffer.max_message_bytes:
                # Static transforms may have been sent long before the 30 s
                # window. Preserve the real last observation in the manifest,
                # explicitly with its receipt time rather than faking a replay.
                self.static_transform_snapshot = {'received_monotonic_sec': now,
                    'received_ros_time_ns': self.get_clock().now().nanoseconds,
                    'message': safe_data}
        interval = max(0., float(self.get_parameter('minimum_incident_interval').value))
        if changed_event and (self.buffer.capture is not None or now-self.last_trigger >= interval):
            self.start_capture(topic+' '+signature)

    def start_capture(self, reason):
        now = time.monotonic()
        started = self.buffer.trigger(reason, now=now,
            ros_time_ns=self.get_clock().now().nanoseconds, wall_time_ns=time.time_ns())
        if started:
            self.last_trigger = now
        self.publish_status('CAPTURING', reason)
        return started

    def capture_service(self, _request, response):
        started = self.start_capture('Manual incident capture')
        response.success = True
        response.message = ('Capturing prebuffer and next %.1f seconds' % self.buffer.post_seconds
                            if started else 'Added manual trigger to the active capture')
        return response

    def _queue_write(self, incident):
        if len(self.pending_writes) >= self.pending_writes.maxlen:
            self.publish_status('ERROR', 'Incident writer queue full; capture dropped')
            return
        self.pending_writes.append(incident)

    def tick(self):
        completed = self.buffer.finish(now=time.monotonic())
        if completed is not None:
            self._queue_write(completed)
        if self.write_future is not None and self.write_future.done():
            try:
                self.last_written_directory = self.write_future.result()
                self.publish_status('SAVED', self.last_written_directory)
            except Exception as exc:
                self.get_logger().error(f'Incident capture could not be saved: {exc}')
                self.publish_status('ERROR', str(exc))
            self.write_future = None
        if self.write_future is None and self.pending_writes:
            self.write_future = self.writer.submit(write_incident, self.output_directory,
                self.pending_writes.popleft(), max_incidents=self.max_incidents,
                metadata={'sample_period_seconds': self.periods,
                          'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                          'last_observed_static_transforms': self.static_transform_snapshot})

    def publish_status(self, status, detail):
        message = String()
        message.data = json.dumps({'status': status, 'message': detail,
            'output_directory': self.output_directory, 'last_capture': self.last_written_directory,
            'buffer_samples': len(self.buffer.samples), 'buffer_bytes': self.buffer.buffer_bytes,
            'capturing': self.buffer.capture is not None}, ensure_ascii=False)
        self.status_pub.publish(message)

    def close(self):
        completed = self.buffer.finish(now=time.monotonic(), force=True)
        if completed is not None:
            self._queue_write(completed)
        self.writer.shutdown(wait=True)
        while self.pending_writes:
            write_incident(self.output_directory, self.pending_writes.popleft(),
                           max_incidents=self.max_incidents,
                           metadata={'sample_period_seconds': self.periods,
                                     'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                                     'last_observed_static_transforms': self.static_transform_snapshot})


def main(args=None):
    rclpy.init(args=args)
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
