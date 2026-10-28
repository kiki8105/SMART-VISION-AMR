#!/usr/bin/env python3
"""Drive one repeatable Gazebo square and score local odometry against truth.

This is an isolated simulation experiment: the route controller uses only
Gazebo ground truth for segment boundaries.  The odometry being evaluated is
never fed back into the controller, so its errors cannot change the route.
"""

import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import time

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from smart_vision_amr.ekf_benchmark_math import (
    Pose2, align_pose, errors, interpolate_pose, summarize_error_rows, wrap_angle,
)


MODES = {
    'baseline': '/odom',
    'ekf_overlay': '/odometry/filtered',
    'ekf_independent': '/odom',
}
TRUTH_TOPIC = '/gazebo/get_entity_state'
ROUTE = [('straight', .55), ('turn', math.pi / 2)] * 4
SAMPLE_COLUMNS = (
    'sim_time_s', 'stage_index', 'segment', 'truth_x', 'truth_y', 'truth_yaw',
    'estimate_x', 'estimate_y', 'estimate_yaw', 'position_error_m',
    'heading_error_rad', 'raw_position_error_m', 'raw_heading_error_rad',
    'interpolation_side_s', 'interpolation_span_s',
)


def pose2(pose):
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return Pose2(float(pose.position.x), float(pose.position.y), yaw)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class BenchmarkTrial(Node):
    def __init__(self, mode, model_name, wall_timeout):
        super().__init__('ekf_benchmark_trial', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.mode = mode
        self.model_name = model_name
        self.estimate_topic = MODES[mode]
        self.wall_timeout = wall_timeout
        self.started_wall = time.monotonic()
        self.phase = 'WAIT'
        self.failure = None
        self.truth = None                  # (stamped Gazebo truth time, pose)
        self.truth_samples = deque(maxlen=100)
        self.truth_request = None
        self.last_truth_request_ns = None
        self.estimates = {}                # topic -> (receipt, header time, pose)
        self.estimate_samples = {}         # topic -> deque[(header time, pose)]
        self.initial = {}                  # topic -> (initial estimate, truth)
        self.started_sim_ns = None
        self.started_run_wall = None
        self.settle_started_ns = None
        self.stage_index = 0
        self.stage_started_ns = None
        self.stage_truth = None
        self.turn_last_yaw = None
        self.turn_progress = 0.0
        self.completed_stages = 0
        self.samples = []
        self.last_sample_ns = None
        self.last_command_ns = None
        self.command_count = 0
        self.truth_distance_m = 0.0
        self.last_distance_truth = None
        self.truth_updates_during_run = 0
        self.match_attempt_count = 0
        self.last_match_attempt_truth_ns = None
        self.estimate_header_lags_s = []
        self.tf_count = 0
        self.tf_count_at_start = 0
        self.tf_duplicate_stamp_count = 0
        self.tf_conflict_count = 0
        self.tf_stale_sample_count = 0
        self.tf_max_gap_s = 0.0
        self.max_interpolation_side_s = 0.0
        self.max_interpolation_span_s = 0.0
        self.last_tf_stamp_ns = None
        self.seen_tf = {}
        self.last_progress_log = time.monotonic()
        self.last_graph_check_ns = None
        self.odom_publishers = []
        self.tf_publishers = []
        self.graph_authority_ok = False

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.truth_client = self.create_client(GetEntityState, TRUTH_TOPIC)
        for topic in sorted(set((self.estimate_topic, '/odom'))):
            self.estimate_samples[topic] = deque(maxlen=500)
            self.create_subscription(Odometry, topic,
                                     lambda message, name=topic: self.on_odom(name, message),
                                     best_effort)
        self.create_subscription(TFMessage, '/tf', self.on_tf, best_effort)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def request_truth(self, now_ns):
        if not self.truth_client.service_is_ready() or self.truth_request is not None:
            return
        if (self.last_truth_request_ns is not None
                and now_ns - self.last_truth_request_ns < 50_000_000):
            return
        request = GetEntityState.Request()
        request.name = self.model_name
        request.reference_frame = 'world'
        self.truth_request = self.truth_client.call_async(request)
        self.last_truth_request_ns = now_ns

    def collect_truth(self):
        if self.truth_request is None or not self.truth_request.done():
            return
        future = self.truth_request
        self.truth_request = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Gazebo truth service failed: {exc}')
            return
        if not response.success:
            return
        current = (stamp_ns(response.header.stamp), pose2(response.state.pose))
        if current[0] <= 0:
            return
        if self.truth is not None and current[0] <= self.truth[0]:
            return
        if self.phase == 'RUN' and self.stage_index < len(ROUTE):
            self.truth_updates_during_run += 1
            if self.last_distance_truth is not None:
                self.truth_distance_m += math.hypot(
                    current[1].x - self.last_distance_truth.x,
                    current[1].y - self.last_distance_truth.y)
            self.last_distance_truth = current[1]
            estimate = self.estimates.get(self.estimate_topic)
            if estimate is not None and estimate[1] > 0:
                self.estimate_header_lags_s.append((current[0] - estimate[1]) / 1e9)
            if ROUTE[self.stage_index][0] == 'turn' and self.turn_last_yaw is not None:
                self.turn_progress += wrap_angle(current[1].yaw - self.turn_last_yaw)
                self.turn_last_yaw = current[1].yaw
        self.truth = current
        self.truth_samples.append(current)

    def on_odom(self, topic, message):
        header_ns = stamp_ns(message.header.stamp)
        pose = pose2(message.pose.pose)
        self.estimates[topic] = (self.now_ns(), header_ns, pose)
        history = self.estimate_samples[topic]
        if not history or header_ns > history[-1][0]:
            history.append((header_ns, pose))

    def on_tf(self, message):
        for item in message.transforms:
            if (item.header.frame_id.lstrip('/') != 'odom'
                    or item.child_frame_id.lstrip('/') not in ('base_footprint', 'base_link')):
                continue
            stamp = stamp_ns(item.header.stamp)
            self.tf_count += 1
            if self.last_tf_stamp_ns is not None and stamp > self.last_tf_stamp_ns:
                self.tf_max_gap_s = max(self.tf_max_gap_s,
                                        (stamp - self.last_tf_stamp_ns) / 1e9)
            self.last_tf_stamp_ns = max(stamp, self.last_tf_stamp_ns or stamp)
            transform = item.transform
            value = (transform.translation.x, transform.translation.y,
                     transform.rotation.x, transform.rotation.y,
                     transform.rotation.z, transform.rotation.w)
            previous = self.seen_tf.get(stamp)
            if previous is not None:
                self.tf_duplicate_stamp_count += 1
                if any(abs(a - b) > 1e-4 for a, b in zip(value, previous)):
                    self.tf_conflict_count += 1
            self.seen_tf[stamp] = value
            if len(self.seen_tf) > 2000:
                oldest = min(self.seen_tf)
                del self.seen_tf[oldest]

    def fresh(self, now_ns, topic):
        entry = self.estimates.get(topic)
        return (entry is not None and now_ns >= entry[0]
                and now_ns - entry[0] < 350_000_000
                and entry[1] > 0 and abs(now_ns - entry[1]) < 500_000_000)

    def truth_fresh(self, now_ns):
        # /clock is often published at 10 Hz while Gazebo stamps the service
        # response at its current simulation tick. A fresh truth stamp can
        # therefore be up to one /clock interval *ahead* of this node's clock.
        return (self.truth is not None
                and -120_000_000 <= now_ns - self.truth[0] < 350_000_000)

    def check_graph_authority(self):
        """Check known odom/TF owners as well as frame data itself.

        ROS TFMessage lacks the sending publisher ID. This catches a duplicate
        known odom owner but cannot prove arbitrary third-party /tf nodes never
        publish the same frame; that limitation is reported in every result.
        """
        odom = self.get_publishers_info_by_topic('/odom')
        tf = self.get_publishers_info_by_topic('/tf')
        self.odom_publishers = sorted(f'{item.node_namespace.rstrip("/")}/{item.node_name}'
                                      for item in odom)
        self.tf_publishers = sorted(f'{item.node_namespace.rstrip("/")}/{item.node_name}'
                                    for item in tf)
        expected = ('ekf_independent' if self.mode == 'ekf_independent'
                    else 'turtlebot3_diff_drive')
        # A node may register a /tf endpoint even with publish_tf=false.
        # Endpoint presence is not evidence it actually broadcasts this frame.
        self.graph_authority_ok = (
            len(odom) == 1 and odom[0].node_name == expected
            and any(item.node_name == expected for item in tf)
        )
        return self.graph_authority_ok

    def matched_sample(self, now_ns):
        """Choose a stamped truth pose bracketed by odom; no extrapolation."""
        topics = (self.estimate_topic, '/odom') if self.mode == 'ekf_overlay' else (self.estimate_topic,)
        for truth_ns, truth_pose in reversed(self.truth_samples):
            if truth_ns > now_ns + 120_000_000 or now_ns - truth_ns > 300_000_000:
                continue
            matches = {}
            for topic in topics:
                interpolated = interpolate_pose(self.estimate_samples[topic], truth_ns)
                if interpolated is None:
                    break
                pose, side_ns, span_ns = interpolated
                # Gazebo /odom is ~30 Hz; the EKF output can be ~10 Hz even
                # with a 30 Hz target. Reject gaps larger than one EKF cycle
                # plus scheduling tolerance while still avoiding extrapolation.
                if side_ns > 110_000_000 or span_ns > 120_000_000:
                    break
                matches[topic] = interpolated
            else:
                return truth_ns, truth_pose, matches
        return None

    def zero(self):
        self.cmd_pub.publish(Twist())
        self.command_count += 1

    def command(self, linear, angular):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.cmd_pub.publish(message)
        self.command_count += 1

    def fail(self, reason):
        if self.phase not in ('FAILED', 'COMPLETE'):
            self.failure = reason
            self.phase = 'FAILED'
            self.zero()
            self.get_logger().error(reason)

    def start_stage(self, now_ns):
        self.stage_started_ns = now_ns
        self.stage_truth = self.truth[1]
        self.turn_last_yaw = self.stage_truth.yaw
        self.turn_progress = 0.0
        name, target = ROUTE[self.stage_index]
        self.get_logger().info(f'Stage {self.stage_index + 1}/{len(ROUTE)}: {name} {target:.3f}')

    def record_sample(self, now_ns):
        if not self.truth_fresh(now_ns):
            return
        if self.truth[0] == self.last_match_attempt_truth_ns:
            return
        self.last_match_attempt_truth_ns = self.truth[0]
        self.match_attempt_count += 1
        if not self.fresh(now_ns, self.estimate_topic):
            return
        matched = self.matched_sample(now_ns)
        if matched is None:
            return
        truth_ns, truth, estimates = matched
        if self.last_sample_ns is not None and truth_ns - self.last_sample_ns < 100_000_000:
            return
        self.last_sample_ns = truth_ns
        estimate, primary_side_ns, primary_span_ns = estimates[self.estimate_topic]
        self.max_interpolation_side_s = max(self.max_interpolation_side_s,
                                            primary_side_ns / 1e9)
        self.max_interpolation_span_s = max(self.max_interpolation_span_s,
                                            primary_span_ns / 1e9)
        aligned = align_pose(estimate, *self.initial[self.estimate_topic])
        position_error, heading_error = errors(aligned, truth)
        row = {
            'sim_time_s': round((truth_ns - self.started_sim_ns) / 1e9, 3),
            'stage_index': self.stage_index + 1,
            'segment': ROUTE[self.stage_index][0],
            'truth_x': truth.x, 'truth_y': truth.y, 'truth_yaw': truth.yaw,
            'estimate_x': aligned.x, 'estimate_y': aligned.y,
            'estimate_yaw': aligned.yaw,
            'position_error_m': position_error,
            'heading_error_rad': heading_error,
            'raw_position_error_m': None,
            'raw_heading_error_rad': None,
            'interpolation_side_s': primary_side_ns / 1e9,
            'interpolation_span_s': primary_span_ns / 1e9,
        }
        if self.mode == 'ekf_overlay' and '/odom' in self.initial:
            raw = align_pose(estimates['/odom'][0], *self.initial['/odom'])
            row['raw_position_error_m'], row['raw_heading_error_rad'] = errors(raw, truth)
        self.samples.append(row)
        if self.last_tf_stamp_ns is None or truth_ns - self.last_tf_stamp_ns > 500_000_000:
            self.tf_stale_sample_count += 1

    def step(self):
        if self.phase in ('FAILED', 'COMPLETE'):
            return
        if time.monotonic() - self.started_wall > self.wall_timeout:
            self.fail(f'Wall timeout after {self.wall_timeout:.0f} s')
            return
        now_ns = self.now_ns()
        if now_ns <= 0:
            return
        self.collect_truth()
        self.request_truth(now_ns)
        if self.last_command_ns is not None and now_ns - self.last_command_ns < 50_000_000:
            return
        self.last_command_ns = now_ns
        if self.phase == 'WAIT':
            if not (self.truth_fresh(now_ns) and self.fresh(now_ns, self.estimate_topic)):
                return
            # The trial must have sole authority over /cmd_vel.  No web mux or
            # teleop process may be active in this isolated simulation domain.
            publishers = self.get_publishers_info_by_topic('/cmd_vel')
            foreign = [item for item in publishers
                       if item.node_name != self.get_name()
                       or item.node_namespace != self.get_namespace()]
            if foreign:
                self.fail('Another /cmd_vel publisher is active; refusing to drive')
                return
            self.phase = 'SETTLE'
            self.settle_started_ns = now_ns
            self.get_logger().info('Truth and estimate ready; settling for 2 simulated seconds')
        if self.phase == 'SETTLE':
            self.zero()
            if now_ns - self.settle_started_ns < 2_000_000_000:
                return
            if not self.truth_fresh(now_ns) or not self.fresh(now_ns, self.estimate_topic):
                self.fail('Truth or estimate stopped during settle')
                return
            if not self.check_graph_authority():
                self.fail('Unexpected /odom or /tf publisher ownership: '
                          f'odom={self.odom_publishers}, tf={self.tf_publishers}')
                return
            self.last_graph_check_ns = now_ns
            matched = self.matched_sample(now_ns)
            if matched is None:
                return
            truth_ns, truth, estimates = matched
            for topic, (estimate, _side, _span) in estimates.items():
                self.initial[topic] = (estimate, truth)
            self.initial_truth = truth
            self.started_sim_ns = truth_ns
            self.started_run_wall = time.monotonic()
            self.last_distance_truth = truth
            self.tf_count_at_start = self.tf_count
            self.phase = 'RUN'
            self.start_stage(now_ns)
        if self.phase != 'RUN':
            return
        if (self.last_graph_check_ns is not None
                and now_ns - self.last_graph_check_ns >= 1_000_000_000):
            self.last_graph_check_ns = now_ns
            if not self.check_graph_authority():
                self.fail('Odometry/TF publisher ownership changed during motion: '
                          f'odom={self.odom_publishers}, tf={self.tf_publishers}')
                return
        if not self.truth_fresh(now_ns):
            self.fail('Gazebo ground truth became stale during motion')
            return
        if not self.fresh(now_ns, self.estimate_topic):
            self.fail(f'{self.estimate_topic} became stale during motion')
            return
        self.record_sample(now_ns)
        name, target = ROUTE[self.stage_index]
        if now_ns - self.stage_started_ns > (14 if name == 'straight' else 12) * 1e9:
            self.fail(f'Stage {self.stage_index + 1} did not make progress: {name}')
            return
        truth = self.truth[1]
        if name == 'straight':
            dx = truth.x - self.stage_truth.x
            dy = truth.y - self.stage_truth.y
            progress = dx * math.cos(self.stage_truth.yaw) + dy * math.sin(self.stage_truth.yaw)
            reached = progress >= target - .015
        else:
            reached = self.turn_progress >= target - .025
        if reached:
            self.zero()
            self.completed_stages += 1
            self.stage_index += 1
            if self.stage_index == len(ROUTE):
                self.phase = 'COMPLETE'
                self.get_logger().info('All eight controlled segments completed')
                return
            self.start_stage(now_ns)
            return
        if name == 'straight':
            self.command(.12, 0.0)
        else:
            self.command(0.0, .35)

    def result(self):
        rows = self.samples
        position = summarize_error_rows(rows)
        raw = summarize_error_rows(rows, 'raw_') if self.mode == 'ekf_overlay' else None
        truth = self.truth[1] if self.truth else None
        initial = getattr(self, 'initial_truth', None)
        duration = ((self.truth[0] - self.started_sim_ns) / 1e9
                    if self.truth and self.started_sim_ns else None)
        active_tf_count = max(0, self.tf_count - self.tf_count_at_start)
        wall_now = time.monotonic()
        wall_active_duration = (wall_now - self.started_run_wall
                                if self.started_run_wall is not None else None)
        lags = self.estimate_header_lags_s
        p95_lag = (sorted(lags)[min(len(lags) - 1, math.ceil(.95 * len(lags)) - 1)]
                   if lags else None)
        return {
            'schema_version': 1,
            'mode': self.mode,
            'estimate_topic': self.estimate_topic,
            'truth_topic': TRUTH_TOPIC,
            'truth_model': self.model_name,
            'trial_status': self.phase,
            'failure': self.failure,
            'completed_stages': self.completed_stages,
            'total_stages': len(ROUTE),
            'route_completion_fraction': self.completed_stages / len(ROUTE),
            'position': position,
            'raw_odom_in_overlay': raw,
            'truth_distance_m': self.truth_distance_m,
            'truth_updates_during_run': self.truth_updates_during_run,
            'final_truth_distance_from_start_m': (math.hypot(truth.x - initial.x,
                                                              truth.y - initial.y)
                                                  if truth and initial else None),
            'final_truth_heading_from_start_deg': (math.degrees(abs(wrap_angle(
                truth.yaw - initial.yaw))) if truth and initial else None),
            'tf_odom_base_count': self.tf_count,
            'tf_odom_base_rate_hz': active_tf_count / duration if duration and duration > 0 else None,
            'tf_max_gap_s': self.tf_max_gap_s if self.tf_count > 1 else None,
            'tf_duplicate_stamp_count': self.tf_duplicate_stamp_count,
            'tf_conflict_count': self.tf_conflict_count,
            'tf_stale_sample_count': self.tf_stale_sample_count,
            'odom_publishers': self.odom_publishers,
            'tf_publishers': self.tf_publishers,
            'graph_authority_ok': self.graph_authority_ok,
            'graph_authority_limit': ('TFMessage has no publisher ID; unexpected third-party '
                                      'nodes publishing the same frame cannot be excluded '
                                      'from message content alone'),
            'max_interpolation_side_s': self.max_interpolation_side_s,
            'max_interpolation_span_s': self.max_interpolation_span_s,
            'command_count': self.command_count,
            'sim_duration_s': duration,
            'wall_active_duration_s': wall_active_duration,
            'sim_realtime_factor': (duration / wall_active_duration
                                    if duration and wall_active_duration else None),
            'matched_sample_rate_hz': len(rows) / duration if duration and duration > 0 else None,
            'match_attempt_count': self.match_attempt_count,
            'matched_sample_fraction': (len(rows) / self.match_attempt_count
                                        if self.match_attempt_count else None),
            'last_matched_age_s': ((self.truth[0] - self.last_sample_ns) / 1e9
                                   if self.truth and self.last_sample_ns else None),
            'estimate_header_lag_median_s': (statistics.median(lags) if lags else None),
            'estimate_header_lag_p95_s': p95_lag,
            'wall_duration_s': wall_now - self.started_wall,
        }


def write_result(prefix, trial):
    prefix.parent.mkdir(parents=True, exist_ok=True)
    result = trial.result()
    with prefix.with_suffix('.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=SAMPLE_COLUMNS,
                                lineterminator='\n')
        writer.writeheader()
        writer.writerows(trial.samples)
    prefix.with_suffix('.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=MODES, required=True)
    parser.add_argument('--output-prefix', required=True, type=Path)
    parser.add_argument('--model-name', default='waffle_pi')
    parser.add_argument('--wall-timeout', type=float, default=150.0)
    args = parser.parse_args(argv)
    rclpy.init()
    trial = BenchmarkTrial(args.mode, args.model_name, args.wall_timeout)
    try:
        while rclpy.ok() and trial.phase not in ('FAILED', 'COMPLETE'):
            rclpy.spin_once(trial, timeout_sec=.05)
            trial.step()
    except KeyboardInterrupt:
        trial.fail('Interrupted')
    finally:
        for _ in range(5):
            trial.zero()
            rclpy.spin_once(trial, timeout_sec=.02)
        result = write_result(args.output_prefix, trial)
        trial.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if (result['trial_status'] == 'COMPLETE'
                 and result['position']['sample_count'] >= 100
                 and result['matched_sample_rate_hz'] is not None
                 and result['matched_sample_rate_hz'] >= 5
                 and result['matched_sample_fraction'] is not None
                 and result['matched_sample_fraction'] >= .4
                 and result['last_matched_age_s'] is not None
                 and result['last_matched_age_s'] <= .5
                 and result['truth_distance_m'] >= 1.8
                 and result['sim_duration_s'] is not None
                 and result['sim_duration_s'] >= 20
                 and result['max_interpolation_side_s'] <= .11
                 and result['max_interpolation_span_s'] <= .12
                 and result['tf_odom_base_count'] >= 2
                 and result['tf_odom_base_rate_hz'] is not None
                 and result['tf_odom_base_rate_hz'] <= 45
                 and result['graph_authority_ok']
                 and result['tf_conflict_count'] == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
