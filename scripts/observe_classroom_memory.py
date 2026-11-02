#!/usr/bin/env python3
"""Record read-only evidence that LiDAR memory survives between sector goals.

Example (in the Gazebo ROS domain)::

    python3 scripts/observe_classroom_memory.py --output /tmp/memory_trial.jsonl

Stop with Ctrl+C. The adjacent ``.summary.json`` records whether a confirmed
cell remained after its live hit disappeared, survived a *different* goal, and
was subsequently removed. A removal is only labeled as having free-ray
evidence when fresh ``observed_visibility`` messages marked that cell free.
This observer never publishes motion or changes memory.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray


class MemoryTimeline:
    """Summarize observed transitions without inferring free space from silence."""

    def __init__(self, stream, *, monotonic=time.monotonic, utc_now=None):
        self.stream = stream
        self.monotonic = monotonic
        self.utc_now = utc_now or (lambda: datetime.now(timezone.utc).isoformat())
        self.started = self.monotonic()
        self.observed: set[int] = set()
        self.have_observed = False
        self.remembered: set[int] = set()
        self.active: dict[int, dict] = {}
        self.episodes: list[dict] = []
        self.goal_key = None
        self.goal_sequence = 0
        self.visibility_samples = 0
        self.observed_samples = 0
        self._write('start', domain_id=os.environ.get('ROS_DOMAIN_ID', 'default'))

    def _write(self, kind, **details):
        item = {'kind': kind, 'monotonic_sec': self.monotonic(),
                'wall_utc': self.utc_now(), **details}
        self.stream.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')
        self.stream.flush()
        return item

    def on_observed(self, indexes):
        self.observed_samples += 1
        self.observed = {int(index) for index in indexes if index >= 0}
        self.have_observed = True
        now = self.monotonic()
        for index, episode in self.active.items():
            if index in self.observed:
                episode['ever_seen_live'] = True
            elif episode['ever_seen_live'] and episode['out_of_view_at'] is None:
                episode['out_of_view_at'] = now
                self._write('remembered_after_live_disappeared', cell_index=index,
                            goal_sequence=self.goal_sequence)
        self._write('observed', count=len(self.observed), cells=sorted(self.observed),
                    remembered_count=len(self.remembered), sample=self.observed_samples)

    def on_memory(self, indexes):
        updated = {int(index) for index in indexes if index >= 0}
        added = updated - self.remembered
        removed = self.remembered - updated
        now = self.monotonic()
        for index in sorted(added):
            episode = {'cell_index': index, 'confirmed_at': now,
                       'goal_at_confirmation': self.goal_key,
                       'ever_seen_live': self.have_observed and index in self.observed,
                       'out_of_view_at': None, 'later_goals': [],
                       'free_ray_samples': 0, 'first_free_ray_at': None,
                       'cleared_at': None}
            self.active[index] = episode
            self.episodes.append(episode)
        for index in sorted(removed):
            episode = self.active.pop(index, None)
            if episode is not None:
                episode['cleared_at'] = now
                self._write('remembered_removed', cell_index=index,
                            free_ray_samples=episode['free_ray_samples'],
                            positive_free_evidence=episode['free_ray_samples'] > 0)
        self.remembered = updated
        self._write('remembered', count=len(updated), cells=sorted(updated),
                    added=sorted(added), removed=sorted(removed),
                    observed_count=len(self.observed) if self.have_observed else None)

    def on_visibility(self, data, stamp_ns):
        self.visibility_samples += 1
        free = [index for index in sorted(self.active)
                if index < len(data) and data[index] == 0]
        now = self.monotonic()
        for index in free:
            episode = self.active[index]
            episode['free_ray_samples'] += 1
            if episode['first_free_ray_at'] is None:
                episode['first_free_ray_at'] = now
        if free:
            self._write('remembered_cells_seen_free', cells=free, count=len(free),
                        scan_stamp_ns=stamp_ns, sample=self.visibility_samples)

    def on_goal(self, x, y, yaw_rad, frame_id, stamp_ns):
        # Consecutive republishes of one destination are not new sector goals.
        key = (frame_id, round(x, 3), round(y, 3), round(yaw_rad, 3))
        if key == self.goal_key:
            return
        self.goal_key = key
        self.goal_sequence += 1
        now = self.monotonic()
        carried = []
        for index, episode in self.active.items():
            if (episode['goal_at_confirmation'] is not None
                    and episode['goal_at_confirmation'] != key
                    and episode['out_of_view_at'] is not None
                    and episode['out_of_view_at'] <= now):
                episode['later_goals'].append(self.goal_sequence)
                carried.append(index)
        self._write('goal', sequence=self.goal_sequence, x=x, y=y, yaw_rad=yaw_rad,
                    frame_id=frame_id, stamp_ns=stamp_ns,
                    remembered_count=len(self.remembered),
                    carried_out_of_view_cells=sorted(carried))

    def summary(self):
        episodes = [dict(episode) for episode in self.episodes]
        for episode in episodes:
            episode['persisted_after_live_disappeared'] = episode['out_of_view_at'] is not None
            episode['carried_into_later_goal'] = bool(episode['later_goals'])
            episode['removed_with_positive_free_evidence'] = (
                episode['cleared_at'] is not None and episode['free_ray_samples'] > 0)
        return {
            'duration_monotonic_sec': round(self.monotonic() - self.started, 3),
            'domain_id': os.environ.get('ROS_DOMAIN_ID', 'default'),
            'observed_samples': self.observed_samples,
            'visibility_samples': self.visibility_samples,
            'goal_count': self.goal_sequence,
            'confirmed_episodes': len(episodes),
            'persisted_after_live_disappeared': sum(
                item['persisted_after_live_disappeared'] for item in episodes),
            'carried_into_later_goal': sum(item['carried_into_later_goal'] for item in episodes),
            'removed_with_positive_free_evidence': sum(
                item['removed_with_positive_free_evidence'] for item in episodes),
            'still_remembered': sorted(self.remembered),
            'episodes': episodes,
        }


class MemoryObserver(Node):
    def __init__(self, timeline):
        super().__init__('classroom_memory_observer')
        self.timeline = timeline
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Int32MultiArray, '/smart_vision_amr/remembered_cells',
                                 lambda msg: timeline.on_memory(msg.data), latched)
        self.create_subscription(Int32MultiArray, '/smart_vision_amr/observed_obstacles',
                                 lambda msg: timeline.on_observed(msg.data), 10)
        self.create_subscription(OccupancyGrid, '/smart_vision_amr/observed_visibility',
                                 self.on_visibility, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/smart_vision_amr/goal', self.on_goal, 10)

    def on_visibility(self, msg):
        stamp = msg.header.stamp
        self.timeline.on_visibility(msg.data, stamp.sec * 1_000_000_000 + stamp.nanosec)

    def on_goal(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp
        self.timeline.on_goal(msg.pose.position.x, msg.pose.position.y, yaw,
                              msg.header.frame_id,
                              stamp.sec * 1_000_000_000 + stamp.nanosec)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='JSONL event file')
    parser.add_argument('--summary', type=Path, help='summary JSON (default: OUTPUT.summary.json)')
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    summary_file = (args.summary.expanduser().resolve() if args.summary else
                    output.with_suffix('.summary.json'))
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_file.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8', buffering=1) as stream:
        timeline = MemoryTimeline(stream)
        rclpy.init()
        node = MemoryObserver(timeline)
        print(f'Memory observer listening in ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "default")}; '
              f'events={output}; Ctrl+C writes {summary_file}', flush=True)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            report = timeline.summary()
            summary_file.write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                               allow_nan=False) + '\n', encoding='utf-8')
            timeline._write('stop', summary_file=str(summary_file),
                            confirmed_episodes=report['confirmed_episodes'])
            print(json.dumps({key: report[key] for key in (
                'confirmed_episodes', 'persisted_after_live_disappeared',
                'carried_into_later_goal', 'removed_with_positive_free_evidence',
                'still_remembered')}, ensure_ascii=False), flush=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
