"""Bounded Home-prior evidence computed away from ROS sensor callbacks.

This observer publishes no pose or command. A supported neighborhood only
permits the unchanged, broad registered-Home prior; AMCL must still converge.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import math


class HomePriorObserver:
    def __init__(self, matcher, executor=None):
        self.matcher = matcher
        self.executor = executor
        self._owned_executor = executor is None
        self.closed = False
        self.generation = 0
        self.key = None
        self.anchor = None
        self.observations = []
        self.last_stamp = 0
        self.last_sample_at = 0.
        self.last_attempt_at = float('-inf')
        self.future = None
        self.pending = None
        self.evidence = None
        self.evidence_at = 0.

    @staticmethod
    def same_stationary_pose(pose, anchor):
        if not pose or not anchor:
            return False
        try:
            values = [pose[k] for k in ('x', 'y', 'yaw')]
            other = [anchor[k] for k in ('x', 'y', 'yaw')]
            if not all(math.isfinite(v) for v in values+other):
                return False
            dyaw = math.atan2(math.sin(values[2]-other[2]), math.cos(values[2]-other[2]))
            return math.hypot(values[0]-other[0], values[1]-other[1]) < .02 and abs(dyaw) < .03
        except (KeyError, TypeError, ValueError):
            return False

    def invalidate(self):
        self.generation += 1
        self.observations.clear()
        self.evidence = None
        self.evidence_at = 0.
        self.anchor = None

    def observe_odom(self, pose, speed):
        try:
            stationary = (all(math.isfinite(speed[k]) for k in ('linear', 'angular'))
                          and abs(speed['linear']) < .01 and abs(speed['angular']) < .03)
        except (KeyError, TypeError, ValueError):
            stationary = False
        if not stationary or (self.anchor is not None and not self.same_stationary_pose(pose, self.anchor)):
            self.invalidate()
        return stationary

    def observe(self, stamp, points, key, home, metadata, distance, occupancy, pose,
                speed, odom_at, now, eligible):
        if self.closed:
            return None
        stationary = self.observe_odom(pose, speed)
        if not eligible or not stationary or not pose or not 0 <= now-odom_at < .8:
            self.invalidate()
            return None
        if key != self.key:
            self.invalidate()
            self.key = key
        if self.anchor is None:
            self.anchor = deepcopy(pose)
        if self.future is not None and self.future.done():
            pending, future = self.pending, self.future
            self.pending = self.future = None
            try:
                evidence = future.result()
            except Exception:
                evidence = None
            if (pending['generation'] == self.generation and pending['key'] == key
                    and self.same_stationary_pose(pose, pending['anchor'])
                    and 0 <= now-pending['at'] < .8 and isinstance(evidence, dict)):
                self.evidence = dict(evidence)
                self.evidence_at = pending['at']
        # Require actual distinct sensor stamps, separated observations and a
        # short acquisition window. Replayed scans cannot count as new proof.
        if type(stamp) is int and stamp > self.last_stamp and now-self.last_sample_at >= .2:
            self.last_stamp = stamp
            self.last_sample_at = now
            self.observations.append((stamp, points.copy(), now))
        self.observations = [o for o in self.observations if 0 <= now-o[2] <= 1.2][-3:]
        if (self.future is None and len(self.observations) == 3
                and self.observations[-1][2]-self.observations[0][2] >= .4-1e-9
                and now-self.last_attempt_at >= .6):
            if self.executor is None:
                self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='home-prior')
            self.pending = {'generation': self.generation, 'key': key,
                            'anchor': deepcopy(self.anchor), 'at': self.observations[-1][2]}
            self.last_attempt_at = now
            observations = [(s, p) for s, p, _ in self.observations]
            self.future = self.executor.submit(self.matcher, observations, deepcopy(home),
                                               deepcopy(metadata), distance, occupancy)
        if self.evidence is None or not 0 <= now-self.evidence_at < .8:
            return None
        return dict(self.evidence, at=self.evidence_at, key=self.key)

    def close(self):
        self.closed = True
        self.invalidate()
        if self.future is not None:
            self.future.cancel()
        if self.executor is not None and self._owned_executor:
            self.executor.shutdown(wait=False, cancel_futures=True)
