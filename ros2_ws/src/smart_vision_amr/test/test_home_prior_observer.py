"""No ROS or robot commands: asynchronous Home evidence lifetime contracts."""
from concurrent.futures import Future
import unittest

import numpy as np

from smart_vision_amr.home_prior_observer import HomePriorObserver


class PendingExecutor:
    def __init__(self):
        self.jobs = []

    def submit(self, function, *args):
        future = Future()
        self.jobs.append((future, function, args))
        return future


class HomePriorObserverTests(unittest.TestCase):
    def setUp(self):
        self.executor = PendingExecutor()
        self.observer = HomePriorObserver(lambda *args: {'supported': True}, self.executor)
        self.pose = {'x': 0., 'y': 0., 'yaw': 0.}
        self.speed = {'linear': 0., 'angular': 0.}
        self.points = np.ones((20, 2))
        self.key = (1, (0., 0., 0.))

    def tearDown(self):
        self.observer.close()

    def scan(self, now, stamp=None, eligible=True):
        return self.observer.observe(int(now*1e9) if stamp is None else stamp,
            self.points, self.key, self.pose, {}, np.zeros((2, 2)), np.zeros((2, 2)),
            self.pose, self.speed, now, now, eligible)

    def pending(self):
        for now in (10., 10.25, 10.5):
            self.assertIsNone(self.scan(now))
        self.assertEqual(len(self.executor.jobs), 1)

    def test_callbacks_do_not_wait_and_only_one_search_is_pending(self):
        self.pending()
        self.scan(10.8)
        self.assertEqual(len(self.executor.jobs), 1)
        self.assertFalse(self.executor.jobs[0][0].done())

    def test_distinct_scans_and_separated_observations_required(self):
        for now in (10., 10.1, 10.25, 10.5):
            self.scan(now, stamp=1)
        self.assertEqual(self.executor.jobs, [])
        self.scan(10.75, stamp=2)
        self.scan(11., stamp=3)
        self.assertEqual(len(self.executor.jobs), 1)

    def test_result_carries_sensor_time_and_expires_without_new_evidence(self):
        self.pending()
        self.executor.jobs[0][0].set_result({'supported': True})
        result = self.scan(10.7)
        self.assertTrue(result['supported'])
        self.assertEqual(result['at'], 10.5)
        self.assertIsNone(self.scan(11.31))

    def test_slow_search_cannot_refresh_old_scans(self):
        self.pending()
        self.executor.jobs[0][0].set_result({'supported': True})
        self.assertIsNone(self.scan(11.31))

    def test_motion_invalidates_even_if_robot_returns_before_search_finishes(self):
        self.pending()
        self.observer.observe_odom(dict(self.pose, x=.03), self.speed)
        self.executor.jobs[0][0].set_result({'supported': True})
        self.assertIsNone(self.scan(10.7))

    def test_speed_and_map_change_invalidate_pending_result(self):
        for change in ('speed', 'map'):
            with self.subTest(change=change):
                self.setUp()
                self.pending()
                if change == 'speed':
                    self.observer.observe_odom(self.pose, {'linear': .02, 'angular': 0.})
                else:
                    self.key = (2, (0., 0., 0.))
                self.executor.jobs[0][0].set_result({'supported': True})
                self.assertIsNone(self.scan(10.7))
                self.observer.close()

    def test_ineligible_scan_clears_old_evidence_and_pending_generation(self):
        self.pending()
        self.scan(10.6, eligible=False)
        self.executor.jobs[0][0].set_result({'supported': True})
        self.assertIsNone(self.scan(10.8))

    def test_failed_worker_is_no_evidence(self):
        self.pending()
        self.executor.jobs[0][0].set_exception(ValueError('invalid geometry'))
        self.assertIsNone(self.scan(10.7))

    def test_stale_odometry_cannot_accumulate_proof(self):
        self.observer.observe(1, self.points, self.key, self.pose, {}, None, None,
                              self.pose, self.speed, 1., 10., True)
        self.assertEqual(self.observer.observations, [])
        self.assertIsNone(self.observer.evidence)
