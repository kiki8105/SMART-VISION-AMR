"""A localization button cannot move on stale data or claim guessed poses."""

import math
import unittest

from smart_vision_amr.assisted_localization import (
    AssistedLocalization, rotation_scan_clearance,
)


class AssistedLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.yaw = 0.0
        self.logic = AssistedLocalization(clock=lambda: self.now)

    def initialize(self):
        self.assertTrue(self.logic.start())
        self.assertEqual(self.logic.tick(), 0.0)
        self.assertTrue(self.logic.acquired())
        self.assertEqual(self.logic.tick(), 0.0)
        self.assertTrue(self.logic.initialized())
        self.assertEqual(self.logic.tick(), 0.0)  # Wait for a new odom baseline.
        self.assertTrue(self.logic.on_odom(self.yaw))

    def step(self, seconds=.1, *, moving=True, estimate=True, quality_ok=True,
             position_std=.1, yaw_std=.1):
        self.now += seconds
        if moving:
            self.yaw = math.atan2(math.sin(self.yaw+.2*seconds),
                                  math.cos(self.yaw+.2*seconds))
        self.logic.on_odom(self.yaw)
        if estimate:
            self.logic.on_estimate(1.0, 2.0, self.yaw, position_std, yaw_std,
                                   quality_ok=quality_ok)
        return self.logic.tick()

    def rotate_until_settling(self):
        self.initialize()
        for _ in range(90):
            self.step()
            if self.logic.state == 'SETTLING':
                return
        self.fail('Expected convergence after at least ninety measured degrees')

    def test_acquisition_and_initialization_acknowledgments_are_required(self):
        self.assertFalse(self.logic.initialized())
        self.assertFalse(self.logic.acquired())
        self.assertTrue(self.logic.start())
        self.assertFalse(self.logic.start())
        self.assertFalse(self.logic.on_odom(0.0))
        self.assertFalse(self.logic.on_estimate(1., 2., 0., .01, .01))
        self.assertEqual(self.logic.tick(), 0.0)
        self.assertIsNone(self.logic.snapshot()['estimate'])

    def test_actual_rotation_good_covariance_and_scan_match_are_required(self):
        self.rotate_until_settling()
        self.assertGreaterEqual(self.logic.rotation, math.pi/2)
        self.assertLess(self.logic.rotation, 2*math.pi)
        self.assertEqual(self.logic.angular, 0.0)
        self.assertFalse(self.logic.succeeded)
        for _ in range(12):
            self.assertEqual(self.step(moving=False), 0.0)
        self.assertTrue(self.logic.succeeded)
        self.assertEqual(self.logic.snapshot()['estimate']['x'], 1.0)
        self.assertEqual(self.logic.tick(), 0.0)

    def test_covariance_alone_never_declares_success(self):
        self.initialize()
        for _ in range(350):
            self.step(quality_ok=False)
        self.assertEqual(self.logic.state, 'SETTLING')
        self.assertGreaterEqual(self.logic.rotation, 2*math.pi)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)
        self.assertFalse(self.logic.succeeded)

    def test_old_preinitialization_estimates_and_odometry_are_not_reused(self):
        self.logic.start()
        self.logic.acquired()
        self.logic.on_estimate(7., 8., 0., .001, .001)
        self.logic.on_odom(0.)
        self.logic.initialized()
        self.assertIsNone(self.logic.estimate)
        self.assertIsNone(self.logic.last_odom_at)
        self.assertEqual(self.logic.rotation, 0.)
        self.assertEqual(self.logic.tick(), 0.)

    def test_yaw_wrap_is_a_small_real_motion(self):
        self.yaw = math.pi-.01
        self.initialize()
        self.step(estimate=False)
        self.assertAlmostEqual(self.logic.rotation, .02)
        self.assertEqual(self.logic.state, 'ROTATING')

    def test_odom_jump_fails_instead_of_completing_a_sweep(self):
        self.initialize()
        self.now += .1
        self.assertFalse(self.logic.on_odom(2.0))
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertEqual(self.logic.rotation, 0.)
        self.assertEqual(self.logic.tick(), 0.)

    def test_duplicate_and_out_of_order_receipts_cannot_accumulate_rotation(self):
        self.initialize()
        self.assertFalse(self.logic.on_odom(.1, now=self.now))
        self.assertFalse(self.logic.on_odom(.1, now=self.now-.1))
        self.assertEqual(self.logic.rotation, 0.)

    def test_stale_odom_is_not_repaired_by_a_late_update(self):
        self.initialize()
        self.now += 1.01
        self.assertFalse(self.logic.on_odom(.05))
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertEqual(self.logic.tick(), 0.)

    def test_stale_odom_immediately_zeroes_rotation(self):
        self.initialize()
        self.assertEqual(self.logic.tick(), .2)
        self.now += 1.01
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.state, 'FAILED')

    def test_sensor_gap_pause_discards_unobserved_yaw_and_old_confidence(self):
        self.initialize()
        self.step()
        before_rotation = self.logic.rotation
        self.assertGreater(self.logic.snapshot()['good_estimates'], 0)
        self.now += .9
        self.logic.pause_for_sensor_gap(self.now)
        self.assertEqual(self.logic.angular, 0.)
        self.assertIsNone(self.logic.last_odom_at)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)
        self.assertIsNone(self.logic.estimate)
        self.logic.on_odom(self.yaw + .2, self.now + .01)
        self.assertEqual(self.logic.rotation, before_rotation)

    def test_missing_odom_after_initialization_never_turns(self):
        self.logic.start()
        self.logic.acquired()
        self.logic.initialized()
        self.now += 1.01
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.state, 'FAILED')

    def test_stale_scan_or_unsafe_clearance_zeroes_motion(self):
        for key in ('sensors_ready', 'rotation_clear'):
            self.setUp()
            self.initialize()
            self.assertEqual(self.logic.tick(), .2)
            self.assertEqual(self.logic.tick(**{key: False}), 0.)
            self.assertEqual(self.logic.state, 'FAILED')

    def test_stationary_odometry_cannot_fake_sufficient_observation(self):
        self.initialize()
        for _ in range(65):
            self.step(moving=False)
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertEqual(self.logic.rotation, 0.)
        self.assertEqual(self.logic.tick(), 0.)

    def test_only_one_sweep_is_allowed_when_uncertain(self):
        self.initialize()
        for _ in range(330):
            self.step(position_std=2.0)
        self.assertEqual(self.logic.state, 'SETTLING')
        rotation = self.logic.rotation
        for _ in range(20):
            self.assertEqual(self.step(moving=False, position_std=2.0), 0.)
        self.assertEqual(self.logic.rotation, rotation)
        self.assertFalse(self.logic.succeeded)

    def test_no_convergence_times_out_and_preserves_failure_message(self):
        self.initialize()
        for _ in range(660):
            moving = self.logic.state == 'ROTATING'
            self.step(moving=moving, position_std=2.0)
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertIn('제한 시간', self.logic.reason)
        reason = self.logic.reason
        self.now += 100
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.reason, reason)

    def test_settling_requires_a_post_stop_amcl_update(self):
        self.rotate_until_settling()
        for _ in range(12):
            self.step(moving=False, estimate=False)
        self.assertEqual(self.logic.state, 'SETTLING')
        self.step(moving=False)
        self.assertTrue(self.logic.succeeded)

    def test_late_estimate_cannot_inherit_an_old_stable_window(self):
        self.rotate_until_settling()
        for _ in range(30):
            self.step(moving=False, estimate=False)
        self.step(moving=False)
        self.assertFalse(self.logic.succeeded)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 1)

    def test_bad_covariance_or_scan_match_resets_stability(self):
        self.initialize()
        for _ in range(70):
            self.step()
        self.assertGreater(self.logic.snapshot()['good_estimates'], 5)
        self.step(quality_ok=False)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)
        self.step()
        self.assertEqual(self.logic.snapshot()['good_estimates'], 1)
        self.step(position_std=1.)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)

    def test_pose_teleport_resets_stability_without_fabricating_a_pose(self):
        self.initialize()
        for _ in range(60):
            self.step()
        self.now += .1
        self.logic.on_odom(self.yaw)
        self.logic.on_estimate(3., 4., self.yaw, .1, .1)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 1)
        self.assertEqual(self.logic.estimate['x'], 3.)
        self.assertFalse(self.logic.succeeded)

    def test_cancel_and_restart_need_new_explicit_acquisition(self):
        self.initialize()
        self.assertTrue(self.logic.cancel())
        self.assertEqual(self.logic.state, 'CANCELED')
        self.assertEqual(self.logic.tick(), 0.)
        self.assertFalse(self.logic.cancel())
        self.assertTrue(self.logic.start())
        self.assertEqual(self.logic.state, 'ACQUIRING')
        self.assertIsNone(self.logic.estimate)
        self.assertEqual(self.logic.rotation, 0.)

    def test_cancellation_and_service_timeouts_are_bounded(self):
        self.logic.start()
        self.now += 8.
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.state, 'FAILED')
        self.logic.start()
        self.logic.acquired()
        self.now += 10.
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.state, 'FAILED')

    def test_invalid_covariance_and_nan_pose_never_count_as_confident(self):
        self.initialize()
        for values in ((math.nan, 2., 0., .1, .1),
                       (1., 2., 0., math.inf, .1),
                       (1., 2., 0., -.1, .1)):
            self.now += .1
            self.assertFalse(self.logic.on_estimate(*values))
            self.assertEqual(self.logic.snapshot()['good_estimates'], 0)

    def test_invalid_config_and_nonfinite_clock_are_rejected(self):
        for kwargs in ({'angular_speed': 1.}, {'timeout': 0.},
                       {'max_rotation': 7.}, {'min_rotation': 7.},
                       {'min_estimates': 1}, {'sensor_timeout': math.nan}):
            with self.assertRaises(ValueError):
                AssistedLocalization(**kwargs)
        with self.assertRaises(ValueError):
            self.logic.start(now=math.nan)


class RotationClearanceTest(unittest.TestCase):
    def test_room_has_enough_measured_returns_and_margin(self):
        self.assertEqual(rotation_scan_clearance([1.]*360, .12, 3.5), (True, 1.))

    def test_single_close_side_or_rear_return_blocks_rotation(self):
        scan = [1.]*360
        scan[270] = .29
        self.assertEqual(rotation_scan_clearance(scan, .12, 3.5), (False, .29))

    def test_all_infinity_nan_or_too_few_measured_returns_are_unsafe(self):
        for scan in ([math.inf]*360, [math.nan]*360, [1.]*7+[math.inf]*353):
            self.assertFalse(rotation_scan_clearance(scan, .12, 3.5)[0])

    def test_zero_below_sensor_minimum_and_negative_returns_are_not_open(self):
        for invalid in (0., .01, .11, -1., -math.inf):
            self.assertEqual(rotation_scan_clearance([1.]*359+[invalid], .12, 3.5),
                             (False, 0.))

    def test_out_of_range_returns_are_not_measured_clearance(self):
        self.assertEqual(rotation_scan_clearance([4.]*360, .12, 3.5), (False, None))


if __name__ == '__main__':
    unittest.main()
