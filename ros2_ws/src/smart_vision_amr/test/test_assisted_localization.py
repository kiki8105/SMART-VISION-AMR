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

    def test_settling_requires_current_healthy_guard_and_actual_stop(self):
        self.rotate_until_settling()
        for _ in range(12):
            self.now += .1
            self.logic.on_odom(self.yaw)
            self.logic.on_estimate(1., 2., self.yaw, .1, .1)
            self.assertEqual(self.logic.tick(settled_health_ok=False), 0.)
        self.assertEqual(self.logic.state, 'SETTLING')
        self.assertEqual(self.logic.tick(stationary=False), 0.)
        self.assertFalse(self.logic.succeeded)
        self.logic.tick(settled_health_ok=True, stationary=True)
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


class HomeHeadingLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.yaw = 0.
        self.target = {'x': 1., 'y': 2., 'yaw': 0.}
        self.logic = AssistedLocalization(clock=lambda: self.now)
        self.logic.start(home_target=self.target)
        self.logic.acquired()
        self.logic.initialized()
        # Odom's origin is deliberately different from map yaw. Only measured
        # AMCL map heading may establish registered Home direction.
        self.logic.on_odom(1.2)

    def step(self, *, estimate=True, quality_ok=True, stationary=True, health=True, x=1.):
        self.now += .1
        self.yaw = math.atan2(math.sin(self.yaw+self.logic.angular*.1),
                              math.cos(self.yaw+self.logic.angular*.1))
        self.logic.on_odom(self.yaw+1.2)
        if estimate:
            self.logic.on_estimate(x, 2., self.yaw, .1, .1, quality_ok=quality_ok)
        return self.logic.tick(stationary=stationary, settled_health_ok=health)

    def until(self, state):
        for _ in range(600):
            self.step()
            if self.logic.state == state:
                return
            self.assertTrue(self.logic.active, self.logic.reason)
        self.fail('Expected '+state)

    def test_home_sweep_returns_to_map_heading_before_fresh_stationary_success(self):
        self.until('RETURNING')
        self.assertGreater(self.yaw, 1.5)
        self.assertEqual(self.logic.angular, 0., 'phase entry cannot command unacknowledged return')
        self.assertFalse(self.logic.succeeded)
        self.assertLess(self.step(), 0.)
        self.until('SETTLING')
        self.assertLessEqual(abs(self.yaw), .10)
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)
        for _ in range(12):
            self.assertEqual(self.step(estimate=False), 0.)
        self.assertEqual(self.logic.state, 'SETTLING', 'sweep samples cannot prove final Home pose')
        for _ in range(30):
            self.step(stationary=False)
        self.assertFalse(self.logic.succeeded, 'fresh AMCL still needs actual stop')
        for _ in range(30):
            self.step(health=False)
        self.assertFalse(self.logic.succeeded, 'warning health cannot complete')
        self.step()
        self.assertTrue(self.logic.succeeded)
        self.assertLess(self.logic.rotation, 2*math.pi)
        self.assertEqual(self.logic.snapshot()['home_target'], self.target)

    def test_home_target_is_frozen_and_bad_coordinates_rejected(self):
        self.target['yaw'] = 2.
        self.assertEqual(self.logic.home_target['yaw'], 0.)
        for target in ({'x': 0., 'y': 0.}, {'x': 0., 'y': 0., 'yaw': math.nan},
                       {'x': True, 'y': 0., 'yaw': 0.}):
            logic = AssistedLocalization()
            with self.assertRaises(ValueError):
                logic.start(home_target=target)

    def test_return_requires_actual_fresh_qualified_amcl_not_the_prior(self):
        self.until('RETURNING')
        self.step(quality_ok=False)
        self.assertEqual(self.logic.angular, 0.)
        self.assertFalse(self.logic.succeeded)
        self.logic.pause_for_sensor_gap()
        self.assertIsNone(self.logic.estimate)
        self.assertEqual(self.logic.tick(), 0.)
        self.step(estimate=False)
        self.assertEqual(self.logic.angular, 0.)
        self.assertEqual(self.logic.state, 'RETURNING')

    def test_return_stale_amcl_cannot_keep_turning_on_new_odometry(self):
        self.until('RETURNING')
        for _ in range(27):
            self.step(estimate=False)
        self.assertEqual(self.logic.angular, 0.)
        self.assertFalse(self.logic.succeeded)

    def test_wrong_home_xy_fails_without_attempting_translation(self):
        self.until('RETURNING')
        self.step(x=1.31)
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertEqual(self.logic.angular, 0.)

    def test_post_stop_heading_drift_returns_again_and_resets_confidence(self):
        self.until('SETTLING')
        self.step()  # Establish the measured-stop interval before new evidence.
        self.yaw += .05
        self.step()
        self.assertEqual(self.logic.state, 'RETURNING')
        self.assertEqual(self.logic.angular, 0.)
        self.until('SETTLING')
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)
        self.assertFalse(self.logic.succeeded)

    def test_full_rotation_budget_never_allows_extra_home_return(self):
        self.until('RETURNING')
        self.logic.rotation = self.logic.max_rotation
        self.assertEqual(self.step(), 0.)
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertIn('회전 한도', self.logic.reason)

    def test_home_heading_at_full_budget_may_settle_without_another_turn(self):
        self.until('RETURNING')
        self.logic.rotation = self.logic.max_rotation
        self.now += .1
        self.logic.on_odom(self.yaw+1.2)
        self.logic.on_estimate(1., 2., 0., .1, .1)
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.state, 'SETTLING')
        self.assertEqual(self.logic.snapshot()['good_estimates'], 0)

    def test_return_uses_original_deadline_and_stops_on_clearance_loss(self):
        self.until('RETURNING')
        original_start = self.logic.started_at
        self.logic.pause_for_sensor_gap()
        self.now = original_start+self.logic.timeout
        self.assertEqual(self.logic.tick(), 0.)
        self.assertEqual(self.logic.state, 'FAILED')
        self.assertEqual(self.logic.started_at, original_start)

    def test_return_cancellation_never_restarts_from_late_measurements(self):
        self.until('RETURNING')
        self.assertTrue(self.logic.cancel())
        self.assertEqual(self.step(), 0.)
        self.assertEqual(self.logic.state, 'CANCELED')
        self.assertFalse(self.logic.on_estimate(1., 2., 0., .01, .01))


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
