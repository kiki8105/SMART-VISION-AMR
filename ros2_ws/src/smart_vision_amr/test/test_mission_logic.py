"""Mission decisions require a correlated arrival and fresh camera evidence."""
import unittest
from smart_vision_amr.mission_logic import MarkerMission, MissionError

HOME = {'x': -1., 'y': -2., 'yaw': .2}
MARKERS = [{'id': i, 'name': str(i), 'x': float(i), 'y': 0., 'yaw': 0.} for i in range(4)]


class MissionLogicTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.m = MarkerMission(MARKERS, [2, 3], clock=lambda: self.now)

    def arrive(self, stamp=(1, 0)):
        self.m.start(home=HOME)
        self.m.drain_commands()
        self.m.goal_dispatched(stamp)
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': stamp[0], 'nanosec': stamp[1]}})

    def observe(self, frame=1, ids=None, stationary=True, pose=None, frame_time=11.):
        return self.m.observe(frame, [2] if ids is None else ids,
                              pose or {'x': 2., 'y': 0., 'yaw': 0.}, stationary, frame_time)

    def test_invalid_empty_unknown_bool_routes_rejected(self):
        for route in ([], [10], [True], '2,3', None, [2]*101):
            with self.assertRaises(MissionError):
                self.m.update_route(route)

    def test_repeats_are_valid(self):
        self.m.update_route([2, 3, 2, 2])
        self.assertEqual(self.m.route, [2, 3, 2, 2])

    def test_camera_before_navigation_success_never_advances(self):
        self.m.start(home=HOME)
        for i in range(10):
            self.observe(i)
        self.assertEqual(self.m.current_index, 0)
        self.assertEqual(self.m.confirmations, 0)

    def test_old_uncorrelated_success_ignored(self):
        self.m.start(home=HOME)
        self.m.goal_dispatched((3, 4))
        for payload in ({'status': 'SUCCEEDED'}, {'status': 'SUCCEEDED', 'goal_stamp': {'sec': 3, 'nanosec': 3}}):
            self.assertFalse(self.m.on_navigation(payload))
        self.assertEqual(self.m.state, 'NAVIGATING')

    def test_requires_three_unique_post_arrival_frames(self):
        self.arrive()
        self.observe(1, frame_time=9.)
        self.assertEqual(self.m.confirmations, 0)
        self.observe(2)
        self.observe(2)
        self.assertEqual(self.m.confirmations, 1)
        self.observe(3)
        self.assertTrue(self.observe(4))
        self.assertEqual(self.m.completed_ids, [2])
        self.assertEqual(self.m.expected_id, 3)
        self.assertEqual(self.m.state, 'NAVIGATING')

    def test_stale_marker_frames_cannot_complete_waiting_confirmation(self):
        self.arrive()
        pose = {'x': 2., 'y': 0., 'yaw': 0.}
        for frame in range(1, 5):
            self.m.observe(frame, [2], pose, True, 11.+frame, camera_fresh=False)
        self.assertEqual(self.m.state, 'CONFIRMING')
        self.assertEqual(self.m.confirmations, 0)
        self.assertEqual(self.m.completed_ids, [])
        for frame in range(5, 8):
            self.m.observe(frame, [2], pose, True, 11.+frame, camera_fresh=True)
        self.assertEqual(self.m.completed_ids, [2])

    def test_wrong_marker_resets_count(self):
        self.arrive()
        self.observe(1)
        self.observe(2, ids=[3])
        self.assertEqual(self.m.confirmations, 0)

    def test_moving_or_wrong_pose_or_wrong_yaw_does_not_confirm(self):
        self.arrive()
        self.observe(1, stationary=False)
        self.observe(2, pose={'x': 3., 'y': 0., 'yaw': 0.})
        self.observe(3, pose={'x': 2., 'y': 0., 'yaw': 3.})
        self.assertEqual(self.m.confirmations, 0)

    def test_replan_wait_keeps_goal_and_route(self):
        self.m.start(home=HOME)
        self.m.goal_dispatched((1, 2))
        self.m.on_navigation({'status': 'WAIT_REPLAN', 'goal_stamp': {'sec': 1, 'nanosec': 2}})
        self.assertEqual(self.m.state, 'WAIT_REPLAN')
        self.assertEqual(self.m.goal_stamp, (1, 2))
        self.assertEqual(self.m.expected_id, 2)
        self.m.on_navigation({'status': 'MOVING', 'goal_stamp': {'sec': 1, 'nanosec': 2}})
        self.assertEqual(self.m.state, 'NAVIGATING')

    def test_reverse_recovery_keeps_delivery_goal_and_does_not_confirm_arrival(self):
        self.m.start(home=HOME)
        self.m.goal_dispatched((1, 2))
        for status in ('RECOVERING', 'RECOVERY_SUCCEEDED', 'RECOVERY_FAILED'):
            self.m.on_navigation({'status': status, 'goal_stamp': {'sec': 1, 'nanosec': 2}})
            self.assertEqual(self.m.state, 'WAIT_REPLAN')
            self.assertEqual(self.m.goal_stamp, (1, 2))
            self.assertEqual(self.m.expected_id, 2)
            self.assertEqual(self.m.completed_ids, [])

    def test_new_route_staged_while_moving_and_promoted_only_new_start(self):
        self.m.start(home=HOME)
        self.m.update_route([3, 1, 0])
        self.assertEqual(self.m.route, [2, 3])
        self.assertEqual(self.m.pending_route, [3, 1, 0])
        self.m.pause()
        self.m.resume()
        self.assertEqual(self.m.route, [2, 3])
        self.m.pause()
        self.m.restart()
        self.assertEqual(self.m.home, HOME)
        self.assertEqual(self.m.route, [3, 1, 0])
        self.assertIsNone(self.m.pending_route)

    def test_pause_requires_explicit_resume_and_ignores_late_status(self):
        self.arrive()
        self.m.pause()
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': 1, 'nanosec': 0}})
        self.observe(1)
        self.assertEqual(self.m.state, 'PAUSED')
        self.assertEqual([item[0] for item in self.m.drain_commands()], ['cancel', 'stop'])
        self.m.resume()
        self.assertEqual(self.m.state, 'NAVIGATING')

    def test_external_manual_stop_pauses(self):
        self.m.start(home=HOME)
        self.m.goal_dispatched((1, 0))
        self.m.on_navigation({'status': 'CANCELED', 'manual_stop': True, 'goal_stamp': {'sec': 1, 'nanosec': 0}})
        self.assertEqual(self.m.state, 'PAUSED')

    def test_emergency_ack_never_restarts(self):
        self.m.start(home=HOME)
        self.m.emergency_stop()
        with self.assertRaises(MissionError):
            self.m.cancel()
        self.assertEqual(self.m.state, 'ESTOP')
        with self.assertRaises(MissionError):
            self.m.start(home=HOME)
        self.m.reset_emergency()
        self.assertEqual(self.m.state, 'CANCELED')
        self.assertFalse(self.m.estop_latched)
        self.assertEqual([c[0] for c in self.m.drain_commands()], ['cancel', 'stop'])

    def test_entire_repeated_route_requires_each_new_arrival(self):
        self.m.update_route([2, 2])
        self.m.start(home=HOME)
        for step in range(2):
            self.m.goal_dispatched((1, step))
            self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': {'sec': 1, 'nanosec': step}})
            for frame in range(3):
                self.observe(step*3+frame)
        self.assertEqual(self.m.state, 'COMPLETED')
        self.assertEqual(self.m.completed_ids, [2, 2])

    def test_route_after_completion_resets_display_until_explicit_start(self):
        self.m.state = 'COMPLETED'
        self.m.current_index = 2
        self.m.completed_ids = [2, 3]
        self.m.confirmations = 3
        self.m.update_route([3, 2])
        self.assertEqual(self.m.state, 'IDLE')
        self.assertEqual(self.m.current_index, 0)
        self.assertEqual(self.m.completed_ids, [])
        self.assertEqual(self.m.confirmations, 0)
        self.assertIsNone(self.m.goal_stamp)
        self.assertFalse(any(kind == 'goal' for kind, _ in self.m.commands))
        self.m.start(home=HOME)
        self.assertEqual(self.m.expected_id, 3)

    def test_route_during_emergency_does_not_unlatch_or_start(self):
        self.m.emergency_stop()
        self.m.update_route([3, 2])
        self.assertEqual(self.m.state, 'ESTOP')
        self.assertTrue(self.m.estop_latched)
        self.assertEqual(self.m.route, [3, 2])
        self.assertFalse(any(kind == 'goal' for kind, _ in self.m.commands))

    def test_snapshot_does_not_mutate_route(self):
        value = self.m.snapshot()
        value['route'].clear()
        self.assertEqual(self.m.route, [2, 3])

    def test_home_is_start_pose_copy_and_restart_retains_it(self):
        pose = dict(HOME)
        self.m.start(home=pose)
        pose['x'] = 99.
        self.assertEqual(self.m.home, HOME)
        self.m.pause()
        self.m.resume()
        self.m.update_route([3, 1])
        self.m.pause()
        self.m.restart()
        self.assertEqual(self.m.home, HOME)
        self.assertEqual(self.m.route, [3, 1])
        self.assertEqual([kind for kind, _ in self.m.drain_commands()], ['cancel', 'stop', 'goal'])

    def test_cancel_returns_to_home_not_marker_zero_and_old_success_ignored(self):
        self.m.start(home=HOME)
        self.m.goal_dispatched((10, 0))
        self.m.cancel()
        commands = self.m.drain_commands()
        self.assertEqual([kind for kind, _ in commands], ['cancel', 'stop', 'goal'])
        self.assertEqual(commands[-1][1], HOME)
        self.assertEqual(self.m.state, 'RETURNING_HOME')
        self.assertTrue(self.m.snapshot()['returning_home'])
        self.assertIsNone(self.m.expected_id)
        self.m.goal_dispatched((11, 0))
        self.assertFalse(self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [10, 0]}, HOME, True))
        self.assertEqual(self.m.state, 'RETURNING_HOME')

    def test_home_success_needs_correlated_result_pose_yaw_and_stationary_not_camera(self):
        self.m.start(home=HOME)
        self.m.return_home()
        self.m.drain_commands()
        self.m.goal_dispatched((20, 0))
        self.assertFalse(self.m.observe_home(HOME, True), 'proximity alone is not arrival')
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [20, 0]})
        self.assertFalse(self.m.observe_home(HOME, False))
        self.assertFalse(self.m.observe_home(dict(HOME, x=9.), True))
        self.assertFalse(self.m.observe_home(dict(HOME, yaw=2.), True))
        self.assertTrue(self.m.observe_home(HOME, True))
        self.assertEqual(self.m.state, 'HOME_REACHED')
        self.assertFalse(self.m.returning_home)
        self.assertEqual(self.m.completed_ids, [])
        self.assertEqual(self.m.drain_commands(), [('stop', None)])
        self.m.start(home={'x': 2., 'y': 3., 'yaw': 0.})
        self.assertEqual(self.m.home['x'], 2.)

    def test_paused_home_resumes_home_and_wait_replan_preserves_return_mode(self):
        self.m.start(home=HOME)
        self.m.cancel()
        self.m.goal_dispatched((30, 0))
        self.m.on_navigation({'status': 'WAIT_REPLAN', 'goal_stamp': [30, 0]})
        self.assertEqual(self.m.state, 'WAIT_REPLAN')
        self.assertTrue(self.m.returning_home)
        self.m.on_navigation({'status': 'MOVING', 'goal_stamp': [30, 0]})
        self.assertEqual(self.m.state, 'RETURNING_HOME')
        self.m.pause()
        self.m.drain_commands()
        self.assertTrue(self.m.returning_home)
        self.m.resume()
        self.assertEqual(self.m.state, 'RETURNING_HOME')
        self.assertEqual(self.m.drain_commands(), [('goal', HOME)])

    def test_missing_home_rejects_idle_return_and_stops_active_mission(self):
        with self.assertRaises(MissionError):
            self.m.return_home()
        self.assertEqual(self.m.state, 'IDLE')
        self.assertEqual(self.m.commands, [])
        self.m.start()
        with self.assertRaises(MissionError):
            self.m.cancel()
        self.assertEqual(self.m.state, 'CANCELED')
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None)])

    def test_estop_return_and_reset_never_queue_home_motion(self):
        self.m.start(home=HOME)
        self.m.return_home()
        self.m.emergency_stop()
        with self.assertRaises(MissionError):
            self.m.return_home()
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None)])
        self.m.reset_emergency()
        self.assertFalse(self.m.returning_home)
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None)])
        self.assertEqual(self.m.home, HOME)


class DeliveryMissionTests(unittest.TestCase):
    """Delivery queues keep their departure home and require real confirmations."""

    def setUp(self):
        self.m = MarkerMission(MARKERS, [2], clock=lambda: 10.)
        self.frame = 0

    def start(self):
        self.m.start(home=HOME, auto_return_home=True)
        self.m.drain_commands()

    def complete_marker(self, stamp=(100, 0)):
        marker_id = self.m.expected_id
        self.m.goal_dispatched(stamp)
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': list(stamp)})
        for _ in range(3):
            self.frame += 1
            self.m.observe(self.frame, [marker_id], MARKERS[marker_id], True, 11.)

    def test_automatic_return_requires_home_without_partial_start(self):
        with self.assertRaises(MissionError):
            self.m.start(auto_return_home=True)
        self.assertEqual(self.m.state, 'IDLE')
        self.assertEqual(self.m.drain_commands(), [])

    def test_confirming_final_marker_starts_home_without_erasing_deliveries(self):
        self.start()
        self.complete_marker()
        self.assertEqual(self.m.state, 'RETURNING_HOME')
        self.assertEqual(self.m.completed_ids, [2])
        self.assertIsNone(self.m.expected_id)
        self.assertTrue(self.m.snapshot()['auto_return_home'])
        self.assertTrue(self.m.snapshot()['auto_returning_home'])
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None), ('goal', HOME)])
        self.assertFalse(self.m.observe_home(HOME, True))
        self.m.goal_dispatched((101, 0))
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [101, 0]}, HOME, True)
        self.assertEqual(self.m.state, 'HOME_REACHED')
        self.assertEqual(self.m.completed_ids, [2])
        self.assertFalse(self.m.snapshot()['auto_returning_home'])

    def test_active_append_keeps_goal_stamp_and_confirmation_progress(self):
        self.start()
        self.m.goal_dispatched((100, 3))
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [100, 3]})
        self.m.observe(1, [2], MARKERS[2], True, 11.)
        self.m.append_waypoints([1, 3, 1])
        self.assertEqual(self.m.route, [2, 1, 3, 1])
        self.assertEqual(self.m.goal_stamp, (100, 3))
        self.assertEqual(self.m.state, 'CONFIRMING')
        self.assertEqual(self.m.confirmations, 1)
        self.assertEqual(self.m.drain_commands(), [])
        self.m.observe(2, [2], MARKERS[2], True, 11.)
        self.m.observe(3, [2], MARKERS[2], True, 11.)
        self.assertEqual(self.m.expected_id, 1)
        self.assertEqual(self.m.completed_ids, [2])

    def test_active_append_does_not_break_replanning(self):
        self.start()
        self.m.goal_dispatched((100, 3))
        self.m.on_navigation({'status': 'WAIT_REPLAN', 'goal_stamp': [100, 3]})
        self.m.append_waypoints([1])
        self.assertEqual(self.m.state, 'WAIT_REPLAN')
        self.assertEqual(self.m.goal_stamp, (100, 3))
        self.assertEqual(self.m.drain_commands(), [])

    def test_append_during_auto_return_cancels_home_before_new_goal(self):
        self.start()
        self.complete_marker()
        self.m.drain_commands()
        self.m.goal_dispatched((101, 0))
        self.m.append_waypoints([3, 0])
        self.assertEqual(self.m.route, [2, 3, 0])
        self.assertEqual(self.m.current_index, 1)
        self.assertEqual(self.m.completed_ids, [2])
        self.assertEqual(self.m.home, HOME)
        self.assertFalse(self.m.returning_home)
        self.assertIsNone(self.m.goal_stamp)
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None), ('goal', MARKERS[3])])
        self.assertFalse(self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [101, 0]}, HOME, True))
        self.m.goal_dispatched((102, 0))
        self.assertFalse(self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [101, 0]}, HOME, True))
        self.assertEqual(self.m.state, 'NAVIGATING')

    def test_append_replaces_not_yet_dispatched_automatic_home_command(self):
        self.start()
        self.complete_marker()
        self.m.append_waypoints([3])
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None), ('goal', MARKERS[3])])

    def test_paused_outbound_append_never_unpauses(self):
        self.start()
        self.m.pause()
        self.m.drain_commands()
        self.m.append_waypoints([3])
        self.assertEqual(self.m.state, 'PAUSED')
        self.assertEqual(self.m.expected_id, 2)
        self.assertEqual(self.m.drain_commands(), [])
        self.m.resume()
        self.assertEqual(self.m.drain_commands(), [('goal', MARKERS[2])])

    def test_paused_automatic_home_append_waits_for_explicit_resume(self):
        self.start()
        self.complete_marker()
        self.m.pause()
        self.m.drain_commands()
        self.m.append_waypoints([3])
        self.assertEqual(self.m.state, 'PAUSED')
        self.assertFalse(self.m.returning_home)
        self.assertEqual(self.m.expected_id, 3)
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None)])
        self.m.resume()
        self.assertEqual(self.m.state, 'NAVIGATING')
        self.assertEqual(self.m.drain_commands(), [('goal', MARKERS[3])])

    def test_manual_return_does_not_accept_autonomous_diversion(self):
        self.start()
        self.m.cancel()
        self.m.drain_commands()
        with self.assertRaises(MissionError):
            self.m.append_waypoints([3])
        self.assertFalse(self.m.auto_return_home)
        self.assertFalse(self.m.auto_returning_home)
        self.assertEqual(self.m.route, [2])
        self.assertEqual(self.m.state, 'RETURNING_HOME')
        self.assertEqual(self.m.drain_commands(), [])

    def test_manual_home_during_automatic_return_closes_extension_window(self):
        self.start()
        self.complete_marker()
        self.m.drain_commands()
        self.m.return_home()
        self.assertEqual(self.m.state, 'RETURNING_HOME')
        self.assertFalse(self.m.auto_return_home)
        with self.assertRaises(MissionError):
            self.m.append_waypoints([3])
        self.assertEqual(self.m.drain_commands(), [])

    def test_estop_and_reset_require_new_explicit_dispatch(self):
        self.start()
        self.m.emergency_stop()
        with self.assertRaises(MissionError):
            self.m.append_waypoints([3])
        self.assertEqual(self.m.route, [2])
        self.assertEqual(self.m.drain_commands(), [('cancel', None), ('stop', None)])
        self.m.reset_emergency()
        with self.assertRaises(MissionError):
            self.m.append_waypoints([3])
        self.assertEqual(self.m.state, 'CANCELED')
        self.assertFalse(any(kind == 'goal' for kind, _ in self.m.drain_commands()))

    def test_home_remains_fixed_across_appended_deliveries(self):
        home = dict(HOME)
        self.m.start(home=home, auto_return_home=True)
        home['x'] = 99.
        self.m.drain_commands()
        self.m.append_waypoints([3])
        self.complete_marker((100, 0))
        self.m.drain_commands()
        self.complete_marker((101, 0))
        self.assertEqual(self.m.completed_ids, [2, 3])
        self.assertEqual(self.m.home, HOME)
        self.assertEqual(self.m.drain_commands()[-1], ('goal', HOME))

    def test_finished_automatic_mission_waits_for_new_operator_dispatch(self):
        self.start()
        self.complete_marker()
        self.m.goal_dispatched((101, 0))
        self.m.on_navigation({'status': 'SUCCEEDED', 'goal_stamp': [101, 0]}, HOME, True)
        self.m.drain_commands()
        with self.assertRaises(MissionError):
            self.m.append_waypoints([3])
        self.assertEqual(self.m.state, 'HOME_REACHED')
        self.assertEqual(self.m.drain_commands(), [])

    def test_invalid_or_oversized_append_never_changes_existing_route(self):
        self.start()
        for route in ([], [True], [99], [3]*100):
            with self.assertRaises(MissionError):
                self.m.append_waypoints(route)
            self.assertEqual(self.m.route, [2])
            self.assertEqual(self.m.drain_commands(), [])


if __name__ == '__main__':
    unittest.main()
