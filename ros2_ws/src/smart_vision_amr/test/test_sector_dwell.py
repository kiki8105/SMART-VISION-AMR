"""Delivery dwell is a stopped, cancellable interval after real marker proof."""
import unittest

from smart_vision_amr.mission_logic import MarkerMission, MissionError
from smart_vision_amr.web_mission_server import build_parser


MARKERS = [{'id': i, 'name': f'Sector {i}', 'x': float(i), 'y': 0., 'yaw': 0.}
           for i in range(4)]
HOME = {'x': 0., 'y': -1., 'yaw': 0.}


class SectorDwellTests(unittest.TestCase):
    def test_web_runtime_default_is_five_seconds_and_cli_can_tune_it(self):
        parser = build_parser()
        self.assertEqual(parser.parse_args([]).sector_dwell_seconds, 5.)
        self.assertEqual(parser.parse_args(['--sector-dwell-seconds', '8']).sector_dwell_seconds, 8.)

    def setUp(self):
        self.now = 10.
        self.mission = MarkerMission(MARKERS, [1, 2], confirmations=2,
                                     clock=lambda: self.now,
                                     sector_dwell_seconds=5.)
        self.mission.start(home=HOME, auto_return_home=True)
        self.assertEqual(self.mission.drain_commands(), [('goal', MARKERS[1])])

    def confirm(self, marker_id=1, stamp=(100, 0)):
        self.mission.goal_dispatched(stamp)
        self.assertTrue(self.mission.on_navigation(
            {'status': 'SUCCEEDED', 'goal_stamp': list(stamp)}))
        pose = MARKERS[marker_id]
        self.assertFalse(self.mission.observe(1, [marker_id], pose, True, self.now,
                                              alignment_verified=True))
        self.assertTrue(self.mission.observe(2, [marker_id], pose, True, self.now,
                                             alignment_verified=True))

    def advance_for(self, seconds, *, stationary=True):
        result = False
        for _ in range(round(seconds * 10)):
            self.now = round(self.now + .1, 9)
            result = self.mission.advance_dwell(stationary)
        return result

    def finish_dwell(self):
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertTrue(self.advance_for(5.))

    def test_next_sector_waits_five_stopped_seconds_and_dispatches_once(self):
        self.confirm()
        self.assertEqual(self.mission.state, 'DWELLING')
        self.assertEqual(self.mission.completed_ids, [1])
        self.assertIsNone(self.mission.goal_stamp)
        self.assertEqual(self.mission.drain_commands(), [('stop', None)])
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertFalse(self.advance_for(4.9))
        self.assertEqual(self.mission.drain_commands(), [])
        self.assertTrue(self.advance_for(.1))
        self.assertEqual(self.mission.state, 'NAVIGATING')
        self.assertEqual(self.mission.drain_commands(), [('goal', MARKERS[2])])
        self.now += 10.
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertEqual(self.mission.drain_commands(), [])

    def test_moving_time_is_not_counted(self):
        self.confirm()
        self.mission.drain_commands()
        self.mission.advance_dwell(True)
        self.now += 3.
        self.assertFalse(self.mission.advance_dwell(False))
        self.now += 4.9
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertEqual(self.mission.state, 'DWELLING')
        self.assertFalse(self.advance_for(4.9))
        self.assertTrue(self.advance_for(.1))

    def test_scheduler_stall_cannot_count_unobserved_stop_time(self):
        self.confirm()
        self.mission.drain_commands()
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertFalse(self.advance_for(1.))
        self.now += 4.5
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertAlmostEqual(self.mission.dwell_elapsed, 1.)
        self.assertFalse(self.advance_for(3.9))
        self.assertTrue(self.advance_for(.1))

    def test_last_sector_waits_before_automatic_home(self):
        self.confirm()
        self.mission.drain_commands()
        self.finish_dwell()
        self.mission.drain_commands()
        self.now += 1.
        self.confirm(2, (101, 0))
        self.assertEqual(self.mission.state, 'DWELLING')
        self.assertFalse(self.mission.returning_home)
        self.assertEqual(self.mission.drain_commands(), [('stop', None)])
        self.finish_dwell()
        self.assertEqual(self.mission.state, 'RETURNING_HOME')
        self.assertEqual(self.mission.drain_commands(),
                         [('cancel', None), ('stop', None), ('goal', HOME)])

    def test_pause_and_resume_preserve_remaining_stop_time(self):
        self.confirm()
        self.mission.drain_commands()
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertFalse(self.advance_for(2.))
        self.mission.pause()
        self.assertEqual(self.mission.drain_commands(), [('cancel', None), ('stop', None)])
        self.now += 100.
        self.assertFalse(self.mission.advance_dwell(True))
        self.mission.resume()
        self.assertEqual(self.mission.state, 'DWELLING')
        self.assertEqual(self.mission.drain_commands(), [('stop', None)])
        self.assertFalse(self.mission.advance_dwell(True))
        self.assertFalse(self.advance_for(2.9))
        self.assertTrue(self.advance_for(.1))
        self.assertEqual(self.mission.drain_commands(), [('goal', MARKERS[2])])

    def test_cancel_and_estop_cannot_dispatch_delayed_goal(self):
        for action, expected in ((lambda mission: mission.cancel(), 'RETURNING_HOME'),
                                 (lambda mission: mission.emergency_stop(), 'ESTOP')):
            with self.subTest(action=expected):
                self.setUp()
                self.confirm()
                self.mission.drain_commands()
                action(self.mission)
                self.assertEqual(self.mission.state, expected)
                self.now += 10.
                self.assertFalse(self.mission.advance_dwell(True))
                commands = self.mission.drain_commands()
                if expected == 'ESTOP':
                    self.assertFalse(any(kind == 'goal' for kind, _ in commands))
                else:
                    self.assertEqual(commands[-1], ('goal', HOME))

    def test_new_request_during_final_dwell_precedes_home(self):
        self.mission = MarkerMission(MARKERS, [1], confirmations=2,
                                     clock=lambda: self.now,
                                     sector_dwell_seconds=5.)
        self.mission.start(home=HOME, auto_return_home=True)
        self.mission.drain_commands()
        self.confirm()
        self.mission.drain_commands()
        self.mission.append_waypoints([3])
        self.finish_dwell()
        self.assertEqual(self.mission.drain_commands(), [('goal', MARKERS[3])])
        self.assertFalse(self.mission.returning_home)

    def test_invalid_dwell_setting_is_rejected(self):
        for value in (-1, float('nan'), float('inf'), None, 'invalid'):
            with self.subTest(value=value), self.assertRaises(MissionError):
                MarkerMission(MARKERS, [1], sector_dwell_seconds=value)


if __name__ == '__main__':
    unittest.main()
