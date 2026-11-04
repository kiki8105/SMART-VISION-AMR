"""Portable regressions: no ROS installation or original /tmp trial is needed."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    'inflation_counter_summary', HERE/'summarize_inflation_trials.py')
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


def sample(at, value, goal='sector', field='global_plan_count'):
    return {'kind': 'planner', 'wall_time': at, 'goal_stamp': goal,
            'details': {field: value}}


class LifetimeCounterRegression(unittest.TestCase):
    def test_goal_change_and_republished_status_do_not_repeat_lifetime_work(self):
        rows = [sample(9., 7), sample(10., 8), sample(10.1, 8),
                sample(11., 9), sample(12., 9, 'home'),
                sample(12.1, 9, 'home'), sample(14., 10, 'next_mission')]
        metrics = SUMMARY.cumulative_counter_metrics(rows, 'global_plan_count', 10., 13.)
        self.assertEqual(metrics['count'], 2)
        self.assertEqual(metrics['baseline']['value'], 7)
        self.assertEqual(len(metrics['increments']), 2)
        self.assertFalse(metrics['is_lower_bound'])

    def test_counter_decrease_does_not_assume_restart_or_count_old_epochs_twice(self):
        rows = [sample(9., 7), sample(10., 8), sample(11., 0),
                sample(12., 2), sample(13., 1), sample(14., 2)]
        metrics = SUMMARY.cumulative_counter_metrics(rows, 'global_plan_count', 10., 14.)
        self.assertIsNone(metrics['count'])
        self.assertEqual(metrics['observed_positive_delta_sum'], 4)
        self.assertEqual(len(metrics['counter_resets']), 2)
        self.assertIsNone(metrics['is_lower_bound'])

    def test_missing_baseline_does_not_charge_previous_mission_counter(self):
        rows = [sample(10., 12), sample(11., 12, 'home'), sample(12., 13, 'home')]
        metrics = SUMMARY.cumulative_counter_metrics(rows, 'global_plan_count', 10., 13.)
        self.assertEqual(metrics['count'], 1)
        self.assertEqual(metrics['first_mission_value'], 12)
        self.assertIsNone(metrics['baseline'])
        self.assertTrue(metrics['is_lower_bound'])

    def test_sparse_field_bad_samples_and_receipt_order_are_handled(self):
        rows = [sample(12., 3), sample(9., 0), sample(10., True),
                sample(10.1, -1), sample(10.2, 500, field='local_attempt_id'),
                sample(float('nan'), 99), sample(11., 1),
                {'kind': 'navigation', 'wall_time': 11.5,
                 'details': {'global_plan_count': 100}}]
        metrics = SUMMARY.cumulative_counter_metrics(rows, 'global_plan_count', 10., 13.)
        self.assertEqual(metrics['count'], 3)
        self.assertEqual(metrics['valid_mission_samples'], 2)
        self.assertEqual([r['increment'] for r in metrics['increments']], [1, 2])

    def test_no_counter_samples_is_unavailable_not_a_zero_attempt_claim(self):
        metrics = SUMMARY.cumulative_counter_metrics([], 'global_plan_count', 10., 13.)
        self.assertIsNone(metrics['count'])
        self.assertIsNone(metrics['is_lower_bound'])

    def test_summary_keeps_pre_mission_baseline_for_carried_counters(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            evidence = {'mission_started_at': 10., 'final_observed_at': 13.,
                        'route': [3], 'completed_ids': [3],
                        'final_state': 'HOME_REACHED'}
            (directory/'derived_evidence.json').write_text(json.dumps(evidence))
            rows = [sample(9., 7), sample(10.1, 8), sample(12., 8, 'home')]
            (directory/'safety_timeline.jsonl').write_text(
                '\n'.join(json.dumps(row) for row in rows)+'\n')
            result = SUMMARY.summarize('carried', directory)
            self.assertEqual(result['global_plan_count'], 1)
            self.assertEqual(result['counter_aggregation']['global_plan_count'][
                'baseline']['value'], 7)

    def test_recorded_clean_and_inventory_counter_transitions(self):
        fixture = json.loads((HERE/'counter_receipt_fixture_20261105.json').read_text())
        for trial in fixture['trials']:
            with self.subTest(stage=trial['stage']):
                for output, field in [('global_plan_count', 'global_plan_count'),
                                      ('local_attempt_count', 'local_attempt_id')]:
                    metrics = SUMMARY.cumulative_counter_metrics(
                        trial['selected_rows'], field,
                        trial['mission_started_at'], trial['final_observed_at'])
                    self.assertEqual(metrics['count'], trial['expected'][output])
                    self.assertFalse(metrics['is_lower_bound'])
                # Reproduce the old bug from the same portable fixture. This
                # demonstrates why a goal change must not reset the baseline.
                for field, old_key in [
                        ('global_plan_count', 'legacy_global_max_per_goal_sum'),
                        ('local_attempt_id', 'legacy_local_max_per_goal_sum')]:
                    maxima = {}
                    for row in trial['selected_rows']:
                        if row['wall_time'] < trial['mission_started_at']:
                            continue
                        key = json.dumps(row['goal_stamp'], sort_keys=True)
                        maxima[key] = max(maxima.get(key, 0), row['details'][field])
                    self.assertEqual(sum(maxima.values()), trial['expected'][old_key])

    def test_cli_refuses_to_overwrite_historical_result(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'historic.json'
            output.write_text('{"historic": true}\n')
            command = [sys.executable, str(HERE/'summarize_inflation_trials.py'),
                       '--trial', f'missing={temp}', '--output', str(output)]
            completed = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 2)
            self.assertIn('already exists', completed.stderr)
            self.assertEqual(output.read_text(), '{"historic": true}\n')


if __name__ == '__main__':
    unittest.main()
