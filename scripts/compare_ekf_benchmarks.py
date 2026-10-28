#!/usr/bin/env python3
"""Summarize replicated Gazebo EKF trials without treating a failed run as zero error."""

import argparse
import json
from pathlib import Path
import statistics


MODES = ('baseline', 'ekf_overlay', 'ekf_independent')


def valid_trial(record):
    metrics = record.get('position') or {}
    samples = metrics.get('sample_count', 0)
    gap = record.get('tf_max_gap_s')
    return (record.get('trial_status') == 'COMPLETE'
            and record.get('route_completion_fraction') == 1
            and samples >= 100
            and record.get('matched_sample_rate_hz', 0) >= 5
            and record.get('matched_sample_fraction', 0) >= .4
            and record.get('last_matched_age_s', 999) <= .5
            and record.get('truth_distance_m', 0) >= 1.8
            and record.get('sim_duration_s', 0) >= 20
            and record.get('max_interpolation_side_s', 1) <= .11
            and record.get('max_interpolation_span_s', 1) <= .12
            and record.get('tf_odom_base_count', 0) >= 2
            and record.get('graph_authority_ok', False)
            and record.get('tf_odom_base_rate_hz', 999) <= 45
            and record.get('tf_conflict_count', 0) == 0
            and gap is not None and gap <= .5
            and record.get('tf_stale_sample_count', samples) <= max(2, samples * .05)
            and metrics.get('position_rmse_m') is not None
            and metrics.get('heading_rmse_deg') is not None)


def summarize(records, minimum_trials=3):
    by_mode = {mode: [] for mode in MODES}
    for record in records:
        if record.get('mode') in by_mode:
            by_mode[record['mode']].append(record)
    summaries = {}
    for mode, trials in by_mode.items():
        valid = [record for record in trials if valid_trial(record)]
        summaries[mode] = {
            'trial_count': len(trials),
            'valid_trial_count': len(valid),
            'completion_rate': (sum(record.get('trial_status') == 'COMPLETE'
                                    for record in trials) / len(trials) if trials else None),
            'median_position_rmse_m': (statistics.median(record['position']['position_rmse_m']
                                                        for record in valid) if valid else None),
            'median_heading_rmse_deg': (statistics.median(record['position']['heading_rmse_deg']
                                                         for record in valid) if valid else None),
            'median_tf_max_gap_s': (statistics.median(record['tf_max_gap_s']
                                                    for record in valid) if valid else None),
            'median_truth_distance_m': (statistics.median(record['truth_distance_m']
                                                          for record in valid) if valid else None),
            'median_matched_sample_rate_hz': (
                statistics.median(record['matched_sample_rate_hz'] for record in valid)
                if valid else None),
            'median_matched_sample_fraction': (
                statistics.median(record['matched_sample_fraction'] for record in valid)
                if valid else None),
            'median_estimate_header_lag_p95_s': (
                statistics.median(record['estimate_header_lag_p95_s'] for record in valid
                                  if record.get('estimate_header_lag_p95_s') is not None)
                if any(record.get('estimate_header_lag_p95_s') is not None for record in valid)
                else None),
            'median_wall_duration_s': (statistics.median(record['wall_duration_s']
                                                        for record in valid) if valid else None),
            'median_sim_realtime_factor': (
                statistics.median(record['sim_realtime_factor'] for record in valid)
                if valid else None),
            'median_raw_odom_position_rmse_m': (
                statistics.median(record['raw_odom_in_overlay']['position_rmse_m']
                                  for record in valid
                                  if record.get('raw_odom_in_overlay')
                                  and record['raw_odom_in_overlay']['position_rmse_m'] is not None)
                if any(record.get('raw_odom_in_overlay')
                       and record['raw_odom_in_overlay']['position_rmse_m'] is not None
                       for record in valid) else None),
            'median_raw_odom_heading_rmse_deg': (
                statistics.median(record['raw_odom_in_overlay']['heading_rmse_deg']
                                  for record in valid
                                  if record.get('raw_odom_in_overlay')
                                  and record['raw_odom_in_overlay']['heading_rmse_deg'] is not None)
                if any(record.get('raw_odom_in_overlay')
                       and record['raw_odom_in_overlay']['heading_rmse_deg'] is not None
                       for record in valid) else None),
            'failures': [record.get('failure') or 'invalid metrics/TF'
                         for record in trials if not valid_trial(record)],
        }
    sufficient = all(summaries[mode]['valid_trial_count'] >= minimum_trials
                     and summaries[mode]['valid_trial_count'] == summaries[mode]['trial_count']
                     for mode in MODES)
    recommendation = 'inconclusive'
    reason = f'Need {minimum_trials} valid, completed trials for each mode'
    if sufficient:
        baseline = summaries['baseline']
        independent = summaries['ekf_independent']
        baseline_pos = baseline['median_position_rmse_m']
        baseline_heading = baseline['median_heading_rmse_deg']
        independent_pos = independent['median_position_rmse_m']
        independent_heading = independent['median_heading_rmse_deg']
        # A diagnostic overlay is not Nav2's odom/TF provider, so it cannot be
        # selected as the operational setting based on these measurements.
        if (baseline_pos - independent_pos >= max(.005, baseline_pos * .10)
                and independent_heading <= baseline_heading * 1.10):
            recommendation = 'ekf_independent'
            reason = ('Independent wheel+IMU EKF reduced position RMSE by >=10% '
                      'without worsening heading RMSE by >10% in these simulated trials')
        else:
            recommendation = 'baseline'
            reason = ('Independent EKF did not meet the prespecified improvement '
                      'criterion in these simulated trials')
    return {
        'minimum_trials_per_mode': minimum_trials,
        'modes': summaries,
        'simulation_recommendation': recommendation,
        'reason': reason,
        'hardware_caution': ('Gazebo diff-drive odometry may be world-derived. '
                             'A simulation winner is not validated for the real TurtleBot3; '
                             'repeat with an external reference and safe physical course.'),
    }


def markdown(summary):
    lines = [
        '| 모드 | 성공/전체 | 위치 RMSE (m) | 방위 RMSE (°) | 표본 (Hz) | 표본 비율 | p95 추정 지연 (s) | TF 공백 (s) | 실제 시간 (s) | RTF |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for mode in MODES:
        item = summary['modes'][mode]
        def fmt(value):
            return f'{value:.3f}' if value is not None else '—'
        def fmt_position(value):
            return f'{value:.6f}' if value is not None else '—'
        lines.append('| {} | {}/{} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
            mode, item['valid_trial_count'], item['trial_count'],
            fmt_position(item['median_position_rmse_m']), fmt(item['median_heading_rmse_deg']),
            fmt(item['median_matched_sample_rate_hz']),
            fmt(item['median_matched_sample_fraction']),
            fmt(item['median_estimate_header_lag_p95_s']),
            fmt(item['median_tf_max_gap_s']),
            fmt(item['median_wall_duration_s']),
            fmt(item['median_sim_realtime_factor'])))
    lines += ['', f"모의 시험 결론: **{summary['simulation_recommendation']}** — {summary['reason']}",
              '', summary['hardware_caution']]
    overlay = summary['modes']['ekf_overlay']
    if overlay['median_raw_odom_position_rmse_m'] is not None:
        lines += ['', ('동일 overlay 시험에서 원본 /odom 위치 RMSE: '
                       f"{overlay['median_raw_odom_position_rmse_m']:.6f} m; "
                       'EKF 후보 /odometry/filtered 위치 RMSE: '
                       f"{overlay['median_position_rmse_m']:.6f} m.")]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_directory', type=Path)
    parser.add_argument('--minimum-trials', type=int, default=3)
    args = parser.parse_args()
    if args.minimum_trials < 1:
        parser.error('--minimum-trials must be >= 1')
    records = []
    for path in sorted(args.trial_directory.glob('*.json')):
        if path.name == 'summary.json':
            continue
        records.append(json.loads(path.read_text()))
    summary = summarize(records, args.minimum_trials)
    args.trial_directory.joinpath('summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    report = markdown(summary)
    args.trial_directory.joinpath('summary.md').write_text(report)
    print(report)
    return 0 if summary['simulation_recommendation'] != 'inconclusive' else 1


if __name__ == '__main__':
    raise SystemExit(main())
