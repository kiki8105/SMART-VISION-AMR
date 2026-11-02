#!/usr/bin/env python3
"""Compare isolated Gazebo TF receipt timing in paired QoS trials.

The two trials must use the same world, web monitor, safety observer and Nav2
profile. This reports observations only; arrival and safety remain separate
acceptance gates in ``run_classroom_staged_validation.py``.
"""

import argparse
import json
from pathlib import Path


def load_trial(directory: Path) -> dict:
    timeline = directory / 'safety_timeline.jsonl'
    records = [json.loads(line) for line in timeline.open() if line.strip()]
    raw_tf = [record for record in records
              if record.get('kind') == 'tf_stamp'
              and record.get('source') == 'raw'
              and record.get('parent') == 'odom'
              and record.get('child') == 'base_footprint']
    odom = [record for record in records if record.get('kind') == 'odom']
    scan = [record for record in records if record.get('kind') == 'scan']
    if len(raw_tf) < 2 or not odom or not scan:
        raise ValueError(f'Missing synchronized TF/odom/scan timeline: {timeline}')

    gaps = [(right['t']-left['t'], left, right)
            for left, right in zip(raw_tf, raw_tf[1:])]
    longest, before, after = max(gaps, key=lambda item: item[0])
    max_odom_gap = max((right['t']-left['t']
                        for left, right in zip(odom, odom[1:])), default=None)
    max_scan_gap = max((right['t']-left['t']
                        for left, right in zip(scan, scan[1:])), default=None)

    # Within the largest raw-TF pause, identify whether other sensor streams
    # continued. Their message stamps are independent of wall-clock speed.
    other_odom = [record for record in odom
                  if before['t'] < record['t'] < after['t']]
    other_scan = [record for record in scan
                  if before['t'] < record['t'] < after['t']]
    health_latch = [record for record in records
                    if record.get('kind') == 'health'
                    and record.get('stop_latched') is True]
    snapshots = [record for record in records
                 if record.get('kind') == 'input_fault_snapshot']
    acceptance_file = directory / 'acceptance.json'
    acceptance = json.loads(acceptance_file.read_text()) if acceptance_file.is_file() else {}
    leg = (acceptance.get('legs') or [{}])[0]
    return {
        'trial': str(directory),
        'raw_tf_count': len(raw_tf),
        'max_raw_tf_receipt_gap_s': round(longest, 3),
        'max_raw_tf_gap_interval': [round(before['t'], 3), round(after['t'], 3)],
        'raw_tf_stamp_before_after_s': [round(before['stamp_ns']/1e9, 3),
                                        round(after['stamp_ns']/1e9, 3)],
        'odom_messages_during_gap': len(other_odom),
        'scan_messages_during_gap': len(other_scan),
        'max_odom_receipt_gap_s': round(max_odom_gap, 3) if max_odom_gap else None,
        'max_scan_receipt_gap_s': round(max_scan_gap, 3) if max_scan_gap else None,
        'clock_stamps_seen': sum(record.get('clock_stamp_ns') is not None
                                 for record in records),
        'health_latched': bool(health_latch),
        'health_latch_at_s': round(health_latch[0]['t'], 3) if health_latch else None,
        'input_fault_snapshots': len(snapshots),
        'route_terminal': leg.get('terminal'),
        'route_passed': leg.get('passed') is True,
        'goal_position_error_m': leg.get('position_error_m'),
        'fault_delivery_passed': (acceptance.get('fault_delivery') or {}).get('passed'),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reliable_trial', type=Path)
    parser.add_argument('best_effort_trial', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--load-controlled', action='store_true',
                        help='Assert no concurrent build/test process changed CPU load')
    args = parser.parse_args()
    evidence = {'reliable_raw_tf': load_trial(args.reliable_trial),
                'best_effort_raw_tf': load_trial(args.best_effort_trial)}
    # A faster TF stream alone is not an end-to-end navigation pass.
    reliable = evidence['reliable_raw_tf']
    best_effort = evidence['best_effort_raw_tf']
    evidence['controlled_load'] = args.load_controlled
    evidence['qos_improved_tf_receipt'] = (
        best_effort['max_raw_tf_receipt_gap_s'] <
        reliable['max_raw_tf_receipt_gap_s'] * .5
        if args.load_controlled else None)
    evidence['qos_reached_goal_safely'] = (
        best_effort['route_passed'] and not best_effort['health_latched'])
    rendered = json.dumps(evidence, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
