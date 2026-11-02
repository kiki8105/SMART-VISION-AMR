#!/usr/bin/env python3
"""Refuse a green fault trial unless its relay actually applied the faults."""

import argparse
import json
from pathlib import Path


def assess(events, profile, mode):
    streams = ('scan', 'odom', 'tf') if mode == 'baseline' else ('scan',)
    applied = (profile == 'clean' or any(event.get('event') == 'profile'
               and event.get('name') == profile for event in events))
    samples = [event for event in events if event.get('event') == 'sample'
               and event.get('profile') == profile]
    latest = samples[-1] if samples else {}
    counts = latest.get('counts', {})
    delays = latest.get('max_delivery_delay_s', {})
    missing = []
    if not applied:
        missing.append('relay_profile_not_applied')
    for stream in streams:
        if counts.get(stream+'_forwarded', 0) < 1:
            missing.append(stream+'_not_forwarded')
        if profile in ('jitter', 'mixed') and delays.get(stream, 0.) < .08:
            missing.append(stream+'_jitter_not_measured')
        if profile in ('outages', 'mixed') and counts.get(stream+'_dropped_outage', 0) < 1:
            missing.append(stream+'_outage_not_measured')
    return {'profile': profile, 'mode': mode, 'streams': list(streams),
            'relay_profile_applied': applied, 'counts': counts,
            'max_delivery_delay_s': delays, 'missing': missing,
            'passed': not missing}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('clean', 'jitter', 'outages', 'mixed'),
                        required=True)
    parser.add_argument('--mode', choices=('baseline', 'ekf_independent'), required=True)
    parser.add_argument('--relay', type=Path, required=True)
    parser.add_argument('--acceptance', type=Path, required=True)
    args = parser.parse_args()
    events = [json.loads(line) for line in args.relay.read_text().splitlines() if line.strip()]
    outcome = assess(events, args.profile, args.mode)
    evidence = json.loads(args.acceptance.read_text())
    evidence['fault_delivery'] = outcome
    if args.profile != 'clean' and not outcome['passed']:
        evidence['passed'] = False
        evidence['failure_class'] = 'fault_injection_not_observed'
    args.acceptance.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(outcome, sort_keys=True))
    return 0 if outcome['passed'] or args.profile == 'clean' else 1


if __name__ == '__main__':
    raise SystemExit(main())
