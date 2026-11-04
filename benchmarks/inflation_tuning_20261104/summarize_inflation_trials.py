#!/usr/bin/env python3
"""Summarize full-web trials without running ROS or changing their verdicts."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path


def load(path, default=None):
    return json.loads(path.read_text()) if path.is_file() else default


def jsonlines(path):
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def goal_key(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'))


def cumulative_counter_metrics(rows, field, begin, end):
    """Count observed increments of a node-lifetime counter, not goal maxima.

    The latest valid sample strictly before the mission is the baseline. Goal
    changes and repeated status messages never reset this baseline. A decrease
    cannot distinguish a node restart from an out-of-order old status without
    a process-epoch ID, so the authoritative count is unavailable in that case.
    With no pre-mission sample, the first visible value is not assumed to belong
    to this mission. Boundaries use status receipt time, not compute start time.
    """
    samples = []
    for index, row in enumerate(rows):
        if row.get('kind') != 'planner':
            continue
        stamp = row.get('wall_time')
        value = (row.get('details') or {}).get(field)
        if (type(stamp) in (int, float) and math.isfinite(stamp)
                and type(value) is int and value >= 0 and stamp <= end):
            samples.append((stamp, index, value))
    samples.sort()
    before = [sample for sample in samples if sample[0] < begin]
    active = [sample for sample in samples if begin <= sample[0] <= end]
    baseline = before[-1] if before else None
    previous = baseline[2] if baseline else None
    count, changes, resets = 0, [], []
    for stamp, _, value in active:
        if previous is None:
            # This can be a carried counter from a previous mission. Do not
            # manufacture the missing baseline by assuming a fresh node.
            previous = value
            continue
        if value < previous:
            resets.append({'wall_time': stamp, 'previous': previous,
                           'current': value})
            increment = 0
        else:
            increment = value-previous
        if increment:
            count += increment
            changes.append({'wall_time': stamp, 'previous': previous,
                            'current': value, 'increment': increment})
        previous = value
    limitations = []
    if baseline is None:
        limitations.append('no pre-mission counter baseline; first visible value is excluded as possible carry-in')
    if resets:
        limitations.append('counter decreased without a process-epoch ID; restart versus delayed old status is ambiguous, so no authoritative count is reported')
    if not active:
        limitations.append('no valid in-mission counter samples')
    return {'count': count if active and not resets else None, 'field': field,
            'scope': 'node_lifetime', 'valid_mission_samples': len(active),
            'baseline': ({'wall_time': baseline[0], 'value': baseline[2]}
                         if baseline else None),
            'first_mission_value': active[0][2] if active else None,
            'last_mission_value': active[-1][2] if active else None,
            'counter_resets': resets, 'increments': changes,
            'observed_positive_delta_sum': count,
            'is_lower_bound': (bool(limitations) if active and not resets else None),
            'limitations': limitations,
            'definition': 'sum of observed positive node-lifetime counter deltas whose status receipt is within the mission; last pre-mission sample is baseline; goal changes and repeated values do not add attempts'}


def overall_verdict(directory):
    """The staged verifier owns overall acceptance, including obstacle actor QA."""
    path = directory.parent/'staged_report.json'
    report = load(path, {})
    entries = [entry for entry in report.get('stages', [])
               if isinstance(entry, dict) and entry.get('stage') == directory.name]
    source = str(path)
    if len(entries) != 1:
        reason = ('staged_report.json unavailable' if not path.is_file() else
                  'matching stage entry missing or ambiguous')
        return {'overall_passed': None, 'overall_reasons': None,
                'overall_unavailable_reason': reason, 'overall_report': source}
    entry = entries[0]
    passed = entry.get('passed')
    return {'overall_passed': passed if type(passed) is bool else None,
            'overall_reasons': entry.get('reasons'),
            'overall_outcome': entry.get('outcome'),
            'overall_unavailable_reason': None if type(passed) is bool else 'stage verdict is not boolean',
            'overall_report': source, 'suite_passed': report.get('passed')}


def yaw_reversals(commands):
    """Actual output turns >=.15 rad/s and >=.10 rad, reversing within 2 s."""
    episodes, current = [], None
    for a, b in zip(commands, commands[1:]):
        angular = a.get('angular', 0.)
        sign = 1 if angular >= .15 else -1 if angular <= -.15 else 0
        dt = max(0., min(.25, b['wall_time']-a['wall_time']))
        if not sign:
            continue
        if current is None or current['sign'] != sign or a['wall_time']-current['end'] > .5:
            if current is not None:
                episodes.append(current)
            current = {'sign': sign, 'start': a['wall_time'], 'end': a['wall_time'], 'angle': 0.}
        current['end'] = b['wall_time']
        current['angle'] += abs(angular)*dt
    if current is not None:
        episodes.append(current)
    qualified = [item for item in episodes if item['angle'] >= .10]
    flips = sum(a['sign'] != b['sign'] and b['start']-a['end'] <= 2.
                for a, b in zip(qualified, qualified[1:]))
    return {'count': flips, 'qualified_turn_episodes': len(qualified),
            'definition': 'actual /cmd_vel |yaw| >=0.15 rad/s, integrated >=0.10 rad; opposite turn starts within 2 s'}


def guard_metrics(rows, begin, end):
    samples = [row for row in rows if begin <= row.get('wall_time', -math.inf) <= end]
    values = [row.get('payload', row) for row in samples]
    if not values or not any(isinstance(value.get('scan_guard_code'), str)
                             and value['scan_guard_code'] for value in values):
        return {'available': False, 'reason': 'timeline omits guard codes; supply --guard-log captured from raw motion_status'}
    codes, transitions, ramp_reasons = Counter(), Counter(), Counter()
    metrics = defaultdict(list)
    prior_code, prior_hold, episodes, held_seconds = None, False, 0, 0.
    for index, (sample, value) in enumerate(zip(samples, values)):
        code = str(value.get('scan_guard_code') or 'UNREPORTED')
        hold = value.get('scan_guard_hold') is True
        codes[code] += 1
        ramp_reasons[str((value.get('command_ramp') or {}).get('reason') or 'UNREPORTED')] += 1
        for name, number in (value.get('scan_guard_metrics') or {}).items():
            if type(number) in (float, int) and math.isfinite(number):
                metrics[name].append(number)
        if hold and code != prior_code:
            transitions[code] += 1
        if hold and not prior_hold:
            episodes += 1
        if hold and index+1 < len(samples):
            held_seconds += max(0., min(.3, samples[index+1]['wall_time']-sample['wall_time']))
        prior_code, prior_hold = code, hold
    return {'available': True, 'samples': len(samples), 'code_samples': dict(codes),
            'held_code_transitions': dict(transitions), 'hold_episode_count': episodes,
            'ramp_reason_samples': dict(ramp_reasons),
            'robot_radius_m': sorted(set(metrics['robot_radius_m'])),
            'horizon_seconds': sorted(set(metrics['horizon_seconds'])),
            'minimum_body_clearance_m': min(metrics['minimum_body_clearance_m'], default=None),
            'minimum_swept_clearance_m': min(metrics['minimum_swept_clearance_m'], default=None),
            'maximum_scan_age_seconds': max(metrics['scan_age_seconds'], default=None),
            'maximum_pose_age_seconds': max(metrics['pose_age_seconds'], default=None),
            'sampled_hold_seconds': round(held_seconds, 3),
            'duration_note': 'sampled intervals capped at 0.3 s; long missing intervals are not inferred'}


def sampled_state_metrics(rows, begin, end, state, *, interval_cap=.5,
                          expected_states=()):
    """Attribute only bounded, in-mission intervals between actual samples.

    Navigation messages can be sparse status-change events, not heartbeats.
    Neither a long receive gap nor the tail after the final sample proves
    that a state persisted for that whole period.
    """
    samples = sorted((row for row in rows
                      if begin <= row.get('wall_time', -math.inf) <= end),
                     key=lambda row: row['wall_time'])
    counts = Counter({name: 0 for name in expected_states})
    seconds = Counter({name: 0. for name in expected_states})
    for index, row in enumerate(samples):
        name = state(row)
        counts[name] += 1
        seconds.setdefault(name, 0.)
        if index+1 < len(samples):
            seconds[name] += max(0., min(
                interval_cap, samples[index+1]['wall_time']-row['wall_time']))
    covered = sum(seconds.values())
    return {'available': bool(samples), 'samples': len(samples),
            'sample_counts': dict(counts),
            'sampled_seconds': {name: round(value, 3) for name, value in seconds.items()},
            'sampled_covered_seconds': round(covered, 3),
            'unmeasured_seconds': round(max(0., end-begin-covered), 3),
            'interval_cap_seconds': interval_cap,
            'duration_note': f'mission-window samples only; each interval to the next sample is capped at {interval_cap:g} s; no carry-in or duration after the final sample is inferred; sparse event streams undercount actual dwell time'}


def command_state(row):
    if abs(row.get('linear', 0.)) > .005:
        return 'TRANSLATING'
    if abs(row.get('angular', 0.)) > .01:
        return 'TURNING_IN_PLACE'
    return 'STOPPED'


def summarize(label, directory, guard_log=None):
    derived = load(directory/'derived_evidence.json', {})
    accepted = load(directory/'acceptance.json', {})
    evidence = derived or accepted
    actor = load(directory/'web_obstacle_injection.json', evidence.get('web_obstacle') or {})
    manifest = load(directory/'runtime_manifest.json', {})
    timeline_path = directory/'safety_timeline.jsonl'
    if not timeline_path.is_file():
        timeline_path = directory.with_name(directory.name+'.safety_timeline.jsonl')
    rows = jsonlines(timeline_path)
    if not rows:
        return {'label': label, 'directory': str(directory), **overall_verdict(directory),
                'error': 'no timeline; trial may still be running'}
    begin = evidence.get('mission_started_at')
    if not isinstance(begin, (int, float)):
        begin = next((r['wall_time'] for r in rows if r['kind'] == 'goal'), rows[0]['wall_time'])
    end = evidence.get('final_observed_at')
    if not isinstance(end, (int, float)):
        end = rows[-1]['wall_time']
    active = [row for row in rows if begin <= row['wall_time'] <= end]
    counters = {name: cumulative_counter_metrics(rows, field, begin, end)
                for name, field in (('local_attempt_count', 'local_attempt_id'),
                                    ('global_plan_count', 'global_plan_count'))}
    commands = [row for row in active if row['kind'] == 'cmd_vel']
    nav = [row for row in active if row['kind'] == 'navigation']
    goals = {row.get('stamp_ns') for row in active if row['kind'] == 'goal'}
    paths = {(row.get('stamp_ns'), goal_key(row.get('points'))) for row in active
             if row['kind'] == 'path' and row.get('count', 0) > 0}
    expected = Counter(evidence.get('route') or [])
    visited = Counter(evidence.get('completed_ids') or [])
    completed = (evidence.get('final_state') == 'HOME_REACHED'
                 and bool(expected) and not expected-visited)
    reverse_episodes, reversing, previous_at = 0, False, None
    for row in commands:
        now_reverse = row.get('linear', 0.) < -.01
        if now_reverse and (not reversing or row['wall_time']-(previous_at or 0.) > .3):
            reverse_episodes += 1
        reversing, previous_at = now_reverse, row['wall_time']
    guard_rows = jsonlines(guard_log) if guard_log else []
    if not guard_rows:
        guard_rows = [row for row in active if row['kind'] == 'motion']
    return {
        'label': label, 'directory': str(directory),
        **overall_verdict(directory),
        'completed_route_and_home': completed,
        'prearm_passed': (evidence.get('prearm') or {}).get('passed'),
        'runner_passed': accepted.get('passed'), 'derived_passed': derived.get('passed'),
        'actor_passed': actor.get('passed'), 'final_state': evidence.get('final_state'),
        'failure': evidence.get('failure'),
        'mission_elapsed_seconds': round(end-begin, 3),
        'goal_dispatch_count': len(goals), 'published_nonempty_path_count': len(paths),
        'published_empty_path_count': sum(r['kind'] == 'path' and r.get('count') == 0 for r in active),
        'local_attempt_count': counters['local_attempt_count']['count'],
        'global_plan_count': counters['global_plan_count']['count'],
        'counter_aggregation': {'version': 2, **counters},
        'initial_spin_start_count': sum('Rotating in place to face the initial' in str(row.get('message')) for row in nav),
        'large_yaw_reversals': yaw_reversals(commands),
        'reverse_motion_episodes': reverse_episodes,
        'recovery_request_count': sum(r['kind'] == 'recovery_request' for r in active),
        'navigation_status': sampled_state_metrics(
            nav, begin, end, lambda row: str(row.get('status') or 'UNREPORTED'),
            expected_states=('WAIT_REPLAN', 'CONTROLLER_HOLD')),
        'actual_command_state': {
            **sampled_state_metrics(commands, begin, end, command_state,
                expected_states=('STOPPED', 'TURNING_IN_PLACE', 'TRANSLATING')),
            'thresholds': {'linear_m_per_s': .005, 'angular_rad_per_s': .01},
            'interpretation': 'zero actual output is not proof of a scan-guard hold; compare navigation_status and guard separately'},
        'guard': guard_metrics(guard_rows, begin, end),
        'tracking_error': {'max_m': None, 'rms_m': None,
            'reason': 'published replacement paths can precede FollowPath cancellation/acceptance; active action path is not uniquely correlated in this timeline'},
        'minimum_actual_box_clearance_m': actor.get('minimum_clearance_m'),
        'box_clearance_radius_m': (actor.get('geometry') or {}).get('robot_radius'),
        'actor_moving_steps': actor.get('moving_steps'),
        'actor_local_bypass_confirmed': actor.get('bypassed_while_live'),
        'footprint': manifest.get('sim_footprint_overrides'),
        'timeline': str(timeline_path), 'guard_log': str(guard_log) if guard_log else None,
    }


def pairs(values):
    result = {}
    for value in values:
        name, path = value.split('=', 1)
        if not name or name in result:
            raise ValueError('trial names must be nonempty and unique')
        result[name] = Path(path).expanduser().resolve()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', action='append', required=True, metavar='NAME=DIRECTORY')
    parser.add_argument('--guard-log', action='append', default=[], metavar='NAME=JSONL')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    trials, guards = pairs(args.trial), pairs(args.guard_log)
    if args.output.exists():
        parser.error('--output already exists; choose a new filename to preserve historic evidence')
    report = {'schema': 2, 'single_trial_per_profile': True,
              'counter_aggregation_version': 2,
              'limitation': 'One run per value compares these scenarios, not statistically proven reliability.',
              'trials': [summarize(name, directory, guards.get(name)) for name, directory in trials.items()]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
