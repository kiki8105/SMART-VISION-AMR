#!/usr/bin/env python3
"""Recompute the archived witnesses without ROS, Gazebo, or /tmp trial files."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def nearest(points, pose):
    best = None
    progress = 0.0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if not length:
            continue
        fraction = min(1.0, max(0.0, ((pose[0] - a[0]) * dx
                                    + (pose[1] - a[1]) * dy) / length**2))
        projection = [a[0] + fraction * dx, a[1] + fraction * dy]
        record = {
            'distance_m': math.dist(pose, projection),
            'progress_m': progress + fraction * length,
            'projection': projection,
            'segment': index,
        }
        if best is None or record['distance_m'] < best['distance_m']:
            best = record
        progress += length
    if best is None:
        raise ValueError('A route needs at least one nonzero segment')
    return best


def point_at(points, arc):
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if not length:
            continue
        if arc <= length:
            return [a[0] + (b[0] - a[0]) * arc / length,
                    a[1] + (b[1] - a[1]) * arc / length]
        arc -= length
    return points[-1]


def body_box_gap(pose, box):
    return math.hypot(max(abs(pose[0] - box[0]) - 0.125, 0.0),
                      max(abs(pose[1] - box[1]) - 0.125, 0.0)) - 0.22


def assert_close(actual, expected, label, tolerance=1e-10):
    if not math.isfinite(actual) or abs(actual - expected) > tolerance:
        raise AssertionError(f'{label}: recomputed {actual}, recorded {expected}')


def owned_intervals(case):
    routes = {route['stamp_ns']: route for route in case['published_routes']}
    intervals = []
    owned = None
    started = None
    for event in case['ownership_events']:
        if event['kind'] == 'path':
            next_owned = None
        else:
            stamp = (event.get('details') or {}).get('path_stamp')
            ns = (stamp['sec'] * 1_000_000_000 + stamp['nanosec']
                  if isinstance(stamp, dict) else None)
            if (event['status'] == 'MOVING' and ns in routes
                    and event.get('message') != 'Rotating in place to face the initial A* segment'):
                assert routes[ns]['wall_time'] <= event['wall_time']
                next_owned = ns
            elif event['status'] in ('FAILED', 'SUCCEEDED', 'CANCELED', 'WAIT_REPLAN', 'RECOVERING'):
                next_owned = None
            else:
                continue
        if next_owned != owned:
            if owned is not None:
                intervals.append((owned, started, event['wall_time']))
            owned = next_owned
            started = event['wall_time'] if owned is not None else None
    if owned is not None:
        # Only supplied witness timestamps, never extrapolate execution duration.
        intervals.append((owned, started, float('inf')))
    return routes, intervals


def verify(folder, verify_original_inputs=False):
    fixture = json.loads((folder / 'accepted_routes_fixture.json').read_text())
    report = json.loads((folder / 'tracking_report.json').read_text())
    before_hash = hashlib.sha256(
        (folder / 'path_follower_before_material_replan_alignment.py').read_bytes()).hexdigest()
    result = {
        'scope': 'Offline arithmetic and exact-stamp ownership witness verification; not a controller replay or new whole-system pass.',
        'requires_ros': False,
        'original_inputs_required': verify_original_inputs,
        'snapshot_follower_sha256': before_hash,
        'cases': {},
    }
    for label, case in fixture['cases'].items():
        assert case['follower_source_sha256'] == before_hash
        assert case['recorded_source_before_equals_after']
        if verify_original_inputs:
            for name, expected in case['input_sha256'].items():
                assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected, name
        routes, intervals = owned_intervals(case)
        witnesses = report['cases'][label]['witness_samples']
        for witness in witnesses:
            stamp, at = witness['path_stamp_ns'], witness['wall_time']
            assert any(stamp == ns and start <= at < end for ns, start, end in intervals), (label, stamp, at)
            points = routes[stamp]['points']
            truth = nearest(points, witness['truth_xy'])
            estimated = nearest(points, witness['estimated_xy'])
            assert 0 <= witness['map_tf_age_s'] <= 0.25
            assert 0 <= witness['odom_tf_age_s'] <= 0.10
            assert_close(truth['distance_m'], witness['truth_cross_track']['distance_m'], 'truth cross-track')
            assert_close(estimated['distance_m'], witness['estimated_cross_track']['distance_m'], 'estimated cross-track')
            assert_close(math.dist(witness['truth_xy'], witness['estimated_xy']),
                         witness['truth_estimate_error_m'], 'truth to estimated position')
            assert_close(body_box_gap(witness['truth_xy'], witness['box_xy']),
                         witness['truth_body_box_gap_m'], 'actual body-box gap')
            assert_close(body_box_gap(truth['projection'], witness['box_xy']),
                         witness['projection_body_box_gap_m'], 'projected route body-box gap')
        headings = report['cases'][label]['owned_start_end_heading_and_commands']
        for heading in headings:
            witness = next(w for w in witnesses if w['path_stamp_ns'] == heading['path_stamp_ns']
                           and w['t'] == heading['observer_t'])
            points = routes[heading['path_stamp_ns']]['points']
            progress = nearest(points, witness['estimated_xy'])['progress_m']
            target = point_at(points, progress + 0.15)
            bearing = math.atan2(target[1] - witness['estimated_xy'][1],
                                 target[0] - witness['estimated_xy'][0])
            error = math.atan2(math.sin(bearing - heading['estimated_yaw_rad']),
                               math.cos(bearing - heading['estimated_yaw_rad']))
            assert_close(math.degrees(error), heading['reconstructed_heading_error_deg'], '15cm lookahead heading')
        last_stamp = max(w['path_stamp_ns'] for w in witnesses)
        final = [w for w in witnesses if w['path_stamp_ns'] == last_stamp]
        last = final[-1]
        result['cases'][label] = {
            'verified_truth_witnesses': len(witnesses),
            'verified_heading_witnesses': len(headings),
            'exact_stamp_owned_intervals': len(intervals),
            'first_local_heading_error_deg': headings[0]['reconstructed_heading_error_deg'],
            'last_local_truth_cross_track_max_m': max(w['truth_cross_track']['distance_m'] for w in final),
            'last_local_estimated_cross_track_max_m': max(w['estimated_cross_track']['distance_m'] for w in final),
            'last_local_final_projection_body_box_gap_m': last['projection_body_box_gap_m'],
            'last_local_final_actual_body_box_gap_m': last['truth_body_box_gap_m'],
            'whole_system_outcome': 'FAIL',
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--verify-original-inputs', action='store_true',
                        help='Also require the original /tmp trial files and verify their full hashes')
    parser.add_argument('--output', type=Path, help='Write JSON results to a new path')
    args = parser.parse_args()
    result = verify(args.folder, args.verify_original_inputs)
    rendered = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(rendered)
    else:
        print(rendered, end='')


if __name__ == '__main__':
    main()
