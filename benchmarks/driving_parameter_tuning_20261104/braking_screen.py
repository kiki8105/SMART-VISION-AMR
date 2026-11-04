#!/usr/bin/env python3
"""Offline speed-ceiling comparison; not a closed-loop driving result."""
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.path_geometry import (  # noqa: E402
    anticipatory_curve_speed_limit, path_length, resample_path)


def main():
    recorded = ROOT/'benchmarks/controller_replay_20261104/recorded_frozen_snapshot.jsonl.gz'
    with gzip.open(recorded, 'rt') as stream:
        events = [json.loads(line) for line in stream if line.strip()]
    entry = next(event for event in reversed(events)
                 if event['topic'] == '/smart_vision_amr/path' and event['data']['poses'])
    captured = [(pose['pose']['position']['x'], pose['pose']['position']['y'])
                for pose in entry['data']['poses']]
    routes = {
        'sparse_bend_calibration': [(0., 0.), (.95, 0.), (1.2, 0.),
                                    (1.2, .25), (1.2, 1.2)],
        'dense_ninety_degree_bend': resample_path([(0., 0.), (1.2, 0.), (1.2, 1.2)], .05),
        'captured_folded_route': captured,
    }
    rows = []
    for label, points in routes.items():
        length = path_length(points)
        progress = [index*.01 for index in range(math.floor(length/.01)+1)]
        for horizon in (.6, 1., 1.2):
            for margin in (.15, .25, .35):
                caps = [anticipatory_curve_speed_limit(
                    points, distance, horizon, .22, .35, .10,
                    minimum_speed=.12, deceleration=.10,
                    braking_margin=margin, minimum_chord=.15)
                    for distance in progress]
                assert all(.12-1.e-9 <= speed <= .22+1.e-9 for speed in caps)
                max_decel = max((max(0., (a*a-b*b)/(.02))
                                 for a, b in zip(caps, caps[1:])), default=0.)
                braking = next((distance for distance, cap in zip(progress, caps)
                                if cap < .22-1.e-5), None)
                rows.append({
                    'route': label, 'preview_m': horizon, 'braking_margin_m': margin,
                    'first_cap_reduction_at_progress_m': braking,
                    'theoretical_minimum_time_at_ceiling_s': round(
                        sum(.01/speed for speed in caps[:-1]), 4),
                    'minimum_ceiling_mps': round(min(caps), 6),
                    'maximum_implied_braking_deceleration_mps2': round(max_decel, 6),
                    'caps_sha256': hashlib.sha256(json.dumps(caps).encode()).hexdigest(),
                })
    corner = [row for row in rows if row['route'] == 'sparse_bend_calibration']
    for margin in (.15, .25, .35):
        onset = next(row for row in corner if row['preview_m'] == 1.
                     and row['braking_margin_m'] == margin)['first_cap_reduction_at_progress_m']
        # .22 -> .12 at a=.10: 0.17 m braking distance plus selected margin.
        assert onset is not None and abs(onset-(1.2-.17-margin)) <= .021
    output = {
        'scope': 'same production speed-ceiling helper on fixed paths; no motors, tracking, arrival or obstacle pass inferred',
        'input_sha256': hashlib.sha256(recorded.read_bytes()).hexdigest(),
        'fixed': {'maximum_forward_mps': .22, 'minimum_bend_ceiling_mps': .12,
                  'maximum_yaw_radps': .35, 'lateral_acceleration_mps2': .10,
                  'braking_deceleration_mps2': .10, 'sampling_m': .01},
        'rows': rows,
        'recommendation': 'Screen 0.25 m braking margin in closed loop against current 0.15 m. Preview changes are not assumed to improve tracking.',
    }
    destination = Path(__file__).with_name('braking_screen.json')
    destination.write_text(json.dumps(output, indent=2)+'\n')
    print(destination)


if __name__ == '__main__':
    main()
