#!/usr/bin/env python3
"""Derive actual reverse/stop observations from a portable recorded subset.

No ROS process, inferred odometry or success status is generated. Truth gaps
are sampled bounds, not a continuous collision certificate or a trial PASS.
"""
import json
import math
from pathlib import Path


def derive(fixture):
    records = fixture['records']
    request = next(row for row in records if row['kind'] == 'recovery_request')
    payload = request['payload']
    episode = payload['episode_id']
    owned = [row for row in records if row['kind'] == 'navigation'
             and (row.get('details') or {}).get('episode_id') == episode]
    begin = next(row for row in owned if row['message'] == 'Nav2 collision-checked reverse started')
    end = next(row for row in owned if row['status'] == 'RECOVERY_SUCCEEDED')
    commands = [row for row in records if row['kind'] == 'cmd_vel'
                and begin['t'] <= row['t'] <= end['t']]
    negative = [row for row in commands if row['linear'] < -.01]
    truth = [row for row in records if row['kind'] == 'truth'
             and begin['t'] <= row['t'] <= end['t']]
    odom = [row for row in records if row['kind'] == 'odom'
            and begin['t']-.3 <= row['t'] < begin['t']]
    stopped_commands = [row for row in records if row['kind'] == 'cmd_vel'
                        and begin['t']-.3 <= row['t'] < begin['t']]
    moves = [event for event in fixture['actor_events']
             if event['event'] in ('obstacle_spawned', 'obstacle_moved')]
    radius = fixture['body_radius_m']
    half_x, half_y = [value/2 for value in fixture['box_size_m'][:2]]
    gaps = []
    for row in truth:
        previous = max((item for item in moves if item['at'] <= row['wall_time']),
                       key=lambda item: item['at'])
        following = min((item for item in moves if item['at'] > row['wall_time']),
                        key=lambda item: item['at'], default=previous)
        # Conservatively cover either endpoint and the entire axis-aligned
        # swept square between successive logged actor positions.
        x_min, x_max = min(previous['x'], following['x'])-half_x, max(previous['x'], following['x'])+half_x
        y_min, y_max = min(previous['y'], following['y'])-half_y, max(previous['y'], following['y'])+half_y
        dx, dy = max(x_min-row['x'], 0., row['x']-x_max), max(y_min-row['y'], 0., row['y']-y_max)
        gaps.append(math.hypot(dx, dy)-radius)
    distance = math.hypot(truth[-1]['x']-truth[0]['x'], truth[-1]['y']-truth[0]['y'])
    return {
        'scope': 'Recorded one reverse only; whole trial remains strict FAIL',
        'episode_id': episode, 'goal_id': payload['goal_id'],
        'request_t': request['t'], 'start_t': begin['t'], 'terminal_t': end['t'],
        'terminal': end['status'], 'requested_distance_m': payload['distance'],
        'negative_output_samples': len(negative),
        'negative_output_first_last': negative[:1]+negative[-1:],
        'truth_displacement_m': distance, 'truth_samples': len(truth),
        'truth_first_last': truth[:1]+truth[-1:],
        'maximum_truth_sample_gap_s': max(b['t']-a['t'] for a,b in zip(truth,truth[1:])),
        'sampled_swept_box_clearance_lower_bound_m': min(gaps),
        'pre_dispatch_300ms_odom_samples': len(odom),
        'pre_dispatch_max_odom_linear_mps': max(abs(row['linear_x']) for row in odom),
        'pre_dispatch_max_odom_angular_radps': max(abs(row['angular_z']) for row in odom),
        'pre_dispatch_output_samples': len(stopped_commands),
        'pre_dispatch_outputs_all_zero': all(abs(row['linear'])<1e-5 and abs(row['angular'])<1e-5 for row in stopped_commands),
        'cold_start_proof': fixture['cold_start_proof'],
        'whole_trial': fixture['whole_trial'],
        'retained_stop_qa_violation': fixture['stop_qa_violation'],
        'limits': ['sampled Gazebo truth, not physical robot validation',
                   'no reconstructed sensor/odom/health/terminal',
                   'no continuous collision guarantee between truth samples',
                   'actual actor bypass failed; stop-rotation QA remains failed'],
    }


if __name__ == '__main__':
    fixture = json.loads(Path(__file__).with_name('fixture.json').read_text())
    print(json.dumps(derive(fixture), ensure_ascii=False, indent=2))
