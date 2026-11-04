#!/usr/bin/env python3
"""Run real Nav2 controller checks in an isolated loopback domain, without motors.

Synthetic scans provide straight, corner, obstacle, and final-goal checks.
An optional captured incident adds a before/after folded-route reproduction.
These are frozen controller-input checks; Gazebo/physical drive completion is
reported separately and cannot be inferred from this suite.
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent


def pose(x, y, yaw=0.):
    return {'position': {'x': x, 'y': y, 'z': 0.},
            'orientation': {'x': 0., 'y': 0., 'z': math.sin(yaw/2.),
                            'w': math.cos(yaw/2.)}}


def sampled_path(vertices, final_yaw=0.):
    points = []
    for start, end in zip(vertices, vertices[1:]):
        dx, dy = end[0]-start[0], end[1]-start[1]
        count = max(1, math.ceil(math.hypot(dx, dy)/.025))
        yaw = math.atan2(dy, dx)
        points.extend(pose(start[0]+dx*index/count, start[1]+dy*index/count, yaw)
                      for index in range(count))
    points.append(pose(*vertices[-1], final_yaw))
    header = {'frame_id': 'map', 'stamp': {'sec': 1, 'nanosec': 0}}
    return {'header': header, 'poses': [{'header': header, 'pose': p} for p in points]}


def ray_box(origin, direction, box):
    near, far = -math.inf, math.inf
    for axis in (0, 1):
        if abs(direction[axis]) < 1.e-12:
            if origin[axis] < box[axis] or origin[axis] > box[axis+2]:
                return math.inf
        else:
            a, b = ((box[axis]-origin[axis])/direction[axis],
                    (box[axis+2]-origin[axis])/direction[axis])
            near, far = max(near, min(a, b)), min(far, max(a, b))
    return near if 0. <= near <= far else math.inf


def fixture(robot, vertices, *, boxes=(), final_yaw=0., speed_limit=0.):
    x, y, yaw = robot
    header = {'frame_id': 'base_scan', 'stamp': {'sec': 1, 'nanosec': 0}}
    origin = (x-.064*math.cos(yaw), y-.064*math.sin(yaw))
    rays = []
    for index in range(360):
        angle = yaw+index*math.pi/180.
        distance = min((ray_box(origin, (math.cos(angle), math.sin(angle)), box)
                        for box in boxes), default=math.inf)
        rays.append(distance if .1 <= distance <= 4. else math.inf)
    scan = {'header': header, 'angle_min': 0., 'angle_max': 359*math.pi/180.,
            'angle_increment': math.pi/180., 'time_increment': 0., 'scan_time': .1,
            'range_min': .1, 'range_max': 4., 'ranges': rays, 'intensities': []}
    identity = {'translation': {'x': 0., 'y': 0., 'z': 0.},
                'rotation': pose(0., 0.)['orientation']}
    transforms = [{'header': {'frame_id': 'map', 'stamp': {'sec': 1, 'nanosec': 0}},
                   'child_frame_id': 'odom', 'transform': identity},
                  {'header': {'frame_id': 'odom', 'stamp': {'sec': 1, 'nanosec': 0}},
                   'child_frame_id': 'base_footprint',
                   'transform': {'translation': {'x': x, 'y': y, 'z': 0.},
                                 'rotation': pose(x, y, yaw)['orientation']}}]
    odom = {'header': {'frame_id': 'odom', 'stamp': {'sec': 1, 'nanosec': 0}},
            'child_frame_id': 'base_footprint', 'pose': {'pose': pose(x, y, yaw)}}
    return [{'ros_time_ns': 1000000000, 'topic': topic, 'data': data}
            for topic, data in (('/scan', scan), ('/odom', odom),
                                ('/tf', {'transforms': transforms}),
                                ('/speed_limit', {'speed_limit': speed_limit, 'percentage': False}),
                                ('/smart_vision_amr/path', sampled_path(vertices, final_yaw)))]


def turn_limit(w):
    yaw = abs(w)
    return (.22-yaw/.08*.07 if yaw < .08 else
            .15-min(1., (yaw-.08)/.27)*.05)


def assess(case, observations):
    commands = [event['data'] for event in observations if event['kind'] == 'command']
    moving = [command for command in commands if abs(command['linear']['x']) > 1.e-6
              or abs(command['angular']['z']) > 1.e-6]
    evaluations = [event['data'] for event in observations if event['kind'] == 'evaluation']
    errors = []
    for command in commands:
        v, w = command['linear']['x'], command['angular']['z']
        if v > turn_limit(w)+1.e-8:
            errors.append('command exceeds physical mux turn envelope')
    if case in ('straight', 'corner', 'incident_fixed') and not any(
            command['linear']['x'] >= .05 for command in commands):
        errors.append('no forward command of at least 0.05 m/s')
    if case == 'corner' and not any(abs(cmd['angular']['z']) >= .08 for cmd in moving):
        errors.append('corner did not generate a steering command')
    if case == 'arrival':
        results = [event['data'] for event in observations if event['kind'] == 'action_result_status']
        if 4 not in results:
            errors.append('actual final action goal did not succeed')
        if moving:
            errors.append('already-reached goal produced motion')
    if case == 'footprint_obstacle':
        if not any(event['invalid_by_critic'].get('ObstacleFootprint', 0)
                   for event in evaluations):
            errors.append('full-body collision candidates were not rejected')
        # The ordered BaseObstacle critic runs before ObstacleFootprint. A
        # footprint rejection therefore proves a body contact that passed the
        # center critic, even if other curved candidates also hit inflated cells.
        # Requiring *all* centers to stay free made this fixture depend on the
        # 5 cm grid phase when changing the configured physical radius.
        if not any(any(score['name'] == 'BaseObstacle'
                       and math.isfinite(score['raw_score']) and score['raw_score'] >= 0.
                       for score in (event.get('best') or {}).get('scores', []))
                   for event in evaluations):
            errors.append('valid near-obstacle trajectory must have a nonlethal center score')
        if not any(command['linear']['x'] >= .05 for command in commands):
            errors.append('valid near-obstacle forward candidates were lost')
    if case == 'obstacle':
        if not any(event['invalid_by_critic'].get('BaseObstacle', 0) for event in evaluations):
            errors.append('obstacle collision candidates were not rejected')
        if any(score['raw_score'] < 0. for event in evaluations
               for score in (event.get('best') or {}).get('scores', [])):
            errors.append('selected trajectory contains an invalid collision score')
    if case == 'incident_original' and not any(
            abs(cmd['linear']['x']) < .01 and abs(cmd['angular']['z']) > .12 for cmd in moving):
        errors.append('original pure-yaw incident did not reproduce')
    return {'case': case, 'passed': not errors, 'errors': sorted(set(errors)),
            'command_count': len(commands), 'evaluation_count': len(evaluations),
            'max_linear': max((cmd['linear']['x'] for cmd in commands), default=0.),
            'max_abs_angular': max((abs(cmd['angular']['z']) for cmd in commands), default=0.)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav-params', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--incident', type=Path)
    parser.add_argument('--incident-time', type=float, default=1793781696.5)
    parser.add_argument('--domain', type=int, default=186)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output directory already exists')
    profile = yaml.safe_load(args.nav_params.read_text())
    critics = profile['controller_server']['ros__parameters']['FollowPath']['critics']
    if ('BaseObstacle' not in critics or 'ObstacleFootprint' not in critics
            or critics.index('BaseObstacle') >= critics.index('ObstacleFootprint')):
        parser.error('regression evidence requires BaseObstacle before ObstacleFootprint')
    radius = float(profile['local_costmap']['local_costmap']['ros__parameters']['robot_radius'])
    if not math.isfinite(radius) or radius <= 0.:
        parser.error('local_costmap robot_radius must be positive and finite')
    args.output.mkdir(parents=True)
    # A wide synthetic corridor gives raytracing valid returns in every view.
    walls = [(-2., -.55, 2., -.5), (-2., .5, 2., .55), (1.8, -.55, 1.85, .55),
             (-1.8, -.55, -1.75, .55)]
    cases = {
        'straight': fixture((0., 0., 0.), [(0., 0.), (1.2, 0.)], boxes=walls),
        'corner': fixture((.35, 0., 0.), [(0., 0.), (.55, 0.), (.55, .8)],
                          boxes=[(-2., -2., 2., -1.9), (-2., 1.9, 2., 2.)]),
        'obstacle': fixture((0., 0., 0.), [(0., 0.), (1.2, 0.)],
                            boxes=walls+[(.38, -.18, .62, .18)]),
        'arrival': fixture((1.2, 0., 0.), [(0., 0.), (1.2, 0.)], boxes=walls),
        # A 15 mm side gap leaves trajectory centers free while a few sampled
        # curved candidates graze the box with the configured body border.
        'footprint_obstacle': fixture((0., 0., math.pi/2), [(0., 0.), (0., 1.2)],
            boxes=[(radius+.015, .375, radius+.265, .625), (-1.2, -1., -1.15, 2.),
                   (1.15, -1., 1.2, 2.)], final_yaw=math.pi/2),
    }
    reports = []
    for name, events in cases.items():
        path = args.output / f'{name}.jsonl'
        path.write_text('\n'.join(json.dumps(event) for event in events))
        commands = [sys.executable, str(SCRIPT_DIR/'replay_incident_controller.py'), str(path),
                    '--at-time', '1.', '--nav-params', str(args.nav_params.resolve()),
                    '--output', str(args.output/name), '--duration', '2.', '--domain', str(args.domain),
                    '--override', 'FollowPath.plugin=smart_vision_amr_dwb::RouteTrackingDWB']
        subprocess.run(commands, check=True, stdout=subprocess.DEVNULL)
        observations = [json.loads(line) for line in (args.output/name/'observations.jsonl').read_text().splitlines()]
        reports.append(assess(name, observations))
    if args.incident:
        for name, plugin in (('incident_original', 'dwb_core::DWBLocalPlanner'),
                             ('incident_fixed', 'smart_vision_amr_dwb::RouteTrackingDWB')):
            subprocess.run([sys.executable, str(SCRIPT_DIR/'replay_incident_controller.py'), str(args.incident),
                            '--at-time', str(args.incident_time), '--nav-params', str(args.nav_params.resolve()),
                            '--output', str(args.output/name), '--duration', '2.', '--domain', str(args.domain),
                            '--override', f'FollowPath.plugin={plugin}'],
                           check=True, stdout=subprocess.DEVNULL)
            observations = [json.loads(line) for line in (args.output/name/'observations.jsonl').read_text().splitlines()]
            # The baseline intentionally predates the envelope implementation.
            result = assess(name, observations)
            if name == 'incident_original':
                result['errors'] = [error for error in result['errors'] if 'envelope' not in error]
                result['passed'] = not result['errors']
            reports.append(result)
    output = {'all_passed': all(report['passed'] for report in reports), 'cases': reports,
              'robot_radius_m': radius,
              'scope': 'real Nav2 DWB with frozen loopback inputs; not a closed-loop driving pass'}
    (args.output/'results.json').write_text(json.dumps(output, indent=2))
    print(json.dumps(output, indent=2))
    return 0 if output['all_passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
