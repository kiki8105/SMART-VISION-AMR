#!/usr/bin/env python3
"""Offline stronger-center screen; 50% one-core CPU duty, no ROS Node or DDS."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from math import atan2, cos, hypot, sin
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[2]
PACKAGE = REPO/'ros2_ws/src/smart_vision_amr'
sys.path[:0] = [str(PACKAGE), str(PACKAGE/'test')]

import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid

from smart_vision_amr.path_geometry import path_length, resample_path
from smart_vision_amr.web_mission_server import load_saved_map
from test_local_detour_planner import path_box_clearance, ready
from test_occlusion_reserve_incident import load_incident
from test_planner_failure_memory import FailureHarness
from test_planner_recovery import make_goal


def point_square_distance(points, lower, upper):
    return np.asarray([np.hypot(np.maximum(np.maximum(lower[:, 0]-x, x-upper[:, 0]), 0.),
                                 np.maximum(np.maximum(lower[:, 1]-y, y-upper[:, 1]), 0.)).min()
                       for x, y in points])


def continuous_square_clearance(points, lower, upper):
    """Exact segment-to-axis-aligned-squares distance, including intersections."""
    best = float('inf')
    for a, b in zip(points, points[1:]):
        a, b = np.asarray(a), np.asarray(b)
        delta = b-a
        entry, leave = np.zeros(len(lower)), np.ones(len(lower))
        intersects = np.ones(len(lower), dtype=bool)
        for axis in (0, 1):
            if abs(delta[axis]) < 1e-14:
                intersects &= (lower[:, axis] <= a[axis]) & (a[axis] <= upper[:, axis])
            else:
                first = (lower[:, axis]-a[axis])/delta[axis]
                last = (upper[:, axis]-a[axis])/delta[axis]
                entry = np.maximum(entry, np.minimum(first, last))
                leave = np.minimum(leave, np.maximum(first, last))
        if np.any(intersects & (entry <= leave)):
            return 0.
        best = min(best, *point_square_distance((a, b), lower, upper))
        squared = float(delta@delta)
        if squared:
            for ix, iy in ((0, 0), (0, 1), (1, 0), (1, 1)):
                corners = np.column_stack(((upper if ix else lower)[:, 0],
                                           (upper if iy else lower)[:, 1]))
                fraction = np.clip(((corners-a)*delta).sum(axis=1)/squared, 0., 1.)
                difference = a+fraction[:, None]*delta-corners
                best = min(best, float(np.hypot(difference[:, 0], difference[:, 1]).min()))
    return best


def geometry_metrics(points, lower, upper, bounds, radius, box=None):
    if len(points) < 2:
        return {'feasible': False}
    lengths = [hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])]
    headings = [atan2(b[1]-a[1], b[0]-a[0])
                for a, b, length in zip(points, points[1:], lengths) if length > 1e-8]
    variation = sum(abs(atan2(sin(b-a), cos(b-a))) for a, b in zip(headings, headings[1:]))
    minimum = continuous_square_clearance(points, lower, upper)
    xmin, ymin, xmax, ymax = bounds
    minimum = min(minimum, min(min(x-xmin, xmax-x, y-ymin, ymax-y) for x, y in points))
    # These samples are for averages, not the continuous minimum certificate.
    samples = resample_path(points, .05)
    middle = samples[6:-6] if len(samples) > 14 else samples
    clearances = point_square_distance(middle, lower, upper)-radius
    return {
        'feasible': True, 'length_m': sum(lengths),
        'heading_total_variation_rad': variation,
        'static_center_min_m': minimum, 'static_body_gap_min_m': minimum-radius,
        'interior_static_body_gap_mean_m': float(np.mean(clearances)),
        'interior_static_body_gap_p10_m': float(np.quantile(clearances, .1)),
        'exact_box_body_gap_min_m': path_box_clearance(points, box)-radius if box else None,
    }


def fresh(node, x, y, yaw=0.):
    ready(node, x=x, y=y, yaw=yaw)
    node.last_observation_time = 100.


def setup_clean(profile, metadata, pixels, start, goal):
    node = FailureHarness({'steady': 100., 'ros': 100.})
    node.params.update(profile)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height = metadata['width'], metadata['height']
    msg.info.resolution = metadata['resolution']
    msg.info.origin.position.x, msg.info.origin.position.y = metadata['origin']['x'], metadata['origin']['y']
    msg.info.origin.orientation.w = 1.
    msg.data = pixels.flatten().tolist()
    node.on_map(msg)
    fresh(node, start['x'], start['y'], start['yaw'])
    node.last_goal = make_goal(stamp=100)
    node.last_goal.pose.position.x, node.last_goal.pose.position.y = goal['x'], goal['y']
    node.last_goal.pose.orientation.z = sin(goal['yaw']/2)
    node.last_goal.pose.orientation.w = cos(goal['yaw']/2)
    return node


_cpu_mark, _wall_mark = time.process_time(), time.perf_counter()


def pause_case():
    """Bound this low-priority offline worker to about half of one CPU core."""
    global _cpu_mark, _wall_mark
    cpu, wall = time.process_time(), time.perf_counter()
    delay = max(0., 2.*(cpu-_cpu_mark)-(wall-_wall_mark))
    if delay:
        time.sleep(delay)
    _cpu_mark, _wall_mark = time.process_time(), time.perf_counter()


def screen(output):
    started = time.perf_counter()
    map_file = PACKAGE/'maps/classroom_20261030_cleaned.yaml'
    marker_file = PACKAGE/'config/aruco_classroom_markers_20261030.yaml'
    profile_file = PACKAGE/'config/classroom_hardware_nodes.yaml'
    metadata, pixels = load_saved_map(map_file)
    assert metadata['origin']['yaw'] == 0., 'This exact square metric uses the native unrotated map.'
    profile = yaml.safe_load(profile_file.read_text())['/astar_planner']['ros__parameters']
    assert profile['robot_radius'] == .22 and profile['obstacle_inflation_cells'] == 5
    markers = {m['id']: m for m in yaml.safe_load(marker_file.read_text())['markers']}
    routes = [[0, 1, 2, 3, 0], [0, 1, 3, 2, 0]]
    legs = sorted(set(zip(routes[0], routes[0][1:])) | set(zip(routes[1], routes[1][1:])))
    r = metadata['resolution']
    origin = np.asarray([metadata['origin']['x'], metadata['origin']['y']])
    cells = np.argwhere(pixels != 0)[:, ::-1]
    lower, upper = cells*r+origin, (cells+1)*r+origin
    bounds = [origin[0], origin[1], origin[0]+metadata['width']*r, origin[1]+metadata['height']*r]
    fixture_names = [
        'occlusion_reserve_inflation022_20261104.json',
        'occlusion_reserve_second_first_20261104.json',
        'occlusion_reserve_second_memory_20261104.json',
    ]
    cert_name = 'occlusion_reserve_certified_start_20261104.json'
    combinations = []
    for preferred in (.45,):
        for gain in (1.5, 2.2, 3., 4., 6.):
            case_profile = dict(profile, preferred_clearance=preferred, clearance_penalty_gain=gain)
            cases = []
            with patch('smart_vision_amr.astar_planner.time.monotonic', return_value=100.):
                for start_id, goal_id in legs:
                    node = setup_clean(case_profile, metadata, pixels, markers[start_id], markers[goal_id])
                    node.plan_to(node.last_goal)
                    case = dict(id=f'clean_{start_id}_{goal_id}', kind='clean', start_sector=start_id,
                                goal_sector=goal_id, **geometry_metrics(node.planned_world_points, lower, upper,
                                                                       bounds, .22))
                    case['points'] = node.planned_world_points
                    cases.append(case)
                    pause_case()
                for name in fixture_names+[cert_name]:
                    for kind in (('box_global', 'box_local') if name != cert_name else ('certified_global',)):
                        node, recorded = load_incident(PACKAGE/'test/fixtures'/name)
                        node.params.update(case_profile)
                        if 'pose_world' in recorded:
                            x, y, yaw = recorded['pose_world']
                        else:
                            x, y = recorded['start_cell'];x, y, yaw = origin[0]+(x+.5)*r, origin[1]+(y+.5)*r, 0.
                        fresh(node, x, y, yaw)
                        if kind == 'box_local':
                            node.planned_world_points = list(node.local_reference_points)
                            accepted = node._try_local_detour(node.observed|node.remembered)
                            points = node.planned_world_points if accepted else []
                        else:
                            node.plan_to(node.last_goal)
                            points = node.planned_world_points
                        case = dict(id=f'{kind}:{name}', kind=kind, fixture=name,
                                    **geometry_metrics(points, lower, upper, bounds, .22,
                                                       recorded['actor']['bounds']))
                        case['details'] = {key:node.plan_metrics.get(key) for key in (
                            'occlusion_constraint_mode', 'occlusion_fallback', 'occlusion_reused_start_certificate',
                            'detour_reason', 'expansions', 'candidates_examined', 'search_duration_ms')}
                        case['points'] = points
                        cases.append(case)
                        pause_case()
            clean = {c['id']:c for c in cases if c['kind'] == 'clean'}
            route_summaries = []
            for sequence in routes:
                elements = [clean[f'clean_{a}_{b}'] for a, b in zip(sequence, sequence[1:])]
                feasible = all(c['feasible'] for c in elements)
                route_summaries.append(dict(sequence=sequence, feasible=feasible,
                    length_m=sum(c['length_m'] for c in elements) if feasible else None,
                    heading_total_variation_rad=sum(c['heading_total_variation_rad'] for c in elements) if feasible else None,
                    static_body_gap_min_m=min(c['static_body_gap_min_m'] for c in elements) if feasible else None))
            feasible = all(c['feasible'] for c in cases)
            physical_clear = feasible and all(c['static_body_gap_min_m'] >= -1e-8 and
                (c['exact_box_body_gap_min_m'] is None or c['exact_box_body_gap_min_m'] >= -1e-8) for c in cases)
            result = dict(preferred_clearance=preferred, gain=gain, all_cases_feasible=feasible,
                          all_cases_noncontact=physical_clear, routes=route_summaries, cases=cases)
            if feasible:
                clean_cases = [c for c in cases if c['kind'] == 'clean']
                result['summary'] = dict(
                    clean_unique_length_sum_m=sum(c['length_m'] for c in clean_cases),
                    clean_interior_gap_mean_m=float(np.mean([c['interior_static_body_gap_mean_m'] for c in clean_cases])),
                    clean_heading_variation_sum_rad=sum(c['heading_total_variation_rad'] for c in clean_cases),
                    all_case_static_body_gap_min_m=min(c['static_body_gap_min_m'] for c in cases),
                    all_box_body_gap_min_m=min(c['exact_box_body_gap_min_m'] for c in cases if c['exact_box_body_gap_min_m'] is not None))
            combinations.append(result)
    base = next(c for c in combinations if c['preferred_clearance'] == .45 and c['gain'] == 1.5)
    candidates = []
    for c in combinations:
        if not c['all_cases_noncontact'] or 'summary' not in c:
            continue
        s, bs = c['summary'], base['summary']
        c['vs_baseline'] = dict(
            clean_length_change_fraction=s['clean_unique_length_sum_m']/bs['clean_unique_length_sum_m']-1,
            clean_interior_gap_mean_change_m=s['clean_interior_gap_mean_m']-bs['clean_interior_gap_mean_m'],
            clean_heading_variation_change_rad=s['clean_heading_variation_sum_rad']-bs['clean_heading_variation_sum_rad'])
        if (c is not base and c['vs_baseline']['clean_length_change_fraction'] <= .05
                and all(a['length_m'] <= b['length_m']*1.05 for a, b in zip(c['routes'], base['routes']))
                and c['vs_baseline']['clean_interior_gap_mean_change_m'] > .001):
            candidates.append(c)
    candidates.sort(key=lambda c:(-c['vs_baseline']['clean_interior_gap_mean_change_m'],
                                  c['vs_baseline']['clean_length_change_fraction']))
    inputs = [map_file, map_file.with_suffix('.pgm'), marker_file, profile_file,
              PACKAGE/'smart_vision_amr/astar_planner.py', PACKAGE/'smart_vision_amr/local_detour.py',
              PACKAGE/'smart_vision_amr/path_geometry.py', PACKAGE/'smart_vision_amr/clearance_cost.py',
              PACKAGE/'smart_vision_amr/continuous_start_egress.py', PACKAGE/'test/test_planner_failure_memory.py',
              PACKAGE/'test/test_planner_recovery.py', PACKAGE/'test/test_occlusion_reserve_incident.py',
              *[PACKAGE/'test/fixtures'/name for name in fixture_names+[cert_name]], Path(__file__)]
    report = dict(schema_version=1, scope='Offline stronger-center planner screen; 50% one-core CPU duty; no ROS/DDS/Gazebo process or robot motion',
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        input_sha256={str(p.relative_to(REPO)):sha256(p.read_bytes()).hexdigest() for p in inputs},
        physical_constants=dict(robot_radius_m=.22,astar_inflation_cells=5,map_resolution_m=.05,
                                occlusion_extension_m=profile['local_detour_occlusion_extension']),
        markers=markers, requested_routes=routes, unique_clean_legs=legs,
        metric_scope=dict(length='Published polyline arc length; fixed final heading/dwell excluded.',
            heading='Absolute wrapped heading change of nonzero geometry segments; not vehicle rotation.',
            static='Exact continuous segment-to-raw occupied/unknown square distance minus .22m; map exterior also blocked.',
            interior='Mean sampled center-to-static-square distance minus .22m at 5cm arc spacing, omitting first/last ~30cm; not a continuous certificate.',
            box='Exact continuous segment-to-physical 25cm rectangle distance minus .22m.',
            simulated_prerequisites='Fake fresh scan/odom/TF/health/automatic authority. Real prearm, actuator tracking, WiFi, DWB, and sensor noise are NOT tested.',
            recommendation='All clean/global/local/extra certified cases must be noncontact and feasible; both route lengths <=baseline+5%; clean interior mean gap improves >1mm. Selection is screening only.'),
        elapsed_wall_seconds=time.perf_counter()-started, baseline=dict(preferred=.45,gain=1.5),
        recommended=[dict(preferred_clearance=c['preferred_clearance'],gain=c['gain'],vs_baseline=c['vs_baseline']) for c in candidates[:2]],
        closed_loop_status='NOT_TESTED', results=combinations)
    output.mkdir(parents=True,exist_ok=True)
    (output/'planner_screen.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'stronger_center')
    report=screen(parser.parse_args().output)
    print(json.dumps({'elapsed_s':report['elapsed_wall_seconds'],'recommended':report['recommended'],
                      'results':[{k:c.get(k) for k in ('preferred_clearance','gain','all_cases_feasible','all_cases_noncontact','summary','vs_baseline')} for c in report['results']]},indent=2))


if __name__=='__main__':
    main()
