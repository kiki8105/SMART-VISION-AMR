#!/usr/bin/env python3
"""Replay the frozen necessary reverse gates; no ROS, simulator, or motors."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells, inflate_cells_footprint, world_to_cell
from types import SimpleNamespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, default=Path(__file__).with_name('fixture.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite audit output')
    source = Path(__file__).with_name('reverse_recovery_before_neutral.py')
    spec = importlib.util.spec_from_file_location('frozen_reverse_selector', source)
    selector = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = selector
    spec.loader.exec_module(selector)
    data = json.loads(args.fixture.read_text())
    image = ROOT/data['map']['image']
    assert hashlib.sha256(image.read_bytes()).hexdigest() == data['map']['image_sha256']
    original_selector = next(value for key, value in data['source_sha256'].items()
                             if key.endswith('/reverse_recovery.py'))
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original_selector
    with image.open('rb') as stream:
        assert stream.readline() == b'P5\n'
        width, height = map(int, stream.readline().split())
        assert stream.readline() == b'255\n'
        pixels = stream.read()
    assert len(pixels) == width*height and set(pixels) <= {0, 205, 254}
    raw = Grid(tuple(tuple(pixels[(height-y-1)*width+x] != 254
                           for x in range(width)) for y in range(height)))
    occupied = {(x, y) for y, row in enumerate(raw.rows) for x, value in enumerate(row) if value}
    params = data['parameters']
    observed, remembered, reserved_observed, reserved_remembered = (
        {tuple(cell) for cell in data[key]} for key in (
            'observed_cells', 'remembered_cells', 'reserved_observed_cells', 'reserved_remembered_cells'))
    static = inflate_cells(occupied, width, height, params['astar_inflation_cells'])
    grid = Grid(tuple(tuple((x, y) in static for x in range(width)) for y in range(height)))
    radius_cells = params['robot_radius']/data['map']['resolution']
    live_mask = inflate_cells_footprint(reserved_observed, width, height, radius_cells)
    memory_mask = inflate_cells(reserved_remembered, width, height, params['astar_inflation_cells'])
    full = live_mask | memory_mask
    measured = (inflate_cells_footprint(observed, width, height, radius_cells)
                | inflate_cells(remembered, width, height, params['astar_inflation_cells']))
    assert len(live_mask) == data['inflated_counts_from_planner_log']['observed']
    assert len(memory_mask) == data['inflated_counts_from_planner_log']['remembered']
    raw_all = occupied | observed | remembered
    raw_with = Grid(tuple(tuple((x, y) in raw_all for x in range(width)) for y in range(height)))
    inflated_all = static | full
    inflated = Grid(tuple(tuple((x, y) in inflated_all for x in range(width)) for y in range(height)))
    resolution = data['map']['resolution']
    origin = data['map']['origin']
    geometry = selector.GridGeometry(resolution, *origin)
    info = SimpleNamespace(resolution=resolution, origin=SimpleNamespace(
        position=SimpleNamespace(x=origin[0], y=origin[1]),
        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    pose = data['pose']
    backward = -math.cos(pose[2]), -math.sin(pose[2])
    clearance = lambda point: selector._raw_clearance(raw_with, geometry, tuple(raw_all), *point)
    point_at = lambda distance: (pose[0]+distance*backward[0], pose[1]+distance*backward[1])
    sampled = [(distance/1000., clearance(point_at(distance/1000.))) for distance in range(301)]
    start_clearance = sampled[0][1]
    minimum = min(value for _, value in sampled)
    # Distance to any closed obstacle set is 1-Lipschitz. At 1mm spacing the
    # unsampled minimum is no more than 0.5mm below the smallest sample.
    lower_bound = minimum-.0005
    required = params['robot_radius']+params['clearance_margin']+params['sample_step']/2.
    endpoints = []
    for distance in (.10, .15, .20, .25, .30):
        point = point_at(distance)
        cell = world_to_cell(info, *point)
        endpoints.append({'distance_m': distance, 'point': point, 'cell': cell,
            'raw_clearance_m': clearance(point),
            'outside_full_static_dynamic_inflation': not inflated.is_static_obstacle(cell),
            'retained_goal_reachable_full': bool(AStarPlanner().plan(grid, cell, tuple(data['goal_cell']), full)),
            'retained_goal_reachable_measured': bool(AStarPlanner().plan(grid, cell, tuple(data['goal_cell']), measured))})
    history = [selector.Breadcrumb(100.-distance/.1, *point_at(distance), pose[2])
               for distance in (.30, .275, .25, .225, .20, .175, .15, .125, .10, .075, .05, .025, 0.)]
    probes = []
    for allow_parallel in (False, True):
        calls = []
        def can_reach(cell):
            calls.append(cell)
            return bool(AStarPlanner().plan(grid, cell, tuple(data['goal_cell']), full))
        candidate = selector.select_reverse_egress(
            history, raw_with, inflated, geometry, can_reach, now=100.,
            robot_radius=params['robot_radius'], clearance_margin=params['clearance_margin'],
            min_distance=params['min_distance'], max_distance=params['max_distance'],
            max_history_age=90., max_sample_gap=30., blocking_cells=set(),
            current_pose=tuple(pose), allow_parallel_static_egress=allow_parallel)
        probes.append({'allow_parallel_static_egress': allow_parallel,
                       'candidate_found': candidate is not None, 'goal_callback_count': len(calls)})
    result = {'scope': data['scope'], 'fixture_sha256': hashlib.sha256(args.fixture.read_bytes()).hexdigest(),
        'raw_start_clearance_m': start_clearance,
        'required_raw_clearance_m': required,
        'rear_0_to_30cm_sampled_minimum_clearance_m': minimum,
        'rear_0_to_30cm_certified_lower_bound_m': lower_bound,
        'raw_sweep_clearance_passed': lower_bound >= required,
        'maximum_initial_clearance_decrease_m': start_clearance-minimum,
        'endpoints': endpoints, 'synthetic_history_probes': probes,
        'interpretation': [
            'Perfectly straight 13-point history is assumed; actual breadcrumbs were not recorded.',
            'The unchanged selector rejects before any goal callback, even with its current parallel-static flag enabled.',
            '15-30cm endpoints exit full inflation, but the full virtual-reserve goal tail has no route; measured-only tail has a route.',
            'Neither this counterfactual nor a rear-sector minimum proves an actual executable/verified reverse maneuver.'],
        'limitations': data['limitations']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
