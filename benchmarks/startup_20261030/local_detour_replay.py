#!/usr/bin/env python3
"""Replay recorded/reconstructed local-planner inputs without ROS or actuation.

Run from any directory: python3 local_detour_replay.py --replays 20 --contended
The optional competing process is pinned to the same logical CPU and is always
terminated. This models CPU competition, not a complete Gazebo execution.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import multiprocessing
import os
from pathlib import Path
import statistics
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.clearance_cost import clearance_penalties
from smart_vision_amr.grid import Grid
from smart_vision_amr.local_detour import plan_local_detour
from smart_vision_amr.map_utils import inflate_cells


def compete():
    until = time.monotonic()+10.
    while time.monotonic() < until:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=HERE/'local_detour_replay_input.json')
    parser.add_argument('--implementation', type=Path, help='Optional comparison module source')
    parser.add_argument('--replays', type=int, default=20)
    parser.add_argument('--budget', type=float, default=.06)
    parser.add_argument('--contended', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.replays <= 100 or not 0 < args.budget <= .1:
        parser.error('Use 1..100 replays and a positive budget at most0.1s')
    plan = plan_local_detour
    module_path = HERE.parents[1]/'ros2_ws/src/smart_vision_amr/smart_vision_amr/local_detour.py'
    if args.implementation:
        name = 'smart_vision_amr._replay_implementation'
        spec = importlib.util.spec_from_file_location(name, args.implementation)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        plan, module_path = module.plan_local_detour, args.implementation
    data = json.loads(args.input.read_text())
    grid = Grid.from_ascii('\n'.join(data['static_grid_rows']))
    raw = Grid.from_ascii('\n'.join(data['raw_grid_rows']))
    planning_hits = set(map(tuple, data['planning_hits']))
    blocked = inflate_cells(planning_hits, grid.width, grid.height,
                            data['footprint_inflation_cells'])
    # Adapter preprocessing is intentionally outside the pure search deadline.
    penalty = clearance_penalties(raw, planning_hits, data['resolution'],
        data['preferred_clearance'], data['clearance_penalty_gain'])
    options = {key: data[key] for key in ('resolution', 'max_rejoin_distance',
        'max_detour_length', 'max_lateral_distance', 'max_expansions',
        'max_candidates', 'rejoin_buffer')}
    contender = None
    if args.contended:
        os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
        contender = multiprocessing.Process(target=compete)
        contender.start()
    wall_ms, cpu_ms, results = [], [], []
    try:
        for _ in range(args.replays):
            before = time.process_time()
            result = plan(grid, tuple(data['provenance']['start_cell']),
                data['reference_points'], blocked, cell_penalty=penalty,
                max_seconds=args.budget, **options)
            cpu_ms.append((time.process_time()-before)*1000.)
            wall_ms.append(result.search_duration_ms)
            results.append(result)
    finally:
        if contender:
            contender.terminate()
            contender.join()
    output = {'budget_ms': args.budget*1000., 'replays': args.replays,
        'contended': args.contended, 'cpu_affinity': sorted(os.sched_getaffinity(0)),
        'median_ms': statistics.median(wall_ms),
        'p95_ms': sorted(wall_ms)[math.ceil(.95*len(wall_ms))-1],
        'max_ms': max(wall_ms), 'median_cpu_ms': statistics.median(cpu_ms),
        'expansions': sorted({result.expansions for result in results}),
        'reasons': {reason: sum(result.reason == reason for result in results)
                    for reason in sorted({result.reason for result in results})},
        'successful_path_hashes': sorted({hashlib.sha256(json.dumps(result.points).encode()).hexdigest()
                                        for result in results if result.points}),
        'input_sha256': hashlib.sha256(args.input.read_bytes()).hexdigest(),
        'implementation_sha256': hashlib.sha256(module_path.read_bytes()).hexdigest()}
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
