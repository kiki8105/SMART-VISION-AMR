#!/usr/bin/env python3
"""Replay one recorded clean Home failure using frozen pure planning helpers."""
import argparse
import hashlib
import json
import pathlib
import sys
from types import SimpleNamespace


def read_p5(path):
    raw = path.read_bytes()
    position = 0

    def token():
        nonlocal position
        while True:
            while raw[position] in b' \t\r\n':
                position += 1
            if raw[position] != ord('#'):
                break
            position = raw.index(b'\n', position) + 1
        start = position
        while raw[position] not in b' \t\r\n':
            position += 1
        return raw[start:position]

    assert token() == b'P5'
    width, height, maximum = int(token()), int(token()), int(token())
    assert maximum == 255
    position += 2 if raw[position:position+2] == b'\r\n' else 1
    pixels = raw[position:]
    assert len(pixels) == width * height
    return width, height, pixels


def replay(folder):
    fixture = json.loads((folder / 'fixture.json').read_text())
    image = folder / fixture['map']['image']
    assert hashlib.sha256(image.read_bytes()).hexdigest() == fixture['map']['image_sha256']
    package = folder / 'snapshot' / 'smart_vision_amr'
    for name, expected in fixture['source_provenance']['snapshot_module_sha256'].items():
        assert hashlib.sha256((package / name).read_bytes()).hexdigest() == expected, name
    sys.path.insert(0, str(folder / 'snapshot'))
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.failure_memory import FailureMemory
    from smart_vision_amr.map_utils import occupancy_to_grid
    from smart_vision_amr.reverse_recovery import GridGeometry

    width, height, pixels = read_p5(image)
    assert (width, height) == (fixture['map']['width'], fixture['map']['height'])
    conversion = {int(k): value for k, value in fixture['map']['pixel_mapping'].items()}
    # PGM image starts at the upper left; ROS grid rows start at the lower left.
    occupancy = [conversion[value]
                 for y in range(height - 1, -1, -1)
                 for value in pixels[y*width:(y+1)*width]]
    message = SimpleNamespace(info=SimpleNamespace(width=width, height=height), data=occupancy)
    grid = occupancy_to_grid(message, fixture['map']['occupied_threshold'],
                             fixture['map']['static_inflation_cells'])
    entry = fixture['failure_entry']
    memory = FailureMemory(**fixture['failure_settings'])
    memory.record_failure(
        x=entry['x'], y=entry['y'], approach_heading=entry['approach_heading'],
        reason=entry['reason'], now=entry['first_seen'],
        map_scope=entry['map_scope'], goal_scope=entry['goal_scope'])
    origin = fixture['map']['origin']
    geometry = GridGeometry(fixture['map']['resolution'], *origin)
    edges, costs = memory.constraints(
        grid, geometry, now=entry['last_seen'] + 0.5,
        map_scope=entry['map_scope'], goal_scope=entry['goal_scope'])
    start, goal = tuple(fixture['start']), tuple(fixture['goal'])
    planner = AStarPlanner()
    hard = planner.plan(grid, start, goal, set(), blocked_edges=edges, edge_penalty=costs)
    soft_costs = dict(costs)
    for edge in edges:
        soft_costs[edge] = max(soft_costs.get(edge, 0.0), fixture['failure_settings']['penalty_cost'])
    soft = planner.plan(grid, start, goal, set(), edge_penalty=soft_costs)
    ordinary = planner.plan(grid, start, goal, set())
    actual = {'hard_blocked_edges': len(edges), 'soft_cost_edges': len(costs),
              'hard_route_cells': len(hard), 'soft_route_cells': len(soft),
              'no_failure_memory_route_cells': len(ordinary)}
    assert actual == fixture['expected_oracle'], (actual, fixture['expected_oracle'])
    assert fixture['observed_snapshot_nearest_failed_global']['count'] == 0
    assert fixture['remembered_snapshot_nearest_failed_global']['count'] == 0
    for route in (soft, ordinary):
        assert route[0] == start and route[-1] == goal
        assert all(not grid.is_static_obstacle(cell) for cell in route)
    canonical = lambda value: hashlib.sha256(json.dumps(value, sort_keys=True,
                                                        separators=(',', ':')).encode()).hexdigest()
    return {
        'scope': 'Offline unchanged-map directional-memory oracle, not motion permission or full local-controller replay.',
        'outcome_of_interrupted_actual_trial': fixture['outcome'],
        'observed_raw': 0, 'remembered_raw': 0,
        'static_inflation_cells': fixture['map']['static_inflation_cells'],
        'unknown_cells_preserved': occupancy.count(-1),
        'unchanged_geometry': True,
        'source_snapshot_hashes_verified': True,
        'reproduced': actual,
        'controller_penalty_only_cost_edges': len(soft_costs),
        'hard_edges_sha256': canonical(sorted(edges)),
        'raw_soft_costs_sha256': canonical(sorted(costs.items())),
        'hard_route': hard,
        'controller_penalty_only_route': soft,
        'no_failure_memory_route': ordinary,
        'conclusion': 'The exact276 directional hard edges reproduce NoPath. Converting only those execution restrictions to bounded costs permits33 cells; ordinary unchanged static/unknown geometry permits27 cells. No occupancy/body/collision margin was removed.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=pathlib.Path, default=pathlib.Path(__file__).resolve().parent)
    parser.add_argument('--output', type=pathlib.Path, help='Write results to a new file')
    args = parser.parse_args()
    rendered = json.dumps(replay(args.folder), ensure_ascii=False, indent=2) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(rendered)
    else:
        print(rendered, end='')


if __name__ == '__main__':
    main()
