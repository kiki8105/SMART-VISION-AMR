#!/usr/bin/env python3
"""Replay the two recorded raster frames; no ROS nodes, DDS or robot commands."""
from hashlib import sha256
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'ros2_ws/src/smart_vision_amr'), str(ROOT/'ros2_ws/src/smart_vision_amr/test')]
from body_geometry_oracle import continuous_raw_body_gap, full_cell_body_mask
from test_occlusion_reserve_incident import load_incident
from smart_vision_amr.continuous_start_egress import certified_start_only_edges
from smart_vision_amr.map_utils import world_to_cell
from smart_vision_amr.path_geometry import route_cells


def verify():
    folder = Path(__file__).parent
    fixture = folder/'fixture.json'
    node, data = load_incident(fixture)
    info, grid = node.map[0].info, node.map[1]
    start, destination = (31,96), (30,96)
    point = tuple((data['pose_world'][axis]-getattr(info.origin.position, 'xy'[axis]))/.05 for axis in (0,1))
    path = data['recorded_emitted_world']
    cells = route_cells([((x+1.65)/.05,(y+5.25)/.05) for x,y in path])
    report = {'scope':data['scope'], 'fixture_sha256':sha256(fixture.read_bytes()).hexdigest(),
              'limits':data['limits'], 'frames':{}}
    for name in ('observed_cells','adjacent_observed_cells'):
        observed = {tuple(cell) for cell in data[name]}
        memory = {tuple(cell) for cell in data['remembered_cells']}
        measured = full_cell_body_mask(observed|memory, info, .22)
        allowed = certified_start_only_edges(node.raw_static_grid, grid, observed|memory,
            measured|{start}, start, point, .22/.05, 0.)
        witness = SimpleNamespace(map=node.map, raw_static_grid=node.raw_static_grid,
            observed=observed, remembered=memory, params={'robot_radius':.22})
        gap = continuous_raw_body_gap(witness, [data['pose_world'][:2], *path[:2]])
        assert (start,destination) in allowed and gap>0
        prefix_cells = {world_to_cell(info, *point) for point in path[:2]}
        assert not (prefix_cells-{start}) & measured
        assert all(not grid.is_static_obstacle(cell) for cell in prefix_cells)
        report['frames'][name] = {'observed_count':len(observed), 'memory_count':len(memory),
            'has_36_94':(36,94) in observed, 'measured_start_blocked':start in measured,
            'measured_route_intersections':sorted(cells&measured),
            'allowed_edges':sorted(allowed), 'raw_first_prefix_gap_m':gap,
            'whole_recorded_route_raw_gap_m':continuous_raw_body_gap(witness,path),
            'whole_recorded_route_may_need_a_new_suffix':bool((cells-{start})&measured)}
    return report


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
