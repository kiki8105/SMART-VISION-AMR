#!/usr/bin/env python3
"""Replay recorded promotion geometry; never initialize ROS or send commands."""
import argparse
import hashlib
import json
from math import ceil, hypot, sqrt
from pathlib import Path
from types import SimpleNamespace
import sys

from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.astar_planner import AStarPlannerNode
from smart_vision_amr.clearance_cost import clearance_penalties
from smart_vision_amr.map_utils import cell_to_world, inflate_cells, occupancy_to_grid


class Context:
    """Only production geometry helpers; no Node constructor/executor exists."""
    _global_occlusion_reserve = AStarPlannerNode._global_occlusion_reserve
    _local_occlusion_reserve = AStarPlannerNode._local_occlusion_reserve
    _live_obstacle_blocked = AStarPlannerNode._live_obstacle_blocked
    _soft_occlusion_penalty = AStarPlannerNode._soft_occlusion_penalty

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def _local_route_key(self):
        return 'recorded-same-goal-map-reference'


def load_fixture(path=None):
    path = path or Path(__file__).with_name('fixture.json')
    data = json.loads(path.read_text())
    model = data['map']
    image = ROOT/'ros2_ws/src/smart_vision_amr/maps'/Path(model['yaml']).with_suffix('.pgm')
    assert hashlib.sha256(image.read_bytes()).hexdigest() == model['image_sha256']
    pixels = list(Image.open(image).getdata())
    width, height = model['width'], model['height']
    values = [100 if pixels[(height-1-y)*width+x] == 0 else
              -1 if pixels[(height-1-y)*width+x] == 205 else 0
              for y in range(height) for x in range(width)]
    assert set(pixels) == {0, 205, 254}, 'Recorded map trinary encoding changed'
    origin = SimpleNamespace(position=SimpleNamespace(x=model['origin'][0], y=model['origin'][1]),
                             orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    info = SimpleNamespace(width=width, height=height, resolution=model['resolution'], origin=origin)
    message = SimpleNamespace(info=info, data=values)
    node = Context()
    node.params = data['parameters']
    node.map = message, occupancy_to_grid(message, 65, 5)
    node.raw_static_grid = occupancy_to_grid(message, 65, 0)
    node.observed = {tuple(cell) for cell in data['observed_cells']}
    node.remembered = {tuple(cell) for cell in data['remembered_cells']}
    node.local_reference_points = [tuple(point) for point in data['reference_world']]
    node.local_reference_key = node._local_route_key()
    return node, data


def route_clearance(node, cells, box, spacing=.002):
    """Sample full segments and subtract half the spacing (Lipschitz bound)."""
    info = node.map[0].info
    radius = node.params['robot_radius']
    world = [cell_to_world(info, cell) for cell in cells]
    box_min = raw_min = float('inf')
    measured = node.observed|node.remembered
    reach = ceil((radius+.3)/info.resolution)+1
    for first, last in zip(world, world[1:]):
        count = max(1, ceil(hypot(last[0]-first[0], last[1]-first[1])/spacing))
        for step in range(count+1):
            x = first[0]+(last[0]-first[0])*step/count
            y = first[1]+(last[1]-first[1])*step/count
            box_min = min(box_min, hypot(max(box[0]-x, 0., x-box[1]),
                                         max(box[2]-y, 0., y-box[3]))-radius)
            gx = (x-info.origin.position.x)/info.resolution
            gy = (y-info.origin.position.y)/info.resolution
            for oy in range(max(0, int(gy)-reach), min(info.height, int(gy)+reach+1)):
                for ox in range(max(0, int(gx)-reach), min(info.width, int(gx)+reach+1)):
                    if not node.raw_static_grid.is_static_obstacle((ox, oy)) and (ox, oy) not in measured:
                        continue
                    distance = hypot(max(ox-gx, 0., gx-ox-1.), max(oy-gy, 0., gy-oy-1.))*info.resolution
                    raw_min = min(raw_min, distance-radius)
    return {'actual_box_gap_lower_bound_m': box_min-spacing/2,
            'raw_static_unknown_observed_memory_gap_lower_bound_m': raw_min-spacing/2,
            'sampling_spacing_m': spacing, 'lipschitz_reserve_m': spacing/2}


def compare(node, data, *, measure_clearance=True):
    start, goal = tuple(data['start_cell']), tuple(data['goal_cell'])
    info, grid = node.map[0].info, node.map[1]
    raw = node.raw_static_grid
    reserved_live = node._global_occlusion_reserve(start, node.observed)
    reserved_memory = node._global_occlusion_reserve(start, node.remembered, remembered=True)
    observed_blocked = node._live_obstacle_blocked(reserved_live)
    costs = clearance_penalties(raw, reserved_live|reserved_memory, info.resolution,
                                node.params['preferred_clearance'], node.params['clearance_penalty_gain'])
    report = {}
    for name in ('old_square_memory', 'consistent_footprint_memory'):
        memory = (inflate_cells(reserved_memory, info.width, info.height, 5)
                  if name == 'old_square_memory' else node._live_obstacle_blocked(reserved_memory))
        measured_memory = (inflate_cells(node.remembered, info.width, info.height, 5)
                           if name == 'old_square_memory' else node._live_obstacle_blocked(node.remembered))
        blocked = observed_blocked|memory
        measured = node._live_obstacle_blocked(node.observed)|measured_memory
        escape = AStarPlannerNode._dynamic_escape_halo(start, blocked, node.observed,
            node.remembered, grid, raw, 5, node.params['robot_radius']/info.resolution,
            node.params['start_egress_clearance_margin']/info.resolution+sqrt(2.)/2)
        primary = AStarPlanner().plan(grid, start, goal, blocked, allow_blocked_start=bool(escape),
                                     escape_cells=escape, cell_penalty=costs)
        # This fixture has no measured start certificate. Keep the original
        # hard memory-start policy and permit fallback only for an ordinary
        # physically unblocked start; never clear measured/static cells.
        fallback_permitted = (not grid.is_static_obstacle(start) and start not in measured
                              and start not in measured_memory and bool(blocked-measured))
        fallback = (AStarPlanner().plan(grid, start, goal, measured,
            cell_penalty=node._soft_occlusion_penalty(costs, blocked, measured))
            if not primary and fallback_permitted else [])
        route = primary or fallback
        assert all(not grid.is_static_obstacle(cell) and cell not in measured for cell in route)
        report[name] = {'remembered_mask_cells': len(memory), 'observed_mask_cells': len(observed_blocked),
            'virtual_cells': len(blocked-measured), 'escape_cells': len(escape),
            'start_in_remembered_mask': start in measured_memory,
            'primary_route_cells': len(primary), 'fallback_permitted': fallback_permitted,
            'fallback_route_cells': len(fallback), 'route_cells': [list(cell) for cell in route],
            'clearance': route_clearance(node, route, data['actual_box_bounds'])
                         if route and measure_clearance else None}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    node, data = load_fixture()
    report = {'scope': 'Offline recorded geometry only; not a closed-loop pass',
              'replay_inputs_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in (Path(__file__), Path(__file__).with_name('fixture.json'),
                    ROOT/'ros2_ws/src/smart_vision_amr/smart_vision_amr/astar_planner.py',
                    ROOT/'ros2_ws/src/smart_vision_amr/smart_vision_amr/map_utils.py')},
              'original_trial_result': data['original_trial_result'],
              'comparison': compare(node, data)}
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:{key:value for key,value in v.items() if key!='route_cells'}
                      for k,v in report['comparison'].items()}, indent=2))


if __name__ == '__main__':
    main()
