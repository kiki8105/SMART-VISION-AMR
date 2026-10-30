"""Validate the physical classroom goals against the actual packaged map."""
from itertools import combinations
from math import cos, isclose, pi, sin
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image
import yaml

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.map_utils import occupancy_to_grid, world_to_cell
from smart_vision_amr.web_mission_server import load_markers


ROOT = Path(__file__).resolve().parents[1]


def classroom():
    path = ROOT/'maps/classroom_20261027_cleaned.yaml'
    metadata = yaml.safe_load(path.read_text())
    raster = np.array(Image.open(path.parent/metadata['image']))
    probability = (255.0-raster.astype(float))/255.0
    data = np.flipud(np.where(probability > metadata['occupied_thresh'],100,
                             np.where(probability < metadata['free_thresh'],0,-1)))
    ox, oy, yaw = metadata['origin']
    info = SimpleNamespace(width=raster.shape[1], height=raster.shape[0],
                           resolution=metadata['resolution'],
                           origin=SimpleNamespace(position=SimpleNamespace(x=ox,y=oy),
                               orientation=SimpleNamespace(x=0.,y=0.,z=sin(yaw/2),w=cos(yaw/2))))
    message = SimpleNamespace(info=info, data=data.ravel().tolist())
    markers = load_markers(ROOT/'config/aruco_classroom_markers.yaml')
    config = yaml.safe_load((ROOT/'config/classroom_hardware_nodes.yaml').read_text())
    assert np.all(data[np.flipud(raster)==205] == -1), 'Exterior unknown became traversable'
    return message, data, markers, config['/astar_planner']['ros__parameters']


def test_classroom_names_and_supplied_coordinates_are_preserved():
    message, _, markers, _ = classroom()
    assert [(m['id'],m['name']) for m in markers] == [(0,'홈'),(1,'Sector 1'),(2,'Sector 2'),(3,'Sector 3')]
    assert [(m['x'],m['y']) for m in markers[:3]] == [(-.11,-.79),(1.6,.08),(1.71,-3.31)]
    grid = occupancy_to_grid(message)
    assert all(not grid.is_static_obstacle(world_to_cell(message.info,m['x'],m['y'])) for m in markers)
    # Sector 3 belongs to the navigable lower-left room, never the unknown box.
    assert -.55 < markers[3]['x'] < .30
    assert -3.75 < markers[3]['y'] < -2.05


def test_classroom_goal_headings_point_selected_camera_at_wall_markers():
    _, _, markers, _ = classroom()
    # Every marker uses the rear camera, with chassis long axis normal to its wall. Navigation
    # goal yaw always describes the chassis front, not a camera optical frame.
    base_headings = [0.0, pi, pi/2, -pi/2]
    wall_headings = [pi, 0.0, -pi/2, pi/2]
    assert [marker['camera'] for marker in markers] == ['rear', 'rear', 'rear', 'rear']
    for marker, base_yaw, wall_yaw in zip(markers, base_headings, wall_headings):
        assert isclose(sin(marker['yaw'] - base_yaw), 0.0, abs_tol=1e-12)
        assert isclose(cos(marker['yaw'] - base_yaw), 1.0, abs_tol=1e-12)
        camera_yaw = marker['yaw'] + (pi if marker['camera'] == 'rear' else 0.0)
        assert isclose(sin(camera_yaw - wall_yaw), 0.0, abs_tol=1e-12)
        assert isclose(cos(camera_yaw - wall_yaw), 1.0, abs_tol=1e-12)


def test_all_classroom_pairs_have_paths_with_physical_footprint_clearance():
    message, data, markers, settings = classroom()
    grid = occupancy_to_grid(message, settings['occupied_threshold'], settings['obstacle_inflation_cells'])
    cells = [world_to_cell(message.info,m['x'],m['y']) for m in markers]
    blocked = np.argwhere(data != 0)[:,::-1]
    minimum = float('inf')
    for start, goal in combinations(cells, 2):
        path = AStarPlanner().plan(grid,start,goal)
        assert path, (start,goal)
        assert path[0] == start and path[-1] == goal
        for cell in path:
            delta = np.maximum(np.abs(blocked - np.asarray(cell)) - .5, 0.)
            clearance = np.hypot(delta[:,0],delta[:,1]).min() * message.info.resolution
            minimum = min(minimum,clearance)
    assert minimum >= settings['robot_radius'] + settings['start_egress_clearance_margin']
