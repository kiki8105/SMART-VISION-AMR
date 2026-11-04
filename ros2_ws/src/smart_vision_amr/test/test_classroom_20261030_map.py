"""The October classroom remap stays in its own map frame and inside the walls."""

from hashlib import sha256
from itertools import combinations
from math import atan2, cos, hypot, pi, sin, sqrt
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
import yaml

from smart_vision_amr.web_mission_server import load_saved_map
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.map_utils import (
    inflate_cells_circular, occupancy_to_grid, world_to_cell)


PACKAGE = Path(__file__).resolve().parents[1]
MAPS = PACKAGE / 'maps'
MARKERS = PACKAGE / 'config' / 'aruco_classroom_markers_20261030.yaml'


class Classroom20261030MapTests(unittest.TestCase):
    def setUp(self):
        self.raw = np.asarray(Image.open(MAPS / 'classroom_20261030_raw.pgm'))
        self.cleaned = np.asarray(Image.open(MAPS / 'classroom_20261030_cleaned.pgm'))

    def test_raw_archive_and_cleaned_metadata_share_native_frame(self):
        raw_file = MAPS / 'classroom_20261030_raw.pgm'
        self.assertEqual(
            sha256(raw_file.read_bytes()).hexdigest(),
            '6911dbc08ae4e2e9292d2aa19064380d1f8c5733781c16778d89cdb7165e7782',
        )
        self.assertEqual(self.raw.shape, (140, 86))
        self.assertEqual(self.cleaned.shape, self.raw.shape)
        for name in ('raw', 'cleaned'):
            metadata, _ = load_saved_map(MAPS / f'classroom_20261030_{name}.yaml')
            self.assertEqual((metadata['width'], metadata['height']), (86, 140))
            self.assertEqual(metadata['resolution'], 0.05)
            self.assertEqual(metadata['origin'], {'x': -1.65, 'y': -5.25, 'yaw': 0.0})
        self.assertTrue((MAPS / 'classroom_20261027_cleaned.pgm').exists())

    def test_cleaning_only_masks_exterior_and_preserves_map_orientation(self):
        changed = self.cleaned != self.raw
        self.assertGreater(np.count_nonzero(changed), 600)
        self.assertTrue(np.all(self.cleaned[changed] == 205))
        self.assertEqual(self.raw[110, 50], 254)  # SLAM leak below bottom wall.
        self.assertEqual(self.cleaned[110, 50], 205)
        self.assertTrue(np.all(self.cleaned[100:, :] == 205))
        self.assertEqual(self.cleaned[31, 50], 0)  # New central obstacle's upper edge.
        self.assertTrue(np.all(
            self.cleaned[:100, 17:75][self.raw[:100, 17:75] == 0] == 0,
        ))
        self.assertEqual(self.cleaned[90, 54], 0)  # Right-hand partition wall.
        self.assertEqual(self.cleaned[90, 31], 254)  # Free room on its left.
        count, _ = cv2.connectedComponents((self.cleaned == 254).astype(np.uint8), 8)
        self.assertEqual(count, 2, 'No disconnected exterior free-space island may remain')

        _, raw_grid = load_saved_map(MAPS / 'classroom_20261030_raw.yaml')
        _, cleaned_grid = load_saved_map(MAPS / 'classroom_20261030_cleaned.yaml')
        self.assertEqual(raw_grid[140 - 1 - 110, 50], 0)
        self.assertEqual(cleaned_grid[140 - 1 - 110, 50], -1)

    def test_home_and_sectors_are_free_with_robot_clearance(self):
        profile = yaml.safe_load(MARKERS.read_text(encoding='utf-8'))
        self.assertEqual({item['id'] for item in profile['markers']}, {0, 1, 2, 3})
        clearance = cv2.distanceTransform(
            (self.cleaned == 254).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE,
        ) * 0.05
        for marker in profile['markers']:
            x, y = float(marker['x']), float(marker['y'])
            column = round((x + 1.65) / 0.05)
            row = round(139 - (y + 5.25) / 0.05)
            with self.subTest(marker=marker['id']):
                self.assertEqual(self.cleaned[row, column], 254)
                self.assertGreaterEqual(clearance[row, column], 0.25)

    def test_all_sector_routes_clear_physical_and_gazebo_astar_inflation(self):
        metadata, data = load_saved_map(MAPS / 'classroom_20261030_cleaned.yaml')
        settings = yaml.safe_load((PACKAGE / 'config/classroom_hardware_nodes.yaml').read_text())[
            '/astar_planner']['ros__parameters']
        # The measured physical envelope needs four 5 cm cells. The stock
        # Gazebo Waffle Pi collision box reaches 23.7 cm, so its generated
        # simulation profile retains five cells on this same map.
        self.assertEqual(settings['robot_radius'], 0.17)
        self.assertEqual(settings['obstacle_inflation_cells'], 4)
        gazebo_inflation_cells = 5
        origin = metadata['origin']
        info = SimpleNamespace(
            width=metadata['width'], height=metadata['height'],
            resolution=metadata['resolution'],
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin['x'], y=origin['y']),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0,
                    z=sin(origin['yaw'] / 2), w=cos(origin['yaw'] / 2))))
        message = SimpleNamespace(info=info, data=data.ravel().tolist())
        markers = yaml.safe_load(MARKERS.read_text(encoding='utf-8'))['markers']
        cells = {marker['id']: world_to_cell(info, marker['x'], marker['y'])
                 for marker in markers}
        for profile, inflation_cells in (
                ('physical', settings['obstacle_inflation_cells']),
                ('stock_gazebo', gazebo_inflation_cells)):
            grid = occupancy_to_grid(
                message, settings['occupied_threshold'], inflation_cells)
            for marker_id, cell in cells.items():
                with self.subTest(profile=profile, goal=marker_id):
                    self.assertFalse(grid.is_static_obstacle(cell))
            for start_id, goal_id in combinations(cells, 2):
                with self.subTest(profile=profile, start=start_id, goal=goal_id):
                    path = AStarPlanner().plan(grid, cells[start_id], cells[goal_id])
                    self.assertTrue(path)
                    self.assertEqual(path[0], cells[start_id])
                    self.assertEqual(path[-1], cells[goal_id])
                    self.assertTrue(all(not grid.is_static_obstacle(cell) for cell in path))

    def test_sector2_goal_survives_captured_wall_edge_returns_and_sees_marker(self):
        """Keep the stop inside the turning area without hiding live scan hits."""
        markers = {item['id']: item for item in
                   yaml.safe_load(MARKERS.read_text(encoding='utf-8'))['markers']}
        metadata, data = load_saved_map(MAPS / 'classroom_20261030_cleaned.yaml')
        origin = metadata['origin']
        info = SimpleNamespace(
            width=metadata['width'], height=metadata['height'],
            resolution=metadata['resolution'],
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin['x'], y=origin['y']),
                orientation=SimpleNamespace(x=0., y=0., z=sin(origin['yaw']/2),
                                            w=cos(origin['yaw']/2))))
        message = SimpleNamespace(info=info, data=data.ravel().tolist())
        grid = occupancy_to_grid(message, 65, 5)
        start = world_to_cell(info, markers[1]['x'], markers[1]['y'])
        stop = markers[2]
        goal = world_to_cell(info, stop['x'], stop['y'])

        # In the baseline Gazebo trial the saved right wall is x=72, but scan
        # endpoints clustered at x=70 while the robot rotated near Sector 2.
        # Retain these live returns and their full circular collision margin.
        wall_edge_returns = {(70, y) for y in range(42, 56)}
        footprint_cells = ((0.22 + 0.02)/info.resolution + sqrt(2.)/2.)
        observed_blocked = inflate_cells_circular(
            wall_edge_returns, info.width, info.height, footprint_cells)
        old_goal = world_to_cell(info, 1.561552, -2.789613)
        self.assertIn(old_goal, observed_blocked)
        self.assertNotIn(goal, observed_blocked)
        self.assertFalse(grid.is_static_obstacle(goal))
        self.assertTrue(AStarPlanner().plan(grid, start, goal, observed_blocked))

        # The ArUco board stays at the physical wall; only the robot stop moves.
        world = ET.parse(PACKAGE / 'worlds/classroom_20261030_replay.world')
        marker = world.find("world/model[@name='classroom_marker_2']")
        self.assertIsNotNone(marker)
        marker_x, marker_y, *_ = map(float, marker.findtext('pose').split())
        self.assertAlmostEqual(marker_x, 1.575770, delta=.01)
        self.assertAlmostEqual(marker_y, -3.197011, delta=.01)
        dx, dy = marker_x-stop['x'], marker_y-stop['y']
        rear_yaw = stop['yaw'] + pi
        error = atan2(sin(atan2(dy, dx)-rear_yaw),
                      cos(atan2(dy, dx)-rear_yaw))
        self.assertLess(abs(error), .35)  # comfortably inside 1.086 rad FOV
        self.assertLess(hypot(dx, dy), .8)


if __name__ == '__main__':
    unittest.main()
