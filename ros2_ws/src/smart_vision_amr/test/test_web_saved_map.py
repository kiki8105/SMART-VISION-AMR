"""Saved map previews are visible but cannot replace the live navigation map."""
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from PIL import Image
import yaml

from smart_vision_amr.mission_logic import MissionError
from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, build_parser, load_saved_map,
    cell_boundary_paths, map_layout,
)


class SavedMapTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.path = self.directory/'map.yaml'
        self.document = {'image': 'map.pgm', 'mode': 'trinary', 'resolution': .05,
                         'origin': [-1.53, -5.69, .2], 'negate': 0,
                         'occupied_thresh': .65, 'free_thresh': .196}
        Image.fromarray(np.array([[0, 205, 254], [255, 64, 127]], np.uint8)).save(self.directory/'map.pgm')
        self.write_yaml()
        self.app = MissionApplication(DEFAULT_MARKERS, 16, False, self.directory/'settings.json')

    def tearDown(self):
        self.temporary.cleanup()

    def write_yaml(self):
        self.path.write_text(yaml.safe_dump(self.document), encoding='utf-8')

    def other_conditions_ready(self):
        self.app.connected = self.app.goal_available = self.app.planner_available = self.app.cancel_available = True
        self.app.aruco_available = True
        self.app.pose = {'x': -.11, 'y': -.79, 'yaw': 0.}
        self.app.pose_at = self.app.scan_at = self.app.odom_at = self.app.camera_at = time.monotonic()

    def test_old_tf_cache_is_not_displayed_as_the_current_vehicle_position(self):
        self.other_conditions_ready()
        self.app.pose_at = time.monotonic()-3.
        state = self.app.state()
        self.assertIsNone(state['robot']['pose'])
        self.assertGreaterEqual(state['robot']['pose_age'], 3.)

    def test_trinary_values_unknown_and_bottom_left_origin(self):
        metadata, data = load_saved_map(self.path)
        np.testing.assert_array_equal(data, [[0, 100, -1], [100, -1, 0]])
        self.assertEqual(metadata, {'width': 3, 'height': 2, 'resolution': .05,
                                   'origin': {'x': -1.53, 'y': -5.69, 'yaw': .2}, 'frame_id': 'map'})

    def test_negate_and_exact_thresholds_follow_map_server_trinary(self):
        Image.fromarray(np.array([[0, 85, 170, 255]], np.uint8)).save(self.directory/'map.pgm')
        self.document.update(free_thresh=1/3, occupied_thresh=2/3)
        self.write_yaml()
        _, data = load_saved_map(self.path)
        np.testing.assert_array_equal(data, [[100, -1, -1, 0]])
        self.document['negate'] = 1
        self.write_yaml()
        _, data = load_saved_map(self.path)
        np.testing.assert_array_equal(data, [[0, -1, -1, 100]])

    def test_thresholds_are_not_silently_overridden_for_gray_205(self):
        # A .25 free threshold makes gray205 free in Nav2; configurations that
        # intend gray205 to stay unknown must use a lower threshold, e.g. .196.
        self.document['free_thresh'] = .25
        self.write_yaml()
        _, data = load_saved_map(self.path)
        self.assertEqual(data[1, 1], 0)

    def test_preview_exposes_actual_raster_without_marking_navigation_ready(self):
        self.app.load_map_preview(self.path)
        self.other_conditions_ready()
        state = self.app.state()
        self.assertTrue(state['map']['preview'])
        self.assertEqual(state['map']['source'], 'saved_preview')
        self.assertIsNone(state['map']['configured_match'])
        self.assertEqual(state['health']['missing'], ['/map'])
        self.assertIsNone(state['robot']['pose'], 'unverified frame must not overlay the configured map')
        self.assertFalse(self.app.readiness(for_home=True)['ready'])
        image = cv2.imdecode(np.frombuffer(self.app.map_png, np.uint8), cv2.IMREAD_GRAYSCALE)
        np.testing.assert_array_equal(image, [[0, 205, 254], [254, 0, 205]])
        with self.assertRaises(MissionError):
            self.app.command('/api/mission/start', {})
        self.assertEqual(self.app.mission.commands, [])

    def test_matching_live_map_unlocks_only_map_condition(self):
        self.app.load_map_preview(self.path)
        metadata, data = load_saved_map(self.path)
        self.app.receive_map(metadata, data)
        self.assertFalse(self.app.map_is_preview)
        self.assertTrue(self.app.map_matches_configured)
        self.assertNotIn('/map', self.app.readiness()['missing'])
        self.assertFalse(self.app.readiness()['ready'], 'other live sensor requirements still apply')
        self.other_conditions_ready()
        self.assertTrue(self.app.readiness()['ready'])
        self.assertEqual(self.app.map['source'], 'ros')

    def test_mismatching_live_grid_or_coordinates_block_navigation(self):
        metadata, data = load_saved_map(self.path)
        mismatches = []
        different_grid = data.copy()
        different_grid[0, 0] = 100
        mismatches.append((metadata, different_grid, 'occupancy'))
        larger_metadata = dict(deepcopy(metadata), width=4, height=4)
        mismatches.append((larger_metadata, np.zeros((4, 4), np.int16), 'dimensions'))
        for key, value in [('resolution', .1), ('frame_id', 'odom')]:
            different_metadata = deepcopy(metadata)
            different_metadata[key] = value
            mismatches.append((different_metadata, data, key))
        for key in ('x', 'y', 'yaw'):
            different_metadata = deepcopy(metadata)
            different_metadata['origin'][key] += .1
            mismatches.append((different_metadata, data, key))
        for changed_metadata, changed_data, label in mismatches:
            with self.subTest(label=label):
                self.app.load_map_preview(self.path)
                preview = deepcopy(self.app.map)
                preview_png = self.app.map_png
                self.other_conditions_ready()
                self.app.path = [{'x': 8., 'y': 9.}]
                self.app.receive_map(changed_metadata, changed_data)
                self.assertTrue(self.app.map['preview'])
                self.assertFalse(self.app.map['configured_match'])
                self.assertEqual(self.app.map['source'], 'saved_preview')
                self.assertEqual((self.app.map['width'], self.app.map['height']), (preview['width'], preview['height']))
                self.assertEqual(self.app.map['origin'], preview['origin'])
                self.assertEqual(self.app.map['resolution'], preview['resolution'])
                self.assertEqual(self.app.map_png, preview_png)
                state = self.app.state()
                self.assertIsNone(state['robot']['pose'])
                self.assertEqual(state['path'], [])
                self.assertIsNotNone(self.app.pose, 'only the rendered pose is hidden')
                self.assertEqual(self.app.readiness()['missing'], ['/map', '설정 지도와 다른 /map'])
                self.assertEqual(state['health']['map']['received'], changed_metadata)
                with self.assertRaises(MissionError):
                    self.app.command('/api/mission/start', {})
                self.app.receive_map(metadata, data)
                state = self.app.state()
                self.assertFalse(state['map']['preview'])
                self.assertEqual(state['map']['source'], 'ros')
                self.assertIsNotNone(state['robot']['pose'])
                self.assertEqual(state['path'], self.app.path)

    def test_reconnect_restores_preview_and_requires_new_matching_live_map(self):
        self.app.load_map_preview(self.path)
        self.app.receive_map(*load_saved_map(self.path))
        version = self.app.map_version
        self.app.restore_map_preview()
        self.other_conditions_ready()
        self.assertGreater(self.app.map_version, version)
        self.assertTrue(self.app.map_is_preview)
        self.assertIsNone(self.app.map_matches_configured)
        self.assertEqual(self.app.readiness()['missing'], ['/map'])

    def test_live_map_replacement_pauses_existing_mission_and_cancels_motion(self):
        self.app.load_map_preview(self.path)
        metadata, data = load_saved_map(self.path)
        self.app.receive_map(metadata, data)
        self.other_conditions_ready()
        self.app.command('/api/mission/start', {})
        data[0, 0] = 100
        self.app.receive_map(metadata, data)
        self.assertEqual(self.app.mission.state, 'PAUSED')
        self.assertEqual(self.app.mission.commands, [('cancel', None), ('stop', None)])
        with self.assertRaises(MissionError):
            self.app.command('/api/mission/resume', {})
        # Receiving the right map again does not resume or publish a goal.
        self.app.receive_map(*load_saved_map(self.path))
        self.assertEqual(self.app.mission.state, 'PAUSED')

    def test_no_configured_map_preserves_existing_live_map_readiness(self):
        self.assertIsNone(build_parser().parse_args([]).map_file)
        self.assertEqual(build_parser().parse_args(['--map-file', str(self.path)]).map_file, str(self.path))
        self.app.receive_map(*load_saved_map(self.path))
        self.other_conditions_ready()
        self.assertTrue(self.app.readiness()['ready'])
        self.assertIsNone(self.app.map_matches_configured)
        self.app.restore_map_preview()
        self.assertIsNone(self.app.map)

    def test_bad_map_parameters_rejected_before_showing_preview(self):
        for key, value in [('mode', 'scale'), ('resolution', 0), ('resolution', float('nan')),
                           ('origin', [0, 0]), ('origin', [0, float('inf'), 0]),
                           ('free_thresh', .8), ('negate', 2)]:
            with self.subTest(key=key, value=value):
                document = deepcopy(self.document)
                document[key] = value
                self.path.write_text(yaml.safe_dump(document), encoding='utf-8')
                with self.assertRaises(ValueError):
                    self.app.load_map_preview(self.path)
                self.assertIsNone(self.app.map)

    def test_repeated_wrong_live_map_keeps_preview_layout_and_version_cached(self):
        with patch('smart_vision_amr.web_mission_server.map_layout', wraps=map_layout) as generate:
            self.app.load_map_preview(self.path)
            version, cached = self.app.map_version, self.app.map_layout
            metadata, data = load_saved_map(self.path)
            metadata['origin']['x'] += 100
            for _ in range(10):
                self.app.receive_map(metadata, data)
            self.assertEqual(generate.call_count, 1)
            self.assertEqual(self.app.map_version, version)
            self.assertIs(self.app.map_layout, cached)
            self.assertEqual(cached['version'], version)
            self.assertFalse(self.app.map['configured_match'])

    def test_live_map_geometry_change_rebuilds_vector_cache_once(self):
        metadata, data = load_saved_map(self.path)
        self.app.receive_map(metadata, data)
        version = self.app.map_version
        self.app.receive_map(metadata, data.copy())
        self.assertEqual(self.app.map_version, version)
        data[0, 0] = 100
        self.app.receive_map(metadata, data)
        self.assertEqual(self.app.map_layout['version'], version+1)
        self.assertEqual(self.app.map_version, version+1)

    def write_floorplan(self, **changes):
        document = {
            'map': {'width': 3, 'height': 2, 'resolution': .05,
                    'origin': self.document['origin']},
            'style': 'architectural', 'label': '강의실 평면도',
            'free_paths': [[[0, 0], [3, 0], [3, 2], [0, 2]]],
            'occupied_paths': [[[0, 0], [.25, 0], [.25, 2], [0, 2]]],
            'obstacle_paths': [[[1, 1], [2, 1], [2, 1.5], [1, 1.5]]],
        }
        document.update(changes)
        path = self.directory/'floorplan.yaml'
        path.write_text(yaml.safe_dump(document), encoding='utf-8')
        return path, document

    def test_authored_floorplan_replaces_only_display_geometry_and_invalidates_cache(self):
        self.app.load_map_preview(self.path)
        metadata, raw_data = load_saved_map(self.path)
        png, version = self.app.map_png, self.app.map_version
        path, document = self.write_floorplan()
        self.app.load_floorplan(path)
        self.assertEqual(self.app.map_version, version+1)
        for key in ('free_paths', 'occupied_paths', 'obstacle_paths', 'style', 'label'):
            self.assertEqual(self.app.map_layout[key], document[key])
        self.assertEqual(self.app.map_layout['version'], version+1)
        self.assertEqual(self.app.map_png, png)
        np.testing.assert_array_equal(self.app._configured_map[1], raw_data)
        np.testing.assert_array_equal(self.app._display_map_data, raw_data)
        self.other_conditions_ready()
        self.assertFalse(self.app.readiness()['ready'], 'display artwork does not replace live /map')
        self.app.receive_map(metadata, raw_data)
        self.assertTrue(self.app.readiness()['ready'])
        self.assertEqual(self.app.map_layout['style'], 'architectural')
        self.assertEqual(self.app.map_version, version+1, 'same live map reuses authored geometry')
        self.app.restore_map_preview()
        self.assertEqual(self.app.map_layout['occupied_paths'], document['occupied_paths'])

    def test_authored_floorplan_needs_saved_map_and_matching_coordinate_frame(self):
        self.assertIsNone(build_parser().parse_args([]).floorplan_file)
        path, document = self.write_floorplan()
        self.assertEqual(build_parser().parse_args(['--floorplan-file', str(path)]).floorplan_file, str(path))
        with self.assertRaisesRegex(ValueError, '--map-file'):
            self.app.load_floorplan(path)
        self.app.load_map_preview(self.path)
        original = deepcopy(self.app.map_layout)
        for key, value in [('width', 4), ('height', 3), ('resolution', .1),
                           ('origin', [0, -5.69, .2]), ('origin', [-1.53, -5.69, .3])]:
            with self.subTest(key=key, value=value):
                geometry = dict(document['map'], **{key: value})
                path, _ = self.write_floorplan(map=geometry)
                with self.assertRaises(ValueError):
                    self.app.load_floorplan(path)
                self.assertEqual(self.app.map_layout, original)

    def test_authored_polygons_reject_nonfinite_out_of_bounds_or_degenerate_vertices(self):
        self.app.load_map_preview(self.path)
        _, document = self.write_floorplan()
        invalid_rings = [[], [[[0, 0], [1, 1]]], [[[0, 0], [1, 1], [2, 2]]],
                         [[[0, 0], [1, 0], [1, float('nan')]]],
                         [[[0, 0], [1, 0], [1, float('inf')]]],
                         [[[0, 0], [1, 0], [4, 1]]],
                         [[[0, 0], [1, 0], [True, 1]]]]
        for key in ('free_paths', 'occupied_paths', 'obstacle_paths'):
            for rings in invalid_rings:
                if key == 'obstacle_paths' and not rings:
                    continue  # The optional fixture layer may be empty.
                with self.subTest(key=key, rings=rings):
                    path, _ = self.write_floorplan(**{key: rings})
                    with self.assertRaises(ValueError):
                        self.app.load_floorplan(path)
        path, _ = self.write_floorplan(obstacle_paths=[])
        self.app.load_floorplan(path)
        self.assertEqual(self.app.map_layout['obstacle_paths'], [])
        self.assertEqual(self.app.map_layout['free_paths'], document['free_paths'])

    def test_wrong_live_map_keeps_authored_preview_and_blocks_coordinate_overlays(self):
        self.app.load_map_preview(self.path)
        path, document = self.write_floorplan()
        self.app.load_floorplan(path)
        metadata, data = load_saved_map(self.path)
        data[0, 0] = 100
        self.other_conditions_ready()
        self.app.path = [{'x': 1., 'y': 2.}]
        self.app.receive_map(metadata, data)
        state = self.app.state()
        self.assertTrue(state['map']['preview'])
        self.assertFalse(state['map']['configured_match'])
        self.assertIsNone(state['robot']['pose'])
        self.assertEqual(state['path'], [])
        self.assertEqual(self.app.map_layout['free_paths'], document['free_paths'])
        self.assertFalse(self.app.readiness()['ready'])
        self.app.load_map_preview(self.path)
        self.assertIsNone(self.app.configured_floorplan_file)
        self.assertNotIn('style', self.app.map_layout, 'a newly selected map discards prior artwork')


class MapFloorplanTests(unittest.TestCase):
    @staticmethod
    def contains(paths, x, y):
        return sum(cv2.pointPolygonTest(np.asarray(ring, np.float32), (x, y), False) > 0
                   for ring in paths) % 2 == 1

    def test_pixel_edges_top_left_origin_and_single_cell_wall(self):
        top_down = np.array([[100, -1, 0], [100, -1, -1], [100, -1, -1]], np.int16)
        layout = map_layout(np.flipud(top_down))
        self.assertEqual((layout['width'], layout['height']), (3, 3))
        self.assertEqual({tuple(point) for point in layout['occupied_paths'][0]},
                         {(0, 0), (1, 0), (1, 3), (0, 3)})
        self.assertEqual({tuple(point) for point in layout['free_paths'][0]},
                         {(2, 0), (3, 0), (3, 1), (2, 1)})
        self.assertFalse(self.contains(layout['free_paths'], 1.5, 1.5), 'unknown is never free floor')

    def test_holes_remain_holes_with_evenodd_fill(self):
        top_down = np.full((7, 7), -1, np.int16)
        top_down[1:6, 1:6] = 0
        top_down[2:5, 2:5] = 100
        layout = map_layout(np.flipud(top_down))
        self.assertEqual(len(layout['free_paths']), 2)
        self.assertTrue(self.contains(layout['free_paths'], 1.5, 3.5))
        self.assertFalse(self.contains(layout['free_paths'], 3.5, 3.5))
        self.assertTrue(self.contains(layout['occupied_paths'], 3.5, 3.5))
        self.assertFalse(self.contains(layout['free_paths'], .5, .5))

    def test_diagonal_single_cells_do_not_collapse_or_join(self):
        paths = cell_boundary_paths(np.array([[1, 0], [0, 1]], bool))
        self.assertEqual(len(paths), 2)
        self.assertTrue(self.contains(paths, .5, .5))
        self.assertTrue(self.contains(paths, 1.5, 1.5))
        self.assertFalse(self.contains(paths, 1.5, .5))

    def test_exact_boundary_tracing_matches_random_cells_including_holes(self):
        generator = np.random.default_rng(16)
        for _ in range(25):
            mask = generator.random((8, 9)) > .5
            paths = cell_boundary_paths(mask, epsilon=0)
            for row in range(mask.shape[0]):
                for column in range(mask.shape[1]):
                    self.assertEqual(self.contains(paths, column+.5, row+.5), mask[row, column])

    def test_layout_is_readonly_and_uses_only_defined_free_and_occupied_cells(self):
        data = np.array([[-1, 0, 20, 64, 65, 100]], np.int16)
        original = data.copy()
        layout = map_layout(data)
        np.testing.assert_array_equal(data, original)
        for column in range(6):
            self.assertEqual(self.contains(layout['free_paths'], column+.5, .5), column == 1)
            self.assertEqual(self.contains(layout['occupied_paths'], column+.5, .5), column >= 4)


if __name__ == '__main__':
    unittest.main()
