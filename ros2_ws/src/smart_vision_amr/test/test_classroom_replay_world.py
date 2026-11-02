"""Regression checks for the raster-faithful classroom Gazebo fixture."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / 'scripts/generate_classroom_replay_world.py'
spec = importlib.util.spec_from_file_location('generate_classroom_replay_world', GENERATOR)
world_generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(world_generator)


def test_map_pixels_are_exactly_covered_by_world_wall_rectangles():
    width, height, _res, _ox, _oy, pixels = world_generator.read_map()
    covered = set()
    for c0, c1, r0, r1 in world_generator.rectangles(pixels, width, height):
        for row in range(r0, r1):
            for col in range(c0, c1):
                assert (col, row) not in covered
                covered.add((col, row))
    expected = {(col, row) for row in range(height) for col in range(width)
                if pixels[row * width + col] == 0}
    assert covered == expected
    assert len(covered) == 604


def test_baseline_and_stress_worlds_match_generator_and_keep_markers():
    occupancy = world_generator.read_map()
    baseline, _, _ = world_generator.build_world(occupancy)
    stress, _, _ = world_generator.build_world(occupancy, stress=True)
    assert world_generator.WORLD.read_bytes() == baseline
    assert world_generator.STRESS_WORLD.read_bytes() == stress
    base_tree = ET.fromstring(baseline)
    stress_tree = ET.fromstring(stress)
    for marker_id in range(4):
        model = f"world/model[@name='classroom_marker_{marker_id}']"
        assert base_tree.find(model) is not None
        assert stress_tree.find(model) is not None
    assert len(base_tree.findall("world/model[@name='map_occupied_cells']/link")) == 47
    assert base_tree.find("world/model[@name='stress_unmapped_crate_left']") is None
    assert stress_tree.find("world/model[@name='stress_unmapped_crate_left']") is None
    center = stress_tree.find("world/model[@name='stress_unmapped_crate_center']/link")
    assert center is not None
    assert center.findtext('pose').startswith('0.350000 -0.600000 ')
    assert base_tree.find("world/plugin[@filename='libgazebo_ros_state.so']") is not None


def test_inferred_center_object_does_not_cover_known_free_map_cells():
    width, height, resolution, origin_x, origin_y, pixels = world_generator.read_map()
    baseline, _, _ = world_generator.build_world((width, height, resolution,
                                                   origin_x, origin_y, pixels))
    tree = ET.fromstring(baseline)
    link = tree.find("world/model[@name='inferred_center_object']/link")
    assert link is not None
    center_x, center_y, *_ = (float(value) for value in link.findtext('pose').split())
    size_x, size_y, *_ = (float(value) for value in
                          link.findtext('collision/geometry/box/size').split())
    bounds = (center_x - size_x / 2, center_y - size_y / 2,
              center_x + size_x / 2, center_y + size_y / 2)
    touched = []
    for row in range(height):
        cell_y = origin_y + (height - 1 - row) * resolution
        for column in range(width):
            cell_x = origin_x + column * resolution
            if (cell_x < bounds[2] - 1e-9 and cell_x + resolution > bounds[0] + 1e-9
                    and cell_y < bounds[3] - 1e-9
                    and cell_y + resolution > bounds[1] + 1e-9):
                touched.append(pixels[row * width + column])
    assert touched
    assert set(touched) == {205}  # only the map's unknown interior


def test_stress_crate_is_unmapped_and_has_footprint_width_exits():
    width, height, resolution, origin_x, origin_y, pixels = world_generator.read_map()
    occupied = []
    for row in range(height):
        for column in range(width):
            if pixels[row * width + column] == 0:
                occupied.append((origin_x + column * resolution,
                                 origin_y + (height - 1 - row) * resolution,
                                 origin_x + (column + 1) * resolution,
                                 origin_y + (height - row) * resolution))

    for _name, x, y, size_x, size_y, _height_m in world_generator.STRESS_CRATES:
        box = (x - size_x / 2, y - size_y / 2,
               x + size_x / 2, y + size_y / 2)
        assert all(box[2] <= cell[0] or box[0] >= cell[2]
                   or box[3] <= cell[1] or box[1] >= cell[3]
                   for cell in occupied)
        for row in range(height):
            cell_y = origin_y + (height - 1 - row) * resolution
            for column in range(width):
                cell_x = origin_x + column * resolution
                if (cell_x < box[2] and cell_x + resolution > box[0]
                        and cell_y < box[3] and cell_y + resolution > box[1]):
                    assert pixels[row * width + column] == 254

    _name, x, y, size_x, size_y, _height_m = world_generator.STRESS_CRATES[0]
    left_x, right_x = x - size_x / 2, x + size_x / 2
    bottom_y, top_y = y - size_y / 2, y + size_y / 2
    same_y = [cell for cell in occupied if cell[1] < top_y and cell[3] > bottom_y]
    left_gap = min(left_x - cell[2] for cell in same_y if cell[2] <= left_x)
    right_gap = min(cell[0] - right_x for cell in same_y if cell[0] >= right_x)
    robot_diameter = 2 * 0.22  # classroom_hardware_nodes.yaml robot_radius
    assert left_gap > robot_diameter
    assert right_gap > robot_diameter
