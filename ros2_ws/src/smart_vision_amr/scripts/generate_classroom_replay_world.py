#!/usr/bin/env python3
"""Build Gazebo collision geometry from the *actual* October 30 map pixels.

Every black cell in classroom_20261030_cleaned.pgm becomes a 5 cm occupied
square at that cell's native map coordinate. Adjacent squares are merged into
rectangles without moving or deleting any occupied cell. Unknown cells do not
silently become walls. The central box is an explicit, documented inference
from its visible boundary and unknown interior in the map.
"""

import argparse
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "maps/classroom_20261030_cleaned.yaml"
WORLD = ROOT / "worlds/classroom_20261030_replay.world"
STRESS_WORLD = ROOT / "worlds/classroom_20261030_stress.world"
EXPECTED_SIZE = (86, 140)
EXPECTED_ORIGIN = (-1.65, -5.25)
EXPECTED_RESOLUTION = 0.05

# Marker wall coordinates were not surveyed. These fixed raycast anchors infer
# the four marker boards from the map walls; they do not follow later changes
# to robot observation goals. In particular, marker 1 remains on the original
# wall when its robot observation pose moves inward to allow a full turn.
# Marker 2 likewise remains at the wall when its robot stop moves inward;
# changing the registered stop must not move a physical wall marker.
# The 80/100 mm sizes refer to the black marker edge; the existing model's
# printed white margin is retained at the same 1.4 ratio.
MARKERS = (
    (0, -0.188431, -0.220497, -0.027514, 0.080),
    (1, 1.475000, 0.575000, 3.114078, 0.080),
    (2, 1.561552, -2.789613, 1.543282, 0.100),
    (3, -0.231852, -1.979969, -1.598311, 0.100),
)

# Added only to the stress world. The crate lies wholly in known free map
# pixels, so it is correctly an *unmapped* obstacle for local avoidance.
# Its placement blocks the old Sector 1→2 path while leaving a safe detour.
# An earlier second crate near Sector 3 blocked the registered observation
# goal's physical safety envelope, so it is deliberately excluded.
STRESS_CRATES = (
    ("stress_unmapped_crate_center", 0.35, -0.60, 0.20, 0.20, 0.48),
)


def read_map(path=MAP_YAML):
    """Read the project's simple ROS map YAML and binary PGM without extras."""
    config = path.read_text(encoding="utf-8")
    image = re.search(r"^image:\s*(\S+)\s*$", config, re.M)
    resolution = re.search(r"^resolution:\s*([\d.]+)\s*$", config, re.M)
    origin = re.search(r"^origin:\s*\[([^]]+)\]", config, re.M)
    if not (image and resolution and origin):
        raise ValueError(f"Incomplete map metadata: {path}")
    res = float(resolution.group(1))
    ox, oy, heading = (float(value) for value in origin.group(1).split(","))
    if abs(heading) > 1e-9:
        raise ValueError("A rotated map origin requires a different raster transform")
    blob = (path.parent / image.group(1)).read_bytes()
    match = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", blob)
    if match is None:
        raise ValueError("Expected an uncommented binary P5 occupancy image")
    width, height, maximum = map(int, match.groups())
    pixels = blob[match.end():]
    if maximum != 255 or len(pixels) != width * height:
        raise ValueError("PGM sample depth or byte count does not match its header")
    if (width, height) != EXPECTED_SIZE or abs(res - EXPECTED_RESOLUTION) > 1e-9:
        raise ValueError("Classroom map resolution or dimensions unexpectedly changed")
    if abs(ox - EXPECTED_ORIGIN[0]) > 1e-9 or abs(oy - EXPECTED_ORIGIN[1]) > 1e-9:
        raise ValueError("Classroom map origin unexpectedly changed")
    return width, height, res, ox, oy, pixels


def rectangles(pixels, width, height):
    """Losslessly merge occupied pixels (PGM value 0) into row-run rectangles."""
    output = []
    active = {}
    for row in range(height + 1):
        runs = []
        if row < height:
            first = None
            for column in range(width + 1):
                occupied = column < width and pixels[row * width + column] == 0
                if occupied and first is None:
                    first = column
                elif not occupied and first is not None:
                    runs.append((first, column))
                    first = None
        keep = set(runs)
        for key in sorted(set(active) - keep):
            output.append(active.pop(key))
        for key in runs:
            if key in active:
                active[key][3] = row + 1
            else:
                active[key] = [key[0], key[1], row, row + 1]
    return sorted(output, key=lambda item: (item[2], item[0], item[3], item[1]))


def add_text(parent, name, value, **attributes):
    child = ET.SubElement(parent, name, attributes)
    child.text = str(value)
    return child


def xyz(*values):
    return " ".join(f"{value:.6f}" for value in values)


def add_box(parent, name, cx, cy, z, sx, sy, sz, color, collision=True):
    link = ET.SubElement(parent, "link", name=name)
    add_text(link, "pose", xyz(cx, cy, z, 0, 0, 0))
    if collision:
        col = ET.SubElement(link, "collision", name="collision")
        add_text(ET.SubElement(ET.SubElement(col, "geometry"), "box"), "size", xyz(sx, sy, sz))
    vis = ET.SubElement(link, "visual", name="visual")
    add_text(ET.SubElement(ET.SubElement(vis, "geometry"), "box"), "size", xyz(sx, sy, sz))
    material = ET.SubElement(vis, "material")
    add_text(material, "ambient", color)
    add_text(material, "diffuse", color)
    return link


def first_wall_hit(marker, occupancy):
    marker_id, x, y, yaw, _size = marker
    width, height, res, ox, oy, pixels = occupancy
    # The rear camera points opposite the robot's heading.
    angle = yaw + math.pi
    for index in range(1, 121):
        distance = index * 0.0125
        xx = x + distance * math.cos(angle)
        yy = y + distance * math.sin(angle)
        col = math.floor((xx - ox) / res)
        row = height - 1 - math.floor((yy - oy) / res)
        if not (0 <= col < width and 0 <= row < height):
            break
        if pixels[row * width + col] == 0:
            return col, row, distance
    raise ValueError(f"No observed wall along rear camera for marker {marker_id}")


def add_marker(world, marker, occupancy):
    marker_id, x, y, yaw, size = marker
    width, height, res, ox, oy, _pixels = occupancy
    col, row, distance = first_wall_hit(marker, occupancy)
    # Position at the *free side* of the first occupied cell. The board does
    # not add a new collision; the raster-derived wall remains authoritative.
    wall_x = ox + (col + 0.5) * res
    wall_y = oy + (height - row - 0.5) * res
    view_x, view_y = math.cos(yaw + math.pi), math.sin(yaw + math.pi)
    board_x = wall_x - view_x * (0.5 * res + 0.003)
    board_y = wall_y - view_y * (0.5 * res + 0.003)
    # Marker mesh front is local -X; turn this side toward the observation pose.
    board_yaw = math.atan2(-view_y, -view_x) - math.pi
    model = ET.SubElement(world, "model", name=f"classroom_marker_{marker_id}")
    add_text(model, "static", "true")
    add_text(model, "pose", xyz(board_x, board_y, 0.22, 0, 0, board_yaw))
    link = ET.SubElement(model, "link", name="printed_sheet")
    board_side = 1.4 * size
    board = ET.SubElement(link, "visual", name="white_board")
    add_text(ET.SubElement(ET.SubElement(board, "geometry"), "box"), "size", xyz(0.003, board_side, board_side))
    material = ET.SubElement(board, "material")
    add_text(material, "ambient", "1 1 1 1")
    add_text(material, "diffuse", "1 1 1 1")
    printed = ET.SubElement(link, "visual", name="aruco_texture")
    add_text(printed, "pose", "-0.0017 0 0 0 0 0")
    mesh = ET.SubElement(ET.SubElement(printed, "geometry"), "mesh")
    add_text(mesh, "uri", f"model://smart_vision_aruco_4x4_50_id_{marker_id}/meshes/marker.obj")
    add_text(mesh, "scale", xyz(size / 0.2, size / 0.2, size / 0.2))
    script = ET.SubElement(ET.SubElement(printed, "material"), "script")
    add_text(script, "uri", f"model://smart_vision_aruco_4x4_50_id_{marker_id}/materials/scripts")
    add_text(script, "uri", f"model://smart_vision_aruco_4x4_50_id_{marker_id}/materials/textures")
    add_text(script, "name", f"SmartVision/smart_vision_aruco_4x4_50_id_{marker_id}")
    add_text(printed, "cast_shadows", "false")
    return distance


def build_world(occupancy, stress=False):
    width, height, res, ox, oy, pixels = occupancy
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world",
                          name="classroom_20261030_stress" if stress else "classroom_20261030_replay")
    add_text(world, "gravity", "0 0 -9.81")
    physics = ET.SubElement(world, "physics", name="default_physics", default="true", type="ode")
    add_text(physics, "max_step_size", "0.001")
    add_text(physics, "real_time_factor", "1")
    add_text(physics, "real_time_update_rate", "1000")
    scene = ET.SubElement(world, "scene")
    add_text(scene, "ambient", "0.6 0.6 0.6 1")
    add_text(scene, "background", "0.8 0.85 0.9 1")
    add_text(scene, "shadows", "false")
    ground = ET.SubElement(world, "model", name="ground_plane")
    add_text(ground, "static", "true")
    ground_link = ET.SubElement(ground, "link", name="floor")
    for kind in ("collision", "visual"):
        part = ET.SubElement(ground_link, kind, name=kind)
        plane = ET.SubElement(ET.SubElement(part, "geometry"), "plane")
        add_text(plane, "normal", "0 0 1")
        add_text(plane, "size", "20 20")
        if kind == "visual":
            material = ET.SubElement(part, "material")
            add_text(material, "ambient", "0.90 0.89 0.86 1")
            add_text(material, "diffuse", "0.90 0.89 0.86 1")
    walls = ET.SubElement(world, "model", name="map_occupied_cells")
    add_text(walls, "static", "true")
    rects = rectangles(pixels, width, height)
    for index, (c0, c1, r0, r1) in enumerate(rects):
        x0 = ox + c0 * res
        x1 = ox + c1 * res
        y0 = oy + (height - r1) * res
        y1 = oy + (height - r0) * res
        add_box(walls, f"occupied_{index:02d}", (x0 + x1) / 2, (y0 + y1) / 2,
                0.35, x1 - x0, y1 - y0, 0.70, "0.34 0.37 0.39 1")
    # The center island has a rectangular outline and unknown interior. Keep
    # this inferred solid strictly inside unknown map pixels: extending into
    # known-free pixels makes an empty baseline world look like a new obstacle.
    # These bounds coincide with 5 cm map-cell boundaries and cover only
    # unknown cells. The occupied outline is already modeled above.
    center = ET.SubElement(world, "model", name="inferred_center_object")
    add_text(center, "static", "true")
    add_box(center, "body", 1.075, -0.100, 0.26, 0.65, 0.50, 0.52,
            "0.62 0.62 0.57 1")
    if stress:
        for name, x, y, sx, sy, height_m in STRESS_CRATES:
            crate = ET.SubElement(world, "model", name=name)
            add_text(crate, "static", "true")
            add_box(crate, "body", x, y, height_m / 2, sx, sy, height_m,
                    "0.92 0.34 0.16 1")
    distances = {marker[0]: add_marker(world, marker, occupancy) for marker in MARKERS}
    light = ET.SubElement(world, "light", name="overhead", type="directional")
    add_text(light, "pose", "0 0 6 0 0 0")
    add_text(light, "diffuse", "0.85 0.85 0.85 1")
    add_text(light, "specular", "0.1 0.1 0.1 1")
    add_text(light, "direction", "-0.3 0.1 -0.9")
    state_plugin = ET.SubElement(world, "plugin", name="gazebo_state", filename="libgazebo_ros_state.so")
    state_ros = ET.SubElement(state_plugin, "ros")
    add_text(state_ros, "namespace", "/gazebo")
    add_text(state_plugin, "update_rate", "10.0")
    gui = ET.SubElement(world, "gui", fullscreen="false")
    camera = ET.SubElement(gui, "camera", name="overview")
    add_text(camera, "pose", "5 -7 7 0 0.8 2.15")
    ET.indent(sdf, space="  ")
    return b"<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(sdf, encoding="utf-8"), rects, distances


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="compare generated world without changing it")
    args = parser.parse_args()
    occupancy = read_map()
    world, rects, distances = build_world(occupancy)
    stress_world, _stress_rects, _stress_distances = build_world(occupancy, stress=True)
    for path, data in ((WORLD, world), (STRESS_WORLD, stress_world)):
        if args.check:
            if not path.is_file() or path.read_bytes() != data:
                parser.exit(1, f"World differs from the current map: {path}\n")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    print(f"{'Verified' if args.check else 'Generated'} "
          f"{WORLD.name} and {STRESS_WORLD.name}: "
          f"{sum((c1-c0)*(r1-r0) for c0,c1,r0,r1 in rects)} occupied cells "
          f"in {len(rects)} exact rectangles; "
          f"rear-view wall distances {distances}")


if __name__ == "__main__":
    main()
