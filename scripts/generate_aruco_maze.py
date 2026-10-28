#!/usr/bin/python3
"""Build a separate, wide-corridor maze world with four wall-mounted markers."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_aruco_models import model_name, save_xml


REPO_ROOT = Path(__file__).resolve().parents[1]
WORLD_DIR = REPO_ROOT / "ros2_ws/src/smart_vision_amr/worlds"

# xmin, xmax, ymin, ymax; the outer room remains 10 x 8 m.
WALLS = (
    ("maze_partition_west", -2.06, -1.94, -4.0, 2.0),
    ("maze_partition_middle", .24, .36, -2.0, 4.0),
    ("maze_partition_east", 2.54, 2.66, -4.0, 2.0),
    ("maze_west_side_branch", -5.0, -3.9, .34, .46),
    ("maze_middle_side_branch", -.5, .30, -.06, .06),
    ("maze_east_side_branch", .30, 1.1, .54, .66),
    ("maze_final_side_branch", 4.1, 5.0, .54, .66),
)

MARKERS = (
    {"id": 0, "role": "start", "pose": [-2.0621, -2.0, .22, 0, 0, 0],
     "observation_pose": [-3.0, -2.0, 0]},
    {"id": 1, "role": "waypoint_1", "pose": [.2379, 3.0, .22, 0, 0, 0],
     "observation_pose": [-1.0, 3.0, 0]},
    {"id": 2, "role": "waypoint_2", "pose": [2.5379, -2.8, .22, 0, 0, 0],
     "observation_pose": [1.4, -2.8, 0]},
    {"id": 3, "role": "goal", "pose": [4.99, -2.5, .22, 0, 0, 0],
     "observation_pose": [3.9, -2.5, 0]},
)


def main():
    for marker in MARKERS:
        name = model_name("DICT_4X4_50", marker["id"])
        if not (REPO_ROOT / "gazebo_models" / name / "model.sdf").is_file():
            raise SystemExit("Generate markers first: /usr/bin/python3 scripts/generate_aruco_models.py --ids 0 1 2 3")
    root = ET.parse(WORLD_DIR / "simulation_room.world").getroot()
    world = root.find("world")
    world.set("name", "simulation_maze_aruco")
    for model in list(world.findall("model")):
        if model.get("name") in {"central_obstacle", "upper_left_shelf", "lower_right_shelf"}:
            world.remove(model)
    for name, xmin, xmax, ymin, ymax in WALLS:
        model = ET.SubElement(world, "model", name=name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{(xmin+xmax)/2} {(ymin+ymax)/2} 0.5 0 0 0"
        link = ET.SubElement(model, "link", name="wall")
        for kind in ("collision", "visual"):
            shape = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(shape, "geometry"), "box")
            ET.SubElement(box, "size").text = f"{xmax-xmin} {ymax-ymin} 1.0"
            if kind == "visual":
                material = ET.SubElement(shape, "material")
                ET.SubElement(material, "ambient").text = ".55 .65 .75 1"
                ET.SubElement(material, "diffuse").text = ".55 .65 .75 1"
    for marker in MARKERS:
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = "model://" + model_name("DICT_4X4_50", marker["id"])
        ET.SubElement(include, "name").text = f"aruco_{marker['role']}_id_{marker['id']}"
        ET.SubElement(include, "pose").text = " ".join(str(v) for v in marker["pose"])
    path = WORLD_DIR / "simulation_maze_aruco.world"
    save_xml(root, path)
    layout = {
        "dictionary": "DICT_4X4_50", "marker_length_m": .20,
        "world": path.name, "spawn": [-3.0, -2.0, 0],
        "roles_are_labels_only_not_automatic_navigation": True,
        "minimum_usable_side_gap_m": 1.44,
        "markers": MARKERS,
        "route_hints_xy": [[-3,-2],[-3,3],[-1,3],[-1,-3],[1.4,-3],
                           [1.6,3],[3.7,3],[3.9,-2.5]],
    }
    manifest = REPO_ROOT / "gazebo_models/aruco_maze_layout.json"
    manifest.write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
    print(f"WORLD={path}\nLAYOUT={manifest}")


if __name__ == "__main__":
    main()
