#!/usr/bin/env python3
"""Derive Gazebo classroom profiles without changing physical robot tuning."""

import argparse
import hashlib
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "ros2_ws/src/smart_vision_amr/config/curve_tuning_waffle_pi.yaml"
NODES_SOURCE = ROOT / "ros2_ws/src/smart_vision_amr/config/classroom_hardware_nodes.yaml"

# Stock Waffle Pi SDF collision geometry reaches about 0.237 m from its
# rotation center. A 0.24 m radius covers it; four 5 cm cells would not.
SIM_ROBOT_RADIUS_M = 0.24
SIM_INFLATION_RADIUS_M = 0.26
SIM_STATIC_INFLATION_CELLS = 5
SIM_PREFERRED_CLEARANCE_M = 0.45


def set_sim_time(value):
    if isinstance(value, dict):
        return {key: True if key == "use_sim_time" else set_sim_time(item)
                for key, item in value.items()}
    if isinstance(value, list):
        return [set_sim_time(item) for item in value]
    return value


def build_nav_profile_data(source, *, physical_footprint=False):
    converted = set_sim_time(source)
    if physical_footprint:
        return converted
    for name in ("local_costmap", "global_costmap"):
        parameters = converted[name][name]["ros__parameters"]
        parameters["robot_radius"] = SIM_ROBOT_RADIUS_M
        parameters["inflation_layer"]["inflation_radius"] = SIM_INFLATION_RADIUS_M
    return converted


def build_nodes_profile_data(source, *, physical_footprint=False):
    converted = set_sim_time(source)
    if physical_footprint:
        return converted
    planner = converted["/astar_planner"]["ros__parameters"]
    planner["robot_radius"] = SIM_ROBOT_RADIUS_M
    planner["obstacle_inflation_cells"] = SIM_STATIC_INFLATION_CELLS
    converted["/path_follower"]["ros__parameters"]["recovery_robot_radius"] = SIM_ROBOT_RADIUS_M
    return converted


def _load_parameters(path, parser):
    if not path.is_file():
        parser.error(f"Source YAML does not exist: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        parser.error(f"Source YAML is not a parameter dictionary: {path}")
    return data


def _write_profile(destination, source, converted, purpose, *, physical_footprint=False):
    header = (f"# Gazebo {purpose} profile generated from checked-in physical tuning.\n"
              f"# Sim overrides: robot radius {SIM_ROBOT_RADIUS_M} m, soft inflation "
              f"{SIM_INFLATION_RADIUS_M} m for Nav2, static inflation "
              f"{SIM_STATIC_INFLATION_CELLS} cells for A*, source preferred clearance retained.\n"
              f"# Source: {source}\n"
              f"# Source SHA256: {hashlib.sha256(source.read_bytes()).hexdigest()}\n")
    if physical_footprint:
        header = (f"# Physical-footprint Gazebo {purpose}: only use_sim_time converted.\n"
                  "# Requires the matching copied 0.22 m collision proxy.\n"
                  f"# Source: {source}\n"
                  f"# Source SHA256: {hashlib.sha256(source.read_bytes()).hexdigest()}\n")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(header + yaml.safe_dump(converted, sort_keys=False,
                           allow_unicode=True), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--source", type=Path, default=SOURCE,
                        help="Nav2 parameter YAML to copy; default is the classroom hardware profile")
    parser.add_argument("--nodes-source", type=Path, default=NODES_SOURCE,
                        help="A*/follower source YAML; default is the classroom hardware profile")
    parser.add_argument("--nodes-destination", type=Path,
                        help="Write the matching Gazebo A*/follower profile to this path")
    parser.add_argument('--physical-footprint', action='store_true',
                        help='Preserve all physical parameters and convert only simulation clock')
    args = parser.parse_args()
    source = _load_parameters(args.source, parser)
    try:
        converted = build_nav_profile_data(source, physical_footprint=args.physical_footprint)
    except (KeyError, TypeError) as exc:
        parser.error(f"Nav2 profile lacks a required costmap setting: {exc}")
    _write_profile(args.destination, args.source, converted, "Nav2", physical_footprint=args.physical_footprint)
    print(f"Gazebo Nav2 profile: {args.destination}")
    if args.nodes_destination is not None:
        source = _load_parameters(args.nodes_source, parser)
        try:
            converted = build_nodes_profile_data(source, physical_footprint=args.physical_footprint)
        except (KeyError, TypeError) as exc:
            parser.error(f"Nodes profile lacks a required planner/follower setting: {exc}")
        _write_profile(args.nodes_destination, args.nodes_source, converted,
                       "A*/follower", physical_footprint=args.physical_footprint)
        print(f"Gazebo nodes profile: {args.nodes_destination}")


if __name__ == "__main__":
    main()
