#!/usr/bin/env python3
"""Copy real classroom Nav2 tuning into a Gazebo-compatible sim-time YAML."""

import argparse
import hashlib
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "ros2_ws/src/smart_vision_amr/config/curve_tuning_waffle_pi.yaml"


def set_sim_time(value):
    if isinstance(value, dict):
        return {key: True if key == "use_sim_time" else set_sim_time(item)
                for key, item in value.items()}
    if isinstance(value, list):
        return [set_sim_time(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--source", type=Path, default=SOURCE,
                        help="Nav2 parameter YAML to copy; default is the classroom hardware profile")
    args = parser.parse_args()
    if not args.source.is_file():
        parser.error(f"Source Nav2 YAML does not exist: {args.source}")
    data = yaml.safe_load(args.source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        parser.error("Hardware Nav2 YAML is not a parameter dictionary")
    converted = set_sim_time(data)
    header = ("# Generated from a specified Nav2 profile; only use_sim_time changes.\n"
              f"# Source: {args.source}\n"
              f"# Source SHA256: {hashlib.sha256(args.source.read_bytes()).hexdigest()}\n")
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    args.destination.write_text(header + yaml.safe_dump(converted, sort_keys=False,
                              allow_unicode=True), encoding="utf-8")
    print(f"Gazebo Nav2 profile: {args.destination}")


if __name__ == "__main__":
    main()
