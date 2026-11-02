#!/usr/bin/env python3
"""Copy a classroom Gazebo fixture and isolate its genuine sensor streams.

The source Waffle Pi SDF, world and map are never changed. The only SDF edits
are output topic remaps to the fault relay; sensor geometry/noise are retained.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source_model, world, map_yaml, output, mode='baseline'):
    if mode not in ('baseline', 'ekf_independent'):
        raise ValueError('mode must be baseline or ekf_independent')
    output.mkdir(parents=True, exist_ok=False)
    model_copy = output/'waffle_pi_fault.sdf'
    world_copy = output/'classroom_fault.world'
    map_copy = output/'classroom_fault.yaml'
    original = ET.parse(source_model)
    laser = original.find(".//plugin[@name='turtlebot3_laserscan']/ros/remapping")
    drive_ros = original.find(".//plugin[@name='turtlebot3_diff_drive']/ros")
    if laser is None or laser.text != '~/out:=scan' or drive_ros is None:
        raise ValueError('Unexpected Waffle Pi sensor plugin layout')
    laser.text = '~/out:=/classroom_fault/raw_scan'
    if mode == 'baseline':
        for remapping in ('odom:=/classroom_fault/raw_odom',
                          '/tf:=/classroom_fault/raw_tf'):
            ET.SubElement(drive_ros, 'remapping').text = remapping
    # spawn_entity.py receives a Unicode string; an XML encoding declaration
    # causes lxml to reject it before a model is inserted.
    original.write(model_copy, encoding='unicode', xml_declaration=False)
    shutil.copyfile(world, world_copy)
    contents = map_yaml.read_text()
    image_line = next((line for line in contents.splitlines()
                       if line.lstrip().startswith('image:')), None)
    if image_line is None:
        raise ValueError('Map YAML lacks image')
    image_name = image_line.split(':', 1)[1].strip().strip('"\'')
    map_image = (map_yaml.parent/image_name).resolve()
    if not map_image.is_file():
        raise FileNotFoundError(map_image)
    map_image_copy = output/('classroom_fault'+map_image.suffix)
    shutil.copyfile(map_image, map_image_copy)
    map_copy.write_text(contents.replace(image_line, 'image: '+map_image_copy.name))
    manifest = {'mode': mode, 'source': {str(path): digest(path) for path in
                           (source_model, world, map_yaml, map_image)},
                'fixture': {str(path): digest(path) for path in
                            (model_copy, world_copy, map_copy, map_image_copy)},
                'edits': ['copied laser output -> /classroom_fault/raw_scan',
                          *(['copied Gazebo odom output -> /classroom_fault/raw_odom',
                             'copied Gazebo dynamic TF output -> /classroom_fault/raw_tf']
                            if mode == 'baseline' else [])],
                'real_sensor_stamps_preserved': True,
                'world_geometry_unchanged': True,
                'source_map_unchanged': True}
    (output/'fixture_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-model', type=Path, required=True)
    parser.add_argument('--mode', choices=('baseline', 'ekf_independent'),
                        default='baseline')
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--map', dest='map_yaml', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source_model, args.world, args.map_yaml, args.output, args.mode)
    print(args.output/'fixture_manifest.json')


if __name__ == '__main__':
    main()
