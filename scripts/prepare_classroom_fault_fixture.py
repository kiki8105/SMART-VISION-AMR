#!/usr/bin/env python3
"""Copy a classroom Gazebo fixture and isolate its genuine sensor streams.

The source Waffle Pi SDF, world and map are never changed. By default only
output topics are remapped. The explicit physical-footprint mode also replaces
the copied body collision with the 0.22 m physical circular model; wheelbase,
mass, sensor geometry/noise and original visual mesh are retained. Optional
hardware stress profiles change actual copied sensor producers or wheel/contact
control, without changing source assets or inventing odometry/web feedback.
"""

import argparse
import hashlib
from itertools import product
import json
from math import cos, hypot, isfinite, sin
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np


HARDWARE_PROFILES = ('nominal', 'sensor_rates', 'wheel_asymmetry', 'actuator_lag')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _pose(element):
    values = [float(value) for value in (element.findtext('pose') or '0 0 0 0 0 0').split()]
    if len(values) != 6:
        raise ValueError('collision pose must contain six numbers')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                            [-sp, cp*sr, cp*cr]]
    transform[:3, 3] = x, y, z
    return transform


def collision_radius_bounds(tree):
    """Conservative horizontal bounds for every stock primitive collision.

    Box corners are exact. A cylinder uses a triangle-inequality bound of its
    rotated disc and axial projection. Mesh/relative-frame layouts fail closed;
    they must be modeled explicitly before claiming footprint parity.
    """
    model = tree.find('model')
    if model is None:
        raise ValueError('SDF model is missing')
    result = {}
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            if any(pose.get('relative_to') for pose in (link.find('pose'), collision.find('pose'))
                   if pose is not None):
                raise ValueError('relative collision frames require an explicit geometry model')
            transform = _pose(link) @ _pose(collision)
            geometry = collision.find('geometry')
            box, cylinder, sphere = (geometry.find(name) for name in ('box', 'cylinder', 'sphere'))
            if box is not None:
                dimensions = [float(value) for value in box.findtext('size').split()]
                radius = max(hypot(*(transform @ np.array([sx*dimensions[0]/2,
                    sy*dimensions[1]/2, sz*dimensions[2]/2, 1.]))[:2])
                    for sx, sy, sz in product((-1, 1), repeat=3))
            elif cylinder is not None:
                radius = (hypot(*transform[:2, 3]) + float(cylinder.findtext('radius'))
                          + float(cylinder.findtext('length'))/2 * hypot(*transform[:2, 2]))
            elif sphere is not None:
                radius = hypot(*transform[:2, 3]) + float(sphere.findtext('radius'))
            else:
                raise ValueError('Only box, cylinder and sphere collisions can certify this proxy')
            result[link.get('name')+'/'+collision.get('name')] = float(radius)
    if not result:
        raise ValueError('SDF has no collision geometry')
    return result


def apply_physical_footprint_proxy(tree, radius=.22):
    """Replace only the copied base collision with the physical circular model."""
    base = tree.find("./model/link[@name='base_link']/collision[@name='base_collision']")
    if base is None or radius != .22:
        raise ValueError('physical-footprint replay requires the known base and radius 0.22 m')
    base.find('pose').text = '0 0 0.048 0 0 0'
    geometry = base.find('geometry')
    geometry.clear()
    cylinder = ET.SubElement(geometry, 'cylinder')
    ET.SubElement(cylinder, 'radius').text = str(radius)
    ET.SubElement(cylinder, 'length').text = '0.089'
    bounds = collision_radius_bounds(tree)
    if max(bounds.values()) > radius + 1e-9:
        raise ValueError('an unchanged wheel/sensor/caster exceeds the 0.22 m proxy')
    return bounds


def apply_hardware_profile(tree, profile):
    """Apply an explicit stress envelope to real producers/contact dynamics.

    This deliberately leaves wheel encoder/plugin calibration, physical mass,
    wheel poses, topic names and odometry authority unchanged. The wheel case
    changes *collision* cylinders, not visual meshes or reported joint values.
    These are stress sensitivities, not measured physical calibration values.
    """
    if profile not in HARDWARE_PROFILES:
        raise ValueError(f'unknown hardware profile: {profile}')
    changes = []
    if profile == 'nominal':
        return changes

    assignments = [
        ("./model/link[@name='base_scan']/sensor[@name='hls_lfcd_lds']/update_rate",
         10., 'Hz', 'laser_sensor_producer'),
        ("./model/link[@name='imu_link']/sensor[@name='tb3_imu']/update_rate",
         20., 'Hz', 'imu_sensor_producer'),
        ("./model/plugin[@name='turtlebot3_joint_state']/update_rate",
         20., 'Hz', 'wheel_joint_state_producer'),
    ]
    if profile == 'wheel_asymmetry':
        # Keep the commanded/encoded wheel diameter nominal while the two
        # actual contact surfaces have opposite one-percent radius errors.
        for side, scale in (('left', .99), ('right', 1.01)):
            path = (f"./model/link[@name='wheel_{side}_link']/"
                    f"collision[@name='wheel_{side}_collision']/geometry/cylinder/radius")
            element = tree.find(path)
            if element is None or abs(float(element.text) - .033) > 1e-12:
                raise ValueError('wheel stress requires known stock 0.033 m collision radii')
            assignments.append((path, .033 * scale, 'm', f'{side}_wheel_contact_radius'))
        diameter = tree.findtext("./model/plugin[@name='turtlebot3_diff_drive']/wheel_diameter")
        if (diameter is None or not isfinite(float(diameter))
                or abs(float(diameter) - .066) > 1e-12):
            raise ValueError('wheel stress requires unchanged nominal 0.066 m drive diameter')
    elif profile == 'actuator_lag':
        assignments.append(("./model/plugin[@name='turtlebot3_diff_drive']/max_wheel_acceleration",
                            .15, 'm/s^2', 'actual_joint_target_acceleration_limit'))

    # Validate the complete known layout before mutating any value.
    validated = []
    for path, after, units, effect in assignments:
        element = tree.find(path)
        if element is None or element.text is None:
            raise ValueError(f'hardware profile requires known model field: {path}')
        before = float(element.text)
        if not isfinite(before) or before <= 0:
            raise ValueError(f'hardware profile requires positive finite model field: {path}')
        # The recorded numeric value exactly matches the serialized SDF value.
        after = float(format(after, '.12g'))
        validated.append((element, path, before, after, units, effect))
    for element, path, before, after, units, effect in validated:
        element.text = format(after, '.12g')
        changes.append({'sdf_path': path, 'before': before, 'after': after,
                        'units': units, 'effect': effect, 'changed': before != after})
    return changes


def prepare(source_model, world, map_yaml, output, mode='baseline', *, physical_footprint=False,
            hardware_profile='nominal'):
    if mode not in ('baseline', 'ekf_independent'):
        raise ValueError('mode must be baseline or ekf_independent')
    if hardware_profile not in HARDWARE_PROFILES:
        raise ValueError(f'unknown hardware profile: {hardware_profile}')
    output.mkdir(parents=True, exist_ok=False)
    model_copy = output/'waffle_pi_fault.sdf'
    world_copy = output/'classroom_fault.world'
    map_copy = output/'classroom_fault.yaml'
    original = ET.parse(source_model)
    stock_bounds = collision_radius_bounds(original) if physical_footprint else None
    if physical_footprint:
        apply_physical_footprint_proxy(original)
    hardware_changes = apply_hardware_profile(original, hardware_profile)
    proxy_bounds = collision_radius_bounds(original) if physical_footprint else None
    if proxy_bounds and max(proxy_bounds.values()) > .22 + 1e-9:
        raise ValueError('hardware profile collision exceeds the physical 0.22 m proxy')
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
                'source_map_unchanged': True,
                'physical_footprint_proxy': physical_footprint,
                'hardware_profile': hardware_profile,
                'hardware_profile_edits': hardware_changes,
                'hardware_profile_calibration': ('stock_model' if hardware_profile == 'nominal'
                                                 else 'stress_envelope_not_physical_calibration'),
                'odometry_pipeline_unchanged': True,
                'nominal_drive_kinematics_unchanged': True}
    manifest['edits'].extend(f"hardware {change['effect']}: {change['before']} -> "
                            f"{change['after']} {change['units']}"
                            for change in hardware_changes if change['changed'])
    if physical_footprint:
        manifest['edits'].append('copied base collision -> centered cylinder radius 0.22 m, height 0.089 m')
        manifest.update(stock_collision_bounds_m=stock_bounds,
            proxy_collision_bounds_m=proxy_bounds,
            stock_max_collision_radius_m=max(stock_bounds.values()),
            proxy_max_collision_radius_m=max(proxy_bounds.values()),
            footprint_parity_limit='circular collision surrogate; stock mass, wheelbase, sensors and visuals retained')
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
    parser.add_argument('--physical-footprint', action='store_true',
                        help='Use a copied 0.22 m circular body collision proxy; keep stock model untouched')
    parser.add_argument('--hardware-profile', choices=HARDWARE_PROFILES, default='nominal',
                        help='Opt-in real sensor/contact/actuation stress in the copied model only')
    args = parser.parse_args()
    prepare(args.source_model, args.world, args.map_yaml, args.output, args.mode,
            physical_footprint=args.physical_footprint, hardware_profile=args.hardware_profile)
    print(args.output/'fixture_manifest.json')


if __name__ == '__main__':
    main()
