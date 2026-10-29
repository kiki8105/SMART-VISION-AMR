#!/usr/bin/env python3
"""Report wheel geometry/noise candidates from an independent reference trial.

CSV columns: time,left_rad,right_rad,reference_x,reference_y,reference_yaw
and optionally imu_wz. SI units; wheel angles must be continuous. Reference
positions/headings must come from surveyed motion, external tracking or a
simulator's ground truth, not the wheel/IMU EKF being calibrated.

Examples:
  python3 scripts/calibrate_wheel_imu_reference.py --csv trial.csv
  python3 scripts/calibrate_wheel_imu_reference.py --bag trial_bag \
      --reference-topic /ground_truth/odom --json-output trial_report.json

This utility never changes live parameters, firmware or deployed YAML.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


REQUIRED = ('time', 'left_rad', 'right_rad', 'reference_x',
            'reference_y', 'reference_yaw')


def read_csv(path):
    with Path(path).open(newline='') as handle:
        reader = csv.DictReader(handle)
        if not set(REQUIRED).issubset(reader.fieldnames or []):
            raise ValueError('CSV requires columns: '+','.join(REQUIRED))
        return [{key: float(value) for key, value in row.items()
                 if value is not None and value.strip()} for row in reader]


def read_bag(path, reference_topic, wheel_topic, imu_topic):
    """Resample independent reference poses onto wheel sample timestamps."""
    if reference_topic in ('/odom', '/wheel/odom', '/wheel/odom_raw', wheel_topic):
        raise ValueError('Reference must be independent of wheel/IMU odometry')
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    if reference_topic not in types or wheel_topic not in types:
        raise ValueError('Bag is missing reference or wheel topic')
    wheels, reference, imu = [], [], []
    while reader.has_next():
        topic, data, _record_stamp = reader.read_next()
        if topic not in (reference_topic, wheel_topic, imu_topic):
            continue
        message = deserialize_message(data, types[topic])
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if topic == wheel_topic:
            try:
                wheels.append((stamp,
                    message.position[message.name.index('wheel_left_joint')],
                    message.position[message.name.index('wheel_right_joint')]))
            except (ValueError, IndexError):
                continue
        elif topic == reference_topic:
            pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            reference.append((stamp, pose.position.x, pose.position.y, yaw))
        else:
            imu.append((stamp, message.angular_velocity.z))
    if len(reference) < 2 or len(wheels) < 3:
        raise ValueError('Bag has insufficient reference/wheel measurements')
    wheel = np.asarray(sorted(wheels), dtype=float)
    ref = np.asarray(sorted(reference), dtype=float)
    ref[:, 3] = np.unwrap(ref[:, 3])
    wheel = wheel[(wheel[:, 0] >= ref[0, 0]) & (wheel[:, 0] <= ref[-1, 0])]
    poses = [np.interp(wheel[:, 0], ref[:, 0], ref[:, index]) for index in (1, 2, 3)]
    gyros = None
    if imu:
        values = np.asarray(sorted(imu), dtype=float)
        gyros = np.interp(wheel[:, 0], values[:, 0], values[:, 1], left=np.nan, right=np.nan)
    return [dict(zip(REQUIRED, (row[0], row[1], row[2], poses[0][i], poses[1][i], poses[2][i])),
                 **({} if gyros is None else {'imu_wz': gyros[i]}))
            for i, row in enumerate(wheel)]


def calibrate(rows):
    if len(rows) < 6:
        raise ValueError('At least six synchronized samples are required')
    data = np.array([[row[key] for key in REQUIRED] for row in rows], dtype=float)
    if not np.all(np.isfinite(data)) or np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError('Input must be finite and timestamps strictly increasing')
    dt = np.diff(data[:, 0])
    left, right = np.diff(data[:, 1]), np.diff(data[:, 2])
    yaw = np.unwrap(data[:, 5])
    dyaw = np.diff(yaw)
    midpoint = (yaw[:-1]+yaw[1:])/2
    dx, dy = np.diff(data[:, 3]), np.diff(data[:, 4])
    # Midpoint integration is exact for the arc chord after this correction.
    chord = dx*np.cos(midpoint)+dy*np.sin(midpoint)
    sinc = np.sinc(dyaw/(2*np.pi))
    reference_distance = chord/np.maximum(sinc, 0.9)
    average, difference = (left+right)/2, right-left
    valid = ((dt > 0.0) & (dt <= 1.0)
             & (np.maximum(abs(left), abs(right))/dt <= 30.0)
             & (np.hypot(dx, dy)/dt <= 1.0) & (abs(dyaw)/dt <= 3.0))
    moving = valid & (abs(average) > 0.01) & (abs(reference_distance) > 0.0001)
    if np.count_nonzero(moving) < 3:
        raise ValueError('Need at least three translational intervals for wheel radius')
    radius = float(np.dot(average[moving], reference_distance[moving])/
                   np.dot(average[moving], average[moving]))
    if not 0.005 < radius < 0.3:
        raise ValueError('Estimated wheel radius is implausible; check reference units/signs')
    turning = valid & (abs(difference) > 0.02) & (abs(dyaw) > 0.002)
    separation = None
    notes = ['Candidates only: inspect the independent reference and validate on a new trial.']
    if np.count_nonzero(turning) >= 3:
        slope = float(np.dot(difference[turning], dyaw[turning])/
                      np.dot(difference[turning], difference[turning]))
        if slope > 0.0 and 0.03 < radius/slope < 2.0:
            separation = radius/slope
        else:
            notes.append('Separation rejected: implausible turn geometry or opposite wheel signs.')
    else:
        notes.append('No separation estimate: record at least three turning intervals.')
    residual_velocity = (radius*average[moving]-reference_distance[moving])/dt[moving]
    noise = {'forward_velocity_variance': float(np.var(residual_velocity)),
             'forward_velocity_rmse_mps': float(np.sqrt(np.mean(residual_velocity**2)))}
    gyro = np.array([row.get('imu_wz', np.nan) for row in rows], dtype=float)
    finite = valid & np.isfinite(gyro[1:])
    if np.count_nonzero(finite) >= 3:
        residual = gyro[1:][finite]-dyaw[finite]/dt[finite]
        noise['gyro_z_residual_variance'] = float(np.var(residual))
        still = finite & (abs(left)/dt < 0.02) & (abs(right)/dt < 0.02) & (abs(dyaw)/dt < 0.03)
        noise['stationary_gyro_bias_radps'] = (
            float(np.median(gyro[1:][still])) if np.count_nonzero(still) >= 3 else None)
    return {'source': 'independent_reference_trial', 'samples': len(rows),
            'accepted_intervals': int(np.count_nonzero(valid)),
            'translation_intervals': int(np.count_nonzero(moving)),
            'turning_intervals': int(np.count_nonzero(turning)),
            'wheel_radius_m': radius, 'wheel_separation_m': separation,
            'noise': noise, 'notes': notes}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--csv', type=Path)
    source.add_argument('--bag', type=Path)
    parser.add_argument('--reference-topic', help='Independent Odometry or Pose[WithCovariance]Stamped topic')
    parser.add_argument('--wheel-topic', default='/joint_states')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument('--json-output', type=Path)
    args = parser.parse_args()
    if args.bag and not args.reference_topic:
        parser.error('--bag requires --reference-topic (independent ground truth)')
    try:
        rows = read_csv(args.csv) if args.csv else read_bag(
            args.bag, args.reference_topic, args.wheel_topic, args.imu_topic)
        report = calibrate(rows)
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(2, f'Calibration rejected: {exc}\n')
    text = json.dumps(report, indent=2, allow_nan=False)
    print(text)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(text+'\n')


if __name__ == '__main__':
    main()
