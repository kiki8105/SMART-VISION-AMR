"""Known geometry recovery from an independent synthetic reference."""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[4]/'scripts/calibrate_wheel_imu_reference.py'
spec = importlib.util.spec_from_file_location('calibrate_reference', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def trial():
    radius, separation = 0.034, 0.292
    x = y = yaw = left = right = 0.0
    rows = []
    for i in range(100):
        rows.append(dict(time=i*0.1+1.0, left_rad=left, right_rad=right,
                         reference_x=x, reference_y=y, reference_yaw=yaw,
                         imu_wz=0.0 if i < 10 else 0.204))
        dl, dr = (0.0, 0.0) if i < 9 else (0.2, 0.375)
        ds, dtheta = (dl+dr)*radius/2, (dr-dl)*radius/separation
        chord = ds if abs(dtheta) < 1e-8 else ds*math.sin(dtheta/2)/(dtheta/2)
        x += chord*math.cos(yaw+dtheta/2)
        y += chord*math.sin(yaw+dtheta/2)
        yaw += dtheta
        left += dl
        right += dr
    return rows


def test_calibration_recovers_measured_geometry_without_rewriting_config():
    report = module.calibrate(trial())
    assert report['wheel_radius_m'] == pytest.approx(0.034, rel=1e-6)
    assert report['wheel_separation_m'] == pytest.approx(0.292, rel=1e-6)
    assert report['translation_intervals'] > 50
    assert report['noise']['forward_velocity_rmse_mps'] < 1e-8
    assert report['source'] == 'independent_reference_trial'


def test_calibration_rejects_nonmonotonic_and_stationary_trials():
    rows = trial()
    rows[4]['time'] = rows[3]['time']
    with pytest.raises(ValueError, match='strictly increasing'):
        module.calibrate(rows)
    with pytest.raises(ValueError, match='translational'):
        module.calibrate(trial()[:8])


def test_bag_reference_cannot_reuse_the_filter_being_calibrated(tmp_path):
    with pytest.raises(ValueError, match='independent'):
        module.read_bag(tmp_path, '/odom', '/joint_states', '/imu')
