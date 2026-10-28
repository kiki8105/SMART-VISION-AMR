"""Exercise stationary gyro calibration without a robot or ROS graph."""

import pytest

from smart_vision_amr.imu_ekf_conditioner import StationaryGyroBias


def test_bias_is_learned_only_while_wheels_are_still():
    estimator = StationaryGyroBias(calibration_seconds=0.25,
                                   calibration_samples=4)
    for index in range(3):
        now = index * 0.1
        estimator.update_wheel(0.0, 0.0, now)
        assert estimator.condition(0.04, 0.0, now) is None

    estimator.update_wheel(0.08, 0.0, 0.3)
    assert estimator.condition(0.04, 0.0, 0.3) is None
    assert estimator.bias is None

    for index in range(4, 8):
        now = index * 0.1
        estimator.update_wheel(0.0, 0.0, now)
        result = estimator.condition(0.04, 0.0, now)
    assert estimator.bias == pytest.approx(0.04)
    assert result == pytest.approx((0.0, 0.02**2))


def test_moving_gyro_is_corrected_and_stale_or_bad_samples_are_dropped():
    estimator = StationaryGyroBias(calibration_seconds=0.2,
                                   calibration_samples=3)
    for index in range(3):
        now = index * 0.1
        estimator.update_wheel(0.0, 0.0, now)
        estimator.condition(0.03, 0.0, now)
    assert estimator.bias == pytest.approx(0.03)

    estimator.update_wheel(0.08, 0.2, 0.3)
    assert estimator.condition(0.33, 0.01, 0.3) == pytest.approx((0.30, 0.01))
    assert estimator.bias == pytest.approx(0.03)
    assert estimator.condition(5.0, 0.0, 0.31) is None
    assert estimator.condition(0.0, 0.0, 0.8) is None


def test_manual_rotation_does_not_become_initial_bias():
    estimator = StationaryGyroBias(calibration_seconds=0.1,
                                   calibration_samples=2)
    estimator.update_wheel(0.0, 0.0, 0.0)
    assert estimator.condition(0.6, 0.0, 0.0) is None
    estimator.update_wheel(0.0, 0.0, 0.1)
    assert estimator.condition(0.6, 0.0, 0.1) is None
    assert estimator.bias is None
