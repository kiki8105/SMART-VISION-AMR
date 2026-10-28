"""Condition the physical IMU gyro for independent wheel/IMU localization.

Only z angular velocity is fused by the EKF. Wheel velocity is used here
solely to establish that the robot is stationary before estimating gyro bias;
it is not copied into the IMU measurement. No estimate is published until
the initial stationary calibration has completed.
"""

from collections import deque
from copy import deepcopy
import math
from statistics import median, pvariance
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from smart_vision_amr.sensor_stamp_guard import SensorStampGuard


class StationaryGyroBias:
    def __init__(self, *, calibration_seconds=2.0, calibration_samples=25,
                 wheel_timeout=0.4, stationary_linear_limit=0.01,
                 stationary_angular_limit=0.04, stationary_gyro_limit=0.12,
                 max_gyro_rate=3.0, minimum_variance=0.02**2,
                 bias_adaptation=0.005):
        self.calibration_seconds = calibration_seconds
        self.calibration_samples = calibration_samples
        self.wheel_timeout = wheel_timeout
        self.stationary_linear_limit = stationary_linear_limit
        self.stationary_angular_limit = stationary_angular_limit
        self.stationary_gyro_limit = stationary_gyro_limit
        self.max_gyro_rate = max_gyro_rate
        self.minimum_variance = minimum_variance
        self.bias_adaptation = bias_adaptation
        self.wheel = None
        self.samples = deque(maxlen=200)
        self.sample_start = None
        self.bias = None
        self.stationary_variance = minimum_variance

    def update_wheel(self, linear_speed, angular_speed, now):
        if not all(math.isfinite(value) for value in (linear_speed, angular_speed, now)):
            self.wheel = None
            self._clear_samples()
            return
        self.wheel = (linear_speed, angular_speed, now)
        if not self._wheel_stationary():
            self._clear_samples()

    def _wheel_stationary(self):
        if self.wheel is None:
            return False
        linear_speed, angular_speed, _ = self.wheel
        return (abs(linear_speed) < self.stationary_linear_limit and
                abs(angular_speed) < self.stationary_angular_limit)

    def _clear_samples(self):
        self.samples.clear()
        self.sample_start = None

    def condition(self, raw_gyro_z, raw_variance, now):
        """Return (bias-corrected z rate, variance), or None for bad input."""
        if not math.isfinite(now) or not math.isfinite(raw_gyro_z):
            return None
        if abs(raw_gyro_z) > self.max_gyro_rate:
            return None
        if self.wheel is None or now - self.wheel[2] > self.wheel_timeout:
            self._clear_samples()
            return None
        if now < self.wheel[2]:
            return None

        stationary = self._wheel_stationary()
        if self.bias is None:
            # A wheel-still robot may have been picked up and rotated by hand;
            # reject obvious rotation rather than learning it as gyro bias.
            if not stationary or abs(raw_gyro_z) > self.stationary_gyro_limit:
                self._clear_samples()
                return None
            if self.sample_start is None:
                self.sample_start = now
            self.samples.append(raw_gyro_z)
            if (len(self.samples) < self.calibration_samples or
                    now - self.sample_start < self.calibration_seconds):
                return None
            self.bias = median(self.samples)
            self.stationary_variance = max(
                self.minimum_variance, pvariance(self.samples))
            self._clear_samples()
        elif stationary and abs(raw_gyro_z - self.bias) <= self.stationary_gyro_limit:
            # Very slow adaptation compensates thermal bias changes only while
            # the independent wheel measurement says the robot is still.
            self.bias += self.bias_adaptation * (raw_gyro_z - self.bias)

        supplied_variance = (raw_variance if math.isfinite(raw_variance) and
                             raw_variance > 0.0 else 0.0)
        return (raw_gyro_z - self.bias,
                max(self.minimum_variance, self.stationary_variance,
                    supplied_variance))


class ImuEkfConditioner(Node):
    def __init__(self):
        super().__init__('imu_ekf_conditioner')
        self.declare_parameter('calibration_seconds', 2.0)
        self.declare_parameter('calibration_samples', 25)
        self.declare_parameter('wheel_timeout', 0.4)
        self.declare_parameter('minimum_gyro_z_variance', 0.02**2)
        self.declare_parameter('max_stamp_age_sec', 1.0)
        self.declare_parameter('future_stamp_tolerance_sec', 0.25)
        self.estimator = StationaryGyroBias(
            calibration_seconds=float(self.get_parameter('calibration_seconds').value),
            calibration_samples=int(self.get_parameter('calibration_samples').value),
            wheel_timeout=float(self.get_parameter('wheel_timeout').value),
            minimum_variance=float(self.get_parameter('minimum_gyro_z_variance').value))
        self.stamp_guard = SensorStampGuard(
            max_age_sec=float(self.get_parameter('max_stamp_age_sec').value),
            future_tolerance_sec=float(self.get_parameter('future_stamp_tolerance_sec').value))
        self.publisher = self.create_publisher(Imu, '/imu/ekf', 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu,
                                 qos_profile_sensor_data)
        self.get_logger().info('Keep the robot stationary for gyro calibration before driving')

    def on_wheel(self, message):
        velocity = message.twist.twist
        self.estimator.update_wheel(velocity.linear.x, velocity.angular.z,
                                    time.monotonic())

    def on_imu(self, message):
        # ROS Imu uses covariance[0] == -1 to mark this field unavailable.
        if message.angular_velocity_covariance[0] == -1.0:
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if not self.stamp_guard.accept(stamp_ns, now_ns):
            self.get_logger().warning(
                f'IMU timestamp rejected (age={(now_ns-stamp_ns)/1e9:.2f}s); '
                'check Pi/PC clock sync and sensor order',
                throttle_duration_sec=5.0)
            return
        prior_bias = self.estimator.bias
        result = self.estimator.condition(
            message.angular_velocity.z, message.angular_velocity_covariance[8],
            time.monotonic())
        if result is None:
            return
        if prior_bias is None:
            self.get_logger().info(
                f'Gyro bias calibrated: {self.estimator.bias:.5f} rad/s')
        corrected_z, variance = result
        conditioned = deepcopy(message)
        conditioned.angular_velocity.z = corrected_z
        conditioned.angular_velocity_covariance[8] = variance
        self.publisher.publish(conditioned)


def main(args=None):
    rclpy.init(args=args)
    node = ImuEkfConditioner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
