"""Integrate independent wheel joint angles into odometry without publishing TF.

This is an independent wheel measurement for the EKF comparison. Gazebo's
stock diff-drive /odom can use world pose, so feeding it to the independent
filter would make the comparison meaningless. On hardware the same integrator
uses encoder JointState angles, without consuming the vendor's fused odom.
"""

import math
from pathlib import Path

import yaml

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from smart_vision_amr.sensor_stamp_guard import SensorStampGuard


def load_stock_wheel_geometry(path):
    """Read TurtleBot3 wheel dimensions without losing repeated /** YAML keys.

    Vendor parameter files repeat the /** root for several node sections.
    ``safe_load`` silently keeps just the last block, which may be the
    controller rather than turtlebot3_node. Composed YAML retains each block;
    ambiguous or incomplete geometry is rejected instead of guessed.
    """
    tree = yaml.compose(Path(path).read_text())
    found = []

    def visit(node, lineage=()):
        if not isinstance(node, yaml.MappingNode):
            return
        for key, value in node.value:
            if not isinstance(key, yaml.ScalarNode):
                continue
            names = lineage+(key.value,)
            if key.value == 'wheels' and any(
                    name.rsplit('/', 1)[-1] == 'turtlebot3_node' for name in lineage):
                if not isinstance(value, yaml.MappingNode):
                    raise ValueError('Invalid wheels parameter block')
                dimensions = {}
                for item, scalar in value.value:
                    if item.value in ('radius', 'separation'):
                        if item.value in dimensions or not isinstance(scalar, yaml.ScalarNode):
                            raise ValueError('Ambiguous stock wheel geometry')
                        dimensions[item.value] = float(scalar.value)
                found.append(dimensions)
            visit(value, names)

    visit(tree)
    if len(found) != 1 or set(found[0]) != {'radius', 'separation'}:
        raise ValueError('Expected exactly one stock turtlebot3_node wheel geometry')
    if not all(math.isfinite(value) and value > 0.0 for value in found[0].values()):
        raise ValueError('Stock wheel dimensions must be finite and positive')
    return {'wheel_radius': found[0]['radius'], 'wheel_separation': found[0]['separation']}


class WheelIntegrator:
    def __init__(self, radius=0.033, separation=0.287, max_wheel_speed=30.0):
        if not all(math.isfinite(value) and value > 0.0
                   for value in (radius, separation, max_wheel_speed)):
            raise ValueError('Wheel radius, separation, and speed limit must be positive')
        self.radius = radius
        self.separation = separation
        self.max_wheel_speed = max_wheel_speed
        self.previous = None
        self.x = self.y = self.yaw = 0.0

    def update(self, stamp_ns, left_rad, right_rad):
        if stamp_ns <= 0 or not all(map(math.isfinite, (left_rad, right_rad))):
            return None
        current = stamp_ns, left_rad, right_rad
        if self.previous is None:
            self.previous = current
            return None
        last_ns, last_left, last_right = self.previous
        dt = (stamp_ns - last_ns) / 1e9
        self.previous = current
        # A Gazebo reset, delayed message, or joint jump must not teleport the
        # filter. Rebaseline and wait for another valid wheel sample instead.
        if dt <= 0 or dt > 0.5:
            return None
        delta_left = left_rad - last_left
        delta_right = right_rad - last_right
        if max(abs(delta_left), abs(delta_right)) / dt > self.max_wheel_speed:
            return None
        left_m = delta_left * self.radius
        right_m = delta_right * self.radius
        distance = (left_m + right_m) / 2.0
        delta_yaw = (right_m - left_m) / self.separation
        self.x += distance * math.cos(self.yaw + delta_yaw / 2.0)
        self.y += distance * math.sin(self.yaw + delta_yaw / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw),
                              math.cos(self.yaw + delta_yaw))
        return self.x, self.y, self.yaw, distance / dt, delta_yaw / dt


class WheelJointOdometry(Node):
    def __init__(self):
        super().__init__('wheel_joint_odometry')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.287)
        self.declare_parameter('max_wheel_speed', 30.0)
        self.declare_parameter('reject_stale_stamps', False)
        self.declare_parameter('max_stamp_age_sec', 1.0)
        self.declare_parameter('future_stamp_tolerance_sec', 0.25)
        # Variances, rather than standard deviations, in m², rad² and
        # (m/s)² / (rad/s)². These defaults are conservative starting values;
        # replace them only after a physical calibration trial.
        defaults = {'pose_xy_variance': 0.05**2,
                    'pose_yaw_variance': 0.08**2,
                    'forward_velocity_variance': 0.02**2,
                    'lateral_velocity_variance': 0.005**2,
                    'yaw_rate_variance': 0.08**2}
        self.variances = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite positive variance')
            self.variances[name] = value
        self.integrator = WheelIntegrator(
            radius=float(self.get_parameter('wheel_radius').value),
            separation=float(self.get_parameter('wheel_separation').value),
            max_wheel_speed=float(self.get_parameter('max_wheel_speed').value))
        self.reject_stale_stamps = bool(self.get_parameter('reject_stale_stamps').value)
        self.stamp_guard = SensorStampGuard(
            max_age_sec=float(self.get_parameter('max_stamp_age_sec').value),
            future_tolerance_sec=float(self.get_parameter('future_stamp_tolerance_sec').value))
        self.publisher = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.on_joints, 10)

    def on_joints(self, message):
        try:
            left = message.position[message.name.index('wheel_left_joint')]
            right = message.position[message.name.index('wheel_right_joint')]
        except (ValueError, IndexError):
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.reject_stale_stamps:
            now_ns = self.get_clock().now().nanoseconds
            if not self.stamp_guard.accept(stamp_ns, now_ns):
                self.get_logger().warning(
                    f'JointState timestamp rejected (age={(now_ns-stamp_ns)/1e9:.2f}s); '
                    'check Pi/PC clock sync and sensor order',
                    throttle_duration_sec=5.0)
                return
        state = self.integrator.update(stamp_ns, left, right)
        if state is None:
            return
        x, y, yaw, vx, wz = state
        odom = Odometry()
        odom.header = message.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        # The filter only fuses wheel longitudinal velocity and the zero
        # lateral-velocity constraint; EKF yaw is integrated from IMU gyro z.
        for index, variance in ((0, self.variances['pose_xy_variance']),
                                (1, self.variances['pose_xy_variance']),
                                (5, self.variances['pose_yaw_variance'])):
            odom.pose.covariance[index * 6 + index] = variance
        for index, variance in ((0, self.variances['forward_velocity_variance']),
                                (1, self.variances['lateral_velocity_variance']),
                                (5, self.variances['yaw_rate_variance'])):
            odom.twist.covariance[index * 6 + index] = variance
        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointOdometry()
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
