"""Integrate Gazebo wheel joint angles into odometry without publishing TF.

This is an independent wheel measurement for the EKF comparison. Gazebo's
stock diff-drive /odom can use world pose, so feeding it to the independent
filter would make the comparison meaningless.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelIntegrator:
    def __init__(self, radius=0.033, separation=0.287, max_wheel_speed=30.0):
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
        self.integrator = WheelIntegrator(
            radius=float(self.get_parameter('wheel_radius').value),
            separation=float(self.get_parameter('wheel_separation').value))
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
        for index, variance in ((0, 0.05**2), (1, 0.05**2), (5, 0.08**2)):
            odom.pose.covariance[index * 6 + index] = variance
        for index, variance in ((0, 0.02**2), (1, 0.005**2), (5, 0.08**2)):
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
