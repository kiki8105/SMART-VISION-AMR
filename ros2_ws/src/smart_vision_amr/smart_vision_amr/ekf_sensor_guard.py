"""Latch a hardware-only motion stop when an EKF input stream is lost.

The guard uses both receive time and ROS header freshness. It starts with a
latched estop, because the EKF can publish wheel-only odom before the IMU is
calibrated. It arms only after all required streams have been healthy for a
continuous period and the operator explicitly resets the stop. After a trip,
it re-sends estop until the motion mux acknowledges it. Recovery of sensor
data alone never resets the mux.
"""

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from smart_vision_amr.sensor_stamp_guard import SensorStampGuard


def publisher_issues(get_publishers, expected_publishers):
    """Report any missing, duplicate, or wrong-owner ROS publishers."""
    issues = []
    for topic, expected_node in expected_publishers.items():
        publishers = get_publishers(topic)
        if len(publishers) != 1:
            issues.append(f'{topic} publishers={len(publishers)} (expected 1)')
        elif (expected_node is not None and
              (publishers[0].node_name != expected_node or
               publishers[0].node_namespace != '/')):
            issues.append(f'{topic} publisher is not /{expected_node}')
    return issues


def odometry_issues(message, *, max_position_variance=0.0,
                    max_yaw_variance=0.0, max_linear_velocity_variance=0.25,
                    max_angular_velocity_variance=1.0, base_frame='base_footprint'):
    """Validate the filter's actual output, not merely its ROS publisher."""
    pose, twist = message.pose.pose, message.twist.twist
    q = pose.orientation
    values = (pose.position.x, pose.position.y, pose.position.z,
              q.x, q.y, q.z, q.w, twist.linear.x, twist.linear.y,
              twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)
    if not all(math.isfinite(value) for value in values):
        return ['nonfinite odometry value']
    if not 0.95 <= sum(value*value for value in (q.x, q.y, q.z, q.w)) <= 1.05:
        return ['invalid odometry quaternion']
    if message.header.frame_id != 'odom' or message.child_frame_id != base_frame:
        return ['unexpected odometry frames']
    for covariance in (message.pose.covariance, message.twist.covariance):
        if (not all(math.isfinite(value) for value in covariance)
                or any(covariance[index] < 0.0 for index in (0, 7, 35))):
            return ['invalid odometry covariance']
    # Local absolute x/y/yaw are unobservable from velocity + gyro alone:
    # their covariance legitimately grows even while AMCL keeps map pose
    # accurate. Only opt into a local-pose magnitude limit explicitly.
    if (max_position_variance > 0.0 and
            max(message.pose.covariance[0], message.pose.covariance[7]) > max_position_variance):
        return ['odometry position covariance too large']
    if max_yaw_variance > 0.0 and message.pose.covariance[35] > max_yaw_variance:
        return ['odometry yaw covariance too large']
    if max(message.twist.covariance[0], message.twist.covariance[7]) > max_linear_velocity_variance:
        return ['odometry linear velocity covariance too large']
    if message.twist.covariance[35] > max_angular_velocity_variance:
        return ['odometry angular velocity covariance too large']
    return []


class EkfSensorGuardCore:
    REQUIRED = ('wheel', 'imu', 'scan', 'odom', 'odom_tf')

    def __init__(self, arm_seconds=2.0, stale_seconds=1.0,
                 max_header_age_seconds=1.0, future_tolerance_seconds=0.25):
        if arm_seconds <= 0.0 or stale_seconds <= 0.0:
            raise ValueError('Guard durations must be positive')
        self.arm_seconds = arm_seconds
        self.stale_seconds = stale_seconds
        self.stamps = {name: SensorStampGuard(max_header_age_seconds,
                                             future_tolerance_seconds)
                       for name in self.REQUIRED}
        self.last_seen = {name: None for name in self.REQUIRED}
        self.value_issues = {}
        self.graph_issues = ['ROS publisher graph not yet checked']
        self.graph_checked_at = None
        self.healthy_since = None
        self.armed = False
        self.latched = True
        self.trip_reason = 'EKF inputs not yet validated'

    def observe(self, name, stamp_ns, wall_ns, monotonic_seconds):
        if name not in self.stamps:
            raise ValueError(f'Unknown sensor stream: {name}')
        if not self.stamps[name].accept(stamp_ns, wall_ns):
            return False
        self.last_seen[name] = monotonic_seconds
        self.value_issues.pop(name, None)
        return True

    def reject(self, name, issues):
        self.last_seen[name] = None
        self.value_issues[name] = ', '.join(issues)

    def missing(self, monotonic_seconds):
        missing = [f'{name}: {self.value_issues[name]}' if name in self.value_issues else name
                   for name in self.REQUIRED
                   if self.last_seen[name] is None or
                   monotonic_seconds - self.last_seen[name] >= self.stale_seconds]
        if (self.graph_checked_at is None or
                monotonic_seconds - self.graph_checked_at >= self.stale_seconds):
            missing.append('ROS publisher graph stale')
        else:
            missing.extend(self.graph_issues)
        return missing

    def update_graph(self, issues, monotonic_seconds):
        self.graph_issues = list(issues)
        self.graph_checked_at = monotonic_seconds

    def tick(self, monotonic_seconds):
        """Return a trip reason once; never auto-clear a latched fault."""
        missing = self.missing(monotonic_seconds)
        if missing:
            self.healthy_since = None
        elif self.healthy_since is None:
            self.healthy_since = monotonic_seconds

        if self.latched:
            return None
        if not self.armed:
            if (self.healthy_since is not None and
                    monotonic_seconds - self.healthy_since >= self.arm_seconds):
                self.armed = True
            return None
        if missing:
            self.armed = False
            self.latched = True
            self.trip_reason = ', '.join(missing) + ' sensor stream stale'
            return self.trip_reason
        return None

    def operator_reset(self, monotonic_seconds):
        """Allow re-arming only after an external reset and stable inputs."""
        if not self.latched or self.missing(monotonic_seconds):
            return False
        if (self.healthy_since is None or
                monotonic_seconds - self.healthy_since < self.arm_seconds):
            return False
        self.latched = False
        self.armed = True
        self.trip_reason = ''
        return True


class EkfSensorGuard(Node):
    SAFETY_SOURCE = 'ekf_sensor_safety'
    EXPECTED_PUBLISHERS = {
        '/odom': 'ekf_hardware',
        '/joint_states': None,
        '/imu': None,
        '/scan': None,
        '/wheel/odom_raw': None,
        '/wheel/odom': 'wheel_joint_odometry',
        '/imu/ekf': 'imu_ekf_conditioner',
    }

    def __init__(self):
        super().__init__('ekf_sensor_guard')
        self.declare_parameter('arm_seconds', 2.0)
        self.declare_parameter('stale_seconds', 1.0)
        self.declare_parameter('max_position_variance', 0.0)
        self.declare_parameter('max_yaw_variance', 0.0)
        self.declare_parameter('max_linear_velocity_variance', 0.25)
        self.declare_parameter('max_angular_velocity_variance', 1.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.core = EkfSensorGuardCore(
            arm_seconds=float(self.get_parameter('arm_seconds').value),
            stale_seconds=float(self.get_parameter('stale_seconds').value))
        self.acknowledged = False
        self.last_estop_at = None
        self.safety_seq = 0
        self.pending_reset = None
        self.last_reset_seq = {}
        self.last_owner_check_at = None
        self.control_pub = self.create_publisher(
            String, '/smart_vision_amr/manual_control', 10)
        self.create_subscription(Odometry, '/wheel/odom',
                                 lambda msg: self.on_sensor('wheel', msg),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/ekf',
                                 lambda msg: self.on_sensor('imu', msg),
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 lambda msg: self.on_sensor('scan', msg),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 qos_profile_sensor_data)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/smart_vision_amr/motion_status',
                                 self.on_motion_status, durable)
        self.create_subscription(String, '/smart_vision_amr/manual_control',
                                 self.on_control, 10)
        self.create_timer(0.1, self.on_tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._publish_estop(time.monotonic())
        self.get_logger().warning(
            'EKF motion held until wheel, IMU, LiDAR, /odom output, odom TF and sole EKF /odom '
            'publisher are healthy for 2 s and the operator explicitly resets')

    def on_sensor(self, name, message):
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self.core.observe(name, stamp_ns, self.get_clock().now().nanoseconds,
                          time.monotonic())

    def on_odom(self, message):
        issues = odometry_issues(
            message, base_frame=self.base_frame,
            max_position_variance=float(self.get_parameter('max_position_variance').value),
            max_yaw_variance=float(self.get_parameter('max_yaw_variance').value),
            max_linear_velocity_variance=float(self.get_parameter('max_linear_velocity_variance').value),
            max_angular_velocity_variance=float(self.get_parameter('max_angular_velocity_variance').value))
        if issues:
            self.core.reject('odom', issues)
        else:
            self.on_sensor('odom', message)

    def update_tf(self, now):
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', self.base_frame, rclpy.time.Time())
            t, q = transform.transform.translation, transform.transform.rotation
            values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            if (not all(math.isfinite(value) for value in values)
                    or not 0.95 <= sum(value*value for value in (q.x, q.y, q.z, q.w)) <= 1.05):
                self.core.reject('odom_tf', ['invalid TF values'])
                return
            stamp = transform.header.stamp
            self.core.observe('odom_tf', stamp.sec*1_000_000_000+stamp.nanosec,
                              self.get_clock().now().nanoseconds, now)
        except TransformException:
            self.core.reject('odom_tf', ['odom to base TF unavailable'])

    def on_tick(self):
        now = time.monotonic()
        if (self.last_owner_check_at is None or
                now - self.last_owner_check_at >= 0.5):
            self.last_owner_check_at = now
            self.update_tf(now)
            try:
                issues = publisher_issues(self.get_publishers_info_by_topic,
                                          self.EXPECTED_PUBLISHERS)
            except Exception as exc:
                self.get_logger().warning(f'Cannot inspect ROS publishers: {exc}')
                issues = [f'ROS publisher graph query failed: {exc}']
            self.core.update_graph(issues, now)
        reason = self.core.tick(now)
        if reason is not None:
            self.get_logger().error(f'EKF input lost: {reason}; emergency stop latched')
        # Keep a slow safety heartbeat even after acknowledgement. If a web
        # mux restarts and its transient unlocked status is lost, a one-shot
        # estop would otherwise leave it unlocked indefinitely.
        resend_after = 1.0 if self.acknowledged else 0.2
        if (self.core.latched and
                (self.last_estop_at is None or now - self.last_estop_at >= resend_after)):
            self._publish_estop(now)

    def _publish_estop(self, now):
        # The mux remembers safety-source sequences across guard restarts.
        self.safety_seq = max(self.safety_seq + 1, time.monotonic_ns())
        message = String()
        message.data = json.dumps({
            'source_id': self.SAFETY_SOURCE,
            'seq': self.safety_seq,
            'sent_at_ns': self.get_clock().now().nanoseconds,
            'mode': 'estop', 'linear': 0.0, 'angular': 0.0,
        })
        self.control_pub.publish(message)
        self.last_estop_at = now

    def on_control(self, message):
        """Record an explicit non-safety reset intent before mux state clears."""
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict) or payload.get('mode') != 'reset':
            return
        source = payload.get('source_id')
        seq = payload.get('seq')
        stamp_ns = payload.get('sent_at_ns')
        now_ns = self.get_clock().now().nanoseconds
        if (not isinstance(source, str) or not source or
                source in ('ekf_sensor_safety', 'path_follower_recovery_safety',
                           'localization_safety') or
                type(seq) is not int or seq <= self.last_reset_seq.get(source, -1) or
                type(stamp_ns) is not int or abs(now_ns - stamp_ns) > 1_000_000_000):
            return
        self.last_reset_seq[source] = seq
        self.pending_reset = (source, time.monotonic())

    def on_motion_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(status, dict) or not self.core.latched:
            return
        if status.get('estop_latched') is True:
            self.acknowledged = True
        elif status.get('estop_latched') is False and self.acknowledged:
            # A mux/web restart also produces true->false without a reset.
            # Require a fresh, matching web reset packet to distinguish it.
            self.acknowledged = False
            now = time.monotonic()
            reset = self.pending_reset
            self.pending_reset = None
            explicit_reset = (reset is not None and reset[0] == status.get('source_id')
                              and now - reset[1] <= 1.0)
            if explicit_reset and self.core.operator_reset(now):
                self.get_logger().info('Operator reset accepted; EKF sensor guard re-armed')
            else:
                self.get_logger().error(
                    'Motion reset rejected: no fresh explicit operator reset '
                    'or EKF inputs not stable')
                self._publish_estop(now)


def main(args=None):
    rclpy.init(args=args)
    node = EkfSensorGuard()
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
