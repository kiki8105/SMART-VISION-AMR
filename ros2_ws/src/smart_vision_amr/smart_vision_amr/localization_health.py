"""Continuously compare localization confidence with independent scan geometry.

The node never publishes an initial pose or a velocity command. Critical
faults use the common motion mux stop, with an explicit operator reset required
after recovery. Scan projection uses the measurement-time transform.
"""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from smart_vision_amr.ekf_sensor_guard import odometry_issues
from smart_vision_amr.localization_geometry import (
    occupied_distance, scan_map_agreement, scan_points,
)
from smart_vision_amr.localization_health_math import (
    LocalizationHealthCore, angle_difference, quaternion_yaw,
)
from smart_vision_amr.sensor_stamp_guard import SensorStampGuard


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def stamp_rejection_reason(guard, stamp, now_ns):
    if stamp <= 0:
        return 'zero_or_invalid_header'
    if now_ns <= 0:
        return 'clock_not_ready'
    if guard.last_stamp_ns is not None and stamp <= guard.last_stamp_ns:
        return 'duplicate_or_reordered_header'
    age = now_ns-stamp
    if age > guard.max_age_ns:
        return 'stale_header'
    if age < -guard.future_tolerance_ns:
        return 'future_header_or_clock_skew'
    return 'timestamp_rejected'


class LocalizationHealth(Node):
    SAFETY_SOURCE = 'localization_safety'

    def __init__(self):
        super().__init__('localization_health')
        defaults = {
            'stable_seconds': 2.0, 'fault_seconds': 0.5,
            'agreement_fault_seconds': 3.0, 'sensor_timeout': 0.5,
            'tf_timeout': 1.0, 'future_tolerance': 1.5,
            'position_warning': 0.35, 'position_critical': 0.8,
            'yaw_warning': 0.5, 'yaw_critical': 1.0,
            'agreement_warning': 0.55, 'agreement_critical': 0.20,
            'amcl_moving_timeout': 4.0, 'motion_dt_limit': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.core = LocalizationHealthCore(**{
            name: float(self.get_parameter(name).value) for name in defaults})
        for name, value in {'base_frame': 'base_footprint', 'map_frame': 'map',
                            'odom_frame': 'odom', 'agreement_tolerance': 0.18,
                            'jump_distance_m': 0.65, 'jump_yaw_rad': 0.8,
                            'nomotion_update_interval': 2.0,
                            'nomotion_update_trigger_age': 1.0,
                            'nomotion_request_timeout': 1.0}.items():
            self.declare_parameter(name, value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_metadata = self.map_distance = None
        self.scan = self.odom = None
        self.scan_at = self.odom_at = self.amcl_at = None
        self.scan_valid = self.odom_valid = False
        self.last_received_at = {'scan': None, 'odom': None}
        self.last_received_stamp = {'scan': None, 'odom': None}
        self.rejected_count = {'scan': 0, 'odom': 0}
        self.rejection_reason = {'scan': '', 'odom': ''}
        self.last_rejection_reason = {'scan': '', 'odom': ''}
        self.position_std = self.yaw_std = None
        self.last_amcl_stamp_ns = None
        self.previous_amcl = None
        self.jump_until = 0.0
        self.agreement = self.agreement_at = self.agreement_stamp = None
        self.last_reset_seq = {}
        self.safety_seq = 0
        self.last_estop_at = None
        self.last_level = None
        self.no_motion_client = self.create_client(Empty, '/request_nomotion_update')
        self.no_motion_future = None
        self.no_motion_started_at = None
        self.last_no_motion_at = None
        self.no_motion_requests = self.no_motion_responses = self.no_motion_timeouts = 0
        self.no_motion_errors = 0
        self.stamps = {name: SensorStampGuard(self.core.sensor_timeout, 0.25)
                       for name in ('scan', 'odom')}
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            String, '/smart_vision_amr/localization_health', durable)
        self.control_pub = self.create_publisher(String, '/smart_vision_amr/manual_control', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, durable)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.create_subscription(String, '/smart_vision_amr/manual_control', self.on_control, 10)
        self.create_timer(0.25, self.on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_map(self, message):
        info = message.info
        try:
            if (message.header.frame_id != self.map_frame or info.resolution <= 0.0
                    or not math.isfinite(info.resolution)
                    or info.width*info.height != len(message.data)):
                raise ValueError('invalid map dimensions/frame')
            yaw = quaternion_yaw(info.origin.orientation)
            if not all(math.isfinite(value) for value in
                       (info.origin.position.x, info.origin.position.y)):
                raise ValueError('invalid map origin')
            self.map_metadata = {
                'width': info.width, 'height': info.height, 'resolution': info.resolution,
                'origin': {'x': info.origin.position.x, 'y': info.origin.position.y, 'yaw': yaw}}
            data = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
            self.map_distance = occupied_distance(data, info.resolution)
            self.agreement = self.agreement_at = self.agreement_stamp = None
        except (ValueError, TypeError):
            self.map_metadata = self.map_distance = None

    def on_scan(self, message):
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        stamp = stamp_ns(message.header)
        self.last_received_at['scan'], self.last_received_stamp['scan'] = now, stamp
        values = np.asarray(message.ranges, dtype=float)
        finite = np.isfinite(values) & (values >= message.range_min) & (values <= message.range_max)
        structurally_valid = (values.ndim == 1 and len(values) >= 12
                           and np.count_nonzero(finite) >= 12
                           and math.isfinite(message.angle_min)
                           and math.isfinite(message.angle_increment)
                           and message.angle_increment != 0.0)
        if not structurally_valid:
            self.scan_valid = False
            self.reject('scan', 'invalid_scan_values')
        elif not self.stamps['scan'].accept(stamp, ros_ns):
            # An extra replay must not flip a good fresh scan invalid; it also
            # must not refresh its receive age. The last good sample ages out.
            self.reject('scan', stamp_rejection_reason(self.stamps['scan'], stamp, ros_ns))
        else:
            self.scan_valid = True
            self.scan, self.scan_at = message, now
            self.rejection_reason['scan'] = ''

    def on_odom(self, message):
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        stamp = stamp_ns(message.header)
        self.last_received_at['odom'], self.last_received_stamp['odom'] = now, stamp
        issues = odometry_issues(message, base_frame=self.base_frame)
        if issues:
            self.odom_valid = False
            self.reject('odom', '; '.join(issues))
        elif not self.stamps['odom'].accept(stamp, ros_ns):
            self.reject('odom', stamp_rejection_reason(self.stamps['odom'], stamp, ros_ns))
        else:
            self.odom_valid = True
            self.odom, self.odom_at = message, now
            self.rejection_reason['odom'] = ''

    def reject(self, name, reason):
        self.rejected_count[name] += 1
        self.rejection_reason[name] = self.last_rejection_reason[name] = str(reason)

    def on_amcl(self, message):
        now = time.monotonic()
        stamp = stamp_ns(message.header)
        if stamp <= 0 or (self.last_amcl_stamp_ns is not None and stamp <= self.last_amcl_stamp_ns):
            return
        try:
            covariance = message.pose.covariance
            if (message.header.frame_id != self.map_frame
                    or any(not math.isfinite(value) for value in covariance)
                    or any(covariance[index] < 0.0 for index in (0, 7, 35))):
                raise ValueError('invalid AMCL covariance/frame')
            pose = message.pose.pose
            yaw = quaternion_yaw(pose.orientation)
            if not all(math.isfinite(value) for value in (pose.position.x, pose.position.y)):
                raise ValueError('invalid AMCL position')
            xy = (covariance[1]+covariance[6])/2.0
            if covariance[0]*covariance[7]-xy*xy < -1e-9:
                raise ValueError('nonphysical AMCL position covariance')
            # Largest principal-axis uncertainty includes correlated x/y error.
            largest = (covariance[0]+covariance[7]+math.sqrt(
                (covariance[0]-covariance[7])**2+4.0*xy*xy))/2.0
            self.position_std = math.sqrt(largest)
            self.yaw_std = math.sqrt(covariance[35])
            # Check correction size against independently integrated odom.
            # Initial acquisition/initialpose has no previous baseline.
            if self.odom is not None:
                odom_pose = self.odom.pose.pose
                current = (pose.position.x, pose.position.y, yaw,
                           odom_pose.position.x, odom_pose.position.y,
                           quaternion_yaw(odom_pose.orientation))
                if self.previous_amcl is not None:
                    previous = self.previous_amcl
                    map_delta = math.hypot(current[0]-previous[0], current[1]-previous[1])
                    odom_delta = math.hypot(current[3]-previous[3], current[4]-previous[4])
                    yaw_residual = abs(angle_difference(
                        angle_difference(current[2], previous[2]),
                        angle_difference(current[5], previous[5])))
                    if (abs(map_delta-odom_delta) > float(self.get_parameter('jump_distance_m').value)
                            or yaw_residual > float(self.get_parameter('jump_yaw_rad').value)):
                        self.jump_until = now+max(2.0, 2.0*self.core.fault_seconds)
                self.previous_amcl = current
            self.amcl_at = now
            self.last_amcl_stamp_ns = stamp
        except (ValueError, TypeError):
            self.position_std = self.yaw_std = None

    def on_initialpose(self, _message):
        self.previous_amcl = None
        self.position_std = self.yaw_std = self.amcl_at = None
        self.agreement = self.agreement_at = self.agreement_stamp = None
        self.jump_until = 0.0
        self.core.reinitialize()

    def _tf(self, target, source, stamp=None):
        return self.tf_buffer.lookup_transform(
            target, source, rclpy.time.Time() if stamp is None else rclpy.time.Time.from_msg(stamp))

    def update_agreement(self, now):
        if (self.scan is None or self.map_distance is None
                or stamp_ns(self.scan.header) == self.agreement_stamp):
            return
        try:
            # Both transforms correspond to exactly the laser measurement,
            # not the newest pose available when this callback is scheduled.
            scan_time = self.scan.header.stamp
            laser = self._tf(self.base_frame, self.scan.header.frame_id, scan_time).transform
            robot = self._tf(self.map_frame, self.base_frame, scan_time).transform
            if not all(math.isfinite(value) for value in (
                    laser.translation.x, laser.translation.y,
                    robot.translation.x, robot.translation.y)):
                raise ValueError('nonfinite scan transform')
            points = scan_points(self.scan, (laser.translation.x, laser.translation.y,
                                             quaternion_yaw(laser.rotation)))
            self.agreement = scan_map_agreement(
                points, {'x': robot.translation.x, 'y': robot.translation.y,
                         'yaw': quaternion_yaw(robot.rotation)},
                self.map_metadata, self.map_distance,
                tolerance=float(self.get_parameter('agreement_tolerance').value))
            self.agreement_at, self.agreement_stamp = now, stamp_ns(self.scan.header)
        except (TransformException, ValueError):
            # Leave the previous score with its original receive time. A TF
            # outage must age out rather than refresh a reassuring old score.
            return

    def on_tick(self):
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        self.request_initial_covariance(now)
        self.update_agreement(now)
        metrics = {
            'map_available': self.map_distance is not None,
            'scan_age_seconds': None if self.scan_at is None else now-self.scan_at,
            'odom_age_seconds': None if self.odom_at is None else now-self.odom_at,
            'scan_header_age_seconds': None if self.scan is None else (ros_ns-stamp_ns(self.scan.header))/1e9,
            'odom_header_age_seconds': None if self.odom is None else (ros_ns-stamp_ns(self.odom.header))/1e9,
            'scan_valid': bool(self.scan_valid), 'odom_valid': bool(self.odom_valid),
            'position_std_m': self.position_std, 'yaw_std_rad': self.yaw_std,
            'amcl_age_seconds': None if self.amcl_at is None else now-self.amcl_at,
            'amcl_update_epoch': self.amcl_at,
            'amcl_header_age_seconds': None if self.last_amcl_stamp_ns is None else (ros_ns-self.last_amcl_stamp_ns)/1e9,
            'moving': self.odom is not None and (
                abs(self.odom.twist.twist.linear.x) > 0.015
                or abs(self.odom.twist.twist.angular.z) > 0.05),
            'pose_jump': now < self.jump_until,
            'scan_map_agreement': self.agreement,
            'agreement_age_seconds': None if self.agreement_at is None else now-self.agreement_at,
            'map_tf_age_seconds': None, 'odom_tf_valid': False,
            'amcl_update_requests': self.no_motion_requests,
            'amcl_update_responses': self.no_motion_responses,
            'amcl_update_timeouts': self.no_motion_timeouts,
            'amcl_update_errors': self.no_motion_errors,
        }
        for name in ('odom', 'scan'):
            received, raw_stamp = self.last_received_at[name], self.last_received_stamp[name]
            metrics.update({
                f'{name}_last_received_age_seconds': None if received is None else now-received,
                f'{name}_raw_header_age_seconds': None if raw_stamp is None else (ros_ns-raw_stamp)/1e9,
                f'{name}_rejected_count': self.rejected_count[name],
                f'{name}_rejection_reason': self.rejection_reason[name],
                f'{name}_last_rejection_reason': self.last_rejection_reason[name],
            })
        try:
            transform = self._tf(self.map_frame, self.base_frame)
            quaternion_yaw(transform.transform.rotation)
            if not all(math.isfinite(value) for value in (
                    transform.transform.translation.x, transform.transform.translation.y)):
                raise ValueError('nonfinite map TF')
            metrics['map_tf_age_seconds'] = (ros_ns-stamp_ns(transform.header))/1e9
        except (TransformException, ValueError):
            pass
        try:
            transform = self._tf(self.odom_frame, self.base_frame)
            quaternion_yaw(transform.transform.rotation)
            age = (ros_ns-stamp_ns(transform.header))/1e9
            metrics['odom_tf_valid'] = -0.25 <= age <= self.core.tf_timeout
        except (TransformException, ValueError):
            pass
        status = self.core.evaluate(now, metrics)
        status['stamp_ns'] = ros_ns
        status['sent_at_ns'] = ros_ns
        if status['level'] != self.last_level:
            self.get_logger().info(f"Localization health: {status['level']} {status['reasons']}")
            self.last_level = status['level']
        if status['stop_latched'] and (self.last_estop_at is None or now-self.last_estop_at >= 0.5):
            self.publish_estop(now)
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.publisher.publish(message)

    def request_initial_covariance(self, now):
        """Request a real filter-only laser update without weakening the guard.

        AMCL may otherwise wait for motion before refreshing covariance, while
        the safety mux correctly waits for covariance before allowing motion.
        Once localized, slow movement can stay below AMCL's update thresholds
        and exhaust the motion watchdog. Ask for an actual filter update then,
        too. A request/response never refreshes the AMCL observation epoch;
        only ``on_amcl`` may do that. A failed service cannot stall evaluation.
        """
        def setting(name, default):
            getter = getattr(self, 'get_parameter', None)
            try:
                value = getter(name).value if callable(getter) else default
                return float(value) if not isinstance(value, bool) and math.isfinite(value) and value > 0 else default
            except (AttributeError, KeyError, TypeError, ValueError):
                return default
        # Never permit a runtime typo to flood AMCL or indefinitely retain a
        # pending request. The separate four-second motion watchdog is unchanged.
        interval = max(2., setting('nomotion_update_interval', 2.))
        trigger_age = max(.1, setting('nomotion_update_trigger_age', 1.))
        deadline = min(1., max(.1, setting('nomotion_request_timeout', 1.)))
        if self.no_motion_future is not None:
            future = self.no_motion_future
            started = getattr(self, 'no_motion_started_at', self.last_no_motion_at)
            if not future.done():
                if started is None:
                    self.no_motion_started_at = now
                    return
                if now-started < deadline:
                    return
                try:
                    future.cancel()
                    self.no_motion_client.remove_pending_request(future)
                except Exception:
                    pass
                self.no_motion_timeouts = getattr(self, 'no_motion_timeouts', 0)+1
            elif callable(getattr(future, 'result', None)):
                try:
                    future.result()
                    self.no_motion_responses = getattr(self, 'no_motion_responses', 0)+1
                except Exception:
                    self.no_motion_errors = getattr(self, 'no_motion_errors', 0)+1
            self.no_motion_future = None
            self.no_motion_started_at = None
        if (self.scan is None or self.map_distance is None
                or self.scan_at is None or now-self.scan_at > self.core.sensor_timeout
                or (self.last_no_motion_at is not None and now-self.last_no_motion_at < interval)
                or not self.no_motion_client.service_is_ready()):
            return
        if self.position_std is not None and not LocalizationHealth._moving_update_ready(self, now, trigger_age):
            return
        self.last_no_motion_at = now
        try:
            self.no_motion_future = self.no_motion_client.call_async(Empty.Request())
            self.no_motion_started_at = now
            self.no_motion_requests = getattr(self, 'no_motion_requests', 0)+1
        except Exception:
            self.no_motion_future = None
            self.no_motion_started_at = None
            self.no_motion_errors = getattr(self, 'no_motion_errors', 0)+1

    def _moving_update_ready(self, now, trigger_age=1.):
        """Only healthy, measured slow motion can request periodic AMCL work."""
        amcl_at, amcl_stamp = getattr(self, 'amcl_at', None), getattr(self, 'last_amcl_stamp_ns', None)
        odom, odom_at = getattr(self, 'odom', None), getattr(self, 'odom_at', None)
        if (amcl_at is None or now-amcl_at < trigger_age or amcl_stamp is None
                or odom is None or odom_at is None or not 0 <= now-odom_at <= self.core.sensor_timeout
                or not getattr(self, 'scan_valid', False) or not getattr(self, 'odom_valid', False)):
            return False
        ros_ns = self.get_clock().now().nanoseconds
        if (ros_ns <= 0 or ros_ns-amcl_stamp < trigger_age*1e9
                or not 0 <= now-self.scan_at <= self.core.sensor_timeout):
            return False
        for sample in (self.scan, odom):
            age = (ros_ns-stamp_ns(sample.header))/1e9
            if not -.25 <= age <= self.core.sensor_timeout:
                return False
        linear, angular = odom.twist.twist.linear.x, odom.twist.twist.angular.z
        if (not math.isfinite(linear) or not math.isfinite(angular)
                or (abs(linear) <= .015 and abs(angular) <= .05)):
            return False
        try:
            for target in (self.map_frame, self.odom_frame):
                transform = self._tf(target, self.base_frame)
                age = (ros_ns-stamp_ns(transform.header))/1e9
                quaternion_yaw(transform.transform.rotation)
                if (not -.25 <= age <= self.core.tf_timeout
                        or not all(math.isfinite(value) for value in (
                            transform.transform.translation.x, transform.transform.translation.y))):
                    return False
        except (TransformException, ValueError, AttributeError):
            return False
        return True

    def publish_estop(self, now):
        self.safety_seq = max(self.safety_seq+1, time.monotonic_ns())
        message = String()
        message.data = json.dumps({
            'source_id': self.SAFETY_SOURCE, 'seq': self.safety_seq,
            'sent_at_ns': self.get_clock().now().nanoseconds,
            'mode': 'estop', 'linear': 0.0, 'angular': 0.0})
        self.control_pub.publish(message)
        self.last_estop_at = now

    def on_control(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict) or payload.get('mode') != 'reset':
            return
        source, seq, sent = (payload.get(key) for key in ('source_id', 'seq', 'sent_at_ns'))
        if (not isinstance(source, str) or not source or source.endswith('_safety')
                or type(seq) is not int or seq <= self.last_reset_seq.get(source, -1)
                or type(sent) is not int or abs(self.get_clock().now().nanoseconds-sent) > 1e9):
            return
        self.last_reset_seq[source] = seq
        if self.core.stop_latched and not self.core.operator_reset(time.monotonic()):
            self.publish_estop(time.monotonic())


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealth()
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
