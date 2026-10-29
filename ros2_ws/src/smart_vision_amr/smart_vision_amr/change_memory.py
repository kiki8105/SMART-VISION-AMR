"""LiDAR visibility based change observation and persistent memory node."""

import hashlib
import json
import math
import os
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from smart_vision_amr.obstacle_memory import ObstacleMemory
from smart_vision_amr.map_utils import (bresenham, cell_to_world, world_to_cell,
                                       occupancy_to_grid, first_static_obstacle_distance)


class ChangeMemoryNode(Node):
    def __init__(self):
        super().__init__("change_memory")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("memory_path", "~/.ros/smart_vision_amr/environment_memory.json")
        self.declare_parameter("occupied_threshold", 65)
        # Compare measured range with the first known wall, not wall proximity.
        self.declare_parameter("mapped_wall_range_tolerance_m", 0.05)
        self.declare_parameter("confirm_observations", 5)
        self.declare_parameter("clear_observations", 5)
        self.declare_parameter("confirm_min_duration_sec", 2.0)
        self.declare_parameter("clear_min_duration_sec", 2.0)
        self.declare_parameter("min_observation_interval_sec", 0.25)
        self.declare_parameter("save_every_scans", 5)
        self.declare_parameter("memory_enabled", True)
        # This short-lived layer is independent of optional persisted memory.
        self.declare_parameter("live_obstacle_ttl_sec", 2.0)
        self.declare_parameter("live_scan_max_age_sec", 1.0)
        self.declare_parameter("live_transform_max_age_sec", 1.0)
        self.declare_parameter("scan_tf_wait_sec", 0.20)
        self.declare_parameter("scan_tf_queue_size", 5)
        self.declare_parameter("scan_tf_stamp_tolerance_sec", 0.02)
        self.declare_parameter("scan_future_tolerance_sec", 0.15)
        self.declare_parameter("live_localization_jump_distance", 0.25)
        self.declare_parameter("live_localization_jump_angle", 0.25)
        self.raw_static_grid = None
        self.map = None
        self.memory = None
        self.live_obstacles = {}
        self.last_scan_stamp_ns = None
        self.last_map_to_odom = None
        self.scans_since_save = 0
        self.pending_scans = deque()
        self.projection_dropped = 0
        self.projection_valid = 0
        self.last_projection_diagnostic = float("-inf")
        self.memory_path = Path(os.path.expanduser(self.get_parameter("memory_path").value))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.overlay_pub = self.create_publisher(Int32MultiArray, "/smart_vision_amr/remembered_cells", latched)
        self.observation_pub = self.create_publisher(
            Int32MultiArray, "/smart_vision_amr/observed_obstacles", 10)
        self.visibility_pub = self.create_publisher(
            OccupancyGrid, "/smart_vision_amr/observed_visibility", qos_profile_sensor_data)
        self.projection_status_pub = self.create_publisher(
            String, "/smart_vision_amr/scan_projection_status", latched)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, latched)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initial_pose, 10)
        self.create_service(Trigger, "/smart_vision_amr/save_memory", self.save_service)
        self.create_service(Trigger, "/smart_vision_amr/clear_memory", self.clear_service)
        from rclpy.clock import Clock, ClockType
        self.create_timer(.02, self.retry_pending_scans,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def map_signature(self, msg):
        info = msg.info
        metadata = {"frame": msg.header.frame_id, "width": info.width, "height": info.height,
                    "resolution": info.resolution, "origin": [info.origin.position.x,
                    info.origin.position.y, info.origin.position.z, info.origin.orientation.x,
                    info.origin.orientation.y, info.origin.orientation.z, info.origin.orientation.w]}
        digest = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode())
        digest.update(bytes((int(value) + 1) & 0xff for value in msg.data))
        return digest.hexdigest()

    def on_map(self, msg):
        signature = self.map_signature(msg)
        if self.map is not None and signature == self.map["signature"]:
            return
        self.map = {"message": msg, "signature": signature}
        self.raw_static_grid = occupancy_to_grid(
            msg, self.get_parameter("occupied_threshold").value,
            unknown_is_obstacle=False)
        self.live_obstacles.clear()
        self.last_scan_stamp_ns = None
        self.last_map_to_odom = None
        self.pending_scans.clear()
        path = self.memory_path
        if not self.get_parameter("memory_enabled").value:
            self.memory = self._new_memory(msg)
            self.get_logger().info("Persistent environment memory disabled for baseline condition")
        elif path.exists():
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                if payload.get("map_signature") != signature:
                    raise ValueError("saved memory belongs to another map; refusing to apply")
                self.memory = ObstacleMemory.load(path, self._as_grid(msg))
                self.get_logger().info(f"Restored {len(self.memory.confirmed_cells)} remembered cells")
            except Exception as exc:
                self.memory = self._new_memory(msg)
                self.get_logger().error(f"Memory not restored: {exc}")
        else:
            self.memory = self._new_memory(msg)
        self.publish_overlay()

    def on_initial_pose(self, message):
        if message.header.frame_id.lstrip('/') != self.get_parameter("map_frame").value.lstrip('/'):
            return
        # These cells were projected using the old localization solution. The
        # next valid scan rebuilds them; do not announce a fresh empty scan here.
        self.live_obstacles.clear()
        self.last_map_to_odom = None
        self.pending_scans.clear()

    def reset_live_on_localization_jump(self, target_frame, measurement_time):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, self.get_parameter("odom_frame").value, measurement_time)
        except TransformException:
            return
        pose = (transform.transform.translation.x, transform.transform.translation.y,
                self._yaw(transform.transform.rotation))
        if not all(math.isfinite(value) for value in pose):
            return
        previous = self.last_map_to_odom
        self.last_map_to_odom = pose
        if previous is None:
            return
        delta_yaw = abs(math.atan2(math.sin(pose[2]-previous[2]), math.cos(pose[2]-previous[2])))
        if (math.hypot(pose[0]-previous[0], pose[1]-previous[1])
                > self.get_parameter("live_localization_jump_distance").value
                or delta_yaw > self.get_parameter("live_localization_jump_angle").value):
            self.live_obstacles.clear()
            self.get_logger().warning(
                "Localization frame changed; rebuilding transient LiDAR obstacles from fresh scans")

    @staticmethod
    def cell_in_unknown_sector(cell, info, sensor_xy, invalid_angles, angular_step):
        """Retain only cells intersecting an invalid beam's narrow angular cone."""
        if not invalid_angles:
            return False
        x, y = cell_to_world(info, cell)
        dx, dy = x-sensor_xy[0], y-sensor_xy[1]
        distance = math.hypot(dx, dy)
        half_diagonal = info.resolution*math.sqrt(2.)*.5
        half_width = (abs(angular_step)*.5
                      + math.asin(min(1., half_diagonal/max(distance, half_diagonal))))
        bearing = math.atan2(dy, dx)
        return any(abs(math.atan2(math.sin(angle-bearing), math.cos(angle-bearing))) <= half_width
                   for angle in invalid_angles)

    def _as_grid(self, msg):
        from smart_vision_amr.map_utils import occupancy_to_grid
        return occupancy_to_grid(msg, self.get_parameter("occupied_threshold").value)

    def _new_memory(self, msg):
        memory = ObstacleMemory(msg.info.width, msg.info.height,
                                self.get_parameter("confirm_observations").value,
                                self.get_parameter("clear_observations").value,
                                confirm_min_duration_sec=self.get_parameter("confirm_min_duration_sec").value,
                                clear_min_duration_sec=self.get_parameter("clear_min_duration_sec").value,
                                min_observation_interval_sec=self.get_parameter(
                                    "min_observation_interval_sec").value)
        memory.map_signature = self.map["signature"]
        return memory

    def on_scan(self, scan):
        if self.map is None or self.memory is None:
            return
        if not self._scan_is_current(scan):
            return
        stamp_ns = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        if any(item[0] == stamp_ns for item in self.pending_scans):
            return
        # A zero-time TF query means "latest" in TF2, never measurement time.
        if stamp_ns == 0:
            self._projection_diagnostic("DROPPED", "unstamped_scan", scan)
            return
        if self._project_scan(scan):
            return
        bound = max(1, min(50, int(self.get_parameter("scan_tf_queue_size").value)))
        while len(self.pending_scans) >= bound:
            _, _, dropped = self.pending_scans.popleft()
            self._projection_diagnostic("DROPPED", "tf_queue_full", dropped)
        self.pending_scans.append((stamp_ns, time.monotonic(), scan))
        self.pending_scans = deque(sorted(self.pending_scans, key=lambda item: item[0]))
        self._projection_diagnostic("PENDING_TF", "waiting_for_measurement_transform", scan)

    def retry_pending_scans(self):
        """Short nonblocking retries allow a scan to arrive just ahead of TF."""
        wait = max(.01, min(1., float(self.get_parameter("scan_tf_wait_sec").value)))
        now = time.monotonic()
        retained = deque()
        while self.pending_scans:
            stamp, arrived, scan = self.pending_scans.popleft()
            if not self._scan_is_current(scan):
                continue
            if now - arrived >= wait:
                self._projection_diagnostic("DROPPED", "measurement_tf_timeout", scan)
                continue
            if not self._project_scan(scan):
                retained.append((stamp, arrived, scan))
        self.pending_scans = retained

    def _scan_is_current(self, scan):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        max_age = max(.05, float(self.get_parameter("live_scan_max_age_sec").value))
        future = max(0., min(.25, float(self.get_parameter("scan_future_tolerance_sec").value)))
        if (now_ns-stamp_ns) > max_age * 1e9 or stamp_ns-now_ns > future*1e9:
            self._projection_diagnostic("DROPPED", "scan_clock_skew_or_stale", scan)
            return False
        if self.last_scan_stamp_ns is not None and stamp_ns <= self.last_scan_stamp_ns:
            return False
        if (not all(math.isfinite(value) for value in (scan.angle_min, scan.angle_increment,
                                                      scan.range_min, scan.range_max))
                or scan.angle_increment == 0.0 or not 0 <= scan.range_min < scan.range_max):
            self._projection_diagnostic("DROPPED", "invalid_scan_geometry", scan)
            return False
        return True

    def _projection_diagnostic(self, status, reason, scan, tf_skew=None):
        if status == "DROPPED":
            self.projection_dropped += 1
            self.get_logger().warning(f"LiDAR projection rejected: {reason}", throttle_duration_sec=5.)
        now = time.monotonic()
        if status == "VALID" and now-self.last_projection_diagnostic < 1.:
            return
        self.last_projection_diagnostic = now
        stamp_ns = scan.header.stamp.sec*1_000_000_000 + scan.header.stamp.nanosec
        message = String()
        message.data = json.dumps({"status": status, "reason": reason,
            "scan_stamp_ns": stamp_ns,
            "scan_age_ms": (self.get_clock().now().nanoseconds-stamp_ns)/1e6,
            "tf_skew_ms": tf_skew, "queue_depth": len(self.pending_scans),
            "valid_scans": self.projection_valid, "dropped_scans": self.projection_dropped})
        self.projection_status_pub.publish(message)

    def _project_scan(self, scan):
        """Return false only when TF may still arrive within the bounded wait."""
        stamp_ns = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        target_frame = self.get_parameter("map_frame").value
        sensor_frame = scan.header.frame_id or self.get_parameter("base_frame").value
        measurement_time = rclpy.time.Time.from_msg(scan.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, sensor_frame, measurement_time)
        except TransformException as exc:
            self.get_logger().warning(f"Waiting for scan-stamped TF: {exc}", throttle_duration_sec=5.0)
            return False
        tf_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
        tf_max_age = max(.05, float(self.get_parameter("live_transform_max_age_sec").value))
        tolerance = min(tf_max_age, max(.000001, float(self.get_parameter(
            "scan_tf_stamp_tolerance_sec").value)))
        skew_ms = (tf_ns-stamp_ns)/1e6 if tf_ns else 0.
        if tf_ns and abs(stamp_ns-tf_ns) > tolerance * 1e9:
            self._projection_diagnostic("DROPPED", "tf_measurement_stamp_skew", scan, skew_ms)
            return True
        q = tf.transform.rotation
        if not all(math.isfinite(value) for value in (tf.transform.translation.x,
                tf.transform.translation.y, q.x, q.y, q.z, q.w)):
            self._projection_diagnostic("DROPPED", "invalid_transform", scan, skew_ms)
            return True
        self.reset_live_on_localization_jump(target_frame, measurement_time)
        info = self.map["message"].info
        origin = world_to_cell(info, tf.transform.translation.x, tf.transform.translation.y)
        yaw = self._yaw(tf.transform.rotation)
        occupied_cells, free_cells = set(), set()
        invalid_angles = []
        valid_beams = 0
        wall_tolerance = float(self.get_parameter("mapped_wall_range_tolerance_m").value)
        # Invalid tolerance must never discard actual obstacles.
        wall_tolerance = max(0., wall_tolerance) if math.isfinite(wall_tolerance) else 0.
        for index, distance in enumerate(scan.ranges):
            angle = yaw + scan.angle_min + index * scan.angle_increment
            # NaN, zero/below-minimum and out-of-range packets are not evidence
            # that the ray is free. Positive infinity is the LaserScan no-hit case.
            if (math.isnan(distance) or distance == -math.inf
                    or distance < scan.range_min
                    or (math.isfinite(distance) and distance > scan.range_max)):
                invalid_angles.append(angle)
                continue
            valid_beams += 1
            valid_hit = scan.range_min <= distance <= scan.range_max and distance < scan.range_max
            ray_length = distance if valid_hit else scan.range_max
            endpoint = (tf.transform.translation.x + ray_length * math.cos(angle),
                        tf.transform.translation.y + ray_length * math.sin(angle))
            end_cell = world_to_cell(info, *endpoint)
            ray = list(bresenham(origin, end_cell))
            for cell in ray[:-1] if valid_hit else ray:
                if self._is_free_in_base_map(cell):
                    free_cells.add(cell)
            if valid_hit and self.raw_static_grid.contains(end_cell):
                wall_distance = first_static_obstacle_distance(
                    info, self.raw_static_grid, tf.transform.translation.x,
                    tf.transform.translation.y, angle, scan.range_max)
                # A nearby object in front of a wall remains a dynamic hit.
                # Unknown cells are not evidence of a mapped wall.
                wall_echo = (wall_distance is not None
                             and abs(wall_distance-distance) <= wall_tolerance)
                if not wall_echo:
                    occupied_cells.add(end_cell)
        if not valid_beams:
            return True
        self.last_scan_stamp_ns = stamp_ns
        self.projection_valid += 1
        free_cells.difference_update(occupied_cells)
        now = time.monotonic()
        ttl = max(.05, float(self.get_parameter("live_obstacle_ttl_sec").value))
        # Keep briefly occluded returns, but clear a cell as soon as a valid ray
        # traverses it. A few hardware edge dropouts must not freeze every old
        # obstacle in the map: protect only each dropout's actual angular cone.
        # Neither TTL nor relocalization edits the saved SLAM grid.
        sensor_xy = (tf.transform.translation.x, tf.transform.translation.y)
        self.live_obstacles = {cell: seen for cell, seen in self.live_obstacles.items()
                               if self.raw_static_grid.contains(cell) and cell not in free_cells
                               and (now-seen < ttl or self.cell_in_unknown_sector(
                                   cell, info, sensor_xy, invalid_angles, scan.angle_increment))}
        self.live_obstacles.update((cell, now) for cell in occupied_cells)
        observed = Int32MultiArray()
        observed.data = sorted(y * self.memory.width + x for x, y in self.live_obstacles)
        self.observation_pub.publish(observed)
        # A missing hit/TTL expiry is not removal evidence. Publish exactly
        # which cells this measurement's valid rays traversed, with its stamp.
        if hasattr(self, "visibility_pub"):
            visibility = OccupancyGrid()
            visibility.header.frame_id = target_frame
            visibility.header.stamp = scan.header.stamp
            visibility.info = info
            data = [-1] * (info.width*info.height)
            for x, y in free_cells:
                data[y*info.width+x] = 0
            for x, y in occupied_cells:
                data[y*info.width+x] = 100
            visibility.data = data
            self.visibility_pub.publish(visibility)
        self._projection_diagnostic("VALID", "measurement_transform_applied", scan, skew_ms)
        if not self.get_parameter("memory_enabled").value:
            return True
        changed = False
        observed_at = time.time()
        for cell in free_cells:
            changed = self.memory.observe_free(cell, observed_at) or changed
        for cell in occupied_cells:
            # Live returns may include raw occupied/unknown cells; persistent
            # changes only describe additions to known free cells of the map.
            if self._is_free_in_base_map(cell):
                changed = self.memory.observe_occupied(cell, observed_at) or changed
        self.scans_since_save += 1
        if changed:
            self.publish_overlay()
            self.persist()
            self.scans_since_save = 0
        interval = self.get_parameter("save_every_scans").value
        if interval > 0 and self.scans_since_save >= interval:
            self.persist()
            self.scans_since_save = 0
        return True

    def _is_free_in_base_map(self, cell):
        msg = self.map["message"]
        x, y = cell
        if not (0 <= x < msg.info.width and 0 <= y < msg.info.height):
            return False
        value = msg.data[y * msg.info.width + x]
        return 0 <= value < self.get_parameter("occupied_threshold").value

    @staticmethod
    def _yaw(q):
        from math import atan2
        return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_overlay(self):
        if self.memory is None:
            return
        msg = Int32MultiArray()
        msg.data = [index for x, y in sorted(self.memory.confirmed_cells)
                    for index in (y * self.memory.width + x,)]
        self.overlay_pub.publish(msg)

    def persist(self):
        if self.memory is not None and self.get_parameter("memory_enabled").value:
            self.memory.map_signature = self.map["signature"]
            self.memory.save(self.memory_path)

    def save_service(self, request, response):
        if not self.get_parameter("memory_enabled").value:
            response.success, response.message = False, "Memory is disabled in baseline mode"
            return response
        try:
            self.persist()
            response.success, response.message = True, "Memory saved"
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def clear_service(self, request, response):
        if self.map is None:
            response.success, response.message = False, "Base map is not available"
            return response
        self.memory = self._new_memory(self.map["message"])
        self.persist()
        self.publish_overlay()
        response.success, response.message = True, "Memory cleared"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ChangeMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.persist()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
