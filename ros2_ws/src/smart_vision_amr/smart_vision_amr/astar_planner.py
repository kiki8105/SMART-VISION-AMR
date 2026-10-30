"""A* ROS adapter: OccupancyGrid + persistent cells + PoseStamped goal -> Path."""

import json
import time
from math import atan2, ceil, cos, hypot, isfinite, pi, sin, sqrt

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from collections import deque

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.local_detour import plan_local_detour
from smart_vision_amr.clearance_cost import clearance_penalties
from smart_vision_amr.grid import Grid
from smart_vision_amr.failure_memory import FailureMemory, EXECUTION_FAILURE_REASONS
from smart_vision_amr.obstacle_hold import ObstacleHold
from smart_vision_amr.localization_geometry import rotation_clearance
from smart_vision_amr.map_utils import (cell_to_world, inflate_cells, occupancy_to_grid,
                                        split_mapped_wall_returns, world_to_cell,
                                        yaw_from_quaternion)
from smart_vision_amr.reverse_recovery import (Breadcrumb, GridGeometry,
                                               select_reverse_egress)
from smart_vision_amr.path_geometry import (route_cells, smooth_grid_path, path_progress,
                                           point_at_progress, path_length)


LOCAL_DETOUR_DEFAULTS = {
    "local_detour_enabled": True, "local_detour_trigger_distance": 1.5,
    "local_detour_max_rejoin_distance": 2.5, "local_detour_max_length": 4.0,
    "local_detour_max_lateral_distance": .9, "local_detour_max_expansions": 1500,
    "local_detour_max_planning_time": .06, "local_detour_max_candidates": 10,
    "local_detour_max_attempts": 3, "local_detour_clearance_margin": .02,
    "local_detour_occlusion_extension": .5, "local_detour_rejoin_buffer": .15,
    "local_detour_observation_margin": .05,
}


FAILURE_DEFAULTS = {
    **LOCAL_DETOUR_DEFAULTS,
    "failure_memory_enabled": True, "failure_memory_ttl": 120.0,
    "failure_memory_radius": .25, "failure_memory_heading_tolerance": pi/3,
    "failure_memory_max_entries": 64, "failure_memory_penalty": 2.0,
    "failure_event_max_age": 3.0, "rejoin_progress_distance": .35,
    "rejoin_cross_track_tolerance": .20, "recovery_turn_heading_threshold": .65,
    "recovery_turn_clearance_margin": .03,
    "obstacle_hold_enabled": True, "obstacle_retry_limit": 3,
    "obstacle_clear_duration": 2.0,
}


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("obstacle_inflation_cells", 2)
        self.declare_parameter("mapped_wall_return_tolerance_cells", 0)
        self.declare_parameter("replan_retry_period", 2.0)
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("start_egress_enabled", True)
        self.declare_parameter("start_egress_clearance_margin", 0.02)
        self.declare_parameter("start_egress_max_cells", 7)
        self.declare_parameter("live_obstacle_layer_required", False)
        self.declare_parameter("live_obstacle_timeout", 2.0)
        self.declare_parameter("preferred_clearance", 0.45)
        self.declare_parameter("clearance_penalty_gain", 1.5)
        self.declare_parameter("path_smoothing_enabled", True)
        self.declare_parameter("path_smoothing_iterations", 2)
        self.declare_parameter("path_smoothing_shortcut_distance", 0.80)
        self.declare_parameter("path_resample_spacing", 0.05)
        self.declare_parameter("reverse_recovery_enabled", True)
        self.declare_parameter("reverse_recovery_min_distance", 0.10)
        self.declare_parameter("reverse_recovery_max_distance", 0.30)
        self.declare_parameter("reverse_recovery_clearance_margin", 0.02)
        self.declare_parameter("reverse_recovery_speed", 0.04)
        self.declare_parameter("reverse_recovery_max_attempts", 3)
        for name, value in FAILURE_DEFAULTS.items():
            self.declare_parameter(name, value)
        self.map = None
        self.raw_static_grid = None
        self.map_signature = None
        self.remembered = set()
        self.observed = set()
        self.last_observation_time = None
        self.last_goal = None
        self.planned_cells = []
        self.waiting_for_transform = False
        self.waiting_for_replan = False
        self.navigation_suspended = False
        self.goal_completed = False
        self.last_plan_time = float("-inf")
        self.invalid_path_published = False
        self.geometry_signature = None
        self.stop_stamp_ns = None
        self.breadcrumbs = deque(maxlen=300)
        self.breadcrumb_odom_anchor = None
        self.latest_odom = None
        self.odom_samples = deque(maxlen=100)
        self.breadcrumb_map_key = None
        self.recovery_active = False
        self.recovery_requested_at = None
        self.recovery_cancel_due_map_change = False
        self.recovery_attempts = 0
        self.recovery_attempted_cells = set()
        self._init_failure_state()
        self.planner = AStarPlanner()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, "/smart_vision_amr/path", 10)
        self.status_pub = self.create_publisher(String, "/smart_vision_amr/planner_status", latched)
        self.recovery_pub = self.create_publisher(String, "/smart_vision_amr/recovery_request", 10)
        self.failure_status_pub = self.create_publisher(
            String, "/smart_vision_amr/failure_memory_status", latched)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, latched)
        self.create_subscription(Int32MultiArray, "/smart_vision_amr/remembered_cells", self.on_memory, latched)
        self.create_subscription(Int32MultiArray, "/smart_vision_amr/observed_obstacles",
                                 self.on_observed, 10)
        self.create_subscription(PoseStamped, "/smart_vision_amr/goal", self.on_goal, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/smart_vision_amr/navigation_status",
                                 self.on_navigation_status, 10)
        self.create_subscription(String, "/smart_vision_amr/execution_failure",
                                 self.on_execution_failure, 10)
        self.create_subscription(String, "/smart_vision_amr/localization_health",
                                 self.on_localization_health, latched)
        self.create_subscription(String, "/smart_vision_amr/motion_status",
                                 self.on_motion_status, latched)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/smart_vision_amr/observed_visibility",
                                 self.on_visibility, qos_profile_sensor_data)
        self.create_timer(0.25, self.retry_transform)
        self.create_timer(.5, self.publish_failure_status,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status("IDLE", "Waiting for an explicit goal")

    def on_map(self, msg):
        # A previous trail is only meaningful in the exact saved map in which
        # its map-frame poses were measured. Repeated publications of that map
        # are harmless; a reload, new SLAM content, or source-frame change is not.
        info = msg.info
        origin = info.origin
        map_key = (msg.header.frame_id, info.width, info.height,
                   info.resolution, info.map_load_time.sec, info.map_load_time.nanosec,
                   origin.position.x, origin.position.y, origin.position.z,
                   origin.orientation.x, origin.orientation.y,
                   origin.orientation.z, origin.orientation.w, tuple(msg.data))
        map_replaced = self.breadcrumb_map_key is not None and map_key != self.breadcrumb_map_key
        if map_replaced:
            self._clear_breadcrumbs()
            self._reset_local_reference()
            self.local_grid_cache = None
        self.breadcrumb_map_key = map_key
        threshold = self.get_parameter("occupied_threshold").value
        inflation = self.get_parameter("obstacle_inflation_cells").value
        self.map = (msg, occupancy_to_grid(msg, threshold, inflation))
        self.raw_static_grid = occupancy_to_grid(msg, threshold, 0)
        self.map_signature = (threshold, inflation,
                              self.get_parameter("mapped_wall_return_tolerance_cells").value)
        if self._goal_is_active():
            if getattr(self, "recovery_active", False):
                if map_replaced and not self.recovery_cancel_due_map_change:
                    # The reverse swept volume was certified against the old
                    # map. The follower consumes this explicit cancellation
                    # even before BackUp starts; an empty Path alone cannot
                    # resolve the cross-topic request/path ordering race.
                    self.recovery_cancel_due_map_change = True
                    goal = self.last_goal
                    cancel = String()
                    cancel.data = json.dumps({
                        "goal_id": f"{goal.header.stamp.sec}:{goal.header.stamp.nanosec}",
                        "cancel": True,
                        "reason": "Map changed during reverse recovery"})
                    self.recovery_pub.publish(cancel)
                    self.invalid_path_published = False
                    self._invalidate_path("Map changed during reverse recovery; canceling reverse")
                return
            if any(self.map[1].is_static_obstacle(cell) for cell in self._remaining_path_cells()):
                self._invalidate_path("Updated map blocks the active route")
            self._request_replan("Map updated")

    def _goal_is_active(self):
        return (self.last_goal is not None and not self.navigation_suspended
                and not self.goal_completed)

    def _init_failure_state(self):
        self.failure_memory = FailureMemory()
        self.failure_state, self.failure_reason = "CLEAR", ""
        self.last_failure_id = None
        self.last_failure_event_stamp_ns = 0
        self.seen_failure_ids = deque(maxlen=256)
        self.pending_failure_id = None
        self.pending_failure_since = None
        self.localization_health = self.latest_health_at = None
        self.latest_scan = self.latest_scan_at = None
        self.planned_world_points = []
        self.rejoin_anchor = None
        self.rejoin_verified = False
        self.navigation_progress_at = None
        self.obstacle_hold = ObstacleHold()
        self.obstacle_episode_failure_ids = set()
        self.obstacle_failure_cells = {}
        self.last_visibility_at = None
        self.last_visibility_stamp_ns = 0
        self.latest_motion_status = self.latest_motion_status_at = None
        self.local_reference_points = []
        self.local_reference_key = None
        self.local_grid_cache = None
        self.local_cell_penalty = {}
        self.local_detour_attempts = 0
        self.local_attempt_id = 0
        self.global_plan_count = 0
        self.local_rejoin_anchor = None
        self.plan_metrics = {"plan_kind": "NONE", "local_detour_state": "NONE",
                             "local_attempt_id": 0, "global_plan_count": 0}

    def on_motion_status(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.latest_motion_status, self.latest_motion_status_at = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def _reset_local_reference(self):
        self.local_reference_points = []
        self.local_reference_key = None
        self.local_detour_attempts = 0
        self.local_rejoin_anchor = None
        self.plan_metrics.update(local_detour_state="NONE")

    def _local_route_key(self):
        goal = self.last_goal
        if goal is None:
            return None
        p, q = goal.pose.position, goal.pose.orientation
        return (self.breadcrumb_map_key, goal.header.frame_id,
                goal.header.stamp.sec, goal.header.stamp.nanosec,
                p.x, p.y, q.x, q.y, q.z, q.w)

    def _invalid_local_configuration(self, reason):
        self.plan_metrics.update(local_detour_state="FAILED", detour_reason="invalid_configuration",
                                 detour_configuration_error=str(reason)[:160])
        return None

    def _local_configuration(self):
        """Runtime edits may disable this optional path, never crash planning."""
        try:
            config = {name.removeprefix("local_detour_"): self.get_parameter(name).value
                      for name in LOCAL_DETOUR_DEFAULTS}
            if type(config["enabled"]) is not bool:
                raise ValueError("local_detour_enabled must be boolean")
            if not config["enabled"]:
                return config
            for name in ("trigger_distance", "max_rejoin_distance", "max_length",
                         "max_lateral_distance", "max_planning_time"):
                value = config[name]
                if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                    raise ValueError("local_detour_"+name+" must be finite and positive")
            if config["max_planning_time"] > .1:
                raise ValueError("local_detour_max_planning_time must be at most 0.1 seconds")
            margin = config["clearance_margin"]
            if (type(margin) not in (int, float) or not isfinite(margin)
                    or not 0 <= margin <= 1.):
                raise ValueError("local_detour_clearance_margin must be finite in 0..1 metres")
            for name in ("occlusion_extension", "rejoin_buffer", "observation_margin"):
                value = config[name]
                if type(value) not in (int, float) or not isfinite(value) or not 0 <= value <= 1.:
                    raise ValueError("local_detour_"+name+" must be finite in 0..1 metres")
            for name, maximum in (("max_expansions", 10000), ("max_candidates", 64),
                                  ("max_attempts", 10)):
                if type(config[name]) is not int or not 1 <= config[name] <= maximum:
                    raise ValueError("local_detour_"+name+" must be an integer in 1.."+str(maximum))
            return config
        except (ValueError, TypeError, OverflowError) as exc:
            return self._invalid_local_configuration(exc)

    def _local_footprint_grid(self, config=None):
        """Cache static/unknown footprint inflation; never clear start cells."""
        config = self._local_configuration() if config is None else config
        if config is None or not config["enabled"]:
            return None, 0
        try:
            return self._build_local_footprint_grid(config)
        except (ValueError, TypeError, OverflowError) as exc:
            self._invalid_local_configuration(exc)
            return None, 0

    def _build_local_footprint_grid(self, config):
        message = self.map[0]
        radius = float(self.get_parameter("robot_radius").value)+config["clearance_margin"]
        if not isfinite(radius) or radius <= 0 or not isfinite(message.info.resolution) or message.info.resolution <= 0:
            raise ValueError("Local footprint radius and map resolution must be finite and positive")
        inflation = max(int(self.get_parameter("obstacle_inflation_cells").value),
                        ceil(radius/message.info.resolution))
        key = (self.breadcrumb_map_key, self.get_parameter("occupied_threshold").value,
               inflation, radius)
        if self.local_grid_cache is None or self.local_grid_cache[0] != key:
            grid = occupancy_to_grid(message, key[1], inflation)
            margin = radius/message.info.resolution
            # Outside-map space is unknown too; the complete footprint must
            # remain inside, even when no occupied edge cells were published.
            rows = tuple(tuple(value or x+.5 < margin or grid.width-x-.5 < margin
                               or y+.5 < margin or grid.height-y-.5 < margin
                               for x, value in enumerate(row))
                         for y, row in enumerate(grid.rows))
            self.local_grid_cache = (key, Grid(rows))
        return self.local_grid_cache[1], inflation

    def _local_occlusion_reserve(self, raw_hits, reference, start, inflation, config):
        """Reserve unseen depth locally without claiming extra sensor returns.

        A visible face does not establish that the old route is free directly
        behind it. Extend only nearby corridor-blocking hits along the original
        forward tangent; keep actual observations and persistent memory intact.
        """
        reserved = set(raw_hits)
        extension = config["occlusion_extension"]/self.map[0].info.resolution
        if extension <= 0:
            return reserved
        origin = start[0]+.5, start[1]+.5
        progress = path_progress(reference, origin)
        trigger = config["trigger_distance"]/self.map[0].info.resolution
        width, height = self.map[0].info.width, self.map[0].info.height
        for cell in raw_hits:
            center = cell[0]+.5, cell[1]+.5
            if hypot(center[0]-origin[0], center[1]-origin[1]) > trigger:
                continue
            along = path_progress(reference, center)
            point = point_at_progress(reference, along)
            if along < progress-1. or hypot(center[0]-point[0], center[1]-point[1]) > inflation+1.:
                continue
            ahead = point_at_progress(reference, along+1.)
            dx, dy = ahead[0]-point[0], ahead[1]-point[1]
            length = hypot(dx, dy)
            if length < 1e-9:
                continue
            end = center[0]+extension*dx/length, center[1]+extension*dy/length
            reserved.update(cell for cell in route_cells([center, end])
                            if 0 <= cell[0] < width and 0 <= cell[1] < height)
        return reserved

    def _try_local_detour(self, changed_cells):
        """Queue a certified nonempty replacement before invalidating Nav2.

        The existing follower owns cancel/terminal/new-goal ordering. Planning
        needs fresh independent evidence, not an extra stop or fixed delay.
        """
        config = self._local_configuration()
        if config is None or not config["enabled"]:
            return False
        if (not self._goal_is_active()
                or not self.planned_world_points or len(self.local_reference_points) < 2
                or self.local_reference_key != self._local_route_key()
                or self.recovery_active or self.pending_failure_id is not None
                or self.obstacle_hold.holding or self.failure_state == "SENSOR_HOLD"
                or not self._automatic_resume_allowed()):
            return False
        current = self._fresh_motion_pose()
        if current is None or not self._observations_ready():
            return False
        if self.plan_metrics.get("local_detour_state") == "ACTIVE":
            self.plan_metrics.update(local_detour_state="FAILED", detour_reason="active_detour_obstructed")
            return False
        if self.local_detour_attempts >= config["max_attempts"]:
            self.plan_metrics.update(local_detour_state="FAILED", detour_reason="attempt_budget")
            return False
        message = self.map[0]
        resolution = message.info.resolution
        start = world_to_cell(message.info, current[0], current[1])
        trigger = config["trigger_distance"]
        nearby = [cell for cell in changed_cells
                  if hypot(cell_to_world(message.info, cell)[0]-current[0],
                           cell_to_world(message.info, cell)[1]-current[1]) <= trigger]
        if not nearby:
            return False
        began = time.perf_counter()
        grid, inflation = self._local_footprint_grid(config)
        if grid is None:
            return False
        edges, edge_cost = self._failure_constraints(grid)
        yaw = yaw_from_quaternion(message.info.origin.orientation)
        c, sn = cos(yaw), sin(yaw)
        def local(point):
            x, y = point[0]-message.info.origin.position.x, point[1]-message.info.origin.position.y
            return ((c*x+sn*y)/resolution, (-sn*x+c*y)/resolution)
        reference = [local(point) for point in self.local_reference_points]
        actual_hits = self.observed | self.remembered
        reserved_hits = self._local_occlusion_reserve(actual_hits, reference, start, inflation, config)
        # Scan endpoints can cross a raster boundary as the viewing angle
        # changes. Reserve that uncertainty before the unchanged footprint
        # inflation, so a one-cell refinement need not cancel a safe bypass.
        observation_cells = ceil(config["observation_margin"]/resolution)
        planning_hits = inflate_cells(reserved_hits, grid.width, grid.height, observation_cells)
        blocked = inflate_cells(planning_hits, grid.width, grid.height, inflation)
        # A cached static-only field cannot price newly exposed obstacle faces.
        # This mask is a conservative local planning constraint, not new data.
        dynamic_penalty = clearance_penalties(self.raw_static_grid, planning_hits, resolution,
            float(self.get_parameter("preferred_clearance").value),
            float(self.get_parameter("clearance_penalty_gain").value))
        self.local_detour_attempts += 1
        self.local_attempt_id += 1
        try:
            result = plan_local_detour(grid, start, reference, blocked,
                resolution=resolution, blocked_edges=edges, cell_penalty=dynamic_penalty,
                edge_penalty=edge_cost,
                max_rejoin_distance=config["max_rejoin_distance"],
                max_detour_length=config["max_length"],
                max_lateral_distance=config["max_lateral_distance"],
                max_expansions=config["max_expansions"], max_candidates=config["max_candidates"],
                max_seconds=config["max_planning_time"], rejoin_buffer=config["rejoin_buffer"])
        except (ValueError, TypeError, OverflowError) as exc:
            self._invalid_local_configuration(exc)
            return False
        elapsed = (time.perf_counter()-began)*1000.
        self.plan_metrics.update(local_attempt_id=self.local_attempt_id,
            search_duration_ms=result.search_duration_ms, adapter_duration_ms=elapsed,
            expansions=result.expansions, detour_reason=result.reason,
            local_detour_state="FAILED", rejoin_progress=result.rejoin_progress,
            occlusion_reserved_cells=len(reserved_hits-actual_hits),
            occlusion_extension=config["occlusion_extension"], rejoin_buffer=config["rejoin_buffer"],
            observation_reserved_cells=len(planning_hits-reserved_hits),
            observation_margin=config["observation_margin"])
        if not result.points:
            return False
        world = [cell_to_world(message.info, (x-.5, y-.5)) for x, y in result.points]
        # Keep the suffix bit-for-bit, without conversion roundoff or a global
        # smoothing pass moving the original waypoints.
        prefix_count = len(result.points)-(len(reference)-result.rejoin_index)
        world[prefix_count:] = self.local_reference_points[result.rejoin_index:]
        scopes = self._memory_scopes()
        if edges and any(self.failure_memory.segment_blocked(a, b, now=time.monotonic(),
                            map_scope=scopes[0], goal_scope=scopes[1]) for a, b in zip(world, world[1:])):
            self.plan_metrics.update(detour_reason="directed_failure_segment", local_detour_state="FAILED")
            return False
        if self._turn_ready(world, current) is not True:
            self.plan_metrics.update(detour_reason="turn_clearance_unverified", local_detour_state="FAILED")
            return False
        path = Path()
        path.header.frame_id = self.get_parameter("map_frame").value
        now_ns = self.get_clock().now().nanoseconds
        path.header.stamp = rclpy.time.Time(nanoseconds=max(now_ns,
            self.last_failure_event_stamp_ns+1)).to_msg()
        for index, point in enumerate(world):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = point
            if index+1 < len(world):
                angle = atan2(world[index+1][1]-point[1], world[index+1][0]-point[0])
                pose.pose.orientation.z, pose.pose.orientation.w = sin(angle/2.), cos(angle/2.)
            else:
                q = self.last_goal.pose.orientation
                norm = sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                if isfinite(norm) and norm > 1e-9:
                    pose.pose.orientation.x, pose.pose.orientation.y = q.x/norm, q.y/norm
                    pose.pose.orientation.z, pose.pose.orientation.w = q.z/norm, q.w/norm
                else:
                    pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.planned_world_points = world
        self.planned_cells = sorted(route_cells(result.points))
        self.waiting_for_replan = self.invalid_path_published = False
        self.local_rejoin_anchor = {"reference": list(self.local_reference_points),
            "progress": result.rejoin_progress, "pose": current, "odom": self.latest_odom}
        rejoin = self.local_reference_points[result.rejoin_index]
        self.plan_metrics.update(plan_kind="LOCAL_DETOUR", local_detour_state="ACTIVE",
            rejoin_point={"x": rejoin[0], "y": rejoin[1]}, global_plan_count=self.global_plan_count)
        self.path_pub.publish(path)
        self.publish_status("READY", "Local prefix repaired; original forward suffix and goal retained")
        return True

    def _update_local_rejoin(self):
        anchor = self.local_rejoin_anchor
        if anchor is None or not self._goal_is_active() or not self._automatic_resume_allowed():
            return
        pose = self._fresh_motion_pose()
        odom = self.latest_odom
        if pose is None or odom is None or odom[5] != anchor["odom"][5]:
            return
        points = anchor["reference"]
        progress = path_progress(points, pose[:2])
        projection = point_at_progress(points, progress)
        ox, oy = odom[2]-anchor["odom"][2], odom[3]-anchor["odom"][3]
        rotation = anchor["pose"][2]-anchor["odom"][4]
        ex, ey = cos(rotation)*ox-sin(rotation)*oy, sin(rotation)*ox+cos(rotation)*oy
        error = hypot(pose[0]-anchor["pose"][0]-ex, pose[1]-anchor["pose"][1]-ey)
        if (progress >= anchor["progress"]+.10 and hypot(ox, oy) >= .20
                and odom[1] > anchor["odom"][1] and error <= .05+.20*hypot(ox, oy)
                and self.navigation_progress_at is not None
                and time.monotonic()-self.navigation_progress_at < 1.
                and hypot(pose[0]-projection[0], pose[1]-projection[1])
                    <= float(self.get_parameter("rejoin_cross_track_tolerance").value)):
            self.local_detour_attempts = 0
            self.local_rejoin_anchor = None
            self.plan_metrics.update(local_detour_state="REJOINED")
            self.publish_status("READY", "Original suffix rejoin verified by map pose and independent odometry")

    def _automatic_resume_allowed(self):
        status = self.latest_motion_status or {}
        stamp = status.get("sent_at_ns")
        return (self.latest_motion_status_at is not None
                and 0 <= time.monotonic()-self.latest_motion_status_at < 1.0
                and type(stamp) is int
                and -.25e9 <= self.get_clock().now().nanoseconds-stamp < 1e9
                and status.get("mode") == "auto" and status.get("available") is True
                and status.get("autonomous_allowed") is True
                and status.get("estop_latched") is not True
                and status.get("reset_pending") is not True)

    def _register_obstacle_failure(self, x, y, reference_cells=None, *, failure_id=None):
        """Only nearby unmapped physical hits can arm automatic obstacle waiting."""
        if not self.get_parameter("obstacle_hold_enabled").value or self.map is None:
            return False
        raw = self.raw_static_grid or occupancy_to_grid(self.map[0],
                        self.get_parameter("occupied_threshold").value, 0)
        # Observed hits already passed the sensor's range-based wall classifier.
        # Preserve real near-wall objects and confirmed memory; only an actual
        # occupied/unknown map cell cannot be a removable dynamic blocker.
        hits = {cell for cell in self.observed | self.remembered
                if not raw.is_static_obstacle(cell)}
        centers = {cell: cell_to_world(self.map[0].info, cell) for cell in hits}
        if reference_cells is not None:
            info = self.map[0].info
            corridor = inflate_cells(set(reference_cells), info.width, info.height,
                                    max(int(self.get_parameter("obstacle_inflation_cells").value)+1,
                                        ceil((float(self.get_parameter("robot_radius").value)+.05)
                                             / info.resolution)))
            nearby = hits & corridor
        else:
            nearby = {cell for cell, p in centers.items() if hypot(p[0]-x, p[1]-y) < 1.5}
        if not nearby:
            return False
        closest = min(nearby, key=lambda cell: hypot(centers[cell][0]-x, centers[cell][1]-y))
        cx, cy = centers[closest]
        # Track the locally blocking object rather than unrelated room objects.
        tracked = {cell for cell in nearby if hypot(centers[cell][0]-cx, centers[cell][1]-cy) <= .60}
        # The retry budget belongs to the retained goal, but removal evidence
        # belongs to its current blocker. A previously passed object behind the
        # robot must not make the current object's removal impossible to prove.
        self.obstacle_hold.blocked_cells.intersection_update(tracked)
        if failure_id is not None:
            self.obstacle_failure_cells[failure_id] = set(tracked)
        live_ids = {entry["id"] for entry in self.failure_memory.snapshot(
            now=time.monotonic())["entries"]}
        self.obstacle_failure_cells = {key: cells for key, cells in self.obstacle_failure_cells.items()
                                       if key in live_ids}
        info = self.map[0].info
        neighborhood = inflate_cells(tracked, info.width, info.height,
                                     max(1, ceil(.15/info.resolution)))
        self.obstacle_episode_failure_ids = {key for key, cells in self.obstacle_failure_cells.items()
                                            if cells & neighborhood}
        self.obstacle_hold.retry_limit = max(1, int(self.get_parameter("obstacle_retry_limit").value))
        self.obstacle_hold.clear_duration = max(.5, float(self.get_parameter("obstacle_clear_duration").value))
        if not self.obstacle_hold.register_failure(tracked, now=time.monotonic()):
            return False
        self.failure_state, self.failure_reason = "OBSTACLE_HOLD", (
            "장애물이 있어 경로 주행을 잠시 멈춥니다. 장애물을 치우면 경로를 재개합니다.")
        self._invalidate_path(self.failure_reason)
        return True

    def on_visibility(self, msg):
        if not self.obstacle_hold.holding or not self._goal_is_active() or self.map is None:
            return
        now = time.monotonic()
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        current = self.map[0]
        matching = (msg.header.frame_id == current.header.frame_id
                    and msg.info.width == current.info.width and msg.info.height == current.info.height
                    and msg.info.resolution == current.info.resolution
                    and msg.info.origin == current.info.origin
                    and len(msg.data) == current.info.width*current.info.height)
        fresh = (matching and stamp > self.last_visibility_stamp_ns and stamp > 0
                 and -.25e9 <= self.get_clock().now().nanoseconds-stamp < .75e9
                 and self._fresh_motion_pose() is not None and self._automatic_resume_allowed())
        if not fresh:
            self.obstacle_hold.observe_free(set(), set(), fresh=False, now=now)
            return
        self.last_visibility_stamp_ns, self.last_visibility_at = stamp, now
        width = msg.info.width
        # Only inspect the bounded tracked object, not an entire map-sized set.
        tracked = set(self.obstacle_hold.blocked_cells)
        if any(not self.map[1].contains(cell) for cell in tracked):
            self.obstacle_hold.observe_free(set(), set(), fresh=False, now=now)
            return
        free = {cell for cell in tracked if msg.data[cell[1]*width+cell[0]] == 0}
        # Small ordinary AMCL/raster shifts cannot make a nearby stationary
        # object appear removed merely by moving its endpoint to the next cell.
        radius = max(1, ceil(.10/msg.info.resolution))
        neighbors = inflate_cells(tracked, width, msg.info.height, radius)
        near_hits = {cell for cell in neighbors if msg.data[cell[1]*width+cell[0]] > 0}
        occupied = tracked if near_hits else set()
        if not self.obstacle_hold.observe_free(free, occupied, fresh=True, now=now):
            return
        # These cells were positively traversed by fresh valid LiDAR rays for
        # two seconds. Clear only failure IDs associated with this same object.
        self.failure_memory.release_verified(self.obstacle_episode_failure_ids, now=now)
        for key in self.obstacle_episode_failure_ids:
            self.obstacle_failure_cells.pop(key, None)
        self.obstacle_episode_failure_ids.clear()
        self.observed.difference_update(free)
        self.remembered.difference_update(free)
        self.recovery_attempts, self.recovery_attempted_cells = 0, set()
        self.rejoin_anchor, self.rejoin_verified = None, False
        self.failure_state, self.failure_reason = "CLEAR", "장애물 제거 확인 · 기존 목표로 경로를 재생성합니다."
        self.last_plan_time = float("-inf")
        self._request_replan(self.failure_reason)

    def _memory_scopes(self):
        if self.map is None or self.last_goal is None:
            return "", ""
        msg, _ = self.map
        origin = msg.info.origin
        # Map updates do not erase a failed approach. Geometry/frame changes
        # naturally isolate it; goal resends use coordinates, not request stamps.
        map_scope = repr((msg.header.frame_id, msg.info.width, msg.info.height,
                          msg.info.resolution, origin.position.x, origin.position.y,
                          origin.orientation.x, origin.orientation.y,
                          origin.orientation.z, origin.orientation.w))
        goal = self.last_goal
        return map_scope, repr((goal.header.frame_id or self.get_parameter("map_frame").value,
                                goal.pose.position.x, goal.pose.position.y))

    def _failure_constraints(self, grid):
        if not self.get_parameter("failure_memory_enabled").value:
            return set(), {}
        info = self.map[0].info
        geometry = GridGeometry(info.resolution, info.origin.position.x,
                                info.origin.position.y, yaw_from_quaternion(info.origin.orientation))
        map_scope, goal_scope = self._memory_scopes()
        return self.failure_memory.constraints(grid, geometry, now=time.monotonic(),
                                                map_scope=map_scope, goal_scope=goal_scope)

    def on_localization_health(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.localization_health, self.latest_health_at = payload, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_scan(self, msg):
        self.latest_scan, self.latest_scan_at = msg, time.monotonic()

    def _fresh_motion_pose(self):
        """Independent receipt/source age checks; a replay is not fresh motion."""
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        health, odom, scan = self.localization_health or {}, self.latest_odom, self.latest_scan
        sent = health.get("sent_at_ns")
        scan_stamp = (0 if scan is None else scan.header.stamp.sec*1_000_000_000
                      + scan.header.stamp.nanosec)
        if (self.latest_health_at is None or not 0 <= now-self.latest_health_at < 1.0
                or isinstance(sent, bool) or not isinstance(sent, int)
                or not -.25e9 <= ros_ns-sent < 1e9
                or health.get("level") != "ok" or health.get("autonomous_allowed") is not True
                or health.get("stop_latched") is True or odom is None
                or not 0 <= now-odom[0] < .75 or not -.25e9 <= ros_ns-odom[1] < 1e9
                or scan is None or self.latest_scan_at is None
                or not 0 <= now-self.latest_scan_at < .75
                or scan_stamp <= 0 or not -.25e9 <= ros_ns-scan_stamp < 1e9
                or not scan.ranges or not isfinite(scan.range_max) or scan.range_max <= 0
                or not isfinite(scan.range_min) or not 0 <= scan.range_min < scan.range_max
                or not isfinite(scan.angle_min) or not isfinite(scan.angle_increment)
                or scan.angle_increment == 0
                or sum((isfinite(v) and scan.range_min <= v <= scan.range_max)
                       or v == float("inf") for v in scan.ranges)/len(scan.ranges) < .8):
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter("map_frame").value,
                    self.get_parameter("base_frame").value, rclpy.time.Time())
            stamp = tf.header.stamp.sec*1_000_000_000 + tf.header.stamp.nanosec
            p, q = tf.transform.translation, tf.transform.rotation
            if (stamp <= 0 or not -.25e9 <= ros_ns-stamp < 1e9
                    or not all(isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                    or not .8 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.2):
                return None
            return float(p.x), float(p.y), yaw_from_quaternion(q)
        except (TransformException, ValueError, AttributeError):
            return None

    def on_execution_failure(self, msg):
        if not self._goal_is_active() or not self.get_parameter("failure_memory_enabled").value:
            return
        # A terminal result may arrive before the latched map on startup.
        # Without map geometry there is no safe scope or corridor for its
        # failure memory, so retain the goal and wait for /map instead.
        if self.map is None:
            self._invalidate_path("Waiting for /map after execution failure")
            return
        try:
            data = json.loads(msg.data)
            event_id, created = data.get("event_id"), data.get("created_at_ns")
            if (type(data.get("schema_version")) is not int or data["schema_version"] != 1
                    or data.get("terminal_confirmed") is not True
                    or not isinstance(event_id, str) or not 1 <= len(event_id) <= 128
                    or event_id in self.seen_failure_ids or not self._status_matches_goal(data)
                    or data.get("reason") not in EXECUTION_FAILURE_REASONS
                    or isinstance(created, bool) or not isinstance(created, int)
                    or not -.25e9 <= self.get_clock().now().nanoseconds-created
                    <= float(self.get_parameter("failure_event_max_age").value)*1e9):
                return
            pose, segment = data["pose"], data["segment"]
            values = (pose["x"], pose["y"], pose["yaw"], segment["heading"],
                      segment["start"]["x"], segment["start"]["y"],
                      segment["end"]["x"], segment["end"]["y"])
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v)
                   for v in values):
                return
            current = self._fresh_motion_pose()
            if current is None or hypot(current[0]-pose["x"], current[1]-pose["y"]) > .35:
                self.failure_state, self.failure_reason = "SENSOR_HOLD", "Failure evidence is not fresh"
                return
        except (ValueError, TypeError, KeyError, AttributeError):
            return
        # Place the restriction ahead of the stopped body, along the attempted
        # route. Retreat and departure in a different direction remain possible.
        heading = segment["heading"]
        map_scope, goal_scope = self._memory_scopes()
        self.last_failure_id = self.failure_memory.record_failure(
            x=pose["x"]+.20*cos(heading), y=pose["y"]+.20*sin(heading),
            approach_heading=heading, reason=data["reason"], now=time.monotonic(),
            map_scope=map_scope, goal_scope=goal_scope, failure_id=event_id)
        self.seen_failure_ids.append(event_id)
        self.last_failure_event_stamp_ns = created
        self.pending_failure_id = self.pending_failure_since = None
        self.rejoin_anchor, self.rejoin_verified = None, False
        self.failure_state, self.failure_reason = "AVOIDING", data["reason"]
        self._invalidate_path("Execution failure remembered; seeking a different approach")
        # A Nav2 progress abort is not itself proof that a physical object
        # blocked the chassis. Only count an observed hit intersecting the
        # attempted local segment (with the configured footprint margin).
        if self.map is not None:
            info = self.map[0].info
            start_cell = world_to_cell(info, segment["start"]["x"], segment["start"]["y"])
            end_cell = world_to_cell(info, segment["end"]["x"], segment["end"]["y"])
            attempted_cells = route_cells([(start_cell[0]+.5, start_cell[1]+.5),
                                           (end_cell[0]+.5, end_cell[1]+.5)])
            if self._register_obstacle_failure(pose["x"], pose["y"], attempted_cells,
                                               failure_id=self.last_failure_id):
                return
        self.last_plan_time = float("-inf")
        self._request_replan("Execution failure recorded")

    def _remaining_path_cells(self):
        """Only newly blocked *remaining* route cancels a healthy active action."""
        points = self.planned_world_points
        if len(points) < 2 or self.map is None:
            return self.planned_cells
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter("map_frame").value,
                    self.get_parameter("base_frame").value, rclpy.time.Time())
            stamp = tf.header.stamp.sec*1_000_000_000 + tf.header.stamp.nanosec
            if not -.25e9 <= self.get_clock().now().nanoseconds-stamp < 1e9:
                return self.planned_cells
            p = tf.transform.translation
            progress = max(0., path_progress(points, (p.x, p.y))
                           -float(self.get_parameter("robot_radius").value)-.05)
            remaining = [point_at_progress(points, progress)]
            traveled = 0.
            for a, b in zip(points, points[1:]):
                traveled += hypot(b[0]-a[0], b[1]-a[1])
                if traveled >= progress:
                    remaining.append(b)
            info = self.map[0].info
            c, s = cos(yaw_from_quaternion(info.origin.orientation)), sin(yaw_from_quaternion(info.origin.orientation))
            def local(point):
                x, y = point[0]-info.origin.position.x, point[1]-info.origin.position.y
                return ((c*x+s*y)/info.resolution, (-s*x+c*y)/info.resolution)
            return list(route_cells([local(point) for point in remaining]))
        except (TransformException, AttributeError, ValueError):
            return self.planned_cells

    def _turn_ready(self, world_points, pose):
        if len(world_points) < 2:
            return True
        target = point_at_progress(world_points, .30)
        heading = atan2(target[1]-pose[1], target[0]-pose[0])
        if abs(self._angle_delta(heading, pose[2])) < float(self.get_parameter("recovery_turn_heading_threshold").value):
            return True
        scan, now = self.latest_scan, time.monotonic()
        if scan is None or self.latest_scan_at is None or now-self.latest_scan_at > .75:
            return None
        stamp = scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
        if not -.25e9 <= self.get_clock().now().nanoseconds-stamp < 1e9:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter("base_frame").value,
                      scan.header.frame_id, rclpy.time.Time.from_msg(scan.header.stamp))
            p, q = tf.transform.translation, tf.transform.rotation
            safe, clearance = rotation_clearance(scan, (p.x, p.y, yaw_from_quaternion(q)),
                radius=float(self.get_parameter("robot_radius").value)
                      +float(self.get_parameter("recovery_turn_clearance_margin").value))
            # Unknown angular coverage cannot authorize a reverse manoeuvre.
            return safe if clearance is not None else None
        except (TransformException, ValueError):
            return None

    def _update_rejoin(self):
        self._update_local_rejoin()
        anchor = self.rejoin_anchor
        if anchor is None or self.rejoin_verified or not self._goal_is_active():
            return
        pose = self._fresh_motion_pose()
        if pose is None:
            return
        points, odom = anchor["points"], self.latest_odom
        if len(points) < 2 or odom[5] != anchor["odom"][5]:
            return
        progress = path_progress(points, pose[:2])
        projection = point_at_progress(points, progress)
        distance = float(self.get_parameter("rejoin_progress_distance").value)
        # Compare a chassis displacement in both coordinate systems; neither
        # an AMCL correction nor an odometry reset alone proves a safe rejoin.
        ox, oy = odom[2]-anchor["odom"][2], odom[3]-anchor["odom"][3]
        rotation = anchor["pose"][2]-anchor["odom"][4]
        ex, ey = cos(rotation)*ox-sin(rotation)*oy, sin(rotation)*ox+cos(rotation)*oy
        error = hypot(pose[0]-anchor["pose"][0]-ex, pose[1]-anchor["pose"][1]-ey)
        if (progress-anchor["progress"] >= distance
                and hypot(odom[2]-anchor["odom"][2], odom[3]-anchor["odom"][3]) >= min(.25, distance)
                and odom[1] > anchor["odom"][1]
                and error <= .05+.20*hypot(ox, oy)
                and self.navigation_progress_at is not None
                and time.monotonic()-self.navigation_progress_at < 1.0
                and hypot(pose[0]-projection[0], pose[1]-projection[1])
                    <= float(self.get_parameter("rejoin_cross_track_tolerance").value)):
            self.rejoin_verified = True
            self.recovery_attempts, self.recovery_attempted_cells = 0, set()
            self.failure_state, self.failure_reason = "AVOIDING", "Alternative route progress verified"

    def publish_failure_status(self):
        # Steady heartbeat also expires memory while simulation /clock is paused.
        try:
            settings = dict(ttl_sec=float(self.get_parameter("failure_memory_ttl").value),
                radius_m=float(self.get_parameter("failure_memory_radius").value),
                heading_tolerance_rad=float(self.get_parameter("failure_memory_heading_tolerance").value),
                max_entries=int(self.get_parameter("failure_memory_max_entries").value),
                penalty_cost=float(self.get_parameter("failure_memory_penalty").value))
            validated = FailureMemory(**settings)
            for name in settings:
                setattr(self.failure_memory, name, getattr(validated, name))
            map_scope, goal_scope = self._memory_scopes()
            snapshot = self.failure_memory.snapshot(now=time.monotonic(),
                map_scope=map_scope, goal_scope=goal_scope)
            self._update_rejoin()
            active = [{**entry, "expires_in": entry["expires_in_sec"],
                       "failures": entry["occurrences"]} for entry in snapshot["entries"]]
            state = self.failure_state
            if not self._goal_is_active() or not self.get_parameter("failure_memory_enabled").value:
                state = "CLEAR"
                active = []
            elif active and self._fresh_motion_pose() is None:
                state = "SENSOR_HOLD"
            elif self.recovery_active:
                state = "RECOVERING"
            elif self.obstacle_hold.holding:
                state = "OBSTACLE_HOLD" if self._fresh_motion_pose() is not None else "SENSOR_HOLD"
            elif (not active and self.rejoin_anchor is None
                  and state in ("CLEAR", "AVOIDING")):
                state = "CLEAR"
            packet = String()
            goal = self.last_goal
            packet.data = json.dumps({"state": state, "reason": self.failure_reason,
                "count": len(active), "active": active, "last_failure_id": self.last_failure_id,
                "rejoin_verified": self.rejoin_verified, "recovery_attempts": self.recovery_attempts,
                "goal_stamp": None if goal is None else {"sec": goal.header.stamp.sec,
                                                         "nanosec": goal.header.stamp.nanosec},
                "sent_at_ns": self.get_clock().now().nanoseconds})
            detail = json.loads(packet.data)
            hold = self.obstacle_hold.snapshot(now=time.monotonic())
            detail.update(retry_count=hold["attempts"], retry_limit=hold["retry_limit"],
                blocked_obstacle_count=hold["tracked_cell_count"],
                clear_wait_seconds=max(0., hold["clear_duration"]-hold["clear_elapsed"]))
            packet.data = json.dumps(detail)
            self.failure_status_pub.publish(packet)
            if (self._goal_is_active() and self.planned_world_points and not self.waiting_for_replan
                    and self.plan_metrics.get("local_detour_state") == "ACTIVE"):
                self.publish_status("READY", "Local detour active; original forward suffix retained")
        except (ValueError, TypeError):
            self.get_logger().warning("Invalid failure-memory limits; keeping last valid settings",
                                      throttle_duration_sec=5.)

    def _retry_period(self):
        # Keep retry rate bounded even if a mistaken runtime parameter is zero.
        return max(0.25, float(self.get_parameter("replan_retry_period").value))

    def retry_transform(self):
        """Retry no-path, missing-map and missing-TF conditions without losing the goal."""
        if self.obstacle_hold.holding:
            if (self.last_visibility_at is None or time.monotonic()-self.last_visibility_at > .5
                    or self._fresh_motion_pose() is None or not self._automatic_resume_allowed()):
                self.obstacle_hold.observe_free(set(), set(), fresh=False, now=time.monotonic())
            return
        if self.pending_failure_id is not None:
            # FAILED and the reliable event use different topics. Never retry
            # the old route while its spatial evidence is still uncorrelated.
            if (self.pending_failure_since is None
                    or time.monotonic()-self.pending_failure_since < 3.0
                    or self._fresh_motion_pose() is None):
                return
            # Lost/rejected evidence cannot poison the map or permanently lose
            # the mission. Retry once sensors recover, without inventing memory.
            self.pending_failure_id = self.pending_failure_since = None
            self.failure_state, self.failure_reason = "WAIT_ALTERNATIVE", "Unconfirmed failure; retrying with fresh sensors"
        if self.map is not None:
            signature = (self.get_parameter("occupied_threshold").value,
                         self.get_parameter("obstacle_inflation_cells").value,
                         self.get_parameter("mapped_wall_return_tolerance_cells").value)
            if signature != getattr(self, "map_signature", None):
                self.on_map(self.map[0])
        geometry = tuple(self.get_parameter(name).value for name in (
            "path_smoothing_enabled", "path_smoothing_iterations",
            "path_smoothing_shortcut_distance", "path_resample_spacing"))
        if geometry != getattr(self, "geometry_signature", None):
            self.geometry_signature = geometry
            if self._goal_is_active():
                self._request_replan("Path geometry settings changed")
        if self._goal_is_active() and not getattr(self, "recovery_active", False):
            self._record_breadcrumb()
        if getattr(self, "recovery_active", False):
            requested = self.recovery_requested_at
            # The follower has a 5 s cancel wait plus a 15 s BackUp watchdog.
            # If no correlated result arrives, release the retained goal from
            # an otherwise permanent RECOVERING hold. The attempted cell stays
            # marked so a lost response cannot trigger repeated reversals.
            if requested is not None and time.monotonic() - requested > 25.0:
                self.recovery_active = False
                self.recovery_requested_at = None
                self.recovery_cancel_due_map_change = False
                self._clear_breadcrumbs()
                self._invalidate_path("Reverse recovery timed out; goal retained")
                self.last_plan_time = time.monotonic()
            return
        if self._goal_is_active() and not self._observations_ready():
            self._invalidate_path("Waiting for fresh LiDAR obstacle observations; goal retained")
            return
        if self.waiting_for_replan and self._goal_is_active():
            self._request_replan("Retrying retained goal")

    def _observations_ready(self):
        if not self.get_parameter("live_obstacle_layer_required").value:
            return True
        timeout = max(.25, float(self.get_parameter("live_obstacle_timeout").value))
        return (self.last_observation_time is not None
                and time.monotonic()-self.last_observation_time < timeout)

    def _request_replan(self, reason):
        if (not self._goal_is_active() or getattr(self, "recovery_active", False)
                or self.pending_failure_id is not None or self.obstacle_hold.holding):
            return
        self.waiting_for_replan = True
        if time.monotonic() - self.last_plan_time >= self._retry_period():
            self.plan_to(self.last_goal)

    def _invalidate_path(self, reason):
        # One empty Path cancels the unsafe action. Repeated retries must not
        # continuously cancel a replacement action or flood navigation_status.
        self.planned_cells = []
        self.planned_world_points = []
        if self.plan_metrics.get("local_detour_state") == "ACTIVE":
            self.plan_metrics.update(local_detour_state="FAILED", detour_reason=reason)
            self.local_rejoin_anchor = None
        if not self.invalid_path_published:
            path = Path()
            path.header.frame_id = self.get_parameter("map_frame").value
            path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(path)
            self.invalid_path_published = True
        self.waiting_for_replan = True
        self.publish_status("WAIT_REPLAN", reason)

    def publish_status(self, status, message):
        goal = self.last_goal
        payload = {"status": status, "message": message, "goal": None, "goal_stamp": None,
                   **self.plan_metrics}
        clock = self.get_clock().now()
        payload["sent_at_ns"] = getattr(clock, "nanoseconds", 0)
        if goal is not None:
            payload["goal"] = {"x": goal.pose.position.x, "y": goal.pose.position.y,
                               "frame_id": goal.header.frame_id or self.get_parameter("map_frame").value}
            payload["goal_stamp"] = {"sec": goal.header.stamp.sec,
                                     "nanosec": goal.header.stamp.nanosec}
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def on_navigation_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        if payload.get("manual_stop"):
            stopped = payload.get("stop_stamp") or {}
            if "sec" not in stopped or "nanosec" not in stopped:
                return
            boundary = stopped["sec"] * 1_000_000_000 + stopped["nanosec"]
            self.stop_stamp_ns = max(boundary, self.stop_stamp_ns or 0)
            goal_stamp = (0 if self.last_goal is None else
                          self.last_goal.header.stamp.sec * 1_000_000_000
                          + self.last_goal.header.stamp.nanosec)
            # /goal and the cancel service have no cross-topic ordering. A stop
            # arriving late must not cancel a deliberately newer stamped goal.
            if goal_stamp == 0 or goal_stamp <= self.stop_stamp_ns:
                self.navigation_suspended = True
                self._reset_local_reference()
                self.recovery_active = False
                self.recovery_requested_at = None
                self.recovery_cancel_due_map_change = False
                self._clear_breadcrumbs()
                self.waiting_for_replan = False
                self.pending_failure_id = None
                self.rejoin_anchor = None
                self.obstacle_hold.reset()
                self.obstacle_episode_failure_ids.clear()
                self.obstacle_failure_cells.clear()
                self.publish_status("IDLE", "Manually stopped; send a newly ROS-stamped goal to resume")
            return
        if self.last_goal is None or not self._status_matches_goal(payload):
            return
        if payload.get("status") == "MOVING":
            self.navigation_progress_at = time.monotonic()
        recovery_status = payload.get("status")
        if recovery_status == "RECOVERING":
            self.publish_status("RECOVERING", "Verified short reverse is in progress")
            return
        if recovery_status == "RECOVERY_STOPPED":
            # The follower has now observed the BackUp action's terminal
            # result. Any route planned while cancellation was still pending
            # may have been discarded, so plan again from the current pose.
            self.recovery_active = False
            self.recovery_requested_at = None
            self.recovery_cancel_due_map_change = False
            self._clear_breadcrumbs()
            if self._goal_is_active():
                self.last_plan_time = float("-inf")
                self._request_replan("Reverse action stopped; replanning retained goal")
            return
        if recovery_status in ("RECOVERY_SUCCEEDED", "RECOVERY_FAILED"):
            if not getattr(self, "recovery_active", False):
                return
            self.recovery_active = False
            self.recovery_requested_at = None
            self.recovery_cancel_due_map_change = False
            self._clear_breadcrumbs()
            # Backing up changes the start pose. Replan from fresh TF, retaining
            # the same explicit mission goal. Failed recovery is never repeated
            # from the same cell because it is recorded in attempted_cells.
            if recovery_status == "RECOVERY_SUCCEEDED":
                self.last_plan_time = float("-inf")
                self._request_replan("Reverse recovery finished; replanning")
            else:
                self._invalidate_path("Reverse recovery failed; goal retained for safe replanning")
                self.last_plan_time = time.monotonic()
            return
        if payload.get("status") == "SUCCEEDED":
            self.goal_completed = True
            self._reset_local_reference()
            self.waiting_for_replan = False
            self.pending_failure_id = None
            self.rejoin_anchor = None
            self.failure_state = "CLEAR"
            self.obstacle_hold.reset()
            self.obstacle_episode_failure_ids.clear()
            self.obstacle_failure_cells.clear()
            self.publish_status("IDLE", "Goal completed")
        elif payload.get("status") == "FAILED" and self._goal_is_active():
            # Nav2 may abort while the geometric route is unchanged. Retry the
            # retained goal; the follower discards its failed duplicate cache.
            if payload.get("route_failure") is True:
                event_id = payload.get("execution_failure_id")
                if event_id in self.seen_failure_ids:
                    return  # Correlated event already replanned this goal.
                if isinstance(event_id, str) and event_id:
                    self.pending_failure_id = event_id
                    self.pending_failure_since = time.monotonic()
                    self.failure_state, self.failure_reason = "WAIT_ALTERNATIVE", "Waiting for terminal failure evidence"
            self._invalidate_path("Controller failed; waiting to replan retained goal")
            self.last_plan_time = time.monotonic()
        elif (payload.get("status") == "WAIT_REPLAN" and payload.get("route_failure") is False
              and self._goal_is_active()):
            # A sensor fault during terminal cancellation disproves the stored
            # motion diagnosis, but must not strand the retained mission goal.
            self.pending_failure_id = self.pending_failure_since = None
            self.failure_state, self.failure_reason = "SENSOR_HOLD", "Sensor recovery required; no failure recorded"
            self._invalidate_path(self.failure_reason)
            self.last_plan_time = time.monotonic()

    def _status_matches_goal(self, payload):
        target = payload.get("goal") or {}
        if not isinstance(target, dict):
            return False
        goal = self.last_goal
        frame = goal.header.frame_id or self.get_parameter("map_frame").value
        if (target.get("frame_id") != frame
                or target.get("x") != goal.pose.position.x
                or target.get("y") != goal.pose.position.y):
            return False
        return payload.get("goal_stamp") == {"sec": goal.header.stamp.sec,
                                              "nanosec": goal.header.stamp.nanosec}

    def on_memory(self, msg):
        if self.map is None:
            return
        width, height = self.map[0].info.width, self.map[0].info.height
        updated = {(index % width, index // width) for index in msg.data
                   if 0 <= index < width * height}
        if updated != self.remembered:
            added = updated - self.remembered
            removed = self.remembered - updated
            self.remembered = updated
            # Empty-route recovery must also react to obstacle removal. While
            # driving, unrelated memory churn must not cancel a valid Nav2 goal.
            if self._goal_is_active() and (self.waiting_for_replan
                    or self._changes_block_path(added) or self._changes_block_path(removed)):
                if self._changes_block_path(added):
                    if self._try_local_detour(added):
                        return
                    if self._count_blocked_published_path():
                        return
                    self._invalidate_path("Remembered obstacle intersects the route")
                self._request_replan("Remembered obstacles changed")

    def on_observed(self, msg):
        if self.map is None:
            return
        self.last_observation_time = time.monotonic()
        width, height = self.map[0].info.width, self.map[0].info.height
        updated = {(index % width, index // width) for index in msg.data
                   if 0 <= index < width * height}
        if updated != self.observed:
            added = updated - self.observed
            self.observed = updated
            # Laser scans vary at sensor frequency. Stop on a newly blocked
            # route, and rate-limit recovery even when no planned cells remain.
            # The publisher already classified static wall echoes by beam range.
            if self._goal_is_active() and (self.waiting_for_replan
                    or self._changes_block_path(added)):
                if self._changes_block_path(added):
                    if self._try_local_detour(added):
                        return
                    if self._count_blocked_published_path():
                        return
                    self._invalidate_path("Observed obstacle intersects the route")
                self._request_replan("Observed obstacles changed")
        elif self.waiting_for_replan and self._goal_is_active():
            # Fresh empty/unchanged scans also recover a stale-sensor hold.
            self._request_replan("LiDAR obstacle observations restored")

    def _count_blocked_published_path(self):
        # One attempted published approach, never individual sensor packets:
        # invalidation clears planned_cells immediately after the first hit.
        # This also bounds paths canceled too quickly for the 6 s detector.
        if not self.planned_cells or not self._automatic_resume_allowed():
            return False
        pose = self._fresh_motion_pose()
        return (pose is not None and self._register_obstacle_failure(
            pose[0], pose[1], self._remaining_path_cells()))

    def _changes_block_path(self, cells, *, filter_near_wall=False):
        if not cells or not self.planned_cells:
            return False
        width, height = self.map[0].info.width, self.map[0].info.height
        inflation = self.get_parameter("obstacle_inflation_cells").value
        effective = (self._split_mapped_wall_returns(cells)[0]
                     if filter_near_wall else cells)
        blocked = inflate_cells(effective, width, height, inflation)
        return any(cell in blocked for cell in self._remaining_path_cells())

    def _split_mapped_wall_returns(self, cells):
        raw_grid = getattr(self, "raw_static_grid", None)
        if raw_grid is None:
            map_msg = self.map[0]
            raw_grid = occupancy_to_grid(map_msg,
                                         self.get_parameter("occupied_threshold").value, 0)
            self.raw_static_grid = raw_grid
        return split_mapped_wall_returns(
            cells, raw_grid,
            self.get_parameter("mapped_wall_return_tolerance_cells").value)

    def on_goal(self, goal):
        stamp_ns = goal.header.stamp.sec * 1_000_000_000 + goal.header.stamp.nanosec
        if self.stop_stamp_ns is not None and (stamp_ns == 0 or stamp_ns <= self.stop_stamp_ns):
            self.get_logger().warning(
                "Ignored delayed/unstamped goal after manual stop; send a newly ROS-stamped goal")
            return
        # A delivery may start the next sector directly from the previous one.
        # Preserve that same physical trail across normal goal boundaries; the
        # reverse attempt budget, however, belongs to the new explicit goal.
        # An interrupted reverse has no known end pose and invalidates it.
        if self.recovery_active:
            self._clear_breadcrumbs()
        self.last_goal = goal
        self._reset_local_reference()
        self.navigation_suspended = False
        self.goal_completed = False
        self.obstacle_hold.ensure_goal_scope(repr(self._memory_scopes()))
        self.last_visibility_at = None
        if self.obstacle_hold.holding:
            self.failure_state = "OBSTACLE_HOLD"
            self.invalid_path_published = False
            self._invalidate_path(self.failure_reason)
            return
        if self.obstacle_hold.attempts == 0:
            self.obstacle_episode_failure_ids.clear()
            self.obstacle_failure_cells.clear()
        self.pending_failure_id = None
        self.pending_failure_since = None
        self.rejoin_anchor, self.rejoin_verified = None, False
        self.navigation_progress_at = None
        self.last_failure_id = None
        self.last_failure_event_stamp_ns = 0
        self.failure_state, self.failure_reason = "CLEAR", ""
        self.planned_world_points = []
        self.navigation_suspended = False
        self.goal_completed = False
        self.recovery_active = False
        self.recovery_requested_at = None
        self.recovery_cancel_due_map_change = False
        self.recovery_attempts = 0
        self.recovery_attempted_cells = set()
        # An explicit request is a new attempt even when coordinates are identical.
        self.invalid_path_published = False
        self._record_breadcrumb()
        self.plan_to(goal)

    def _clear_breadcrumbs(self):
        self.breadcrumbs.clear()
        self.breadcrumb_odom_anchor = None

    def on_odom(self, msg):
        """Keep the latest wheel/IMU odometry as an independent motion witness."""
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        pose = msg.pose.pose
        sample = (time.monotonic(), stamp_ns, float(pose.position.x),
                  float(pose.position.y), yaw_from_quaternion(pose.orientation),
                  msg.header.frame_id)
        if (stamp_ns <= 0 or not all(isfinite(value) for value in sample[:5])
                or not sample[5]):
            self.latest_odom = None
            self.odom_samples.clear()
            self._clear_breadcrumbs()
            return
        self.latest_odom = sample
        self.odom_samples.append(sample)

    @staticmethod
    def _angle_delta(a, b):
        return (a - b + pi) % (2.0 * pi) - pi

    def _record_breadcrumb(self, transform=None):
        """Record map poses only when fresh, synchronized odometry verifies travel.

        AMCL may move ``map -> odom`` while the chassis is stationary. Using TF
        alone would turn that correction into a fictitious retreat trail.
        """
        if not self._goal_is_active():
            return False
        if transform is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter("map_frame").value,
                    self.get_parameter("base_frame").value, rclpy.time.Time())
            except TransformException:
                self._clear_breadcrumbs()
                return False
        stamp = getattr(getattr(transform, "header", None), "stamp", None)
        now = time.monotonic()
        if stamp is None or not self.odom_samples:
            self._clear_breadcrumbs()
            return False
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        # Choose the odometry sample closest to the map TF timestamp. The
        # newest /odom can describe motion that the delayed AMCL TF has not
        # reached yet, so comparing those two would reject real travel.
        odom = min(self.odom_samples, key=lambda item: abs(item[1] - stamp_ns))
        now_ns = getattr(self.get_clock().now(), "nanoseconds", None)
        if (stamp_ns <= 0 or now - odom[0] > .75 or now < odom[0]
                or abs(stamp_ns - odom[1]) > 150_000_000
                or (now_ns is not None and (now_ns - stamp_ns > 1_000_000_000
                                            or stamp_ns - now_ns > 200_000_000))):
            self._clear_breadcrumbs()
            return False
        translation = transform.transform.translation
        rotation = getattr(transform.transform, "rotation", None)
        if rotation is None:
            self._clear_breadcrumbs()
            return False
        yaw = yaw_from_quaternion(rotation)
        sample = Breadcrumb(now, float(translation.x),
                            float(translation.y), yaw)
        if not all(isfinite(value) for value in (sample.x, sample.y, sample.yaw)):
            self._clear_breadcrumbs()
            return False
        history = self.breadcrumbs
        anchor = self.breadcrumb_odom_anchor
        if not history or anchor is None:
            self._clear_breadcrumbs()
            history.append(sample)
            self.breadcrumb_odom_anchor = odom
            return True
        previous = history[-1]
        distance = hypot(sample.x - previous.x, sample.y - previous.y)
        turn = self._angle_delta(sample.yaw, previous.yaw)
        odom_dx, odom_dy = odom[2] - anchor[2], odom[3] - anchor[3]
        odom_distance = hypot(odom_dx, odom_dy)
        odom_turn = self._angle_delta(odom[4], anchor[4])
        # Express the odometry delta in the map heading at the anchor. A map
        # correction moves the TF pose but does not move this odometry delta.
        rotation_to_map = previous.yaw - anchor[4]
        expected_dx = cos(rotation_to_map) * odom_dx - sin(rotation_to_map) * odom_dy
        expected_dy = sin(rotation_to_map) * odom_dx + cos(rotation_to_map) * odom_dy
        error = hypot(sample.x - previous.x - expected_dx,
                      sample.y - previous.y - expected_dy)
        odom_forward = odom_dx * cos(anchor[4]) + odom_dy * sin(anchor[4])
        odom_lateral = abs(-odom_dx * sin(anchor[4]) + odom_dy * cos(anchor[4]))
        map_forward = ((sample.x - previous.x) * cos(previous.yaw)
                       + (sample.y - previous.y) * sin(previous.yaw))
        if (odom[5] != anchor[5] or odom[1] < anchor[1]
                or distance > .15 or odom_distance > .15
                or abs(turn) > .15 or abs(odom_turn) > .15
                or abs(turn - odom_turn) > .08
                or error > .015 + .20 * odom_distance
                or odom_lateral > .015 + .20 * odom_distance
                or (distance >= .025 and (odom_forward < .015 or map_forward < .015))
                or odom_forward < -.005):
            self._clear_breadcrumbs()
            history.append(sample)
            self.breadcrumb_odom_anchor = odom
            return True
        if distance < .025 or odom_distance < .025:
            # Keep the verified location, not an AMCL-only correction. Refresh
            # its age while stationary; movement accumulates from the anchor.
            history[-1] = Breadcrumb(now, previous.x, previous.y, previous.yaw)
            return True
        history.append(sample)
        self.breadcrumb_odom_anchor = odom
        return True

    def _maybe_request_reverse(self, start, end, map_msg, grid, raw_grid,
                               raw_dynamic, inflated_dynamic, cell_penalty,
                               trail_verified):
        """Request a bounded Nav2 backup only when a verified A* exit exists."""
        if (not trail_verified or not self.get_parameter("reverse_recovery_enabled").value
                or getattr(self, "recovery_active", False)
                or start in self.recovery_attempted_cells
                or self.recovery_attempts >= int(self.get_parameter("reverse_recovery_max_attempts").value)
                or not self._observations_ready()
                or grid.is_static_obstacle(end) or end in inflated_dynamic):
            return False

        # Dynamic raw scan hits and confirmed memory have the same hard-stop
        # status as raw map occupancy while checking the reverse swept volume.
        raw_with_live = Grid(tuple(tuple(raw_grid.rows[y][x] or (x, y) in raw_dynamic
                                        for x in range(raw_grid.width))
                                   for y in range(raw_grid.height)))
        inflated_with_live = Grid(tuple(tuple(grid.rows[y][x] or (x, y) in inflated_dynamic
                                             for x in range(grid.width))
                                        for y in range(grid.height)))
        origin = map_msg.info.origin
        geometry = GridGeometry(map_msg.info.resolution, origin.position.x,
                                origin.position.y, yaw_from_quaternion(origin.orientation))

        failure_edges, failure_costs = self._failure_constraints(grid)

        def can_reach_retained_goal(candidate_cell):
            return bool(self.planner.plan(grid, candidate_cell, end, inflated_dynamic,
                                          cell_penalty=cell_penalty,
                                          blocked_edges=failure_edges, edge_penalty=failure_costs))

        candidate = select_reverse_egress(
            tuple(self.breadcrumbs), raw_with_live, inflated_with_live, geometry,
            can_reach_retained_goal, now=time.monotonic(),
            robot_radius=float(self.get_parameter("robot_radius").value),
            clearance_margin=float(self.get_parameter("reverse_recovery_clearance_margin").value),
            min_distance=float(self.get_parameter("reverse_recovery_min_distance").value),
            max_distance=float(self.get_parameter("reverse_recovery_max_distance").value),
            max_history_age=90.0, max_sample_gap=30.0)
        if candidate is None:
            return False

        self.recovery_attempts += 1
        self.recovery_attempted_cells.add(start)
        self.recovery_active = True
        self.recovery_requested_at = time.monotonic()
        self.failure_state, self.failure_reason = "RECOVERING", "Verified recent track retreat"
        goal = self.last_goal
        request = String()
        request.data = json.dumps({
            "goal_id": f"{goal.header.stamp.sec}:{goal.header.stamp.nanosec}",
            "distance": candidate.distance_m,
            "speed": float(self.get_parameter("reverse_recovery_speed").value),
            "reason": "No forward A* route; verified recent straight track leads to a safe A* start",
            "start_cell": list(start), "exit_cell": list(candidate.target_cell)})
        self.recovery_pub.publish(request)
        self.publish_status("RECOVERING", f"Backing up {candidate.distance_m:.2f} m to replan retained goal")
        self.get_logger().warning(
            f"No forward route; requested collision-checked {candidate.distance_m:.2f} m reverse "
            f"to {candidate.target_cell} (attempt {self.recovery_attempts})")
        return True

    def plan_to(self, goal):
        if (getattr(self, "recovery_active", False) or self.pending_failure_id is not None
                or self.obstacle_hold.holding):
            return
        if self.failure_state == "SENSOR_HOLD" and self._fresh_motion_pose() is None:
            self._invalidate_path("Waiting for fresh independent sensor evidence before retry")
            return
        self.last_plan_time = time.monotonic()
        global_started = time.perf_counter()
        self.plan_metrics.update(plan_kind="GLOBAL",
            local_detour_state="FAILED" if self.plan_metrics.get("local_detour_state") == "FAILED" else "NONE")
        self.geometry_signature = tuple(self.get_parameter(name).value for name in (
            "path_smoothing_enabled", "path_smoothing_iterations",
            "path_smoothing_shortcut_distance", "path_resample_spacing"))
        self.waiting_for_replan = True
        self.publish_status("PLANNING", "Planning to retained goal")
        if self.map is None:
            self.waiting_for_transform = True
            self._invalidate_path("Waiting for /map")
            return
        if not self._observations_ready():
            self._invalidate_path("Waiting for fresh LiDAR obstacle observations; goal retained")
            return
        map_msg, grid = self.map
        signature = (self.get_parameter("occupied_threshold").value,
                     self.get_parameter("obstacle_inflation_cells").value,
                     self.get_parameter("mapped_wall_return_tolerance_cells").value)
        if signature != getattr(self, "map_signature", None):
            grid = occupancy_to_grid(map_msg, signature[0], signature[1])
            self.map = (map_msg, grid)
            self.raw_static_grid = occupancy_to_grid(map_msg, signature[0], 0)
            self.map_signature = signature
        raw_grid = self.raw_static_grid
        frame = self.get_parameter("map_frame").value
        if goal.header.frame_id and goal.header.frame_id != frame:
            self._invalidate_path(f"Goal frame must be {frame}, got {goal.header.frame_id}")
            self.waiting_for_replan = False
            self.publish_status("INVALID_GOAL", "Goal frame differs from map frame")
            return
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.get_parameter("base_frame").value,
                                                 rclpy.time.Time())
        except TransformException as exc:
            self.waiting_for_transform = True
            self._invalidate_path(f"Waiting for robot transform: {exc}")
            return
        self.waiting_for_transform = False
        trail_verified = self._record_breadcrumb(transform=tf)
        start = world_to_cell(map_msg.info, tf.transform.translation.x, tf.transform.translation.y)
        end = world_to_cell(map_msg.info, goal.pose.position.x, goal.pose.position.y)
        width, height = map_msg.info.width, map_msg.info.height
        inflation = self.get_parameter("obstacle_inflation_cells").value
        # Both layers contain range-classified dynamic hits. A second proximity
        # filter here could hide a real object beside a mapped wall. Confirmed
        # memory also retains full inflation wherever it was observed.
        remembered_effective = self.remembered
        observed_effective = self.observed
        # Diagnostic only: show how many live returns are already covered by
        # the inflated static map. Do not suppress a real near-wall object.
        observed_inside_static_inflation = sum(
            1 for cell in self.observed if grid.is_static_obstacle(cell))
        remembered_blocked = inflate_cells(remembered_effective, width, height, inflation)
        observed_blocked = inflate_cells(observed_effective, width, height, inflation)
        blocked = remembered_blocked | observed_blocked
        cell_penalty = clearance_penalties(
            raw_grid, observed_effective | remembered_effective,
            map_msg.info.resolution,
            float(self.get_parameter("preferred_clearance").value),
            float(self.get_parameter("clearance_penalty_gain").value))
        failure_edges, failure_costs = self._failure_constraints(grid)
        if failure_edges and self._fresh_motion_pose() is None:
            self.failure_state, self.failure_reason = "SENSOR_HOLD", "Fresh motion evidence required for failure recovery"
            self._invalidate_path(self.failure_reason)
            return
        # A scan obstacle can enter the inflation band around the robot itself
        # even when the footprint is still clear. Permit the A* search to leave
        # that one dynamic cell; static-map and confirmed-memory collisions stay
        # hard failures, while Nav2's local costmap retains its collision safety.
        escape_cells = self._dynamic_escape_halo(
            start, observed_blocked | remembered_blocked, observed_effective, remembered_effective,
            grid, raw_grid, inflation,
            self.get_parameter("robot_radius").value / map_msg.info.resolution,
            (self.get_parameter("start_egress_clearance_margin").value
             / map_msg.info.resolution + sqrt(2.0) / 2.0))
        allow_dynamic_escape = bool(escape_cells)
        egress_steps = 0
        self.global_plan_count += 1
        self.plan_metrics.update(global_plan_count=self.global_plan_count)
        try:
            if grid.is_static_obstacle(start) and self.get_parameter("start_egress_enabled").value:
                resolution = map_msg.info.resolution
                # The measured pose may lie anywhere inside its cell: preserve
                # an extra half-cell diagonal in addition to footprint margin.
                margin = self.get_parameter("start_egress_clearance_margin").value + resolution*sqrt(2.0)/2.0
                cells = self.planner.plan_with_start_egress(
                    grid, start, end, blocked, raw_grid=raw_grid,
                    raw_blocked=observed_effective | remembered_effective,
                    robot_radius_cells=self.get_parameter("robot_radius").value/resolution,
                    margin_cells=margin/resolution,
                    max_egress_cells=min(int(inflation)+1, int(self.get_parameter("start_egress_max_cells").value)),
                    allow_blocked_start=allow_dynamic_escape, escape_cells=escape_cells,
                    cell_penalty=cell_penalty, blocked_edges=failure_edges, edge_penalty=failure_costs)
                if cells:
                    egress_steps = next((index for index, cell in enumerate(cells)
                                         if not grid.is_static_obstacle(cell)), 0)
            else:
                cells = self.planner.plan(grid, start, end, blocked,
                                          allow_blocked_start=allow_dynamic_escape,
                                          escape_cells=escape_cells,
                                          cell_penalty=cell_penalty,
                                          blocked_edges=failure_edges, edge_penalty=failure_costs)
        except ValueError:
            self.plan_metrics.update(global_duration_ms=(time.perf_counter()-global_started)*1000.)
            self._invalidate_path(f"Robot or goal lies outside the map: {start} -> {end}")
            return
        self.plan_metrics.update(global_duration_ms=(time.perf_counter()-global_started)*1000.)
        self.planned_cells = cells
        if not cells:
            reference_path = self._static_reference_path(
                grid, raw_grid, start, end, map_msg.info.resolution, inflation)
            reference_hits = [cell for cell in reference_path if cell in blocked]
            self.get_logger().error(
                f"No path from {start} to {end} (static_start={grid.is_static_obstacle(start)}, "
                f"raw_static_start={raw_grid.is_static_obstacle(start)}, "
                f"remembered_start={start in remembered_blocked}, "
                f"observed_start={start in observed_blocked}, "
                f"raw_observed_start={start in self.observed}, "
                f"dynamic_escape_cells={len(escape_cells)}, "
                f"static_goal={grid.is_static_obstacle(end)}, "
                f"remembered_goal={end in remembered_blocked}, "
                f"observed_goal={end in observed_blocked}, "
                f"observed_raw={len(self.observed)}, "
                f"observed_near_map_wall=0, "
                f"observed_inside_static_inflation={observed_inside_static_inflation}, "
                f"observed_effective={len(observed_effective)}, "
                f"observed_inflated={len(observed_blocked)}, "
                f"remembered_raw={len(self.remembered)}, "
                f"remembered_inflated={len(remembered_blocked)}, "
                f"wall_match_tolerance_cells={signature[2]}, "
                f"static_reference_cells={len(reference_path)}, "
                f"static_reference_dynamic_hits={len(reference_hits)}, "
                f"first_blocked_reference_cells={reference_hits[:8]})",
                throttle_duration_sec=10.0)
            self._invalidate_path(f"No safe A* route from {start} to {end}; retrying")
            self.failure_state, self.failure_reason = "WAIT_ALTERNATIVE", "No safe alternative; goal retained"
            # Re-running A* against the same blocked scan is not a new physical
            # approach. Only an attempted published route or a correlated
            # execution failure may spend the obstacle retry budget.
            self._maybe_request_reverse(start, end, map_msg, grid, raw_grid,
                                        observed_effective | remembered_effective,
                                        blocked, cell_penalty, trail_verified)
            return
        self.waiting_for_replan = False
        self.invalid_path_published = False
        path = Path()
        path.header.frame_id = frame
        now = self.get_clock().now()
        # Cross-topic ordering has no DDS guarantee. A replacement must be
        # strictly newer than the failed request even within one /clock tick.
        cutoff = self.last_failure_event_stamp_ns
        path.header.stamp = (rclpy.time.Time(nanoseconds=max(now.nanoseconds, cutoff+1)).to_msg()
                             if cutoff else now.to_msg())
        from math import atan2, sin, cos, isfinite
        points = [(x + .5, y + .5) for x, y in cells]
        smoothed = False
        if self.get_parameter("path_smoothing_enabled").value:
            resolution = map_msg.info.resolution
            points, smoothed = smooth_grid_path(
                cells, lambda cell: grid.is_static_obstacle(cell) or cell in blocked,
                spacing=max(.01, float(self.get_parameter("path_resample_spacing").value)) / resolution,
                shortcut_distance=max(.01, float(self.get_parameter(
                    "path_smoothing_shortcut_distance").value)) / resolution,
                iterations=int(self.get_parameter("path_smoothing_iterations").value))
        # Dynamic obstacle invalidation must cover the route actually sent to
        # Nav2, including all crossed cells, rather than only the old A* cells.
        self.planned_cells = list(cells) + sorted(route_cells(points) - set(cells))
        world_points = [cell_to_world(map_msg.info, (x-.5, y-.5)) for x, y in points]
        if failure_edges:
            map_scope, goal_scope = self._memory_scopes()
            if any(self.failure_memory.segment_blocked(a, b, now=time.monotonic(),
                    map_scope=map_scope, goal_scope=goal_scope)
                   for a, b in zip(world_points, world_points[1:])):
                # Occupancy-only shortcuts could otherwise undo A*'s directed
                # constraints. The original certified grid path honours both.
                points = [(x+.5, y+.5) for x, y in cells]
                world_points = [cell_to_world(map_msg.info, cell) for cell in cells]
                smoothed = False
                self.planned_cells = list(cells)
            current = self._fresh_motion_pose()
            turn_ready = self._turn_ready(world_points, current) if current is not None else None
            if turn_ready is not True:
                self._invalidate_path("Alternative requires more turn clearance" if turn_ready is False
                                      else "Waiting for fresh 360-degree turn clearance")
                self.failure_state = "WAIT_ALTERNATIVE" if turn_ready is False else "SENSOR_HOLD"
                if turn_ready is False:
                    self._maybe_request_reverse(start, end, map_msg, grid, raw_grid,
                        observed_effective | remembered_effective, blocked, cell_penalty, trail_verified)
                return
            self.failure_state, self.failure_reason = "REJOINING", "Following a different approach; verifying progress"
            if self.rejoin_anchor is None:
                self.rejoin_anchor = {"progress": path_progress(world_points, current[:2]),
                                      "odom": self.latest_odom, "points": world_points,
                                      "pose": current}
                self.rejoin_verified = False
        self.planned_world_points = world_points
        self.local_reference_points = list(world_points)
        self.local_reference_key = self._local_route_key()
        self.local_cell_penalty = cell_penalty
        self.local_rejoin_anchor = None
        self._local_footprint_grid()
        self.plan_metrics.update(global_duration_ms=(time.perf_counter()-global_started)*1000.)
        if not failure_edges:
            self.failure_state, self.failure_reason = "CLEAR", ""
        for index, point in enumerate(world_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = point
            if index + 1 < len(world_points):
                next_x, next_y = world_points[index + 1]
                yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x)
                pose.pose.orientation.z, pose.pose.orientation.w = sin(yaw / 2.0), cos(yaw / 2.0)
            else:
                q = goal.pose.orientation
                norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
                if isfinite(norm) and norm > 1e-9:
                    pose.pose.orientation.x, pose.pose.orientation.y = q.x / norm, q.y / norm
                    pose.pose.orientation.z, pose.pose.orientation.w = q.z / norm, q.w / norm
                elif path.poses:
                    pose.pose.orientation = path.poses[-1].pose.orientation
                else:
                    pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        detail = f"A* route ready: {len(path.poses)} poses"
        detail += "; collision-checked smoothing" if smoothed else "; original grid route"
        if egress_steps:
            detail += f"; safe start egress {egress_steps} cells"
            self.get_logger().warning(
                f"Start is only inside map inflation; physically clear egress {egress_steps} cells before normal A*")
        self.publish_status("READY", detail)
        self.get_logger().info(f"Published A* path with {len(path.poses)} poses")

    def _static_reference_path(self, grid, raw_grid, start, end, resolution, inflation):
        """Diagnostic only: show where dynamic cells block an otherwise safe map route."""
        try:
            if not grid.is_static_obstacle(start):
                return self.planner.plan(grid, start, end)
            if not self.get_parameter("start_egress_enabled").value:
                return []
            margin = (self.get_parameter("start_egress_clearance_margin").value
                      + resolution * sqrt(2.0) / 2.0)
            return self.planner.plan_with_start_egress(
                grid, start, end, set(), raw_grid=raw_grid, raw_blocked=set(),
                robot_radius_cells=self.get_parameter("robot_radius").value / resolution,
                margin_cells=margin / resolution,
                max_egress_cells=min(
                    int(inflation) + 1,
                    int(self.get_parameter("start_egress_max_cells").value)))
        except ValueError:
            return []

    @staticmethod
    def _dynamic_escape_halo(start, soft_blocked, raw_observed, raw_remembered,
                             grid, raw_grid, inflation, radius_cells, margin_cells):
        """Allow egress from a soft inflation halo around observed/remembered cells.

        Raw occupied returns, confirmed memory cells, and static map obstacles remain
        hard blocks. Only the bounded inflated border may be crossed on egress
        when the complete circular robot footprint clears raw occupied cells.
        """
        required_clearance = float(radius_cells) + float(margin_cells)
        if not isfinite(required_clearance) or required_clearance < 0:
            return set()
        raw_blocked = raw_observed | raw_remembered

        def footprint_clear(cell):
            x, y = cell
            if not raw_grid.contains(cell):
                return False
            if min(x + .5, raw_grid.width - x - .5,
                   y + .5, raw_grid.height - y - .5) < required_clearance:
                return False
            reach = int(required_clearance + 1.5)
            for oy in range(max(0, y - reach), min(raw_grid.height, y + reach + 1)):
                for ox in range(max(0, x - reach), min(raw_grid.width, x + reach + 1)):
                    obstacle = (ox, oy)
                    if (raw_grid.is_static_obstacle(obstacle) or obstacle in raw_blocked):
                        distance = hypot(max(abs(x - ox) - .5, 0.0),
                                         max(abs(y - oy) - .5, 0.0))
                        if distance + 1e-9 < required_clearance:
                            return False
            return True

        if (start not in soft_blocked or start in raw_observed
                or start in raw_remembered or grid.is_static_obstacle(start)
                or not footprint_clear(start)):
            return set()
        limit = max(1, int(inflation) + 1)
        escaped = {start}
        queue = deque([(start, 0)])
        while queue:
            cell, depth = queue.popleft()
            if depth >= limit:
                continue
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)):
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor in escaped or neighbor not in soft_blocked:
                    continue
                if (neighbor in raw_observed or neighbor in raw_remembered
                        or grid.is_static_obstacle(neighbor)
                        or not footprint_clear(neighbor)):
                    continue
                escaped.add(neighbor)
                queue.append((neighbor, depth + 1))
        return escaped


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
