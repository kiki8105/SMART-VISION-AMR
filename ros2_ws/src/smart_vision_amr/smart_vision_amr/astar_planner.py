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
from smart_vision_amr.continuous_start_egress import certified_start_only_edges
from smart_vision_amr.local_detour import plan_local_detour
from smart_vision_amr.clearance_cost import clearance_penalties
from smart_vision_amr.grid import Grid
from smart_vision_amr.failure_memory import FailureMemory, EXECUTION_FAILURE_REASONS
from smart_vision_amr.obstacle_hold import ObstacleHold
from smart_vision_amr.localization_geometry import rotation_clearance
from smart_vision_amr.map_utils import (cell_to_world, inflate_cells, inflate_cells_circular, occupancy_to_grid,
                                        split_mapped_wall_returns, world_to_cell,
                                        yaw_from_quaternion)
from smart_vision_amr.reverse_recovery import (Breadcrumb, GridGeometry,
                                               select_reverse_egress)
from smart_vision_amr.path_geometry import (route_cells, smooth_grid_path, path_progress,
                                           point_at_progress, path_length)


def _normalized_pose_yaw(quaternion):
    """Match the follower's normalized yaw for motion/retreat evidence.

    Keep this local to measured chassis poses: map-origin conversion retains
    its existing semantics. Invalid quaternion norms fail existing finite
    checks instead of creating a fabricated zero-heading breadcrumb.
    """
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = x*x+y*y+z*z+w*w
    if not all(isfinite(value) for value in (x, y, z, w, norm)) or not .8 <= norm <= 1.2:
        return float("nan")
    return atan2(2*(w*z+x*y), norm-2*(y*y+z*z))


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
        # The latched memory overlay may arrive before the latched map. Keep
        # its latest raw indices until map dimensions are known.
        self.pending_memory_indices = None
        self.memory_map_key = None
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
        # /path and /planner_status are different ROS topics. The exact path
        # stamp lets consumers correlate them regardless of arrival order.
        self.last_path_stamp_ns = -1
        self.current_path_stamp_ns = None
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
        # Memory indices refer to a particular map's cells. Match the memory
        # producer's map identity (geometry + content, excluding load time),
        # so reloading identical content does not discard valid memory.
        memory_map_key = (msg.header.frame_id, info.width, info.height,
                          info.resolution, origin.position.x, origin.position.y,
                          origin.position.z, origin.orientation.x,
                          origin.orientation.y, origin.orientation.z,
                          origin.orientation.w, tuple(msg.data))
        map_key = (memory_map_key[:4]
                   + (info.map_load_time.sec, info.map_load_time.nanosec)
                   + memory_map_key[4:])
        previous_memory_map_key = getattr(self, "memory_map_key", None)
        memory_map_replaced = (previous_memory_map_key is not None
                               and memory_map_key != previous_memory_map_key)
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
        self.memory_map_key = memory_map_key
        if memory_map_replaced:
            # The memory publisher resets on a new saved map. Never interpret
            # old linear indices in the replacement map while its new overlay
            # is still in flight.
            self.remembered.clear()
            self.pending_memory_indices = None
        elif previous_memory_map_key is None:
            pending = getattr(self, "pending_memory_indices", None)
            self.pending_memory_indices = None
            if pending is not None:
                self.remembered = self._memory_cells(pending)
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
        self.hold_reference_cells = []
        self.last_invalidated_route_cells = []
        self.last_hold_probe_time = float("-inf")
        # A blocked published path is canceled immediately, but only an
        # actual no-route A* result may spend this physical-obstacle budget.
        self.last_no_route_obstacle_charge_at = float("-inf")
        self.observed_revision = 0
        self.last_no_route_observed_revision = 0
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

    def _live_obstacle_blocked(self, cells, *, observation_margin=0.):
        """Inflate only live scan returns by the robot's circular footprint.

        The map and confirmed memory retain their existing conservative square
        inflation. A half-cell diagonal covers an unknown robot-center position
        inside each A* cell; the occupied LiDAR cell itself is treated as a
        complete square. Nav2's live local costmap still checks the raw scan.
        """
        message = self.map[0]
        width, height, resolution = message.info.width, message.info.height, message.info.resolution
        robot_radius = float(self.get_parameter("robot_radius").value)
        margin = float(self.get_parameter("local_detour_clearance_margin").value)
        if (not all(isfinite(value) for value in (resolution, robot_radius, margin, observation_margin))
                or resolution <= 0 or robot_radius <= 0 or margin < 0 or observation_margin < 0):
            # An invalid runtime parameter must never silently reduce collision
            # clearance. Preserve the old square exclusion until it is fixed.
            inflation = max(0, int(self.get_parameter("obstacle_inflation_cells").value))
            return inflate_cells(cells, width, height, inflation)
        radius_cells = ((robot_radius + margin + observation_margin) / resolution
                        + sqrt(2.) / 2.)
        return inflate_cells_circular(cells, width, height, radius_cells)

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
        reserved_observed = self._local_occlusion_reserve(self.observed, reference, start, inflation, config)
        reserved_remembered = self._local_occlusion_reserve(self.remembered, reference, start, inflation, config)
        reserved_hits = reserved_observed | reserved_remembered
        # Scan endpoints can cross a raster boundary as the viewing angle
        # changes. Reserve that uncertainty before the unchanged footprint
        # inflation, so a one-cell refinement need not cancel a safe bypass.
        observation_cells = ceil(config["observation_margin"]/resolution)
        planning_hits = inflate_cells(reserved_hits, grid.width, grid.height, observation_cells)
        remembered_planning_hits = inflate_cells(
            reserved_remembered, grid.width, grid.height, observation_cells)
        blocked = (self._live_obstacle_blocked(
            reserved_observed, observation_margin=config["observation_margin"])
            | inflate_cells(remembered_planning_hits, grid.width, grid.height, inflation))
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
        self._publish_path(path)
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

    def _register_obstacle_failure(self, x, y, reference_cells=None, *,
                                   failure_id=None, candidate_cells=None):
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
        if candidate_cells is not None:
            hits.intersection_update(candidate_cells)
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
        before = self.obstacle_hold.attempts
        holding = self.obstacle_hold.register_failure(tracked, now=time.monotonic())
        self.get_logger().warning(
            f"Observed blocker retry {before}->{self.obstacle_hold.attempts}/"
            f"{self.obstacle_hold.retry_limit}: cells={sorted(tracked)} "
            f"source={'execution_failure' if failure_id is not None else 'no_safe_alternative'} "
            f"holding={holding}")
        if not holding:
            return False
        # Preserve the actually failed route before the empty Path clears it.
        # A later plan must make a meaningful departure from this approach,
        # not merely rediscover the same route after a transient hit expires.
        self.hold_reference_cells = list(self.last_invalidated_route_cells)
        self.last_hold_probe_time = time.monotonic()
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
        source_fresh = (matching and stamp > self.last_visibility_stamp_ns and stamp > 0
                        and -.25e9 <= self.get_clock().now().nanoseconds-stamp < .75e9)
        pose_fresh = source_fresh and self._fresh_motion_pose() is not None
        mux_ready = pose_fresh and self._automatic_resume_allowed()
        fresh = bool(source_fresh and pose_fresh and mux_ready)
        if not fresh:
            self.get_logger().info(
                f"Obstacle hold: free-space evidence unavailable "
                f"map_geometry={matching} source_fresh={source_fresh} "
                f"pose_fresh={pose_fresh} mux_ready={mux_ready}",
                throttle_duration_sec=1.0)
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
        if not tracked <= free or occupied:
            self.get_logger().info(
                f"Obstacle hold: tracked={sorted(tracked)} free={sorted(free)} "
                f"near_hits={sorted(near_hits)[:8]}", throttle_duration_sec=1.0)
        if not self.obstacle_hold.observe_free(free, occupied, fresh=True, now=now):
            return
        self.get_logger().info("Obstacle hold: measured free-space interval completed; retrying retained goal")
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

    def _safe_failure_edge_fallback(self, map_msg, grid, start, end, blocked,
                                    escape_cells, allow_dynamic_escape,
                                    cell_penalty, failure_edges, failure_costs,
                                    safety_egress_edges=None):
        """Retry a nonphysical failed edge as a costly edge, never an occupied cell.

        A progress abort can happen during a valid in-place turn. In a narrow
        corridor, rasterizing that event as hard directed edges may disconnect
        the only collision-free route until its TTL expires. Only a bounded
        number of independently confirmed execution failures may use this
        fallback, and a fresh raw scan must find no hit near the failed pose.
        The same static, remembered and observed obstacle cells remain blocked.
        """
        if not failure_edges or grid.is_static_obstacle(start):
            return []
        if self._fresh_motion_pose() is None or not self._automatic_resume_allowed():
            return []
        map_scope, goal_scope = self._memory_scopes()
        entries = self.failure_memory.snapshot(
            now=time.monotonic(), map_scope=map_scope, goal_scope=goal_scope)["entries"]
        if (not entries or any(entry["reason"] not in
                               ("execution_no_progress", "execution_oscillation")
                               for entry in entries)
                or sum(entry["occurrences"] for entry in entries)
                   >= max(1, int(self.get_parameter("obstacle_retry_limit").value))
                or time.monotonic()-max(entry["last_seen"] for entry in entries) < 1.0):
            return []
        radius = float(self.get_parameter("failure_memory_radius").value)
        body = float(self.get_parameter("robot_radius").value)
        # A filtered cell layer can miss a real object. Require the raw scan
        # endpoint check as well as the observed/remembered map-cell check.
        for cell in self.observed | self.remembered:
            if not grid.contains(cell):
                continue
            x, y = cell_to_world(map_msg.info, cell)
            if any(hypot(x-entry["x"], y-entry["y"]) <= radius+body+.05
                   for entry in entries):
                return []
        scan = self.latest_scan
        if scan is None or not scan.header.frame_id:
            return []
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value, scan.header.frame_id,
                rclpy.time.Time.from_msg(scan.header.stamp))
        except (TransformException, ValueError):
            return []
        origin = transform.transform.translation
        orientation = transform.transform.rotation
        norm = sum(value*value for value in (
            orientation.x, orientation.y, orientation.z, orientation.w))
        heading = yaw_from_quaternion(orientation)
        if (not all(isfinite(value) for value in (origin.x, origin.y, heading, norm))
                or not .8 <= norm <= 1.2):
            return []
        for index, reading in enumerate(scan.ranges):
            if not isfinite(reading) or not scan.range_min <= reading <= scan.range_max:
                continue
            angle = heading+scan.angle_min+index*scan.angle_increment
            x, y = origin.x+reading*cos(angle), origin.y+reading*sin(angle)
            if any(hypot(x-entry["x"], y-entry["y"]) <= radius+body+.05
                   for entry in entries):
                return []
        # Hard-edge A* has already failed. A soft edge cost discourages the
        # same approach, but cannot invent a path through live obstacles.
        penalty = max(10.0, 5.0*float(self.get_parameter("failure_memory_penalty").value))
        costs = dict(failure_costs)
        for edge in failure_edges:
            costs[edge] = max(costs.get(edge, 0.0), penalty)
        try:
            return self.planner.plan(
                grid, start, end, blocked,
                allow_blocked_start=allow_dynamic_escape,
                escape_cells=escape_cells,
                cell_penalty=cell_penalty, edge_penalty=costs,
                blocked_edges=safety_egress_edges)
        except ValueError:
            return []

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
            self.last_fresh_motion_pose_stamp_ns = stamp
            return float(p.x), float(p.y), _normalized_pose_yaw(q)
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

    def _hold_probe_ready(self):
        """An occupied-route probe needs the same independent motion evidence.

        A disappearing transient overlay is not free-space proof. The tracked
        physical object must still be represented in the current obstacle
        layers; otherwise only on_visibility() may release this hold.
        """
        now = time.monotonic()
        return (self.obstacle_hold.holding and self._goal_is_active()
                and self.map is not None and not self.recovery_active
                and self.pending_failure_id is None
                and self.obstacle_hold.goal_scope == repr(self._memory_scopes())
                and self.last_observation_time is not None
                and 0 <= now-self.last_observation_time < .75
                and self._observations_ready()
                and bool(self.obstacle_hold.blocked_cells & (self.observed | self.remembered))
                and self._fresh_motion_pose() is not None
                and self._automatic_resume_allowed())

    def _hold_route_is_distinct(self, cells, reference, blocked, grid, escape_cells,
                                *, allowed_initial_escape=None):
        """Reject a rediscovered approach and any path through occupied cells."""
        cells = tuple(cells)
        if len(cells) < 2 or not reference:
            return False
        # The blocked start may be a certified *dynamic inflation halo* cell.
        # It is allowed only as an initial prefix of the existing safe egress;
        # no static, raw, or remembered occupied cell is ever excepted.
        if allowed_initial_escape is None:
            allowed_initial_escape = set()
            for cell in cells:
                if cell not in blocked:
                    break
                if cell not in escape_cells:
                    return False
                allowed_initial_escape.add(cell)
        for cell in cells:
            if not grid.contains(cell) or grid.is_static_obstacle(cell):
                return False
            if cell in blocked and cell not in allowed_initial_escape:
                return False
        # A one-cell raster shift or a new initial pose is not a new approach.
        # Require at least one interior cell outside the old route's 8-neighbor
        # band. The planner's occupied mask still decides actual clearance.
        near_old = inflate_cells(set(reference), grid.width, grid.height, 1)
        return any(cell not in near_old for cell in cells)

    def retry_transform(self):
        """Retry no-path, missing-map and missing-TF conditions without losing the goal."""
        if self.obstacle_hold.holding:
            now = time.monotonic()
            if (self.last_visibility_at is None or now-self.last_visibility_at > .5
                    or self._fresh_motion_pose() is None or not self._automatic_resume_allowed()):
                self.obstacle_hold.observe_free(set(), set(), fresh=False, now=now)
            if (now-self.last_hold_probe_time >= self._retry_period()
                    and self._hold_probe_ready()):
                self.last_hold_probe_time = now
                self.plan_to(self.last_goal, hold_probe=True)
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
        if self.planned_cells:
            self.last_invalidated_route_cells = list(self.planned_cells)
        self.planned_cells = []
        self.planned_world_points = []
        if self.plan_metrics.get("local_detour_state") == "ACTIVE":
            self.plan_metrics.update(local_detour_state="FAILED", detour_reason=reason)
            self.local_rejoin_anchor = None
        if not self.invalid_path_published:
            path = Path()
            path.header.frame_id = self.get_parameter("map_frame").value
            path.header.stamp = self.get_clock().now().to_msg()
            self._publish_path(path)
            self.invalid_path_published = True
        self.waiting_for_replan = True
        self.publish_status("WAIT_REPLAN", reason)

    def _publish_path(self, path):
        """Give every published path, including an empty invalidation, a unique stamp."""
        stamp = path.header.stamp
        requested_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        next_ns = max(1, requested_ns, getattr(self, 'last_path_stamp_ns', -1) + 1,
                      getattr(self, 'last_failure_event_stamp_ns', 0) + 1)
        path.header.stamp = rclpy.time.Time(nanoseconds=next_ns).to_msg()
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
        self.last_path_stamp_ns = next_ns
        self.current_path_stamp_ns = next_ns
        self.path_pub.publish(path)

    def publish_status(self, status, message):
        goal = self.last_goal
        payload = {"status": status, "message": message, "goal": None, "goal_stamp": None,
                   **self.plan_metrics}
        clock = self.get_clock().now()
        payload["sent_at_ns"] = getattr(clock, "nanoseconds", 0)
        path_stamp_ns = getattr(self, 'current_path_stamp_ns', None)
        if path_stamp_ns is not None:
            sec, nanosec = divmod(path_stamp_ns, 1_000_000_000)
            payload["path_stamp"] = {"sec": sec, "nanosec": nanosec}
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
            # Preserve even an empty message: it can supersede an earlier
            # nonempty latched overlay before the first map is delivered.
            self.pending_memory_indices = tuple(msg.data)
            return
        updated = self._memory_cells(msg.data)
        if updated != self.remembered:
            added = updated - self.remembered
            removed = self.remembered - updated
            self.remembered = updated
            # Empty-route recovery must also react to obstacle removal. While
            # driving, unrelated memory churn must not cancel a valid Nav2 goal.
            if self._goal_is_active() and (self.waiting_for_replan
                    or self._changes_block_path(added, layer="memory")
                    or self._changes_block_path(removed, layer="memory")):
                if self._changes_block_path(added, layer="memory"):
                    if self._try_local_detour(added):
                        return
                    self._invalidate_path("Remembered obstacle intersects the route")
                self._request_replan("Remembered obstacles changed")

    def _memory_cells(self, indices):
        width, height = self.map[0].info.width, self.map[0].info.height
        return {(index % width, index // width) for index in indices
                if 0 <= index < width * height}

    def on_observed(self, msg):
        if self.map is None:
            return
        self.last_observation_time = time.monotonic()
        # A fresh repeat observation is independent evidence that a fixed
        # blocker is still present, even when its occupied cell set is stable.
        self.observed_revision += 1
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
                    self.get_logger().warning(
                        f"Live observed cells intersect remaining route: added={sorted(added)[:16]} "
                        f"observed_count={len(updated)} path_cells={len(self.planned_cells)}")
                    if self._try_local_detour(added):
                        return
                    self._invalidate_path("Observed obstacle intersects the route")
                self._request_replan("Observed obstacles changed")
        elif self.waiting_for_replan and self._goal_is_active():
            # Fresh empty/unchanged scans also recover a stale-sensor hold.
            self._request_replan("LiDAR obstacle observations restored")

    def _changes_block_path(self, cells, *, layer="observed", filter_near_wall=False):
        if not cells or not self.planned_cells:
            return False
        width, height = self.map[0].info.width, self.map[0].info.height
        inflation = self.get_parameter("obstacle_inflation_cells").value
        effective = (self._split_mapped_wall_returns(cells)[0]
                     if filter_near_wall else cells)
        blocked = (inflate_cells(effective, width, height, inflation) if layer == "memory"
                   else self._live_obstacle_blocked(effective))
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
        self.current_path_stamp_ns = None
        self._reset_local_reference()
        self.navigation_suspended = False
        self.goal_completed = False
        old_hold_scope = self.obstacle_hold.goal_scope
        self.obstacle_hold.ensure_goal_scope(repr(self._memory_scopes()))
        if self.obstacle_hold.goal_scope != old_hold_scope:
            self.hold_reference_cells = []
            self.last_invalidated_route_cells = []
            self.last_hold_probe_time = float("-inf")
        if self.obstacle_hold.attempts == 0:
            self.last_no_route_obstacle_charge_at = float("-inf")
            self.last_no_route_observed_revision = self.observed_revision
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
        child_frame = getattr(msg, "child_frame_id", "")
        previous_child = getattr(self, "latest_odom_child_frame", None)
        if previous_child is not None and child_frame != previous_child:
            self._clear_breadcrumbs()
            self.odom_samples.clear()
        self.latest_odom_child_frame = child_frame
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        pose = msg.pose.pose
        sample = (time.monotonic(), stamp_ns, float(pose.position.x),
                  float(pose.position.y), _normalized_pose_yaw(pose.orientation),
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
        yaw = _normalized_pose_yaw(rotation)
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

    def _reverse_start_stopped(self):
        """Require independently measured stillness before certifying a retreat.

        The invalid/empty Path is already published before this check. Wait
        for at least .20 s of dense odometry; a command of zero or a single
        pose is insufficient evidence that the old FollowPath has stopped.
        Allow bounded 10 Hz timing jitter without weakening the pose bound.
        """
        samples = tuple(self.odom_samples)
        if len(samples) < 4:
            return False
        latest = samples[-1]
        now = time.monotonic()
        window = [sample for sample in samples if 0 <= latest[0]-sample[0] <= .50]
        if (len(window) < 4 or not 0 <= now-latest[0] < .20
                or any(sample[5] != latest[5] for sample in window)
                or latest[0]-window[0][0] < .20-1e-9):
            return False
        for previous, sample in zip(window, window[1:]):
            if (not 0 < sample[0]-previous[0] <= .18+1e-9
                    or not 0 < sample[1]-previous[1] <= 180_000_000
                    or sample[5] != latest[5]):
                return False
        return all(hypot(sample[2]-latest[2], sample[3]-latest[3]) <= .002
                   and abs(self._angle_delta(sample[4], latest[4])) <= .005
                   for sample in window)

    def _reverse_unavailable(self, code, start):
        """Expose the first failed reverse gate without flooding retry logs."""
        previous_code = getattr(self, "last_reverse_rejection_code", None)
        self.last_reverse_rejection_code = code
        now = time.monotonic()
        previous = getattr(self, "last_reverse_rejection_log_at", float("-inf"))
        if code != previous_code or now < previous or now - previous >= 5.0:
            self.get_logger().warning(f"REVERSE_REJECT[{code}] start={start}")
            self.last_reverse_rejection_log_at = now
        return False

    def _maybe_request_reverse(self, start, end, map_msg, grid, raw_grid,
                               raw_dynamic, inflated_dynamic, cell_penalty,
                               trail_verified, blocking_evidence=None):
        """Request a bounded Nav2 backup only when a verified A* exit exists."""
        if not trail_verified:
            return self._reverse_unavailable("TRAIL_UNVERIFIED", start)
        if not self.get_parameter("reverse_recovery_enabled").value:
            return self._reverse_unavailable("DISABLED", start)
        if getattr(self, "recovery_active", False):
            return self._reverse_unavailable("ALREADY_ACTIVE", start)
        if start in self.recovery_attempted_cells:
            return self._reverse_unavailable("START_ALREADY_ATTEMPTED", start)
        if self.recovery_attempts >= int(self.get_parameter("reverse_recovery_max_attempts").value):
            return self._reverse_unavailable("ATTEMPT_LIMIT", start)
        if not self._goal_is_active():
            return self._reverse_unavailable("NO_ACTIVE_GOAL", start)
        if not self._observations_ready():
            return self._reverse_unavailable("SENSORS_UNREADY", start)
        if not self._automatic_resume_allowed():
            return self._reverse_unavailable("AUTONOMY_UNAVAILABLE", start)
        if not self._reverse_start_stopped():
            return self._reverse_unavailable("NOT_STOPPED", start)
        if not getattr(self, "latest_odom_child_frame", ""):
            return self._reverse_unavailable("ODOM_CHILD_FRAME_MISSING", start)
        if grid.is_static_obstacle(end):
            return self._reverse_unavailable("GOAL_STATIC_BLOCKED", start)
        if end in inflated_dynamic:
            return self._reverse_unavailable("GOAL_DYNAMIC_BLOCKED", start)
        current = self._fresh_motion_pose()
        physical_radius = float(self.get_parameter("robot_radius").value)
        if current is None:
            return self._reverse_unavailable("POSE_UNREADY", start)
        if not self.breadcrumbs:
            return self._reverse_unavailable("TRAIL_MISSING", start)
        if not isfinite(physical_radius) or physical_radius <= 0:
            return self._reverse_unavailable("RADIUS_INVALID", start)
        physical_radius = max(.24, physical_radius)
        # Keep measurement time and odom baseline before any map/A* work. Do
        # not refresh certificate age after a costly candidate search.
        certified_odom = tuple(self.latest_odom)
        certified_child = self.latest_odom_child_frame
        certified_map_stamp_ns = getattr(self, "last_fresh_motion_pose_stamp_ns", None)
        if type(certified_map_stamp_ns) is not int or certified_map_stamp_ns <= 0:
            return self._reverse_unavailable("MAP_POSE_STAMP_MISSING", start)
        certificate_at_ns = min(self.get_clock().now().nanoseconds,
                                certified_odom[1], certified_map_stamp_ns)
        latest = self.breadcrumbs[-1]
        if (hypot(current[0]-latest.x, current[1]-latest.y) > .05
                or abs(self._angle_delta(current[2], latest.yaw)) > .15):
            return self._reverse_unavailable("TRAIL_POSE_MISMATCH", start)

        # A newly seen object can leave a physically clear corridor parallel
        # to a wall. Requiring clearance to that *wall* to improve as well as
        # clearance to the blocker unnecessarily rejects a useful retreat.
        # Nominate only fresh, unmapped physical hits ahead of this chassis;
        # the selector still checks every raw wall/hit over the entire sweep.
        blocking_cells = set()
        if (self.last_observation_time is not None
                and 0 <= time.monotonic()-self.last_observation_time < .75):
            body = physical_radius
            for cell in raw_dynamic & self.observed & set(blocking_evidence or ()):
                if raw_grid.is_static_obstacle(cell):
                    continue
                x, y = cell_to_world(map_msg.info, cell)
                dx, dy = x-current[0], y-current[1]
                forward = dx*cos(current[2])+dy*sin(current[2])
                lateral = abs(-dx*sin(current[2])+dy*cos(current[2]))
                if 0 < forward <= .80 and lateral <= body+.10:
                    blocking_cells.add(cell)

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
            robot_radius=physical_radius,
            # Dispatch may differ by <=3 mm / .01 rad from this stationary
            # certificate. The separate .01 m envelope covers that handover
            # (3 mm + .30 m*sin(.01) < 7 mm), beyond the physical margin.
            clearance_margin=float(self.get_parameter("reverse_recovery_clearance_margin").value)+.01,
            min_distance=float(self.get_parameter("reverse_recovery_min_distance").value),
            max_distance=float(self.get_parameter("reverse_recovery_max_distance").value),
            max_history_age=90.0, max_sample_gap=30.0,
            blocking_cells=blocking_cells, current_pose=current,
            # A physically clear start can still be trapped by the static
            # inflation band. Parallel retreat need not increase distance to
            # an unrelated side wall, but must exit inflation and preserve
            # the selector's complete physical sweep and A* goal checks.
            allow_parallel_static_egress=(
                grid.is_static_obstacle(start)
                and not raw_with_live.is_static_obstacle(start)
                and not blocking_cells))
        if candidate is None:
            return self._reverse_unavailable("NO_SAFE_EGRESS", start)
        if not 0 <= self.get_clock().now().nanoseconds-certificate_at_ns < 750_000_000:
            return self._reverse_unavailable("CERTIFICATE_EXPIRED", start)

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
            "start_cell": list(start), "exit_cell": list(candidate.target_cell),
            "blocking_cells": [list(cell) for cell in sorted(blocking_cells)],
            "certified_start": {
                "frame_id": certified_odom[5],
                "child_frame_id": certified_child,
                "stamp_ns": certified_odom[1],
                "x": certified_odom[2], "y": certified_odom[3],
                "yaw": certified_odom[4],
                "certified_at_ns": certificate_at_ns,
                "max_position_drift_m": .003, "max_yaw_drift_rad": .01,
                "sweep_reserve_m": .01,
                "map_frame": self.get_parameter("map_frame").value,
                "map_base_frame": self.get_parameter("base_frame").value,
                "map_x": current[0], "map_y": current[1], "map_yaw": current[2],
                "map_stamp_ns": certified_map_stamp_ns}})
        self.recovery_pub.publish(request)
        self.publish_status("RECOVERING", f"Backing up {candidate.distance_m:.2f} m to replan retained goal")
        self.get_logger().warning(
            f"No forward route; requested collision-checked {candidate.distance_m:.2f} m reverse "
            f"to {candidate.target_cell} (attempt {self.recovery_attempts})")
        return True

    def plan_to(self, goal, *, hold_probe=False):
        if (getattr(self, "recovery_active", False) or self.pending_failure_id is not None
                or (self.obstacle_hold.holding and not hold_probe)
                or (hold_probe and not self._hold_probe_ready())):
            return
        if self.failure_state == "SENSOR_HOLD" and self._fresh_motion_pose() is None:
            self._invalidate_path("Waiting for fresh independent sensor evidence before retry")
            return
        self.last_plan_time = time.monotonic()
        global_started = time.perf_counter()
        self.plan_metrics.update(plan_kind="GLOBAL", failure_edge_fallback=False,
            local_detour_state="FAILED" if self.plan_metrics.get("local_detour_state") == "FAILED" else "NONE")
        self.geometry_signature = tuple(self.get_parameter(name).value for name in (
            "path_smoothing_enabled", "path_smoothing_iterations",
            "path_smoothing_shortcut_distance", "path_resample_spacing"))
        self.waiting_for_replan = True
        if not hold_probe:
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
        observed_blocked = self._live_obstacle_blocked(observed_effective)
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
        safety_egress_edges = set()
        certified_start_edges = set()
        if (self.get_parameter("start_egress_enabled").value
                and not escape_cells
                and (grid.is_static_obstacle(start) or start in observed_blocked)
                and not raw_grid.is_static_obstacle(start)
                and start not in observed_effective and start not in remembered_blocked
                and self.last_observation_time is not None
                and 0 <= time.monotonic() - self.last_observation_time < .75):
            # Correct a static/live inflation start-cell quantization artifact.
            # The complete stock collision model reaches .237693 m from base;
            # round up to .24 m here, then retain the existing clearance margin.
            # Global inflation, raw occupied/unknown cells and later edges stay
            # unchanged. This does not certify unmodeled hardware extensions.
            current = self._fresh_motion_pose()
            if (current is not None
                    and world_to_cell(map_msg.info, current[0], current[1]) == start
                    and hypot(current[0] - tf.transform.translation.x,
                              current[1] - tf.transform.translation.y) <= .05):
                origin = map_msg.info.origin.position
                angle = yaw_from_quaternion(map_msg.info.origin.orientation)
                dx, dy = current[0] - origin.x, current[1] - origin.y
                pose_cells = ((cos(angle) * dx + sin(angle) * dy) / map_msg.info.resolution,
                              (-sin(angle) * dx + cos(angle) * dy) / map_msg.info.resolution)
                physical_radius = float(self.get_parameter("robot_radius").value)
                physical_radius = (max(.24, physical_radius)
                                   if isfinite(physical_radius) and physical_radius > 0 else physical_radius)
                safe_edges = certified_start_only_edges(
                    raw_grid, grid, observed_effective | remembered_effective,
                    blocked, start, pose_cells, physical_radius / map_msg.info.resolution,
                    float(self.get_parameter("start_egress_clearance_margin").value)
                    / map_msg.info.resolution)
                if safe_edges:
                    certified_start_edges = safe_edges
                    escape_cells = {start}
                    safety_egress_edges = {
                        (start, (start[0] + dx, start[1] + dy))
                        for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                        if (dx or dy) and (start, (start[0] + dx, start[1] + dy)) not in safe_edges
                    }
                    # Clearing the start must not change later diagonal
                    # corner checks. Block all reentry and both directions of
                    # every neighbor-to-neighbor diagonal that would use this
                    # cleared cell as an orthogonal side cell.
                    safety_egress_edges.update(
                        ((start[0]+dx, start[1]+dy), start)
                        for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy)
                    for dx in (-1, 1):
                        for dy in (-1, 1):
                            horizontal = (start[0]+dx, start[1])
                            vertical = (start[0], start[1]+dy)
                            safety_egress_edges.update(((horizontal, vertical), (vertical, horizontal)))
                    self.get_logger().info(
                        f"Fresh continuous pose certified {len(safe_edges)} start-only egress edges "
                        f"with physical radius {physical_radius:.3f} m and unchanged clearance margin")
        # A statically inflated start cannot be passed to ordinary A*. Build
        # a private planning view clearing that one *certified* cell only. The
        # stored map, goal, neighboring inflation and every later edge remain
        # unchanged; the directed first-edge constraints are never softened.
        planning_grid = grid
        if certified_start_edges and grid.is_static_obstacle(start):
            planning_grid = Grid(tuple(tuple(
                False if (x, y) == start else grid.rows[y][x]
                for x in range(grid.width)) for y in range(grid.height)))
        allow_dynamic_escape = bool(escape_cells)
        plan_blocked_edges = failure_edges | safety_egress_edges
        egress_steps = 0
        self.global_plan_count += 1
        self.plan_metrics.update(global_plan_count=self.global_plan_count)
        try:
            if (grid.is_static_obstacle(start) and not certified_start_edges
                    and self.get_parameter("start_egress_enabled").value):
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
                    cell_penalty=cell_penalty, blocked_edges=plan_blocked_edges,
                    edge_penalty=failure_costs)
                if cells:
                    egress_steps = next((index for index, cell in enumerate(cells)
                                         if not grid.is_static_obstacle(cell)), 0)
            else:
                cells = self.planner.plan(planning_grid, start, end, blocked,
                                          allow_blocked_start=allow_dynamic_escape,
                                          escape_cells=escape_cells,
                                          cell_penalty=cell_penalty,
                                          blocked_edges=plan_blocked_edges,
                                          edge_penalty=failure_costs)
        except ValueError:
            self.plan_metrics.update(global_duration_ms=(time.perf_counter()-global_started)*1000.)
            self._invalidate_path(f"Robot or goal lies outside the map: {start} -> {end}")
            return
        self.plan_metrics.update(global_duration_ms=(time.perf_counter()-global_started)*1000.)
        failure_edge_fallback = False
        if not cells and failure_edges and not hold_probe:
            cells = self._safe_failure_edge_fallback(
                map_msg, planning_grid, start, end, blocked, escape_cells,
                allow_dynamic_escape, cell_penalty, failure_edges, failure_costs,
                safety_egress_edges)
            failure_edge_fallback = bool(cells)
            if failure_edge_fallback:
                self.plan_metrics.update(failure_edge_fallback=True)
                self.get_logger().warning(
                    "Hard failed-edge route unavailable; trying bounded, scan-clear "
                    "penalty-only route with all obstacle cells still blocked")
        if hold_probe and (not cells or certified_start_edges):
            self.get_logger().info("Obstacle hold: no certified alternative around current blocker",
                                   throttle_duration_sec=5.0)
            return
        if certified_start_edges and cells and (len(cells) < 2
                or (cells[0], cells[1]) not in certified_start_edges):
            self._invalidate_path("Continuous start egress lost its certified first edge")
            return
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
                f"failed_edges={len(failure_edges)}, "
                f"failed_edge_penalties={len(failure_costs)}, "
                f"clearance_penalty_cells={len(cell_penalty)}, "
                f"wall_match_tolerance_cells={signature[2]}, "
                f"static_reference_cells={len(reference_path)}, "
                f"static_reference_dynamic_hits={len(reference_hits)}, "
                f"first_blocked_reference_cells={reference_hits[:8]})",
                throttle_duration_sec=10.0)
            self._invalidate_path(f"No safe A* route from {start} to {end}; retrying")
            self.failure_state, self.failure_reason = "WAIT_ALTERNATIVE", "No safe alternative; goal retained"
            # A focused obstacle retreat needs causal evidence: a physical
            # fresh hit intersects the static route's body corridor and the
            # same retained goal is reachable with only live hits removed.
            # A static/failure-memory dead end alone cannot authorize it.
            body_corridor_m = float(self.get_parameter("robot_radius").value)+.05
            reverse_blockers = {cell for cell in observed_effective
                if any(hypot(cell[0]-waypoint[0], cell[1]-waypoint[1])
                       * map_msg.info.resolution <= body_corridor_m
                       for waypoint in reference_path)}
            if reverse_blockers:
                try:
                    without_live = self.planner.plan(
                        planning_grid, start, end, remembered_blocked,
                        allow_blocked_start=allow_dynamic_escape,
                        escape_cells=escape_cells, cell_penalty=cell_penalty,
                        blocked_edges=plan_blocked_edges, edge_penalty=failure_costs)
                except ValueError:
                    without_live = []
                if not without_live:
                    reverse_blockers.clear()
            # A verified reverse remains available before latching a persistent
            # obstacle hold; a robot can clear the blocked start without waiting
            # for a stationary object to disappear.
            if self._maybe_request_reverse(start, end, map_msg, grid, raw_grid,
                                        observed_effective | remembered_effective,
                                        blocked, cell_penalty, trail_verified,
                                        blocking_evidence=reverse_blockers):
                return
            # A changed scan invalidating a path is not a failed *alternative*.
            # Charge only a completed no-route search with a newly observed,
            # physical hit close enough to touch the robot's swept body along
            # the static reference. Inflated planner cells alone may merely
            # reflect a deliberately conservative clearance. The observation
            # message sequence also prevents retry_transform() from spending
            # the whole budget on one cached scan and unchanged A* result.
            now = time.monotonic()
            body_corridor_m = (float(self.get_parameter("robot_radius").value)
                               + .05)
            physical_hits = [cell for cell in observed_effective
                             if any(hypot(cell[0]-waypoint[0],
                                          cell[1]-waypoint[1])
                                    * map_msg.info.resolution <= body_corridor_m
                                    for waypoint in reference_path)]
            if (physical_hits and self.observed_revision > self.last_no_route_observed_revision
                    and self.last_observation_time is not None
                    and 0 <= now-self.last_observation_time < .75
                    and self._automatic_resume_allowed()
                    and now-self.last_no_route_obstacle_charge_at >=
                    max(2.0, self._retry_period())):
                pose = self._fresh_motion_pose()
                route_without_live_hit = []
                if pose is not None:
                    # The live return must actually be pivotal. A no-route
                    # caused by a static wall, retained failed edge, or old
                    # memory is not evidence that moving this object helps.
                    try:
                        route_without_live_hit = self.planner.plan(
                            grid, start, end, remembered_blocked,
                            cell_penalty=cell_penalty,
                            blocked_edges=failure_edges, edge_penalty=failure_costs)
                    except ValueError:
                        route_without_live_hit = []
                if pose is not None and route_without_live_hit:
                    before = self.obstacle_hold.attempts
                    self._register_obstacle_failure(
                        pose[0], pose[1], physical_hits,
                        candidate_cells=physical_hits)
                    if self.obstacle_hold.attempts > before:
                        self.last_no_route_obstacle_charge_at = now
                        self.last_no_route_observed_revision = self.observed_revision
            return
        if hold_probe:
            reference = self.hold_reference_cells or self._static_reference_path(
                grid, raw_grid, start, end, map_msg.info.resolution, inflation)
            if not self._hold_route_is_distinct(cells, reference, blocked, grid, escape_cells):
                self.get_logger().info("Obstacle hold: route is unchanged or not collision-free",
                                       throttle_duration_sec=5.0)
                return
        path = Path()
        path.header.frame_id = frame
        now = self.get_clock().now()
        # Cross-topic ordering has no DDS guarantee. A replacement must be
        # strictly newer than the failed request even within one /clock tick.
        cutoff = self.last_failure_event_stamp_ns
        path.header.stamp = (rclpy.time.Time(nanoseconds=max(now.nanoseconds, cutoff+1)).to_msg()
                             if cutoff else now.to_msg())
        points = [(x + .5, y + .5) for x, y in cells]
        smoothed = False
        if self.get_parameter("path_smoothing_enabled").value and not certified_start_edges:
            resolution = map_msg.info.resolution
            points, smoothed = smooth_grid_path(
                cells, lambda cell: grid.is_static_obstacle(cell) or cell in blocked,
                spacing=max(.01, float(self.get_parameter("path_resample_spacing").value)) / resolution,
                shortcut_distance=max(.01, float(self.get_parameter(
                    "path_smoothing_shortcut_distance").value)) / resolution,
                iterations=int(self.get_parameter("path_smoothing_iterations").value))
        # Dynamic obstacle invalidation must cover the route actually sent to
        # Nav2, including all crossed cells, rather than only the old A* cells.
        planned_cells = list(cells) + sorted(route_cells(points) - set(cells))
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
                planned_cells = list(cells)
            current = self._fresh_motion_pose()
            turn_ready = self._turn_ready(world_points, current) if current is not None else None
            if turn_ready is not True:
                if hold_probe:
                    self.get_logger().info("Obstacle hold: alternate approach turn is not verified",
                                           throttle_duration_sec=5.0)
                    return
                self._invalidate_path("Alternative requires more turn clearance" if turn_ready is False
                                      else "Waiting for fresh 360-degree turn clearance")
                self.failure_state = "WAIT_ALTERNATIVE" if turn_ready is False else "SENSOR_HOLD"
                if turn_ready is False:
                    self._maybe_request_reverse(start, end, map_msg, grid, raw_grid,
                        observed_effective | remembered_effective, blocked, cell_penalty, trail_verified)
                return
            self.failure_state, self.failure_reason = (
                "REJOINING",
                ("Retrying a sensor-clear failed approach; verifying real progress"
                 if failure_edge_fallback else
                 "Following a different approach; verifying progress"))
            if self.rejoin_anchor is None:
                self.rejoin_anchor = {"progress": path_progress(world_points, current[:2]),
                                      "odom": self.latest_odom, "points": world_points,
                                      "pose": current}
                self.rejoin_verified = False
        if hold_probe:
            # Check the path actually sent after smoothing as well as the raw
            # A* route, and recheck the source before releasing the latch.
            actual_cells = route_cells(points)
            allowed_initial_escape = set()
            for cell in cells:
                if cell not in blocked:
                    break
                allowed_initial_escape.add(cell)
            refreshed_pose = self._fresh_motion_pose()
            if (not self._hold_route_is_distinct(actual_cells, reference, blocked,
                                                 grid, escape_cells,
                                                 allowed_initial_escape=allowed_initial_escape)
                    or not self._hold_probe_ready() or refreshed_pose is None
                    or self._turn_ready(world_points, refreshed_pose) is not True):
                self.failure_state = "OBSTACLE_HOLD"
                self.rejoin_anchor = None
                return
            self.obstacle_hold.release_for_alternative(now=time.monotonic())
            self.hold_reference_cells = []
            self.get_logger().info("Obstacle hold: distinct scan-clear route found; resuming retained goal")
        self.planned_cells = planned_cells
        self.waiting_for_replan = False
        self.invalid_path_published = False
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
        self._publish_path(path)
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
