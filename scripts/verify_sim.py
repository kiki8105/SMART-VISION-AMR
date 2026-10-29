#!/usr/bin/env python3
"""Exercise the running, isolated Gazebo A* -> FollowPath navigation stack.

Source the same ROS/workspace setup and simulation environment as run_sim.sh
before invoking this script. It attaches to existing nodes and sends one goal;
it never starts Gazebo or navigation processes.
"""

import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path as FilePath
import sys
import tempfile
import time
from urllib.parse import urlparse


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=-1.0)
    parser.add_argument("--goal-y", type=float, default=-2.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0,
                        help="Goal heading in radians (default: 0).")
    parser.add_argument("--ready-timeout", type=float, default=120.0,
                        help="Wall-clock seconds allowed for stack readiness.")
    parser.add_argument("--timeout", type=float, default=150.0,
                        help="Wall-clock seconds allowed after sending the goal.")
    parser.add_argument("--via-controller", action="store_true",
                        help="Send the goal through the actual PyQt controller map panel; "
                             "also verify its live map, robot pose and displayed A* path.")
    parser.add_argument("--controller", type=FilePath,
                        default=FilePath(__file__).resolve().parents[1] / "tools" / "cmd_vel_controller.py",
                        help="Controller source used by --via-controller.")
    parser.add_argument("--controller-screenshot", type=FilePath,
                        help="Optional new PNG path for the verified controller window.")
    args = parser.parse_args()
    for name in ("goal_x", "goal_y", "goal_yaw", "ready_timeout", "timeout"):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if args.ready_timeout <= 0 or args.timeout <= 0:
        parser.error("timeouts must be positive")
    if args.controller_screenshot and not args.via_controller:
        parser.error("--controller-screenshot requires --via-controller")
    return args


def check_isolation():
    """Refuse a goal unless the caller selected the dedicated simulator domain."""
    expected = {"ROS_LOCALHOST_ONLY": "1", "ROS_DOMAIN_ID": "76"}
    wrong = [f"{name}={os.environ.get(name)!r}; expected {value!r}"
             for name, value in expected.items() if os.environ.get(name) != value]
    uri = os.environ.get("GAZEBO_MASTER_URI", "")
    try:
        parsed = urlparse(uri)
        gazebo_is_local = (parsed.scheme == "http"
                           and parsed.hostname in ("127.0.0.1", "localhost", "::1")
                           and parsed.port == 11376)
    except ValueError:
        gazebo_is_local = False
    if not gazebo_is_local:
        wrong.append(f"GAZEBO_MASTER_URI={uri!r}; expected http://127.0.0.1:11376")
    if wrong:
        raise RuntimeError("Simulation isolation check failed: " + "; ".join(wrong)
                           + ". Source the simulation environment used by run_sim.sh.")


def run_verification(args):
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, Twist
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import FollowPath
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                              qos_profile_sensor_data)
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python packages are unavailable. Source "
                           "/opt/ros/humble/setup.bash and the Team2 workspace "
                           f"install/setup.bash first. Original error: {exc}") from exc

    app = window = temporary_settings = None
    controller_source = None
    controller_goal_seen = False
    controller_path_seen = False

    def spin_once(node, timeout_sec=.1):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
        if app is not None:
            app.processEvents()

    def controller_checks():
        return {
            "controller_goal_published": controller_goal_seen,
            "controller_live_map": bool(window.map_panel.live and window.map_panel.view.data is not None),
            "controller_robot_pose": window.bridge.pose is not None,
            "controller_path_displayed": controller_path_seen,
        }

    class VerificationNode(Node):
        def __init__(self):
            remaps = (["--ros-args", "-r", "/goal_pose:=/smart_vision_amr/goal",
                       "-r", "/plan:=/smart_vision_amr/path"] if args.via_controller else None)
            super().__init__("smart_vision_amr_sim_verifier", parameter_overrides=[
                Parameter("use_sim_time", value=True)], cli_args=remaps)
            self.first_clock = None
            self.last_clock = None
            self.map = None
            self.scan = None
            self.odom = None
            self.controller_state = "unknown"
            self.state_future = None
            self.next_state_request = 0.0
            self.tf_error = "not queried"
            self.started = False
            self.start_odom = None
            self.path_poses = 0
            self.path_count = 0
            self.path_received = False
            self.velocity_seen = False
            self.max_displacement = 0.0
            self.navigation_status = None
            self.last_status_payload = None
            self.status_parse_errors = 0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.controller = self.create_client(GetState, "/controller_server/get_state")
            self.follow_path = ActionClient(self, FollowPath, "/follow_path")
            self.cancel = self.create_client(Trigger, "/smart_vision_amr/cancel_navigation")
            self.goal_pub = self.create_publisher(PoseStamped, "/smart_vision_amr/goal", 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel", self.on_velocity, 10)
            self.create_subscription(Path, "/smart_vision_amr/path", self.on_path, 10)
            self.create_subscription(String, "/smart_vision_amr/navigation_status",
                                     self.on_status, 10)
            if args.via_controller:
                self.create_subscription(PoseStamped, "/smart_vision_amr/goal", self.on_goal, 10)

        def on_goal(self, message):
            nonlocal controller_goal_seen
            if self.started and message.header.frame_id == "map":
                point = message.pose.position
                controller_goal_seen = (abs(point.x - args.goal_x) < 1e-6
                                        and abs(point.y - args.goal_y) < 1e-6)

        def on_clock(self, message):
            value = message.clock.sec + message.clock.nanosec / 1e9
            if self.first_clock is None:
                self.first_clock = value
            self.last_clock = value

        def on_map(self, message):
            self.map = message

        def on_scan(self, message):
            self.scan = message

        def on_odom(self, message):
            self.odom = message
            if self.started and self.start_odom is not None:
                point = message.pose.pose.position
                self.max_displacement = max(self.max_displacement, math.hypot(
                    point.x - self.start_odom[0], point.y - self.start_odom[1]))

        def on_velocity(self, message):
            if self.started and (math.hypot(message.linear.x, message.linear.y) > 1e-3
                                 or abs(message.angular.z) > 1e-3):
                self.velocity_seen = True

        def on_path(self, message):
            if self.started:
                self.path_received = True
                self.path_poses = len(message.poses)
                if message.poses:
                    self.path_count += 1

        def on_status(self, message):
            # Status has no goal ID. Gate on a newly observed A* path so a
            # status received during startup cannot satisfy this run.
            if not self.started or not self.path_received:
                return
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("status JSON is not an object")
            except (ValueError, TypeError):
                self.status_parse_errors += 1
                return
            self.last_status_payload = payload
            self.navigation_status = payload.get("status")

        def update_controller_state(self):
            if self.state_future is not None and self.state_future.done():
                try:
                    state = self.state_future.result().current_state
                    self.controller_state = ("active" if state.id == State.PRIMARY_STATE_ACTIVE
                                             else f"{state.label} ({state.id})")
                except Exception as exc:
                    self.controller_state = f"get_state error: {exc}"
                self.state_future = None
            now = time.monotonic()
            if (self.state_future is None and now >= self.next_state_request
                    and self.controller.service_is_ready()):
                self.state_future = self.controller.call_async(GetState.Request())
                self.next_state_request = now + 1.0

        def robot_pose(self):
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time())
                self.tf_error = None
                point = transform.transform.translation
                return (point.x, point.y)
            except TransformException as exc:
                self.tf_error = str(exc)
                return None

        def readiness(self):
            self.update_controller_state()
            pose = self.robot_pose()
            checks = {
                "clock_advances": (self.first_clock is not None and self.last_clock is not None
                                   and self.last_clock - self.first_clock >= 0.1),
                "map": self.map is not None and self.map.info.width > 0,
                "scan": self.scan is not None and bool(self.scan.ranges),
                "odom": self.odom is not None,
                "map_to_base_footprint_tf": pose is not None,
                "controller_active": self.controller_state == "active",
                "follow_path_action": self.follow_path.server_is_ready(),
                "goal_subscriber": self.goal_pub.get_subscription_count() > 0,
                "status_publisher": self.count_publishers(
                    "/smart_vision_amr/navigation_status") > 0,
            }
            if args.via_controller:
                checks.update({
                    "controller_live_map": bool(window.map_panel.live and window.map_panel.view.data is not None),
                    "controller_robot_pose": window.bridge.pose is not None,
                    "controller_goal_subscriber": window.bridge.goal_pose_pub.get_subscription_count() > 0,
                })
            return checks

        def validate_goal(self):
            # The existing planner raises on an out-of-bounds goal. Refuse
            # obviously invalid requests before exercising it.
            info = self.map.info
            q = info.origin.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            dx, dy = args.goal_x - info.origin.position.x, args.goal_y - info.origin.position.y
            cell_x = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / info.resolution)
            cell_y = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / info.resolution)
            if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                raise RuntimeError(f"Goal ({args.goal_x}, {args.goal_y}) maps to out-of-bounds "
                                   f"cell ({cell_x}, {cell_y}) in {info.width}x{info.height} map")
            occupancy = self.map.data[cell_y * info.width + cell_x]
            if occupancy < 0 or occupancy >= 65:
                raise RuntimeError(f"Goal cell ({cell_x}, {cell_y}) is occupied/unknown "
                                   f"(occupancy {occupancy}); select a free-space goal")

        def send_goal(self):
            self.validate_goal()
            point = self.odom.pose.pose.position
            self.start_odom = (point.x, point.y)
            self.path_poses = self.path_count = 0
            self.path_received = False
            self.velocity_seen = False
            self.max_displacement = 0.0
            self.navigation_status = self.last_status_payload = None
            self.started = True
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = args.goal_x, args.goal_y
            goal.pose.orientation.z = math.sin(args.goal_yaw / 2.0)
            goal.pose.orientation.w = math.cos(args.goal_yaw / 2.0)
            if args.via_controller:
                # Use the same selected map point and Goal handler as the GUI.
                # This path does not publish a substitute goal from the verifier.
                window.map_panel.view.waypoints = [(args.goal_x, args.goal_y, args.goal_yaw)]
                window.map_panel.view.draw_overlays()
                window.map_panel.start_v3_goal()
            else:
                self.goal_pub.publish(goal)

        def evidence(self):
            pose = self.robot_pose()
            distance = None if pose is None else math.hypot(
                pose[0] - args.goal_x, pose[1] - args.goal_y)
            return {
                "nonempty_path": self.path_count > 0,
                "nonzero_cmd_vel": self.velocity_seen,
                "odom_displacement_over_0_3m": self.max_displacement > 0.3,
                "navigation_succeeded": self.navigation_status == "SUCCEEDED",
                "goal_distance_within_0_35m": distance is not None and distance <= 0.35,
            }, {
                "goal": [args.goal_x, args.goal_y, args.goal_yaw],
                "robot_map_xy": pose,
                "final_distance_m": distance,
                "max_odom_displacement_m": self.max_displacement,
                "last_path_poses": self.path_poses,
                "nonempty_paths_received": self.path_count,
                "last_navigation_status": self.last_status_payload,
                "status_parse_errors": self.status_parse_errors,
                "tf_error": self.tf_error,
            }

        def cancel_on_failure(self):
            if not self.started or not self.cancel.service_is_ready():
                return
            future = self.cancel.call_async(Trigger.Request())
            deadline = time.monotonic() + 3.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    reply = future.result()
                    print(f"Cancellation after failed verification: {reply.message}", flush=True)
                except Exception as exc:
                    print(f"Cancellation service failed: {exc}", file=sys.stderr, flush=True)

    rclpy.init(args=[])
    node = VerificationNode()
    passed = False
    try:
        if args.via_controller:
            os.environ["QT_QPA_PLATFORM"] = "offscreen"
            from PyQt5 import QtCore, QtWidgets
            controller_source = args.controller.expanduser().resolve(strict=True)
            sys.path.insert(0, str(controller_source.parent))
            spec = importlib.util.spec_from_file_location("navigation_validation_controller", controller_source)
            program = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(program)
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["verify_sim"])
            app.setQuitOnLastWindowClosed(False)
            temporary_settings = tempfile.TemporaryDirectory(prefix="navigation-controller-")
            settings = QtCore.QSettings(str(FilePath(temporary_settings.name) / "controller.ini"),
                                       QtCore.QSettings.IniFormat)
            window = program.Controller(
                program.LazyVelocityPublisher(node), domain_id=76,
                settings=settings, node=node, camera=None,
                navigation_cancel_service="/smart_vision_amr/cancel_navigation")

            def path_displayed(plan):
                nonlocal controller_path_seen
                if node.started and plan:
                    controller_path_seen = True
            window.bridge.planReceived.connect(path_displayed)
        readiness_deadline = time.monotonic() + args.ready_timeout
        next_report = 0.0
        while rclpy.ok():
            spin_once(node, timeout_sec=0.1)
            checks = node.readiness()
            if all(checks.values()):
                break
            now = time.monotonic()
            if now >= readiness_deadline:
                details = {"checks": checks, "controller_state": node.controller_state,
                           "tf_error": node.tf_error, "clock": node.last_clock}
                raise RuntimeError("Readiness timed out: " + json.dumps(details, ensure_ascii=False))
            if now >= next_report:
                waiting = ", ".join(name for name, ready in checks.items() if not ready)
                print(f"Waiting for simulation: {waiting}", flush=True)
                next_report = now + 10.0
        if not rclpy.ok():
            raise RuntimeError("ROS shut down before the simulation became ready")
        node.send_goal()
        started_at = time.monotonic()
        deadline = started_at + args.timeout
        next_report = started_at + 10.0
        print(f"Simulation ready; sent map goal ({args.goal_x}, {args.goal_y}, "
              f"yaw={args.goal_yaw}). Waiting up to {args.timeout:g} wall seconds.", flush=True)
        while rclpy.ok():
            spin_once(node, timeout_sec=0.1)
            checks, details = node.evidence()
            if args.via_controller:
                checks.update(controller_checks())
                details["controller"] = {
                    "source": str(controller_source),
                    "sha256": hashlib.sha256(controller_source.read_bytes()).hexdigest(),
                    "goal_handler": "MapPanel.start_v3_goal",
                    "goal_topic": window.bridge.goal_pose_pub.topic_name,
                    "displayed_path_poses": len(window.map_panel.view.plan),
                    "robot_map_pose": window.bridge.pose,
                    "map_panel_status": window.map_panel.status.text(),
                }
            if node.navigation_status in ("FAILED", "CANCELED"):
                raise RuntimeError("Navigation ended without success: "
                                   + json.dumps(details, ensure_ascii=False))
            if all(checks.values()):
                details["wall_elapsed_seconds"] = round(time.monotonic() - started_at, 3)
                if args.via_controller:
                    screenshot = args.controller_screenshot
                    if screenshot is None:
                        workspace = FilePath(os.environ.get("SMART_VISION_SIM_WORKSPACE", "/tmp"))
                        screenshot = workspace / "log/controller_validation" / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".png")
                    screenshot = screenshot.expanduser().resolve()
                    if screenshot.exists():
                        raise RuntimeError(f"Refusing to overwrite controller screenshot: {screenshot}")
                    screenshot.parent.mkdir(parents=True, exist_ok=True)
                    # Stay hidden throughout driving; render only after arrival.
                    window.resize(1280, 960)
                    window.show()
                    app.processEvents()
                    window.map_panel.view.fit_map()
                    app.processEvents()
                    if not window.grab().save(str(screenshot)):
                        raise RuntimeError(f"Could not save controller screenshot: {screenshot}")
                    details["controller"]["screenshot"] = str(screenshot)
                print("PASS " + json.dumps({"checks": checks, **details},
                                           ensure_ascii=False), flush=True)
                passed = True
                return 0
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError("Navigation verification timed out: " + json.dumps(
                    {"checks": checks, **details}, ensure_ascii=False))
            if now >= next_report:
                print("Navigation evidence: " + json.dumps({"checks": checks, **details},
                                                           ensure_ascii=False), flush=True)
                next_report = now + 10.0
        raise RuntimeError("ROS shut down during navigation verification")
    finally:
        if not passed and rclpy.ok():
            if window is not None:
                window.emergency_stop()
            node.cancel_on_failure()
        if window is not None:
            window.timer.stop()
            window.bridge.timer.stop()
            window.video_timer.stop()
            app.removeEventFilter(window)
            window.deleteLater()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if temporary_settings is not None:
            temporary_settings.cleanup()


def main():
    args = parse_args()
    try:
        check_isolation()
        return run_verification(args)
    except KeyboardInterrupt:
        print("FAIL Verification interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
