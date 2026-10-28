#!/usr/bin/env python3
"""Drive the default validation room with the real PyQt controller and save SLAM.

Start ./scripts/run_sim.sh slam:=true first, then ./scripts/check_slam.sh.
This verifier attaches to the running simulator; it does not start or reset it.
The fixed route assumes simulation_room.world and the default (-3, -2, 0) spawn.
Existing maps are never loaded. Navigation/AMCL nodes must be absent.
"""

import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from verify_sim import check_isolation


SURVEY_CORNERS = [(2.2, 2.8), (2.2, 1.2), (3.8, 1.2), (3.8, 2.8)]
START_SURVEY_WORLD_ROUTE = [(-2.0, -2.0), (-3.0, -2.0)]
# The interior survey supplies finite east/north wall returns through the
# standard (3, 2) goal. Perimeter-only rays often exceed the LDS 3.5 m range
# and leave unknown wedges even in physically unobstructed floor space.
WORLD_ROUTE = [(-3.0, -3.0), (4.0, -3.0), (4.0, 3.0), *SURVEY_CORNERS, (4.0, 3.0),
               (-4.0, 3.0), (-4.0, -3.0), (-3.0, -2.0), *START_SURVEY_WORLD_ROUTE]
CORE_CHECKS = ("real_controller_drives_robot", "all_waypoints_reached", "release_stops_robot",
               "fresh_map_grows", "map_has_free_and_occupied_cells", "map_to_base_tf",
               "controller_live_map", "controller_live_robot_pose")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", type=Path,
                        default=Path("/home/kiki/turtlebot3/tools/cmd_vel_controller.py"),
                        help="Existing controller source, imported without copying it.")
    parser.add_argument("--output", type=Path,
                        help="New map prefix; default WORKSPACE/log/slam_validation/TIMESTAMP/map.")
    parser.add_argument("--continue-from", type=Path, metavar="FAILED_REPORT.json",
                        help="Continue the same live SLAM run after a map-usability-only failure, "
                             "retaining its fresh-map and motion evidence and surveying the start pad.")
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.add_argument("--timeout", type=float, default=420.0,
                        help="Wall-clock budget for driving, excluding initial readiness.")
    parser.add_argument("--linear-speed", type=float, default=0.22)
    parser.add_argument("--angular-speed", type=float, default=0.8)
    options = parser.parse_args()
    for name in ("ready_timeout", "timeout", "linear_speed", "angular_speed"):
        if not math.isfinite(getattr(options, name)) or getattr(options, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if options.linear_speed > 0.22 or not 0.1 <= options.angular_speed <= 1.0:
        parser.error("linear speed must be <= 0.22 and angular speed 0.1..1.0")
    if options.linear_speed < 0.01:
        parser.error("linear speed must be >= 0.01")
    return options


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y**2 + quaternion.z**2))


def map_evidence(message):
    cells = message.data
    known = sum(value >= 0 for value in cells)
    occupied = sum(value >= 65 for value in cells)
    return {"width": message.info.width, "height": message.info.height,
            "resolution_m": message.info.resolution, "frame": message.header.frame_id,
            "known_cells": known, "occupied_cells": occupied,
            "known_area_m2": round(known * message.info.resolution**2, 3)}


def navigation_usability(message, initial_map_pose):
    """Check the actual team's inflated A* against the standard detour goal."""
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.map_utils import occupancy_to_grid, world_to_cell

    # The first map pose anchors the known world spawn (-3, -2, yaw=0).
    # SLAM's map gauge need not share the world origin on other installations.
    mx, my, heading = initial_map_pose
    goal = (mx + math.cos(heading) * 6 - math.sin(heading) * 4,
            my + math.sin(heading) * 6 + math.cos(heading) * 4)
    grid = occupancy_to_grid(message, 65, 6)
    start_cell = world_to_cell(message.info, mx, my)
    goal_cell = world_to_cell(message.info, *goal)
    path = (AStarPlanner().plan(grid, start_cell, goal_cell)
            if grid.contains(start_cell) and grid.contains(goal_cell) else [])
    # A full 0.4 m square around the goal also covers the team's 6-cell
    # square inflation band, including cells diagonally beside the goal.
    radius = math.ceil(.4 / message.info.resolution)
    unknown = occupied = outside = free = 0
    for y in range(goal_cell[1] - radius, goal_cell[1] + radius + 1):
        for x in range(goal_cell[0] - radius, goal_cell[0] + radius + 1):
            if not grid.contains((x, y)):
                outside += 1
                continue
            value = message.data[y * message.info.width + x]
            if value < 0:
                unknown += 1
            elif value >= 65:
                occupied += 1
            else:
                free += 1
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) * message.info.resolution
                 for a, b in zip(path, path[1:]))
    checks = {"standard_detour_astar_path": bool(path),
              "standard_goal_known_free_neighborhood": unknown == occupied == outside == 0}
    details = {"start_map_xy": [mx, my], "goal_world_xy": [3.0, 2.0],
               "goal_map_xy": list(goal), "start_cell": list(start_cell), "goal_cell": list(goal_cell),
               "occupied_threshold": 65, "inflation_cells": 6,
               "static_start_blocked": grid.is_static_obstacle(start_cell),
               "static_goal_blocked": grid.is_static_obstacle(goal_cell),
               "astar_path_cells": len(path), "astar_path_length_m": length,
               "goal_neighborhood_half_width_m": .4,
               "goal_neighborhood_free_cells": free, "goal_neighborhood_unknown_cells": unknown,
               "goal_neighborhood_occupied_cells": occupied, "goal_neighborhood_outside_cells": outside}
    return checks, details


def continuation_report(path, controller_sha256):
    """Accept only a completed fresh drive that failed final map usability."""
    source = path.expanduser().resolve(strict=True)
    raw = source.read_bytes()
    previous = json.loads(raw)
    checks = previous.get("checks", {})
    allowed_failures = {"standard_detour_astar_path", "standard_goal_known_free_neighborhood"}
    failures = {name for name, passed in checks.items() if passed is not True}
    if (previous.get("passed") is not False
            or not all(checks.get(name) is True for name in CORE_CHECKS)
            or not failures or not failures <= allowed_failures
            or not previous.get("error", "").startswith("SLAM verification checks failed:")
            or previous.get("controller", {}).get("sha256") != controller_sha256
            or previous.get("isolation", {}).get("ROS_DOMAIN_ID") != "76"
            or previous.get("isolation", {}).get("ROS_LOCALHOST_ONLY") != "1"):
        raise RuntimeError("--continue-from requires a failed map-usability report whose fresh SLAM, "
                           "complete route, stops, controller source and simulator isolation all passed")
    if len(previous.get("waypoints", [])) != len(previous.get("route_world_xy", [])):
        raise RuntimeError("Continuation report has an incomplete route")
    for name in ("initial_odom", "initial_robot_map_pose"):
        value = previous.get(name)
        if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(v) for v in value):
            raise RuntimeError(f"Continuation report has no valid {name}")
    return previous, {"report": str(source), "sha256": hashlib.sha256(raw).hexdigest(),
                      "previous_failure": previous["error"], "previous_started_at": previous["started_at"]}


def run(args, prefix, report):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from PyQt5 import QtCore, QtWidgets
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                          qos_profile_sensor_data)
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    # Import the actual program. Do not launch its main(), read saved domains,
    # or touch the user's persistent controller settings.
    source = args.controller.expanduser().resolve(strict=True)
    sys.path.insert(0, str(source.parent))
    spec = importlib.util.spec_from_file_location("slam_validation_controller", source)
    program = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(program)
    report["controller"] = {"source": str(source),
                            "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                            "input": "real QPushButton pressed/released signals and 20 Hz Controller.tick"}
    previous = None
    if args.continue_from is not None:
        previous, provenance = continuation_report(args.continue_from, report["controller"]["sha256"])
        report["continuation"] = provenance

    class Observer(Node):
        def __init__(self):
            super().__init__("slam_controller_verifier", parameter_overrides=[
                Parameter("use_sim_time", value=True)])
            self.odom = self.scan = self.map = None
            self.first_clock = self.last_clock = None
            self.last_odom_wall = None
            self.last_scan_wall = None
            self.path_length = 0.0
            self.cmd_nonzero = self.cmd_zero = 0
            self.map_messages = 0
            self.map_save_samples = None
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, durable)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

        def on_map(self, message):
            self.map = message
            self.map_messages += 1
            if self.map_save_samples is not None:
                self.map_save_samples.append(message)

        def on_odom(self, message):
            if self.odom is not None:
                before, after = self.odom.pose.pose.position, message.pose.pose.position
                self.path_length += math.hypot(after.x - before.x, after.y - before.y)
            self.odom = message
            self.last_odom_wall = time.monotonic()

        def on_scan(self, message):
            self.scan = message
            self.last_scan_wall = time.monotonic()

        def on_clock(self, message):
            self.last_clock = message.clock.sec + message.clock.nanosec / 1e9
            if self.first_clock is None:
                self.first_clock = self.last_clock

        def on_cmd(self, message):
            if abs(message.linear.x) > 1e-3 or abs(message.angular.z) > 1e-3:
                self.cmd_nonzero += 1
            else:
                self.cmd_zero += 1

        def pose(self):
            p = self.odom.pose.pose
            return p.position.x, p.position.y, yaw_of(p.orientation)

        def map_pose(self):
            try:
                tf = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                p = tf.transform.translation
                return [p.x, p.y, yaw_of(tf.transform.rotation)]
            except TransformException:
                return None

        def stack_check(self):
            names = {name for name, _namespace in self.get_node_names_and_namespaces()}
            forbidden = names.intersection({"amcl", "map_server", "controller_server",
                                             "astar_planner", "path_follower", "change_memory"})
            if forbidden:
                raise RuntimeError("Fresh SLAM requires navigation/localization stopped; found "
                                   + ", ".join(sorted(forbidden)))
            publishers = self.get_publishers_info_by_topic("/map")
            if publishers and (len(publishers) != 1 or publishers[0].node_name != "slam_toolbox"
                               or publishers[0].node_namespace != "/"):
                endpoints = [info.node_namespace.rstrip("/") + "/" + info.node_name
                             for info in publishers]
                raise RuntimeError("Fresh SLAM requires exactly one /map publisher, /slam_toolbox; found "
                                   + ", ".join(endpoints))
            return "slam_toolbox" in names and len(publishers) == 1

        def ready(self):
            return {"slam_toolbox_only_map_source": self.stack_check(),
                    "clock_advances": self.first_clock is not None and self.last_clock > self.first_clock + .1,
                    "odom": self.odom is not None,
                    "scan": self.scan is not None and bool(self.scan.ranges),
                    "map": self.map is not None and self.map.info.width > 0,
                    "map_to_base_tf": self.map_pose() is not None}

        def forward_clearance(self):
            scan = self.scan
            values = [distance for index, distance in enumerate(scan.ranges)
                      if abs(angle(scan.angle_min + index * scan.angle_increment)) < .25
                      and math.isfinite(distance) and scan.range_min < distance < scan.range_max]
            return min(values, default=math.inf)

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Observer()
    temporary = tempfile.TemporaryDirectory(prefix="slam-controller-")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["verify_slam"])
    app.setQuitOnLastWindowClosed(False)
    publisher = node.create_publisher(Twist, "/cmd_vel", 1)
    settings = QtCore.QSettings(str(Path(temporary.name) / "controller.ini"), QtCore.QSettings.IniFormat)
    window = program.Controller(publisher, domain_id=76, settings=settings, node=node, camera=None)
    window.linear_speed.setValue(args.linear_speed)
    window.angular_speed.setValue(args.angular_speed)
    current_button = None
    started = time.monotonic()
    route_deadline = None

    def pump(duration=.05):
        until = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)
            app.processEvents()
        if not rclpy.ok():
            raise RuntimeError("ROS context shut down during verification")

    def stop():
        nonlocal current_button
        if current_button is not None:
            button = window.buttons[current_button]
            button.setDown(False)
            button.released.emit()
            current_button = None
        window.stop()

    def hold(name):
        nonlocal current_button
        if current_button == name:
            return
        stop()
        button = window.buttons[name]
        button.setDown(True)
        button.pressed.emit()
        current_button = name
        if window.active is not button:
            raise RuntimeError("Controller refused manual input: " + window.status.text())

    def ensure_running():
        if time.monotonic() > route_deadline:
            raise RuntimeError(f"SLAM route exceeded {args.timeout:g} wall seconds")
        if time.monotonic() - node.last_odom_wall > 2:
            raise RuntimeError("Odometry stopped updating; manual drive stopped")
        if node.last_scan_wall is None or time.monotonic() - node.last_scan_wall > 2:
            raise RuntimeError("Laser scan stopped updating; manual drive stopped")

    def turn_to(target_yaw):
        while True:
            ensure_running()
            error = angle(target_yaw - node.pose()[2])
            if abs(error) < .025:
                break
            window.angular_speed.setValue(min(args.angular_speed, max(.1, abs(error) * 1.8)))
            hold("left" if error > 0 else "right")
            pump()
        stop()
        pump(.15)

    def assert_stop(label):
        stop()
        zero_before = node.cmd_zero
        pump(.8)
        origin = node.pose()
        pump(.7)
        displacement = math.hypot(node.pose()[0] - origin[0], node.pose()[1] - origin[1])
        speed = node.odom.twist.twist
        passed = (displacement < .035 and abs(speed.linear.x) < .025
                  and abs(speed.angular.z) < .05 and node.cmd_zero > zero_before)
        evidence = {"stage": label, "passed": passed,
                    "stationary_displacement_m": displacement,
                    "linear_m_s": speed.linear.x, "angular_rad_s": speed.angular.z,
                    "zero_commands_received": node.cmd_zero - zero_before}
        report.setdefault("stop_checks", []).append(evidence)
        if not passed:
            raise RuntimeError("Controller release did not stop robot: " + json.dumps(evidence))

    try:
        next_report = 0.0
        while True:
            pump(.1)
            checks = node.ready()
            if all(checks.values()) and window.map_panel.view.data is not None and window.bridge.pose:
                break
            now = time.monotonic()
            if now - started > args.ready_timeout:
                raise RuntimeError("SLAM/controller readiness timed out: " + json.dumps(checks))
            if now >= next_report:
                print("Waiting: " + ", ".join(name for name, ready in checks.items() if not ready), flush=True)
                next_report = now + 10
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("Controller /cmd_vel publisher has no simulator subscriber")
        current_start_map = map_evidence(node.map)
        current_start_odom = list(node.pose())
        current_start_map_pose = node.map_pose()
        route = START_SURVEY_WORLD_ROUTE if previous else WORLD_ROUTE
        report["initial_map"] = current_start_map
        report["initial_odom"] = current_start_odom
        report["initial_robot_map_pose"] = current_start_map_pose
        report["route_world_xy"] = list(route)
        report["waypoints"] = []
        if previous:
            old_map = previous["final_map"]
            if (current_start_map["known_cells"] < old_map["known_cells"] * .95
                    or current_start_map["frame"] != old_map["frame"]
                    or abs(current_start_map["resolution_m"] - old_map["resolution_m"]) > 1e-6):
                raise RuntimeError("Live map does not match the completed prior SLAM run; "
                                   "do not reset/restart SLAM before --continue-from")
            report["continuation_start_map"] = current_start_map
            report["continuation_start_odom"] = current_start_odom
            report["continuation_start_robot_map_pose"] = current_start_map_pose
            for name in ("initial_map", "initial_odom", "initial_robot_map_pose"):
                report[name] = previous[name]
            report["route_world_xy"] = previous["route_world_xy"] + list(route)
            report["waypoints"] = list(previous["waypoints"])
            report["stop_checks"] = list(previous["stop_checks"])
            report["fresh_slam_started_at"] = previous.get("fresh_slam_started_at", previous["started_at"])
        report["readiness"] = checks
        report["speeds"] = {"linear_m_s": args.linear_speed, "angular_rad_s": args.angular_speed}
        current_x, current_y, current_yaw = node.pose()
        # The validation model publishes Gazebo world-frame odometry. Refuse a
        # moved robot or custom spawn before applying this world-specific route.
        # Blindly shifting the route to any initial odometry pose is unsafe.
        spawn_distance = math.hypot(current_x + 3, current_y + 2)
        if spawn_distance > .15 or abs(angle(current_yaw)) > .1:
            raise RuntimeError("SLAM route requires a fresh default spawn (-3, -2, yaw=0); "
                               f"odom was ({current_x:.3f}, {current_y:.3f}, {current_yaw:.3f}). "
                               "Restart run_sim.sh slam:=true without custom spawn arguments.")
        report["default_spawn_check"] = {"passed": True, "position_error_m": spawn_distance,
                                         "yaw_error_rad": abs(angle(current_yaw))}
        ox, oy, oyaw = report["initial_odom"]
        # Account for the small settling error of the validated default spawn.
        def target_odom(wx, wy):
            dx, dy = wx + 3, wy + 2
            return ox + math.cos(oyaw) * dx - math.sin(oyaw) * dy, oy + math.sin(oyaw) * dx + math.cos(oyaw) * dy
        route_deadline = time.monotonic() + args.timeout
        route_started = time.monotonic()
        print(("Continuing prior SLAM start survey" if previous else "Fresh SLAM ready")
              + "; driving via real controller button signals.", flush=True)
        assert_stop("before_continuation" if previous else "before_route")
        min_clearance = previous.get("minimum_forward_scan_m") if previous else None
        min_clearance = min_clearance if min_clearance is not None else math.inf
        previous_waypoints = len(report["waypoints"])
        for index, (wx, wy) in enumerate(route, previous_waypoints + 1):
            tx, ty = target_odom(wx, wy)
            before = node.pose()
            turn_to(math.atan2(ty - before[1], tx - before[0]))
            stall_origin = node.pose()
            stall_started = time.monotonic()
            while True:
                ensure_running()
                px, py, heading = node.pose()
                distance = math.hypot(tx - px, ty - py)
                if distance < .075:
                    break
                error = angle(math.atan2(ty - py, tx - px) - heading)
                if abs(error) > .065:
                    turn_to(math.atan2(ty - py, tx - px))
                    stall_origin = node.pose()
                    stall_started = time.monotonic()
                clearance = node.forward_clearance()
                min_clearance = min(min_clearance, clearance)
                if clearance < .34:
                    raise RuntimeError(f"Obstacle within {clearance:.3f} m of forward scan; route stopped")
                window.linear_speed.setValue(min(args.linear_speed, max(.05, distance * .9)))
                hold("forward")
                pump()
                if time.monotonic() - stall_started >= 5:
                    if math.hypot(node.pose()[0] - stall_origin[0], node.pose()[1] - stall_origin[1]) < .08:
                        raise RuntimeError("Robot stalled despite controller forward commands")
                    stall_origin = node.pose()
                    stall_started = time.monotonic()
            assert_stop(f"waypoint_{index}")
            if (wx, wy) in SURVEY_CORNERS:
                pump(1)
            waypoint = {"index": index, "world_xy": [wx, wy], "odom_xy": [tx, ty],
                        "actual_odom": list(node.pose()), "error_m": distance}
            report["waypoints"].append(waypoint)
            print(f"Waypoint {index}/{len(report['route_world_xy'])} reached; odom error {distance:.3f} m, "
                  f"known area {map_evidence(node.map)['known_area_m2']:.1f} m²", flush=True)
        turn_to(oyaw)
        assert_stop("final")
        # Allow asynchronous scan matching/map publication to include the loop closure.
        pump(4)
        report["final_map"] = map_evidence(node.map)
        report["final_robot_map_pose"] = node.map_pose()
        report["path_length_m"] = node.path_length + (previous["path_length_m"] if previous else 0)
        report["map_messages"] = node.map_messages + (previous["map_messages"] if previous else 0)
        report["nonzero_commands_received"] = node.cmd_nonzero + (previous["nonzero_commands_received"] if previous else 0)
        report["minimum_forward_scan_m"] = min_clearance if math.isfinite(min_clearance) else None
        growth = report["final_map"]["known_cells"] - report["initial_map"]["known_cells"]
        report["known_cell_growth"] = growth
        report["checks"] = {
            "real_controller_drives_robot": report["nonzero_commands_received"] > 20 and report["path_length_m"] > 20,
            "all_waypoints_reached": len(report["waypoints"]) == len(report["route_world_xy"]),
            "release_stops_robot": all(check["passed"] for check in report["stop_checks"]),
            "fresh_map_grows": growth >= 500,
            "map_has_free_and_occupied_cells": report["final_map"]["occupied_cells"] > 100
                                              and report["final_map"]["known_cells"] > 1000,
            "map_to_base_tf": node.map_pose() is not None,
            "controller_live_map": window.map_panel.live and window.map_panel.view.data is not None,
            "controller_live_robot_pose": window.bridge.pose is not None,
        }
        usability_checks, usability = navigation_usability(node.map, report["initial_robot_map_pose"])
        report["checks"].update(usability_checks)
        report["navigation_usability"] = usability
        if not all(report["checks"].values()):
            raise RuntimeError("SLAM verification checks failed: " + json.dumps(report["checks"]))
        screenshot = prefix.with_name(prefix.name + "_controller.png")
        window.resize(1280, 960)
        window.show()
        pump(.2)
        window.map_panel.view.fit_map()
        app.processEvents()
        if not window.grab().save(str(screenshot)):
            raise RuntimeError("Could not save controller map screenshot")
        report["controller_screenshot"] = str(screenshot)
        command = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-t", "/map", "-f", str(prefix),
                   "--fmt", "pgm", "--occ", "0.65", "--free", "0.196", "--ros-args",
                   "-p", "use_sim_time:=true", "-p", "map_subscribe_transient_local:=true",
                   "-p", "save_map_timeout:=10.0"]
        # Retain grids seen during saving: the map saver receives its own
        # transient-local subscription, potentially one update before ours.
        node.map_save_samples = [node.map]
        with tempfile.TemporaryFile(mode="w+t") as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, text=True)
            save_started = time.monotonic()
            try:
                while process.poll() is None:
                    if time.monotonic() - save_started > 25:
                        raise RuntimeError("map_saver_cli exceeded 25 wall seconds")
                    pump(.1)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            output.seek(0)
            saver_log = output.read()
        yaml_path, image_path = prefix.with_suffix(".yaml"), prefix.with_suffix(".pgm")
        if process.returncode != 0 or not yaml_path.is_file() or not image_path.is_file():
            raise RuntimeError("map_saver_cli failed: " + saver_log[-4000:])
        import yaml
        metadata = yaml.safe_load(yaml_path.read_text())
        if not isinstance(metadata, dict) or metadata.get("resolution", 0) <= 0 or image_path.stat().st_size < 100:
            raise RuntimeError("Saved map metadata/image is invalid")
        # Exercise the controller's YAML/image loader against the freshly saved
        # output as well as its live /map path.
        from navigation_panels import MapData
        import numpy as np
        reloaded = MapData.from_yaml(str(yaml_path))
        if reloaded.w < 20 or reloaded.h < 20 or (reloaded.cells == 0).sum() < 100:
            raise RuntimeError("Saved map could not be reloaded as useful free space in the controller")
        report["checks"]["controller_reloads_saved_map"] = True
        pump(.2)
        matching_live_map = None
        matched_origin_error = None
        origin_tolerance = reloaded.resolution * .1
        for sample in node.map_save_samples:
            if sample.info.width != reloaded.w or sample.info.height != reloaded.h:
                continue
            raw = np.asarray(sample.data, dtype=np.int16).reshape(reloaded.h, reloaded.w)
            origin = sample.info.origin
            origin_error = [abs(reloaded.origin[0] - origin.position.x),
                            abs(reloaded.origin[1] - origin.position.y)]
            if (abs(reloaded.resolution - sample.info.resolution) < 1e-6
                    # Humble's saver writes YAML origins with only three
                    # significant digits. Permit its sub-cell rounding while
                    # retaining exact occupancy-cell agreement.
                    and max(origin_error) <= origin_tolerance
                    and abs(angle(reloaded.origin[2] - yaw_of(origin.orientation))) < 1e-6
                    and np.array_equal(raw, reloaded.cells)):
                matching_live_map = sample
                matched_origin_error = origin_error
                break
        if matching_live_map is None:
            raise RuntimeError("Saved map differs from live SLAM cells after controller reload; "
                               "unknown/free/occupied states must survive the YAML/PGM round trip")
        report["checks"]["saved_map_preserves_all_live_cells"] = True
        report["saved_map_fidelity"] = {
            "unknown_cells": int((reloaded.cells < 0).sum()),
            "free_cells": int((reloaded.cells == 0).sum()),
            "occupied_cells": int((reloaded.cells == 100).sum()),
            "all_cell_values_equal_live_map": True,
            "live_map_candidates": len(node.map_save_samples),
            "free_threshold": metadata["free_thresh"],
            "origin_rounding_error_xy_m": matched_origin_error,
            "origin_rounding_error_distance_m": math.hypot(*matched_origin_error),
            "origin_rounding_tolerance_per_axis_m": origin_tolerance,
        }
        # The output map must satisfy the same planning precondition as the
        # live final map; do not accept a different saver snapshot blindly.
        from copy import deepcopy
        saved_grid = deepcopy(matching_live_map)
        saved_grid.info.resolution = reloaded.resolution
        saved_grid.info.origin.position.x, saved_grid.info.origin.position.y = reloaded.origin[:2]
        saved_grid.info.origin.orientation.z = math.sin(reloaded.origin[2] / 2)
        saved_grid.info.origin.orientation.w = math.cos(reloaded.origin[2] / 2)
        saved_grid.data = reloaded.cells.ravel().tolist()
        saved_checks, saved_usability = navigation_usability(saved_grid, report["initial_robot_map_pose"])
        report["saved_map_navigation_usability"] = saved_usability
        report["checks"]["saved_map_standard_detour_astar_path"] = all(saved_checks.values())
        if not all(saved_checks.values()):
            raise RuntimeError("Saved map cannot support the standard A* detour: "
                               + json.dumps(saved_usability))
        node.map_save_samples = None
        report["saved_map"] = {"yaml": str(yaml_path), "image": str(image_path),
                               "yaml_sha256": hashlib.sha256(yaml_path.read_bytes()).hexdigest(),
                               "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                               "controller_reload_size": [reloaded.w, reloaded.h],
                               "metadata": metadata}
        report["checks"]["map_saved"] = True
        report["route_wall_seconds"] = round(time.monotonic() - route_started, 3)
    finally:
        if rclpy.ok():
            stop()
            window.emergency_stop()
            try:
                pump(.4)
            except Exception:
                pass
        window.timer.stop()
        window.bridge.timer.stop()
        window.video_timer.stop()
        app.removeEventFilter(window)
        window.deleteLater()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        temporary.cleanup()


def main():
    args = arguments()
    report = {"passed": False, "started_at": datetime.now().astimezone().isoformat(),
              "isolation": {name: os.environ.get(name) for name in
                            ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "GAZEBO_MASTER_URI")}}
    prefix = None
    started = time.monotonic()
    try:
        check_isolation()
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        if args.output is None:
            workspace = Path(os.environ["SMART_VISION_SIM_WORKSPACE"])
            args.output = workspace / "log/slam_validation" / datetime.now().strftime("%Y%m%d_%H%M%S_%f") / "map"
        prefix = args.output.expanduser().resolve()
        if prefix.suffix:
            raise RuntimeError("--output must be a map prefix without a file extension")
        for suffix in (".yaml", ".pgm", ".png", ".json", "_controller.png"):
            candidate = prefix.with_name(prefix.name + suffix)
            if candidate.exists():
                raise RuntimeError(f"Refusing to overwrite existing validation artifact: {candidate}")
        prefix.parent.mkdir(parents=True, exist_ok=True)
        def interrupted(_signum, _frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
        run(args, prefix, report)
        report["passed"] = True
        code = 0
    except KeyboardInterrupt:
        report["error"] = "Verification interrupted; controller stop requested"
        code = 130
    except Exception as exc:
        report["error"] = str(exc)
        code = 1
    report["wall_seconds"] = round(time.monotonic() - started, 3)
    if prefix is not None and prefix.parent.is_dir():
        # Exclusive creation also protects prior evidence when argument checks fail.
        try:
            evidence = prefix.with_suffix(".json")
            with evidence.open("x") as output:
                json.dump(report, output, ensure_ascii=False, indent=2)
                output.write("\n")
            print(f"Evidence: {evidence}", flush=True)
        except FileExistsError:
            pass
    print(("PASS " if report["passed"] else "FAIL ") + json.dumps(report, ensure_ascii=False), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
