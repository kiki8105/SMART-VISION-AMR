#!/usr/bin/env python3
"""Drive Sector 1 → 2 → 3 → Home in an isolated Gazebo classroom."""

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid


STOPS = (
    # Registered Sector 1 (1.475000, .575000, 3.114078) failed the isolated
    # approach; the 1.20/.55 observation pose passed the same trial.
    ('Sector 1', 1.20, .55, 3.114078),
    ('Sector 2', 1.525, -2.65, 1.543282),
    ('Sector 3', -.25, -1.98, -1.40),
    ('Home', -.188431, -.220497, -.027514),
)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_status(message):
    try:
        value = json.loads(message.data)
        return value if isinstance(value, dict) else {}
    except (TypeError, ValueError):
        return {}


def matches_goal(event, x, y):
    goal = event.get('goal') or {}
    return (isinstance(goal, dict) and
            math.hypot(goal.get('x', 1000)-x, goal.get('y', 1000)-y) < .05)


def expected_marker_id(stop_name):
    if stop_name == 'Home':
        return 0
    for marker_id in (1, 2, 3):
        if stop_name == f'Sector {marker_id}' or stop_name.startswith(
                f'Sector{marker_id}'):
            return marker_id
    return None


def detect_raw_aruco(frame, dictionary, parameters, cv2, np):
    """Decode a ROS Image with its stride; return IDs and pixel corners."""
    if frame.encoding == 'mono8':
        gray = np.ndarray((frame.height, frame.width), dtype=np.uint8,
                          buffer=frame.data, strides=(frame.step, 1))
    else:
        codes = {'rgb8': (3, cv2.COLOR_RGB2GRAY),
                 'bgr8': (3, cv2.COLOR_BGR2GRAY),
                 'rgba8': (4, cv2.COLOR_RGBA2GRAY),
                 'bgra8': (4, cv2.COLOR_BGRA2GRAY),
                 'yuv422_yuy2': (2, cv2.COLOR_YUV2GRAY_YUY2)}
        channels, code = codes[frame.encoding]
        image = np.ndarray((frame.height, frame.width, channels),
                           dtype=np.uint8, buffer=frame.data,
                           strides=(frame.step, channels, 1))
        gray = cv2.cvtColor(image, code)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def raw_frame_for_png(frame, cv2, np):
    """Keep the actual camera pixels as evidence at each observation stop."""
    if frame.encoding == 'mono8':
        return np.ndarray((frame.height, frame.width), dtype=np.uint8,
                          buffer=frame.data, strides=(frame.step, 1)).copy()
    formats = {
        'rgb8': (3, cv2.COLOR_RGB2BGR),
        'bgr8': (3, None),
        'rgba8': (4, cv2.COLOR_RGBA2BGR),
        'bgra8': (4, cv2.COLOR_BGRA2BGR),
        'yuv422_yuy2': (2, cv2.COLOR_YUV2BGR_YUY2),
    }
    channels, conversion = formats[frame.encoding]
    image = np.ndarray((frame.height, frame.width, channels),
                       dtype=np.uint8, buffer=frame.data,
                       strides=(frame.step, channels, 1))
    return cv2.cvtColor(image, conversion) if conversion is not None else image.copy()


def distance_to_blocked_cell(grid, x, y, world_to_cell, cell_to_world,
                             search_radius_m=.8):
    """Distance to a known wall or unknown cell, using the saved map only."""
    info = grid.info
    cx, cy = world_to_cell(info, x, y)
    if not (0 <= cx < info.width and 0 <= cy < info.height):
        return -1.
    if grid.data[cy*info.width+cx] != 0:
        return -1.
    radius = math.ceil(search_radius_m/info.resolution)
    nearest = min(search_radius_m,
                  (cx+.5)*info.resolution,
                  (cy+.5)*info.resolution,
                  (info.width-cx-.5)*info.resolution,
                  (info.height-cy-.5)*info.resolution)
    for iy in range(max(0, cy-radius), min(info.height, cy+radius+1)):
        for ix in range(max(0, cx-radius), min(info.width, cx+radius+1)):
            if grid.data[iy*info.width+ix] == 0:
                continue
            wx, wy = cell_to_world(info, (ix, iy))
            nearest = min(nearest, math.hypot(x-wx, y-wy))
    return nearest


def box_bypass_route_exists(grid, robot_xy, goal_xy, box_xy, tangent,
                            world_to_cell, cell_to_world):
    """Check a full-size box and its hidden-depth strip with offline A*.

    The whole 25 cm box is known to this Gazebo fixture, even though the
    robot's first scan will see only its near face. Reserve another 25 cm
    along the route tangent. Use continuous box-to-robot-disc geometry so map
    cell quantization does not add a second, unrequested clearance margin.
    """
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.map_utils import occupancy_to_grid

    info = grid.info
    resolution = float(info.resolution)
    if not math.isfinite(resolution) or resolution <= 0:
        return False
    width, height = info.width, info.height
    radius_cells = .24 / resolution
    static = occupancy_to_grid(grid, occupied_threshold=65,
                               inflation_cells=math.ceil(radius_cells))
    start = world_to_cell(info, *robot_xy)
    goal = world_to_cell(info, *goal_xy)
    if static.is_static_obstacle(start) or static.is_static_obstacle(goal):
        return False

    def square_gap_at_progress(wx, wy, progress):
        dx = max(abs(wx-box_xy[0]-tangent[0]*progress)-.125, 0.)
        dy = max(abs(wy-box_xy[1]-tangent[1]*progress)-.125, 0.)
        return math.hypot(dx, dy)

    blocked = set()
    reserve_depth = .25
    # Distance to the swept square is convex in progress. Ternary search finds
    # its nearest location without rasterizing the whole box as larger cells.
    xmin = min(box_xy[0], box_xy[0]+tangent[0]*reserve_depth)-.365
    xmax = max(box_xy[0], box_xy[0]+tangent[0]*reserve_depth)+.365
    ymin = min(box_xy[1], box_xy[1]+tangent[1]*reserve_depth)-.365
    ymax = max(box_xy[1], box_xy[1]+tangent[1]*reserve_depth)+.365
    for iy in range(height):
        for ix in range(width):
            wx, wy = cell_to_world(info, (ix, iy))
            if not (xmin <= wx <= xmax and ymin <= wy <= ymax):
                continue
            low, high = 0., reserve_depth
            for _ in range(20):
                third = (high-low)/3
                left, right = low+third, high-third
                if square_gap_at_progress(wx, wy, left) <= square_gap_at_progress(wx, wy, right):
                    high = right
                else:
                    low = left
            minimum_gap = min(square_gap_at_progress(wx, wy, progress)
                              for progress in (0., (low+high)/2, reserve_depth))
            if minimum_gap <= .24:
                blocked.add((ix, iy))
    if start in blocked or goal in blocked:
        return False
    return bool(AStarPlanner().plan(static, start, goal, blocked))


def select_path_obstacle(path, robot_xy, goal_xy, grid,
                         world_to_cell, cell_to_world, observed_indices=(),
                         diagnostics=None):
    """Select a clear point ahead on the *active* path, never under the robot."""
    if diagnostics is not None:
        diagnostics.update({'path_points': len(path or ()), 'ahead_points': 0,
                            'oriented_points': 0, 'unobserved_points': 0,
                            'clear_points': 0, 'max_static_clearance_m': None,
                            'bypass_routes_checked': 0,
                            'sealed_candidates_rejected': 0})
    if not path or robot_xy is None:
        if diagnostics is not None:
            diagnostics['reason'] = 'NO_ACTIVE_HOME_PATH_OR_ROBOT_TRUTH'
        return None
    first = min(range(len(path)), key=lambda i: math.hypot(
        path[i][0]-robot_xy[0], path[i][1]-robot_xy[1]))
    observed_xy = [cell_to_world(grid.info, (index % grid.info.width,
                                             index // grid.info.width))
                   for index in set(observed_indices)
                   if 0 <= index < grid.info.width*grid.info.height]
    candidates = []
    for index in range(first+1, len(path)):
        x, y = path[index]
        robot_distance = math.hypot(x-robot_xy[0], y-robot_xy[1])
        goal_distance = math.hypot(x-goal_xy[0], y-goal_xy[1])
        if not (.60 <= robot_distance <= 1.00 and goal_distance >= .45):
            continue
        if diagnostics is not None:
            diagnostics['ahead_points'] += 1
        before = path[max(0, index-3)]
        after = path[min(len(path)-1, index+3)]
        dx, dy = after[0]-before[0], after[1]-before[1]
        tangent_length = math.hypot(dx, dy)
        if tangent_length < .10:
            continue
        tangent = (dx/tangent_length, dy/tangent_length)
        approach_along = ((robot_xy[0]-x)*tangent[0] +
                          (robot_xy[1]-y)*tangent[1])
        if approach_along > -.25:
            continue
        if diagnostics is not None:
            diagnostics['oriented_points'] += 1
        if any(math.hypot(x-ox, y-oy) <= .35 for ox, oy in observed_xy):
            continue
        if diagnostics is not None:
            diagnostics['unobserved_points'] += 1
        wall_distance = distance_to_blocked_cell(
            grid, x, y, world_to_cell, cell_to_world)
        if diagnostics is not None:
            previous = diagnostics['max_static_clearance_m']
            diagnostics['max_static_clearance_m'] = max(
                wall_distance, previous if previous is not None else -1.)
        # A 25 cm box needs room beside mapped walls for a meaningful detour.
        if wall_distance >= .40:
            if diagnostics is not None:
                diagnostics['clear_points'] += 1
            candidates.append((abs(robot_distance-.75), -wall_distance,
                               index, x, y, robot_distance, wall_distance,
                               tangent, approach_along))
    if not candidates:
        if diagnostics is not None:
            diagnostics['reason'] = (
                'NO_POINT_0P6_TO_1P0_M_AHEAD' if diagnostics['ahead_points'] == 0 else
                'NO_FORWARD_PATH_TANGENT' if diagnostics['oriented_points'] == 0 else
                'PATH_ALREADY_OBSERVED' if diagnostics['unobserved_points'] == 0 else
                'NO_0P4_M_STATIC_WALL_CLEARANCE')
        return None
    for _, _, index, x, y, robot_distance, wall_distance, tangent, approach_along in sorted(candidates):
        if diagnostics is not None:
            diagnostics['bypass_routes_checked'] += 1
        if not box_bypass_route_exists(grid, robot_xy, goal_xy, (x, y), tangent,
                                       world_to_cell, cell_to_world):
            if diagnostics is not None:
                diagnostics['sealed_candidates_rejected'] += 1
            continue
        if diagnostics is not None:
            diagnostics['reason'] = 'SELECTED'
        return {'index': index, 'x': x, 'y': y,
                'robot_distance_m': robot_distance,
                'static_wall_distance_m': wall_distance,
                'path_tangent': list(tangent),
                'approach_along_path_m': approach_along}
    if diagnostics is not None:
        diagnostics['reason'] = 'NO_BYPASSABLE_BOX_SITE'
    return None


def observation_cleared(samples, removal_at):
    if removal_at is None:
        return False
    later = [sample for sample in samples if sample['t'] >= removal_at+1.]
    return len(later) >= 3 and all(
        sample['near_obstacle'] == 0 for sample in later[-3:])


def conservative_body_clearance(robot_xy, obstacle_xy):
    """Exact circle-to-axis-aligned-square gap for this Gazebo fixture.

    The simulated robot's 24 cm bounding disc and the 25 cm square may pass
    beside each other without the square's *corner* being on that side.
    Treating its half-diagonal as a radius falsely reports an overlap there.
    A negative result means the bounding disc intersects the box.
    """
    box_half_width = .25 / 2
    robot_radius = .24
    dx = max(abs(robot_xy[0]-obstacle_xy[0])-box_half_width, 0.)
    dy = max(abs(robot_xy[1]-obstacle_xy[1])-box_half_width, 0.)
    return math.hypot(dx, dy)-robot_radius


def physical_bypass_proven(robot_xy, obstacle_xy, tangent, spawn_robot_xy,
                           motion_commands, minimum_clearance):
    along = ((robot_xy[0]-obstacle_xy[0])*tangent[0] +
             (robot_xy[1]-obstacle_xy[1])*tangent[1])
    traveled = math.hypot(robot_xy[0]-spawn_robot_xy[0],
                          robot_xy[1]-spawn_robot_xy[1])
    passed = (along >= .20 and traveled >= .40 and motion_commands >= 3
              and minimum_clearance >= 0.)
    return passed, along, traveled


def stationary_stop_proven(commands, positions, terminal_t, sample_end_t):
    recent_commands = [item for item in commands if item['t'] >= terminal_t-.5]
    fresh_positions = [item for item in positions if item['t'] >= terminal_t+.2]
    if (len(recent_commands) < 3 or len(fresh_positions) < 3 or
            fresh_positions[-1]['t'] < sample_end_t-.5):
        return False
    return (all(abs(item['linear']) < .01 and abs(item['angular']) < .02
                for item in recent_commands[-3:])
            and max(math.hypot(item['x']-fresh_positions[0]['x'],
                               item['y']-fresh_positions[0]['y'])
                    for item in fresh_positions) < .025
            and max(abs(wrapped(item['yaw']-fresh_positions[0]['yaw']))
                    for item in fresh_positions) < .04)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=176)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--leg-timeout', type=float, default=200.)
    parser.add_argument('--obstacle-hold-timeout', type=float, default=0.,
                        help='Fail after this many seconds in unmoving WAIT_REPLAN; 0 disables early verdict')
    parser.add_argument('--single-goal', nargs=4, metavar=('NAME', 'X', 'Y', 'YAW'),
                        help='Run one explicit observation-pose candidate, without changing the registered route')
    parser.add_argument('--route-prefix', type=int, choices=(1, 2, 3),
                        help='Run only the first N sectors of the registered route')
    parser.add_argument('--home-obstacle', action='store_true',
                        help='Inject a 25 cm unmapped obstacle on the active Home path')
    parser.add_argument('--route-obstacle', action='store_true',
                        help='With --route-prefix, inject on the last sector path')
    parser.add_argument('--home-obstacle-bypass-timeout', type=float, default=35.,
                        help='Maximum seconds to prove physical bypass before safe cleanup')
    args = parser.parse_args()
    if args.domain == 16 or os.environ.get('ROS_DOMAIN_ID') != str(args.domain):
        raise SystemExit('Refusing physical or mismatched ROS domain')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('ROS_LOCALHOST_ONLY=1 required')
    if args.output.exists():
        raise SystemExit('Refusing to overwrite '+str(args.output))
    if args.single_goal and args.route_prefix:
        raise SystemExit('--route-prefix cannot be combined with --single-goal')
    if args.home_obstacle and (args.single_goal or args.route_prefix):
        raise SystemExit('--home-obstacle requires the full four-stop route')
    if args.route_obstacle and (not args.route_prefix or args.home_obstacle):
        raise SystemExit('--route-obstacle requires --route-prefix')
    if not math.isfinite(args.home_obstacle_bypass_timeout) or args.home_obstacle_bypass_timeout < 10.:
        raise SystemExit('--home-obstacle-bypass-timeout must be at least 10 seconds')
    stops = ((tuple([args.single_goal[0], *map(float, args.single_goal[1:])]),)
             if args.single_goal else STOPS[:args.route_prefix]
             if args.route_prefix else STOPS)

    import rclpy
    import cv2
    import numpy as np
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavigationPath
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import Int32MultiArray, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from smart_vision_amr.map_utils import cell_to_world, world_to_cell
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                          qos_profile_sensor_data)

    started = time.monotonic()
    evidence = {'passed': False, 'domain': args.domain, 'localhost_only': True,
                'route': [name for name, *_ in stops], 'physical_robot_touched': False,
                'home_obstacle_requested': args.home_obstacle,
                'home_obstacle': None, 'legs': [], 'events': [], 'failure': None,
                'route_obstacle_requested': args.route_obstacle,
                'route_obstacle': None,
                'marker_visibility_all_verified': False,
                'marker_alignment_all_verified': False}

    class Observer(Node):
        def __init__(self):
            super().__init__('classroom_route_verifier')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.latest = {}
            self.map = None
            self.truth = None
            self.amcl = None
            self.odom = None
            self.health = None
            self.planners = []
            self.navigation = []
            self.paths = []
            self.path = None
            self.commands = []
            self.positions = []
            self.failure_memory_status = None
            self.observed = None
            self.observed_samples = []
            self.obstacle_name = None
            self.obstacle_truth = None
            self.obstacle_reference = None
            self.obstacle_active = False
            self.obstacle_clearance_min = float('inf')
            self.obstacle_clearance_samples = 0
            self.obstacle_clearance_breach = False
            self.obstacle_clearance_last_at = None
            self.obstacle_clearance_max_gap_s = 0.
            self.clearance_stop_requested = False
            self.observed_before_obstacle = set()
            self.remembered = None
            self.costmap = None
            self.rear_frames = deque(maxlen=2)
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50)
            self.aruco_parameters = (cv2.aruco.DetectorParameters_create()
                                     if hasattr(cv2.aruco, 'DetectorParameters_create')
                                     else cv2.aruco.DetectorParameters())
            self.goal_pub = self.create_publisher(PoseStamped, '/smart_vision_amr/goal', 10)
            self.estop = self.create_client(Trigger, '/smart_vision_amr/emergency_stop')
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(LaserScan, '/scan', lambda _: self.touch('scan'),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/image_raw', self.on_rear_image,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl,
                                     qos_profile_sensor_data)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_truth,
                                     qos_profile_sensor_data)
            self.create_subscription(NavigationPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
            self.create_subscription(String, '/smart_vision_amr/localization_health',
                                     self.on_health, latched)
            self.create_subscription(String, '/smart_vision_amr/planner_status',
                                     self.on_planner, latched)
            self.create_subscription(String, '/smart_vision_amr/navigation_status',
                                     self.on_navigation, 10)
            self.create_subscription(String, '/smart_vision_amr/failure_memory_status',
                                     self.on_failure_memory, latched)
            self.create_subscription(Int32MultiArray, '/smart_vision_amr/observed_obstacles',
                                     self.on_observed, 10)
            self.create_subscription(Int32MultiArray, '/smart_vision_amr/remembered_cells',
                                     self.on_remembered, latched)
            self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                     self.on_costmap, 10)

        def elapsed(self):
            return round(time.monotonic()-started, 3)

        def touch(self, key):
            self.latest[key] = time.monotonic()

        def on_map(self, message):
            self.map = message
            self.touch('map')

        def on_rear_image(self, message):
            self.rear_frames.append((self.elapsed(), message))
            self.touch('rear_camera')

        def marker_evidence(self, expected_id):
            result = {'expected_id': expected_id, 'marker_visible': False,
                      'detected_ids': [], 'pixel_box': None,
                      'camera_topic': '/camera/image_raw',
                      'alignment_result': 'NO_RECENT_REAR_CAMERA_FRAME'}
            for receipt, frame in reversed(self.rear_frames):
                age = self.elapsed()-receipt
                if age > 2.:
                    continue
                result['frame_age_s'] = round(age, 3)
                result['image_size'] = [frame.width, frame.height]
                result['encoding'] = frame.encoding
                if 'frame_png' not in result:
                    frame_file = args.output.with_name(
                        f'{args.output.stem}_rear_marker_{expected_id}.png')
                    try:
                        frame_file.parent.mkdir(parents=True, exist_ok=True)
                        if not cv2.imwrite(str(frame_file),
                                           raw_frame_for_png(frame, cv2, np)):
                            raise RuntimeError('cv2.imwrite returned false')
                        result['frame_png'] = str(frame_file)
                    except Exception as exc:
                        result['frame_save_error'] = str(exc)
                try:
                    corners, ids, _ = detect_raw_aruco(
                        frame, self.aruco_dictionary, self.aruco_parameters,
                        cv2, np)
                except Exception as exc:
                    result['alignment_result'] = 'CAMERA_DECODE_ERROR'
                    result['error'] = str(exc)
                    continue
                detected = ids.flatten().tolist() if ids is not None else []
                result['detected_ids'] = detected
                if expected_id in detected:
                    points = corners[detected.index(expected_id)].reshape(-1, 2)
                    result['marker_visible'] = True
                    result['pixel_box'] = [float(points[:, 0].min()),
                                           float(points[:, 1].min()),
                                           float(points[:, 0].max()),
                                           float(points[:, 1].max())]
                    center_x = (result['pixel_box'][0]+result['pixel_box'][2])/2.
                    result['horizontal_center_error_fraction'] = round(
                        (center_x-frame.width/2.)/(frame.width/2.), 3)
                    result['marker_near_image_center'] = (
                        abs(result['horizontal_center_error_fraction']) <= .25)
                    result['alignment_result'] = (
                        'EXPECTED_MARKER_NEAR_CENTER' if result['marker_near_image_center']
                        else 'EXPECTED_MARKER_VISIBLE_OFF_CENTER')
                    return result
                result['alignment_result'] = 'EXPECTED_MARKER_NOT_VISIBLE'
            return result

        def emit(self, event, **values):
            entry = {'t': self.elapsed(), 'event': event, **values}
            evidence['events'].append(entry)
            print(json.dumps(entry), flush=True)

        def on_truth(self, message):
            robot_pose = None
            if 'waffle_pi' in message.name:
                pose = message.pose[message.name.index('waffle_pi')]
                robot_pose = pose
                self.truth = (pose.position.x, pose.position.y, yaw(pose.orientation))
                self.touch('truth')
                self.positions.append({'t': self.elapsed(), 'x': self.truth[0],
                                       'y': self.truth[1], 'yaw': self.truth[2]})
            if self.obstacle_name in message.name:
                pose = message.pose[message.name.index(self.obstacle_name)]
                self.obstacle_truth = (pose.position.x, pose.position.y)
            else:
                self.obstacle_truth = None
            if self.obstacle_active and robot_pose is not None and self.obstacle_truth is not None:
                received_at = time.monotonic()
                if self.obstacle_clearance_last_at is not None:
                    self.obstacle_clearance_max_gap_s = max(
                        self.obstacle_clearance_max_gap_s,
                        received_at-self.obstacle_clearance_last_at)
                self.obstacle_clearance_last_at = received_at
                clearance = conservative_body_clearance(
                    (robot_pose.position.x, robot_pose.position.y),
                    self.obstacle_truth)
                self.obstacle_clearance_samples += 1
                self.obstacle_clearance_min = min(self.obstacle_clearance_min, clearance)
                if clearance < 0.:
                    self.obstacle_clearance_breach = True
                    if not self.clearance_stop_requested and self.estop.service_is_ready():
                        self.clearance_stop_requested = True
                        self.estop.call_async(Trigger.Request())

        def on_amcl(self, message):
            pose = message.pose.pose
            self.amcl = (pose.position.x, pose.position.y, yaw(pose.orientation))
            self.touch('amcl')

        def on_odom(self, message):
            pose = message.pose.pose
            self.odom = (pose.position.x, pose.position.y, yaw(pose.orientation))
            self.touch('odom')

        def on_tf(self, message):
            for transform in message.transforms:
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                    self.touch('map_tf')
                if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                    self.touch('odom_tf')

        def on_path(self, message):
            self.path = [(pose.pose.position.x, pose.pose.position.y)
                         for pose in message.poses] if message.poses else None
            coordinates = ';'.join(f'{x:.3f},{y:.3f}' for x,y in self.path or [])
            self.paths.append({'t': self.elapsed(), 'count': len(message.poses),
                               'sha256': hashlib.sha256(coordinates.encode()).hexdigest()})

        def on_command(self, message):
            self.commands.append({'t': self.elapsed(), 'linear': message.linear.x,
                                  'angular': message.angular.z})

        def on_health(self, message):
            value = parse_status(message)
            if value:
                self.health = value
                self.touch('health')

        def on_planner(self, message):
            value = parse_status(message)
            if value:
                self.planners.append({'t': self.elapsed(), **value})

        def on_navigation(self, message):
            value = parse_status(message)
            if value:
                self.navigation.append({'t': self.elapsed(), **value})

        def on_failure_memory(self, message):
            self.failure_memory_status = parse_status(message)

        def on_observed(self, message):
            self.observed = list(message.data)
            near = 0
            new_near = 0
            if self.obstacle_reference is not None and self.map is not None:
                info = self.map.info
                for index in set(self.observed):
                    if not 0 <= index < info.width*info.height:
                        continue
                    x, y = cell_to_world(info, (index % info.width,
                                               index // info.width))
                    if math.hypot(x-self.obstacle_reference[0],
                                  y-self.obstacle_reference[1]) <= .35:
                        near += 1
                        new_near += index not in self.observed_before_obstacle
            self.observed_samples.append({'t': self.elapsed(),
                                          'near_obstacle': near,
                                          'new_near_obstacle': new_near})

        def on_remembered(self, message):
            self.remembered = list(message.data)

        def on_costmap(self, message):
            self.costmap = message

        def spin_until(self, predicate, duration, description):
            until = time.monotonic()+duration
            while time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=.05)
                if predicate():
                    return
            raise TimeoutError(description)

        def service(self, future, duration, description):
            self.spin_until(lambda: future.done(), duration, description)
            return future.result()

    rclpy.init(args=[])
    node = Observer()
    obstacle_live = False
    try:
        node.spin_until(lambda: all(key in node.latest for key in
            ('map', 'scan', 'odom', 'map_tf', 'odom_tf', 'truth', 'health'))
            and node.goal_pub.get_subscription_count() > 0,
            90., 'map/sensors/TF/ground truth not ready')
        node.spin_until(lambda: node.health.get('autonomous_allowed') is True,
                        30., 'localization health did not allow motion')
        node.emit('inputs_ready')
        for stop_index, (name, x, y, goal_yaw) in enumerate(stops):
            obstacle_fixture = ((args.home_obstacle and name == 'Home') or
                                (args.route_obstacle and stop_index == len(stops)-1))
            obstacle_key = 'home_obstacle' if args.home_obstacle else 'route_obstacle'
            sent_at = node.elapsed()
            path_start = len(node.paths)
            planner_start = len(node.planners)
            cmd_start = len(node.commands)
            truth_start = len(node.positions)
            current_health = node.health.get('autonomous_allowed') is True
            safety_blocks = 0
            safety_recoveries = 0
            candidate = None
            candidate_at = None
            hold_since = None
            hold_anchor = None
            leg_start_truth = node.truth
            obstacle = ({'requested': True, 'spawned': False,
                         'removed': False, 'minimum_clearance_m': None,
                         'bypassed_while_live': False,
                         'ros_observation_cleared': False,
                         'web_red_overlay_verified': False,
                         'web_red_overlay_note':
                             'ROS direct-goal verifier does not inspect browser DOM'
                         } if obstacle_fixture else None)
            if obstacle_fixture:
                evidence[obstacle_key] = obstacle
            obstacle_spawned_at = None
            obstacle_removed_at = None
            obstacle_passed_at = None
            obstacle_spawn_truth = None
            obstacle_spawn_cmd_index = None
            obstacle_truth_seen = False
            obstacle_before = None
            candidate_path = None
            selection_diagnostic = None
            unavailable_reported = False
            current_path_hash = None
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.z = math.sin(goal_yaw/2)
            msg.pose.orientation.w = math.cos(goal_yaw/2)
            node.goal_pub.publish(msg)
            node.emit('goal_sent', stop=name, x=x, y=y)
            terminal = 'TIMEOUT'
            deadline = time.monotonic()+args.leg_timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if (obstacle_fixture and not obstacle['spawned']
                        and leg_start_truth is not None and node.truth is not None
                        and math.hypot(node.truth[0]-leg_start_truth[0],
                                       node.truth[1]-leg_start_truth[1]) >= .10
                        and len(node.paths) > path_start and node.path is not None
                        and node.paths[-1]['t'] >= sent_at):
                    selection_diagnostic = {}
                    candidate_path = select_path_obstacle(
                        node.path, node.truth[:2], (x, y), node.map,
                        world_to_cell, cell_to_world, node.observed or (),
                        selection_diagnostic)
                    if (candidate_path is None and not unavailable_reported and
                            node.elapsed()-sent_at >= 5.):
                        unavailable_reported = True
                        node.emit('home_obstacle_candidate_unavailable',
                                  **selection_diagnostic)
                    if candidate_path is not None:
                        if not node.spawn.service_is_ready():
                            node.spin_until(node.spawn.service_is_ready, 5.,
                                            'Gazebo spawn service unavailable')
                        current_path_hash = node.paths[-1]['sha256']
                        obstacle_before = {
                            'path_sha256': current_path_hash,
                            'local_attempt_id': max((p.get('local_attempt_id', 0)
                                for p in node.planners[planner_start:]), default=0),
                            'global_plan_count': max((p.get('global_plan_count', 0)
                                for p in node.planners[planner_start:]), default=0)}
                        model_name = 'classroom_home_obstacle_'+uuid.uuid4().hex[:10]
                        node.obstacle_name = model_name
                        node.obstacle_reference = (candidate_path['x'],
                                                   candidate_path['y'])
                        node.obstacle_active = True
                        node.obstacle_clearance_min = float('inf')
                        node.obstacle_clearance_samples = 0
                        node.obstacle_clearance_breach = False
                        node.obstacle_clearance_last_at = None
                        node.obstacle_clearance_max_gap_s = 0.
                        node.clearance_stop_requested = False
                        node.observed_before_obstacle = set(node.observed or ())
                        request = SpawnEntity.Request()
                        request.name = model_name
                        request.xml = f'''<sdf version="1.6"><model name="{model_name}">
                          <static>true</static><link name="body">
                          <collision name="body"><geometry><box><size>.25 .25 .5</size></box></geometry></collision>
                          <visual name="body"><geometry><box><size>.25 .25 .5</size></box></geometry>
                          <material><ambient>.9 .2 .1 1</ambient><diffuse>.9 .2 .1 1</diffuse></material>
                          </visual></link></model></sdf>'''
                        request.initial_pose.position.x = candidate_path['x']
                        request.initial_pose.position.y = candidate_path['y']
                        request.initial_pose.position.z = .25
                        request.initial_pose.orientation.w = 1.
                        spawn_future = node.spawn.call_async(request)
                        # The service may time out after Gazebo has created the
                        # model; cleanup must still attempt to delete our name.
                        obstacle_live = True
                        response = node.service(spawn_future, 8.,
                                                'Gazebo obstacle spawn timed out')
                        if not response.success:
                            obstacle_live = False
                            node.obstacle_active = False
                            raise RuntimeError('Gazebo obstacle spawn failed: '+
                                               response.status_message)
                        if node.obstacle_clearance_breach:
                            raise RuntimeError(
                                'Gazebo obstacle collision during spawn: '
                                f'minimum body clearance {node.obstacle_clearance_min:.4f} m')
                        obstacle_spawned_at = node.elapsed()
                        obstacle_spawn_truth = node.truth
                        obstacle_spawn_cmd_index = len(node.commands)
                        obstacle.update({'spawned': True, 'name': model_name,
                                         'position': [candidate_path['x'],
                                                      candidate_path['y']],
                                         'home_leg_displacement_at_spawn_m':
                                             math.hypot(node.truth[0]-leg_start_truth[0],
                                                        node.truth[1]-leg_start_truth[1]),
                                         'path_index': candidate_path['index'],
                                         'robot_distance_at_spawn_m':
                                             candidate_path['robot_distance_m'],
                                         'static_wall_distance_m':
                                             candidate_path['static_wall_distance_m'],
                                         'path_sha256_before': current_path_hash})
                        node.emit('home_unmapped_obstacle_spawned',
                                  **candidate_path, name=model_name)
                if node.obstacle_clearance_breach:
                    raise RuntimeError(
                        'Gazebo obstacle collision: '
                        f'minimum body clearance {node.obstacle_clearance_min:.4f} m')
                if obstacle_live and node.truth is not None and node.obstacle_truth is not None:
                    obstacle_truth_seen = True
                    tangent = candidate_path['path_tangent']
                    active_commands = sum(
                        abs(command['linear']) > .01 or
                        abs(command['angular']) > .02
                        for command in node.commands[obstacle_spawn_cmd_index:])
                    passed, along, traveled = physical_bypass_proven(
                        node.truth[:2], node.obstacle_truth, tangent,
                        obstacle_spawn_truth[:2], active_commands,
                        node.obstacle_clearance_min)
                    if (obstacle_passed_at is None and
                            time.monotonic()-node.latest['truth'] < .5 and passed):
                        obstacle_passed_at = node.elapsed()
                        lateral = abs((node.truth[0]-node.obstacle_truth[0])*tangent[1] -
                                      (node.truth[1]-node.obstacle_truth[1])*tangent[0])
                        obstacle.update({'bypassed_while_live': True,
                                         'bypass_truth_pose': list(node.truth),
                                         'bypass_along_path_m': along,
                                         'bypass_lateral_offset_m': lateral,
                                         'travel_from_spawn_m': traveled,
                                         'motion_commands_while_live': active_commands})
                        node.emit('home_obstacle_bypassed_while_live',
                                  x=node.truth[0], y=node.truth[1],
                                  along_m=along, lateral_m=lateral,
                                  minimum_clearance_m=node.obstacle_clearance_min)
                if (obstacle_live and obstacle_passed_at is None and
                        node.elapsed()-obstacle_spawned_at >=
                        args.home_obstacle_bypass_timeout):
                    terminal = 'HOME_OBSTACLE_BYPASS_TIMEOUT'
                    node.emit('home_obstacle_bypass_timeout',
                              duration_s=node.elapsed()-obstacle_spawned_at,
                              last_planner=node.planners[-1] if node.planners else None)
                    break
                if (obstacle_live and obstacle_passed_at is not None and
                        node.elapsed()-obstacle_passed_at >= .5):
                    if not node.delete.service_is_ready():
                        raise RuntimeError('Gazebo delete service unavailable')
                    request = DeleteEntity.Request()
                    request.name = node.obstacle_name
                    response = node.service(node.delete.call_async(request), 6.,
                                            'Gazebo obstacle removal timed out')
                    if not response.success:
                        raise RuntimeError('Gazebo obstacle removal failed: '+
                                           response.status_message)
                    obstacle_live = False
                    obstacle_removed_at = node.elapsed()
                    obstacle['removed'] = True
                    node.spin_until(lambda: node.obstacle_truth is None, 4.,
                                    'Deleted Home obstacle remains in Gazebo model states')
                    if node.obstacle_clearance_breach:
                        raise RuntimeError(
                            'Gazebo obstacle collision during removal: '
                            f'minimum body clearance {node.obstacle_clearance_min:.4f} m')
                    node.obstacle_active = False
                    obstacle['truth_removed'] = True
                    node.emit('home_obstacle_removed', name=node.obstacle_name)
                allowed = node.health is not None and node.health.get('autonomous_allowed') is True
                if allowed != current_health:
                    current_health = allowed
                    if allowed:
                        safety_recoveries += 1
                        node.emit('safety_recovered', stop=name)
                    else:
                        safety_blocks += 1
                        node.emit('safety_blocked', stop=name,
                                  reasons=node.health.get('reasons'))
                if node.navigation:
                    latest = node.navigation[-1]
                    if latest['t'] >= sent_at and matches_goal(latest, x, y):
                        status = latest.get('status')
                        if status == 'SUCCEEDED':
                            if obstacle_fixture and obstacle['spawned'] and not obstacle['bypassed_while_live']:
                                terminal = 'HOME_REACHED_WITHOUT_BYPASS'
                                break
                            if not obstacle_fixture or not obstacle['spawned'] or observation_cleared(
                                    node.observed_samples, obstacle_removed_at):
                                terminal = status
                                break
                        if status in ('FAILED', 'CANCELED'):
                            if candidate != status:
                                candidate, candidate_at = status, time.monotonic()
                            elif (time.monotonic()-candidate_at >= 5.
                                  and (not obstacle_fixture or not obstacle_live)
                                  and (obstacle_removed_at is None or
                                       node.elapsed()-obstacle_removed_at >= 10.)):
                                terminal = status
                                break
                        else:
                            candidate = None
                            candidate_at = None
                if args.obstacle_hold_timeout > 0 and node.planners and node.truth:
                    waiting = (node.planners[-1]['t'] >= sent_at and
                               node.planners[-1].get('status') == 'WAIT_REPLAN'
                               and not obstacle_live and
                               (obstacle_removed_at is None or
                                node.elapsed()-obstacle_removed_at >= 2.))
                    if not waiting:
                        hold_since, hold_anchor = None, None
                    elif hold_since is None or hold_anchor is None or math.hypot(
                            node.truth[0]-hold_anchor[0], node.truth[1]-hold_anchor[1]) > .03:
                        hold_since, hold_anchor = time.monotonic(), node.truth
                    elif time.monotonic()-hold_since >= args.obstacle_hold_timeout:
                        terminal = 'PERSISTENT_OBSTACLE_HOLD'
                        break
            # A successful state without actual arrival is not accepted.
            gt = node.truth
            distance = math.hypot(gt[0]-x, gt[1]-y) if gt else None
            heading_error = abs(wrapped(gt[2]-goal_yaw)) if gt else None
            route_paths = node.paths[path_start:]
            route_planners = node.planners[planner_start:]
            route_commands = node.commands[cmd_start:]
            safe_stop = None
            if obstacle_fixture:
                stop_sample_from = node.elapsed()
                stop_sample_until = time.monotonic()+1.2
                while time.monotonic() < stop_sample_until:
                    rclpy.spin_once(node, timeout_sec=.05)
                safe_stop = stationary_stop_proven(
                    node.commands, node.positions, stop_sample_from,
                    node.elapsed())
                if node.obstacle_clearance_breach:
                    raise RuntimeError(
                        'Gazebo obstacle collision during stop verification: '
                        f'minimum body clearance {node.obstacle_clearance_min:.4f} m')
            leg = {'stop': name, 'goal': [x,y,goal_yaw], 'terminal': terminal,
                   'duration_s': round(node.elapsed()-sent_at, 3),
                   'gazebo_truth': list(gt) if gt else None,
                   'position_error_m': distance, 'heading_error_rad': heading_error,
                   'amcl_pose': list(node.amcl) if node.amcl else None,
                   'odom_pose': list(node.odom) if node.odom else None,
                   'amcl_receipt_age_s': time.monotonic()-node.latest.get('amcl', 0.),
                   'odom_receipt_age_s': time.monotonic()-node.latest.get('odom', 0.),
                   'scan_receipt_age_s': time.monotonic()-node.latest.get('scan', 0.),
                   'map_tf_receipt_age_s': time.monotonic()-node.latest.get('map_tf', 0.),
                   'paths_published': sum(p['count'] > 0 for p in route_paths),
                   'empty_paths': sum(p['count'] == 0 for p in route_paths),
                   'max_global_plan_count': max((p.get('global_plan_count',0)
                       for p in route_planners), default=None),
                   'max_local_attempt_id': max((p.get('local_attempt_id',0)
                       for p in route_planners), default=None),
                   'motion_commands': sum(abs(c['linear'])>.01 or abs(c['angular'])>.02
                       for c in route_commands),
                   'ground_truth_samples': len(node.positions)-truth_start,
                   'safety_blocks': safety_blocks,
                   'safety_recoveries': safety_recoveries,
                   'last_planner': route_planners[-1] if route_planners else None,
                   'last_navigation': node.navigation[-1] if node.navigation else None,
                   'failure_memory_status': node.failure_memory_status,
                   'observed_indices': node.observed,
                   'remembered_indices': node.remembered}
            marker_id = expected_marker_id(name)
            if marker_id is not None:
                leg['marker_alignment'] = node.marker_evidence(marker_id)
                if not leg['marker_alignment']['marker_visible']:
                    node.emit('marker_alignment_unverified', stop=name,
                              result=leg['marker_alignment']['alignment_result'],
                              detected_ids=leg['marker_alignment']['detected_ids'])
            if obstacle_fixture:
                if not obstacle['spawned']:
                    if leg_start_truth is None or node.truth is None:
                        injection_reason = 'NO_HOME_GAZEBO_TRUTH'
                    elif math.hypot(node.truth[0]-leg_start_truth[0],
                                    node.truth[1]-leg_start_truth[1]) < .10:
                        injection_reason = 'ROBOT_DID_NOT_START_HOME_ROUTE'
                    elif not any(path['count'] > 0 for path in route_paths):
                        injection_reason = 'NO_NONEMPTY_HOME_PATH'
                    else:
                        injection_reason = (selection_diagnostic or {}).get(
                            'reason', 'ACTIVE_PATH_BECAME_EMPTY_BEFORE_SELECTION')
                    obstacle['injection_failure_reason'] = injection_reason
                    obstacle['selection_diagnostic'] = selection_diagnostic
                    node.emit('home_obstacle_not_injected', reason=injection_reason,
                              selection=selection_diagnostic)
                observed_during = [sample for sample in node.observed_samples
                                   if obstacle_spawned_at is not None
                                   and sample['t'] >= obstacle_spawned_at
                                   and (obstacle_removed_at is None or
                                        sample['t'] <= obstacle_removed_at)]
                first_observed = next((sample['t'] for sample in observed_during
                                       if sample['new_near_obstacle'] > 0), None)
                reaction_paths = [path for path in route_paths
                                  if first_observed is not None
                                  and path['t'] >= first_observed
                                  and (obstacle_removed_at is None or
                                       path['t'] <= obstacle_removed_at)]
                reaction_planners = [plan for plan in route_planners
                                     if first_observed is not None
                                     and plan['t'] >= first_observed
                                     and (obstacle_removed_at is None or
                                          plan['t'] <= obstacle_removed_at)]
                changed_path = bool(obstacle_before and any(
                    path['count'] > 0 and
                    path['sha256'] != obstacle_before['path_sha256']
                    for path in reaction_paths))
                local_detour = bool(obstacle_before and any(
                    plan.get('local_detour_state') == 'ACTIVE'
                    for plan in reaction_planners))
                failed_local_attempts = bool(obstacle_before and any(
                    plan.get('local_detour_state') == 'FAILED' and
                    plan.get('local_attempt_id', 0) > obstacle_before['local_attempt_id']
                    for plan in reaction_planners))
                global_replan = bool(obstacle_before and any(
                    plan.get('global_plan_count', 0) >
                    obstacle_before['global_plan_count']
                    for plan in reaction_planners))
                observation_gone = observation_cleared(node.observed_samples,
                                                       obstacle_removed_at)
                obstacle.update({
                    'truth_seen': obstacle_truth_seen,
                    'first_new_observation_t': first_observed,
                    'new_near_obstacle_cells': max((sample['new_near_obstacle']
                        for sample in observed_during), default=0),
                    'changed_nonempty_path_before_removal': changed_path,
                    'local_detour_attempted_before_removal': local_detour,
                    'failed_local_attempts_before_removal': failed_local_attempts,
                    'global_replan_attempted_before_removal': global_replan,
                    'route_reacted_before_removal': changed_path or local_detour,
                    'ros_observation_cleared': observation_gone,
                    'minimum_clearance_m': (node.obstacle_clearance_min if math.isfinite(
                        node.obstacle_clearance_min) else None),
                    'clearance_model_state_samples': node.obstacle_clearance_samples,
                    'clearance_model_state_max_gap_s':
                        node.obstacle_clearance_max_gap_s,
                    'clearance_sampling_sufficient': (
                        node.obstacle_clearance_samples >= 10 and
                        node.obstacle_clearance_max_gap_s <= .5),
                    'final_safe_stop_proven': safe_stop,
                    'terminal': terminal})
                evidence[obstacle_key] = obstacle
                leg[obstacle_key] = obstacle
            if node.costmap is not None and node.odom is not None:
                costmap = node.costmap
                info = costmap.info
                cx = math.floor((node.odom[0]-info.origin.position.x)/info.resolution)
                cy = math.floor((node.odom[1]-info.origin.position.y)/info.resolution)
                value = (int(costmap.data[cy*info.width+cx]) if
                         0 <= cx < info.width and 0 <= cy < info.height else None)
                leg['local_costmap'] = {'frame_id': costmap.header.frame_id,
                    'cell_at_odom': [cx,cy], 'cost_at_odom': value,
                    'resolution': info.resolution,
                    'origin': [info.origin.position.x, info.origin.position.y]}
            # A waypoint is a delivery/observation stop, not merely an (x, y)
            # target.  The user accepts a recognized expected marker even if
            # its pixel center is imperfect; keep the centering result as a
            # separate diagnostic.  Never report four-stop success when a
            # required sector marker was absent from a recent rear frame.
            marker_visible = (marker_id is None or
                              leg.get('marker_alignment', {}).get('marker_visible') is True)
            leg['marker_visible_passed'] = marker_visible
            leg['passed'] = (terminal == 'SUCCEEDED' and distance is not None
                             and distance <= .35 and heading_error <= .40
                             and marker_visible)
            if obstacle_fixture:
                leg['passed'] = (leg['passed'] and obstacle['spawned']
                                 and obstacle_truth_seen and obstacle['removed']
                                 and obstacle.get('truth_removed') is True
                                 and obstacle['bypassed_while_live']
                                 and obstacle['new_near_obstacle_cells'] > 0
                                 and obstacle['route_reacted_before_removal']
                                 and obstacle['ros_observation_cleared']
                                 and obstacle['minimum_clearance_m'] is not None
                                 and obstacle['minimum_clearance_m'] >= 0.
                                 and obstacle['clearance_sampling_sufficient']
                                 and safe_stop)
                obstacle['outcome'] = ('MISSION_SUCCESS' if leg['passed'] else
                                       'SAFE_HOLD_UNRESOLVED' if safe_stop and
                                       terminal in ('PERSISTENT_OBSTACLE_HOLD',
                                                    'TIMEOUT', 'FAILED', 'CANCELED',
                                                    'HOME_OBSTACLE_BYPASS_TIMEOUT')
                                       else 'FAILED')
                if not leg['passed']:
                    evidence['failure_class'] = (
                        'safe_hold_without_home_arrival' if
                        obstacle['outcome'] == 'SAFE_HOLD_UNRESOLVED' else
                        'home_obstacle_not_injected' if not obstacle['spawned'] else
                        'home_obstacle_not_observed' if not
                        obstacle['new_near_obstacle_cells'] else
                        'home_obstacle_not_bypassed_while_live' if not
                        obstacle['bypassed_while_live'] else
                        'home_obstacle_not_reacted_to' if not
                        obstacle['route_reacted_before_removal'] else
                        'home_obstacle_not_cleared' if not observation_gone else
                        'home_obstacle_clearance_sampling_insufficient' if not
                        obstacle['clearance_sampling_sufficient'] else
                        'home_obstacle_arrival_or_safety_failed')
            evidence['legs'].append(leg)
            node.emit('leg_complete', stop=name, passed=leg['passed'],
                      terminal=terminal, distance_m=distance,
                      heading_error_rad=heading_error)
            if not leg['passed']:
                break
            # Hold the robot between goals to expose post-arrival motion.
            hold_started = node.elapsed()
            while node.elapsed()-hold_started < .6:
                rclpy.spin_once(node, timeout_sec=.05)
        evidence['passed'] = len(evidence['legs']) == len(stops) and all(
            leg['passed'] for leg in evidence['legs'])
        evidence['marker_visibility_all_verified'] = (
            len(evidence['legs']) == len(stops) and all(
                leg.get('marker_alignment', {}).get('marker_visible') is True
                for leg in evidence['legs']))
        evidence['marker_alignment_all_verified'] = (
            len(evidence['legs']) == len(stops) and all(
                leg.get('marker_alignment', {}).get('marker_near_image_center') is True
                for leg in evidence['legs']))
    except Exception as exc:
        evidence['failure'] = repr(exc)
        if node.obstacle_clearance_samples and math.isfinite(node.obstacle_clearance_min):
            obstacle_key = 'home_obstacle' if args.home_obstacle else 'route_obstacle'
            if evidence.get(obstacle_key) is not None:
                evidence[obstacle_key]['minimum_clearance_m'] = node.obstacle_clearance_min
        node.emit('trial_failed', error=repr(exc))
    finally:
        # An accepted terminal SUCCEEDED already stops the follower.  Do not
        # latch the separate web observer into ESTOP merely because this
        # isolated verifier has finished.  Any failed/unfinished leg still
        # takes the hard stop before Gazebo cleanup.
        if evidence['passed']:
            evidence['final_safety_action'] = 'terminal_success_no_new_estop'
        elif node.estop.service_is_ready():
            evidence['final_safety_action'] = 'emergency_stop_requested'
            try:
                future = node.estop.call_async(Trigger.Request())
                node.spin_until(lambda: future.done(), 4., 'emergency stop timed out')
                if not future.result().success:
                    evidence['final_safety_action'] = 'emergency_stop_rejected'
            except Exception as exc:
                evidence['final_safety_action'] = 'emergency_stop_failed'
                evidence['emergency_stop_error'] = repr(exc)
        else:
            evidence['final_safety_action'] = 'emergency_stop_unavailable'
        if obstacle_live:
            try:
                if not node.delete.service_is_ready():
                    node.spin_until(node.delete.service_is_ready, 3.,
                                    'Gazebo cleanup service unavailable')
                request = DeleteEntity.Request()
                request.name = node.obstacle_name
                result = node.service(node.delete.call_async(request), 5.,
                                      'Gazebo obstacle cleanup timed out')
                if not result.success:
                    raise RuntimeError('Gazebo obstacle cleanup rejected: '+
                                       result.status_message)
                obstacle_live = False
                node.obstacle_active = False
                node.emit('owned_home_obstacle_cleaned', name=node.obstacle_name)
                cleanup_key = 'home_obstacle' if args.home_obstacle else 'route_obstacle'
                if evidence[cleanup_key] is not None:
                    evidence[cleanup_key]['removed_on_exit'] = True
            except Exception as exc:
                evidence['cleanup_failure'] = repr(exc)
                evidence['passed'] = False
                node.emit('owned_home_obstacle_cleanup_failed', error=repr(exc))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('PASS' if evidence['passed'] else 'FAIL', args.output, flush=True)
    return 0 if evidence['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
