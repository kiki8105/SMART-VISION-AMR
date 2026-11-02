#!/usr/bin/env python3
"""Inject one owned moving Gazebo box onto a *web-owned* classroom route.

This process only observes ROS and mutates its own Gazebo entity. In particular,
it never publishes a navigation goal, a velocity, a reset, or an emergency stop.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

from verify_classroom_route import (conservative_body_clearance,
                                    distance_to_blocked_cell,
                                    select_path_obstacle)


def hash_path(path):
    coordinates = ';'.join(f'{x:.3f},{y:.3f}' for x, y in path)
    return hashlib.sha256(coordinates.encode()).hexdigest()


def path_clearance_from_box(path, center):
    """Conservative centerline clearance for the robot and 25 cm square box."""
    if not path or center is None:
        return -math.inf
    nearest = math.inf
    for start, end in zip(path, path[1:]):
        dx, dy = end[0]-start[0], end[1]-start[1]
        squared = dx*dx+dy*dy
        fraction = 0. if squared == 0 else max(0., min(1.,
            ((center[0]-start[0])*dx+(center[1]-start[1])*dy)/squared))
        nearest = min(nearest, math.hypot(
            start[0]+fraction*dx-center[0],
            start[1]+fraction*dy-center[1]))
    if len(path) == 1:
        nearest = math.hypot(path[0][0]-center[0], path[0][1]-center[1])
    return nearest-(.24+.25*math.sqrt(2)/2)


def candidate_still_ahead(candidate, robot_xy, current_path):
    """Reject a box that the robot reached while the topology search ran."""
    if candidate is None or robot_xy is None or not current_path:
        return False
    center = (candidate['x'], candidate['y'])
    tangent = candidate['path_tangent']
    along = sum((robot_xy[index]-center[index])*tangent[index]
                for index in (0, 1))
    distance = math.hypot(robot_xy[0]-center[0], robot_xy[1]-center[1])
    return (along <= -.25 and .60 <= distance <= 1.00 and
            path_clearance_from_box(current_path, center) <= .05)


def near_obstacle_indices(indices, before, grid, center, cell_to_world):
    if grid is None or center is None:
        return []
    info = grid.info
    nearby = []
    for index in set(indices)-set(before):
        if not 0 <= index < info.width*info.height:
            continue
        x, y = cell_to_world(info, (index % info.width, index // info.width))
        if math.hypot(x-center[0], y-center[1]) <= .35:
            nearby.append(index)
    return sorted(nearby)


def preflight_box_route(grid_message, robot_xy, goal_xy, center, path,
                        world_to_cell, cell_to_world, movement_side=None):
    """Prove a full-size box leaves an inflated A* route at every moved pose.

    This is an injection-fixture gate. It uses the same static inflation and
    circular live-obstacle footprint as the classroom planner, never a smaller
    clearance just to make a test pass.
    """
    from math import sqrt
    from smart_vision_amr.astar import AStarPlanner
    from smart_vision_amr.map_utils import (inflate_cells_circular,
                                            occupancy_to_grid)

    info = grid_message.info
    width, height = info.width, info.height
    static = occupancy_to_grid(grid_message, 65, 5)
    raw = occupancy_to_grid(grid_message, 65, 0)
    start = world_to_cell(info, *robot_xy)
    goal = world_to_cell(info, *goal_xy)
    if not static.contains(start) or not static.contains(goal):
        return None
    planner = AStarPlanner()
    if not planner.plan(static, start, goal):
        return None
    if path_clearance_from_box(path, center) > .05:
        return None
    tangent = None
    if movement_side is not None:
        tangent, side = movement_side
        perpendicular = (-tangent[1], tangent[0])
    else:
        side = 0.
        perpendicular = (0., 0.)
    routes = []
    for step in range(5):
        position = (center[0]+.025*step*side*perpendicular[0],
                    center[1]+.025*step*side*perpendicular[1])
        cx, cy = world_to_cell(info, *position)
        reach = math.ceil((.125 + info.resolution/2)/info.resolution)
        box = set()
        for y in range(cy-reach, cy+reach+1):
            for x in range(cx-reach, cx+reach+1):
                if not raw.contains((x, y)):
                    return None
                wx, wy = cell_to_world(info, (x, y))
                if (abs(wx-position[0]) <= .125+info.resolution/2 and
                        abs(wy-position[1]) <= .125+info.resolution/2):
                    if raw.is_static_obstacle((x, y)):
                        return None
                    box.add((x, y))
        if not box:
            return None
        radius_cells = (.22+.02)/info.resolution+sqrt(2)/2
        blocked = inflate_cells_circular(box, width, height, radius_cells)
        route = planner.plan(static, start, goal, blocked)
        if not route:
            return None
        routes.append(len(route))
    return {'route_cells_by_step': routes,
            'movement_side': side,
            'initial_centerline_clearance_m': round(path_clearance_from_box(path, center), 4)}


def select_feasible_web_obstacle(path, robot_xy, goal_xy, grid,
                                 world_to_cell, cell_to_world, observed_indices=(),
                                 diagnostics=None):
    """Select an unexpected box that blocks the path but preserves an A* bypass."""
    if not path or robot_xy is None or grid is None:
        return None
    observed_xy = [cell_to_world(grid.info, (index % grid.info.width,
                                             index // grid.info.width))
                   for index in set(observed_indices)
                   if 0 <= index < grid.info.width*grid.info.height]
    first = min(range(len(path)), key=lambda i: math.hypot(
        path[i][0]-robot_xy[0], path[i][1]-robot_xy[1]))
    proposals = []
    for index in range(first+1, len(path)):
        base = path[index]
        base_distance = math.hypot(base[0]-robot_xy[0], base[1]-robot_xy[1])
        if not .35 <= base_distance <= 1.00:
            continue
        before, after = path[max(0, index-3)], path[min(len(path)-1, index+3)]
        dx, dy = after[0]-before[0], after[1]-before[1]
        length = math.hypot(dx, dy)
        if length < .10:
            continue
        tangent = (dx/length, dy/length)
        perpendicular = (-tangent[1], tangent[0])
        for offset in (-.50, -.45, -.40, -.35, -.30, -.25, -.20,
                        .20, .25, .30, .35, .40, .45, .50):
            center = (base[0]+offset*perpendicular[0],
                      base[1]+offset*perpendicular[1])
            distance = math.hypot(center[0]-robot_xy[0],
                                  center[1]-robot_xy[1])
            goal_distance = math.hypot(center[0]-goal_xy[0],
                                       center[1]-goal_xy[1])
            approach = ((robot_xy[0]-center[0])*tangent[0] +
                        (robot_xy[1]-center[1])*tangent[1])
            if not (.60 <= distance <= 1.00 and goal_distance >= .45 and
                    approach <= -.25):
                continue
            if any(math.hypot(center[0]-ox, center[1]-oy) <= .35
                   for ox, oy in observed_xy):
                continue
            # Grid cell rounding can block the old A* route even with a few
            # centimeters of continuous geometric clearance.
            old_route_clearance = path_clearance_from_box(path, center)
            if old_route_clearance > .05:
                continue
            wall_distance = distance_to_blocked_cell(
                grid, *center, world_to_cell, cell_to_world)
            if wall_distance < .16:
                continue
            proposals.append(((-wall_distance, abs(distance-.75),
                               abs(old_route_clearance)),
                              index, base, center, offset, distance,
                              wall_distance, tangent, approach))
    proposals.sort()
    if diagnostics is not None:
        diagnostics['geometric_candidates'] = len(proposals)
    for tried, (_, index, base, center, offset, distance,
                wall_distance, tangent, approach) in enumerate(proposals[:60], 1):
        perpendicular = (-tangent[1], tangent[0])
        sides = sorted((-1., 1.), key=lambda side: -distance_to_blocked_cell(
            grid, center[0]+.10*side*perpendicular[0],
            center[1]+.10*side*perpendicular[1],
            world_to_cell, cell_to_world))
        for movement_side in sides:
            result = preflight_box_route(grid, robot_xy, goal_xy,
                center, path, world_to_cell, cell_to_world,
                (tangent, movement_side))
            if result is None:
                continue
            candidate = {'index': index, 'x': center[0], 'y': center[1],
                'robot_distance_m': distance,
                'static_wall_distance_m': wall_distance,
                'path_tangent': list(tangent),
                'approach_along_path_m': approach,
                'base_path_xy': list(base), 'lateral_offset_m': offset,
                'preflight': result}
            if diagnostics is not None:
                diagnostics.update({'reason': 'FEASIBLE_BYPASS',
                                    'topology_candidates_tried': tried})
            return candidate
    if diagnostics is not None:
        diagnostics.update({'reason': 'NO_TOPOLOGICALLY_FEASIBLE_BOX',
                            'topology_candidates_tried': min(len(proposals), 60)})
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--wait-timeout', type=float, default=800.)
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID') != '176' or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        parser.error('Only isolated domain 176 with localhost DDS is permitted')
    if args.output.exists():
        parser.error('Refusing to overwrite '+str(args.output))
    if not all(math.isfinite(value) for value in (args.goal_x, args.goal_y)):
        parser.error('Goal must be finite')

    import rclpy
    from gazebo_msgs.msg import EntityState, ModelStates
    from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
    from std_msgs.msg import Int32MultiArray, String
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                          qos_profile_sensor_data)
    from smart_vision_amr.map_utils import cell_to_world, world_to_cell

    started = time.monotonic()
    running = True
    evidence = {'passed': False, 'domain': 176, 'localhost_only': True,
                'physical_robot_touched': False, 'goal': [args.goal_x, args.goal_y],
                'goal_owner': 'web_mission_server', 'goal_published_by_injector': False,
                'spawned': False, 'truth_seen': False, 'removed': False,
                'moving_steps': 0, 'observed_while_live': False,
                'observed_cleared': False, 'path_reacted_before_removal': False,
                'bypassed_while_live': False, 'minimum_clearance_m': None,
                'recovery_requests': 0, 'events': [], 'failure': None}

    def elapsed():
        return round(time.monotonic()-started, 3)

    def emit(name, **values):
        record = {'t': elapsed(), 'at': time.time(), 'event': name, **values}
        evidence['events'].append(record)
        print(json.dumps(record), flush=True)
        return record

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(args=[])

    class Observer(Node):
        def __init__(self):
            super().__init__('classroom_web_obstacle_injector')
            self.grid = None
            self.goal_seen = False
            self.goal_stamp = None
            self.path = []
            self.robot = None
            self.robot_at = None
            self.robot_at_goal_start = None
            self.obstacle = None
            self.name = None
            self.observed = set()
            self.before_observed = set()
            self.clear_observations = 0
            self.post_remove_samples = 0
            self.last_post_remove_log = 0.
            self.candidate = None
            self.tangent = None
            self.path_before = None
            self.planner_before = 0
            self.planner_local_before = 0
            self.planner_reacted = False
            self.last_status = {}
            self.removed_at = None
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
            self.create_subscription(PoseStamped, '/smart_vision_amr/goal', self.on_goal, 10)
            self.create_subscription(NavigationPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_truth,
                                     qos_profile_sensor_data)
            self.create_subscription(Int32MultiArray,
                                     '/smart_vision_amr/observed_obstacles',
                                     self.on_observed, 10)
            self.create_subscription(String, '/smart_vision_amr/planner_status',
                                     self.on_planner, durable)
            self.create_subscription(String, '/smart_vision_amr/recovery_request',
                                     self.on_recovery, 10)
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.move = self.create_client(SetEntityState, '/gazebo/set_entity_state')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')

        def on_map(self, message):
            self.grid = message

        def on_goal(self, message):
            point = message.pose.position
            matching = math.hypot(point.x-args.goal_x, point.y-args.goal_y) <= .05
            if matching and not self.goal_seen:
                self.goal_seen = True
                self.goal_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
                self.robot_at_goal_start = self.robot
                emit('web_goal_observed', x=point.x, y=point.y,
                     stamp=self.goal_stamp)

        def on_path(self, message):
            if not self.goal_seen:
                return
            path = [(pose.pose.position.x, pose.pose.position.y)
                    for pose in message.poses]
            if path and math.hypot(path[-1][0]-args.goal_x,
                                   path[-1][1]-args.goal_y) <= .35:
                self.path = path
                before_box = False
                if self.robot and self.candidate and self.tangent:
                    delta = (self.robot[0]-self.candidate[0],
                             self.robot[1]-self.candidate[1])
                    before_box = sum(a*b for a, b in zip(delta, self.tangent)) <= -.15
                if (len(path) >= 2 and evidence['spawned'] and not evidence['removed']
                        and not evidence['bypassed_while_live'] and before_box
                        and self.path_before != hash_path(path)):
                    clearance = path_clearance_from_box(
                        path, self.obstacle or self.candidate)
                    evidence['changed_path_clearance_m'] = round(clearance, 4)
                    if clearance >= .02:
                        if not self.planner_reacted:
                            reaction = emit('matching_path_changed_clear_of_box',
                                 previous_sha256=self.path_before,
                                 new_sha256=hash_path(path),
                                 robot_xy=self.robot,
                                 conservative_clearance_m=round(clearance, 4))
                            evidence['path_reaction_before_box'] = True
                            evidence['path_reaction_at'] = reaction['at']
                        self.planner_reacted = True

        def on_truth(self, message):
            if 'waffle_pi' in message.name:
                pose = message.pose[message.name.index('waffle_pi')]
                self.robot = (pose.position.x, pose.position.y)
                self.robot_at = time.monotonic()
            if self.name in message.name:
                pose = message.pose[message.name.index(self.name)]
                self.obstacle = (pose.position.x, pose.position.y)
                evidence['truth_seen'] = True
            else:
                self.obstacle = None
            if evidence['spawned'] and not evidence['removed'] and self.robot and self.obstacle:
                clearance = conservative_body_clearance(self.robot, self.obstacle)
                current = evidence['minimum_clearance_m']
                evidence['minimum_clearance_m'] = min(current if current is not None else math.inf,
                                                     clearance)
                if self.tangent and self.candidate:
                    delta = (self.robot[0]-self.candidate[0],
                             self.robot[1]-self.candidate[1])
                    if (sum(a*b for a, b in zip(delta, self.tangent)) > .25 and
                            math.hypot(*delta) > .50):
                        evidence['bypassed_while_live'] = True

        def on_observed(self, message):
            self.observed = set(message.data)
            if self.candidate is None:
                return
            count = near_obstacle_indices(self.observed, self.before_observed,
                                          self.grid, self.candidate, cell_to_world)
            if evidence['spawned'] and not evidence['removed'] and count:
                if not evidence['observed_while_live']:
                    observation = emit('obstacle_seen_in_lidar_map', near_cells=len(count))
                    evidence['first_observed_at'] = observation['at']
                evidence['observed_while_live'] = True
                evidence['observed_indices_live'] = sorted(set(
                    evidence.get('observed_indices_live', [])) | set(count))
            if self.removed_at is not None and time.monotonic()-self.removed_at >= 2.:
                self.post_remove_samples += 1
                owned_remaining = len(self.observed & set(
                    evidence.get('observed_indices_live', [])))
                now = time.monotonic()
                if now-self.last_post_remove_log >= 1.:
                    emit('post_remove_observation', near_count=len(count),
                         owned_remaining=owned_remaining,
                         total_observed=len(self.observed),
                         robot_distance_m=(round(math.hypot(
                             self.robot[0]-self.candidate[0],
                             self.robot[1]-self.candidate[1]), 3)
                             if self.robot and self.candidate else None),
                         samples=self.post_remove_samples)
                    self.last_post_remove_log = now
                self.clear_observations = (
                    self.clear_observations+1 if owned_remaining == 0 else 0)
                if self.clear_observations >= 3:
                    evidence['observed_cleared'] = True
                    evidence['observed_clear_latency_s'] = round(
                        time.monotonic()-self.removed_at, 3)

        def on_planner(self, message):
            try:
                state = json.loads(message.data)
            except (ValueError, TypeError):
                return
            if not isinstance(state, dict):
                return
            self.last_status = state
            plans = state.get('global_plan_count', 0)
            local = state.get('local_attempt_id', 0)
            if evidence['spawned'] and not evidence['removed']:
                # Periodic planner counters are diagnostic only. A changed
                # matching Path is needed to prove an actual route reaction.
                evidence['planner_counters_while_live'] = {
                    'global_plan_count': plans,
                    'local_attempt_id': local,
                    'plan_kind': state.get('plan_kind'),
                }

        def on_recovery(self, message):
            evidence['recovery_requests'] += 1
            emit('recovery_request', data=message.data[:500])

        def await_future(self, future, timeout, operation):
            deadline = time.monotonic()+timeout
            while running and rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
            if not future.done():
                raise TimeoutError(operation+' timed out')
            return future.result()

    node = Observer()
    live = False
    try:
        deadline = time.monotonic()+args.wait_timeout
        diagnostics = {}
        next_candidate_check = 0.
        while running and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.grid is None or not node.goal_seen or not node.path or not node.robot:
                continue
            if (node.robot_at_goal_start is not None and math.hypot(
                    node.robot[0]-node.robot_at_goal_start[0],
                    node.robot[1]-node.robot_at_goal_start[1]) < .10):
                continue
            # The Home corridor has no feasible 25 cm box until the robot
            # reaches its wider final turn. Keep ingesting ROS callbacks
            # between expensive full-grid topology checks.
            now = time.monotonic()
            if now < next_candidate_check:
                continue
            next_candidate_check = now+1.
            candidate = select_feasible_web_obstacle(node.path, node.robot,
                (args.goal_x, args.goal_y), node.grid, world_to_cell,
                cell_to_world, node.observed, diagnostics)
            if candidate is None:
                continue
            # The full-grid search is synchronous. Gazebo can move the robot
            # substantially while the callback queue is not being serviced.
            # Drain fresh truth before spawning rather than trusting the pose
            # used at the start of the search.
            search_finished_at = time.monotonic()
            refresh_deadline = search_finished_at+.30
            while running and rclpy.ok() and time.monotonic() < refresh_deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            if (node.robot_at is None or node.robot_at < search_finished_at or
                    not candidate_still_ahead(candidate, node.robot, node.path)):
                diagnostics['stale_or_passed_candidates'] = (
                    diagnostics.get('stale_or_passed_candidates', 0)+1)
                continue
            if not node.spawn.service_is_ready() or not node.move.service_is_ready():
                continue
            node.candidate = (candidate['x'], candidate['y'])
            node.tangent = tuple(candidate['path_tangent'])
            node.before_observed = set(node.observed)
            node.path_before = hash_path(node.path)
            node.planner_before = node.last_status.get('global_plan_count', 0)
            node.planner_local_before = node.last_status.get('local_attempt_id', 0)
            evidence['candidate'] = candidate
            node.name = 'classroom_web_obstacle_'+uuid.uuid4().hex[:10]
            request = SpawnEntity.Request()
            request.name = node.name
            request.xml = f'''<sdf version="1.6"><model name="{node.name}"><static>false</static>
              <link name="body"><gravity>false</gravity><inertial><mass>3</mass>
              <inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
              <collision name="body"><geometry><box><size>.25 .25 .5</size></box></geometry></collision>
              <visual name="body"><geometry><box><size>.25 .25 .5</size></box></geometry>
              <material><ambient>.9 .2 .1 1</ambient><diffuse>.9 .2 .1 1</diffuse></material>
              </visual></link></model></sdf>'''
            request.initial_pose.position.x = node.candidate[0]
            request.initial_pose.position.y = node.candidate[1]
            request.initial_pose.position.z = .25
            request.initial_pose.orientation.w = 1.
            response = node.await_future(node.spawn.call_async(request), 8., 'spawn')
            if response is None or not response.success:
                raise RuntimeError('Gazebo rejected obstacle spawn: '+str(response))
            evidence['spawned'] = live = True
            evidence['obstacle_name'] = node.name
            emit('obstacle_spawned', x=node.candidate[0], y=node.candidate[1],
                 path_sha256=node.path_before, robot_xy=node.robot)
            break
        if not live and running:
            evidence['candidate_diagnostics'] = diagnostics
            raise TimeoutError('No safe point on the matching web-owned route')

        spawned_at = time.monotonic()
        next_move_at = None
        side = evidence['candidate']['preflight']['movement_side']
        perpendicular = (-node.tangent[1], node.tangent[0])
        while running and rclpy.ok() and live:
            rclpy.spin_once(node, timeout_sec=.05)
            if (evidence['minimum_clearance_m'] is not None and
                    evidence['minimum_clearance_m'] < .02):
                raise RuntimeError('Robot-obstacle conservative clearance below 2 cm')
            now = time.monotonic()
            # Do not shift the obstacle away while A* is still proving a
            # changed, collision-clear route ahead of the robot. Otherwise a
            # route that becomes clear only after the box moved could appear
            # to be obstacle-aware planning in the acceptance record.
            if node.planner_reacted and next_move_at is None:
                next_move_at = now+1.
            if (next_move_at is not None and evidence['moving_steps'] < 4
                    and now >= next_move_at):
                step = evidence['moving_steps']+1
                state = EntityState()
                state.name = node.name
                state.reference_frame = 'world'
                state.pose.position.x = node.candidate[0]+.025*step*side*perpendicular[0]
                state.pose.position.y = node.candidate[1]+.025*step*side*perpendicular[1]
                state.pose.position.z = .25
                state.pose.orientation.w = 1.
                response = node.await_future(node.move.call_async(
                    SetEntityState.Request(state=state)), 5., 'move')
                if response is None or not response.success:
                    raise RuntimeError('Gazebo rejected obstacle movement')
                evidence['moving_steps'] = step
                emit('obstacle_moved', step=step, x=state.pose.position.x,
                     y=state.pose.position.y)
                next_move_at = now+.8
            # Keep the box long enough for a slow real-speed robot to plan
            # and pass it. An early removal is allowed only after a measured
            # safe bypass and a matching clear path; the 60 s limit prevents
            # an indefinitely blocked trial.
            if (now-spawned_at >= 60. or
                    (now-spawned_at >= 14. and
                     evidence['bypassed_while_live'] and node.planner_reacted)):
                observed_at = evidence.get('first_observed_at')
                reaction_at = evidence.get('path_reaction_at')
                # DDS callbacks can arrive out of order. A changed route more
                # than one second before the LiDAR obstacle evidence is not
                # credible as an obstacle-triggered reaction.
                evidence['path_observation_temporal_link'] = bool(
                    observed_at and reaction_at and
                    observed_at-1. <= reaction_at <= time.time())
                evidence['path_reacted_before_removal'] = bool(
                    node.planner_reacted and
                    evidence['path_observation_temporal_link'])
                response = node.await_future(node.delete.call_async(
                    DeleteEntity.Request(name=node.name)), 6., 'delete')
                if response is None or not response.success:
                    raise RuntimeError('Gazebo rejected obstacle removal')
                evidence['removed'] = True
                node.removed_at = time.monotonic()
                live = False
                emit('obstacle_removed', obstacle_name=node.name)
        cleanup_deadline = time.monotonic()+40.
        while running and rclpy.ok() and time.monotonic() < cleanup_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if evidence['observed_cleared']:
                break
        evidence['passed'] = all(evidence[key] for key in (
            'spawned', 'truth_seen', 'removed', 'observed_while_live',
            'observed_cleared', 'path_reacted_before_removal',
            'bypassed_while_live')) and evidence['moving_steps'] == 4 and (
            evidence['minimum_clearance_m'] is not None and
            evidence['minimum_clearance_m'] >= .02)
    except Exception as exc:
        evidence['failure'] = repr(exc)
        emit('injector_failed', reason=repr(exc))
    finally:
        if live and node.delete.service_is_ready():
            try:
                node.await_future(node.delete.call_async(
                    DeleteEntity.Request(name=node.name)), 4., 'cleanup delete')
                evidence['removed'] = True
                emit('obstacle_removed_on_cleanup', obstacle_name=node.name)
            except Exception as exc:
                evidence['cleanup_error'] = repr(exc)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('PASS' if evidence['passed'] else 'FAIL', args.output, flush=True)
    return 0 if evidence['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
