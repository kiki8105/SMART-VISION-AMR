#!/usr/bin/env python3
"""Gazebo-only fixture: remember a Sector 1 obstacle, remove it during Sector 2.

The web server owns the 1→2→3→Home mission. This actor only creates and
deletes its own Gazebo model and observes ROS. It never publishes a goal or
velocity command, resets localization, or clears memory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

from inject_classroom_web_obstacle import (
    conservative_body_clearance,
    current_observed_bypass_route,
    path_clearance_from_box,
    preflight_box_route,
)


SECTOR1 = (1.2, .575)
SECTOR2 = (1.525, -2.65)
# This point is visible early on the Home→Sector 1 leg but stays well clear of
# that leg. The mapped partition occludes it at Sector 1; the unmodified
# Sector 1→2 route crosses it, so the next plan must use remembered cells.
# This center leaves a bypass when a LiDAR return projects one map cell beyond
# the box raster and keeps the memory-aware Sector 2 path clear of the box.
BOX = (.650, -.575)


def matching_goal(point, xy, tolerance=.05):
    return math.hypot(point[0] - xy[0], point[1] - xy[1]) <= tolerance


def matching_path(poses, xy, tolerance=.35):
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in poses]
    return points if points and matching_goal(points[-1], xy, tolerance) else []


def path_hash(points):
    return hashlib.sha256(';'.join(f'{x:.3f},{y:.3f}' for x, y in points).encode()).hexdigest()


def nearby_indices(indexes, info, center, cell_to_world, radius=.26):
    return {index for index in indexes
            if 0 <= index < info.width * info.height
            and math.dist(cell_to_world(info, (index % info.width,
                                               index // info.width)), center) <= radius}


def removal_outcome(before_hash, after_hash, before_count, after_count):
    """Describe observed behavior, without assuming a removal caused a replan."""
    if before_hash is None or after_hash is None:
        return 'UNDETERMINED'
    if after_count > before_count or after_hash != before_hash:
        return 'NEW_PATH_OR_REPLAN'
    return 'CONTINUED_EXISTING_PATH'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--wait-timeout', type=float, default=700.)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '176' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('This Gazebo fixture requires ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1')
    if args.output.exists():
        parser.error('Refusing to overwrite '+str(args.output))
    if not math.isfinite(args.wait_timeout) or args.wait_timeout < 30.:
        parser.error('wait-timeout must be at least 30 seconds')

    import rclpy
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from smart_vision_amr.map_utils import cell_to_world, world_to_cell
    from std_msgs.msg import Int32MultiArray, String

    started = time.monotonic()
    running = True
    evidence = {
        'passed': False, 'domain': 176, 'localhost_only': True,
        'physical_robot_touched': False,
        'goal_owner': 'web_mission_server', 'goal_published_by_injector': False,
        'candidate_xy': list(BOX), 'spawned': False, 'truth_seen': False,
        'observed_on_sector1': False, 'confirmed_on_sector1': False,
        'sector2_planned_with_memory': False,
        'sector2_initial_plan_without_live_hit': False,
        'removed_on_sector2': False,
        'free_ray_evidence_after_removal': False, 'memory_cleared': False,
        'minimum_clearance_m': None, 'events': [], 'failure': None,
    }

    def emit(kind, **fields):
        row = {'event': kind, 'elapsed_s': round(time.monotonic()-started, 3),
               'wall_time': time.time(), **fields}
        evidence['events'].append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        return row

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(args=[])

    class Actor(Node):
        def __init__(self):
            super().__init__('classroom_memory_transition_actor')
            self.grid = None
            self.current_goal = None
            self.goal_at = None
            self.path = []
            self.path_at = None
            self.path_goal = None
            self.robot = None
            self.robot_at = None
            self.box_truth = None
            self.observed = set()
            self.observed_at = None
            self.remembered = set()
            self.remembered_at = None
            self.owned_cells = set()
            self.owned_at_delete = set()
            self.before_memory = set()
            self.free_rays = {}
            self.planner = {}
            self.planner_timeline = []
            self.path_timeline = []
            self.model_name = 'classroom_memory_box_'+uuid.uuid4().hex[:10]
            self.model_created = False
            self.spawn_requested = False
            self.model_deleted = False
            self.removed_at = None
            self.sector2_path_before_delete = None
            self.sector2_plan_before_delete = None
            self.sector2_path_at_clear = None
            self.sector2_plan_at_clear = None
            self.clear_at = None
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(PoseStamped, '/smart_vision_amr/goal', self.on_goal, 10)
            self.create_subscription(NavigationPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_truth,
                                     qos_profile_sensor_data)
            self.create_subscription(Int32MultiArray,
                                     '/smart_vision_amr/observed_obstacles',
                                     self.on_observed, 10)
            self.create_subscription(Int32MultiArray,
                                     '/smart_vision_amr/remembered_cells',
                                     self.on_memory, latched)
            self.create_subscription(OccupancyGrid,
                                     '/smart_vision_amr/observed_visibility',
                                     self.on_visibility, qos_profile_sensor_data)
            self.create_subscription(String, '/smart_vision_amr/planner_status',
                                     self.on_planner, latched)
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')

        def near(self, indexes):
            if self.grid is None:
                return set()
            return nearby_indices(indexes, self.grid.info, BOX, cell_to_world)

        def on_map(self, msg):
            self.grid = msg

        def on_goal(self, msg):
            xy = (msg.pose.position.x, msg.pose.position.y)
            match = ('sector1' if matching_goal(xy, SECTOR1)
                     else 'sector2' if matching_goal(xy, SECTOR2) else 'other')
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if (match, stamp) != self.current_goal:
                self.current_goal = (match, stamp)
                self.goal_at = time.monotonic()
                self.path = []
                self.path_at = None
                self.path_goal = None
                emit('goal', marker=match, xy=list(xy), stamp=stamp,
                     remembered_near_box=len(self.near(self.remembered)),
                     observed_near_box=len(self.near(self.observed)))

        def on_path(self, msg):
            marker = self.current_goal[0] if self.current_goal else None
            target = SECTOR1 if marker == 'sector1' else SECTOR2 if marker == 'sector2' else None
            path = matching_path(msg.poses, target) if target else []
            if not path:
                return
            self.path = path
            self.path_at = time.monotonic()
            self.path_goal = marker
            digest = path_hash(path)
            if not self.path_timeline or (self.path_timeline[-1]['hash'],
                                          self.path_timeline[-1]['marker']) != (digest, marker):
                item = emit('path', marker=marker, hash=digest, count=len(path),
                            remembered_near_box=len(self.near(self.remembered)),
                            clearance_from_box_m=round(path_clearance_from_box(path, BOX), 4),
                            global_plan_count=self.planner.get('global_plan_count'))
                self.path_timeline.append(item)

        def on_truth(self, msg):
            if 'waffle_pi' in msg.name:
                p = msg.pose[msg.name.index('waffle_pi')].position
                self.robot = (p.x, p.y)
                self.robot_at = time.monotonic()
            if self.model_name in msg.name:
                p = msg.pose[msg.name.index(self.model_name)].position
                self.box_truth = (p.x, p.y)
                evidence['truth_seen'] = True
            else:
                self.box_truth = None
            if self.model_created and not self.model_deleted and self.robot and self.box_truth:
                clearance = conservative_body_clearance(self.robot, self.box_truth)
                old = evidence['minimum_clearance_m']
                evidence['minimum_clearance_m'] = round(min(
                    clearance, old if old is not None else math.inf), 4)

        def on_observed(self, msg):
            self.observed = set(msg.data)
            self.observed_at = time.monotonic()
            if self.model_created and not self.model_deleted and self.current_goal and (
                    self.current_goal[0] == 'sector1') and self.near(self.observed):
                evidence['observed_on_sector1'] = True

        def on_memory(self, msg):
            self.remembered = set(msg.data)
            self.remembered_at = time.monotonic()
            if self.model_created and not self.model_deleted:
                fresh = self.near(self.remembered - self.before_memory)
                if fresh:
                    new = fresh-self.owned_cells
                    self.owned_cells |= fresh
                    if new:
                        emit('box_memory_confirmed', new_cells=sorted(new),
                             all_owned_cells=sorted(self.owned_cells))
                    if self.current_goal and self.current_goal[0] == 'sector1':
                        evidence['confirmed_on_sector1'] = True
            if (self.model_deleted and self.owned_at_delete and
                    not (self.owned_at_delete & self.remembered) and
                    not evidence['memory_cleared']):
                evidence['memory_cleared'] = True
                self.clear_at = time.monotonic()
                self.sector2_path_at_clear = path_hash(self.path) if self.path_goal == 'sector2' else None
                self.sector2_plan_at_clear = self.planner.get('global_plan_count', 0)
                emit('box_memory_cleared', owned_cells=sorted(self.owned_at_delete),
                     free_ray_counts={str(i): len(self.free_rays.get(i, []))
                                      for i in self.owned_at_delete},
                     sector2_path_hash=self.sector2_path_at_clear,
                     global_plan_count=self.sector2_plan_at_clear)

        def on_visibility(self, msg):
            if not self.model_deleted:
                return
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            for index in self.owned_at_delete:
                if 0 <= index < len(msg.data) and msg.data[index] == 0:
                    self.free_rays.setdefault(index, set()).add(stamp)
            evidence['free_ray_evidence_after_removal'] = any(
                self.free_rays.get(index) for index in self.owned_at_delete)

        def on_planner(self, msg):
            try:
                packet = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if not isinstance(packet, dict):
                return
            self.planner = packet
            count = packet.get('global_plan_count', 0)
            if not self.planner_timeline or self.planner_timeline[-1]['global_plan_count'] != count:
                self.planner_timeline.append(emit(
                    'planner', marker=self.current_goal[0] if self.current_goal else None,
                    global_plan_count=count, status=packet.get('status'),
                    message=packet.get('message'), plan_kind=packet.get('plan_kind')))

        def await_service(self, future, timeout, operation):
            deadline = time.monotonic()+timeout
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
            if not future.done():
                raise TimeoutError(operation+' timed out')
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError(operation+' failed: '+repr(result))
            return result

    node = Actor()
    try:
        deadline = time.monotonic()+args.wait_timeout
        last_wait_log = 0.
        while running and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if (evidence['minimum_clearance_m'] is not None and
                    evidence['minimum_clearance_m'] < .02):
                raise RuntimeError('Robot-obstacle conservative clearance below 2 cm')
            if not node.model_created:
                ready = (node.grid is not None and node.current_goal is not None and
                         node.current_goal[0] == 'sector1' and node.path_goal == 'sector1' and
                         node.path_at is not None and now-node.path_at <= 4. and
                         node.robot_at is not None and now-node.robot_at <= .4 and
                         node.observed_at is not None and now-node.observed_at <= 2. and
                         node.spawn.service_is_ready() and node.delete.service_is_ready())
                if not ready:
                    if now-last_wait_log >= 5.:
                        emit('waiting_for_sector1_route', map_ready=node.grid is not None,
                             current_goal=node.current_goal[0] if node.current_goal else None,
                             path_ready=node.path_goal == 'sector1', truth_ready=node.robot is not None,
                             spawn_service_ready=node.spawn.service_is_ready())
                        last_wait_log = now
                    continue
                robot_distance = math.dist(node.robot, BOX)
                if robot_distance < .50 or robot_distance > .95:
                    continue
                if node.near(node.observed) or node.near(node.remembered):
                    continue
                from smart_vision_amr.map_utils import occupancy_to_grid
                baseline = occupancy_to_grid(node.grid, 65, 5)
                sector2_static = [cell_to_world(node.grid.info, cell)
                                  for cell in __import__(
                                      'smart_vision_amr.astar', fromlist=['AStarPlanner']).AStarPlanner().plan(
                                      baseline, world_to_cell(node.grid.info, *SECTOR1),
                                      world_to_cell(node.grid.info, *SECTOR2))]
                first = current_observed_bypass_route(
                    node.grid, node.robot, SECTOR1, BOX, node.observed,
                    world_to_cell, cell_to_world,
                    remembered_indices=node.remembered)
                second = preflight_box_route(
                    node.grid, SECTOR1, SECTOR2, BOX, sector2_static,
                    world_to_cell, cell_to_world)
                first_clearance = path_clearance_from_box(
                    [cell_to_world(node.grid.info, cell) for cell in first], BOX
                ) if first else -math.inf
                if (not first or not second or
                        path_clearance_from_box(node.path, BOX) < .10 or
                        first_clearance < .10):
                    raise RuntimeError('Fixed memory box lacks a clear Sector 1 route or a verified Sector 2 bypass')
                # Drain Gazebo truth once more before the entity mutation.
                for _ in range(3):
                    rclpy.spin_once(node, timeout_sec=.02)
                if (node.current_goal[0] != 'sector1' or node.robot_at is None or
                        time.monotonic()-node.robot_at > .4 or
                        math.dist(node.robot, BOX) < .50 or node.near(node.observed)):
                    continue
                node.before_memory = set(node.remembered)
                request = SpawnEntity.Request()
                request.name = node.model_name
                request.xml = f'''<sdf version="1.6"><model name="{node.model_name}">
                  <static>true</static><link name="body">
                  <collision name="body"><geometry><box><size>.25 .25 .5</size></box></geometry></collision>
                  <visual name="body"><geometry><box><size>.25 .25 .5</size></box></geometry>
                  <material><ambient>.9 .2 .1 1</ambient><diffuse>.9 .2 .1 1</diffuse></material>
                  </visual></link></model></sdf>'''
                request.initial_pose.position.x = BOX[0]
                request.initial_pose.position.y = BOX[1]
                request.initial_pose.position.z = .25
                request.initial_pose.orientation.w = 1.
                node.spawn_requested = True
                node.await_service(node.spawn.call_async(request), 8., 'spawn')
                node.model_created = evidence['spawned'] = True
                evidence['obstacle_name'] = node.model_name
                evidence['sector1_safe_route_cells'] = len(first)
                evidence['sector1_safe_route_clearance_m'] = round(first_clearance, 4)
                evidence['sector2_static_bypass_cells'] = second['route_cells_by_step']
                emit('box_spawned_off_sector1_route', xy=list(BOX), robot_xy=list(node.robot),
                     sector1_route_cells=len(first),
                     sector1_route_clearance_m=round(first_clearance, 4),
                     sector2_route_cells=second['route_cells_by_step'])
                continue

            if (node.model_created and not node.model_deleted and
                    evidence['confirmed_on_sector1'] and node.current_goal and
                    node.current_goal[0] == 'sector2' and node.path_goal == 'sector2' and
                    node.path_at is not None and node.goal_at is not None and
                    node.path_at >= node.goal_at and node.owned_cells & node.remembered and
                    not evidence['sector2_planned_with_memory']):
                evidence['sector2_planned_with_memory'] = True
                evidence['sector2_first_path_hash'] = path_hash(node.path)
                evidence['sector2_first_path_clearance_m'] = round(
                    path_clearance_from_box(node.path, BOX), 4)
                evidence['sector2_first_global_plan_count'] = node.planner.get('global_plan_count', 0)
                evidence['sector2_initial_plan_without_live_hit'] = not node.near(node.observed)
                emit('sector2_first_path_with_memory',
                     path_hash=evidence['sector2_first_path_hash'],
                     clearance_m=evidence['sector2_first_path_clearance_m'],
                     remembered_cells=len(node.owned_cells & node.remembered),
                     observed_cells=len(node.near(node.observed)),
                     global_plan_count=evidence['sector2_first_global_plan_count'])

            if (node.model_created and not node.model_deleted and
                    evidence['sector2_planned_with_memory'] and node.robot_at is not None and
                    now-node.robot_at <= .4 and node.path_goal == 'sector2'):
                distance = math.dist(node.robot, BOX)
                departed = math.dist(node.robot, SECTOR1)
                if (.55 <= distance <= .9 and departed >= .12 and
                        node.owned_cells & node.remembered):
                    node.sector2_path_before_delete = path_hash(node.path)
                    node.sector2_plan_before_delete = node.planner.get('global_plan_count', 0)
                    node.owned_at_delete = node.owned_cells & node.remembered
                    node.await_service(node.delete.call_async(
                        DeleteEntity.Request(name=node.model_name)), 6., 'delete')
                    node.model_deleted = evidence['removed_on_sector2'] = True
                    node.removed_at = time.monotonic()
                    evidence['sector2_path_hash_at_delete'] = node.sector2_path_before_delete
                    evidence['sector2_global_plan_count_at_delete'] = node.sector2_plan_before_delete
                    emit('box_removed_on_sector2', robot_xy=list(node.robot),
                         distance_from_box_m=round(distance, 3),
                         sector2_path_hash=node.sector2_path_before_delete,
                         global_plan_count=node.sector2_plan_before_delete,
                         owned_cells=sorted(node.owned_at_delete))

            if evidence['memory_cleared'] and node.clear_at is not None and (
                    now-node.clear_at >= 5.):
                break
        if running and time.monotonic() >= deadline:
            raise TimeoutError('Timed out waiting for Sector 1→2 memory removal evidence')
        evidence['sector2_final_path_hash'] = (
            path_hash(node.path) if node.path_goal == 'sector2' else None)
        evidence['sector2_final_global_plan_count'] = node.planner.get('global_plan_count', 0)
        evidence['after_removal'] = removal_outcome(
            node.sector2_path_before_delete,
            evidence['sector2_final_path_hash'],
            node.sector2_plan_before_delete or 0,
            evidence['sector2_final_global_plan_count'])
        evidence['owned_confirmed_cells'] = sorted(node.owned_cells)
        evidence['owned_cells_at_delete'] = sorted(node.owned_at_delete)
        evidence['free_ray_counts'] = {str(i): len(node.free_rays.get(i, []))
                                       for i in node.owned_at_delete}
        evidence['passed'] = all(evidence[key] for key in (
            'spawned', 'truth_seen', 'observed_on_sector1',
            'confirmed_on_sector1', 'sector2_planned_with_memory',
            'sector2_initial_plan_without_live_hit',
            'removed_on_sector2', 'free_ray_evidence_after_removal',
            'memory_cleared')) and (evidence['minimum_clearance_m'] is not None and
                                    evidence['minimum_clearance_m'] >= .02) and (
            evidence.get('sector2_first_path_clearance_m', -math.inf) >= .02)
    except Exception as exc:
        evidence['failure'] = repr(exc)
        emit('actor_failed', reason=repr(exc))
    finally:
        if node.spawn_requested and not node.model_deleted and node.delete.service_is_ready():
            try:
                node.await_service(node.delete.call_async(
                    DeleteEntity.Request(name=node.model_name)), 5., 'cleanup delete')
                node.model_deleted = True
                emit('owned_box_cleaned_after_failure', name=node.model_name)
            except Exception as exc:
                evidence['cleanup_error'] = repr(exc)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2,
                                          allow_nan=False)+'\n', encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('PASS' if evidence['passed'] else 'FAIL', args.output, flush=True)
    return 0 if evidence['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
