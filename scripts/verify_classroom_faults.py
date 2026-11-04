#!/usr/bin/env python3
"""Finite, instrumented classroom Gazebo route and sensor-fault acceptance.

This observer cannot run on the physical robot domain. It never fabricates a
laser, odometry, TF, planner status, or successful navigation result. The only
world mutation is an owned collision box placed on the current Gazebo path.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


def parse_status(message):
    try:
        value = json.loads(message.data)
        return value if isinstance(value, dict) else {}
    except (ValueError, TypeError):
        return {}


def assess_sustained_outage_hold(terminal, safe_hold, health, motion, planner,
                                navigation, goal_xy, goal_sent_t, sampled_at):
    """Accept only a latched, stationary, goal-retaining sensor outage."""
    latched_times = [item['t'] for item in health
                     if item.get('stop_latched') is True
                     and item.get('autonomous_allowed') is False
                     and item['t'] >= goal_sent_t]
    planner_goal = planner.get('goal') or {}
    x = planner_goal.get('x') if isinstance(planner_goal, dict) else None
    y = planner_goal.get('y') if isinstance(planner_goal, dict) else None
    retained = (type(x) in (int, float) and type(y) in (int, float)
                and math.hypot(x-goal_xy[0], y-goal_xy[1]) < .05
                and planner.get('status') != 'IDLE'
                and bool(latched_times)
                and planner.get('t', -1.) >= latched_times[0])
    latched_health = bool(latched_times)
    latched_motion = (bool(motion) and motion[-1].get('estop_latched') is True
                      and sampled_at-motion[-1].get('t', -1.) <= 1.0
                      and bool(latched_times) and motion[-1].get('t', -1.) >= latched_times[0])
    manual_cancel = any(item.get('manual_stop') is True for item in navigation)
    metrics = {
        'outage_health_stop_latched': latched_health,
        'outage_mux_estop_latched': latched_motion,
        'outage_goal_retained': retained,
        'outage_manual_cancel_seen': manual_cancel,
        'outage_manual_reset_required': latched_motion,
    }
    passed = (terminal == 'SAFE_HOLD' and safe_hold and latched_health
              and latched_motion and retained and not manual_cancel)
    return metrics, passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=176)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=('clean', 'jitter', 'outages', 'mixed'), default='clean')
    parser.add_argument('--obstacle', choices=('none', 'moving'), default='none')
    parser.add_argument('--goal-x', type=float, default=-.25)
    parser.add_argument('--goal-y', type=float, default=-1.98)
    parser.add_argument('--goal-yaw', type=float, default=-1.40)
    parser.add_argument('--timeout', type=float, default=150.)
    parser.add_argument('--allow-safe-hold', action='store_true',
                        help='For deliberately impassable stress fixtures only')
    parser.add_argument('--expect-sustained-outage-hold', action='store_true',
                        help='For the outages profile: require a latched, stationary hold with the goal retained')
    parser.add_argument('--stress', action='store_true',
                        help='The world contains an intentionally impassable fixture')
    args = parser.parse_args()
    if args.domain == 16 or os.environ.get('ROS_DOMAIN_ID') != str(args.domain):
        raise SystemExit('Refusing physical or mismatched ROS domain')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Verifier requires ROS_LOCALHOST_ONLY=1')
    if not math.isfinite(args.timeout) or args.timeout < 10.:
        raise SystemExit('timeout must be at least 10 seconds')
    if args.output.exists():
        raise SystemExit('Refusing to overwrite '+str(args.output))
    if args.allow_safe_hold and not args.stress:
        raise SystemExit('--allow-safe-hold is restricted to the stress world')
    if args.expect_sustained_outage_hold and (args.profile != 'outages' or
            args.obstacle != 'none' or args.stress or args.allow_safe_hold):
        raise SystemExit('--expect-sustained-outage-hold requires the plain outages profile')

    import rclpy
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
    from gazebo_msgs.msg import EntityState
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavigationPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32MultiArray, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from smart_vision_amr.map_utils import cell_to_world, world_to_cell
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                          qos_profile_sensor_data)

    start = time.monotonic()
    evidence = {'passed': False, 'domain': args.domain, 'localhost_only': True,
                'profile': args.profile, 'obstacle': args.obstacle,
                'goal': [args.goal_x, args.goal_y, args.goal_yaw],
                'physical_robot_touched': False, 'events': [], 'planner': [],
                'navigation': [], 'health': [], 'trajectory': [], 'scan_age_s': [],
                'motion': [],
                'odom_age_s': [], 'tf_age_s': [], 'paths': [], 'observed_counts': [],
                'localization_error': [],
                'commands': [], 'failure': None}

    class Observer(Node):
        def __init__(self):
            super().__init__('classroom_fault_verifier')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.latest = {}
            self.map = None
            self.path = None
            self.truth = None
            self.truth_at = None
            self.amcl = None
            self.amcl_at = None
            self.obstacle_truth = None
            self.obstacle_reference = None
            self.obstacle_name = None
            self.pre_observed_indices = set()
            self.seen_indices = set()
            self.goal_sent_at = None
            self.goal = self.create_publisher(PoseStamped, '/smart_vision_amr/goal', 10)
            self.fault = self.create_publisher(String, '/classroom_fault/control', 10)
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.move = self.create_client(SetEntityState, '/gazebo/set_entity_state')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')
            self.estop = self.create_client(Trigger, '/smart_vision_amr/emergency_stop')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
            self.create_subscription(NavigationPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_truth,
                                     qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl,
                                     qos_profile_sensor_data)
            self.create_subscription(Int32MultiArray, '/smart_vision_amr/observed_obstacles',
                                     self.on_observed, 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
            for suffix, field, qos in (
                    ('planner_status', 'planner', latched),
                    ('navigation_status', 'navigation', 10),
                    ('localization_health', 'health', latched),
                    ('motion_status', 'motion', latched)):
                self.create_subscription(String, '/smart_vision_amr/'+suffix,
                                         lambda msg, name=field: self.on_status(msg, name), qos)

        def now(self):
            return round(time.monotonic()-start, 3)

        def emit(self, event_name, **values):
            event = {'t': self.now(), 'event': event_name, **values}
            evidence['events'].append(event)
            print(json.dumps(event), flush=True)

        def age(self, stamp):
            clock = self.get_clock().now().nanoseconds
            return round((clock-stamp_ns(stamp))/1e9, 3) if clock > 0 else None

        def on_map(self, message):
            self.map = message
            self.latest['map'] = time.monotonic()

        def on_scan(self, message):
            self.latest['scan'] = time.monotonic()
            evidence['scan_age_s'].append(self.age(message.header.stamp))

        def on_odom(self, message):
            self.latest['odom'] = time.monotonic()
            evidence['odom_age_s'].append(self.age(message.header.stamp))

        def on_tf(self, message):
            for transform in message.transforms:
                pair = (transform.header.frame_id, transform.child_frame_id)
                if pair in (('odom', 'base_footprint'), ('map', 'odom')):
                    self.latest['tf_'+pair[0]] = time.monotonic()
                    if pair[0] == 'odom':
                        evidence['tf_age_s'].append(self.age(transform.header.stamp))

        def on_path(self, message):
            self.path = [(pose.pose.position.x, pose.pose.position.y)
                         for pose in message.poses] if message.poses else None
            coordinates = ';'.join(f'{x:.3f},{y:.3f}' for x,y in (self.path or []))
            evidence['paths'].append({'t': self.now(), 'count': len(message.poses),
                'sha256': hashlib.sha256(coordinates.encode()).hexdigest()})

        def on_truth(self, message):
            if 'waffle_pi' in message.name:
                pose = message.pose[message.name.index('waffle_pi')]
                self.truth = (pose.position.x, pose.position.y)
                q = pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                self.truth_yaw = yaw
                self.truth_at = time.monotonic()
                evidence['trajectory'].append({'t': self.now(), 'x': self.truth[0],
                                               'y': self.truth[1]})
                self.record_localization_pair()
            if self.obstacle_name in message.name:
                pose = message.pose[message.name.index(self.obstacle_name)]
                self.obstacle_truth = (pose.position.x, pose.position.y)
            else:
                self.obstacle_truth = None

        def on_amcl(self, message):
            pose = message.pose.pose
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.amcl = (pose.position.x, pose.position.y, yaw)
            self.amcl_at = time.monotonic()
            self.record_localization_pair()

        def record_localization_pair(self):
            # Gazebo ModelStates has no header stamp. Compare near-simultaneous
            # receipt samples only; never call this exact ground-truth sync.
            if (self.truth is None or self.amcl is None or self.truth_at is None
                    or self.amcl_at is None or abs(self.truth_at-self.amcl_at) > .15):
                return
            error = math.hypot(self.amcl[0]-self.truth[0], self.amcl[1]-self.truth[1])
            yaw_error = math.atan2(math.sin(self.amcl[2]-self.truth_yaw),
                                   math.cos(self.amcl[2]-self.truth_yaw))
            evidence['localization_error'].append({'t': self.now(), 'position_m': error,
                'yaw_rad': yaw_error, 'receipt_gap_s': abs(self.truth_at-self.amcl_at),
                'amcl': list(self.amcl), 'gazebo': [*self.truth, self.truth_yaw]})

        def on_observed(self, message):
            indices = set(message.data)
            self.seen_indices.update(indices)
            near_new = 0
            if self.map is not None and self.obstacle_reference is not None:
                info = self.map.info
                for index in indices-self.pre_observed_indices:
                    if 0 <= index < info.width*info.height:
                        xy = cell_to_world(info, (index % info.width, index // info.width))
                        near_new += math.hypot(xy[0]-self.obstacle_reference[0],
                                               xy[1]-self.obstacle_reference[1]) <= .35
            evidence['observed_counts'].append({'t': self.now(),
                                               'count': len(indices),
                                               'new_near_obstacle': near_new})

        def on_command(self, message):
            evidence['commands'].append({'t': self.now(), 'linear': message.linear.x,
                                         'angular': message.angular.z})

        def on_status(self, message, field):
            value = parse_status(message)
            if value:
                evidence[field].append({'t': self.now(), **value})
                self.latest[field] = time.monotonic()

        def wait(self, predicate, seconds, description):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
                if predicate():
                    return True
            raise TimeoutError(description)

        def service(self, future, seconds, description):
            self.wait(lambda: future.done(), seconds, description)
            return future.result()

    rclpy.init(args=[])
    node = Observer()
    obstacle_spawned = False
    obstacle_live = False
    minimum_clearance = float('inf')
    try:
        node.wait(lambda: (node.map is not None and all(k in node.latest for k in
                  ('scan', 'odom', 'tf_odom', 'tf_map', 'health')) and node.goal.get_subscription_count() > 0),
                  90., 'map, scan, odom, map/base TF and planner were not ready')
        node.emit('inputs_ready')
        node.wait(lambda: evidence['health'] and
                  evidence['health'][-1].get('autonomous_allowed') is True,
                  25., 'localization health never allowed autonomous motion')
        if args.profile != 'clean':
            node.wait(lambda: node.fault.get_subscription_count() > 0, 5., 'fault relay missing')
            node.fault.publish(String(data=json.dumps({'profile': args.profile})))
            node.emit('fault_profile_requested', profile=args.profile)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = args.goal_x
        goal.pose.position.y = args.goal_y
        goal.pose.orientation.z = math.sin(args.goal_yaw/2.)
        goal.pose.orientation.w = math.cos(args.goal_yaw/2.)
        node.goal.publish(goal)
        node.goal_sent_at = time.monotonic()
        node.emit('goal_sent')
        node.wait(lambda: any(path['count'] > 0 and path['t'] >= node.now()-.1
                              for path in evidence['paths']) or bool(node.path),
                  20., 'planner did not produce a nonempty route')
        node.emit('route_generated', points=len(node.path))
        if args.obstacle == 'moving':
            if not node.spawn.service_is_ready() or not node.move.service_is_ready():
                raise RuntimeError('Gazebo entity services are unavailable')
            if node.truth is None:
                raise RuntimeError('Gazebo ground-truth state plugin unavailable')
            # Pick a planned pose 0.6-0.9m ahead. Never place a fixture inside
            # the robot's current footprint, even if a path starts nearby.
            candidate_index = next((index for index, (x, y) in enumerate(node.path)
                if .65 <= math.hypot(x-node.truth[0], y-node.truth[1]) <= 1.), None)
            if candidate_index is None:
                raise RuntimeError('No route point 0.65-1.0 m ahead for sudden obstacle')
            candidate = node.path[candidate_index]
            before_index = max(0, candidate_index-2)
            after_index = min(len(node.path)-1, candidate_index+2)
            direction = (node.path[after_index][0]-node.path[before_index][0],
                         node.path[after_index][1]-node.path[before_index][1])
            length = math.hypot(*direction)
            if length < .05:
                raise RuntimeError('Path tangent is undefined at obstacle fixture')
            perpendicular = (-direction[1]/length, direction[0]/length)
            def wall_clearance(point):
                grid = node.map
                cell = world_to_cell(grid.info, *point)
                if not (0 <= cell[0] < grid.info.width and 0 <= cell[1] < grid.info.height):
                    return -1.
                index = cell[1]*grid.info.width+cell[0]
                if grid.data[index] != 0:
                    return -1.
                nearby = [cell_to_world(grid.info, (j % grid.info.width,
                                                   j // grid.info.width))
                          for j, value in enumerate(grid.data) if value != 0]
                return min((math.hypot(point[0]-x, point[1]-y) for x,y in nearby),
                           default=float('inf'))
            side = max((1., -1.), key=lambda sign: wall_clearance(
                (candidate[0]+.10*sign*perpendicular[0],
                 candidate[1]+.10*sign*perpendicular[1])))
            node.pre_observed_indices = set(node.seen_indices)
            before_spawn = {'local_attempts': max((event.get('local_attempt_id', 0)
                for event in evidence['planner']), default=0),
                'global_plans': max((event.get('global_plan_count', 0)
                for event in evidence['planner']), default=0),
                'path_sha256': evidence['paths'][-1]['sha256'] if evidence['paths'] else None}
            name = 'classroom_fault_'+uuid.uuid4().hex[:10]
            node.obstacle_name = name
            request = SpawnEntity.Request()
            request.name = name
            request.xml = f'''<sdf version="1.6"><model name="{name}"><static>false</static>
              <link name="body"><gravity>false</gravity><inertial><mass>3</mass>
              <inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
              <collision name="body"><geometry><box><size>.25 .25 .5</size></box></geometry></collision>
              <visual name="body"><geometry><box><size>.25 .25 .5</size></box></geometry>
              <material><ambient>.9 .2 .1 1</ambient><diffuse>.9 .2 .1 1</diffuse></material>
              </visual></link></model></sdf>'''
            request.initial_pose.position.x, request.initial_pose.position.y = candidate
            request.initial_pose.position.z = .25
            request.initial_pose.orientation.w = 1.
            response = node.service(node.spawn.call_async(request), 8., 'Gazebo spawn timed out')
            if not response.success:
                raise RuntimeError('Gazebo obstacle spawn failed: '+response.status_message)
            obstacle_spawned = True
            obstacle_live = True
            node.obstacle_reference = candidate
            node.emit('unmapped_obstacle_spawned', name=name, x=candidate[0], y=candidate[1])
            spawned_at = node.now()
            node.wait(lambda: node.obstacle_truth is not None, 5., 'spawned obstacle truth missing')
            obstacle_truth_seen = True
            next_move_at = node.now()+1.
            movement_step = 0
            removed_at = None
        deadline = time.monotonic()+args.timeout
        terminal = None
        blocked = False
        terminal_candidate = None
        terminal_candidate_at = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if obstacle_live and node.truth and node.obstacle_truth:
                center_distance = math.hypot(node.truth[0]-node.obstacle_truth[0],
                                             node.truth[1]-node.obstacle_truth[1])
                clearance = center_distance-(.24+.25*math.sqrt(2)/2)
                minimum_clearance = min(minimum_clearance, clearance)
                if clearance < .02:
                    raise RuntimeError('Actual robot/obstacle footprint clearance below 2 cm')
            if obstacle_live and movement_step < 4 and node.now() >= next_move_at:
                movement_step += 1
                shift = .025*movement_step*side
                state = EntityState()
                state.name = node.obstacle_name
                state.reference_frame = 'world'
                state.pose.position.x = candidate[0]+shift*perpendicular[0]
                state.pose.position.y = candidate[1]+shift*perpendicular[1]
                state.pose.position.z = .25
                state.pose.orientation.w = 1.
                moved = node.service(node.move.call_async(SetEntityState.Request(state=state)),
                                     5., 'Gazebo obstacle move timed out')
                if not moved.success:
                    raise RuntimeError('Gazebo moving obstacle command rejected')
                node.emit('obstacle_moved', step=movement_step,
                          x=state.pose.position.x, y=state.pose.position.y)
                next_move_at = node.now()+.8
            if obstacle_live and node.now()-spawned_at >= 14.:
                request = DeleteEntity.Request()
                request.name = node.obstacle_name
                deleted = node.service(node.delete.call_async(request), 6.,
                                       'Gazebo obstacle removal timed out')
                if not deleted.success:
                    raise RuntimeError('Gazebo obstacle removal rejected')
                obstacle_live = False
                removed_at = node.now()
                node.emit('obstacle_removed', name=node.obstacle_name)
            if evidence['navigation']:
                recent = evidence['navigation'][-1]
                status = recent.get('status')
                if status == 'SUCCEEDED':
                    terminal = status
                    break
                if status in ('FAILED', 'CANCELED'):
                    if terminal_candidate != status:
                        terminal_candidate = status
                        terminal_candidate_at = time.monotonic()
                        node.emit('navigation_terminal_candidate', status=status)
                    elif time.monotonic()-terminal_candidate_at >= 5.:
                        terminal = status
                        break
                elif terminal_candidate is not None:
                    node.emit('navigation_recovered', previous=terminal_candidate,
                              status=status)
                    terminal_candidate = None
                    terminal_candidate_at = None
            if (args.expect_sustained_outage_hold and
                    time.monotonic()-node.goal_sent_at >= 21.0 and
                    evidence['health'] and evidence['health'][-1].get('stop_latched') is True and
                    evidence['motion'] and evidence['motion'][-1].get('estop_latched') is True):
                terminal = 'SAFE_HOLD'
                break
            # An injected dropout may briefly trip the independent interlock.
            # Observe the whole recovery window; a safe stop alone does not
            # prove the retained goal was resumed or completed.
            if evidence['health'] and time.monotonic()-node.goal_sent_at > .5:
                allowed = evidence['health'][-1].get('autonomous_allowed') is True
                if not allowed and not blocked:
                    blocked = True
                    node.emit('safety_blocked', reason=evidence['health'][-1].get('reasons'))
                elif allowed and blocked:
                    blocked = False
                    node.emit('safety_recovered')
        else:
            terminal = 'TIMEOUT'
        # Check the physical stop after every terminal state. Navigation may
        # fail while still issuing commands; that is a separate, more serious
        # result than a safely held goal. A transient zero command is not proof.
        first_hold = node.now()
        until = time.monotonic()+1.2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        later_cmd = [sample for sample in evidence['commands']
                     if sample['t'] >= first_hold+.2]
        later_truth = [sample for sample in evidence['trajectory']
                       if sample['t'] >= first_hold+.2]
        safe_hold = (len(later_cmd) >= 3 and len(later_truth) >= 3
                     and all(abs(sample['linear']) < .01 and abs(sample['angular']) < .02
                             for sample in later_cmd)
                     and max(math.hypot(sample['x']-later_truth[0]['x'],
                                        sample['y']-later_truth[0]['y'])
                             for sample in later_truth) < .025)
        node.emit('trial_terminal', status=terminal or 'SAFETY_HOLD', safe_hold=safe_hold)
        latest_plan = evidence['planner'][-1] if evidence['planner'] else {}
        local_attempts = max((event.get('local_attempt_id', 0) for event in evidence['planner']), default=0)
        global_plans = max((event.get('global_plan_count', 0) for event in evidence['planner']), default=0)
        local_active = any(event.get('local_detour_state') == 'ACTIVE'
                           for event in evidence['planner'])
        goal_distance = (None if node.truth is None else
                         math.hypot(node.truth[0]-args.goal_x,
                                    node.truth[1]-args.goal_y))
        success_pose = goal_distance is not None and goal_distance <= .35
        evidence['metrics'] = {'terminal': terminal or 'SAFETY_HOLD',
                               'safe_hold_proven': safe_hold,
                               'actual_goal_distance_m': goal_distance,
                               'actual_goal_within_35cm': success_pose,
                               'local_attempts': local_attempts,
                               'safety_block_events': sum(event['event'] == 'safety_blocked'
                                    for event in evidence['events']),
                               'safety_recovery_events': sum(event['event'] == 'safety_recovered'
                                    for event in evidence['events']),
                               'local_detour_selected': local_active,
                               'new_near_obstacle_after_spawn': max((event.get('new_near_obstacle', 0)
                                   for event in evidence['observed_counts']
                                   if obstacle_spawned and event['t'] > spawned_at
                                      and (removed_at is None or event['t'] < removed_at)), default=0)
                                   if obstacle_spawned else None,
                               'obstacle_motion_steps': movement_step if obstacle_spawned else None,
                               'obstacle_removed': removed_at is not None if obstacle_spawned else None,
                               'global_plan_count': global_plans,
                               'minimum_obstacle_clearance_m':
                                   None if not obstacle_spawned else minimum_clearance,
                               'path_publications': len(evidence['paths']),
                               'localization_error_max_m': max((sample['position_m']
                                   for sample in evidence['localization_error']), default=None),
                               'final_planner': latest_plan}
        if args.expect_sustained_outage_hold:
            hold_metrics, hold_passed = assess_sustained_outage_hold(
                terminal, safe_hold, evidence['health'], evidence['motion'],
                latest_plan, evidence['navigation'],
                (args.goal_x, args.goal_y), node.goal_sent_at-start, node.now())
            evidence['metrics'].update(hold_metrics)
            evidence['passed'] = hold_passed
        elif args.obstacle == 'moving':
            obstacle_observed = evidence['metrics']['new_near_obstacle_after_spawn'] > 0
            first_observed_at = min((event['t'] for event in evidence['observed_counts']
                if event['t'] > spawned_at and event.get('new_near_obstacle', 0) > 0),
                default=None)
            after_removal = [event for event in evidence['observed_counts']
                             if removed_at is not None and event['t'] > removed_at+2.]
            observation_cleared = (len(after_removal) >= 3 and all(
                event['new_near_obstacle'] == 0 for event in after_removal[-3:]))
            path_changed = any(first_observed_at is not None
                               and path['t'] >= first_observed_at and path['count'] > 0
                               and path['sha256'] != before_spawn['path_sha256']
                               for path in evidence['paths'])
            reacted = (path_changed or any(first_observed_at is not None
                       and event['t'] >= first_observed_at
                       and (event.get('local_attempt_id', 0) > before_spawn['local_attempts']
                            or event.get('global_plan_count', 0) > before_spawn['global_plans'])
                       for event in evidence['planner']))
            evidence['metrics']['obstacle_observed'] = obstacle_observed
            evidence['metrics']['route_reacted_after_obstacle'] = reacted
            evidence['metrics']['obstacle_observation_cleared'] = observation_cleared
            evidence['passed'] = (obstacle_spawned and obstacle_truth_seen
                                  and movement_step == 4 and removed_at is not None
                                  and obstacle_observed and observation_cleared and reacted
                                  and math.isfinite(minimum_clearance)
                                  and minimum_clearance >= .02
                                  and ((terminal == 'SUCCEEDED' and success_pose)
                                       or (args.allow_safe_hold and safe_hold)))
        else:
            evidence['passed'] = ((terminal == 'SUCCEEDED' and success_pose)
                                  or (args.allow_safe_hold and safe_hold))
        if args.profile != 'clean':
            evidence['passed'] = evidence['passed'] and bool(evidence['scan_age_s'])
        if not evidence['passed']:
            evidence['failure_class'] = ('navigation_failed' if terminal == 'FAILED' else
                'navigation_canceled' if terminal == 'CANCELED' else
                'trial_timeout' if terminal == 'TIMEOUT' else
                'safety_hold' if terminal is None else
                'goal_pose_mismatch' if terminal == 'SUCCEEDED' and not success_pose else
                'acceptance_metric_failed')
    except Exception as exc:
        evidence['failure'] = repr(exc)
        node.emit('trial_failed', error=repr(exc))
    finally:
        if node.estop.service_is_ready():
            try:
                node.service(node.estop.call_async(Trigger.Request()), 4., 'simulation stop timed out')
            except Exception:
                pass
        if obstacle_live and node.delete.service_is_ready():
            try:
                request = DeleteEntity.Request()
                request.name = node.obstacle_name
                node.service(node.delete.call_async(request), 4., 'owned obstacle deletion timed out')
            except Exception:
                pass
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('PASS' if evidence['passed'] else 'FAIL', args.output, flush=True)
    return 0 if evidence['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
