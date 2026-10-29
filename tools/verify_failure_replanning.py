#!/usr/bin/env python3
"""Isolated Gazebo unseen-obstacle test; never inject execution-failure reports.

Requires an already running simulator in ROS domain 185, localhost-only, Gazebo
port 11585. The tool sends one goal, inserts a real collision+LiDAR box into its
initial route, observes retained-goal replanning/completion, and deletes only its
own obstacle. Estimated map TF distance is not Gazebo ground-truth accuracy.
"""
import argparse
import json
import math
import os
from pathlib import Path as FilePath
import sys
import time
import uuid


def isolation_check():
    expected = {'ROS_DOMAIN_ID': '185', 'ROS_LOCALHOST_ONLY': '1',
                'GAZEBO_MASTER_URI': 'http://127.0.0.1:11585'}
    if any(os.environ.get(key) != value for key, value in expected.items()):
        raise RuntimeError('Refusing mutation outside isolated domain185/localhost/Gazebo11585')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=FilePath, required=True)
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--stop-file', type=FilePath, help='Operator may create this new file to end the trial with an honest FAIL record and normal owned-goal cleanup.')
    parser.add_argument('--scenario', choices=('blocking', 'unrelated', 'blocking_hold_remove'), default='blocking')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 300:
        parser.error('--timeout must be finite and between 0 and 300 seconds')
    box_xy = (-1.6, -2.) if args.scenario != 'unrelated' else (-2.2, -.3)
    isolation_check()
    if args.stop_file is not None and args.stop_file.exists():
        parser.error('Stop file already exists; select a new operator stop path')
    if args.output.exists():
        raise RuntimeError('Refusing to overwrite previous trial: '+str(args.output))
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener, TransformException
    from smart_vision_amr.navigation_graph import goal_consumers_ready

    class Probe(Node):
        def __init__(self):
            super().__init__('unseen_obstacle_trial_'+uuid.uuid4().hex[:8],
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.goal_pub = self.create_publisher(PoseStamped, '/smart_vision_amr/goal', 10)
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')
            self.cancel = self.create_client(Trigger, '/smart_vision_amr/cancel_navigation')
            self.tf = Buffer(node=self)
            self.listener = TransformListener(self.tf, self)
            self.map = self.odom = self.scan = self.motion = self.health = None
            self.started = self.spawned = False
            self.goal_stamp = None
            self.start_xy = None
            self.displacement = 0.
            self.command_seen = False
            self.paths_before, self.paths_after = [], []
            self.statuses, self.failures, self.memory_states, self.recoveries = [], [], [], []
            self.planner_statuses, self.path_records, self.map_xy_trace = [], [], []
            self.latest_status = None
            self.hold_started = self.hold_settled_at = None
            self.hold_odom_anchor = None
            self.hold_stationary_samples = []
            self.hold_command_samples = []
            self.obstacle_removed_at = self.resumed_after_removal_at = None
            self.obstacle_removed_ros_ns = None
            self.hold_verified = False
            self.hold_state_snapshot = None
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), latched)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
            self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
            self.create_subscription(Path, '/smart_vision_amr/path', self.on_path, 10)
            for topic, field, qos in [('/smart_vision_amr/motion_status', 'motion', latched),
                                      ('/smart_vision_amr/localization_health', 'health', latched),
                                      ('/smart_vision_amr/navigation_status', 'navigation', 10),
                                      ('/smart_vision_amr/planner_status', 'planner', latched),
                                      ('/smart_vision_amr/execution_failure', 'failure', 10),
                                      ('/smart_vision_amr/failure_memory_status', 'memory', latched),
                                      ('/smart_vision_amr/recovery_request', 'recovery', 10)]:
                self.create_subscription(String, topic, lambda m, f=field: self.on_json(m, f), qos)

        def on_json(self, message, field):
            try:
                value = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if field in ('motion', 'health'):
                setattr(self, field, value)
            elif self.started:
                if field == 'navigation':
                    self.latest_status = value
                    if value.get('goal_stamp') == self.goal_stamp:
                        if not self.statuses or (value.get('status'), value.get('message')) != (self.statuses[-1].get('status'), self.statuses[-1].get('message')):
                            self.statuses.append(value)
                else:
                    target = {'failure': self.failures, 'memory': self.memory_states,
                              'recovery': self.recoveries, 'planner': self.planner_statuses}[field]
                    if not target or value != target[-1]:
                        target.append(value)

        def pose(self):
            try:
                transform = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                p = transform.transform.translation
                return [p.x, p.y]
            except TransformException:
                return None

        def on_odom(self, message):
            self.odom = message
            if self.started:
                p = message.pose.pose.position
                self.displacement = max(self.displacement, math.hypot(p.x-self.start_xy[0], p.y-self.start_xy[1]))

        def on_command(self, message):
            now = time.monotonic()
            if self.hold_settled_at is not None and self.obstacle_removed_at is None:
                self.hold_command_samples.append([now-self.hold_settled_at, message.linear.x, message.angular.z])
            if self.obstacle_removed_at is not None and self.resumed_after_removal_at is None and (abs(message.linear.x) > .01 or abs(message.angular.z) > .05):
                self.resumed_after_removal_at = now
            if self.started and (abs(message.linear.x) > .01 or abs(message.angular.z) > .05):
                self.command_seen = True

        def on_path(self, message):
            if not self.started or not message.poses:
                return
            points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
            (self.paths_after if self.spawned else self.paths_before).append(points)
            self.path_records.append({'after_obstacle': self.spawned, 'after_removal': self.obstacle_removed_at is not None, 'stamp_ns': message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec, 'points': points})

        def ready(self):
            consumers = {entry.node_name for entry in self.get_subscriptions_info_by_topic('/smart_vision_amr/goal')}
            return (self.map is not None and self.odom is not None and self.scan is not None
                    and self.pose() is not None and self.spawn.service_is_ready()
                    and goal_consumers_ready(consumers) and (self.health or {}).get('level') == 'ok'
                    and (self.health or {}).get('autonomous_allowed') is True
                    and (self.motion or {}).get('mode') == 'auto'
                    and (self.motion or {}).get('estop_latched') is False
                    and (self.motion or {}).get('autonomous_allowed') is True)

    def spin_until(future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done():
            raise RuntimeError('Gazebo service timeout')
        return future.result()

    rclpy.init(args=[])
    node = Probe()
    box_name = 'unseen_obstacle_trial_'+uuid.uuid4().hex[:12]
    start = time.monotonic()
    result = {'trial': 'actual_unseen_collision_lidar_box', 'domain': 185,
              'goal': [-.5, -2., 0.], 'obstacle': {'name': box_name, 'xy': box_xy, 'size': [.5, .9, 1.]},
              'failure_events_injected': False, 'scenario': args.scenario, 'passed': False}
    try:
        deadline = time.monotonic()+90.
        while not node.ready():
            if time.monotonic() > deadline:
                raise RuntimeError('Stack not ready: '+json.dumps({'health': node.health, 'motion': node.motion, 'pose': node.pose()}))
            rclpy.spin_once(node, timeout_sec=.1)
        # Verify this obstacle was absent from the actual saved map.
        info = node.map.info
        cx, cy = int((box_xy[0]-info.origin.position.x)/info.resolution), int((box_xy[1]-info.origin.position.y)/info.resolution)
        occupancy = node.map.data[cy*info.width+cx]
        if occupancy != 0:
            raise RuntimeError('The injected obstacle center is not known-free in the saved map')
        result['original_obstacle_cell_occupancy'] = occupancy
        p = node.odom.pose.pose.position
        node.start_xy = [p.x, p.y]
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w = -.5, -2., 1.
        node.goal_stamp = {'sec': goal.header.stamp.sec, 'nanosec': goal.header.stamp.nanosec}
        node.started = True
        node.goal_pub.publish(goal)
        start = time.monotonic()
        next_report = start+10.
        while time.monotonic()-start < args.timeout:
            if args.stop_file is not None and args.stop_file.exists():
                result['operator_terminated'] = True
                raise RuntimeError('Operator ended trial via explicit stop file: '+str(args.stop_file))
            rclpy.spin_once(node, timeout_sec=.1)
            pose_now = node.pose()
            if pose_now is not None and (not node.map_xy_trace or time.monotonic()-start-node.map_xy_trace[-1][0] >= .2):
                node.map_xy_trace.append([round(time.monotonic()-start,3), *pose_now])
            if not node.spawned and node.displacement >= .25 and node.paths_before:
                request = SpawnEntity.Request()
                request.name = box_name
                request.xml = f'''<sdf version="1.6"><model name="{box_name}"><static>true</static><link name="body"><collision name="collision"><geometry><box><size>0.5 0.9 1.0</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.5 0.9 1.0</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>'''
                request.initial_pose.position.x, request.initial_pose.position.y = box_xy
                request.initial_pose.position.z, request.initial_pose.orientation.w = .5, 1.
                reply = spin_until(node.spawn.call_async(request), time.monotonic()+10.)
                if not reply.success:
                    raise RuntimeError('Obstacle spawn failed: '+reply.status_message)
                node.spawned = True
                result['obstacle_spawn_elapsed_seconds'] = time.monotonic()-start
                result['robot_map_xy_at_spawn'] = node.pose()
                print('Inserted actual unseen collision/LiDAR obstacle', flush=True)
            if args.scenario == 'blocking_hold_remove' and node.obstacle_removed_at is None:
                latest_memory = node.memory_states[-1] if node.memory_states else {}
                in_hold = latest_memory.get('state') == 'OBSTACLE_HOLD' or (node.latest_status or {}).get('status') == 'OBSTACLE_HOLD'
                if in_hold and node.hold_started is None:
                    node.hold_started = time.monotonic()
                    node.hold_state_snapshot = latest_memory
                    result['obstacle_hold_elapsed_seconds'] = node.hold_started-start
                    print('Observed actual OBSTACLE_HOLD; checking stopped output and odometry', flush=True)
                if in_hold and node.hold_started is not None and time.monotonic()-node.hold_started >= .5:
                    if node.hold_settled_at is None:
                        node.hold_settled_at = time.monotonic()
                        odom_p = node.odom.pose.pose.position
                        node.hold_odom_anchor = [odom_p.x, odom_p.y]
                    odom_p, odom_v = node.odom.pose.pose.position, node.odom.twist.twist
                    node.hold_stationary_samples.append([time.monotonic()-node.hold_settled_at,
                        math.hypot(odom_p.x-node.hold_odom_anchor[0], odom_p.y-node.hold_odom_anchor[1]),
                        odom_v.linear.x, odom_v.angular.z])
                    if time.monotonic()-node.hold_settled_at >= 3.:
                        node.hold_verified = (len(node.hold_stationary_samples) >= 10 and len(node.hold_command_samples) >= 10
                            and all(displacement <= .03 and abs(v) <= .01 and abs(w) <= .03 for _,displacement,v,w in node.hold_stationary_samples)
                            and all(abs(v) <= .001 and abs(w) <= .001 for _,v,w in node.hold_command_samples))
                        if not node.hold_verified:
                            raise RuntimeError('OBSTACLE_HOLD did not remain stopped for 3 seconds')
                        request = DeleteEntity.Request()
                        request.name = box_name
                        reply = spin_until(node.delete.call_async(request), time.monotonic()+5.)
                        if not reply.success:
                            raise RuntimeError('Actual obstacle deletion failed: '+reply.status_message)
                        node.obstacle_removed_at = time.monotonic()
                        node.obstacle_removed_ros_ns = node.get_clock().now().nanoseconds
                        result['obstacle_removed_elapsed_seconds'] = node.obstacle_removed_at-start
                        print('Stopped hold verified; deleted actual obstacle, awaiting live-scan clear and same-goal resume', flush=True)
            if node.latest_status and node.latest_status.get('goal_stamp') not in (None, node.goal_stamp):
                raise RuntimeError('Another goal took ownership; not interfering')
            if (node.motion or {}).get('estop_latched') is True:
                raise RuntimeError('Safety stop latched during test: '+str((node.motion or {}).get('reason')))
            if node.statuses and node.statuses[-1].get('status') == 'SUCCEEDED':
                break
            if time.monotonic() >= next_report:
                print(json.dumps({'elapsed': round(time.monotonic()-start, 1), 'pose': node.pose(),
                                  'paths_after': len(node.paths_after), 'last_status': node.statuses[-1] if node.statuses else None,
                                  'execution_failure_events': len(node.failures)}), flush=True)
                next_report += 10.
        pose = node.pose()
        distance = None if pose is None else math.hypot(pose[0]+.5, pose[1]+2.)
        detours = [path for path in node.paths_after if max(abs(y+2.) for x, y in path) >= .60]
        checks = {'real_obstacle_spawned': node.spawned,
                  'original_route_existed_before_obstacle': bool(node.paths_before),
                  'same_explicit_goal_retained': bool(node.statuses) and all(s.get('goal_stamp') == node.goal_stamp for s in node.statuses),
                  'actual_motion_observed': node.command_seen and node.displacement > .3,
                  'navigation_succeeded': bool(node.statuses) and node.statuses[-1].get('status') == 'SUCCEEDED',
                  'estimated_goal_error_under_0_35m': distance is not None and distance <= .35}
        checks['every_path_retained_original_goal_endpoint'] = bool(node.paths_before) and all(math.hypot(points[-1][0]+.5, points[-1][1]+2.) <= .1 for points in node.paths_before+node.paths_after)
        if args.scenario == 'blocking':
            checks['alternative_path_around_obstacle'] = bool(detours)
        from smart_vision_amr.path_geometry import route_cells
        from smart_vision_amr.map_utils import inflate_cells, cell_to_world, yaw_from_quaternion
        raw_cells = {(x,y) for y in range(info.height) for x in range(info.width)
                     if abs(cell_to_world(info,(x,y))[0]-box_xy[0]) <= .25
                     and abs(cell_to_world(info,(x,y))[1]-box_xy[1]) <= .45}
        inflated_cells = inflate_cells(raw_cells, info.width, info.height, 6)
        yaw = yaw_from_quaternion(info.origin.orientation)
        def overlap(points):
            projected = [((math.cos(yaw)*(x-info.origin.position.x)+math.sin(yaw)*(y-info.origin.position.y))/info.resolution,
                          (-math.sin(yaw)*(x-info.origin.position.x)+math.cos(yaw)*(y-info.origin.position.y))/info.resolution) for x,y in points]
            cells = route_cells(projected)
            return {'raw_intersection_cells': len(cells & raw_cells),
                    'inflated_intersection_cells': len(cells & inflated_cells)}
        for record in node.path_records:
            record.update(overlap(record['points']))
        after_records = [r for r in node.path_records if r['after_obstacle']]
        if after_records and args.scenario != 'blocking_hold_remove':
            checks['final_replanned_path_clear_actual_raw_obstacle'] = after_records[-1]['raw_intersection_cells'] == 0
            checks['final_replanned_path_clear_actual_inflated_obstacle'] = after_records[-1]['inflated_intersection_cells'] == 0
        elif args.scenario == 'blocking':
            checks['final_replanned_path_clear_actual_raw_obstacle'] = False
        if args.scenario == 'blocking_hold_remove':
            checks['blocked_retry_hold_observed'] = node.hold_started is not None
            hold_snapshot = node.hold_state_snapshot or {}
            checks['hold_after_three_confirmed_blocked_approaches'] = hold_snapshot.get('retry_count') == hold_snapshot.get('retry_limit') == 3
            checks['hold_zero_command_and_stationary_for_3s'] = node.hold_verified
            checks['actual_obstacle_removed_after_hold'] = node.obstacle_removed_at is not None
            checks['fresh_clear_before_automatic_resume'] = (node.resumed_after_removal_at is not None and node.obstacle_removed_at is not None and node.resumed_after_removal_at-node.obstacle_removed_at >= 1.8)
            result['removal_to_motion_seconds'] = (None if node.resumed_after_removal_at is None or node.obstacle_removed_at is None else node.resumed_after_removal_at-node.obstacle_removed_at)
            result['hold_state_snapshot'] = node.hold_state_snapshot
            result['hold_stationary_samples'] = node.hold_stationary_samples
            result['hold_command_samples'] = node.hold_command_samples
        if args.scenario == 'unrelated':
            from smart_vision_amr.path_geometry import paths_are_equivalent
            checks['unrelated_route_remained_equivalent'] = bool(node.paths_before) and all(paths_are_equivalent(node.paths_before[0], points) for points in node.paths_after)
            checks['unrelated_obstacle_no_large_detour'] = all(max(abs(y+2.) for x,y in points) <= .35 for points in node.paths_after+node.paths_before)
            checks['unrelated_obstacle_no_execution_failure_or_reverse'] = not node.failures and not node.recoveries
            result['unrelated_path_publication_count'] = len(node.paths_after)
        result['path_records'] = node.path_records
        result['actual_obstacle_raw_cell_count'] = len(raw_cells)
        result['actual_obstacle_inflation_cells'] = 6
        result['actual_obstacle_inflation_m'] = 6*info.resolution
        result.update({'checks': checks, 'passed': all(checks.values()), 'final_map_xy': pose,
                       'estimated_goal_distance_m': distance, 'path_publications_before': len(node.paths_before),
                       'path_publications_after': len(node.paths_after), 'maximum_odom_displacement_m': node.displacement})
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        result.update({'wall_elapsed_seconds': round(time.monotonic()-start, 3),
                       'navigation_statuses': node.statuses, 'planner_statuses': node.planner_statuses,
                       'path_records': node.path_records, 'estimated_map_xy_trace': node.map_xy_trace, 'execution_failure_events': node.failures,
                       'failure_memory_states': node.memory_states, 'recovery_requests': node.recoveries,
                       'final_localization_health': node.health, 'obstacle_removed': node.obstacle_removed_at is not None,
                       'obstacle_removed_ros_ns': node.obstacle_removed_ros_ns,
                       'hold_state_snapshot': node.hold_state_snapshot,
                       'hold_stationary_samples': node.hold_stationary_samples,
                       'hold_command_samples': node.hold_command_samples,
                       'obstacle_removed_elapsed_seconds': None if node.obstacle_removed_at is None else node.obstacle_removed_at-start,
                       'removal_to_motion_seconds': None if node.obstacle_removed_at is None or node.resumed_after_removal_at is None else node.resumed_after_removal_at-node.obstacle_removed_at})
        # A failed test cancels only its own stamped goal, never a foreign route.
        if (not result['passed'] and node.started and node.cancel.service_is_ready()
                and (node.latest_status is None or node.latest_status.get('goal_stamp') == node.goal_stamp)):
            try:
                spin_until(node.cancel.call_async(Trigger.Request()), time.monotonic()+3.)
            except Exception:
                pass
        if node.spawned and node.obstacle_removed_at is None and node.delete.service_is_ready():
            request = DeleteEntity.Request()
            request.name = box_name
            try:
                spin_until(node.delete.call_async(request), time.monotonic()+3.)
            except Exception:
                pass
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(('PASS ' if result['passed'] else 'FAIL ')+str(args.output), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
