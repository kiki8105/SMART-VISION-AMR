#!/usr/bin/env python3
"""Move only named warehouse actors in a loopback Gazebo simulation.

The world owns the visible/collidable models. This process changes their poses
through SetEntityState; it never publishes robot commands, goals or ownership.
"""
from dataclasses import dataclass, replace
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import time

import yaml


def number(value, name, low=None, high=None):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or low is not None and value < low or high is not None and value > high):
        raise ValueError('Invalid '+name)
    return float(value)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float = 0.
    yaw: float = 0.


@dataclass(frozen=True)
class Actor:
    model_name: str
    waypoints: tuple
    speed_mps: float
    collision_radius_m: float
    dwell_seconds: float = 2.
    loop: str = 'ping_pong'
    max_yaw_rate_rps: float = 1.
    max_step_m: float = .03


@dataclass(frozen=True)
class Motion:
    pose: Pose
    target: int = 1
    direction: int = 1
    dwell_remaining: float = 0.
    finished: bool = False


def segment_point_distance(a, b, p):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(a[0]+t*dx-p[0], a[1]+t*dy-p[1])


def segment_box_distance(a, b, box):
    """Exact segment-to-original occupied rectangle distance, including edges."""
    lo, hi = 0., 1.
    for axis in (0, 1):
        delta = b[axis]-a[axis]
        if abs(delta) < 1.e-12:
            if not box[axis] <= a[axis] <= box[axis+2]:
                break
        else:
            t0, t1 = (box[axis]-a[axis])/delta, (box[axis+2]-a[axis])/delta
            lo, hi = max(lo, min(t0, t1)), min(hi, max(t0, t1))
            if lo > hi:
                break
    else:
        return 0.
    point_gap = lambda p: math.hypot(max(box[0]-p[0], 0., p[0]-box[2]),
                                   max(box[1]-p[1], 0., p[1]-box[3]))
    return min(point_gap(a), point_gap(b), *(segment_point_distance(a, b, (x, y))
        for x in (box[0], box[2]) for y in (box[1], box[3])))


def load_routes(path):
    source = Path(path).read_bytes()
    data = yaml.safe_load(source)
    if not isinstance(data, dict) or data.get('frame_id') != 'world':
        raise ValueError('Routes must explicitly use Gazebo world coordinates')
    rate = number(data.get('update_rate_hz', 10.), 'update_rate_hz', 2., 30.)
    maximum_step = number(data.get('max_step_m', .03), 'max_step_m', .001, .05)
    robot_radius = number(data.get('robot_radius_m', .22), 'robot_radius_m', .20, .5)
    clearance = number(data.get('robot_clearance_m', .15), 'robot_clearance_m', .02, 1.)
    static_margin = number(data.get('static_clearance_m', .02), 'static_clearance_m', 0., .5)
    robot_name = data.get('robot_model_name', 'waffle_pi')
    if not isinstance(robot_name, str) or not robot_name:
        raise ValueError('Invalid robot_model_name')
    walls = []
    if not isinstance(data.get('fixed_obstacles'), list) or not data['fixed_obstacles']:
        raise ValueError('Fixed occupied rectangles are required to validate entire actor routes')
    for item in data['fixed_obstacles']:
        if not isinstance(item, dict):
            raise ValueError('Invalid fixed obstacle')
        box = tuple(number(item.get(key), key) for key in ('x_min', 'y_min', 'x_max', 'y_max'))
        if box[0] >= box[2] or box[1] >= box[3]:
            raise ValueError('Invalid fixed obstacle bounds')
        walls.append(box)
    actors, names = [], set()
    if not isinstance(data.get('actors'), list) or not 1 <= len(data['actors']) <= 10:
        raise ValueError('Expected 1--10 bounded warehouse actors')
    for item in data['actors']:
        name = item.get('model_name') if isinstance(item, dict) else None
        if (not isinstance(name, str) or not re.fullmatch(r'warehouse_[A-Za-z0-9_]+', name)
                or name in names or name == robot_name):
            raise ValueError('Only unique warehouse_* actor models may be controlled')
        names.add(name)
        speed = number(item.get('speed_mps'), 'speed_mps', .01, .4)
        radius = number(item.get('collision_radius_m'), 'collision_radius_m', .05, 1.)
        dwell = number(item.get('dwell_seconds', 2.), 'dwell_seconds', 0., 60.)
        yaw_rate = number(item.get('max_yaw_rate_rps', 1.), 'max_yaw_rate_rps', .1, 2.)
        actor_step = number(item.get('max_step_m', maximum_step), 'actor.max_step_m', .001, .05)
        mode = item.get('loop', 'ping_pong')
        if mode not in {'ping_pong', 'loop', 'once'}:
            raise ValueError('Unsupported actor loop')
        waypoints = item.get('waypoints')
        if (not isinstance(waypoints, list) or len(waypoints) < 2
                or not all(isinstance(p, dict) for p in waypoints)):
            raise ValueError('Every actor requires at least two waypoints')
        points = tuple(Pose(number(p.get('x'), 'waypoint.x'), number(p.get('y'), 'waypoint.y'),
                            number(p.get('z', 0.), 'waypoint.z', 0., 2.),
                            number(p.get('yaw', 0.), 'waypoint.yaw')) for p in waypoints)
        segments = list(zip(points, points[1:]))
        if mode == 'loop':
            segments.append((points[-1], points[0]))
        for a, b in segments:
            if abs(a.z-b.z) > 1.e-6:
                raise ValueError('Warehouse actors must keep a fixed ground height')
            if math.hypot(a.x-b.x, a.y-b.y) < .01:
                raise ValueError('Duplicate/degenerate actor waypoint')
            if any(segment_box_distance((a.x, a.y), (b.x, b.y), box) < radius+static_margin
                   for box in walls):
                raise ValueError('Actor '+name+' route intersects a fixed wall/fixture clearance')
        actors.append(Actor(name, points, speed, radius, dwell, mode, yaw_rate, actor_step))
    return {'actors': tuple(actors), 'walls': tuple(walls), 'rate': rate, 'max_step': maximum_step,
            'robot_radius': robot_radius, 'robot_clearance': clearance, 'robot_name': robot_name,
            'static_margin': static_margin,
            'sha256': hashlib.sha256(source).hexdigest(), 'source': str(Path(path).resolve())}


def propose(actor, state, dt, max_step=.03):
    """One bounded sim-time step; yielded/rejected movement never advances it."""
    number(dt, 'simulation delta', 0.)
    if dt == 0. or state.finished:
        return state
    if state.dwell_remaining > 0.:
        return replace(state, dwell_remaining=max(0., state.dwell_remaining-dt))
    target = actor.waypoints[state.target]
    dx, dy = target.x-state.pose.x, target.y-state.pose.y
    distance = math.hypot(dx, dy)
    yaw_target = math.atan2(dy, dx) if distance > 1.e-9 else state.pose.yaw
    error = angle(yaw_target-state.pose.yaw)
    yaw = angle(state.pose.yaw+max(-actor.max_yaw_rate_rps*dt,
                                  min(actor.max_yaw_rate_rps*dt, error)))
    # Carts turn in place instead of strafing abruptly around a corner. Their
    # circumscribed collision circle also covers every intermediate heading.
    length = 0. if abs(error) > .2 else min(distance, actor.speed_mps*dt, max_step, actor.max_step_m)
    pose = Pose(state.pose.x+dx*length/distance if distance else state.pose.x,
                state.pose.y+dy*length/distance if distance else state.pose.y, state.pose.z, yaw)
    result = replace(state, pose=pose)
    if distance-length <= 1.e-9:
        if actor.loop == 'once' and state.target == len(actor.waypoints)-1:
            return replace(result, finished=True)
        direction = state.direction
        if actor.loop == 'ping_pong' and state.target in {0, len(actor.waypoints)-1}:
            direction *= -1
        next_target = ((state.target+1) % len(actor.waypoints) if actor.loop == 'loop'
                       else state.target+direction)
        result = replace(result, target=next_target, direction=direction,
                         dwell_remaining=actor.dwell_seconds)
    return result


def movement_allowed(actor, start, end, robot, truth_age, *, robot_radius=.22,
                     clearance=.15, robot_max_speed=.22, reaction=.25, others=()):
    """Certify the whole actor step against fresh robot/other actor truth.

    The circle encloses the complete actor collision model for any yaw. The
    robot reaction/age disc prevents a service update from entering its near
    future footprint. A close robot causes yielding, never actor relocation.
    """
    if (robot is None or truth_age is None or not math.isfinite(truth_age)
            or not 0. <= truth_age < .5):
        return False, 'robot_truth_unavailable', None
    gap = segment_point_distance((start.x, start.y), (end.x, end.y),
                                 (robot.x, robot.y))-actor.collision_radius_m-robot_radius
    required = clearance+robot_max_speed*(reaction+truth_age)
    if gap < required:
        return False, 'yield_to_robot', gap
    for pose, radius, speed in others:
        if segment_point_distance((start.x, start.y), (end.x, end.y), (pose.x, pose.y)) < (
                actor.collision_radius_m+radius+.05+speed*(reaction+truth_age)):
            return False, 'yield_to_actor', gap
    return True, 'clear', gap


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--routes', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=178)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--startup-timeout', type=float, default=60.)
    parser.add_argument('--check-only', action='store_true', help='Validate geometry without importing ROS')
    args = parser.parse_args(argv)
    if args.domain == 16 or not 0 <= args.domain <= 232:
        parser.error('Physical domain 16 is prohibited; use isolated simulation domain 178')
    number(args.startup_timeout, 'startup-timeout', 1., 180.)
    config = load_routes(args.routes)
    if args.check_only:
        print(json.dumps({'valid': True, 'routes_sha256': config['sha256'],
                          'models': [a.model_name for a in config['actors']]}))
        return 0
    # Force local transport even if an unrelated shell set otherwise. No
    # robot-side control API, velocity publisher or navigation action exists.
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.clock import Clock as RosClock, ClockType
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import SetEntityState
    from rosgraph_msgs.msg import Clock

    class Driver(Node):
        def __init__(self, context):
            super().__init__('warehouse_dynamic_obstacles', context=context)
            self.sim_time = None
            self.last_tick = None
            self.clock_advances = 0
            self.truth, self.truth_at = {}, None
            self.motion, self.pending, self.ack_at = {}, {}, {}
            self.steps = {a.model_name: 0 for a in config['actors']}
            self.distance = {a.model_name: 0. for a in config['actors']}
            self.required_models = {config['robot_name'], *self.steps}
            self.yields = {a.model_name: 0 for a in config['actors']}
            self.events = deque(maxlen=10000)
            self.started_at, self.last_write = time.monotonic(), -math.inf
            self.stopping, self.fault, self.ready = False, None, False
            self.minimum_robot_gap = math.inf
            self.minimum_observed_gap = math.inf
            self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
            self.create_subscription(ModelStates, '/gazebo/model_states', self.models, qos_profile_sensor_data)
            self.create_timer(1./config['rate'], self.tick,
                clock=RosClock(clock_type=ClockType.STEADY_TIME))

        def clock(self, message):
            value = message.clock.sec+message.clock.nanosec/1.e9
            if self.sim_time is not None and value < self.sim_time-1.e-9:
                self.fault = 'simulation_clock_rewound'
            if self.sim_time is not None and value > self.sim_time:
                self.clock_advances += 1
            self.sim_time = value

        def models(self, message):
            poses = {}
            for name, p in zip(message.name, message.pose):
                if name not in self.required_models:
                    continue
                q = p.orientation
                if all(math.isfinite(v) for v in (p.position.x, p.position.y, p.position.z,
                                                  q.x, q.y, q.z, q.w)):
                    poses[name] = Pose(p.position.x, p.position.y, p.position.z,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            self.truth, self.truth_at = poses, time.monotonic()
            robot = poses.get(config['robot_name'])
            if robot is not None:
                for actor in config['actors']:
                    pose = poses.get(actor.model_name)
                    if pose is not None:
                        gap = math.hypot(pose.x-robot.x, pose.y-robot.y)-actor.collision_radius_m-config['robot_radius']
                        self.minimum_observed_gap = min(self.minimum_observed_gap, gap)

        def record(self, **event):
            self.events.append({'wall_time': time.time(), 'sim_time': self.sim_time, **event})

        def report(self, force=False):
            if args.output is None or not force and time.monotonic()-self.last_write < 1.:
                return
            args.output.parent.mkdir(parents=True, exist_ok=True)
            result = {'schema_version': 1, 'domain': args.domain, 'localhost_only': True,
                'routes': config['source'], 'routes_sha256': config['sha256'],
                'status': 'FAILED' if self.fault else 'STOPPED' if self.stopping else 'RUNNING' if self.ready else 'WAITING',
                'fault': self.fault, 'sim_time': self.sim_time, 'pose_updates': self.steps,
                'translated_distance_m': self.distance,
                'yielded_updates': self.yields, 'owned_models': list(self.steps),
                'minimum_proposed_robot_circle_gap_m': self.minimum_robot_gap if math.isfinite(self.minimum_robot_gap) else None,
                'minimum_observed_robot_circle_gap_m': self.minimum_observed_gap if math.isfinite(self.minimum_observed_gap) else None,
                'robot_commands_published': 0, 'inflight_models': list(self.pending),
                'events': list(self.events), 'event_history_bounded': 10000}
            temporary = args.output.with_suffix(args.output.suffix+'.tmp')
            temporary.write_text(json.dumps(result, indent=2)+'\n')
            temporary.replace(args.output)
            self.last_write = time.monotonic()

        def tick(self):
            now = time.monotonic()
            if self.stopping or self.fault:
                return
            if any(now-item['at'] > .75 for item in self.pending.values()):
                self.fault = 'actor_service_timeout'
                return
            age = now-self.truth_at if self.truth_at is not None else math.inf
            if (self.sim_time is None or self.clock_advances < 1 or not self.client.service_is_ready()
                    or age >= .5 or not self.required_models <= self.truth.keys()):
                if not self.ready and now-self.started_at >= args.startup_timeout:
                    self.fault = 'clock_service_or_model_truth_not_ready'
                self.last_tick = self.sim_time
                self.report()
                return
            if not self.ready:
                for actor in config['actors']:
                    actual, first = self.truth[actor.model_name], actor.waypoints[0]
                    if math.dist((actual.x, actual.y, actual.z), (first.x, first.y, first.z)) > .025:
                        self.fault = 'actor_not_at_route_start:'+actor.model_name
                        return  # Never teleport a misplaced model onto a route.
                    self.motion[actor.model_name] = Motion(actual, dwell_remaining=actor.dwell_seconds)
                self.ready, self.last_tick = True, self.sim_time
                self.record(event='READY')
                self.get_logger().info('Warehouse actors ready: '+', '.join(self.motion))
                return
            dt = min(max(0., self.sim_time-self.last_tick), 1./config['rate'])
            self.last_tick = self.sim_time
            if dt <= 0.:
                return
            robot = self.truth[config['robot_name']]
            for actor in config['actors']:
                name = actor.model_name
                if name in self.pending:
                    if now-self.pending[name]['at'] > .75:
                        self.fault = 'actor_service_timeout:'+name
                    continue
                current = self.motion[name]
                actual = self.truth[name]
                if name in self.ack_at and self.truth_at <= self.ack_at[name]:
                    continue  # Wait for independent truth after service ACK.
                if math.dist((current.pose.x, current.pose.y), (actual.x, actual.y)) > .025:
                    self.fault = 'actor_pose_not_confirmed:'+name
                    break
                current = replace(current, pose=actual)
                proposal = propose(actor, current, dt, config['max_step'])
                if proposal.pose == current.pose:
                    self.motion[name] = proposal
                    continue
                others = [(self.truth[a.model_name], a.collision_radius_m, a.speed_mps)
                          for a in config['actors'] if a.model_name != name]
                if any(segment_box_distance((actual.x, actual.y), (proposal.pose.x, proposal.pose.y), box)
                       < actor.collision_radius_m+config['static_margin'] for box in config['walls']):
                    self.fault = 'actor_drift_enters_fixed_obstacle:'+name
                    break
                safe, reason, gap = movement_allowed(actor, actual, proposal.pose, robot, age,
                    robot_radius=config['robot_radius'], clearance=config['robot_clearance'], others=others)
                if gap is not None:
                    self.minimum_robot_gap = min(self.minimum_robot_gap, gap)
                if not safe:
                    self.yields[name] += 1
                    if self.yields[name] % 10 == 1:
                        self.record(event='YIELD', model=name, reason=reason, robot_body_gap_m=gap)
                    continue
                request = SetEntityState.Request()
                request.state.name, request.state.reference_frame = name, 'world'
                request.state.pose.position.x, request.state.pose.position.y = proposal.pose.x, proposal.pose.y
                request.state.pose.position.z = proposal.pose.z
                request.state.pose.orientation.z = math.sin(proposal.pose.yaw/2)
                request.state.pose.orientation.w = math.cos(proposal.pose.yaw/2)
                future = self.client.call_async(request)
                self.pending[name] = {'future': future, 'at': now, 'from_pose': actual}
                future.add_done_callback(lambda done, a=actor, step=proposal: self.finished(a, step, done))
            self.report()

        def finished(self, actor, proposal, future):
            previous = self.pending.pop(actor.model_name, None)
            try:
                result = future.result()
                if not result.success:
                    raise RuntimeError('Gazebo rejected actor pose')
                self.motion[actor.model_name] = proposal
                if previous is not None:
                    start = previous['from_pose']
                    self.distance[actor.model_name] += math.hypot(start.x-proposal.pose.x, start.y-proposal.pose.y)
                self.ack_at[actor.model_name] = time.monotonic()
                self.steps[actor.model_name] += 1
                self.record(event='POSE_APPLIED', model=actor.model_name,
                    x=proposal.pose.x, y=proposal.pose.y, yaw=proposal.pose.yaw)
            except Exception as exc:
                self.fault = 'actor_update_failed:'+actor.model_name+':'+str(exc)[:180]

    context = Context()
    rclpy.init(context=context, domain_id=args.domain)
    driver = Driver(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(driver)
    stopped = [False]
    old_handlers = {sig: signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        while context.ok() and not stopped[0] and driver.fault is None:
            executor.spin_once(timeout_sec=.05)
        driver.stopping = True
        # Drain already-issued bounded steps only; never reset/relocate models.
        until = time.monotonic()+.5
        while driver.pending and time.monotonic() < until and context.ok():
            executor.spin_once(timeout_sec=.02)
        driver.report(force=True)
        return 1 if driver.fault else 0
    finally:
        executor.shutdown()
        driver.destroy_node()
        context.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
