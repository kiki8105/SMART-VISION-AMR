#!/usr/bin/env python3
"""Bound reaction time and stopping distance when a planned route is invalidated."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


# Waffle Pi SDF: laser x=-0.064 m; forward body edge x≈+0.068 m.
FRONT_LASER_TO_BODY_M = .132
MIN_BODY_CLEARANCE_M = .02
COMMAND_STOP_BOUND_S = .25
STOP_DISTANCE_BOUND_M = .05
STOP_HEADING_BOUND_RAD = .12
# The observer samples Gazebo truth at about 10 Hz.  A WAIT entry must be
# bracketed by fresh samples; an old pose must never inflate or hide travel.
TRUTH_ENTRY_BRACKET_MAX_S = .15
# Defaults match the repository's curve_tuning_waffle_pi.yaml.  A staged run
# with another Nav2 profile must pass that profile's velocity and deceleration.
DEFAULT_MAX_LINEAR_SPEED_MPS = .12
DEFAULT_DECEL_LIMIT_MPS2 = 2.5
# The outermost collision on the fixture's Waffle Pi is the chassis rear
# corner: hypot(.064+.265/2, .265/2)=.237 m.  A .24 m circle also encloses
# its wheels/casters.  This is only a Gazebo witness for a known static world.
SIM_ROBOT_COLLISION_RADIUS_M = .24
SIM_STATIC_SWEEP_MARGIN_M = .05
SIM_LASER_X_M = -.064


def _pose(text):
    values = [float(value) for value in text.split()] if text else []
    return (values+[0.]*6)[:6]


def _compose(parent, child):
    c, s = math.cos(parent[5]), math.sin(parent[5])
    return [parent[0]+c*child[0]-s*child[1],
            parent[1]+s*child[0]+c*child[1], parent[2]+child[2],
            0., 0., parent[5]+child[5]]


def load_static_world_boxes(world_file):
    """Read Gazebo collision boxes, excluding the floor and visual-only tags."""
    boxes = []
    root = ET.parse(world_file).getroot()
    for model in root.findall('./world/model'):
        if model.findtext('static', 'false').strip().lower() != 'true':
            continue
        model_pose = _pose(model.findtext('pose'))
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose(link.findtext('pose')))
            for collision in link.findall('collision'):
                size_text = collision.findtext('geometry/box/size')
                if not size_text:
                    continue
                sx, sy, sz = (float(value) for value in size_text.split())
                location = _compose(link_pose, _pose(collision.findtext('pose')))
                # A floor-level robot only collides with obstacles crossing
                # its vertical envelope; marker sheets without collision do
                # not influence this witness.
                if location[2]-sz/2. > .3 or location[2]+sz/2. < 0.:
                    continue
                c, s = math.cos(location[5]), math.sin(location[5])
                polygon = [(location[0]+c*x-s*y, location[1]+s*x+c*y)
                           for x, y in ((-sx/2., -sy/2.), (sx/2., -sy/2.),
                                        (sx/2., sy/2.), (-sx/2., sy/2.))]
                boxes.append((model.attrib.get('name', '')+'/'+link.attrib.get('name', ''),
                              polygon))
    return boxes


def _point_segment_distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    along = (((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2
             if length2 else 0.)
    along = max(0., min(1., along))
    return math.hypot(point[0]-start[0]-along*dx,
                      point[1]-start[1]-along*dy)


def _point_polygon_distance(point, polygon):
    # Generated box vertices are counter-clockwise, including rotated boxes.
    inside = True
    for index, start in enumerate(polygon):
        end = polygon[(index+1) % len(polygon)]
        cross = ((end[0]-start[0])*(point[1]-start[1])-
                 (end[1]-start[1])*(point[0]-start[0]))
        if cross < -1.e-10:
            inside = False
            break
    if inside:
        return 0.
    return min(_point_segment_distance(point, polygon[index],
                                      polygon[(index+1) % len(polygon)])
               for index in range(len(polygon)))


def _ray_polygon_distance(origin, direction, polygon):
    nearest = math.inf
    for index, start in enumerate(polygon):
        end = polygon[(index+1) % len(polygon)]
        vx, vy = end[0]-start[0], end[1]-start[1]
        wx, wy = start[0]-origin[0], start[1]-origin[1]
        denominator = direction[0]*vy-direction[1]*vx
        if abs(denominator) < 1.e-10:
            continue
        distance = (wx*vy-wy*vx)/denominator
        edge_fraction = (wx*direction[1]-wy*direction[0])/denominator
        if distance >= 0. and 0. <= edge_fraction <= 1.:
            nearest = min(nearest, distance)
    return nearest


def static_sweep_clearance(truth_samples, boxes, *, robot_radius=SIM_ROBOT_COLLISION_RADIUS_M):
    """Minimum center-to-box distance minus an enclosing robot radius."""
    if not truth_samples or not boxes:
        return None
    return min(_point_polygon_distance((pose['x'], pose['y']), polygon)-
               robot_radius
               for pose in truth_samples for _, polygon in boxes)


def _isolated_static_outlier(entry_time, scan_records, truth_records, boxes):
    """Prove one minimum-range scan conflicts with a known static fixture.

    This is *never* used for a moving-obstacle or physical-robot trial.  A
    repeated close return, missing adjacent samples, or unknown geometry
    remains a failure instead of being dismissed as noise.
    """
    if not scan_records or not truth_records or not boxes:
        return None
    nearby = [scan for scan in scan_records if
              entry_time-.5 <= scan['t'] <= entry_time+.3 and
              scan.get('front_core_min_m') is not None]
    close = [scan for scan in nearby if scan['front_core_min_m'] < .18]
    if len(close) != 1:
        return None
    outlier = close[0]
    prior = [scan for scan in nearby if scan['t'] < outlier['t']]
    following = [scan for scan in nearby if scan['t'] > outlier['t']]
    if not prior or not following:
        return None
    before, after = prior[-1], following[0]
    if (outlier['t']-before['t'] > .26 or after['t']-outlier['t'] > .26 or
            before['front_core_min_m'] < .35 or after['front_core_min_m'] < .35):
        return None
    pose = min(truth_records, key=lambda item: abs(item['t']-outlier['t']))
    if abs(pose['t']-outlier['t']) > .08:
        return None
    bearing = outlier.get('front_core_angle_deg')
    if bearing is None:
        return None
    yaw = pose['yaw']
    origin = (pose['x']+SIM_LASER_X_M*math.cos(yaw),
              pose['y']+SIM_LASER_X_M*math.sin(yaw))
    direction = (math.cos(yaw+math.radians(bearing)),
                 math.sin(yaw+math.radians(bearing)))
    expected = min(_ray_polygon_distance(origin, direction, polygon)
                   for _, polygon in boxes)
    if not math.isfinite(expected) or expected < .5 or expected-outlier[
            'front_core_min_m'] < .35:
        return None
    return {'scan_t': round(outlier['t'], 4),
            'observed_m': round(outlier['front_core_min_m'], 4),
            'bearing_deg': round(bearing, 2),
            'static_world_expected_m': round(expected, 4),
            'prior_min_m': round(before['front_core_min_m'], 4),
            'next_min_m': round(after['front_core_min_m'], 4)}


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def truth_at_entry(truth_records, entry_time):
    """Interpolate a WAIT onset only when both adjacent truth samples are fresh."""
    before = next((pose for pose in reversed(truth_records)
                   if pose['t'] <= entry_time), None)
    after = next((pose for pose in truth_records
                  if pose['t'] >= entry_time), None)
    if (before is None or after is None or
            entry_time-before['t'] > TRUTH_ENTRY_BRACKET_MAX_S or
            after['t']-entry_time > TRUTH_ENTRY_BRACKET_MAX_S):
        return None, None
    dt = after['t']-before['t']
    if dt <= 0.:
        return before, 0.
    fraction = (entry_time-before['t'])/dt
    anchor = {'t': entry_time,
              'x': before['x']+(after['x']-before['x'])*fraction,
              'y': before['y']+(after['y']-before['y'])*fraction,
              'yaw': before['yaw']+wrapped(after['yaw']-before['yaw'])*fraction}
    observed_speed = math.hypot(after['x']-before['x'],
                                after['y']-before['y'])/dt
    return anchor, observed_speed


def stopping_distance_bound(entry_speed_mps, *, max_linear_speed_mps,
                            decel_limit_mps2):
    """Reaction travel plus configured braking distance, with a 5 cm floor."""
    speed = min(max_linear_speed_mps, max(0., entry_speed_mps))
    return max(STOP_DISTANCE_BOUND_M,
               speed*COMMAND_STOP_BOUND_S+speed*speed/(2.*decel_limit_mps2))


def _stamp_ns(value):
    if not isinstance(value, dict):
        return None
    sec, ns = value.get('sec'), value.get('nanosec')
    if type(sec) is int and type(ns) is int and sec >= 0 and 0 <= ns < 1_000_000_000:
        return sec*1_000_000_000+ns or None
    return None


def _navigation_context(record):
    return (_stamp_ns(record.get('goal_stamp')),
            _stamp_ns((record.get('details') or {}).get('path_stamp')))


def _owned_motion_boundary(record, owner, current_goal, current_path, recovery,
                           recovery_episode, recovery_phase):
    """Recorded action ownership, not a generic MOVING/RECOVERING label."""
    goal, path = _navigation_context(record)
    if goal is None or goal != current_goal or (owner is not None and goal != owner['goal']):
        return None
    details = record.get('details') or {}
    if record.get('status') == 'MOVING' and owner is not None:
        if path is None or path != current_path:
            return None
        if (path > owner['path'] or (owner['wait_stop'] and path == owner['path'])):
            if details.get('heading_alignment_phase') == 'SPIN':
                return 'spin'
            if record.get('message') == 'Following A* route':
                return 'follow'
    if (record.get('status') == 'RECOVERING'
            and record.get('message') == 'Nav2 collision-checked reverse started'
            and isinstance(recovery, dict) and recovery.get('cancel') is not True
            and recovery.get('goal_id') == f'{goal//1_000_000_000}:{goal%1_000_000_000}'
            and isinstance(details.get('episode_id'), str)
            and details['episode_id'] and details['episode_id'] == recovery.get('episode_id')
            and details['episode_id'] == recovery_episode and recovery_phase == 'REVERSING'
            and (owner is None or _stamp_ns(recovery.get('source_path_stamp')) == owner['path'])):
        return 'reverse'
    return None


def assess(records, *, static_world_boxes=None,
           max_linear_speed_mps=DEFAULT_MAX_LINEAR_SPEED_MPS,
           decel_limit_mps2=DEFAULT_DECEL_LIMIT_MPS2,
           robot_collision_radius_m=SIM_ROBOT_COLLISION_RADIUS_M):
    if max_linear_speed_mps <= 0. or decel_limit_mps2 <= 0.:
        raise ValueError('Nav2 max speed and deceleration must be positive')
    if not math.isfinite(robot_collision_radius_m) or robot_collision_radius_m <= 0.:
        raise ValueError('collision radius must be finite and positive')
    # Only a caller that has proved the trial contains no spawned/dynamic
    # obstacle may pass static_world_boxes.  Otherwise raw LiDAR limits apply.
    static_world_boxes = static_world_boxes or []
    all_scans = [item for item in records if item.get('kind') == 'scan']
    all_truth = [item for item in records if item.get('kind') == 'truth']
    planner = None
    planner_since = None
    navigation = None
    navigation_at = None
    navigation_stop_owner = None
    checked_reverse_active = False
    checked_spin_active = False
    recovery_request = None
    recovery_episode = recovery_phase = None
    current_path_stamp = None
    current_goal_stamp = None
    unverified_stop_events = []
    path_count = 0
    scan = None
    truth = None
    waiting = None
    waiting_intervals = []
    static_world_outliers = []
    recovery_transitions = []
    counts = {'planner': 0, 'path': 0, 'scan': 0, 'truth': 0, 'cmd_vel': 0,
              'planning_active_front_clear': 0,
              'planning_route_unverified': 0,
              'wait_replan_samples': 0, 'recovery_exempt_samples': 0}
    violations = []
    last_planner_change = None
    last_linear_command = 0.
    last_command_t = None

    def violation(name, record, **detail):
        if len(violations) < 100:
            violations.append({'kind': name, 't': record['t'], **detail})

    def begin_wait(t, source='planner_wait_replan'):
        entry_truth, observed_speed = truth_at_entry(all_truth, t)
        commanded_speed = (abs(last_linear_command)
                           if last_command_t is not None and
                           t-last_command_t <= COMMAND_STOP_BOUND_S else 0.)
        entry_speed = max(commanded_speed, observed_speed or 0.)
        entry_core = (scan.get('front_core_min_m') if scan is not None
                      and t-scan['t'] <= .35 else None)
        entry_angle = (scan.get('front_core_angle_deg') if scan is not None
                       and t-scan['t'] <= .35 else None)
        return {'start_t': t, 'start_truth': entry_truth, 'stop_sources': [source],
                'entry_speed_mps': entry_speed,
                'entry_command_speed_mps': commanded_speed,
                'entry_truth_speed_mps': observed_speed,
                'entry_core_m': entry_core,
                'entry_core_angle_deg': entry_angle,
                'max_displacement_m': 0., 'max_forward_m': 0.,
                'max_heading_rad': 0., 'truth_samples': 0,
                'last_nonzero_command_t': None,
                'first_zero_command_t': None}

    def finish_wait(end_t):
        nonlocal waiting
        if waiting is None:
            return
        duration = max(0., end_t-waiting['start_t'])
        stopping_time = waiting['last_nonzero_command_t']
        stopping_time = (None if stopping_time is None else
                         max(0., stopping_time-waiting['start_t']))
        zero_time = waiting['first_zero_command_t']
        zero_time = (None if zero_time is None else
                     max(0., zero_time-waiting['start_t']))
        interval = {'start_t': waiting['start_t'], 'duration_s': round(duration, 4),
                    'stop_sources': waiting['stop_sources'],
                    'last_nonzero_command_after_s': stopping_time,
                    'first_zero_command_after_s': zero_time,
                    'max_displacement_m': round(waiting['max_displacement_m'], 4),
                    'max_forward_displacement_m': round(waiting['max_forward_m'], 4),
                    'max_heading_change_rad': round(waiting['max_heading_rad'], 4),
                    'truth_samples': waiting['truth_samples'],
                    'entry_speed_mps': round(waiting['entry_speed_mps'], 4),
                    'entry_command_speed_mps': round(waiting['entry_command_speed_mps'], 4),
                    'entry_truth_speed_mps': (None if waiting['entry_truth_speed_mps'] is None
                                              else round(waiting['entry_truth_speed_mps'], 4)),
                    'entry_front_core_min_m': waiting['entry_core_m'],
                    'entry_front_core_angle_deg': waiting['entry_core_angle_deg']}
        distance_bound = stopping_distance_bound(
            waiting['entry_speed_mps'],
            max_linear_speed_mps=max_linear_speed_mps,
            decel_limit_mps2=decel_limit_mps2)
        interval['ground_truth_stopping_distance_bound_m'] = round(distance_bound, 4)
        sweep_truth = [pose for pose in all_truth if
                       waiting['start_t']-.15 <= pose['t'] <= end_t]
        static_margin = static_sweep_clearance(sweep_truth, static_world_boxes,
                                               robot_radius=robot_collision_radius_m)
        if static_margin is not None:
            interval['static_world_swept_clearance_m'] = round(static_margin, 4)
            if static_margin < SIM_STATIC_SWEEP_MARGIN_M:
                violation('insufficient_static_world_swept_clearance',
                          {'t': end_t}, **interval)
        core = waiting['entry_core_m']
        if core is None and waiting['max_forward_m'] > .005:
            violation('front_scan_missing_at_wait_replan', {'t': end_t}, **interval)
        if core is not None:
            angle = waiting['entry_core_angle_deg'] or 0.
            projected = core*math.cos(math.radians(angle))
            margin = projected-FRONT_LASER_TO_BODY_M-waiting['max_forward_m']
            interval['estimated_front_body_clearance_m'] = round(margin, 4)
            if waiting['max_forward_m'] > .005 and margin < MIN_BODY_CLEARANCE_M:
                isolated = (_isolated_static_outlier(waiting['start_t'],
                            all_scans, all_truth, static_world_boxes)
                            if static_margin is not None and
                            static_margin >= SIM_STATIC_SWEEP_MARGIN_M else None)
                if isolated is None:
                    violation('insufficient_front_stopping_clearance',
                              {'t': end_t}, **interval)
                else:
                    interval['isolated_static_world_scan_outlier'] = isolated
                    static_world_outliers.append(isolated)
        if stopping_time is not None and stopping_time > COMMAND_STOP_BOUND_S:
            violation('late_nonzero_command_after_wait_replan',
                      {'t': end_t}, **interval)
        if duration >= COMMAND_STOP_BOUND_S and (zero_time is None or
                                                  zero_time > COMMAND_STOP_BOUND_S):
            violation('zero_command_not_observed_in_time', {'t': end_t}, **interval)
        if duration >= COMMAND_STOP_BOUND_S and waiting['truth_samples'] < 1:
            violation('ground_truth_missing_during_wait_replan',
                      {'t': end_t}, **interval)
        if ((duration >= COMMAND_STOP_BOUND_S or waiting['truth_samples'] > 0)
                and waiting['start_truth'] is None):
            violation('ground_truth_missing_at_wait_replan_entry',
                      {'t': end_t}, **interval)
        if waiting['max_displacement_m'] > distance_bound:
            violation('excessive_ground_truth_stopping_distance',
                      {'t': end_t}, **interval)
        if waiting['max_heading_rad'] > STOP_HEADING_BOUND_RAD:
            if static_margin is None or static_margin < SIM_STATIC_SWEEP_MARGIN_M:
                violation('excessive_ground_truth_stopping_rotation',
                          {'t': end_t}, **interval)
            else:
                interval['rotation_verified_by_static_world_sweep'] = True
        waiting_intervals.append(interval)
        waiting = None

    last_time = 0.
    for record in records:
        kind = record.get('kind')
        t = record.get('t')
        if not isinstance(t, (int, float)):
            continue
        last_time = t
        if kind in counts:
            counts[kind] += 1
        if kind == 'planner':
            goal_stamp = _stamp_ns(record.get('goal_stamp'))
            old_context = (current_goal_stamp, recovery_episode, recovery_phase)
            if goal_stamp is not None:
                current_goal_stamp = max(current_goal_stamp or 0, goal_stamp)
            detail = record.get('details') or {}
            recovery_episode, recovery_phase = (detail.get('recovery_episode_id'),
                                                detail.get('recovery_phase'))
            if old_context != (current_goal_stamp, recovery_episode, recovery_phase):
                checked_reverse_active = checked_spin_active = False
            next_status = record.get('status')
            if next_status != planner:
                if (waiting is not None and navigation_stop_owner is None
                        and next_status not in {'WAIT_REPLAN', 'RECOVERING'}):
                    finish_wait(t)
                planner = next_status
                planner_since = t
                last_planner_change = {'t': t, 'status': planner}
                if planner == 'WAIT_REPLAN':
                    if waiting is None:
                        waiting = begin_wait(t)
                    elif 'planner_wait_replan' not in waiting['stop_sources']:
                        waiting['stop_sources'].append('planner_wait_replan')
                elif planner == 'RECOVERING':
                    recovery_transitions.append({'t': t, 'source': 'planner'})
        elif kind == 'navigation':
            previous_navigation = navigation
            navigation = record.get('status')
            navigation_at = t
            details = record.get('details') or {}
            if navigation in {'WAIT_CANCEL', 'WAIT_STOP'} or details.get('heading_alignment_phase') == 'WAIT_STOP':
                goal, path = _navigation_context(record)
                if goal is None or path is None or current_goal_stamp is None:
                    unverified_stop_events.append({'t': t, 'reason': 'cancel_wait_context_missing'})
                elif goal == current_goal_stamp:
                    # A later wait can update its target, never refund the
                    # original reaction/deceleration clock with a new anchor.
                    if (navigation_stop_owner is None or goal != navigation_stop_owner['goal'] or
                            (goal == navigation_stop_owner['goal'] and path >= navigation_stop_owner['path'])):
                        navigation_stop_owner = {'goal': goal, 'path': path,
                            'wait_stop': details.get('heading_alignment_phase') == 'WAIT_STOP'}
                    if waiting is None:
                        waiting = begin_wait(t, 'navigation_cancel_wait')
                    elif 'navigation_cancel_wait' not in waiting['stop_sources']:
                        waiting['stop_sources'].append('navigation_cancel_wait')
            boundary = _owned_motion_boundary(record, navigation_stop_owner,
                current_goal_stamp, current_path_stamp, recovery_request,
                recovery_episode, recovery_phase)
            if boundary is not None:
                navigation_stop_owner = None
                if waiting is not None and (boundary == 'spin' or
                        (boundary == 'follow' and planner != 'WAIT_REPLAN')):
                    finish_wait(t)
            if boundary == 'spin':
                checked_spin_active = True
            elif navigation != 'MOVING' or details.get('heading_alignment_phase') != 'SPIN':
                checked_spin_active = False
            elif _navigation_context(record) != (current_goal_stamp, current_path_stamp):
                checked_spin_active = False
            if navigation != 'RECOVERING':
                checked_reverse_active = False
            elif boundary == 'reverse':
                checked_reverse_active = True
            else:
                checked_reverse_active = False
                if (record.get('message') == 'Nav2 collision-checked reverse started'
                        and (not isinstance(recovery_request, dict) or not details.get('episode_id')
                             or not recovery_episode or recovery_phase != 'REVERSING')):
                    unverified_stop_events.append({'t': t, 'reason': 'reverse_start_ownership_unverified'})
            if boundary == 'reverse' and waiting is not None:
                # A checked reverse is a distinct phase. Do not let a later
                # recovery erase unsafe motion during the preceding hold.
                finish_wait(t)
                recovery_transitions.append({'t': t, 'source': 'navigation'})
            elif (previous_navigation == 'RECOVERING' and
                  navigation != 'RECOVERING' and planner == 'WAIT_REPLAN' and
                  waiting is None):
                waiting = begin_wait(t)
        elif kind == 'recovery_request':
            recovery_request = record.get('payload')
        elif kind == 'goal':
            stamp = record.get('stamp_ns')
            if type(stamp) is int and stamp > 0:
                if stamp != current_goal_stamp:
                    checked_reverse_active = checked_spin_active = False
                current_goal_stamp = max(current_goal_stamp or 0, stamp)
        elif kind == 'path':
            path_count = max(0, int(record.get('count') or 0))
            if not path_count or record.get('stamp_ns') != current_path_stamp:
                checked_reverse_active = checked_spin_active = False
            current_path_stamp = record.get('stamp_ns') if path_count else None
        elif kind == 'scan':
            scan = record
            if (waiting is not None and waiting['entry_core_m'] is None
                    and t-waiting['start_t'] <= .2):
                waiting['entry_core_m'] = record.get('front_core_min_m')
                waiting['entry_core_angle_deg'] = record.get('front_core_angle_deg')
        elif kind == 'truth':
            truth = record
            if waiting is not None:
                waiting['truth_samples'] += 1
                anchor = waiting['start_truth']
                if anchor is None:
                    continue
                dx = record['x']-anchor['x']
                dy = record['y']-anchor['y']
                waiting['max_displacement_m'] = max(
                    waiting['max_displacement_m'], math.hypot(dx, dy))
                waiting['max_forward_m'] = max(
                    waiting['max_forward_m'],
                    dx*math.cos(anchor['yaw'])+dy*math.sin(anchor['yaw']))
                waiting['max_heading_rad'] = max(
                    waiting['max_heading_rad'],
                    abs(wrapped(record['yaw']-anchor['yaw'])))
        elif kind == 'cmd_vel':
            linear = float(record.get('linear') or 0.)
            angular = float(record.get('angular') or 0.)
            last_linear_command = linear
            last_command_t = t
            nonzero = abs(linear) > .015 or abs(angular) > .04
            recovering = (checked_reverse_active and navigation_at is not None
                          and t-navigation_at <= 1.)
            if recovering:
                counts['recovery_exempt_samples'] += 1
            if waiting is not None:
                counts['wait_replan_samples'] += 1
                if nonzero:
                    waiting['last_nonzero_command_t'] = t
                elif waiting['first_zero_command_t'] is None:
                    waiting['first_zero_command_t'] = t
            if planner == 'PLANNING':
                fresh_scan = scan is not None and t-scan['t'] <= .5
                front = scan.get('front_min_m') if fresh_scan else None
                if path_count > 0 and front is not None and front >= .35:
                    counts['planning_active_front_clear'] += 1
                else:
                    counts['planning_route_unverified'] += 1
                    if (planner_since is not None and
                            t-planner_since > COMMAND_STOP_BOUND_S and
                            nonzero and not recovering and not checked_spin_active):
                        violation('motion_during_unverified_planning', record,
                                  path_count=path_count, front_min_m=front)
    finish_wait(last_time)

    missing = [name for name in ('planner', 'path', 'scan', 'truth', 'cmd_vel')
               if counts[name] < 1]
    if not any(record.get('kind') == 'path' and record.get('count', 0) > 0
               for record in records):
        missing.append('nonempty_path')
    if unverified_stop_events:
        missing.append('navigation_stop_ownership_unverified')
    return {'passed': not missing and not violations,
            'missing_evidence': missing, 'violations': violations,
            'counts': counts, 'wait_replan_intervals': waiting_intervals,
            'unverified_navigation_stop_events': unverified_stop_events,
            'static_world_outliers': static_world_outliers,
            'recovery_transitions': recovery_transitions,
            'last_planner_change': last_planner_change,
            'bounds': {'command_stop_s': COMMAND_STOP_BOUND_S,
                       'ground_truth_stopping_distance_m': STOP_DISTANCE_BOUND_M,
                       'ground_truth_stopping_distance_rule':
                           'max(0.05, v*command_stop_s+v^2/(2*decel_limit_mps2))',
                       'max_linear_speed_mps': max_linear_speed_mps,
                       'decel_limit_mps2': decel_limit_mps2,
                       'truth_entry_bracket_max_s': TRUTH_ENTRY_BRACKET_MAX_S,
                       'ground_truth_heading_change_rad': STOP_HEADING_BOUND_RAD,
                       'front_laser_to_body_m': FRONT_LASER_TO_BODY_M,
                       'front_body_clearance_m': MIN_BODY_CLEARANCE_M,
                       'simulation_robot_collision_radius_m': robot_collision_radius_m,
                       'simulation_static_swept_clearance_m': SIM_STATIC_SWEEP_MARGIN_M},
            'interpretation': ('PLANNING with a nonempty prior path and a fresh, clear '
                               'front scan may move while replanning; this is not a full '
                               'swept-path proof. Planner WAIT_REPLAN and owned navigation '
                               'WAIT_CANCEL/WAIT_STOP permit bounded deceleration '
                               'for 0.25 s and speed-dependent ground-truth travel '
                               'based on the active Nav2 speed/deceleration profile. '
                               'Navigation RECOVERING is logged separately. No instant '
                               'command or inertial motion is automatically a failure. '
                               'Only a known static Gazebo fixture can use the independent '
                               'collision-geometry witness; it does not certify real-world '
                               'or moving-obstacle safety.')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--static-world', type=Path,
                        help='Known-static Gazebo collision world; never use for physical or moving-obstacle trials')
    parser.add_argument('--max-linear-speed-mps', type=float,
                        default=DEFAULT_MAX_LINEAR_SPEED_MPS)
    parser.add_argument('--decel-limit-mps2', type=float,
                        default=DEFAULT_DECEL_LIMIT_MPS2)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.timeline.read_text().splitlines()
               if line.strip()]
    boxes = load_static_world_boxes(args.static_world) if args.static_world else None
    result = assess(records, static_world_boxes=boxes,
                    max_linear_speed_mps=args.max_linear_speed_mps,
                    decel_limit_mps2=args.decel_limit_mps2)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'passed': result['passed'],
                      'missing_evidence': result['missing_evidence'],
                      'violations': len(result['violations'])}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
