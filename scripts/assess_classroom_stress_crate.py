#!/usr/bin/env python3
"""Assess fixed-crate bypass from raw LiDAR, mapped cells, path, and truth.

The web mission verifier remains responsible for marker alignment and dwell.
This additional witness refuses to infer obstacle avoidance from a route hash,
a planner status label, or the source world XML alone.
"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from capture_classroom_stress_crate import (CRATE_CENTER, CRATE_HALF_SIZE,
                                            CRATE_NAME)


BODY_RADIUS_M = .24
REQUIRED_BODY_MARGIN_M = .02
MIN_ROUTE_TO_BOX_M = BODY_RADIUS_M+REQUIRED_BODY_MARGIN_M
# A mission that stays well away from the fixed crate must not be reported as
# a crate bypass merely because its LiDAR saw the crate from afar.
MAX_CROSSING_LEG_BODY_CLEARANCE_M = .40
MAX_LINK_AGE_S = .30
MAX_LINK_BRACKET_S = .75
MIN_DIRECT_LINK_FRACTION = .995
MAX_TRUTH_GAP_S = .75
CRATE_POSE_TOLERANCE_M = .005
# motion_mux clamps both forward and reverse speed to 0.25 m/s. Between two
# truth samples the robot can move away from, then return to, the same point.
MAX_ENVELOPE_LINEAR_MPS = .25


def positive_robot_radius(value):
    if isinstance(value, bool):
        raise ValueError('robot radius must be numeric, not boolean')
    radius = float(value)
    if not math.isfinite(radius) or radius <= 0.:
        raise ValueError('robot radius must be finite and positive')
    return radius


def verified_runtime_robot_radius(manifest_path):
    """Use the declared body only after checking its copied collision proxy.

    Stock Gazebo retains the existing enclosing radius. A physical proxy must
    agree across both manifest declarations and the actual centered SDF body;
    a missing/mismatched fixture fails closed instead of choosing a smaller body.
    """
    manifest_path = Path(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    radius = positive_robot_radius(
        manifest['sim_footprint_overrides']['robot_radius_m'])
    if manifest.get('physical_footprint') is True:
        declared = positive_robot_radius(
            manifest['collision_model']['copied_centered_cylinder_radius_m'])
        root = ET.parse(manifest_path.parent/'fixture/waffle_pi_fault.sdf').getroot()
        body = next((node for node in root.iter('collision')
                     if node.get('name') == 'base_collision'), None)
        if body is None:
            raise ValueError('copied SDF base collision is missing')
        actual = positive_robot_radius(body.findtext('geometry/cylinder/radius'))
        pose = [float(value) for value in (body.findtext('pose') or '').split()]
        if (declared != radius or actual != radius or len(pose) != 6 or
                not all(math.isfinite(value) for value in pose) or
                any(abs(pose[index]) > 1.e-12 for index in (0, 1, 3, 4))):
            raise ValueError('physical body radius/center disagrees with manifest or copied SDF')
    elif manifest.get('physical_footprint') is not False or radius != BODY_RADIUS_M:
        raise ValueError('stock enclosing radius or physical-footprint declaration is unverified')
    return radius


def point_rectangle_distance(point, center=CRATE_CENTER, half_size=CRATE_HALF_SIZE):
    return math.hypot(max(abs(point[0]-center[0])-half_size[0], 0.),
                      max(abs(point[1]-center[1])-half_size[1], 0.))


def point_segment_distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    fraction = (max(0., min(1., ((point[0]-start[0])*dx+
                              (point[1]-start[1])*dy)/length2))
                if length2 else 0.)
    return math.hypot(point[0]-start[0]-fraction*dx,
                      point[1]-start[1]-fraction*dy)


def segment_intersects_rectangle(start, end, center=CRATE_CENTER,
                                 half_size=CRATE_HALF_SIZE):
    """Liang-Barsky clipping; edge contact counts as an intersection."""
    left, right = center[0]-half_size[0], center[0]+half_size[0]
    bottom, top = center[1]-half_size[1], center[1]+half_size[1]
    dx, dy = end[0]-start[0], end[1]-start[1]
    lower, upper = 0., 1.
    for p, q in ((-dx, start[0]-left), (dx, right-start[0]),
                 (-dy, start[1]-bottom), (dy, top-start[1])):
        if abs(p) < 1.e-12:
            if q < 0:
                return False
            continue
        fraction = q/p
        if p < 0:
            lower = max(lower, fraction)
        else:
            upper = min(upper, fraction)
        if lower > upper:
            return False
    return True


def segment_rectangle_distance(start, end, center=CRATE_CENTER,
                               half_size=CRATE_HALF_SIZE):
    if segment_intersects_rectangle(start, end, center, half_size):
        return 0.
    corners = [(center[0]+sign_x*half_size[0],
                center[1]+sign_y*half_size[1])
               for sign_x in (-1., 1.) for sign_y in (-1., 1.)]
    return min(point_rectangle_distance(start, center, half_size),
               point_rectangle_distance(end, center, half_size),
               *(point_segment_distance(corner, start, end)
                 for corner in corners))


def path_rectangle_clearance(points, center=CRATE_CENTER, *, robot_radius=BODY_RADIUS_M):
    """Exact segment-to-rectangle distance minus an enclosing robot radius."""
    if not points:
        return None
    if len(points) == 1:
        return point_rectangle_distance(points[0], center)-robot_radius
    return min(segment_rectangle_distance(start, end, center)-robot_radius
               for start, end in zip(points, points[1:]))


def _fixed_link_pose(record):
    """Return a *fresh* link pose only when it agrees with the known fixture."""
    age, pose = record.get('crate_link_age_s'), record.get('crate')
    if (not isinstance(age, (int, float)) or not 0. <= age <= MAX_LINK_AGE_S or
            not isinstance(pose, list) or len(pose) < 3 or
            not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in pose[:3]) or
            math.hypot(pose[0]-CRATE_CENTER[0], pose[1]-CRATE_CENTER[1]) >
            CRATE_POSE_TOLERANCE_M or abs(pose[2]) > .02):
        return None
    return tuple(pose[:3])


def resolve_fixed_crate_truth(truth):
    """Bridge rare link-callback gaps only between matching, fresh poses.

    Gazebo ModelStates must still show the crate in *every* robot sample.
    A fresh link at an unexpected pose, missing model, unbracketed gap, or
    insufficient directly observed fraction fails closed.
    """
    n = len(truth)
    direct = [_fixed_link_pose(record) for record in truth]
    valid_count = sum(pose is not None for pose in direct)
    metrics = {'mission_truth_samples': n,
               'fixed_crate_truth_samples': valid_count,
               'direct_link_fraction': round(valid_count/n, 6) if n else 0.,
               'interpolated_link_samples': 0,
               'max_link_bracket_s': None,
               'crate_model_present_every_sample': bool(n and all(
                   record.get('crate_model_present') is True for record in truth))}
    enough = (n >= 20 and valid_count >= 20 and
              valid_count/n >= MIN_DIRECT_LINK_FRACTION)
    if not metrics['crate_model_present_every_sample'] or not enough:
        return None, metrics
    # A recent but displaced/invalid link is a different physical object,
    # never a callback gap that can be safely interpolated.
    for record, pose in zip(truth, direct):
        age = record.get('crate_link_age_s')
        if pose is None and (record.get('crate') is not None or
                             not isinstance(age, (int, float)) or
                             not math.isfinite(age) or age <= MAX_LINK_AGE_S):
            return None, metrics
    resolved = direct[:]
    brackets = []
    for i, pose in enumerate(direct):
        if pose is not None:
            continue
        left = next((j for j in range(i-1, -1, -1)
                     if direct[j] is not None), None)
        right = next((j for j in range(i+1, n)
                      if direct[j] is not None), None)
        if left is None or right is None:
            return None, metrics
        start, end = truth[left]['at'], truth[right]['at']
        span = end-start
        if (not 0. < span <= MAX_LINK_BRACKET_S or
                math.hypot(direct[left][0]-direct[right][0],
                           direct[left][1]-direct[right][1]) >
                CRATE_POSE_TOLERANCE_M or
                abs(direct[left][2]-direct[right][2]) > .02):
            return None, metrics
        fraction = (truth[i]['at']-start)/span
        if not 0. <= fraction <= 1.:
            return None, metrics
        resolved[i] = tuple(a+fraction*(b-a)
                            for a, b in zip(direct[left], direct[right]))
        brackets.append(span)
    metrics['interpolated_link_samples'] = len(brackets)
    metrics['max_link_bracket_s'] = round(max(brackets), 4) if brackets else 0.
    return resolved, metrics


def conservative_truth_body_clearance(truth, *, robot_radius=BODY_RADIUS_M):
    """Minimum clearance with bounded travel between robot-truth samples."""
    if not truth:
        return None
    half = tuple(value+CRATE_POSE_TOLERANCE_M for value in CRATE_HALF_SIZE)
    if len(truth) == 1:
        return point_rectangle_distance(truth[0]['robot'], CRATE_CENTER,
                                        half)-robot_radius
    values = []
    for before, after in zip(truth, truth[1:]):
        gap = after['at']-before['at']
        if not 0. < gap <= MAX_TRUTH_GAP_S:
            return None
        # A trajectory of length <= v*gap cannot deviate from its endpoint
        # chord by more than v*gap/2: every point is within that distance of
        # at least one endpoint. This also covers reverse/out-and-back motion.
        excursion_reserve = MAX_ENVELOPE_LINEAR_MPS*gap/2.
        values.append(segment_rectangle_distance(
            before['robot'], after['robot'], CRATE_CENTER, half)-
            robot_radius-excursion_reserve)
    return min(values)


def _target(evidence, marker_id):
    target = (evidence.get('marker_targets') or {}).get(str(marker_id))
    if (not isinstance(target, (list, tuple)) or len(target) < 2 or
            not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in target[:2])):
        return None
    return target[:2]


def assess(records, web_evidence, expected_route=(1, 2, 3), *, robot_radius=BODY_RADIUS_M):
    """Return a conservative verdict; absence of any source fails closed."""
    robot_radius = positive_robot_radius(robot_radius)
    reasons = []
    checks = {}
    metrics = {'crate_center': list(CRATE_CENTER),
               'crate_size_m': [2*value for value in CRATE_HALF_SIZE],
               'robot_enclosing_radius_m': robot_radius,
               'required_margin_m': REQUIRED_BODY_MARGIN_M}

    meta = next((record for record in records if record.get('kind') == 'meta'), {})
    checks['isolated_observer'] = bool(
        meta.get('domain') == 176 and meta.get('localhost_only') is True and
        meta.get('crate_name') == CRATE_NAME and
        meta.get('crate_center') == list(CRATE_CENTER) and
        meta.get('crate_size') == [2*value for value in CRATE_HALF_SIZE])
    if not checks['isolated_observer']:
        reasons.append('isolation_or_crate_identity_unproven')

    goals = [record for record in records if record.get('kind') == 'goal' and
             isinstance(record.get('at'), (int, float))]
    mission_start = goals[0]['at'] if goals else math.inf
    truth = [record for record in records if record.get('kind') == 'truth' and
             isinstance(record.get('at'), (int, float)) and
             isinstance(record.get('robot'), list) and
             len(record['robot']) >= 2]
    mission_truth = [record for record in truth if record['at'] >= mission_start]
    # Select the goal actually active when the robot first traverses the
    # crate's north/south corridor.  In 1→2→3 this is Sector 2; in 1→3→2 it
    # is Sector 3.  Never treat a later, box-clear Sector 2 path as evidence
    # that an earlier Sector 3 passage was planned safely.
    first_crossing = next((record for record in mission_truth
                           if record['robot'][1] < -.90), None)
    active_goals = [goal for goal in goals if first_crossing is not None and
                    goal['at'] <= first_crossing['at']]
    crossing_goal = active_goals[-1] if active_goals else None
    crossing_marker = next((marker_id for marker_id in expected_route
                            if crossing_goal is not None and
                            (target := _target(web_evidence, marker_id)) is not None and
                            math.hypot(crossing_goal.get('x', math.inf)-target[0],
                                       crossing_goal.get('y', math.inf)-target[1]) <= .05),
                           None)
    goal_prefix = f'sector{crossing_marker}' if crossing_marker else 'crossing_leg'
    target = (_target(web_evidence, crossing_marker)
              if crossing_marker is not None else None)
    metrics['crossing_leg_marker_id'] = crossing_marker
    metrics['crossing_leg_goal_at'] = crossing_goal['at'] if crossing_goal else None
    metrics[f'{goal_prefix}_goal_at'] = crossing_goal['at'] if crossing_goal else None
    checks[f'{goal_prefix}_goal_seen'] = crossing_marker is not None
    if crossing_marker is None:
        reasons.append('crate_crossing_leg_ros_goal_unproven')
    crossing_leg_truth = [record for record in mission_truth
                          if crossing_goal is not None and first_crossing is not None
                          and crossing_goal['at'] <= record['at'] <= first_crossing['at']]
    closest_crossing_leg = min(
        (point_rectangle_distance(record['robot'])-robot_radius
         for record in crossing_leg_truth), default=None)
    metrics['crossing_leg_minimum_body_clearance_m'] = (
        round(closest_crossing_leg, 4)
        if closest_crossing_leg is not None else None)
    checks['crossing_leg_near_crate'] = bool(
        closest_crossing_leg is not None and
        closest_crossing_leg <= MAX_CROSSING_LEG_BODY_CLEARANCE_M)
    if not checks['crossing_leg_near_crate']:
        reasons.append('crate_crossing_leg_never_near_box')
    crate_poses, link_metrics = resolve_fixed_crate_truth(mission_truth)
    metrics.update(link_metrics)
    checks['fixed_crate_present_in_live_gazebo'] = crate_poses is not None
    crate_by_record = ({id(record): pose for record, pose in
                        zip(mission_truth, crate_poses)}
                       if crate_poses is not None else {})
    if not checks['fixed_crate_present_in_live_gazebo']:
        reasons.append('fixed_crate_runtime_presence_unproven')

    path_candidates = [record for record in records
                       if record.get('kind') == 'path' and
                       record.get('count', 0) >= 2 and
                       isinstance(record.get('points'), list) and
                       len(record['points']) == record['count'] and
                       target is not None and
                       math.hypot(record['points'][-1][0]-target[0],
                                  record['points'][-1][1]-target[1]) <= .35 and
                       crossing_goal is not None and
                       record['at'] >= crossing_goal['at'] and
                       (first_crossing is None or
                        record['at'] < first_crossing['at'])]
    metrics[f'{goal_prefix}_matching_path_count'] = len(path_candidates)
    first_path_margin = (path_rectangle_clearance(path_candidates[0]['points'], robot_radius=robot_radius)
                         if path_candidates else None)
    metrics[f'initial_{goal_prefix}_path_body_clearance_m'] = (
        round(first_path_margin, 4) if first_path_margin is not None else None)

    raw_hits = [record for record in records if record.get('kind') == 'scan_crate'
                and record.get('matching_beams', 0) >= 2 and
                record.get('truth_age_s', math.inf) <= .30 and
                isinstance(record.get('at'), (int, float)) and
                crossing_goal is not None and
                record['at'] >= crossing_goal['at']]
    observed = [record for record in records if record.get('kind') == 'observed'
                and record.get('map_ready') is True and
                len(set(record.get('nearby_indices') or [])) >= 2 and
                isinstance(record.get('at'), (int, float)) and
                crossing_goal is not None and
                record['at'] >= crossing_goal['at']]
    joint_witnesses = [(obs, hit) for obs in observed for hit in raw_hits
                       if abs(obs['at']-hit['at']) <= 1.]
    latest_before_path = None
    if path_candidates:
        joint_witnesses = [(obs, hit) for obs, hit in joint_witnesses
                           if obs['at'] <= max(path['at'] for path in path_candidates)
                           and hit['at'] <= max(path['at'] for path in path_candidates)]
    checks['raw_lidar_crate_hit'] = bool(raw_hits)
    checks['two_fresh_observed_cells'] = bool(joint_witnesses)
    metrics['raw_scan_hit_samples'] = len(raw_hits)
    metrics['joint_observed_lidar_samples'] = len(joint_witnesses)
    if not checks['raw_lidar_crate_hit']:
        reasons.append('raw_lidar_crate_return_unproven')
    if not checks['two_fresh_observed_cells']:
        reasons.append('two_fresh_lidar_observed_cells_unproven')

    safe_paths = []
    for path in path_candidates:
        witness = next(((obs, hit) for obs, hit in joint_witnesses
                        if 0. <= path['at']-obs['at'] <= 2.0
                        and 0. <= path['at']-hit['at'] <= 2.0), None)
        if witness is None:
            continue
        # The route must be available before the robot reaches the crate.
        # A retrospective safe path after physical passage is no evidence of
        # the planner's ability to steer through this obstacle.
        near_truth = min(mission_truth,
                         key=lambda sample: abs(sample['at']-path['at']),
                         default=None)
        near_crate = crate_by_record.get(id(near_truth)) if near_truth else None
        if (near_truth is None or
                abs(near_truth['at']-path['at']) > .30 or
                near_truth['robot'][1] <= -.35 or
                near_crate is None):
            continue
        margin = path_rectangle_clearance(path['points'], near_crate[:2], robot_radius=robot_radius)
        if margin is not None and margin >= REQUIRED_BODY_MARGIN_M:
            safe_paths.append((path, margin, witness, near_truth))
    safe_path = min(safe_paths, key=lambda item: item[0]['at']) if safe_paths else None
    checks[f'{goal_prefix}_path_after_lidar_is_box_clear'] = bool(safe_path)
    metrics[f'safe_{goal_prefix}_path_at'] = safe_path[0]['at'] if safe_path else None
    metrics[f'safe_{goal_prefix}_path_body_clearance_m'] = (
        round(safe_path[1], 4) if safe_path else None)
    metrics[f'safe_{goal_prefix}_path_points'] = (
        safe_path[0]['count'] if safe_path else None)
    prior_unsafe_paths = ([path for path in path_candidates
                           if safe_path and path['at'] < safe_path[0]['at'] and
                           path['points'] != safe_path[0]['points'] and
                           (path_rectangle_clearance(path['points'], robot_radius=robot_radius) or 0.)
                           < REQUIRED_BODY_MARGIN_M]
                          if safe_path else [])
    checks['unsafe_route_replanned_after_lidar'] = bool(prior_unsafe_paths)
    metrics[f'last_unsafe_{goal_prefix}_path_at'] = (
        prior_unsafe_paths[-1]['at'] if prior_unsafe_paths else None)
    metrics['route_reaction_kind'] = (
        'replanned_bypass' if prior_unsafe_paths else
        'box_clear_initial_route_after_lidar' if safe_path else 'unproven')
    metrics[f'robot_y_at_safe_{goal_prefix}_path_m'] = (
        round(safe_path[3]['robot'][1], 4) if safe_path else None)
    if safe_path:
        latest_before_path = safe_path[2]
        metrics['observed_cells_before_path'] = len(set(
            latest_before_path[0].get('nearby_indices') or []))
        metrics['observed_at'] = latest_before_path[0]['at']
        metrics['raw_lidar_hit_at'] = latest_before_path[1]['at']
    if not checks[f'{goal_prefix}_path_after_lidar_is_box_clear']:
        reasons.append(f'{goal_prefix}_box_aware_safe_path_unproven')

    minimum = (conservative_truth_body_clearance(mission_truth, robot_radius=robot_radius)
               if crate_poses is not None else None)
    metrics['minimum_truth_body_clearance_m'] = (
        round(minimum, 4) if minimum is not None else None)
    checks['body_clearance_2cm_entire_mission'] = bool(
        minimum is not None and minimum >= REQUIRED_BODY_MARGIN_M and
        checks['fixed_crate_present_in_live_gazebo'])
    if not checks['body_clearance_2cm_entire_mission']:
        reasons.append('gazebo_body_clearance_below_2cm_or_unproven')
    truth_gaps = [b['at']-a['at'] for a, b in zip(mission_truth, mission_truth[1:])]
    metrics['max_truth_gap_s'] = round(max(truth_gaps), 4) if truth_gaps else None
    final_at = web_evidence.get('final_observed_at')
    metrics['first_truth_after_goal_s'] = (
        round(mission_truth[0]['at']-mission_start, 4)
        if mission_truth else None)
    metrics['last_truth_after_home_s'] = (
        round(mission_truth[-1]['at']-final_at, 4)
        if mission_truth and isinstance(final_at, (int, float)) else None)
    checks['truth_sampling_sufficient'] = bool(
        len(mission_truth) >= 30 and truth_gaps and
        all(0. < gap <= MAX_TRUTH_GAP_S for gap in truth_gaps) and
        mission_truth[0]['at'] <= mission_start+.5 and
        isinstance(final_at, (int, float)) and
        mission_truth[-1]['at'] >= final_at-.5)
    if not checks['truth_sampling_sufficient']:
        reasons.append('gazebo_truth_sampling_insufficient')

    crossed = [record for record in mission_truth if safe_path is not None and
               first_crossing is not None and
               record['at'] == first_crossing['at'] and
               record['at'] > safe_path[0]['at'] and
               id(record) in crate_by_record]
    checks['gazebo_truth_bypassed_crate'] = bool(crossed)
    metrics['first_y_below_minus_0p90_at'] = crossed[0]['at'] if crossed else None
    if not checks['gazebo_truth_bypassed_crate']:
        reasons.append('gazebo_truth_box_bypass_unproven')

    continuity = web_evidence.get('mission_continuity') or {}
    checks['web_full_route_and_home'] = bool(
        web_evidence.get('passed') is True and
        web_evidence.get('mission_accepted_via_web') is True and
        web_evidence.get('route') == list(expected_route) and
        web_evidence.get('final_route') == list(expected_route) and
        web_evidence.get('completed_ids') == list(expected_route) and
        web_evidence.get('final_state') == 'HOME_REACHED' and
        web_evidence.get('home_stationary_verified') is True and
        continuity.get('route_preserved') is True and
        continuity.get('start_event_count') == 1 and
        continuity.get('cancel_event_count') == 0)
    if not checks['web_full_route_and_home']:
        route_label = ''.join(map(str, expected_route))
        reasons.append(f'web_sector{route_label}_home_completion_unproven')

    return {'passed': not reasons, 'reasons': reasons,
            'checks': checks, 'metrics': metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--web-evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--robot-radius', type=positive_robot_radius, default=BODY_RADIUS_M,
                        help='Verified enclosing body radius in metres; stock Gazebo default 0.24')
    parser.add_argument('--expected-route', default='1,2,3',
                        help='Ordered sector IDs from the staged trial, e.g. 1,3,2')
    args = parser.parse_args()
    try:
        records = [json.loads(line) for line in args.trace.read_text().splitlines()
                   if line.strip()]
        web_evidence = json.loads(args.web_evidence.read_text())
        expected_route = tuple(int(value) for value in args.expected_route.split(','))
        if not expected_route or len(set(expected_route)) != len(expected_route):
            raise ValueError('expected_route must contain distinct sector IDs')
        result = assess(records, web_evidence, expected_route, robot_radius=args.robot_radius)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        result = {'passed': False, 'reasons': ['evidence_read_error: '+str(exc)],
                  'checks': {}, 'metrics': {}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('PASS' if result['passed'] else 'FAIL', args.output, flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
