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

from capture_classroom_stress_crate import (CRATE_CENTER, CRATE_HALF_SIZE,
                                            CRATE_NAME)


BODY_RADIUS_M = .24
REQUIRED_BODY_MARGIN_M = .02
MIN_ROUTE_TO_BOX_M = BODY_RADIUS_M+REQUIRED_BODY_MARGIN_M


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


def path_rectangle_clearance(points, center=CRATE_CENTER):
    """Exact segment-to-rectangle distance minus an enclosing robot radius."""
    if not points:
        return None
    if len(points) == 1:
        return point_rectangle_distance(points[0], center)-BODY_RADIUS_M
    return min(segment_rectangle_distance(start, end, center)-BODY_RADIUS_M
               for start, end in zip(points, points[1:]))


def _target(evidence, marker_id):
    target = (evidence.get('marker_targets') or {}).get(str(marker_id))
    if (not isinstance(target, (list, tuple)) or len(target) < 2 or
            not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in target[:2])):
        return None
    return target[:2]


def assess(records, web_evidence):
    """Return a conservative verdict; absence of any source fails closed."""
    reasons = []
    checks = {}
    metrics = {'crate_center': list(CRATE_CENTER),
               'crate_size_m': [2*value for value in CRATE_HALF_SIZE],
               'robot_enclosing_radius_m': BODY_RADIUS_M,
               'required_margin_m': REQUIRED_BODY_MARGIN_M}

    meta = next((record for record in records if record.get('kind') == 'meta'), {})
    checks['isolated_observer'] = bool(
        meta.get('domain') == 176 and meta.get('localhost_only') is True and
        meta.get('crate_name') == CRATE_NAME and
        meta.get('crate_center') == list(CRATE_CENTER) and
        meta.get('crate_size') == [2*value for value in CRATE_HALF_SIZE])
    if not checks['isolated_observer']:
        reasons.append('isolation_or_crate_identity_unproven')

    goals = [record for record in records if record.get('kind') == 'goal']
    target = _target(web_evidence, 2)
    matching_goals = [goal for goal in goals if target is not None and
                      math.hypot(goal.get('x', math.inf)-target[0],
                                 goal.get('y', math.inf)-target[1]) <= .05]
    sector2_goal = matching_goals[0] if matching_goals else None
    metrics['sector2_goal_at'] = sector2_goal.get('at') if sector2_goal else None
    checks['sector2_goal_seen'] = bool(sector2_goal)
    if not checks['sector2_goal_seen']:
        reasons.append('sector2_ros_goal_unproven')

    mission_start = goals[0]['at'] if goals else math.inf
    truth = [record for record in records if record.get('kind') == 'truth' and
             isinstance(record.get('at'), (int, float)) and
             isinstance(record.get('robot'), list) and
             len(record['robot']) >= 2]
    mission_truth = [record for record in truth if record['at'] >= mission_start]
    fixed = [record for record in mission_truth
             if record.get('crate_model_present') is True and
             isinstance(record.get('crate_link_age_s'), (int, float)) and
             record['crate_link_age_s'] <= .30 and
             isinstance(record.get('crate'), list) and
             len(record['crate']) >= 2 and
             math.hypot(record['crate'][0]-CRATE_CENTER[0],
                        record['crate'][1]-CRATE_CENTER[1]) <= .005 and
             len(record['crate']) >= 3 and abs(record['crate'][2]) <= .02]
    checks['fixed_crate_present_in_live_gazebo'] = bool(
        len(fixed) >= 20 and len(fixed) == len(mission_truth))
    metrics['mission_truth_samples'] = len(mission_truth)
    metrics['fixed_crate_truth_samples'] = len(fixed)
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
                       sector2_goal is not None and
                       record['at'] >= sector2_goal['at']]
    metrics['sector2_matching_path_count'] = len(path_candidates)
    first_path_margin = (path_rectangle_clearance(path_candidates[0]['points'])
                         if path_candidates else None)
    metrics['initial_sector2_path_body_clearance_m'] = (
        round(first_path_margin, 4) if first_path_margin is not None else None)

    raw_hits = [record for record in records if record.get('kind') == 'scan_crate'
                and record.get('matching_beams', 0) >= 2 and
                record.get('truth_age_s', math.inf) <= .30 and
                isinstance(record.get('at'), (int, float)) and
                record['at'] >= mission_start]
    observed = [record for record in records if record.get('kind') == 'observed'
                and record.get('map_ready') is True and
                len(set(record.get('nearby_indices') or [])) >= 2 and
                isinstance(record.get('at'), (int, float)) and
                record['at'] >= mission_start]
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
        if (near_truth is None or
                abs(near_truth['at']-path['at']) > .30 or
                near_truth['robot'][1] <= -.35 or
                not isinstance(near_truth.get('crate'), list)):
            continue
        margin = path_rectangle_clearance(path['points'], near_truth['crate'][:2])
        if margin is not None and margin >= REQUIRED_BODY_MARGIN_M:
            safe_paths.append((path, margin, witness, near_truth))
    safe_path = min(safe_paths, key=lambda item: item[0]['at']) if safe_paths else None
    checks['sector2_path_after_lidar_is_box_clear'] = bool(safe_path)
    metrics['safe_sector2_path_at'] = safe_path[0]['at'] if safe_path else None
    metrics['safe_sector2_path_body_clearance_m'] = (
        round(safe_path[1], 4) if safe_path else None)
    metrics['safe_sector2_path_points'] = (
        safe_path[0]['count'] if safe_path else None)
    prior_unsafe_paths = ([path for path in path_candidates
                           if safe_path and path['at'] < safe_path[0]['at'] and
                           path['points'] != safe_path[0]['points'] and
                           (path_rectangle_clearance(path['points']) or 0.)
                           < REQUIRED_BODY_MARGIN_M]
                          if safe_path else [])
    checks['unsafe_route_replanned_after_lidar'] = bool(prior_unsafe_paths)
    metrics['last_unsafe_sector2_path_at'] = (
        prior_unsafe_paths[-1]['at'] if prior_unsafe_paths else None)
    metrics['route_reaction_kind'] = (
        'replanned_bypass' if prior_unsafe_paths else
        'box_clear_initial_route_after_lidar' if safe_path else 'unproven')
    metrics['robot_y_at_safe_sector2_path_m'] = (
        round(safe_path[3]['robot'][1], 4) if safe_path else None)
    if safe_path:
        latest_before_path = safe_path[2]
        metrics['observed_cells_before_path'] = len(set(
            latest_before_path[0].get('nearby_indices') or []))
        metrics['observed_at'] = latest_before_path[0]['at']
        metrics['raw_lidar_hit_at'] = latest_before_path[1]['at']
    if not checks['sector2_path_after_lidar_is_box_clear']:
        reasons.append('sector2_box_aware_safe_path_unproven')

    clearances = [(point_rectangle_distance(
                    record['robot'], record['crate'][:2])-BODY_RADIUS_M,
                   record['at']) for record in fixed]
    minimum = min((value for value, _ in clearances), default=None)
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
        len(mission_truth) >= 30 and truth_gaps and max(truth_gaps) <= .5 and
        mission_truth[0]['at'] <= mission_start+.5 and
        isinstance(final_at, (int, float)) and
        mission_truth[-1]['at'] >= final_at-.5)
    if not checks['truth_sampling_sufficient']:
        reasons.append('gazebo_truth_sampling_insufficient')

    crossed = [record for record in mission_truth if safe_path is not None and
               record['at'] > safe_path[0]['at'] and record['robot'][1] < -.90 and
               isinstance(record.get('crate'), list)]
    checks['gazebo_truth_bypassed_crate'] = bool(crossed)
    metrics['first_y_below_minus_0p90_at'] = crossed[0]['at'] if crossed else None
    if not checks['gazebo_truth_bypassed_crate']:
        reasons.append('gazebo_truth_box_bypass_unproven')

    continuity = web_evidence.get('mission_continuity') or {}
    checks['web_full_route_and_home'] = bool(
        web_evidence.get('passed') is True and
        web_evidence.get('mission_accepted_via_web') is True and
        web_evidence.get('route') == [1, 2, 3] and
        web_evidence.get('final_route') == [1, 2, 3] and
        web_evidence.get('completed_ids') == [1, 2, 3] and
        web_evidence.get('final_state') == 'HOME_REACHED' and
        web_evidence.get('home_stationary_verified') is True and
        continuity.get('route_preserved') is True and
        continuity.get('start_event_count') == 1 and
        continuity.get('cancel_event_count') == 0)
    if not checks['web_full_route_and_home']:
        reasons.append('web_sector123_home_completion_unproven')

    return {'passed': not reasons, 'reasons': reasons,
            'checks': checks, 'metrics': metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--web-evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        records = [json.loads(line) for line in args.trace.read_text().splitlines()
                   if line.strip()]
        web_evidence = json.loads(args.web_evidence.read_text())
        result = assess(records, web_evidence)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        result = {'passed': False, 'reasons': ['evidence_read_error: '+str(exc)],
                  'checks': {}, 'metrics': {}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print('PASS' if result['passed'] else 'FAIL', args.output, flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
