#!/usr/bin/env python3
"""Run evidence-gated classroom Gazebo stages without touching robot domain 16.

Each stage starts a fresh headless world through run_classroom_fault_sim.sh.
The runner owns Gazebo cleanup; this wrapper never starts two worlds at once.
"""

import argparse
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen
import xml.etree.ElementTree as ET
import yaml

import assess_classroom_safety_timeline as safety_timeline
from build_classroom_replay_nav_profile import (
    SIM_INFLATION_RADIUS_M, SIM_PREFERRED_CLEARANCE_M, SIM_ROBOT_RADIUS_M,
    SIM_STATIC_INFLATION_CELLS, build_nav_profile_data, build_nodes_profile_data,
)


REPO = Path(__file__).resolve().parents[1]
RUNNER = REPO / 'scripts/run_classroom_fault_sim.sh'
MASTER_PORT = 11576
SECTOR3 = ('-.25', '-1.98', '-1.40')
HOME = (-.188431, -.220497, -.027514)
WEB_FULL_STACK_STAGES = {
    '03_web_sector3_clean',
    '06_web_sector3_moving_obstacle',
    '12_web_integrated_clean',
    '12_web_integrated_home_obstacle',
    '12_web_integrated_stress_clean',
    '12_web_integrated_stress_permuted',
    '12_web_integrated_stress_home_obstacle',
    '17_web_hardware_value_parity',
    '20_web_memory_session',
    '21_web_memory_sector2',
    '22_web_memory_integrated_clean',
    '23_web_memory_integrated_obstacle',
    '24_web_memory_sector1_to_sector2_removal',
    '18_web_sector2_same_session_reentry',
    '19_web_inventory_sector2_same_session_reentry',
    '25_web_inventory_sector3_moving_obstacle',
}
WEB_RUNTIME_KINDS = {'web_dwell', 'web_transient', 'web_moving_obstacle',
                     'web_reentry'}
MAX_WEB_TRUTH_XY_TOLERANCE_M = .25


def web_runtime_source_hashes(stage: 'Stage') -> dict[str, str]:
    """Fingerprint the checked-in software and assets a full web trial can load.

    Enumerate both before and after the trial, so a newly added or deleted
    source file also invalidates its evidence. Exclude tests, docs and generated
    ROS logs; changes there do not alter a running mission.
    """
    if stage.kind not in WEB_RUNTIME_KINDS:
        return {}
    package = REPO/'ros2_ws/src/smart_vision_amr'
    roots = (
        (REPO/'scripts', {'.py', '.sh'}),
        (package/'smart_vision_amr', {'.py'}),
        (package/'launch', {'.py'}),
        (package/'web', {'.html', '.js', '.css', '.svg'}),
        (package/'config', {'.yaml', '.xml'}),
        (package/'maps', {'.yaml', '.pgm'}),
        (package/'worlds', {'.world', '.sdf'}),
        (REPO/'ros2_ws/src/smart_vision_amr_dwb', {'.cpp', '.hpp', '.xml', '.txt'}),
        (REPO/'gazebo_models', {'.config', '.sdf', '.material', '.png', '.jpg',
                              '.obj', '.json'}),
    )
    paths = sorted({path for directory, suffixes in roots
                    for path in directory.rglob('*')
                    if path.is_file() and path.suffix in suffixes
                    and 'test' not in path.relative_to(REPO).parts})
    return {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


@dataclass(frozen=True)
class Stage:
    key: str
    description: str
    options: tuple[str, ...]
    route: tuple[str, ...] = ()
    kind: str = 'route'


STAGES = (
    Stage('01_sector_1', 'Home → Sector 1 단독',
          ('--single-goal', 'Sector1Candidate', '1.20', '.55', '3.114078'),
          ('Sector1Candidate',)),
    Stage('02_sector_2', 'Home → Sector 2 단독',
          ('--single-goal', 'Sector2Candidate', '1.525', '-2.65', '1.543282'),
          ('Sector2Candidate',)),
    Stage('03_sector_3', 'Home → Sector 3 단독',
          ('--single-goal', 'Sector3', *SECTOR3), ('Sector3',)),
    Stage('03_web_sector3_clean',
          '전체 시스템: 웹 Sector 3→Home 단독, ArUco/5초 대기',
          (), ('Sector 3', 'Home'), 'web_dwell'),
    Stage('04_sectors_1_2', 'Home → Sector 1 → 2',
          ('--route-prefix', '2'), ('Sector 1', 'Sector 2')),
    Stage('05_sectors_1_2_3', 'Home → Sector 1 → 2 → 3',
          ('--route-prefix', '3'), ('Sector 1', 'Sector 2', 'Sector 3')),
    Stage('06_single_unmapped_obstacle',
          '주행 계층 진단: Sector 3 단독 중 지도에 없는 이동 장애물',
          ('--goal-x', SECTOR3[0], '--goal-y', SECTOR3[1],
           '--goal-yaw', SECTOR3[2], '--obstacle', 'moving'),
          kind='moving_obstacle'),
    Stage('06_web_sector3_moving_obstacle',
          '전체 시스템: 웹 Sector 3→Home 중 비식별 이동 장애물',
          (), ('Sector 3', 'Home'), 'web_moving_obstacle'),
    Stage('07_cumulative_unmapped_obstacle',
          'Sector 1 → 2 → 3 중 지도에 없는 장애물',
          ('--route-prefix', '3', '--route-obstacle'),
          ('Sector 1', 'Sector 2', 'Sector 3'), 'route_obstacle'),
    Stage('08_browser_client_disconnect',
          '웹에서 접수한 임무 중 브라우저만 연결 해제', (), kind='web_disconnect'),
    Stage('09_wan_absent',
          '외부 인터넷 경로가 없는 격리 네트워크에서 단독 주행',
          ('--single-goal', 'Sector3', *SECTOR3), ('Sector3',), 'wan_absent'),
    Stage('10_sensor_dds_outage',
          '라이다/오도메트리/TF 지속 단절: 비상정지와 수동 재설정 대기',
          ('--profile', 'outages', '--goal-x', SECTOR3[0],
           '--goal-y', SECTOR3[1], '--goal-yaw', SECTOR3[2],
           '--expect-sustained-outage-hold'),
          kind='sensor_outage'),
    Stage('11_map_world_mismatch',
          '저장 지도와 다른 좁은 통로/상자 월드',
          ('--stress', '--allow-safe-hold', '--goal-x', SECTOR3[0],
           '--goal-y', SECTOR3[1], '--goal-yaw', SECTOR3[2]),
          kind='map_mismatch'),
    Stage('12_integrated_clean',
          'Sector 1 → 2 → 3 → Home, 새 장애물 없이 통합 기준 주행',
          ('--mission-route',),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home')),
    Stage('12_integrated_home_obstacle',
          'Sector 1 → 2 → 3 → Home 중 장애물 주입·우회·제거',
          ('--mission-route', '--home-obstacle'),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'home_obstacle'),
    Stage('12_web_integrated_clean',
          '전체 시스템: 웹 Sector 1→2→3→Home, ArUco/5초 대기',
          (), ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('12_web_integrated_home_obstacle',
          '전체 시스템: 웹 1→2→3→Home 복귀 중 비식별 이동 장애물',
          (), ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_moving_obstacle'),
    Stage('12_web_integrated_stress_clean',
          '전체 시스템: 악조건 교실 월드 웹 1→2→3→Home, 새 장애물 없음',
          ('--stress',), ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('12_web_integrated_stress_permuted',
          '전체 시스템: 악조건 교실 월드 웹 1→3→2→Home, 순서 변경',
          ('--stress',), ('Sector 1', 'Sector 3', 'Sector 2', 'Home'), 'web_dwell'),
    Stage('12_web_integrated_stress_home_obstacle',
          '전체 시스템: 악조건 교실 월드 Home 복귀 중 이동 장애물',
          ('--stress',), ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_moving_obstacle'),
    Stage('17_web_hardware_value_parity',
          '실차 설정값 비교: 악조건 교실 웹 1→2→3→Home 최종 재시험',
          ('--stress',), ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('20_web_memory_session',
          '런타임 장애물 기억: 악조건 교실 웹 1→2→3→Home',
          ('--stress', '--memory-session'),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('21_web_memory_sector2',
          '런타임 장애물 기억: 웹 Sector 2→Home 단독',
          ('--memory-session',), ('Sector 2', 'Home'), 'web_dwell'),
    Stage('22_web_memory_integrated_clean',
          '런타임 장애물 기억: 웹 1→2→3→Home 통합',
          ('--memory-session',),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('23_web_memory_integrated_obstacle',
          '런타임 장애물 기억: 웹 1→2→3→Home 중 새 장애물',
          ('--memory-session',),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_moving_obstacle'),
    Stage('24_web_memory_sector1_to_sector2_removal',
          '런타임 기억: Sector 1 장애물을 기억한 채 2번 계획 후 주행 중 제거',
          ('--memory-session',),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('18_web_sector2_same_session_reentry',
          '같은 웹·ROS 프로세스에서 Sector 2→Home 두 번: AMCL 수동 초기화·비상정지 해제',
          (), ('Sector 2', 'Home'), 'web_reentry'),
    Stage('19_web_inventory_sector2_same_session_reentry',
          '관리자 재고 요청→운용자 적재·출발→Sector 2→Home 두 번, 같은 웹·ROS 프로세스',
          ('--inventory',), ('Sector 2', 'Home'), 'web_reentry'),
    Stage('25_web_inventory_sector3_moving_obstacle',
          '전체 운용 절차: 재고 요청→웹 초기 위치·비상정지 해제·pre-arm→적재 출발→Sector 3 장애물→Home',
          ('--inventory-before-prearm',),
          ('Sector 3', 'Home'), 'web_moving_obstacle'),
    Stage('13_web_sector_dwell_1_2',
          '웹 임무 Sector 1 → 2: 각 5초 정지 후 다음 목표, 마지막 Home',
          (), ('Sector 1', 'Sector 2', 'Home'), 'web_dwell'),
    Stage('14_web_sector_dwell_1_2_3',
          '웹 임무 Sector 1 → 2 → 3: 각 5초 정지 후 다음 목표, 마지막 Home',
          (), ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'web_dwell'),
    Stage('15_integrated_with_recorder',
          '비행기록기 켠 실차 기본 모드에서 1 → 2 → 3 → Home 재검증',
          ('--mission-route', '--home-obstacle', '--record-incidents'),
          ('Sector 1', 'Sector 2', 'Sector 3', 'Home'), 'home_obstacle'),
    Stage('16_web_transient_resume',
          '웹 소유 Sector 1 → 2 → Home 중 0.3~0.4초 센서 지연 후 목표 유지·자동 재개',
          ('--ekf-delivery-faults',), ('Sector 1', 'Sector 2', 'Home'), 'web_transient'),
)
BY_KEY = {stage.key: stage for stage in STAGES}


def diagnosed_leg_terminal(leg: dict) -> str | None:
    """Do not report a sensor interlock as an obstacle-navigation failure."""
    terminal = leg.get('terminal')
    if (terminal == 'PERSISTENT_OBSTACLE_HOLD' and
            ((leg.get('failure_memory_status') or {}).get('state') == 'SENSOR_HOLD'
             or (leg.get('last_navigation') or {}).get('input_hold') is True)):
        return 'SENSOR_HOLD'
    return terminal


@dataclass(frozen=True)
class _ColdStartContinuation:
    """Assessor-only provenance for cycle 2; never fabricated cold evidence."""
    runtime_identity: dict
    first_cycle_proof: dict


def _cold_reentry_continuation(evidence: dict, cycles: list):
    """A cold boot happens once, only on a proved uninterrupted runtime."""
    initial = evidence.get('runtime_identity_initial') or {}
    identities = [evidence.get(key) for key in (
        'runtime_identity_after_first', 'runtime_identity_after_second')]
    if (len(cycles) != 2 or evidence.get('same_runtime') is not True
            or not isinstance(initial, dict)
            or not {'web', 'gazebo_runner'} <= initial.keys()
            or any(not isinstance(initial[key], dict) or any(
                type(initial[key].get(field)) is not int or initial[key][field] <= 0
                for field in ('pid', 'start_ticks')) for key in ('web', 'gazebo_runner'))
            or any(identity != initial for identity in identities)):
        return None, 'cold_reentry_runtime_identity_unverified'
    first = cycles[0]
    proof = evidence.get('cold_start_proof') or {}
    before, after = proof.get('before') or {}, proof.get('after') or {}
    manual = first.get('manual_initial_pose') or {}
    observed = before.get('observed_seconds')
    if (proof.get('passed') is not True or proof != first.get('cold_start_proof')
            or first.get('cold_start_requested') is not True
            or first.get('passed') is not True
            or before.get('amcl_set_initial_pose') is not False
            or any(type(before.get(key)) is not int or before[key] != 0 for key in
                   ('initial_pose_messages', 'amcl_messages', 'map_tf_messages'))
            or 'web_pose' not in before or before['web_pose'] is not None
            or type(observed) not in (int, float) or not math.isfinite(observed)
            or observed < .5
            or after.get('amcl_set_initial_pose') is not False
            or type(after.get('initial_pose_messages')) is not int
            or after['initial_pose_messages'] != 0
            or any(type(after.get(key)) is not int or after[key] <= 0
                   for key in ('amcl_messages', 'map_tf_messages'))
            or manual.get('state') != 'VERIFIED'
            or manual.get('estop_latched_after_verification') is not True
            or manual.get('stop_acknowledged_after_verification') is not True):
        return None, 'cold_reentry_first_cycle_proof_unverified'
    return _ColdStartContinuation(initial, proof), None


def assess(stage: Stage, evidence: dict, *,
           _cold_start_continuation: _ColdStartContinuation | None = None) -> dict:
    """Require observed behavior, rather than trusting only a runner exit code."""
    reasons = []
    if evidence.get('domain') != 176 or evidence.get('localhost_only') is not True:
        reasons.append('not_isolated_domain_176')
    if evidence.get('physical_robot_touched') is not False:
        reasons.append('physical_robot_scope_not_proven')
    if evidence.get('passed') is not True:
        reasons.append('verifier_failed')
    if evidence.get('failure_class') == 'STARTUP_LIFECYCLE_ABORT':
        reasons.append('startup_lifecycle_abort')
        if evidence.get('fault_execution') == 'TRANSIENT_NOT_STARTED':
            reasons.append('transient_not_started')
    if (evidence.get('web_runtime_manifest') or {}).get('cold_start') is True:
        if ((evidence.get('cold_start_proof') or {}).get('passed') is not True
                and not isinstance(_cold_start_continuation, _ColdStartContinuation)):
            reasons.append('cold_start_without_prior_and_web_initialization_unproven')
    if (stage.kind not in {'web_disconnect', 'web_dwell', 'web_transient',
                           'web_moving_obstacle', 'web_reentry'} and
            evidence.get('fault_delivery', {}).get('passed') is not True):
        reasons.append('sensor_delivery_or_fault_not_proven')
    if stage.key == '12_integrated_clean' and (
            evidence.get('home_obstacle_requested') is not False or
            evidence.get('route_obstacle_requested') is not False or
            evidence.get('home_obstacle') is not None or
            evidence.get('route_obstacle') is not None):
        reasons.append('clean_route_obstacle_fixture_present')

    if stage.kind == 'web_disconnect':
        if evidence.get('mission_accepted_via_web') is not True:
            reasons.append('web_mission_not_accepted')
        if evidence.get('web_client_disconnected') is not True:
            reasons.append('initiating_browser_not_disconnected')
        if evidence.get('mission_route_after_disconnect') != [3]:
            reasons.append('mission_route_lost_after_disconnect')
        if evidence.get('state_after_disconnect') not in {
                'NAVIGATING', 'CONFIRMING', 'DWELLING', 'RETURNING_HOME',
                'HOME_REACHED', 'COMPLETED'}:
            reasons.append('mission_paused_or_canceled_after_disconnect')
        if evidence.get('displacement_m') is None or evidence['displacement_m'] < .08:
            reasons.append('gazebo_motion_after_disconnect_not_proven')
        if evidence.get('web_disconnect_truth', {}).get('passed') is not True:
            reasons.append('gazebo_truth_motion_after_disconnect_not_proven')
        return {'stage': stage.key,
                'outcome': 'PASS' if not reasons else 'FAIL',
                'passed': not reasons, 'reasons': reasons,
                'runner_passed': evidence.get('passed') is True,
                'failure_class': evidence.get('failure_class'),
                'fault_execution': evidence.get('fault_execution'),
                'route': evidence.get('route'), 'legs': []}

    if stage.kind == 'web_reentry':
        cycles = evidence.get('cycles') or []
        if len(cycles) != 2 or evidence.get('same_runtime') is not True:
            reasons.append('two_deliveries_same_runtime_unverified')
        continuation = None
        if (evidence.get('web_runtime_manifest') or {}).get('cold_start') is True:
            continuation, error = _cold_reentry_continuation(evidence, cycles)
            if error:
                reasons.append(error)
        reentry = evidence.get('reentry') or {}
        if (reentry.get('manual_pose_state') != 'VERIFIED' or
                not isinstance(reentry.get('pose_verified_at'), (int, float))):
            reasons.append('amcl_and_health_reentry_unverified')
        if len(cycles) == 2 and not (
                cycles[0].get('final_observed_at', math.inf) <
                reentry.get('pose_requested_at', -math.inf) <
                reentry.get('pose_verified_at', -math.inf) <
                cycles[1].get('mission_started_at', -math.inf)):
            reasons.append('reentry_sequence_unverified')
        for index, cycle in enumerate(cycles):
            if '--inventory' in stage.options and (
                    cycle.get('dispatch_mode') != 'inventory' or
                    (cycle.get('inventory_completion') or {}).get('status') != 'DELIVERED'):
                reasons.append(f'cycle_{index+1}:inventory_delivery_unverified')
            cycle_stage = Stage('12_web_integrated_clean',
                                'Sector 2 strict cycle', (),
                                ('Sector 2', 'Home'), 'web_dwell')
            derived = (evidence.get('cycle_independent') or [])
            if index < len(derived):
                cycle = {**cycle, **derived[index],
                         'web_runtime_manifest': evidence.get('web_runtime_manifest')}
            # Keep the actual launch manifest unchanged. Only this second
            # cycle may reuse the first cycle's genuine cold-start provenance;
            # all present-time pre-arm, pose, dwell and truth checks still run.
            outcome = assess(cycle_stage, cycle, _cold_start_continuation=(
                continuation if index == 1 else None))
            if outcome['passed'] is not True:
                reasons.extend(f'cycle_{index+1}:{reason}'
                               for reason in outcome['reasons'])
        return {'stage': stage.key,
                'outcome': 'PASS' if not reasons else 'FAIL',
                'passed': not reasons, 'reasons': reasons,
                'runner_passed': evidence.get('passed') is True,
                'route': [2, 2], 'legs': []}

    if stage.kind in {'web_dwell', 'web_transient', 'web_moving_obstacle'}:
        if evidence.get('mission_accepted_via_web') is not True:
            reasons.append('web_mission_not_accepted')
        expected = [int(stop.removeprefix('Sector ')) for stop in stage.route
                    if stop.startswith('Sector ')]
        if evidence.get('route') != expected or evidence.get('final_route') != expected:
            reasons.append('web_route_order_mismatch')
        if evidence.get('final_state') != 'HOME_REACHED' or evidence.get(
                'completed_ids') != expected:
            reasons.append('sector_route_and_home_not_completed')
        if stage.key in WEB_FULL_STACK_STAGES:
            prearm = evidence.get('prearm') or {}
            if prearm.get('passed') is not True:
                reasons.extend('prearm:'+gate for gate in
                               (prearm.get('failed_gates') or ['evidence_missing']))
            if type(evidence.get('front_camera_enabled')) is not bool:
                reasons.append('web_camera_configuration_unverified')
            if evidence.get('startup_reset_requested_via_web') is not True or evidence.get(
                    'startup_reset_completed') is not True:
                reasons.append('physical_style_web_startup_reset_unverified')
            manifest = evidence.get('web_runtime_manifest') or {}
            if manifest.get('mode') != 'ekf_independent':
                reasons.append('independent_ekf_runtime_unverified')
            if manifest.get('record_incidents') is not True:
                reasons.append('flight_recorder_runtime_unverified')
            if '--stress' in stage.options and evidence.get('web_world_variant') != 'stress':
                reasons.append('stress_world_runtime_unverified')
            if '--stress' in stage.options and evidence.get(
                    'stress_crate_assessment', {}).get('passed') is not True:
                reasons.append('unmapped_stress_crate_bypass_unverified')
            if '--memory-session' in stage.options:
                if manifest.get('memory_session') is not True:
                    reasons.append('runtime_memory_not_enabled')
                if manifest.get('laser_filter_for_change_memory') is not False:
                    reasons.append('laser_filter_not_disabled')
            if stage.key == '17_web_hardware_value_parity':
                parity = evidence.get('physical_value_parity') or {}
                if (parity.get('passed') is not True or
                        parity.get('copy_preserves_selected_values') is not True):
                    reasons.append('physical_source_yaml_value_parity_unverified')
                if parity.get('authoritative_physical_sources') is not True:
                    reasons.append('authoritative_physical_sources_unverified')
        for marker_id in expected:
            item = evidence.get('dwell', {}).get(str(marker_id), {})
            if item.get('event_duration_s') is None or item['event_duration_s'] < 4.9:
                reasons.append(f'sector_{marker_id}_five_second_dwell_unverified')
            if item.get('marker_confirmed') is not True:
                reasons.append(f'sector_{marker_id}_aruco_unverified')
            if stage.key in WEB_FULL_STACK_STAGES:
                alignment = item.get('alignment_at_dwell') or {}
                if (alignment.get('state') != 'CONFIRMED' or
                        alignment.get('soft_center_result') != 'centered' or
                        not isinstance(alignment.get('center_error'), (int, float)) or
                        abs(alignment['center_error']) > .36):
                    reasons.append(f'sector_{marker_id}_rear_camera_center_unverified')
                rotation = alignment.get('rotation_degrees')
                if not isinstance(rotation, (int, float)) or rotation > 60.:
                    reasons.append(f'sector_{marker_id}_excessive_marker_search_rotation')
                frame = (evidence.get('rear_camera_frames') or {}).get(str(marker_id)) or {}
                if (frame.get('bytes', 0) < 100 or
                        frame.get('unannotated') is not True or
                        not isinstance(frame.get('raw_camera_age_s'), (int, float)) or
                        frame['raw_camera_age_s'] >= .8 or
                        frame.get('expected_visible') is not True or
                        not isinstance(frame.get('center_error'), (int, float)) or
                        abs(frame['center_error']) > .36):
                    reasons.append(f'sector_{marker_id}_rear_camera_frame_missing')
                if evidence.get('front_camera_enabled') is True:
                    front = (evidence.get('front_camera_frames') or {}).get(str(marker_id)) or {}
                    if (front.get('bytes', 0) < 100 or
                            not isinstance(front.get('camera_age_s'), (int, float)) or
                            front['camera_age_s'] >= .8 or
                            front.get('width', 0) < 100 or front.get('height', 0) < 100):
                        reasons.append(f'sector_{marker_id}_fresh_front_camera_unverified')
        checks = evidence.get('web_ground_truth_checks', {})
        for target in (*map(str, expected), 'home'):
            if checks.get(target, {}).get('passed') is not True:
                reasons.append(f'{target}_gazebo_truth_unverified')
        dwell_checks = evidence.get('web_independent_dwell_checks', {})
        for marker_id in expected:
            if dwell_checks.get(str(marker_id), {}).get('passed') is not True:
                reasons.append(f'sector_{marker_id}_independent_dwell_unverified')
        if stage.key in WEB_FULL_STACK_STAGES and evidence.get(
                'home_independent_settle_check', {}).get('passed') is not True:
            reasons.append('home_independent_settle_unverified')
        if stage.kind == 'web_transient':
            transient = evidence.get('web_transient_checks', {})
            if transient.get('passed') is not True:
                reasons.extend(transient.get('reasons') or
                               ['web_transient_goal_retention_unverified'])
        if stage.kind == 'web_moving_obstacle':
            obstacle = evidence.get('web_obstacle') or {}
            if obstacle.get('passed') is not True:
                reasons.append('web_obstacle_injector_failed')
            for key in ('spawned', 'truth_seen', 'removed', 'observed_while_live',
                        'observed_cleared', 'path_reacted_before_removal',
                        'bypassed_while_live'):
                if obstacle.get(key) is not True:
                    reasons.append('web_obstacle_'+key+'_unverified')
            if obstacle.get('moving_steps') != 4:
                reasons.append('web_obstacle_motion_unverified')
            clearance = obstacle.get('minimum_clearance_m')
            if clearance is None or clearance < .02:
                reasons.append('web_obstacle_clearance_unverified')
            overlay = evidence.get('web_obstacle_overlay_checks') or {}
            if overlay.get('red_cells_while_live') is not True:
                reasons.append('web_red_obstacle_overlay_unverified')
            if overlay.get('red_cells_cleared_after_removal') is not True:
                reasons.append('web_red_obstacle_clearance_unverified')
            if stage.key in {'12_web_integrated_home_obstacle',
                             '12_web_integrated_stress_home_obstacle'} and evidence.get(
                    'home_obstacle_temporal_check', {}).get('passed') is not True:
                reasons.append('home_return_obstacle_timing_unverified')
        if '--inventory-before-prearm' in stage.options:
            request = evidence.get('inventory_request') or {}
            initial = evidence.get('manual_initial_pose') or {}
            completion = evidence.get('inventory_completion') or {}
            pages = evidence.get('web_pages') or {}
            if pages.get('inventory') is not True or pages.get('operator') is not True:
                reasons.append('inventory_and_operator_web_pages_unverified')
            if (evidence.get('dispatch_mode') != 'inventory' or
                    request.get('marker_id') != 3 or request.get('quantity') != 1 or
                    not isinstance(request.get('id'), int)):
                reasons.append('sector3_inventory_request_unverified')
            if (initial.get('state') != 'VERIFIED' or
                    initial.get('estop_latched_after_verification') is not True or
                    initial.get('stop_acknowledged_after_verification') is not True):
                reasons.append('web_initial_pose_verification_unverified')
            timestamps = [evidence.get('inventory_request_created_at'),
                          initial.get('requested_at'), initial.get('verified_at'),
                          evidence.get('startup_reset_accepted_at'),
                          (evidence.get('prearm') or {}).get('sampled_at'),
                          evidence.get('mission_started_at')]
            if (not all(isinstance(value, (int, float)) and math.isfinite(value)
                        for value in timestamps) or
                    any(left >= right for left, right in zip(timestamps, timestamps[1:]))):
                reasons.append('inventory_pose_reset_prearm_dispatch_order_unverified')
            if evidence.get('dispatch_load_confirmed') is not True:
                reasons.append('operator_load_confirmation_unverified')
            if (completion.get('request_id') != request.get('id') or
                    completion.get('status') != 'DELIVERED' or
                    completion.get('service_active') is not False):
                reasons.append('inventory_delivery_completion_unverified')
        if stage.key == '24_web_memory_sector1_to_sector2_removal':
            transition = evidence.get('memory_transition') or {}
            if transition.get('passed') is not True:
                reasons.append('sector1_to_sector2_memory_transition_failed')
            for key in ('spawned', 'truth_seen', 'observed_on_sector1',
                        'confirmed_on_sector1', 'sector2_planned_with_memory',
                        'sector2_initial_plan_without_live_hit',
                        'removed_on_sector2', 'free_ray_evidence_after_removal',
                        'memory_cleared'):
                if transition.get(key) is not True:
                    reasons.append('memory_transition_'+key+'_unverified')
            if transition.get('minimum_clearance_m') is None or transition[
                    'minimum_clearance_m'] < .02:
                reasons.append('memory_transition_clearance_unverified')
            if transition.get('after_removal') not in {
                    'CONTINUED_EXISTING_PATH', 'NEW_PATH_OR_REPLAN'}:
                reasons.append('path_behavior_after_removal_unverified')
        return {'stage': stage.key,
                'outcome': 'PASS' if not reasons else 'FAIL',
                'passed': not reasons, 'reasons': reasons,
                'runner_passed': evidence.get('passed') is True,
                'failure_class': evidence.get('failure_class'),
                'fault_execution': evidence.get('fault_execution'),
                'route': evidence.get('route'), 'legs': []}

    route = stage.kind in ('route', 'wan_absent', 'route_obstacle', 'home_obstacle')
    legs = evidence.get('legs') or []
    if route:
        if evidence.get('route') != list(stage.route):
            reasons.append('route_order_mismatch')
        if len(legs) != len(stage.route):
            reasons.append('missing_route_leg')
        for index, stop in enumerate(stage.route):
            if index >= len(legs):
                break
            leg = legs[index]
            terminal = diagnosed_leg_terminal(leg)
            if leg.get('stop') != stop or terminal != 'SUCCEEDED':
                reasons.append(f'{stop}:{terminal or "missing_terminal"}')
            if leg.get('passed') is not True:
                reasons.append(f'{stop}:acceptance_failed')
            if (leg.get('position_error_m') is None or
                    leg['position_error_m'] > .35):
                reasons.append(f'{stop}:truth_position_unverified')
            if (leg.get('heading_error_rad') is None or
                    leg['heading_error_rad'] > .40):
                reasons.append(f'{stop}:truth_heading_unverified')
            if leg.get('paths_published', 0) < 1:
                reasons.append(f'{stop}:no_nonempty_path')
            if leg.get('motion_commands', 0) < 1:
                reasons.append(f'{stop}:no_motion_command')
            if leg.get('marker_alignment', {}).get('marker_visible') is not True:
                reasons.append(f'{stop}:rear_marker_unverified')
    else:
        metrics = evidence.get('metrics') or {}
        if not any(path.get('count', 0) > 0 for path in evidence.get('paths', [])):
            reasons.append('no_nonempty_path')
        if not any(abs(cmd.get('linear', 0)) > .01 or
                   abs(cmd.get('angular', 0)) > .02
                   for cmd in evidence.get('commands', [])):
            reasons.append('no_motion_command')
        if stage.kind not in {'map_mismatch', 'sensor_outage'}:
            if metrics.get('terminal') != 'SUCCEEDED' or not metrics.get(
                    'actual_goal_within_35cm'):
                reasons.append('gazebo_goal_not_reached')
        if stage.kind == 'moving_obstacle':
            for key in ('obstacle_observed', 'route_reacted_after_obstacle',
                        'obstacle_observation_cleared', 'obstacle_removed'):
                if metrics.get(key) is not True:
                    reasons.append(f'{key}_not_proven')
            if metrics.get('obstacle_motion_steps') != 4:
                reasons.append('obstacle_motion_unverified')
            clearance = metrics.get('minimum_obstacle_clearance_m')
            if clearance is None or clearance < .02:
                reasons.append('collision_clearance_unverified')
        if stage.kind == 'sensor_outage':
            counts = evidence.get('fault_delivery', {}).get('counts', {})
            if not all(counts.get(f'{name}_dropped_outage', 0) > 0
                       for name in ('scan', 'odom', 'tf')):
                reasons.append('outage_not_injected_on_all_streams')
            if (metrics.get('safe_hold_proven') is not True or
                    metrics.get('terminal') != 'SAFE_HOLD' or
                    metrics.get('safety_block_events', 0) < 1):
                reasons.append('persistent_outage_safe_hold_not_proven')
            for key in ('outage_health_stop_latched', 'outage_mux_estop_latched',
                        'outage_goal_retained', 'outage_manual_reset_required'):
                if metrics.get(key) is not True:
                    reasons.append(key+'_not_proven')
            health = evidence.get('health') or []
            if not health or health[-1].get('autonomous_allowed') is not False:
                reasons.append('autonomous_drive_not_inhibited')
            retained = metrics.get('final_planner', {}).get('goal') or {}
            goal = evidence.get('goal') or []
            if (len(goal) < 2 or abs(retained.get('x', math.inf)-goal[0]) > .05 or
                    abs(retained.get('y', math.inf)-goal[1]) > .05):
                reasons.append('goal_not_retained_for_operator_reset')
        if stage.kind == 'map_mismatch':
            if not any(item.get('count', 0) > 0 for item in evidence.get(
                    'observed_counts', [])):
                reasons.append('map_world_difference_not_observed')
            if metrics.get('terminal') != 'SUCCEEDED' and metrics.get(
                    'safe_hold_proven') is not True:
                reasons.append('neither_arrival_nor_safe_hold')

    if stage.kind in ('route_obstacle', 'home_obstacle'):
        obstacle = evidence.get('route_obstacle') if stage.kind == 'route_obstacle' else evidence.get('home_obstacle')
        obstacle = obstacle or {}
        for key in ('spawned', 'truth_seen', 'bypassed_while_live', 'removed',
                    'route_reacted_before_removal', 'ros_observation_cleared',
                    'clearance_sampling_sufficient', 'final_safe_stop_proven'):
            if obstacle.get(key) is not True:
                reasons.append(f'{key}_not_proven')
        clearance = obstacle.get('minimum_clearance_m')
        if clearance is None or clearance < .02:
            reasons.append('collision_clearance_unverified')
    if stage.key == '15_integrated_with_recorder' and evidence.get(
            'record_incidents_enabled') is not True:
        reasons.append('flight_recorder_not_enabled')
    if stage.kind == 'wan_absent':
        namespace_file = evidence.get('staged_network_namespace')
        if (not isinstance(namespace_file, dict) or
                namespace_file.get('no_default_route') is not True or
                namespace_file.get('loopback_up') is not True):
            reasons.append('wan_isolation_not_proven')

    return {'stage': stage.key,
            'outcome': ('SAFE_HOLD' if not reasons and (
                        stage.kind == 'sensor_outage' or
                        (stage.kind == 'map_mismatch' and evidence.get(
                            'metrics', {}).get('terminal') != 'SUCCEEDED'))
                        else 'PASS' if not reasons else 'FAIL'),
            'passed': not reasons, 'reasons': reasons,
            'runner_passed': evidence.get('passed') is True,
            'route': evidence.get('route'),
            'legs': [{'stop': leg.get('stop'), 'terminal': diagnosed_leg_terminal(leg),
                      'raw_terminal': leg.get('terminal'),
                      'position_error_m': leg.get('position_error_m'),
                      'heading_error_rad': leg.get('heading_error_rad'),
                      'paths_published': leg.get('paths_published')}
                     for leg in legs]}


def assert_master_free(port: int) -> None:
    with socket.socket() as sock:
        # A just-stopped trial may leave TIME_WAIT sockets but no listener.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', port))
        except OSError as exc:
            raise RuntimeError(f'Gazebo master {port} is in use; finish the other trial first') from exc


def assert_readonly_monitor_or_free() -> None:
    """A non-ROS GET proxy may persist on 8791; a second mission owner may not."""
    with socket.socket() as sock:
        try:
            sock.bind(('127.0.0.1', 8791))
            return
        except OSError:
            pass
    try:
        with urlopen('http://127.0.0.1:8791/__readonly_proxy_health', timeout=2) as response:
            identity = json.load(response)
    except Exception as exc:
        raise RuntimeError('Port 8791 is occupied by an unverified web owner') from exc
    if identity != {'read_only_proxy': True, 'ros_owner': False,
                    'upstream_port': 8792}:
        raise RuntimeError('Port 8791 is not the isolated read-only monitor')


def selected_stages(value: str) -> list[Stage]:
    if value == 'all':
        return list(STAGES)
    names = [item.strip() for item in value.split(',')]
    unknown = sorted(set(names)-BY_KEY.keys())
    if unknown:
        raise ValueError('Unknown stage(s): '+', '.join(unknown))
    if not names or any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError('Select unique comma-separated stage names')
    return [BY_KEY[name] for name in names]


def start_timeline(output: Path):
    workspace = REPO.parents[1]
    script = REPO/'scripts/capture_classroom_safety_timeline.py'
    profile = REPO/'ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml'
    shell = (
        'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
        'ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH; '
        'source /opt/ros/humble/setup.bash; '
        f'source {shlex.quote(str(workspace/"install/local_setup.bash"))}; '
        'export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; '
        f'export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(profile))}; '
        'export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; '
        f'exec /usr/bin/python3 {shlex.quote(str(script))} '
        f'--output {shlex.quote(str(output))}')
    log = output.with_suffix('.observer.log')
    with log.open('w') as stream:
        observer = subprocess.Popen(['bash', '-c', shell], cwd=REPO,
                                    stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
    return observer, log


def start_stress_crate_observer(output: Path):
    """Record the fixed unmapped crate without publishing any ROS commands."""
    workspace = REPO.parents[1]
    script = REPO/'scripts/capture_classroom_stress_crate.py'
    profile = REPO/'ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml'
    shell = (
        'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
        'ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH; '
        'source /opt/ros/humble/setup.bash; '
        f'source {shlex.quote(str(workspace/"install/local_setup.bash"))}; '
        'export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; '
        f'export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(profile))}; '
        'export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; '
        f'exec /usr/bin/python3 {shlex.quote(str(script))} '
        f'--output {shlex.quote(str(output))}')
    log = output.with_suffix('.observer.log')
    with log.open('w') as stream:
        observer = subprocess.Popen(['bash', '-c', shell], cwd=REPO,
                                    stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
    return observer, log


def start_memory_observer(output: Path):
    """Observe remembered/live cell changes without publishing any commands."""
    workspace = REPO.parents[1]
    script = REPO/'scripts/observe_classroom_memory.py'
    profile = REPO/'ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml'
    shell = (
        'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
        'ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH; '
        'source /opt/ros/humble/setup.bash; '
        f'source {shlex.quote(str(workspace/"install/local_setup.bash"))}; '
        'export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; '
        f'export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(profile))}; '
        'export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; '
        f'exec /usr/bin/python3 {shlex.quote(str(script))} '
        f'--output {shlex.quote(str(output))}')
    log = output.with_suffix('.observer.log')
    with log.open('w') as stream:
        observer = subprocess.Popen(['bash', '-c', shell], cwd=REPO,
                                    stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
    return observer, log


def start_web_obstacle_injector(stage: Stage, directory: Path,
                               obstacle_scenario: Path | None = None):
    """Launch a Gazebo-only obstacle actor; the web server alone owns goals."""
    target = (-.25, -1.98) if stage.key in {
        '06_web_sector3_moving_obstacle',
        '25_web_inventory_sector3_moving_obstacle'} else HOME[:2]
    workspace = REPO.parents[1]
    profile = REPO/'ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml'
    script = REPO/'scripts/inject_classroom_web_obstacle.py'
    output = directory/'web_obstacle_injection.json'
    shell = (
        'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
        'ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH; '
        'source /opt/ros/humble/setup.bash; '
        f'source {shlex.quote(str(workspace/"install/local_setup.bash"))}; '
        'export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; '
        f'export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(profile))}; '
        'export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; '
        f'exec /usr/bin/python3 {shlex.quote(str(script))} '
        f'--output {shlex.quote(str(output))} '
        f'--goal-x {target[0]} --goal-y {target[1]} '
        f'--nodes-params {shlex.quote(str(directory/"classroom_nodes_sim.yaml"))}')
    if obstacle_scenario is not None:
        shell += ' --obstacle-scenario '+shlex.quote(str(obstacle_scenario))
    log = directory/'web_obstacle_injection.log'
    with log.open('w') as stream:
        actor = subprocess.Popen(['bash', '-c', shell], cwd=REPO,
                                 stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    return actor


def start_memory_transition_actor(directory: Path):
    """Create/delete only an owned Gazebo box; the web server owns all goals."""
    workspace = REPO.parents[1]
    profile = REPO/'ros2_ws/src/smart_vision_amr/config/dds_udp_localhost.xml'
    script = REPO/'scripts/inject_classroom_memory_transition.py'
    output = directory/'memory_transition_injection.json'
    shell = (
        'unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH '
        'ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH; '
        'source /opt/ros/humble/setup.bash; '
        f'source {shlex.quote(str(workspace/"install/local_setup.bash"))}; '
        'export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; '
        f'export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(profile))}; '
        'export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; '
        f'exec /usr/bin/python3 {shlex.quote(str(script))} '
        f'--output {shlex.quote(str(output))}')
    log = directory/'memory_transition_injection.log'
    with log.open('w') as stream:
        actor = subprocess.Popen(['bash', '-c', shell], cwd=REPO,
                                 stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    return actor


def web_stage_route(stage: Stage) -> str:
    route = [stop.removeprefix('Sector ') for stop in stage.route
             if stop.startswith('Sector ')]
    return ','.join(route)


def terminate_group(child: subprocess.Popen, timeout: float = 20.) -> None:
    # Every caller starts its own session. The leader can exit before a child
    # (notably Nav2) does, so poll() alone does not prove its group is gone.
    def signal_owned_group(sig: signal.Signals) -> None:
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            pass

    signal_owned_group(signal.SIGINT)
    try:
        child.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        pass
    signal_owned_group(signal.SIGTERM)
    try:
        child.wait(timeout=2.)
    except subprocess.TimeoutExpired:
        pass
    signal_owned_group(signal.SIGKILL)
    child.wait(timeout=2.)


def realism_options(stage: Stage) -> list[str]:
    """Only explicit, simulation-only model/delivery/cold-start switches."""
    options = [flag for flag in ('--cold-start', '--ekf-delivery-faults')
               if flag in stage.options]
    if '--hardware-profile' in stage.options:
        index = stage.options.index('--hardware-profile')
        options += ['--hardware-profile', stage.options[index+1]]
    return options


def web_stage_runner_command(stage: Stage, directory: Path,
                             laser_filter_for_change_memory: bool = False,
                             measure_scan_filter: bool = False,
                             nav_params_source: Path | None = None,
                             physical_footprint: bool = False,
                             nodes_params_source: Path | None = None) -> list[str]:
    dwell = stage.kind in {'web_dwell', 'web_transient', 'web_moving_obstacle',
                           'web_reentry'}
    command = [str(RUNNER), '--no-verifier', '--lifetime',
               '2000' if stage.kind == 'web_reentry' else
               '1400' if dwell else '180', '--mode',
               'ekf_independent' if dwell else 'baseline',
               '--output', str(directory)]
    if dwell:
        command.append('--record-incidents')
    if '--stress' in stage.options:
        command.append('--stress')
    if '--memory-session' in stage.options:
        command.append('--memory-session')
    if laser_filter_for_change_memory:
        command.append('--laser-filter-for-change-memory')
    if measure_scan_filter:
        command.append('--measure-scan-filter')
    if nav_params_source is not None:
        command += ['--nav-params-source', str(nav_params_source)]
    if physical_footprint:
        command.append('--physical-footprint')
    if nodes_params_source is not None:
        command += ['--nodes-params-source', str(nodes_params_source)]
    command += realism_options(stage)
    return command


def execute_web_stage(stage: Stage, root: Path,
                      laser_filter_for_change_memory: bool = False,
                      measure_scan_filter: bool = False,
                      nav_params_source: Path | None = None,
                      physical_footprint: bool = False,
                      nodes_params_source: Path | None = None,
                      obstacle_scenario: Path | None = None) -> int:
    assert_master_free(8792)
    # A second web_mission_server is also a potential control publisher.
    # The operator observes the mission on this stage's 8792 owner instead.
    assert_readonly_monitor_or_free()
    directory = root/stage.key
    timeline_file = root/(stage.key+'.safety_timeline.jsonl')
    observer, observer_log = start_timeline(timeline_file)
    stress_file = root/(stage.key+'.stress_crate_trace.jsonl')
    stress_observer, stress_log = (start_stress_crate_observer(stress_file)
                                   if '--stress' in stage.options else (None, None))
    memory_file = root/(stage.key+'.memory_observer.jsonl')
    memory_observer, memory_log = (start_memory_observer(memory_file)
                                   if '--memory-session' in stage.options else (None, None))
    runner = None
    web = None
    injector = None
    web_log = root/(stage.key+'.web.log')
    try:
        dwell = stage.kind in {'web_dwell', 'web_transient', 'web_moving_obstacle',
                               'web_reentry'}
        command = web_stage_runner_command(
            stage, directory, laser_filter_for_change_memory,
            measure_scan_filter, nav_params_source, physical_footprint,
            nodes_params_source)
        runner = subprocess.Popen(command, cwd=REPO, start_new_session=True)
        deadline = time.monotonic()+15.
        while not directory.is_dir() and time.monotonic() < deadline:
            if runner.poll() is not None:
                return runner.returncode or 1
            time.sleep(.2)
        if not directory.is_dir():
            return 1
        if stage.kind == 'web_moving_obstacle':
            injector = start_web_obstacle_injector(stage, directory, **(
                {'obstacle_scenario': obstacle_scenario} if obstacle_scenario is not None else {}))
        elif stage.key == '24_web_memory_sector1_to_sector2_removal':
            injector = start_memory_transition_actor(directory)
        environment = dict(os.environ)
        environment.update({'SMART_VISION_CLASSROOM_SIM_DOMAIN': '176',
                            'SMART_VISION_CLASSROOM_SIM_PORT': '8792',
                            'SMART_VISION_CLASSROOM_SIM_CONFIG':
                                str(root/(stage.key+'.web-config.json'))})
        if physical_footprint:
            environment['SMART_VISION_CLASSROOM_SIM_ROTATION_RADIUS'] = '0.22'
        environment['SMART_VISION_CLASSROOM_SIM_FRONT_CAMERA_TOPIC'] = ''
        web_command = [str(REPO/'scripts/run_classroom_replay_web.sh'),
                       '--host', '127.0.0.1']
        if dwell:
            web_command += ['--require-battery-health',
                            '--minimum-battery-voltage', '10.8']
        with web_log.open('w') as stream:
            web = subprocess.Popen(web_command, cwd=REPO, env=environment,
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        verifier_name = ('verify_classroom_web_reentry.py'
                         if stage.kind == 'web_reentry' else
                         'verify_classroom_web_dwell.py' if dwell else
                         'verify_classroom_web_disconnect.py')
        verifier = [sys.executable, str(REPO/'scripts'/verifier_name),
                    '--url', 'http://127.0.0.1:8792',
                    '--output', str(directory/'acceptance.json')]
        if stage.kind == 'web_reentry':
            verifier += ['--web-pid', str(web.pid), '--runner-pid', str(runner.pid),
                         '--mission-timeout', '900']
            if '--inventory' in stage.options:
                verifier += ['--dispatch-mode', 'inventory']
        elif dwell:
            verifier += ['--route', web_stage_route(stage),
                         '--mission-timeout', '1200']
            if '--inventory-before-prearm' in stage.options:
                verifier += ['--dispatch-mode', 'inventory', '--inventory-before-prearm',
                             '--readiness-timeout', '180']
            if '--stress' in stage.options:
                verifier += ['--obstacle-hold-timeout', '120']
            if stage.kind == 'web_transient':
                verifier += ['--inject-transient']
        if '--cold-start' in stage.options:
            verifier.append('--cold-start')
        if '--ekf-delivery-faults' in stage.options:
            verifier.append('--ekf-delivery-faults')
        verdict = subprocess.run(verifier, cwd=REPO, env={**os.environ,
                                 'ROS_DOMAIN_ID': '176', 'ROS_LOCALHOST_ONLY': '1'},
                                 check=False, timeout=(1950 if stage.kind == 'web_reentry'
                                                       else 1300 if dwell else 145)).returncode
        if injector is not None:
            try:
                # The actor may need up to 40 s of LiDAR observations after
                # removal to prove that the red obstacle cells cleared.
                actor_verdict = injector.wait(timeout=50.)
            except subprocess.TimeoutExpired:
                actor_verdict = 124
            if actor_verdict != 0:
                verdict = verdict or actor_verdict
        return verdict
    finally:
        if injector is not None:
            terminate_group(injector, timeout=8.)
        if web is not None:
            terminate_group(web)
        if runner is not None:
            terminate_group(runner)
        terminate_group(observer, timeout=8.)
        if stress_observer is not None:
            terminate_group(stress_observer, timeout=8.)
        if memory_observer is not None:
            terminate_group(memory_observer, timeout=8.)
        if directory.is_dir():
            for source, destination in (
                    (timeline_file, directory/'safety_timeline.jsonl'),
                    (observer_log, directory/'safety_observer.log'),
                    (web_log, directory/'web_server.log')):
                if source.is_file():
                    shutil.move(str(source), destination)
            if stress_observer is not None:
                for source, destination in (
                        (stress_file, directory/'stress_crate_trace.jsonl'),
                        (stress_log, directory/'stress_crate_observer.log')):
                    if source.is_file():
                        shutil.move(str(source), destination)
            if memory_observer is not None:
                for source, destination in (
                        (memory_file, directory/'memory_observer.jsonl'),
                        (memory_file.with_suffix('.summary.json'),
                         directory/'memory_observer.summary.json'),
                        (memory_log, directory/'memory_observer.log')):
                    if source.is_file():
                        shutil.move(str(source), destination)


def stage_runner_command(stage: Stage, directory: Path, leg_timeout: int,
                         obstacle_hold_timeout: int,
                         tf_source_best_effort: bool = False,
                         laser_filter_for_change_memory: bool = False,
                         measure_scan_filter: bool = False,
                         nav_params_source: Path | None = None,
                         physical_footprint: bool = False,
                         nodes_params_source: Path | None = None) -> list[str]:
    command = [str(RUNNER), '--mode', 'baseline', '--profile', 'clean',
               '--timeout', str(leg_timeout),
               '--obstacle-hold-timeout', str(obstacle_hold_timeout),
               '--output', str(directory), *stage.options]
    if tf_source_best_effort:
        command.append('--tf-source-best-effort')
    if laser_filter_for_change_memory:
        command.append('--laser-filter-for-change-memory')
    if measure_scan_filter:
        command.append('--measure-scan-filter')
    if nav_params_source is not None:
        command += ['--nav-params-source', str(nav_params_source)]
    if physical_footprint:
        command.append('--physical-footprint')
    if nodes_params_source is not None:
        command += ['--nodes-params-source', str(nodes_params_source)]
    return command


def execute_stage(stage: Stage, root: Path, leg_timeout: int,
                  obstacle_hold_timeout: int,
                  tf_source_best_effort: bool = False,
                  laser_filter_for_change_memory: bool = False,
                  measure_scan_filter: bool = False,
                  nav_params_source: Path | None = None,
                  physical_footprint: bool = False,
                  nodes_params_source: Path | None = None,
                  obstacle_scenario: Path | None = None) -> int:
    assert_master_free(MASTER_PORT)
    if stage.kind in {'web_disconnect', 'web_dwell', 'web_transient',
                      'web_moving_obstacle', 'web_reentry'}:
        return execute_web_stage(stage, root, laser_filter_for_change_memory,
                                 measure_scan_filter, nav_params_source, physical_footprint,
                                 nodes_params_source, **(
                                     {'obstacle_scenario': obstacle_scenario}
                                     if obstacle_scenario is not None else {}))
    directory = root/stage.key
    command = stage_runner_command(stage, directory, leg_timeout,
                                   obstacle_hold_timeout, tf_source_best_effort,
                                   laser_filter_for_change_memory, measure_scan_filter,
                                   nav_params_source, physical_footprint,
                                   nodes_params_source)
    print('\n'+stage.key+': '+' '.join(command), flush=True)
    timeline_file = root/(stage.key+'.safety_timeline.jsonl')
    observer, observer_log = start_timeline(timeline_file)
    time.sleep(.5)
    try:
        child = subprocess.Popen(command, cwd=REPO, start_new_session=True)
        maximum = 240 + leg_timeout*max(1, len(stage.route))
        try:
            return child.wait(timeout=maximum)
        except subprocess.TimeoutExpired:
            print(f'{stage.key}: exceeded {maximum}s outer trial bound',
                  file=sys.stderr, flush=True)
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)
            return 124
        except KeyboardInterrupt:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)
            raise
    finally:
        terminate_group(observer, timeout=8.)
        if directory.is_dir():
            if timeline_file.is_file():
                shutil.move(str(timeline_file), directory/'safety_timeline.jsonl')
            if observer_log.is_file():
                shutil.move(str(observer_log), directory/'safety_observer.log')


def run_in_network_namespace(stage: Stage, root: Path, leg_timeout: int,
                             obstacle_hold_timeout: int,
                             laser_filter_for_change_memory: bool = False,
                             measure_scan_filter: bool = False,
                             nav_params_source: Path | None = None,
                             physical_footprint: bool = False,
                             nodes_params_source: Path | None = None,
                             obstacle_scenario: Path | None = None) -> int:
    # Isolate the entire Gazebo/DDS/observer process tree in *one* new user
    # and network namespace. Host Wi-Fi, domain 16, and host routing stay intact.
    env = dict(os.environ)
    env['SVAM_STAGE_NET_EVIDENCE'] = str(root/(stage.key+'.network.txt'))
    shell = ('ip link set lo up && '
             '{ ip -o link show lo; ip -br addr; ip route; } > "$SVAM_STAGE_NET_EVIDENCE" && '
             'exec "$@"')
    command = ['unshare', '-Urn', '/bin/bash', '-c', shell, 'staged-net',
               sys.executable, str(Path(__file__).resolve()), '--run-inner', stage.key,
               '--output', str(root), '--leg-timeout', str(leg_timeout),
               '--obstacle-hold-timeout', str(obstacle_hold_timeout)]
    if laser_filter_for_change_memory:
        command.append('--laser-filter-for-change-memory')
    if measure_scan_filter:
        command.append('--measure-scan-filter')
    if nav_params_source is not None:
        command += ['--nav-params-source', str(nav_params_source)]
    if physical_footprint:
        command.append('--physical-footprint')
    if nodes_params_source is not None:
        command += ['--nodes-params-source', str(nodes_params_source)]
    if obstacle_scenario is not None:
        command += ['--obstacle-scenario', str(obstacle_scenario)]
    if '--memory-session' in stage.options:
        command.append('--memory-session')
    command += realism_options(stage)
    return subprocess.run(command, cwd=REPO, env=env, check=False).returncode


def web_truth_xy_tolerance(nav_config: dict) -> float:
    """Keep the independent 25 cm arrival requirement fixed across controller tuning.

    A tighter internal Nav2 stopping band reserves room for localization error;
    it must not move the independent world-position goalposts as well.
    """
    effective_nav2_goal_tolerance(nav_config)
    return MAX_WEB_TRUTH_XY_TOLERANCE_M


def effective_nav2_goal_tolerance(nav_config: dict) -> float:
    """Validate the internal stopping band is no looser than external QA."""
    try:
        controller = nav_config['controller_server']['ros__parameters']
        plugins = controller['goal_checker_plugins']
        if not isinstance(plugins, list) or len(plugins) != 1 or not isinstance(
                plugins[0], str):
            raise ValueError('expected exactly one Nav2 goal checker plugin')
        tolerance = controller[plugins[0]]['xy_goal_tolerance']
    except (KeyError, TypeError, IndexError) as exc:
        raise ValueError('effective Nav2 xy_goal_tolerance missing') from exc
    if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or
            not math.isfinite(tolerance) or tolerance <= 0 or
            tolerance > MAX_WEB_TRUTH_XY_TOLERANCE_M):
        raise ValueError('effective Nav2 xy_goal_tolerance invalid or above '
                         f'{MAX_WEB_TRUTH_XY_TOLERANCE_M:.2f} m')
    return float(tolerance)


def check_web_ground_truth(records: list[dict], evidence: dict,
                           position_tolerance_m: float) -> dict:
    """Cross-check web marker/home events against independent Gazebo truth."""
    marker_wall_front_yaws = {}
    world = REPO/'ros2_ws/src/smart_vision_amr/worlds/classroom_20261030_stress.world'
    try:
        root = ET.parse(world).getroot()
        for marker_id in (1, 2, 3):
            pose = root.find(f".//model[@name='classroom_marker_{marker_id}']/pose")
            if pose is not None and pose.text:
                wall_yaw = float(pose.text.split()[5])
                marker_wall_front_yaws[str(marker_id)] = math.atan2(
                    math.sin(wall_yaw+math.pi), math.cos(wall_yaw+math.pi))
    except (OSError, ET.ParseError, ValueError, IndexError):
        pass
    truth = [record for record in records if record.get('kind') == 'truth'
             and isinstance(record.get('wall_time'), (int, float))]
    checks = {}
    for marker_id in (*evidence.get('route', []), 'home'):
        key = str(marker_id)
        if marker_id == 'home':
            at = evidence.get('final_observed_at')
            goal = evidence.get('home_target')
            goal = ([goal.get('x'), goal.get('y'), goal.get('yaw')]
                    if isinstance(goal, dict) else None)
        else:
            at = evidence.get('dwell', {}).get(key, {}).get('start_event_at')
            goal = evidence.get('marker_targets', {}).get(key)
        item = {'passed': False, 'event_at': at, 'goal': goal,
                'position_tolerance_m': position_tolerance_m}
        if (isinstance(at, (int, float)) and isinstance(goal, (list, tuple))
                and len(goal) == 3 and all(isinstance(value, (int, float))
                                           for value in goal) and truth):
            closest = min(truth, key=lambda value: abs(value['wall_time']-at))
            delta = abs(closest['wall_time']-at)
            position = math.hypot(closest['x']-goal[0], closest['y']-goal[1])
            heading = abs(math.atan2(math.sin(closest['yaw']-goal[2]),
                                     math.cos(closest['yaw']-goal[2])))
            wall_front_yaw = marker_wall_front_yaws.get(key)
            wall_error = (abs(math.atan2(math.sin(closest['yaw']-wall_front_yaw),
                                        math.cos(closest['yaw']-wall_front_yaw)))
                          if wall_front_yaw is not None else None)
            item.update({'sample_time_delta_s': round(delta, 3),
                         'position_error_m': round(position, 3),
                         'heading_error_rad': round(heading, 3),
                         'fixture_wall_front_yaw_rad': wall_front_yaw,
                         'fixture_wall_normal_error_rad': (round(wall_error, 3)
                                                           if wall_error is not None else None),
                         'truth': [closest['x'], closest['y'], closest['yaw']],
                         'passed': (delta <= .75 and
                                    position <= position_tolerance_m and heading <= .25
                                    and (marker_id == 'home' or
                                         (wall_error is not None and wall_error <= .25)))})
        checks[key] = item
    return checks


def check_web_dwell_independent(records: list[dict], evidence: dict) -> dict:
    """Prove continuous Gazebo stillness and next ROS goal after each dwell."""
    truth = [record for record in records if record.get('kind') == 'truth'
             and isinstance(record.get('wall_time'), (int, float))]
    commands = [record for record in records if record.get('kind') == 'cmd_vel'
                and isinstance(record.get('wall_time'), (int, float))]
    goals = [record for record in records if record.get('kind') == 'goal'
             and isinstance(record.get('wall_time'), (int, float))]
    route = evidence.get('route', [])
    checks = {}
    for index, marker_id in enumerate(route):
        key = str(marker_id)
        dwell = evidence.get('dwell', {}).get(key, {})
        start, end = dwell.get('start_event_at'), dwell.get('end_event_at')
        item = {'passed': False, 'start_event_at': start, 'end_event_at': end}
        checks[key] = item
        if not isinstance(start, (int, float)) or not isinstance(end, (int, float)):
            item['reason'] = 'dwell_event_missing'
            continue
        if end-start < 4.9:
            item['reason'] = 'dwell_shorter_than_five_seconds'
            continue
        samples = [sample for sample in truth if start <= sample['wall_time'] <= end]
        cmds = [sample for sample in commands if start <= sample['wall_time'] <= end]
        item['truth_samples'] = len(samples)
        item['command_samples'] = len(cmds)
        if len(samples) < 20 or len(cmds) < 10:
            item['reason'] = 'independent_sample_coverage_missing'
            continue
        truth_gap = max(b['wall_time']-a['wall_time']
                        for a, b in zip(samples, samples[1:]))
        command_gap = max(b['wall_time']-a['wall_time']
                          for a, b in zip(cmds, cmds[1:]))
        start_delta = samples[0]['wall_time']-start
        end_delta = end-samples[-1]['wall_time']
        item.update({'truth_max_sample_gap_s': round(truth_gap, 3),
                     'command_max_sample_gap_s': round(command_gap, 3),
                     'truth_start_delta_s': round(start_delta, 3),
                     'truth_end_delta_s': round(end_delta, 3)})
        if (truth_gap > .35 or command_gap > .5 or start_delta > .3 or
                end_delta > .3):
            item['reason'] = 'continuous_observation_missing'
            continue
        first = samples[0]
        max_delta = max(math.hypot(sample['x']-first['x'],
                                   sample['y']-first['y']) for sample in samples)
        max_heading = max(abs(math.atan2(
            math.sin(sample['yaw']-first['yaw']),
            math.cos(sample['yaw']-first['yaw']))) for sample in samples)
        max_linear = max(abs(sample['linear']) for sample in cmds)
        max_angular = max(abs(sample['angular']) for sample in cmds)
        item.update({'gazebo_max_translation_m': round(max_delta, 4),
                     'gazebo_max_heading_rad': round(max_heading, 4),
                     'command_max_linear_mps': round(max_linear, 4),
                     'command_max_angular_rps': round(max_angular, 4)})
        if (max_delta > .025 or max_heading > .06 or max_linear > .015 or
                max_angular > .04):
            item['reason'] = 'robot_or_command_moved_during_dwell'
            continue
        premature = [goal for goal in goals if start <= goal['wall_time'] < end]
        if premature:
            item['reason'] = 'next_goal_published_before_dwell_end'
            item['premature_goal_count'] = len(premature)
            continue
        next_key = str(route[index+1]) if index+1 < len(route) else 'home'
        next_target = (evidence.get('marker_targets', {}).get(next_key)
                       if next_key != 'home' else evidence.get('home_target'))
        if isinstance(next_target, dict):
            next_target = [next_target.get('x'), next_target.get('y'),
                           next_target.get('yaw')]
        if (not isinstance(next_target, (list, tuple)) or len(next_target) != 3 or
                not all(isinstance(value, (int, float)) for value in next_target)):
            item['reason'] = 'next_target_missing'
            continue
        following = [goal for goal in goals if end <= goal['wall_time'] <= end+10.]
        if not following:
            item['reason'] = 'next_ros_goal_not_published_after_dwell'
            continue
        next_goal = following[0]
        item['next_ros_goal_at'] = next_goal['wall_time']
        item['next_ros_goal_delay_s'] = round(next_goal['wall_time']-end, 3)
        item['next_target'] = next_target
        position_error = math.hypot(next_goal['x']-next_target[0],
                                    next_goal['y']-next_target[1])
        heading_error = abs(math.atan2(
            math.sin(next_goal['yaw']-next_target[2]),
            math.cos(next_goal['yaw']-next_target[2])))
        item['next_ros_goal_position_error_m'] = round(position_error, 4)
        item['next_ros_goal_heading_error_rad'] = round(heading_error, 4)
        item['passed'] = position_error <= .05 and heading_error <= .25
        if not item['passed']:
            item['reason'] = 'next_ros_goal_does_not_match_route'
    return checks


def check_home_settle_independent(records: list[dict], evidence: dict) -> dict:
    """Require independent Gazebo truth and zero commands during terminal settling."""
    end = evidence.get('final_observed_at')
    duration = evidence.get('home_stationary_duration_s')
    result = {'passed': False, 'end_at': end, 'duration_s': duration}
    if not isinstance(end, (int, float)) or not isinstance(duration, (int, float)):
        return result
    start = end-duration
    truth = [item for item in records if item.get('kind') == 'truth' and
             start <= item.get('wall_time', -math.inf) <= end]
    commands = [item for item in records if item.get('kind') == 'cmd_vel' and
                start <= item.get('wall_time', -math.inf) <= end]
    result['truth_samples'] = len(truth)
    result['command_samples'] = len(commands)
    if len(truth) < 5 or len(commands) < 5:
        return result
    truth_gap = max(b['wall_time']-a['wall_time'] for a, b in zip(truth, truth[1:]))
    command_gap = max(b['wall_time']-a['wall_time'] for a, b in zip(commands, commands[1:]))
    first = truth[0]
    max_translation = max(math.hypot(item['x']-first['x'], item['y']-first['y'])
                          for item in truth)
    max_heading = max(abs(math.atan2(math.sin(item['yaw']-first['yaw']),
                                     math.cos(item['yaw']-first['yaw']))) for item in truth)
    max_linear = max(abs(item['linear']) for item in commands)
    max_angular = max(abs(item['angular']) for item in commands)
    result.update({'truth_max_gap_s': round(truth_gap, 3),
                   'command_max_gap_s': round(command_gap, 3),
                   'max_translation_m': round(max_translation, 4),
                   'max_heading_rad': round(max_heading, 4),
                   'max_command_linear_mps': round(max_linear, 4),
                   'max_command_angular_rps': round(max_angular, 4),
                   'passed': (duration >= 1.2 and
                              truth[0]['wall_time']-start <= .3 and
                              end-truth[-1]['wall_time'] <= .3 and
                              commands[0]['wall_time']-start <= .3 and
                              end-commands[-1]['wall_time'] <= .3 and
                              truth_gap <= .35 and command_gap <= .5 and
                              max_translation <= .025 and max_heading <= .06 and
                              max_linear <= .015 and max_angular <= .04)})
    return result


def check_web_transient(records: list[dict], relay_file: Path,
                        evidence: dict) -> dict:
    """Correlate a real web-owned goal with injected delays and auto-resume."""
    result = {'passed': False, 'reasons': []}
    reasons = result['reasons']
    requested = evidence.get('transient_profile_request') or {}
    if (evidence.get('transient_injected_mid_route') is not True or
            requested.get('profile_requested') != 'transient'):
        reasons.append('mid_route_web_fault_not_requested')
        return result
    if not relay_file.is_file():
        reasons.append('fault_relay_evidence_missing')
        return result
    try:
        relay = [json.loads(line) for line in relay_file.read_text().splitlines()
                 if line.strip()]
    except (OSError, ValueError):
        reasons.append('fault_relay_evidence_invalid')
        return result
    profile = next((item for item in relay if item.get('event') == 'profile'
                    and item.get('name') == 'transient' and
                    isinstance(item.get('monotonic'), (int, float)) and
                    abs(item['monotonic']-requested.get(
                        'requested_monotonic', math.inf)) <= 2.), None)
    if profile is None:
        reasons.append('transient_profile_not_applied')
        return result
    profile_at = profile['monotonic']
    result['profile_applied_monotonic'] = profile_at
    initial_goal = evidence.get('marker_targets', {}).get('1')
    first_dwell_at = evidence.get('dwell', {}).get('1', {}).get('start_event_at')
    if not isinstance(first_dwell_at, (int, float)) or not initial_goal:
        reasons.append('first_web_sector_evidence_missing')
        return result
    goals = [item for item in records if item.get('kind') == 'goal' and
             isinstance(item.get('monotonic'), (int, float))]
    first_sector_goals = [item for item in goals if item.get('wall_time', math.inf)
                          < first_dwell_at and math.hypot(
                              item.get('x', math.inf)-initial_goal[0],
                              item.get('y', math.inf)-initial_goal[1]) <= .05]
    result['first_sector_goal_count'] = len(first_sector_goals)
    if len(first_sector_goals) != 1 or first_sector_goals[0]['monotonic'] >= profile_at:
        reasons.append('web_goal_not_unique_and_prior_to_fault')
    other_goals = [item for item in goals if item.get('wall_time', math.inf)
                   < first_dwell_at and item not in first_sector_goals]
    if other_goals:
        reasons.append('unexpected_goal_before_first_sector')

    last_sample = next((item for item in reversed(relay)
                        if item.get('event') == 'sample'), {})
    before_sample = next((item for item in reversed(relay)
                          if item.get('event') == 'sample' and
                          item.get('monotonic', math.inf) < profile_at), {})
    counts = last_sample.get('counts', {})
    before_counts = before_sample.get('counts', {})
    delay = last_sample.get('max_delivery_delay_s', {})
    baseline_delay = before_sample.get('max_delivery_delay_s', {})
    delivery = {}
    for stream in ('scan', 'odom', 'tf'):
        drops = sum(value-before_counts.get(key, 0)
                    for key, value in counts.items()
                    if key.startswith(stream+'_dropped_transient'))
        extra_delay = delay.get(stream, 0.)-baseline_delay.get(stream, 0.)
        delivery[stream] = {'dropped_transient': drops,
                            'max_delivery_delay_s': delay.get(stream),
                            'increase_over_clean_s': round(extra_delay, 3),
                            'fault_observed': drops > 0 or extra_delay >= .22}
        if not delivery[stream]['fault_observed']:
            reasons.append(stream+'_transient_delivery_not_proven')
    result['fault_delivery'] = delivery

    continuity = evidence.get('mission_continuity') or {}
    result['mission_continuity'] = continuity
    if (continuity.get('samples', 0) < 10 or
            continuity.get('route_preserved') is not True or
            continuity.get('completed_prefix_valid') is not True or
            continuity.get('start_event_count') != 1 or
            continuity.get('cancel_event_count') != 0):
        reasons.append('original_web_mission_not_preserved')
    if any(continuity.get('marker_confirmations', {}).get(str(marker)) != 1
           for marker in (1, 2)):
        reasons.append('duplicate_or_missing_sector_delivery')

    fault_window = [item for item in records if isinstance(item.get('monotonic'),
                    (int, float)) and profile_at <= item['monotonic'] <= profile_at+5.]
    commands = [item for item in fault_window if item.get('kind') == 'cmd_vel']
    truth = [item for item in fault_window if item.get('kind') == 'truth']
    health = [item for item in fault_window if item.get('kind') == 'health']
    nav = [item for item in fault_window if item.get('kind') == 'navigation']
    result['fault_window_samples'] = {'cmd_vel': len(commands), 'truth': len(truth),
                                      'health': len(health), 'navigation': len(nav)}
    if len(commands) < 10 or len(truth) < 10 or not health:
        reasons.append('fault_window_observation_missing')
    else:
        max_linear = max(abs(item['linear']) for item in commands)
        max_angular = max(abs(item['angular']) for item in commands)
        result['fault_window_command_max'] = [round(max_linear, 3),
                                              round(max_angular, 3)]
        command_limits = evidence.get('runtime_command_limits') or {
            'linear_mps': .17, 'angular_radps': 1.05,
            'source': 'legacy evidence without captured runtime profile'}
        result['fault_window_command_limits'] = command_limits
        linear_limit, angular_limit = (command_limits.get('linear_mps'),
                                       command_limits.get('angular_radps'))
        if (not all(type(value) in (int, float) and math.isfinite(value) and value > 0
                    for value in (linear_limit, angular_limit)) or
                max_linear > linear_limit+1e-6 or max_angular > angular_limit+1e-6):
            reasons.append('unbounded_command_during_transient')
        if any(item.get('stop_latched') is True for item in health):
            reasons.append('short_transient_caused_emergency_latch')
        blocked = next((item for item in health if
                        item.get('autonomous_allowed') is False and
                        item['monotonic'] >= profile_at+1.4), None)
        recovered = next((item for item in health if blocked is not None and
                          item['monotonic'] > blocked['monotonic'] and
                          item.get('autonomous_allowed') is True), None)
        result['health_hold_seen'] = blocked is not None
        result['health_auto_recovered'] = recovered is not None
        if blocked is None or recovered is None:
            reasons.append('bounded_localization_hold_and_recovery_not_proven')
        else:
            hold_start, hold_end = blocked['monotonic'], recovered['monotonic']
            during = [item for item in commands if hold_start <=
                      item['monotonic'] < hold_end]
            zero = [item for item in during if item['monotonic'] <= hold_start+.25
                    and abs(item.get('linear', math.inf)) <= .015 and
                    abs(item.get('angular', math.inf)) <= .04]
            late_nonzero = [item for item in during if item['monotonic'] >
                            hold_start+.25 and (abs(item.get('linear', 0.)) > .015 or
                            abs(item.get('angular', 0.)) > .04)]
            result['zero_command_latency_s'] = (round(zero[0]['monotonic']-hold_start, 3)
                                                if zero else None)
            result['late_nonzero_commands_in_hold'] = len(late_nonzero)
            if not zero or late_nonzero:
                reasons.append('zero_command_during_invalid_health_not_proven')
            before = min(truth, key=lambda item: abs(item['monotonic']-hold_start))
            after = min(truth, key=lambda item: abs(item['monotonic']-hold_end))
            drift = math.hypot(after['x']-before['x'], after['y']-before['y'])
            yaw_drift = abs(math.atan2(math.sin(after['yaw']-before['yaw']),
                                       math.cos(after['yaw']-before['yaw'])))
            result['health_hold_truth_drift_m'] = round(drift, 4)
            result['health_hold_truth_yaw_rad'] = round(yaw_drift, 4)
            if (abs(before['monotonic']-hold_start) > .2 or
                    abs(after['monotonic']-hold_end) > .2 or
                    drift > .05 or yaw_drift > .12):
                reasons.append('bounded_ground_truth_hold_not_proven')
        # Internal FollowPath cancellation/reissue is expected when its input
        # guard catches stale scan/odom. It may report WAIT_CANCEL, CANCELED,
        # or FAILED; only a manual cancellation or loss of the web mission is
        # a user-visible mission failure.
        cancel_overlap = [item for item in nav if item.get('input_hold') is True
                          or item.get('status') in {'WAIT_CANCEL', 'CANCELED', 'FAILED'}]
        result['follower_cancel_overlap_seen'] = bool(cancel_overlap)
        result['follower_cancel_statuses'] = [item.get('status') for item in cancel_overlap]
        if not cancel_overlap:
            reasons.append('follower_cancel_overlap_not_observed')
        if any(item.get('manual_stop') is True or item.get('status') == 'SAFETY_STOP'
               for item in nav):
            reasons.append('manual_or_latched_navigation_stop_during_transient')
        nav_goal_stamps = [json.dumps(item['goal_stamp'], sort_keys=True)
                           for item in nav if item.get('goal_stamp')]
        result['nav_goal_stamp_count'] = len(set(nav_goal_stamps))
        if len(set(nav_goal_stamps)) > 1:
            reasons.append('navigation_goal_changed_during_transient')
        if cancel_overlap and not any(
                item.get('status') in {'MOVING', 'FOLLOWING', 'ACTIVE'} and
                item['monotonic'] > cancel_overlap[-1]['monotonic']
                for item in records if item.get('kind') == 'navigation' and
                isinstance(item.get('monotonic'), (int, float))):
            reasons.append('follower_goal_not_reissued_after_input_hold')

    later_motion = [item for item in records if item.get('kind') == 'cmd_vel'
                    and isinstance(item.get('monotonic'), (int, float))
                    and profile_at+2.4 <= item['monotonic'] <= profile_at+30.
                    and abs(item.get('linear', 0.)) > .03]
    later_health = [item for item in records if item.get('kind') == 'health'
                    and isinstance(item.get('monotonic'), (int, float))
                    and profile_at+2.4 <= item['monotonic'] <= profile_at+30.
                    and item.get('autonomous_allowed') is True]
    result['post_fault_motion_seen'] = bool(later_motion)
    result['post_fault_localization_allowed'] = bool(later_health)
    if not later_motion or not later_health:
        reasons.append('automatic_resume_after_transient_not_proven')
    if evidence.get('final_state') != 'HOME_REACHED':
        reasons.append('web_mission_not_completed_after_transient')
    result['passed'] = not reasons
    return result


def check_web_disconnect_truth(records: list[dict], evidence: dict) -> dict:
    truth = [record for record in records if record.get('kind') == 'truth'
             and isinstance(record.get('wall_time'), (int, float))]
    result = {'passed': False}
    start = evidence.get('disconnect_start_at')
    end = evidence.get('disconnect_end_at')
    if not truth or not isinstance(start, (int, float)) or not isinstance(
            end, (int, float)):
        return result
    before = min(truth, key=lambda record: abs(record['wall_time']-start))
    after = min(truth, key=lambda record: abs(record['wall_time']-end))
    first_delta = abs(before['wall_time']-start)
    last_delta = abs(after['wall_time']-end)
    displacement = math.hypot(after['x']-before['x'], after['y']-before['y'])
    result.update({'displacement_m': round(displacement, 3),
                   'start_sample_delta_s': round(first_delta, 3),
                   'end_sample_delta_s': round(last_delta, 3),
                   'passed': first_delta <= .75 and last_delta <= .75 and
                   displacement >= .08})
    return result


def check_web_obstacle_overlay(evidence: dict) -> dict:
    """Correlate the red map cells with this trial's owned obstacle lifetime."""
    obstacle = evidence.get('web_obstacle') or {}
    events = obstacle.get('events') or []
    spawned = next((item.get('at') for item in events
                    if item.get('event') == 'obstacle_spawned'), None)
    removed = next((item.get('at') for item in events
                    if item.get('event') == 'obstacle_removed'), None)
    indices = set(obstacle.get('observed_indices_live') or [])
    trace = evidence.get('observed_obstacle_overlay_trace') or []
    result = {'red_cells_while_live': False,
              'red_cells_cleared_after_removal': False,
              'spawned_at': spawned, 'removed_at': removed,
              'owned_observed_cells': sorted(indices)}
    if not (isinstance(spawned, (int, float)) and
            isinstance(removed, (int, float)) and indices):
        return result
    result['red_cells_while_live'] = any(
        spawned <= sample.get('at', -math.inf) <= removed and
        sample.get('renderable') is True and
        bool(indices & set(sample.get('cells') or [])) for sample in trace)
    later = [sample for sample in trace
             if sample.get('at', -math.inf) >= removed+2.]
    result['red_cells_cleared_after_removal'] = bool(later) and (
        later[-1].get('renderable') is True and not
        (indices & set(later[-1].get('cells') or [])))
    return result


def check_home_obstacle_timing(evidence: dict) -> dict:
    """Prove the box existed only after Sector 3's dwell and Home goal."""
    events = ((evidence.get('web_obstacle') or {}).get('events') or [])
    def at(name):
        return next((event.get('at') for event in events
                     if event.get('event') == name), None)
    sector_end = ((evidence.get('dwell') or {}).get('3') or {}).get('end_event_at')
    home_goal = at('web_goal_observed')
    spawned = at('obstacle_spawned')
    removed = at('obstacle_removed')
    final = evidence.get('final_observed_at')
    moments = (sector_end, home_goal, spawned, removed, final)
    passed = (all(isinstance(value, (int, float)) and math.isfinite(value)
                  for value in moments) and
              sector_end <= home_goal+.2 and
              home_goal < spawned < removed < final)
    return {'passed': passed, 'sector_3_dwell_end_at': sector_end,
            'home_ros_goal_at': home_goal, 'obstacle_spawned_at': spawned,
            'obstacle_removed_at': removed, 'home_arrival_observed_at': final}


def check_physical_source_value_parity(directory: Path, manifest: dict) -> dict:
    """Verify the exact declared Gazebo geometry overrides of physical tuning.

    This verifies source values, not the robot's live ROS parameters or its
    different OpenCR, network, camera and process topology.
    """
    config = REPO/'ros2_ws/src/smart_vision_amr/config'
    sources = {
        'nav': Path(manifest.get('nav_profile_source') or
                    config/'curve_tuning_waffle_pi.yaml').expanduser().resolve(),
        'nodes': Path(manifest.get('nodes_profile_source') or
                      config/'classroom_hardware_nodes.yaml').expanduser().resolve(),
        'ekf': config/'ekf_hardware.yaml',
    }
    authoritative = {
        'nav': (config/'classroom_demo_operator_20261104.yaml').resolve(),
        'nodes': (config/'classroom_hardware_nodes.yaml').resolve(),
    }
    authoritative_checks = {
        name+'_source_selected': sources[name] == path
        for name, path in authoritative.items()}
    generated_nav = directory/'classroom_nav_sim_time.yaml'
    generated_nodes = directory/'classroom_nodes_sim.yaml'
    checks = {'source_files_available': all(path.is_file() for path in sources.values()),
              'generated_nav_available': generated_nav.is_file(),
              'generated_nodes_available': generated_nodes.is_file(),
              'nav_source_selected': manifest.get('nav_profile_source') == str(sources['nav']),
              'nodes_source_selected': manifest.get('nodes_profile_source') == str(sources['nodes'])}
    hashes = manifest.get('config_sha256') or {}
    source_hashes = {}
    for name, path in sources.items():
        digest = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        source_hashes[name] = digest
        checks[f'{name}_source_hash_matches_runtime'] = bool(
            digest and hashes.get(str(path)) == digest)
    for name, path in (('nav', generated_nav), ('nodes', generated_nodes)):
        checks[f'generated_{name}_hash_matches_runtime'] = bool(
            path.is_file() and hashes.get(str(path)) ==
            hashlib.sha256(path.read_bytes()).hexdigest())

    def without_sim_time(value, stamps):
        if isinstance(value, dict):
            result = {}
            for key, child in value.items():
                if key == 'use_sim_time':
                    stamps.append(child)
                else:
                    result[key] = without_sim_time(child, stamps)
            return result
        if isinstance(value, list):
            return [without_sim_time(child, stamps) for child in value]
        return value

    source_times, generated_times = [], []
    if (sources['nav'].is_file() and generated_nav.is_file() and
            sources['nodes'].is_file() and generated_nodes.is_file()):
        try:
            source_nav = yaml.safe_load(sources['nav'].read_text())
            sim_nav = yaml.safe_load(generated_nav.read_text())
            source_nodes = yaml.safe_load(sources['nodes'].read_text())
            sim_nodes = yaml.safe_load(generated_nodes.read_text())
            physical = manifest.get('physical_footprint') is True
            checks['nav_values_equal_except_declared_sim_overrides'] = (
                sim_nav == build_nav_profile_data(source_nav, physical_footprint=physical))
            checks['nodes_values_equal_except_declared_sim_overrides'] = (
                sim_nodes == build_nodes_profile_data(source_nodes, physical_footprint=physical))
            without_sim_time(source_nav, source_times)
            without_sim_time(sim_nav, generated_times)
            checks['sim_clock_is_true_after_override'] = bool(
                source_times and len(source_times) == len(generated_times)
                and all(value is False for value in source_times)
                and all(value is True for value in generated_times))
            expected_radius = .22 if physical else SIM_ROBOT_RADIUS_M
            expected_inflation = (source_nav['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius']
                                  if physical else SIM_INFLATION_RADIUS_M)
            expected_cells = (source_nodes['/astar_planner']['ros__parameters']['obstacle_inflation_cells']
                              if physical else SIM_STATIC_INFLATION_CELLS)
            checks['sim_footprint_matches_stock_sdf'] = bool(
                all(sim_nav[name][name]['ros__parameters']['robot_radius'] ==
                    expected_radius and
                    sim_nav[name][name]['ros__parameters']['inflation_layer'][
                        'inflation_radius'] == expected_inflation
                    for name in ('local_costmap', 'global_costmap')) and
                sim_nodes['/astar_planner']['ros__parameters']['robot_radius'] ==
                expected_radius and
                sim_nodes['/astar_planner']['ros__parameters'][
                    'obstacle_inflation_cells'] == expected_cells and
                sim_nodes['/astar_planner']['ros__parameters'][
                    'preferred_clearance'] == source_nodes['/astar_planner']['ros__parameters']['preferred_clearance'] and
                sim_nodes['/path_follower']['ros__parameters'][
                    'recovery_robot_radius'] == expected_radius)
        except (OSError, yaml.YAMLError, KeyError, TypeError):
            checks['nav_values_equal_except_declared_sim_overrides'] = False
            checks['nodes_values_equal_except_declared_sim_overrides'] = False
            checks['sim_clock_is_true_after_override'] = False
            checks['sim_footprint_matches_stock_sdf'] = False
    else:
        checks['nav_values_equal_except_declared_sim_overrides'] = False
        checks['nodes_values_equal_except_declared_sim_overrides'] = False
        checks['sim_clock_is_true_after_override'] = False
        checks['sim_footprint_matches_stock_sdf'] = False
    copy_preserved = all(checks.values())
    authoritative_selected = all(authoritative_checks.values())
    return {
        'passed': copy_preserved and authoritative_selected, 'checks': checks,
        'copy_preserves_selected_values': copy_preserved,
        'authoritative_physical_sources': authoritative_selected,
        'authoritative_source_checks': authoritative_checks,
        'authoritative_source_paths': {name: str(path) for name, path in authoritative.items()},
        'source_paths': {name: str(path) for name, path in sources.items()},
        'source_sha256': source_hashes,
        'sim_clock_entries': {'physical': len(source_times), 'simulation': len(generated_times)},
        'scope': 'selected YAML copy preservation is separate from strict official physical-source path selection; '
                 'Nav2 and classroom nodes YAML, checked-in EKF source values with '
                 'declared Gazebo collision geometry overrides only; not live '
                 'robot ros2 param dump or hardware topology',
    }


def classify_startup_lifecycle_failure(directory: Path, evidence: dict) -> dict:
    """Label a witnessed pre-mission abort; never turn a failed trial green."""
    if (evidence.get('passed') is True or evidence.get('mission_accepted_via_web') is not False
            or evidence.get('mission_started_at') is not None
            or evidence.get('transient_injected_mid_route') is True
            or evidence.get('transient_profile_request')):
        return {}
    log, relay = directory/'gazebo_launch.log', directory/'sensor_faults.jsonl'
    timeline = directory/'safety_timeline.jsonl'
    if not all(path.is_file() for path in (log, relay, timeline)):
        return {}
    try:
        raw = log.read_bytes()
        lines = raw.decode(errors='replace').splitlines()
        marker = 'Failed to bring up all requested nodes. Aborting bringup.'
        index = next((i for i, line in enumerate(lines) if marker in line), None)
        events = [json.loads(line) for line in relay.read_text().splitlines() if line.strip()]
        records = [json.loads(line) for line in timeline.read_text().splitlines() if line.strip()]
        if (index is None or not any(event.get('event') == 'sample' for event in events)
                or any(event.get('event') == 'profile' and event.get('name') != 'clean'
                       for event in events)
                or any(row.get('kind') == 'goal' or
                       row.get('kind') == 'cmd_vel' and
                       (abs(row.get('linear', 0.)) > 1e-6 or abs(row.get('angular', 0.)) > 1e-6)
                       for row in records)):
            return {}
    except (OSError, ValueError, TypeError):
        return {}
    return {'failure_class': 'STARTUP_LIFECYCLE_ABORT',
            'fault_execution': ('TRANSIENT_NOT_STARTED' if evidence.get('transient_requested') is True
                                else 'NOT_STARTED'),
            'startup_failure_witness': {'log': str(log), 'sha256': hashlib.sha256(raw).hexdigest(),
                'lines': lines[max(0, index-7):index+1],
                'mission_accepted_via_web': False, 'nonzero_command_seen': False,
                'fault_profile_applied': False}}


def attach_timeline_and_network(stage: Stage, directory: Path, evidence: dict) -> dict:
    evidence.update(classify_startup_lifecycle_failure(directory, evidence))
    if '--memory-session' in stage.options:
        memory_summary = directory/'memory_observer.summary.json'
        if memory_summary.is_file():
            try:
                evidence['memory_session_observer'] = json.loads(memory_summary.read_text())
            except (OSError, ValueError) as exc:
                evidence['memory_session_observer'] = {'error': f'cannot_read_summary: {exc}'}
        else:
            evidence['memory_session_observer'] = {'error': 'observer_summary_missing'}
    timeline_file = directory/'safety_timeline.jsonl'
    if timeline_file.is_file():
        try:
            records = [json.loads(line) for line in timeline_file.read_text().splitlines()
                       if line.strip()]
            # A static-world geometric witness is available only when this
            # trial cannot spawn an unmapped object. Dynamic-obstacle stages
            # must keep the raw LiDAR stopping gate: the SDF fixture cannot
            # certify a newly inserted vehicle or box.
            static_fixture = directory/'fixture/classroom_fault.world'
            geometry_allowed = (stage.kind not in {
                'moving_obstacle', 'route_obstacle', 'home_obstacle',
                'web_moving_obstacle'} and
                stage.key != '24_web_memory_sector1_to_sector2_removal')
            boxes = (safety_timeline.load_static_world_boxes(static_fixture)
                     if geometry_allowed and static_fixture.is_file() else None)
            nav_parameters = directory/'classroom_nav_sim_time.yaml'
            stop_model = {}
            nav_config = None
            if nav_parameters.is_file():
                # A fast physical-profile trial must use the same commanded
                # speed and deceleration in its independent stopping witness.
                nav_config = yaml.safe_load(nav_parameters.read_text())
                try:
                    follower = nav_config['controller_server']['ros__parameters']['FollowPath']
                    stop_model = {
                        'max_linear_speed_mps': float(follower['max_vel_x']),
                        'decel_limit_mps2': abs(float(follower['decel_lim_x'])),
                        'robot_collision_radius_m': float(nav_config['local_costmap']['local_costmap']['ros__parameters']['robot_radius']),
                    }
                    evidence['runtime_command_limits'] = {
                        'linear_mps': float(follower['max_vel_x']),
                        'angular_radps': float(follower['max_vel_theta']),
                        'source': str(nav_parameters),
                        'sha256': hashlib.sha256(nav_parameters.read_bytes()).hexdigest()}
                except (KeyError, TypeError, AttributeError) as exc:
                    raise ValueError('Nav2 FollowPath stopping model missing') from exc
            safety = safety_timeline.assess(records, static_world_boxes=boxes,
                                            **stop_model)
            safety['static_geometry_witness'] = (
                str(static_fixture) if boxes is not None else None)
            if stage.kind in {'web_dwell', 'web_transient', 'web_moving_obstacle',
                              'web_reentry'}:
                truth_xy_tolerance = web_truth_xy_tolerance(nav_config)
                evidence['web_truth_xy_tolerance_m'] = truth_xy_tolerance
                evidence['nav2_internal_xy_tolerance_m'] = effective_nav2_goal_tolerance(nav_config)
                manifest = directory/'runtime_manifest.json'
                if manifest.is_file():
                    evidence['web_runtime_manifest'] = json.loads(manifest.read_text())
                    if stage.key == '17_web_hardware_value_parity':
                        evidence['physical_value_parity'] = check_physical_source_value_parity(
                            directory, evidence['web_runtime_manifest'])
                fixture_manifest = directory/'fixture/fixture_manifest.json'
                if fixture_manifest.is_file():
                    fixture = json.loads(fixture_manifest.read_text())
                    sources = (fixture.get('source') or {}).keys()
                    evidence['web_world_variant'] = (
                        'stress' if any(str(path).endswith(
                            '/classroom_20261030_stress.world') for path in sources) else
                        'replay' if any(str(path).endswith(
                            '/classroom_20261030_replay.world') for path in sources) else
                        'unknown')
                if stage.kind == 'web_reentry':
                    evidence['cycle_independent'] = [
                        {'web_ground_truth_checks': check_web_ground_truth(
                            records, cycle, truth_xy_tolerance),
                         'web_independent_dwell_checks': check_web_dwell_independent(
                             records, cycle),
                         'home_independent_settle_check': check_home_settle_independent(
                             records, cycle)}
                        for cycle in evidence.get('cycles', [])]
                else:
                    evidence['web_ground_truth_checks'] = check_web_ground_truth(
                        records, evidence, truth_xy_tolerance)
                    evidence['web_independent_dwell_checks'] = check_web_dwell_independent(
                        records, evidence)
                    evidence['home_independent_settle_check'] = check_home_settle_independent(
                        records, evidence)
                if stage.kind == 'web_transient':
                    evidence['web_transient_checks'] = check_web_transient(
                        records, directory/'sensor_faults.jsonl', evidence)
                if stage.kind == 'web_moving_obstacle':
                    injection = directory/'web_obstacle_injection.json'
                    if injection.is_file():
                        evidence['web_obstacle'] = json.loads(injection.read_text())
                    evidence['web_obstacle_overlay_checks'] = check_web_obstacle_overlay(
                        evidence)
                    if stage.key in {'12_web_integrated_home_obstacle',
                                     '12_web_integrated_stress_home_obstacle'}:
                        evidence['home_obstacle_temporal_check'] = check_home_obstacle_timing(
                            evidence)
                if stage.key == '24_web_memory_sector1_to_sector2_removal':
                    transition = directory/'memory_transition_injection.json'
                    if transition.is_file():
                        evidence['memory_transition'] = json.loads(transition.read_text())
            elif stage.kind == 'web_disconnect':
                evidence['web_disconnect_truth'] = check_web_disconnect_truth(records, evidence)
        except (ValueError, OSError, TypeError, yaml.YAMLError) as exc:
            safety = {'passed': False, 'missing_evidence': ['timeline_parse_error: '+str(exc)],
                      'violations': []}
    else:
        safety = {'passed': False, 'missing_evidence': ['timeline_missing'],
                  'violations': []}
    (directory/'safety_assessment.json').write_text(json.dumps(
        safety, ensure_ascii=False, indent=2)+'\n')
    if stage.key == '15_integrated_with_recorder':
        manifest = directory/'runtime_manifest.json'
        if manifest.is_file():
            try:
                evidence['record_incidents_enabled'] = json.loads(
                    manifest.read_text()).get('record_incidents') is True
            except (OSError, ValueError):
                evidence['record_incidents_enabled'] = False
    if stage.key in WEB_FULL_STACK_STAGES and '--stress' in stage.options:
        trace = directory/'stress_crate_trace.jsonl'
        assessment = directory/'stress_crate_assessment.json'
        assessor = REPO/'scripts/assess_classroom_stress_crate.py'
        if trace.is_file() and assessor.is_file():
            try:
                from assess_classroom_stress_crate import verified_runtime_robot_radius
                robot_radius = verified_runtime_robot_radius(directory/'runtime_manifest.json')
            except (OSError, ValueError, TypeError, KeyError, ET.ParseError) as exc:
                evidence['stress_crate_assessment'] = {
                    'passed': False, 'reasons': ['runtime_body_radius_unverified'],
                    'detail': str(exc)}
            else:
                verdict = subprocess.run(
                    [sys.executable, str(assessor), '--trace', str(trace),
                     '--web-evidence', str(directory/'acceptance.json'),
                     '--output', str(assessment), '--robot-radius', str(robot_radius),
                     '--expected-route',
                     ','.join(stop.removeprefix('Sector ') for stop in stage.route
                              if stop.startswith('Sector '))], cwd=REPO,
                    check=False, capture_output=True, text=True, timeout=30)
                if assessment.is_file():
                    evidence['stress_crate_assessment'] = json.loads(assessment.read_text())
                    evidence['stress_crate_assessment']['runtime_body_radius_verified'] = True
                else:
                    evidence['stress_crate_assessment'] = {
                        'passed': False, 'reasons': ['assessor_no_output'],
                        'returncode': verdict.returncode,
                        'stderr': verdict.stderr[-1000:]}
        else:
            evidence['stress_crate_assessment'] = {
                'passed': False, 'reasons': ['observer_or_assessor_missing']}
    if stage.kind == 'wan_absent':
        network_file = directory.parent/(stage.key+'.network.txt')
        if network_file.is_file():
            network_text = network_file.read_text()
            no_default_route = 'default' not in network_text
            evidence['staged_network_namespace'] = {
                'no_default_route': no_default_route,
                'loopback_up': 'lo' in network_text and 'UP' in network_text,
                'evidence': str(network_file)}
        else:
            evidence['staged_network_namespace'] = {'no_default_route': False,
                                                      'evidence': 'missing'}
    return safety


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--list', action='store_true', help='Print staged matrix only')
    action.add_argument('--run', metavar='STAGES',
                        help='Run all or comma-separated stage IDs sequentially')
    action.add_argument('--assess-existing', metavar='STAGES',
                        help='Reassess existing stage folders without launching Gazebo')
    action.add_argument('--run-inner', metavar='STAGE', help=argparse.SUPPRESS)
    parser.add_argument('--output', type=Path,
                        help='Fresh output root for --run; existing root for --assess-existing')
    parser.add_argument('--leg-timeout', type=int, default=180)
    parser.add_argument('--obstacle-hold-timeout', type=int, default=15,
                        help='Route WAIT_REPLAN stationary early verdict (s); 0 disables')
    parser.add_argument('--continue-on-fail', action='store_true')
    parser.add_argument('--tf-source-best-effort', action='store_true',
                        help='Sim-only TF QoS A/B: use best-effort relay raw TF subscriber')
    parser.add_argument('--laser-filter-for-change-memory', action='store_true',
                        help='A/B: weak laser speckle filter for change_memory only')
    parser.add_argument('--measure-scan-filter', action='store_true',
                        help='Record paired raw/filtered scan arrival and singleton statistics')
    parser.add_argument('--nav-params-source', type=Path,
                        help='Nav2 YAML for the trial (default: classroom Gazebo profile)')
    parser.add_argument('--nodes-params-source', type=Path,
                        help='A*/follower YAML for the trial (default: classroom hardware profile)')
    parser.add_argument('--obstacle-scenario', type=Path,
                        help='Fixed JSON actor input for paired web obstacle tests; never relocated')
    parser.add_argument('--physical-footprint', action='store_true',
                        help='Copied 0.22 m collision proxy and unchanged physical parameter values')
    parser.add_argument('--memory-session', action='store_true',
                        help='Enable session obstacle memory in every selected stage')
    parser.add_argument('--cold-start', action='store_true',
                        help='Web stages: AMCL boots without a prior; verify stopped Home initialization through actual backend API')
    parser.add_argument('--ekf-delivery-faults', action='store_true',
                        help='Web stages: relay actual EKF odom/TF output delivery (not EKF CPU/raw-input faults)')
    parser.add_argument('--hardware-profile', default='nominal',
                        choices=('nominal', 'sensor_rates', 'wheel_asymmetry', 'actuator_lag'),
                        help='Copied-model stress envelope; not calibrated physical equivalence')
    args = parser.parse_args()
    if args.leg_timeout < 30 or args.leg_timeout > 600:
        parser.error('--leg-timeout must be 30..600 seconds')
    if not 0 <= args.obstacle_hold_timeout <= 600:
        parser.error('--obstacle-hold-timeout must be 0..600 seconds')
    if args.list:
        for stage in STAGES:
            print(f'{stage.key}: {stage.description}')
        print('Manual gates: real PC↔robot Wi-Fi/DDS outage, hardware deployment/drive')
        return 0
    if args.output is None:
        parser.error('--output is required')
    if args.tf_source_best_effort and args.run not in ('03_sector_3',):
        parser.error('--tf-source-best-effort is only for the isolated Stage03 A/B run')
    for name in ('nav_params_source', 'nodes_params_source'):
        value = getattr(args, name)
        if value is not None and (args.run or args.run_inner):
            value = value.expanduser().resolve()
            if (value.suffix.lower() not in {'.yaml', '.yml'} or
                    not value.is_file() or not value.stat().st_size):
                parser.error('--'+name.replace('_', '-')+' must name a nonempty YAML file')
            setattr(args, name, value)
    try:
        stages = selected_stages(args.run or args.assess_existing or args.run_inner)
    except ValueError as exc:
        parser.error(str(exc))
    scenario_manifest = None
    if args.obstacle_scenario is not None:
        from inject_classroom_web_obstacle import obstacle_scenario_manifest
        if not any(stage.kind == 'web_moving_obstacle' for stage in stages):
            parser.error('--obstacle-scenario requires a web moving-obstacle stage')
        args.obstacle_scenario = args.obstacle_scenario.expanduser().resolve()
        try:
            scenario_manifest = obstacle_scenario_manifest(args.obstacle_scenario)
        except (OSError, ValueError, TypeError) as exc:
            parser.error('--obstacle-scenario: '+str(exc))
    if args.memory_session:
        stages = [replace(stage, options=(*stage.options, '--memory-session'))
                  if '--memory-session' not in stage.options else stage for stage in stages]
    if (args.cold_start or args.ekf_delivery_faults) and any(
            stage.kind not in WEB_RUNTIME_KINDS for stage in stages):
        parser.error('--cold-start/--ekf-delivery-faults require full web stages')
    selected_realism = [flag for enabled, flag in (
        (args.cold_start, '--cold-start'),
        (args.ekf_delivery_faults, '--ekf-delivery-faults')) if enabled]
    if args.hardware_profile != 'nominal':
        selected_realism += ['--hardware-profile', args.hardware_profile]
    stages = [replace(stage, options=(*stage.options,
                 *(flag for flag in selected_realism if flag not in stage.options)))
              for stage in stages]
    root = args.output.expanduser().resolve()
    if args.run:
        if root.exists():
            parser.error('Refusing to overwrite output root: '+str(root))
        assert_master_free(MASTER_PORT)
        root.mkdir(parents=True)
    elif not root.is_dir():
        parser.error('Existing evidence root missing: '+str(root))
    if scenario_manifest is not None and args.run:
        copied = root/'obstacle_scenario.json'
        copied.write_bytes(args.obstacle_scenario.read_bytes())
        preserved = obstacle_scenario_manifest(copied)
        if preserved['sha256'] != scenario_manifest['sha256']:
            parser.error('Obstacle scenario changed while copying')
        scenario_manifest['copied_path'] = str(copied)
        args.obstacle_scenario = copied
    scenario_options = ({'obstacle_scenario': args.obstacle_scenario}
                        if args.obstacle_scenario is not None else {})
    if args.run_inner:
        if len(stages) != 1:
            parser.error('--run-inner requires one stage')
        return execute_stage(stages[0], root, args.leg_timeout,
                             args.obstacle_hold_timeout,
                             args.tf_source_best_effort,
                             args.laser_filter_for_change_memory,
                             args.measure_scan_filter,
                             args.nav_params_source, args.physical_footprint,
                             args.nodes_params_source, **scenario_options)
    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'domain': 176, 'localhost_only': True,
              'physical_robot_touched': False,
              'physical_footprint': args.physical_footprint,
              'memory_session_forced': args.memory_session,
              'cold_start_requested': args.cold_start,
              'ekf_delivery_faults_requested': args.ekf_delivery_faults,
              'hardware_profile': args.hardware_profile,
              'visual_ui_qa': 'NOT_RUN_BACKEND_API_AND_STATIC_JS_ONLY',
              'web_client_outage': 'NOT_RUN_SEPARATE_MANUAL_GATE',
              'physical_dds_outage': 'NOT_RUN_REQUIRES_SUPERVISED_ROBOT',
              'physical_drive': 'NOT_RUN_USER_OPERATED',
              'laser_filter_for_change_memory': args.laser_filter_for_change_memory,
              'scan_pair_observer': args.measure_scan_filter,
              'nav_params_source': (str(args.nav_params_source)
                                    if args.nav_params_source is not None else None),
              'nodes_params_source': (str(args.nodes_params_source)
                                      if args.nodes_params_source is not None else None),
              'obstacle_scenario': scenario_manifest,
              'stages': []}
    for stage in stages:
        directory = root/stage.key
        source_before = web_runtime_source_hashes(stage)
        returncode = None
        if args.run:
            assert_master_free(MASTER_PORT)
            try:
                returncode = (run_in_network_namespace(stage, root, args.leg_timeout,
                                                       args.obstacle_hold_timeout,
                                                       args.laser_filter_for_change_memory,
                                                       args.measure_scan_filter,
                                                       args.nav_params_source, args.physical_footprint,
                                                       args.nodes_params_source, **scenario_options)
                              if stage.kind == 'wan_absent' else
                              execute_stage(stage, root, args.leg_timeout,
                                            args.obstacle_hold_timeout,
                                            args.tf_source_best_effort,
                                            args.laser_filter_for_change_memory,
                                            args.measure_scan_filter,
                                            args.nav_params_source, args.physical_footprint,
                                            args.nodes_params_source, **scenario_options))
            except KeyboardInterrupt:
                print('Interrupted; the child runner owns Gazebo cleanup.', file=sys.stderr)
                raise
        path = directory/'acceptance.json'
        if path.is_file():
            try:
                evidence = json.loads(path.read_text())
                safety = attach_timeline_and_network(stage, directory, evidence)
                (directory/'derived_evidence.json').write_text(json.dumps(
                    evidence, ensure_ascii=False, indent=2)+'\n')
                outcome = assess(stage, evidence)
                if scenario_manifest is not None and stage.kind == 'web_moving_obstacle':
                    actor_scene = (evidence.get('web_obstacle') or {}).get('obstacle_scenario') or {}
                    outcome['obstacle_scenario_hash_matches'] = (
                        actor_scene.get('sha256') == scenario_manifest['sha256'])
                    if not outcome['obstacle_scenario_hash_matches']:
                        outcome['passed'] = False
                        outcome['outcome'] = 'FAIL'
                        outcome['reasons'].append('fixed_obstacle_scenario_hash_mismatch')
                outcome['safety_gate_passed'] = safety['passed']
                outcome['safety_violations'] = len(safety['violations'])
                if not safety['passed']:
                    outcome['passed'] = False
                    outcome['outcome'] = 'FAIL'
                    outcome['reasons'].append('safety_timeline_failed')
                if returncode not in (None, 0):
                    outcome['passed'] = False
                    outcome['outcome'] = 'FAIL'
                    outcome['reasons'].append('runner_nonzero_exit')
            except (ValueError, OSError, TypeError) as exc:
                outcome = {'stage': stage.key, 'outcome': 'FAIL', 'passed': False,
                           'reasons': ['invalid_acceptance_json: '+str(exc)]}
        else:
            outcome = {'stage': stage.key, 'outcome': 'NOT_RUN' if returncode is None else 'FAIL',
                       'passed': False, 'reasons': ['acceptance_json_missing']}
        outcome['runner_exit_code'] = returncode
        outcome['evidence'] = str(path)
        if source_before:
            source_after = web_runtime_source_hashes(stage)
            outcome['source_sha256_before'] = source_before
            outcome['source_sha256_after'] = source_after
            if source_before != source_after:
                outcome['passed'] = False
                outcome['outcome'] = 'FAIL'
                outcome['reasons'].append('runtime_source_changed_during_stage')
        report['stages'].append(outcome)
        report['passed'] = all(item['passed'] for item in report['stages'])
        (root/'staged_report.json').write_text(json.dumps(
            report, ensure_ascii=False, indent=2)+'\n')
        print(f"{stage.key}: {outcome['outcome']} {', '.join(outcome['reasons'])}", flush=True)
        if not outcome['passed'] and not args.continue_on_fail:
            print('Stopped at failed stage; fix the cause before advancing.', flush=True)
            break
    print('Evidence report: '+str(root/'staged_report.json'))
    return 0 if len(report['stages']) == len(stages) and report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
