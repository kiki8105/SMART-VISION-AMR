#!/usr/bin/env python3
"""Archive completed, isolated Gazebo evidence without overwriting past trials."""

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def incident_window(timeline):
    """Use actual observer wall times, never file mtimes, for serial trials."""
    if not timeline.is_file():
        return None
    stamps = []
    for line in timeline.open():
        value = json.loads(line).get('wall_time')
        if type(value) in (int, float) and math.isfinite(value):
            stamps.append(value)
    return (min(stamps), max(stamps)) if stamps else None


def matching_incidents(directory, window):
    """Only recorder-owned domain-176 captures triggered during this trial.

    The matrix is serial and localhost-only. Time membership is documented
    rather than pretending these captures contain an embedded trial ID.
    """
    if window is None:
        return []
    result = []
    for manifest in sorted(directory.glob('incident_*/manifest.json')):
        if manifest.parent.is_symlink():
            continue
        try:
            data = json.loads(manifest.read_text())
            stamp = (data.get('trigger') or {}).get('wall_time_ns')
            if (data.get('generated_by') == 'smart_vision_amr.flight_recorder'
                    and str(data.get('ros_domain_id')) == '176'
                    and type(stamp) is int
                    and window[0] <= stamp/1e9 <= window[1]
                    and (manifest.parent/'events.jsonl.gz').is_file()):
                result.append(manifest.parent)
        except (OSError, ValueError, TypeError):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9_-]+', args.label):
        parser.error('label must contain letters, numbers, underscore or hyphen')
    root = args.root.resolve()
    report = json.loads((root/'staged_report.json').read_text())
    assert report['domain'] == 176 and report['physical_robot_touched'] is False
    destination = Path(__file__).resolve().parent/args.label
    destination.mkdir()  # Never overwrite a prior outcome.
    copied = []

    def preserve(source, target, compress=False):
        if not source.is_file():
            return
        target.parent.mkdir(parents=True, exist_ok=True)
        data = source.read_bytes()
        if compress:
            target.write_bytes(gzip.compress(data, mtime=0))
        else:
            shutil.copyfile(source, target)
        copied.append({'original_path': str(source),
                       'archive_path': str(target.relative_to(destination)),
                       'original_sha256': hashlib.sha256(data).hexdigest(),
                       'archive_sha256': sha(target), 'gzip': compress})

    preserve(root/'staged_report.json', destination/'staged_report.json')
    preserve(Path(str(root)+'-source-rates.json'), destination/'source_rates.json')
    preserve(Path(str(root)+'.log'), destination/'runner.log.gz', True)
    summaries = []
    for stage in report['stages']:
        source = root/stage['stage']
        target = destination/stage['stage']
        for name in ('acceptance.json', 'derived_evidence.json', 'safety_assessment.json',
                     'runtime_manifest.json', 'web_obstacle_injection.json',
                     'stress_crate_assessment.json', 'classroom_nav_sim_time.yaml',
                     'classroom_nodes_sim.yaml', 'ekf_hardware_like_sim.yaml',
                     'obstacle_scenario.json', 'memory_observer.summary.json',
                     'actual_spawned_model.sdf',
                     'operator_abort_reason.json',
                     'fixture/fixture_manifest.json', 'fixture/waffle_pi_fault.sdf',
                     'fixture/classroom_fault.world'):
            preserve(source/name, target/name)
        for name in ('safety_timeline.jsonl', 'memory_observer.jsonl', 'sensor_faults.jsonl',
                     'stress_crate_trace.jsonl', 'gazebo_launch.log', 'web_server.log',
                     'web_obstacle_injection.log'):
            preserve(source/name, target/(name+'.gz'), True)
        for image in source.glob('*_rear_confirmed.jpg'):
            preserve(image, target/image.name)
        window = incident_window(source/'safety_timeline.jsonl')
        captures = matching_incidents(
            Path.home()/'.ros/smart_vision_amr/incidents', window)
        for capture in captures:
            for name in ('manifest.json', 'events.jsonl.gz'):
                preserve(capture/name, target/'incidents'/capture.name/name)
        log = source/'gazebo_launch.log'
        if log.is_file():
            match = re.search(r'Loading entity XML from file (\S+)', log.read_text())
            if match and not (target/'actual_spawned_model.sdf').exists():
                preserve(Path(match[1]), target/'actual_spawned_model.sdf')
        acceptance = json.loads((source/'acceptance.json').read_text())
        derived = json.loads((source/'derived_evidence.json').read_text())
        actor_path = source/'web_obstacle_injection.json'
        actor = json.loads(actor_path.read_text()) if actor_path.exists() else {}
        safety = json.loads((source/'safety_assessment.json').read_text())
        summaries.append({
            'stage': stage['stage'], 'outcome': stage['outcome'],
            'reasons': stage['reasons'],
            'mission_backend_passed': acceptance.get('passed'),
            'final_state': acceptance.get('final_state'),
            'cold_start_proof': acceptance.get('cold_start_proof'),
            'prearm_passed': (acceptance.get('prearm') or {}).get('passed'),
            'source_unchanged': stage.get('source_sha256_before') ==
                                stage.get('source_sha256_after'),
            'actor_moving_steps': actor.get('moving_steps'),
            'actor_bypassed_while_live': actor.get('bypassed_while_live'),
            'actor_minimum_clearance_m': actor.get('minimum_clearance_m'),
            'safety_passed': safety.get('passed'),
            'safety_violations': safety.get('violations'),
            'incident_capture_count': len(captures),
            'operator_aborted_after_stall': (source/'operator_abort_reason.json').is_file(),
            'incident_selection': {'method': 'serial domain-176 trigger wall-time membership',
                                   'observer_wall_time_window': window,
                                   'sampling_is_not_rosbag': True},
            'transient': derived.get('web_transient_checks'),
            'physical_drive': False, 'visual_browser_qa': False})
    (destination/'archive_manifest.json').write_text(json.dumps(
        {'root_at_capture': str(root), 'files': copied}, indent=2)+'\n')
    (destination/'summary.json').write_text(json.dumps(
        {'stages': summaries, 'passed': report['passed']}, indent=2)+'\n')
    print(destination)


if __name__ == '__main__':
    main()
