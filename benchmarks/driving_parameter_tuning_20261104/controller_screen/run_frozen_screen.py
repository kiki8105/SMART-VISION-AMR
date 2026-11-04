#!/usr/bin/env python3
"""Bench-only 6-profile actual DWB screen; no robot/Gazebo/closed-loop claim.

Source ROS Humble and the built workspace overlay first. Every subprocess uses
loopback domain 186, and controller output is remapped to /replay/controller_cmd_vel.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
REPLAY = REPO/'scripts/replay_incident_controller.py'
BASE = REPO/'benchmarks/inflation_tuning_20261104/profiles/nav_inflation_030.yaml'
PLUGIN = next((Path(prefix)/'lib/libroute_tracking_dwb.so'
               for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
               if prefix and (Path(prefix)/'lib/libroute_tracking_dwb.so').is_file()), None)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def envelope(w):
    yaw = abs(w)
    return (.22-yaw/.08*.07 if yaw < .08 else
            .15-min(1., (yaw-.08)/.27)*.05)


def load_replay():
    spec = importlib.util.spec_from_file_location('original_replay', REPLAY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.summarize_evaluation
    def summarize(evaluation):
        result = original(evaluation)
        valid = [v for v in evaluation.get('twists', []) if v.get('total', -1.) >= 0.]
        velocities = [v['traj']['velocity'] for v in valid]
        result.update({
            'valid_envelope_violation_count': sum(v['x'] > envelope(v['theta'])+1.e-8 for v in velocities),
            'valid_pure_yaw_count': sum(abs(v['x']) < 1.e-6 and abs(v['theta']) > .08 for v in velocities),
        })
        return result
    module.summarize_evaluation = summarize
    return module


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--single':
        sys.argv = [str(REPLAY)] + sys.argv[2:]
        load_replay().main()
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=HERE,
                        help='Use a fresh directory when reproducing archived runs.')
    options = parser.parse_args()
    destination = options.output_root.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'profiles').mkdir(exist_ok=True)
    if PLUGIN is None:
        parser.error('Source the built workspace overlay containing smart_vision_amr_dwb first.')
    os.environ.update(ROS_DOMAIN_ID='186', ROS_LOCALHOST_ONLY='1', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    profile = yaml.safe_load(BASE.read_text())
    assert profile['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] == .22
    inputs = [('folded_physical_incident', HERE/'inputs/folded_physical_incident.jsonl.gz', 1793781696.5),
              ('corner', HERE/'inputs/corner.jsonl', 1.)]
    manifest = {
        'scope': 'Actual Nav2 controller with identical frozen scan/TF/odom/path inputs; NOT a closed-loop or physical driving pass.',
        'costmap_limitation': 'Costmap rebuilt from each frozen scan by the original replay script; not captured costmap history.',
        'domain': 186, 'localhost_only': True, 'output_topic': '/replay/controller_cmd_vel', 'duration_seconds': 2.,
        'base_profile': str(BASE.relative_to(REPO)), 'base_profile_sha256': sha(BASE),
        'replay_script': str(REPLAY.relative_to(REPO)), 'replay_script_sha256': sha(REPLAY),
        'plugin_binary_sha256': sha(PLUGIN),
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
        'working_tree': 'Uncommitted runtime improvements present; authoritative settings frozen by parent before screen.',
        'input_sha256': {name: sha(path) for name,path,_ in inputs},
        'diagnostic_augmentation': 'Original evaluation summarizer additionally counts ALL valid candidate velocity-envelope violations and pure-yaw candidates; no controller behavior changed.',
    }
    manifest['executed_screen_driver_sha256'] = sha(Path(__file__))
    (destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    rows = []
    for horizon in (.45,.55,.65):
        for scale in (.02,.04):
            name = f'h{round(horizon*100):03d}_b{round(scale*1000):03d}'
            params = yaml.safe_load(yaml.safe_dump(profile))
            params['controller_server']['ros__parameters']['FollowPath'].update(
                {'route_horizon_distance': horizon, 'BaseObstacle.scale': scale})
            parameter_path = destination/'profiles'/f'{name}.yaml'
            parameter_path.write_text(yaml.safe_dump(params, sort_keys=False))
            for case, input_path, stamp in inputs:
                output = destination/'runs'/name/case
                command = [sys.executable, str(Path(__file__).resolve()), '--single', str(input_path),
                           '--at-time', str(stamp), '--nav-params', str(parameter_path), '--output', str(output),
                           '--duration', '2', '--domain', '186']
                completed = subprocess.run(command, cwd=REPO, capture_output=True, text=True)
                (output/'runner.log').write_text(completed.stdout+completed.stderr)
                if completed.returncode:
                    raise RuntimeError(f'{name}/{case} failed; see {output}/runner.log')
                observations = [json.loads(line) for line in (output/'observations.jsonl').open()]
                evaluations = [x['data'] for x in observations if x['kind']=='evaluation']
                commands = [x['data'] for x in observations if x['kind']=='command']
                if not evaluations:
                    raise RuntimeError(f'{name}/{case}: no DWB evaluations')
                winner = evaluations[-1]['best']['velocity']
                selected = [x['best']['velocity'] for x in evaluations if x.get('best')]
                rejects = {}
                for evaluation in evaluations:
                    for key, count in evaluation['invalid_by_critic'].items():
                        rejects[key] = max(rejects.get(key,0), count)
                row = {'profile': name, 'case': case, 'route_horizon_distance': horizon,
                       'BaseObstacle.scale': scale, 'profile_sha256': sha(parameter_path),
                       'evaluation_count': len(evaluations),
                       'forward_valid_min': min(x['forward_valid_count'] for x in evaluations),
                       'forward_valid_last': evaluations[-1]['forward_valid_count'],
                       'last_valid_count': evaluations[-1]['valid_count'],
                       'selected_linear_mps': winner['x'], 'selected_angular_radps': winner['theta'],
                       'selected_pure_yaw_count': sum(abs(v['x']) < 1.e-6 and abs(v['theta']) > .08 for v in selected),
                       'valid_candidate_envelope_violations': sum(x['valid_envelope_violation_count'] for x in evaluations),
                       'command_envelope_violations': sum(c['linear']['x'] > envelope(c['angular']['z'])+1.e-8 for c in commands),
                       'max_collision_rejects_by_critic': {k:v for k,v in rejects.items() if k in ('BaseObstacle','ObstacleFootprint')},
                       'collision_rejects_last_by_critic': {k:v for k,v in evaluations[-1]['invalid_by_critic'].items()
                                                          if k in ('BaseObstacle','ObstacleFootprint')},
                       'empty_trajectory_evaluation_count': sum(x['trajectory_count'] == 0 for x in evaluations),
                       'no_legal_candidate_evaluation_count': sum(x['trajectory_count'] > 0 and not x.get('best') for x in evaluations),
                       'selected_linear_min_mps': min(x['x'] for x in selected),
                       'selected_linear_max_mps': max(x['x'] for x in selected),
                       'controller_diagnostics_last': next((x['data'] for x in reversed(observations)
                                                            if x['kind']=='controller_diagnostics'), None),
                       'last_evaluation': evaluations[-1]}
                rows.append(row)
                (destination/'results.json').write_text(json.dumps({'scope':manifest['scope'], 'rows': rows}, indent=2)+'\n')
                with gzip.open(output/'observations.jsonl.gz', 'wt') as archive:
                    archive.write((output/'observations.jsonl').read_text())
                (output/'observations.jsonl').unlink()
                print(f'{name}/{case}: forward={row["forward_valid_last"]} v={winner["x"]:.6f} w={winner["theta"]:.6f} envelope={row["valid_candidate_envelope_violations"]} pureyaw={row["selected_pure_yaw_count"]}', flush=True)
    assert len(rows) == 12


if __name__ == '__main__':
    main()
