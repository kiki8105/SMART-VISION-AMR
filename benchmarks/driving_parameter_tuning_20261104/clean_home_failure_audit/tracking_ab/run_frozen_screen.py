#!/usr/bin/env python3
"""Reproduce three captured-input single-factor DWB comparisons on domain186.

Source ROS Humble and the built Team2 workspace first. No Gazebo/robot is
started; the existing diagnostic helper remaps output to a replay-only topic.
"""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='new directory; existing results are never overwritten')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output directory already exists')
    here = Path(__file__).resolve().parent
    repo = next(parent for parent in here.parents if (parent/'ros2_ws').is_dir())
    helper = repo/'benchmarks/controller_footprint_soft_cost_ab_20261104/replay_captured_costmap.py'
    args.output.mkdir(parents=True)
    results = []
    for frame in (159.30, 161.34, 163.44):
        for label, override in (('baseline', None),
                                ('pathdist64', 'FollowPath.PathDist.scale=64.0'),
                                ('pathalign64', 'FollowPath.PathAlign.scale=64.0')):
            output = args.output/f't{frame:.2f}_{label}'
            command = ['/usr/bin/python3', str(helper), str(here/'captured_inputs.jsonl.gz'),
                       '--at-time', str(frame), '--nav-params', str(here/'recorded_nav_profile.yaml'),
                       '--output', str(output), '--duration', '1.2', '--domain', '186']
            if override:
                command += ['--override', override]
            with (args.output/f't{frame:.2f}_{label}.log').open('w') as log:
                completed = subprocess.run(command, cwd=repo, stdout=log, stderr=subprocess.STDOUT)
            record = {'frame_ros_seconds': frame, 'variant': label, 'returncode': completed.returncode}
            if (output/'summary.json').exists():
                summary = json.loads((output/'summary.json').read_text())
                evaluation = summary.get('best') or {}
                record.update(evaluations=summary['evaluation_count'], winner=evaluation.get('best'),
                              valid_count=evaluation.get('valid_count'), rejected=evaluation.get('invalid_by_critic'))
            results.append(record)
            print(json.dumps(record), flush=True)
            (args.output/'results.json').write_text(json.dumps(results, indent=2))
    return 0 if all(result['returncode'] == 0 for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
