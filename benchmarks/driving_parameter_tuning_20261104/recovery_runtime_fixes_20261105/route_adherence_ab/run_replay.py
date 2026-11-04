#!/usr/bin/env python3
"""Repeat the two real DWB runs without Gazebo or robot output."""
import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('use a fresh output directory')
    here = Path(__file__).resolve().parent
    repo = next(p for p in here.parents if (p/'ros2_ws').is_dir())
    args.output.mkdir(parents=True)
    for label, scale in [('off', 0.), ('on', 32.)]:
        command = ['/usr/bin/python3',
            str(repo/'benchmarks/controller_footprint_soft_cost_ab_20261104/replay_captured_costmap.py'),
            str(here/'permuted_approach_frozen.jsonl.gz'), '--at-time', '62.68',
            '--nav-params', str(here/label/'controller_parameters.yaml'),
            '--output', str(args.output/label), '--domain', '186', '--duration', '2',
            '--override', f'FollowPath.route_adherence_scale={scale}']
        with (args.output/(label+'.log')).open('w') as stream:
            subprocess.run(command, cwd=repo, stdout=stream, stderr=subprocess.STDOUT, check=True)


if __name__ == '__main__':
    main()
