#!/usr/bin/env python3
"""Summarize closed-loop outcomes and prove which parameter actually changed."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'inflation_summary', ROOT/'benchmarks/inflation_tuning_20261104/summarize_inflation_trials.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def differences(a, b, key=''):
    if isinstance(a, dict) and isinstance(b, dict):
        result = []
        for name in sorted(a.keys() | b.keys()):
            result += differences(a.get(name), b.get(name), f'{key}/{name}')
        return result
    return [] if a == b else [{'parameter': key, 'baseline': a, 'candidate': b}]


def read_trial(label, path):
    manifest = summary.load(path/'runtime_manifest.json', {})
    report = summary.load(path.parent/'staged_report.json', {})
    stage = next((s for s in report.get('stages', []) if s['stage'] == path.name), {})
    sources = stage.get('source_sha256_before', {})
    after = stage.get('source_sha256_after', {})
    config = {}
    for name in ('nav', 'nodes'):
        profile = Path(manifest.get(f'{name}_profile_generated', 'missing'))
        config[name] = yaml.safe_load(profile.read_text()) if profile.is_file() else None
    result = summary.summarize(label, path)
    interruption = summary.load(path.parent/'diagnostic_stop.json', None)
    if interruption is not None:
        result['diagnostic_interruption'] = interruption
    result.update(source_sha256=sources,
                  source_unchanged_during_trial=bool(sources) and sources == after,
                  source_manifest_sha256=hashlib.sha256(
                      json.dumps(sources, sort_keys=True).encode()).hexdigest(),
                  generated_parameters=config)
    actor = summary.load(path/'web_obstacle_injection.json', {})
    candidate = actor.get('candidate') or {}
    result['actor_input_status'] = actor.get('input_status')
    result['actor_input_reason'] = actor.get('input_reason')
    diagnostics = actor.get('candidate_diagnostics') or {}
    result['actor_input_status'] = (result['actor_input_status']
                                    or diagnostics.get('input_status'))
    result['actor_input_reason'] = (result['actor_input_reason']
                                    or diagnostics.get('reason'))
    result['obstacle_scene'] = {k: candidate.get(k) for k in ('x', 'y', 'path_tangent')}
    result['obstacle_scene']['movement_side'] = (candidate.get('preflight') or {}).get('movement_side')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', action='append', required=True, metavar='NAME=STAGE_DIRECTORY')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('--output already exists; choose a new filename to preserve historic evidence')
    rows = [read_trial(label, path) for label, path in summary.pairs(args.trial).items()]
    base = rows[0]
    for row in rows:
        row['changed_parameters_from_first_trial'] = differences(
            base['generated_parameters'], row['generated_parameters'])
        row['same_source_as_first_trial'] = row['source_sha256'] == base['source_sha256']
        row['same_obstacle_scene_as_first_trial'] = row['obstacle_scene'] == base['obstacle_scene']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({
        'schema': 2, 'counter_aggregation_version': 2,
        'scope': 'One trial per candidate is a scenario comparison, not proven reliability or global optimality.',
        'causal_comparison_requires': 'same runtime source, one changed parameter, same obstacle scene; inspect all three fields',
        'trials': rows}, ensure_ascii=False, indent=2)+'\n')
    for row in rows:
        print(row['label'], 'overall=', row['overall_passed'],
              'home=', row['completed_route_and_home'], 'source=', row['same_source_as_first_trial'],
              'scene=', row['same_obstacle_scene_as_first_trial'],
              'parameter_changes=', len(row['changed_parameters_from_first_trial']))


if __name__ == '__main__':
    main()
