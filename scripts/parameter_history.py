#!/usr/bin/env python3
"""Inspect/restore controller YAML tuning transactions (does not change live ROS)."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from parameter_defaults import restore_parameter_history


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', help='YAML directory for history, or receipt JSON with --restore')
    parser.add_argument('--restore', action='store_true', help='Restore only if current hashes still match the receipt')
    args = parser.parse_args()
    path = Path(args.path).expanduser().resolve()
    if args.restore:
        for restored in restore_parameter_history(path):
            print('Restored YAML:', restored)
        print('Live nodes are unchanged. Apply the settings or restart the affected nodes explicitly.')
        return
    directory = path if path.name == '.parameter-history' else path/'.parameter-history'
    for entry in sorted(directory.glob('*.json'), reverse=True):
        record = json.loads(entry.read_text())
        print(record['at'], record['status'], entry)
        for change in record.get('changes', []):
            print(' ', change['node'], change['parameter'], '=', change['value'])


if __name__ == '__main__':
    main()
