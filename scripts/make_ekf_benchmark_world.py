#!/usr/bin/env python3
"""Create a test-only copy of a Gazebo Classic world with world-pose truth."""

import argparse
from pathlib import Path


STATE_PLUGIN = '''
    <!-- EKF benchmark only: ground-truth pose independent of /odom and AMCL. -->
    <plugin name="ekf_benchmark_ground_truth" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
'''


def make_world(source):
    text = source.read_text()
    if 'libgazebo_ros_state.so' in text:
        raise ValueError('Source world already publishes Gazebo state')
    closing = '</world>'
    if text.count(closing) != 1:
        raise ValueError('Expected exactly one world closing tag')
    return text.replace(closing, STATE_PLUGIN + '  ' + closing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(make_world(args.source))
    print(args.output)


if __name__ == '__main__':
    main()
