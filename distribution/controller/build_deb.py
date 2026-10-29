#!/usr/bin/env python3
"""Build a workspace-independent Ubuntu 22.04 controller package."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

VERSION = '1.1.0'
PACKAGE = 'turtlebot3-controller'
DEPENDENCIES = [
    'python3 (>= 3.10)', 'python3 (<< 3.11)', 'python3-pyqt5',
    'python3-numpy', 'python3-pil', 'python3-yaml',
    'ros-humble-ros-workspace', 'ros-humble-ament-index-python',
    'ros-humble-rclpy', 'ros-humble-action-msgs',
    'ros-humble-geometry-msgs', 'ros-humble-nav-msgs',
    'ros-humble-nav2-msgs', 'ros-humble-rcl-interfaces',
    'ros-humble-sensor-msgs', 'ros-humble-std-srvs',
    'ros-humble-tf2-msgs', 'ros-humble-tf2-ros-py',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    packaging = Path(__file__).resolve().parent
    project = packaging.parents[1]
    source = project / 'tools'
    output = (args.output_dir or project / 'distribution/releases').resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Stable timestamp for release 1.1.0 (2026-10-29 UTC).
    epoch = int(os.environ.get('SOURCE_DATE_EPOCH', '1793232000'))
    with tempfile.TemporaryDirectory(prefix='tb3-controller-deb-') as temporary:
        root = Path(temporary) / 'root'
        payload = {
            'cmd_vel_controller.py': 'usr/lib/turtlebot3-controller/cmd_vel_controller.py',
            'navigation_panels.py': 'usr/lib/turtlebot3-controller/navigation_panels.py',
            'parameter_defaults.py': 'usr/lib/turtlebot3-controller/parameter_defaults.py',
        }
        for name, destination in payload.items():
            target = root / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / name, target)
        resources = {
            'turtlebot3-controller': 'usr/bin/turtlebot3-controller',
            'turtlebot3-controller.desktop': 'usr/share/applications/turtlebot3-controller.desktop',
            'turtlebot3-controller.svg': 'usr/share/icons/hicolor/scalable/apps/turtlebot3-controller.svg',
            'README.ko.txt': 'usr/share/doc/turtlebot3-controller/README.ko.txt',
            'copyright': 'usr/share/doc/turtlebot3-controller/copyright',
        }
        for name, destination in resources.items():
            target = root / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(packaging / name, target)
        files = sorted(p for p in root.rglob('*') if p.is_file())
        installed_size = sum(p.stat().st_size for p in files)
        control = root / 'DEBIAN'
        control.mkdir()
        (control / 'control').write_text(
            f'Package: {PACKAGE}\nVersion: {VERSION}\nSection: education\nPriority: optional\n'
            'Architecture: all\nMaintainer: kiki8105\n'
            f'Installed-Size: {(installed_size + 1023) // 1024}\n'
            f'Depends: {", ".join(DEPENDENCIES)}\n'
            'Recommends: fonts-noto-cjk\n'
            'Homepage: https://github.com/kiki8105/SMART-VISION-AMR\n'
            'Description: ROS 2 Humble TurtleBot3 desktop controller\n'
            ' Camera, map, waypoint navigation, manual driving, and persistent\n'
            ' ROS parameter editing. Select a ROS domain in the desktop app.\n'
            ' No TurtleBot3 source workspace is needed on the controller PC.\n'
            ' Built for Ubuntu 22.04 and Python 3.10.\n', encoding='utf-8')
        (control / 'md5sums').write_text(''.join(
            f'{hashlib.md5(p.read_bytes()).hexdigest()}  {p.relative_to(root)}\n'
            for p in files), encoding='utf-8')
        for path in root.rglob('*'):
            path.chmod(0o755 if path.is_dir() or path == root / 'usr/bin/turtlebot3-controller' else 0o644)
            os.utime(path, (epoch, epoch))
        root.chmod(0o755)
        os.utime(root, (epoch, epoch))
        destination = output / f'{PACKAGE}_{VERSION}_all.deb'
        environment = dict(os.environ, SOURCE_DATE_EPOCH=str(epoch))
        subprocess.run(['dpkg-deb', '--root-owner-group', '-Zxz', '--build',
                        str(root), str(destination)], check=True, env=environment)
        digest = hashlib.sha256(destination.read_bytes()).hexdigest()
        (output / f'{destination.name}.sha256').write_text(f'{digest}  {destination.name}\n')
        print(f'{destination}\nSHA256 {digest}')


if __name__ == '__main__':
    main()
