#!/usr/bin/env python3
"""Deterministically assemble a redistributable lab from the existing local projects.

No network access, cloning, recipient installation, or Gazebo startup is performed.
The output is a clean staging tree; build/install/log and machine settings are excluded.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile


PINNED = {
    'turtlebot3': '90a68bd2e3c61c12966779da89d8eeaec82730e9',
    'turtlebot3_simulations': 'a35a56c8b04877dc89772b598084d8ce648a9023',
}
EXCLUDED = {'.git', 'build', 'install', 'log', '__pycache__', '.pytest_cache', '.mypy_cache',
            '.vscode', '.idea', '.DS_Store', '.env', 'checkpoints', '.parameter-history'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_tree(source, target):
    if not source.is_dir():
        raise RuntimeError(f'Missing source directory: {source}')
    for path in sorted(source.rglob('*')):
        rel = path.relative_to(source)
        if any(part in EXCLUDED for part in rel.parts) or path.suffix in ('.pyc', '.pyo') or '.bak-' in path.name:
            continue
        destination = target / rel
        if path.is_symlink():
            raise RuntimeError(f'Unexpected symlink in distributable source: {path}')
        if path.is_dir():
            destination.mkdir(parents=True, exist_ok=True)
        elif path.is_file():
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, destination)
            destination.chmod(0o755 if os.access(path, os.X_OK) else 0o644)


def git_revision(path):
    return subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()


def write_checksums(root):
    lines = [f'{digest(path)}  {path.relative_to(root).as_posix()}\n'
             for path in sorted(root.rglob('*'))
             if path.is_file() and path.name != 'SHA256SUMS']
    (root / 'SHA256SUMS').write_text(''.join(lines))


def build(args):
    project = args.project_root.resolve()
    workspace = args.team_workspace.resolve()
    output = args.output.resolve()
    templates = Path(__file__).resolve().parent / 'templates'
    if output in (Path('/'), Path.home(), workspace, project) or output in project.parents:
        raise RuntimeError('Refusing unsafe output directory')
    if output.exists() and any(output.iterdir()) and not args.replace:
        raise RuntimeError(f'Output exists; pass --replace to recreate it: {output}')
    if output.exists() and any(output.iterdir()) and not (output / 'source_manifest.json').is_file():
        raise RuntimeError('Refusing to replace a nonempty directory without a lab source_manifest.json')
    for name, expected in PINNED.items():
        actual = git_revision(workspace / 'src' / name)
        if actual != expected:
            raise RuntimeError(f'{name}: expected {expected}, found {actual}')
        packages = ['turtlebot3_description', 'turtlebot3_navigation2'] if name == 'turtlebot3' else ['turtlebot3_gazebo']
        dirty = subprocess.check_output(['git', '-C', str(workspace / 'src' / name),
                                         'status', '--porcelain', '--', *packages], text=True).strip()
        if dirty:
            raise RuntimeError(f'Pinned upstream package has local changes: {name}\n{dirty}')
    world = project / 'ros2_ws/src/smart_vision_amr/worlds/simulation_maze_aruco.world'
    package_maps = project / 'ros2_ws/src/smart_vision_amr/maps'
    map_stem = 'maze_aruco_20261026'
    map_yaml_source = package_maps / f'{map_stem}.yaml'
    map_image_source = package_maps / f'{map_stem}.pgm'
    map_info_source = package_maps / f'{map_stem}_mapping_info.json'
    info = json.loads(map_info_source.read_text())
    if info['world_sha256'] != digest(world):
        raise RuntimeError('Measured map world hash differs from current maze world')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=output.name + '.tmp-', dir=output.parent))
    try:
        copy_tree(templates, temporary)
        shutil.copyfile(project / 'scripts/dds_env.bash', temporary / 'tools/dds_env.bash')
        copy_tree(project / 'tools', temporary / 'tools/controller')
        # The launch template belongs to the ROS package, not the bundle root.
        launch_template = temporary / 'aruco_lab.launch.py'
        src = temporary / 'ros2_ws/src'
        copy_tree(project / 'ros2_ws/src/smart_vision_amr', src / 'smart_vision_amr')
        launch_target = src / 'smart_vision_amr/launch/aruco_lab.launch.py'
        launch_template.replace(launch_target)
        for repo, names in (
            ('turtlebot3', ('turtlebot3_description', 'turtlebot3_navigation2')),
            ('turtlebot3_simulations', ('turtlebot3_gazebo',)),
        ):
            for name in names:
                copy_tree(workspace / 'src' / repo / name, src / name)
            license_source = workspace / 'src' / repo / 'LICENSE'
            license_target = temporary / 'licenses' / (repo + '_LICENSE')
            license_target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(license_source, license_target)
        models = temporary / 'assets/models'
        for marker_id in range(4):
            name = f'smart_vision_aruco_4x4_50_id_{marker_id}'
            copy_tree(project / 'gazebo_models' / name, models / name)
            metadata = json.loads((models / name / 'marker.json').read_text())
            assert metadata['id'] == marker_id and metadata['dictionary'] == 'DICT_4X4_50'
            assert metadata['marker_length_m'] == 0.2
            ET.parse(models / name / 'model.sdf')
        layout = json.loads((project / 'gazebo_models/aruco_maze_layout.json').read_text())
        layout['world'] = 'worlds/maze.world'
        (temporary / 'assets/aruco_maze_layout.json').write_text(json.dumps(layout, indent=2) + '\n')
        (temporary / 'worlds').mkdir()
        shutil.copyfile(world, temporary / 'worlds/maze.world')
        ET.parse(temporary / 'worlds/maze.world')
        map_target = temporary / 'maps/maze_aruco_20261026'
        map_target.mkdir(parents=True)
        shutil.copyfile(map_image_source, map_target / 'map.pgm')
        (map_target / 'map.yaml').write_text(map_yaml_source.read_text().replace(
            f'image: {map_stem}.pgm', 'image: map.pgm'))
        preview = package_maps / f'{map_stem}_preview.png'
        if preview.is_file():
            shutil.copyfile(preview, map_target / 'map_preview.png')
        # Replace only the producer's absolute source path with its packaged location.
        info['world'] = 'worlds/maze.world'
        (map_target / 'mapping_info.json').write_text(json.dumps(info, indent=2) + '\n')
        (temporary / 'deb').mkdir()
        if args.deb:
            deb = args.deb.resolve()
            if not deb.is_file() or deb.suffix != '.deb':
                raise RuntimeError(f'Missing controller DEB: {deb}')
            shutil.copyfile(deb, temporary / 'deb' / deb.name)
            # Source execution is always available. The DEB is an optional
            # desktop-menu convenience, not a prerequisite for this lab.
            (temporary / 'install_controller.sh').write_text(
                '#!/usr/bin/env bash\nset -eo pipefail\n'
                'LAB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)\n'
                'sudo apt-get update\n'
                f'sudo apt-get install -y "$LAB_DIR/deb/{deb.name}"\n')
        manifest = {
            'format_version': 1,
            'target': {'os': 'Ubuntu 22.04', 'ros': 'Humble', 'simulator': 'Gazebo Classic 11'},
            'upstream': [
                {'repository': 'https://github.com/ROBOTIS-GIT/turtlebot3', 'commit': PINNED['turtlebot3'],
                 'packages': ['turtlebot3_description', 'turtlebot3_navigation2'], 'license': 'Apache-2.0'},
                {'repository': 'https://github.com/ROBOTIS-GIT/turtlebot3_simulations', 'commit': PINNED['turtlebot3_simulations'],
                 'packages': ['turtlebot3_gazebo'], 'license': 'Apache-2.0'},
            ],
            'team_source_commit': git_revision(project),
            'team_source_note': 'Local working-tree source copied; SHA256SUMS records exact distributed content.',
            'team_package_declared_license': 'MIT (as declared in package.xml/setup.py; no separate license text in source)',
            'maze_world_sha256': digest(world),
            'map_pgm_sha256': digest(map_image_source),
            'original_mapping_info_sha256': digest(map_info_source),
            'map_metadata_portability_change': 'world field rewritten to worlds/maze.world; measured values preserved',
            'marker_layout_portability_change': 'world field rewritten to worlds/maze.world; all marker and observation poses preserved',
            'marker_dictionary': 'DICT_4X4_50', 'marker_ids': [0, 1, 2, 3], 'marker_length_m': 0.2,
            'runtime_aruco_detector_included': True,
            'controller_source_included': True,
        }
        (temporary / 'source_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        for path in temporary.rglob('*'):
            if path.is_file():
                path.chmod(0o755 if path.suffix in ('.sh', '.bash') or path.name == 'save_map.py' else 0o644)
        write_checksums(temporary)
        if output.exists():
            shutil.rmtree(output)
        temporary.replace(output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    print(f'Assembled {output}')
    print(f'Files: {sum(path.is_file() for path in output.rglob("*"))}')
    if args.zip:
        archive = output.with_suffix('.zip')
        with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as bundle:
            for path in sorted(output.rglob('*')):
                if path.is_file():
                    bundle.write(path, Path(output.name) / path.relative_to(output))
        checksum = digest(archive)
        archive.with_suffix('.zip.sha256').write_text(f'{checksum}  {archive.name}\n')
        print(f'Archive: {archive}\nSHA256 {checksum}')
    return 0


def main():
    project = Path(__file__).resolve().parents[2]
    workspace = project.parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=project)
    parser.add_argument('--team-workspace', type=Path, default=workspace)
    parser.add_argument('--output', type=Path, default=project / 'distribution/staging/turtlebot3-aruco-lab')
    parser.add_argument('--deb', type=Path, help='Include an already built controller DEB')
    parser.add_argument('--zip', action='store_true', help='Create OUTPUT.zip and its SHA-256 beside the staging tree')
    parser.add_argument('--replace', action='store_true', help='Recreate the specified staging directory')
    return build(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
