"""Exercise saved-map launch wrappers without ROS or a personal Gazebo folder."""

import json
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]


@pytest.fixture
def portable_repo(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    for name in ('run_web_mission_sim.sh', 'run_curve_tuning_sim.sh', 'run_sim_aruco.sh'):
        shutil.copyfile(REPO / 'scripts' / name, scripts / name)
        (scripts / name).chmod(0o755)
    # Preserve the real wrapper's input checks while avoiding a ROS launch.
    (scripts / 'sim_env.bash').write_text(
        'export SMART_VISION_SIM_WORKSPACE="$1"\n'
        'export GAZEBO_MODEL_PATH="$REPO_DIR/gazebo_models"\n')
    package = tmp_path / 'ros2_ws/src/smart_vision_amr'
    for directory, names in (
        ('config', ('curve_tuning_simulation.yaml', 'curve_tuning_simulation_nodes.yaml')),
        ('worlds', ('simulation_maze_aruco.world',)),
        ('maps', ('maze_aruco_20261026.yaml',)),
    ):
        target = package / directory
        target.mkdir(parents=True)
        for name in names:
            (target / name).write_text('fixture\n')
    for marker_id in range(4):
        model = tmp_path / f'gazebo_models/smart_vision_aruco_4x4_50_id_{marker_id}'
        model.mkdir(parents=True)
        (model / 'model.sdf').write_text('<sdf/>\n')
    binary_dir = tmp_path / 'bin'
    binary_dir.mkdir()
    ros2 = binary_dir / 'ros2'
    ros2.write_text(
        '#!/usr/bin/env python3\nimport json, os, sys\n'
        'print(json.dumps({"args": sys.argv[1:], '
        '"models": os.environ["GAZEBO_MODEL_PATH"]}))\n')
    ros2.chmod(0o755)
    environment = dict(os.environ, PATH=f'{binary_dir}:{os.environ["PATH"]}',
                       HOME=str(tmp_path / 'empty_home'))
    return tmp_path, environment


def run_wrapper(fixture, *arguments, script='run_web_mission_sim.sh'):
    root, environment = fixture
    return subprocess.run(
        ['bash', str(root / 'scripts' / script), *arguments],
        cwd=root, env=environment, text=True, capture_output=True, timeout=5)


def test_saved_map_image_is_bundled_and_relative():
    map_file = PACKAGE / 'maps/maze_aruco_20261026.yaml'
    metadata = yaml.safe_load(map_file.read_text())
    assert not Path(metadata['image']).is_absolute()
    image = map_file.parent / metadata['image']
    assert image.is_file()
    assert image.read_bytes().startswith(b'P5\n202 164\n255\n')


def test_saved_map_matches_world_and_marker_assets_are_complete():
    metadata = json.loads((PACKAGE / 'maps/maze_aruco_20261026_mapping_info.json').read_text())
    world = REPO / metadata['world']
    assert hashlib.sha256(world.read_bytes()).hexdigest() == metadata['world_sha256']
    models = REPO / 'gazebo_models'
    for marker_id in range(4):
        model = models / f'smart_vision_aruco_4x4_50_id_{marker_id}'
        assert (model / 'model.config').is_file()
        assert list((model / 'materials/textures').glob('*.png'))
        for uri in ET.parse(model / 'model.sdf').findall('.//uri'):
            if uri.text.startswith('model://'):
                assert (models / uri.text.removeprefix('model://')).exists()


@pytest.mark.parametrize('script', ['run_web_mission_sim.sh', 'run_curve_tuning_sim.sh'])
def test_default_launch_uses_repo_map_and_models_without_home_install(portable_repo, script):
    root, _environment = portable_repo
    result = run_wrapper(portable_repo, script=script)
    assert result.returncode == 0, result.stderr
    recorded = json.loads(result.stdout.splitlines()[-1])
    assert f'map:={root}/ros2_ws/src/smart_vision_amr/maps/maze_aruco_20261026.yaml' in recorded['args']
    assert recorded['models'].split(':')[0] == str(root / 'gazebo_models')


def test_custom_map_works_when_unused_default_is_absent(portable_repo):
    root, _environment = portable_repo
    (root / 'ros2_ws/src/smart_vision_amr/maps/maze_aruco_20261026.yaml').unlink()
    custom_map = root / 'custom map.yaml'
    custom_map.write_text('image: custom.pgm\n')
    result = run_wrapper(portable_repo, 'map:=/missing/earlier.yaml', f'map:={custom_map}')
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout.splitlines()[-1])['args'][-1] == f'map:={custom_map}'


def test_missing_selected_input_is_reported_before_launch(portable_repo):
    result = run_wrapper(portable_repo, 'map:=/missing/selected.yaml')
    assert result.returncode == 2
    assert 'Simulation input is missing: /missing/selected.yaml' in result.stderr
    assert '"args"' not in result.stdout


def test_fresh_aruco_mapping_needs_no_personal_model_registration(portable_repo):
    root, _environment = portable_repo
    downstream = root / 'scripts/run_sim.sh'
    downstream.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    downstream.chmod(0o755)
    result = run_wrapper(portable_repo, 'gui:=false', script='run_sim_aruco.sh')
    assert result.returncode == 0, result.stderr
    assert 'slam:=true' in result.stdout.splitlines()
    assert 'gui:=false' in result.stdout.splitlines()
    assert not (root / 'empty_home/.gazebo/models').exists()
