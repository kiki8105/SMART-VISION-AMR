"""The fixed-box QA must use the actual copied body, with the same 2 cm margin."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from test_classroom_stress_crate_evidence import (
    assessment, complete_web_evidence, safe_trace,
)


def test_actual_22cm_body_passes_26mm_gap_while_stock_24cm_fails_6mm_gap():
    records = safe_trace()
    for row in records:
        if row.get('kind') == 'truth':
            row['robot'][0] = .7135
    stock = assessment.assess(records, complete_web_evidence())
    physical = assessment.assess(records, complete_web_evidence(), robot_radius=.22)
    assert stock['metrics']['minimum_truth_body_clearance_m'] == pytest.approx(.006)
    assert physical['metrics']['minimum_truth_body_clearance_m'] == pytest.approx(.026)
    assert not stock['passed']
    assert stock['reasons'] == ['gazebo_body_clearance_below_2cm_or_unproven']
    assert physical['passed'], physical
    assert stock['metrics']['required_margin_m'] == physical['metrics']['required_margin_m'] == .02
    assert assessment.BODY_RADIUS_M == .24  # no global mutable radius between trials


@pytest.mark.parametrize('radius', [.22, .24])
def test_radius_selection_never_manufactures_missing_truth_or_lidar(radius):
    for kind, reason in [('truth', 'fixed_crate_runtime_presence_unproven'),
                         ('scan_crate', 'raw_lidar_crate_return_unproven')]:
        rows = [row for row in safe_trace() if row.get('kind') != kind]
        result = assessment.assess(rows, complete_web_evidence(), robot_radius=radius)
        assert not result['passed']
        assert reason in result['reasons']


@pytest.mark.parametrize('radius', [0., -1., float('inf'), float('nan'), True])
def test_invalid_radius_is_rejected(radius):
    with pytest.raises(ValueError):
        assessment.assess(safe_trace(), complete_web_evidence(), robot_radius=radius)


def make_manifest(tmp_path, *, radius=.22, actual=.22, centered=True):
    manifest = {'physical_footprint': True,
                'sim_footprint_overrides': {'robot_radius_m': radius},
                'collision_model': {'copied_centered_cylinder_radius_m': radius}}
    path = tmp_path/'runtime_manifest.json'
    path.write_text(json.dumps(manifest))
    fixture = tmp_path/'fixture'
    fixture.mkdir(exist_ok=True)
    (fixture/'waffle_pi_fault.sdf').write_text(
        f'<sdf><model><link><collision name="base_collision">'
        f'<pose>{0 if centered else .01} 0 .048 0 0 0</pose>'
        f'<geometry><cylinder><radius>{actual}</radius><length>.089</length>'
        '</cylinder></geometry></collision></link></model></sdf>')
    return path


def test_physical_radius_requires_matching_manifest_and_actual_centered_sdf(tmp_path):
    path = make_manifest(tmp_path)
    assert assessment.verified_runtime_robot_radius(path) == .22
    make_manifest(tmp_path, actual=.24)
    with pytest.raises(ValueError, match='disagrees'):
        assessment.verified_runtime_robot_radius(path)
    make_manifest(tmp_path, centered=False)
    with pytest.raises(ValueError, match='disagrees'):
        assessment.verified_runtime_robot_radius(path)
    (tmp_path/'fixture/waffle_pi_fault.sdf').unlink()
    with pytest.raises(OSError):
        assessment.verified_runtime_robot_radius(path)


def test_stock_manifest_keeps_24cm_and_unverified_smaller_body_is_rejected(tmp_path):
    path = tmp_path/'runtime_manifest.json'
    manifest = {'physical_footprint': False,
                'sim_footprint_overrides': {'robot_radius_m': .24}}
    path.write_text(json.dumps(manifest))
    assert assessment.verified_runtime_robot_radius(path) == .24
    manifest['sim_footprint_overrides']['robot_radius_m'] = .22
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='unverified'):
        assessment.verified_runtime_robot_radius(path)


def test_cli_passes_explicit_radius_without_changing_qa(tmp_path):
    trace, evidence, output = (tmp_path/name for name in ('trace.jsonl', 'web.json', 'qa.json'))
    trace.write_text('\n'.join(json.dumps(row) for row in safe_trace()))
    evidence.write_text(json.dumps(complete_web_evidence()))
    command = [sys.executable, str(Path(assessment.__file__)), '--trace', str(trace),
               '--web-evidence', str(evidence), '--output', str(output),
               '--robot-radius', '.22']
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_text())
    assert report['metrics']['robot_enclosing_radius_m'] == .22
    assert report['metrics']['required_margin_m'] == .02
    output.unlink()
    result = subprocess.run(command[:-1]+['nan'], capture_output=True, text=True)
    assert result.returncode == 2
    assert not output.exists()


def test_staged_assessor_receives_verified_sdf_radius_and_rejects_mismatch(tmp_path, monkeypatch):
    import run_classroom_staged_validation as staged
    make_manifest(tmp_path)
    (tmp_path/'stress_crate_trace.jsonl').write_text('')
    stage = staged.Stage('17_web_hardware_value_parity', 'radius regression',
                         ('--stress',), ('Sector 1', 'Sector 2', 'Sector 3'), 'web_dwell')
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        assert command[command.index('--robot-radius')+1] == '0.22'
        assert command[command.index('--expected-route')+1] == '1,2,3'
        Path(command[command.index('--output')+1]).write_text(json.dumps(
            {'passed': True, 'metrics': {'robot_enclosing_radius_m': .22,
                                        'required_margin_m': .02}}))
        return SimpleNamespace(returncode=0, stderr='')

    monkeypatch.setattr(staged.subprocess, 'run', run)
    result = {}
    staged.attach_timeline_and_network(stage, tmp_path, result)
    assert len(commands) == 1
    assert result['stress_crate_assessment']['runtime_body_radius_verified'] is True
    make_manifest(tmp_path, actual=.24)
    result = {}
    staged.attach_timeline_and_network(stage, tmp_path, result)
    assert len(commands) == 1  # stale old assessment cannot override verification failure
    assert result['stress_crate_assessment']['passed'] is False
    assert result['stress_crate_assessment']['reasons'] == ['runtime_body_radius_unverified']
