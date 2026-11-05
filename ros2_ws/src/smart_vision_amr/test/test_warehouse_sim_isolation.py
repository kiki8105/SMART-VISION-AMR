"""Warehouse additions must not enable mock power or launch on the real domain."""
import importlib.util
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location('warehouse_sim_battery', ROOT/'scripts/classroom_sim_battery.py')
battery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(battery)


@pytest.mark.parametrize('domain', [176, 178])
def test_mock_battery_only_joins_matching_local_simulation(domain):
    battery.require_simulation_environment(domain, {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1'})


@pytest.mark.parametrize('domain,environment', [
    (16, {'ROS_DOMAIN_ID': '16', 'ROS_LOCALHOST_ONLY': '1'}),
    (178, {'ROS_DOMAIN_ID': '16', 'ROS_LOCALHOST_ONLY': '1'}),
    (176, {'ROS_DOMAIN_ID': '178', 'ROS_LOCALHOST_ONLY': '1'}),
    (178, {'ROS_DOMAIN_ID': '178', 'ROS_LOCALHOST_ONLY': '0'}),
    (178, {'ROS_DOMAIN_ID': '178'}),
    (179, {'ROS_DOMAIN_ID': '179', 'ROS_LOCALHOST_ONLY': '1'}),
])
def test_mock_battery_refuses_real_nonlocal_or_mismatched_environment(domain, environment):
    with pytest.raises(ValueError):
        battery.require_simulation_environment(domain, environment)


@pytest.mark.parametrize('args,explanation', [
    (['--domain', '16', '--no-verifier'], 'isolated simulation domain'),
    (['--domain', '179', '--no-verifier'], 'isolated simulation domain'),
    (['--domain', '178'], 'requires --no-verifier'),
])
def test_runner_rejects_invalid_domain_before_creating_files_or_launching(tmp_path, args, explanation):
    output = tmp_path/'must-not-exist'
    result = subprocess.run(['bash', str(ROOT/'scripts/run_classroom_fault_sim.sh'),
                             *args, '--output', str(output)], capture_output=True, text=True,
                            env=dict(os.environ, ROS_DOMAIN_ID='16'), timeout=5)
    assert result.returncode == 2
    assert explanation in result.stderr
    assert not output.exists()
