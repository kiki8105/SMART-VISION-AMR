"""The Gazebo TF QoS experiment must not change the physical or forwarded TF path."""

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


SCRIPTS = Path(__file__).resolve().parents[4] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location('classroom_fault_relay',
                                               SCRIPTS / 'classroom_fault_relay.py')
RELAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RELAY)


def test_raw_tf_qos_is_reliable_by_default_and_best_effort_only_when_requested():
    default = RELAY.tf_source_qos(False)
    experimental = RELAY.tf_source_qos(True)
    assert (default.reliability, default.depth, default.durability) == (
        ReliabilityPolicy.RELIABLE, 100, DurabilityPolicy.VOLATILE)
    assert (experimental.reliability, experimental.depth, experimental.durability) == (
        ReliabilityPolicy.BEST_EFFORT, 20, DurabilityPolicy.VOLATILE)
    with pytest.raises(ValueError, match='boolean'):
        RELAY.tf_source_qos(1)


def test_runner_exposes_opt_in_without_allowing_irrelevant_ekf_mode():
    runner = SCRIPTS / 'run_classroom_fault_sim.sh'
    help_result = subprocess.run(['bash', str(runner), '--help'], capture_output=True,
                                 text=True, check=True)
    assert '--tf-source-best-effort' in help_result.stdout
    invalid = subprocess.run(['bash', str(runner), '--mode', 'ekf_independent',
                              '--tf-source-best-effort'], capture_output=True, text=True)
    assert invalid.returncode == 2
    assert 'only applies to baseline' in invalid.stderr
