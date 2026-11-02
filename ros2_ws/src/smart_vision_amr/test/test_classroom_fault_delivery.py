"""Fault acceptance must distinguish an applied dropout from a requested one."""

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[4]/'scripts/assess_classroom_fault_delivery.py'
SPEC = importlib.util.spec_from_file_location('assess_classroom_fault_delivery', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_clean_profile_requires_real_sensor_forwarding_but_no_fault_request():
    events = [{'event': 'sample', 'profile': 'clean',
               'counts': {'scan_forwarded': 3, 'odom_forwarded': 5, 'tf_forwarded': 5}}]
    assert MODULE.assess(events, 'clean', 'baseline')['passed']


def test_outage_request_without_actual_loss_is_rejected():
    events = [{'event': 'profile', 'name': 'outages'},
              {'event': 'sample', 'profile': 'outages',
               'counts': {'scan_forwarded': 3, 'odom_forwarded': 5, 'tf_forwarded': 5}}]
    result = MODULE.assess(events, 'outages', 'baseline')
    assert not result['passed']
    assert 'scan_outage_not_measured' in result['missing']


def test_independent_ekf_only_requires_scan_fault():
    events = [{'event': 'profile', 'name': 'mixed'},
              {'event': 'sample', 'profile': 'mixed',
               'counts': {'scan_forwarded': 3, 'scan_dropped_outage': 5},
               'max_delivery_delay_s': {'scan': .2}}]
    assert MODULE.assess(events, 'mixed', 'ekf_independent')['passed']
    assert not MODULE.assess(events, 'mixed', 'baseline')['passed']
