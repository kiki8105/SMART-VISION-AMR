import hashlib
import json
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parent))
from parameter_defaults import save_parameter_defaults, restore_parameter_history, ParameterDefaultsError


def fixture(tmp_path):
    path = tmp_path/'nodes.yaml'
    original = b'planner:\n  ros__parameters:\n    speed: 0.1 # preserve explanation\n'
    path.write_bytes(original)
    return path, original


def test_history_and_safe_rollback(tmp_path):
    path, original = fixture(tmp_path)
    save_parameter_defaults([path], {('/planner', 'speed'): 0.2})
    receipt = next((tmp_path/'.parameter-history').glob('*.json'))
    record = json.loads(receipt.read_text())
    assert record['status'] == 'committed'
    assert record['changes'] == [{'node': '/planner', 'parameter': 'speed', 'value': 0.2, 'previous_value': 0.1}]
    assert record['files'][0]['after_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert '# preserve explanation' in path.read_text()
    restore_parameter_history(receipt)
    assert path.read_bytes() == original
    assert len(list((tmp_path/'.parameter-history').glob('*.json'))) == 2


def test_rollback_never_overwrites_later_tuning(tmp_path):
    path, _ = fixture(tmp_path)
    save_parameter_defaults([path], {('/planner', 'speed'): 0.2})
    receipt = next((tmp_path/'.parameter-history').glob('*.json'))
    path.write_text('planner:\n  ros__parameters:\n    speed: 0.3\n')
    with pytest.raises(ParameterDefaultsError, match='다른 변경'):
        restore_parameter_history(receipt)
    assert '0.3' in path.read_text()


def test_rollback_rejects_tampered_backup(tmp_path):
    path, _ = fixture(tmp_path)
    save_parameter_defaults([path], {('/planner', 'speed'): 0.2})
    receipt = next((tmp_path/'.parameter-history').glob('*.json'))
    backup = Path(json.loads(receipt.read_text())['files'][0]['backup'])
    backup.write_text('bad')
    with pytest.raises(ParameterDefaultsError, match='해시'):
        restore_parameter_history(receipt)
