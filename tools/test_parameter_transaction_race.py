"""A failed multi-file save must not undo a later edit from another process."""

import json
import os
from pathlib import Path
from unittest.mock import patch

import pytest

import parameter_defaults as defaults


def test_rollback_preserves_external_edit_after_an_earlier_file_was_replaced(tmp_path):
    first, second = tmp_path/'first.yaml', tmp_path/'second.yaml'
    original_first = 'first:\n  ros__parameters:\n    speed: 0.1\n'
    original_second = 'second:\n  ros__parameters:\n    speed: 0.2\n'
    later_edit = 'first:\n  ros__parameters:\n    speed: 0.3 # later operator tuning\n'
    first.write_text(original_first)
    second.write_text(original_second)
    replace = os.replace

    def fail_after_external_edit(source, target):
        if Path(target) == second:
            first.write_text(later_edit)
            raise OSError('second file replacement failed')
        return replace(source, target)

    with patch.object(defaults.os, 'replace', side_effect=fail_after_external_edit):
        with pytest.raises(defaults.ParameterDefaultsError, match='다른 변경'):
            defaults.save_parameter_defaults([first, second], {
                ('/first', 'speed'): .15, ('/second', 'speed'): .25})
    assert first.read_text() == later_edit
    assert second.read_text() == original_second
    receipt = json.loads(next((tmp_path/'.parameter-history').glob('*.json')).read_text())
    assert receipt['status'] == 'prepared'
    assert Path(receipt['files'][0]['backup']).read_text() == original_first
