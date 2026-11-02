"""The read-only observer distinguishes unseen memory from verified clearing."""

import io
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from observe_classroom_memory import MemoryTimeline
import run_classroom_staged_validation as staged


def timeline():
    clock = [0.]
    stream = io.StringIO()
    observer = MemoryTimeline(stream, monotonic=lambda: clock[0],
                              utc_now=lambda: '2026-11-03T00:00:00Z')
    return observer, clock, stream


def test_confirmed_cell_survives_loss_of_live_scan_and_enters_next_goal():
    observer, clock, stream = timeline()
    observer.on_goal(1., 0., 0., 'map', 1)
    observer.on_observed([17])
    clock[0] = 2.
    observer.on_memory([17])
    clock[0] = 3.
    observer.on_observed([])
    assert observer.summary()['persisted_after_live_disappeared'] == 1
    clock[0] = 4.
    observer.on_goal(1., 0., 0., 'map', 2)  # Goal republish, not another sector.
    assert observer.summary()['carried_into_later_goal'] == 0
    observer.on_goal(3., 0., 0., 'map', 3)
    observer.on_visibility([-1] * 17 + [0], 4000000000)
    clock[0] = 5.
    observer.on_memory([])
    report = observer.summary()
    assert report['goal_count'] == 2
    assert report['confirmed_episodes'] == 1
    assert report['persisted_after_live_disappeared'] == 1
    assert report['carried_into_later_goal'] == 1
    assert report['removed_with_positive_free_evidence'] == 1
    assert report['still_remembered'] == []
    events = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert any(event['kind'] == 'goal' and
               event['carried_out_of_view_cells'] == [17] for event in events)
    assert any(event['kind'] == 'remembered_removed' and
               event['positive_free_evidence'] is True for event in events)
    assert all('wall_utc' in event and 'monotonic_sec' in event for event in events)


def test_unseen_cell_or_external_clear_is_not_mislabeled_as_positive_free_ray():
    observer, clock, _ = timeline()
    observer.on_goal(1., 0., 0., 'map', 1)
    observer.on_memory([17])  # Observer attached after hit had already vanished.
    clock[0] = 1.
    observer.on_observed([])
    clock[0] = 2.
    observer.on_goal(3., 0., 0., 'map', 2)
    clock[0] = 3.
    observer.on_memory([])  # Could be manual clear or a process/map reset.
    report = observer.summary()
    assert report['persisted_after_live_disappeared'] == 0
    assert report['carried_into_later_goal'] == 0
    assert report['removed_with_positive_free_evidence'] == 0


def test_memory_stage_includes_readonly_summary_in_derived_evidence(tmp_path):
    stage = staged.BY_KEY['20_web_memory_session']
    assert stage.kind == 'web_dwell' and '--memory-session' in stage.options
    assert stage.route == ('Sector 1', 'Sector 2', 'Sector 3', 'Home')
    expected = {'confirmed_episodes': 2, 'carried_into_later_goal': 1}
    (tmp_path/'memory_observer.summary.json').write_text(json.dumps(expected))
    evidence = {}
    staged.attach_timeline_and_network(stage, tmp_path, evidence)
    assert evidence['memory_session_observer'] == expected
