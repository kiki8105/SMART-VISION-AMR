"""A real pre-mission lifecycle abort is not a delivered transient fault."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import run_classroom_staged_validation as staged


@pytest.fixture
def failed_startup(tmp_path):
    # Eight actual witness lines around the first v2 failure, with ANSI stripped.
    text = '\n'.join([
        '[controller_server] DWB local route horizon: 0.55 m along path; anchor search: 0.85 m',
        '[controller_server] Controller Server has FollowPath controllers available.',
        '[controller_server] failed to send response to /controller_server/get_state (timeout): client will not receive response',
        '[lifecycle_manager] Failed to change state for node: controller_server. Exception: controller_server/get_state service client: async_send_request failed.',
        '[lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.',
        '[gazebo_state] Publishing states of gazebo models at [/gazebo/model_states]',
        '[spawn_entity] Calling service /spawn_entity',
        '[spawn_entity] Spawn status: SpawnEntity: Successfully spawned entity [waffle_pi]',
    ])+'\n'
    (tmp_path/'gazebo_launch.log').write_text(text)
    (tmp_path/'sensor_faults.jsonl').write_text(json.dumps(
        {'event':'sample','profile':'clean','counts':{'odom_forwarded':1235,'tf_forwarded':1235}})+'\n')
    (tmp_path/'safety_timeline.jsonl').write_text(json.dumps(
        {'kind':'cmd_vel','linear':0.,'angular':0.})+'\n')
    return tmp_path, {'passed':False,'mission_accepted_via_web':False,
                      'transient_requested':True,'route':[1,2]}


def test_actual_abort_without_motion_or_fault_is_failure_not_transient_result(failed_startup):
    path, evidence = failed_startup
    result = staged.classify_startup_lifecycle_failure(path,evidence)
    assert result['failure_class']=='STARTUP_LIFECYCLE_ABORT'
    assert result['fault_execution']=='TRANSIENT_NOT_STARTED'
    assert result['startup_failure_witness']['sha256']==hashlib.sha256((path/'gazebo_launch.log').read_bytes()).hexdigest()
    evidence.update(result)
    verdict=staged.assess(staged.BY_KEY['16_web_transient_resume'],evidence)
    assert not verdict['passed'] and verdict['outcome']=='FAIL'
    assert verdict['failure_class']=='STARTUP_LIFECYCLE_ABORT'
    assert 'transient_not_started' in verdict['reasons']


@pytest.mark.parametrize('change',[
    {'passed':True}, {'mission_accepted_via_web':True}, {'mission_started_at':10.},
    {'transient_injected_mid_route':True}, {'transient_profile_request':{'profile_requested':'transient'}},
])
def test_success_or_started_mission_or_requested_fault_cannot_be_relabelled(failed_startup,change):
    path,evidence=failed_startup
    assert staged.classify_startup_lifecycle_failure(path,{**evidence,**change})=={}


@pytest.mark.parametrize('witness',['no_abort','profile_applied','goal','nonzero','missing_relay'])
def test_positive_evidence_is_required_and_actual_execution_is_not_hidden(failed_startup,witness):
    path,evidence=failed_startup
    if witness=='no_abort': (path/'gazebo_launch.log').write_text('only normal configure logs\n')
    if witness=='profile_applied':
        with (path/'sensor_faults.jsonl').open('a') as stream:
            stream.write(json.dumps({'event':'profile','name':'transient'})+'\n')
    if witness in ('goal','nonzero'):
        with (path/'safety_timeline.jsonl').open('a') as stream:
            stream.write(json.dumps({'kind':'goal'} if witness=='goal' else
                                    {'kind':'cmd_vel','linear':.02,'angular':0.})+'\n')
    if witness=='missing_relay': (path/'sensor_faults.jsonl').unlink()
    assert staged.classify_startup_lifecycle_failure(path,evidence)=={}
