"""Owned route/recovery identity across delayed action and DDS callbacks."""
from copy import deepcopy
import json
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import test_path_follower_backup as backup
from test_path_follower_cancel import goal_for, make_path, result_future
from test_local_detour_planner import planner_factory, hits
from test_path_follower_cancel import FollowerHarness
from smart_vision_amr.recovery_episode import RecoveryPhase


@pytest.fixture
def follower():
    case=backup.FollowerBackupTest(methodName='test_recovery_waits_for_follow_path_to_finish')
    case.setUp()
    case.feed_sensors()
    return case


def packets(case):
    return [json.loads(call.args[0].data) for call in case.node.status_pub.publish.call_args_list]


def request(case, episode='episode-A', **changes):
    payload={'goal_id':'10:0','distance':.2,'speed':.04,
             'certified_start':case.certificate()}
    if episode is not None:
        payload['episode_id']=episode
    payload.update(changes)
    case.node.on_recovery_request(String(data=json.dumps(payload)))


def cancel(case, episode, goal_id='10:0'):
    case.node.on_recovery_request(String(data=json.dumps({
        'goal_id':goal_id,'episode_id':episode,'cancel':True,'reason':'map changed'})))


def test_follow_context_exact_path_stamp_is_owned_snapshot(follower):
    context=follower.follow_context
    assert context['path_stamp']=={'sec':11,'nanosec':0}
    original=deepcopy(context)
    follower.node.clock_seconds=14
    follower.node.on_goal(goal_for(make_path(stamp=14),stamp=13))
    follower.node.publish_status('FAILED','Delayed old route terminal',context=context)
    packet=packets(follower)[-1]
    assert packet['goal_stamp']==original['goal_stamp']=={'sec':10,'nanosec':0}
    assert packet['path_stamp']==original['path_stamp']=={'sec':11,'nanosec':0}
    assert context==original


def test_execution_failure_echoes_actual_owned_path_stamp(follower):
    event=follower.node._execution_payload('execution_no_progress',{
        'ros_ns':12_000_000_000,'pose':{'x':0.,'y':0.,'yaw':0.},
        'segment':{'start':{'x':0.,'y':0.},'end':{'x':.22,'y':0.},'heading':0.}},
        {'duration_sec':6.})
    assert event['path_stamp']==follower.follow_context['path_stamp']=={'sec':11,'nanosec':0}
    assert event['goal_stamp']=={'sec':10,'nanosec':0}


@pytest.mark.parametrize('suffix',[False,True])
def test_equivalent_route_alias_updates_shared_action_context_without_restart(follower,suffix):
    owned=follower.follow_context
    alias=make_path(stamp=13)
    if suffix:
        alias.poses=alias.poses[1:]
    before=deepcopy(owned)
    follower.node.status_pub.publish.reset_mock()
    follower.node.on_path(alias)
    assert owned is follower.node.active_context
    assert owned['goal_stamp']==before['goal_stamp']
    assert owned['path_stamp']=={'sec':13,'nanosec':0}
    ack=packets(follower)[-1]
    assert ack['status']=='MOVING' and ack['route_alias_accepted'] is True
    assert ack['path_stamp']=={'sec':13,'nanosec':0}
    follower.node.on_path(alias)
    assert len(packets(follower))==1
    follower.follow_handle.cancel_goal_async.assert_not_called()
    assert follower.node.client.send_goal_async.call_count==1
    follower.node.on_result(result_future(GoalStatus.STATUS_SUCCEEDED),owned)
    assert packets(follower)[-1]['status']=='SUCCEEDED'
    assert packets(follower)[-1]['path_stamp']=={'sec':13,'nanosec':0}


def test_unaccepted_different_route_cannot_relabel_old_action_terminal(follower):
    owned=follower.follow_context
    changed=make_path(y=1.,stamp=13)
    follower.node.on_goal(goal_for(changed,stamp=12))
    follower.node.on_path(changed)
    assert owned['path_stamp']=={'sec':11,'nanosec':0}
    follower.node.on_result(result_future(GoalStatus.STATUS_SUCCEEDED),owned)
    assert not any(p['status']=='SUCCEEDED' for p in packets(follower))


def test_alias_during_existing_cancel_waits_for_actual_terminal(follower):
    owned=follower.follow_context
    follower.node._cancel_active_goal()
    alias=make_path(stamp=13)
    follower.node.on_path(alias)
    assert follower.node.pending_path is alias
    assert owned['path_stamp']=={'sec':11,'nanosec':0}
    assert follower.node.client.send_goal_async.call_count==1
    follower.node.on_result(result_future(GoalStatus.STATUS_CANCELED),owned)
    assert follower.node.client.send_goal_async.call_count==2
    assert follower.node.request_context['path_stamp']=={'sec':13,'nanosec':0}


@pytest.mark.parametrize('recovery_before_terminal',[False,True])
def test_planner_follower_alias_success_connection_preserves_episode_ownership(
        planner_factory,recovery_before_terminal):
    planner=planner_factory()
    # The harness clears publisher history after initial planning. Reconstruct
    # that exact stored geometry without invoking another search.
    original=Path();original.header.frame_id='map'
    original.header.stamp.sec=planner.current_path_stamp_ns//1_000_000_000
    original.header.stamp.nanosec=planner.current_path_stamp_ns%1_000_000_000
    for x,y in planner.planned_world_points:
        pose=PoseStamped();pose.header=deepcopy(original.header)
        pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.
        original.poses.append(pose)
    original.poses[-1].pose.orientation=deepcopy(planner.last_goal.pose.orientation)
    follower=FollowerHarness();follower.clock_seconds=100
    follower.on_goal(deepcopy(planner.last_goal));follower.on_path(original)
    owned=follower.request_context
    handle=Mock(accepted=True)
    follower.on_goal_response(Mock(result=Mock(return_value=handle)),owned)
    planner.recovery_coordinator.begin('alias-episode',planner._local_route_key(),
                                       {(17,25)},set(),set())
    alias=deepcopy(original);alias.header.stamp.nanosec+=1
    alias.poses=alias.poses[2:]
    planner._publish_path(alias)
    follower.on_path(alias)
    ack=json.loads(follower.status_pub.publish.call_args.args[0].data)
    planner.on_navigation_status(String(data=json.dumps(ack)))
    assert planner.recovery_coordinator.current.phase is RecoveryPhase.FOLLOW
    assert owned['path_stamp']==ack['path_stamp']
    assert follower.client.send_goal_async.call_count==1
    handle.cancel_goal_async.assert_not_called()
    if recovery_before_terminal:
        planner._try_local_detour=Mock(return_value=False)
        planner.on_observed(hits(planner,{(17,25)}))
        assert planner.recovery_coordinator.current.phase is RecoveryPhase.STOP_PROOF
    follower.on_result(result_future(GoalStatus.STATUS_SUCCEEDED),owned)
    terminal=json.loads(follower.status_pub.publish.call_args.args[0].data)
    assert terminal['status']=='SUCCEEDED'
    planner.on_navigation_status(String(data=json.dumps(terminal)))
    assert planner.goal_completed==(not recovery_before_terminal)


def test_recovery_id_echoed_by_accept_start_and_actual_terminal(follower):
    request(follower)
    assert follower.node.recovery_request['context']['episode_id']=='episode-A'
    follower.finish_follow_cancel()
    handle,sequence=follower.accept_backup()
    follower.node._on_backup_result(result_future(GoalStatus.STATUS_SUCCEEDED),sequence,handle)
    owned=[packet for packet in packets(follower) if packet['status'].startswith('RECOVER')]
    assert [packet['status'] for packet in owned]==['RECOVERING','RECOVERING','RECOVERY_SUCCEEDED']
    assert all(packet['episode_id']=='episode-A' for packet in owned)
    assert all(packet['goal_stamp']=={'sec':10,'nanosec':0} for packet in owned)


def test_backup_stopped_after_goal_change_preserves_cancelled_episode_identity(follower):
    request(follower)
    follower.finish_follow_cancel()
    handle,sequence=follower.accept_backup()
    follower.node.clock_seconds=14
    follower.node.on_goal(goal_for(make_path(stamp=14),stamp=13))
    assert follower.node.recovery_request is None
    follower.node._on_backup_result(result_future(GoalStatus.STATUS_CANCELED),sequence,handle)
    stopped=[packet for packet in packets(follower) if packet['status']=='RECOVERY_STOPPED']
    assert len(stopped)==1
    assert stopped[0]['episode_id']=='episode-A'
    assert stopped[0]['goal_stamp']=={'sec':10,'nanosec':0}
    assert follower.node.requested_goal.header.stamp.sec==13


def test_new_route_discarded_at_old_backup_stop_requests_fresh_route_once(follower):
    request(follower)
    follower.finish_follow_cancel()
    handle,sequence=follower.accept_backup()
    follower.node.clock_seconds=14
    pending=make_path(y=1.,stamp=14)
    follower.node.on_goal(goal_for(pending,stamp=13))
    follower.node.on_path(pending)
    assert follower.node.pending_path is pending
    assert follower.node.client.send_goal_async.call_count==1
    follower.node._on_backup_result(result_future(GoalStatus.STATUS_CANCELED),sequence,handle)
    stopped=[packet for packet in packets(follower) if packet['status']=='RECOVERY_STOPPED']
    holds=[packet for packet in packets(follower) if packet['status']=='WAIT_REPLAN']
    assert len(stopped)==len(holds)==1
    assert stopped[0]['episode_id']=='episode-A'
    assert stopped[0]['goal_stamp']=={'sec':10,'nanosec':0}
    assert holds[0]['goal_stamp']=={'sec':13,'nanosec':0}
    assert holds[0]['path_stamp']=={'sec':14,'nanosec':0}
    assert holds[0]['route_failure'] is False and holds[0]['input_hold'] is True
    assert follower.node.pending_path is None
    # Duplicate old terminal cannot request another route or dispatch motion.
    follower.node._on_backup_result(result_future(GoalStatus.STATUS_CANCELED),sequence,handle)
    assert len([p for p in packets(follower) if p['status']=='WAIT_REPLAN'])==1
    assert follower.node.client.send_goal_async.call_count==1
    # A fresh planner replacement can now pass the real action barrier.
    replacement=make_path(y=1.,stamp=15)
    follower.node.on_path(replacement)
    assert follower.node.client.send_goal_async.call_count==2
    assert follower.node.backup_goal is None


def test_old_same_goal_cancel_cannot_abort_different_active_episode(follower):
    request(follower,'episode-B')
    before=follower.node.recovery_request
    packets_before=len(packets(follower))
    cancel(follower,'episode-A')
    assert follower.node.recovery_request is before
    assert follower.node.pending_recovery_cancel is None
    assert len(packets(follower))==packets_before


@pytest.mark.parametrize('backup_started',[False,True])
def test_delayed_same_goal_path_cannot_supersede_episode_bound_reverse(follower,backup_started):
    request(follower)
    owned=follower.node.recovery_request
    if backup_started:
        follower.finish_follow_cancel()
        handle,_=follower.accept_backup()
        handle.cancel_goal_async.reset_mock()
    for stamp,nanosec in ((12,1),(13,0)):
        delayed=make_path(stamp=stamp)
        delayed.header.stamp.nanosec=nanosec
        follower.node.on_path(delayed)
    assert follower.node.recovery_request is owned
    assert follower.node.pending_path is None
    assert follower.node.client.send_goal_async.call_count==1
    if backup_started:
        handle.cancel_goal_async.assert_not_called()
    else:
        follower.node.backup_client.send_goal_async.assert_not_called()


def test_matching_cancel_echoes_episode_and_still_cancels(follower):
    request(follower)
    cancel(follower,'episode-A')
    assert follower.node.recovery_request is None
    assert packets(follower)[-1]['status']=='RECOVERY_FAILED'
    assert packets(follower)[-1]['episode_id']=='episode-A'


def test_old_buffered_cancel_does_not_reject_new_same_goal_episode(follower):
    cancel(follower,'episode-A')
    assert follower.node.pending_recovery_cancel['episode_id']=='episode-A'
    request(follower,'episode-B')
    assert follower.node.recovery_request['context']['episode_id']=='episode-B'


def test_future_buffered_episode_not_discarded_by_older_same_goal_cancel(follower):
    request(follower,'episode-B',goal_id='13:0')
    pending=follower.node.pending_recovery_request
    assert pending is not None
    cancel(follower,'episode-A',goal_id='13:0')
    assert follower.node.pending_recovery_request is pending
    assert follower.node.pending_recovery_cancel is None
    follower.node.on_goal(goal_for(make_path(stamp=14),stamp=13))
    assert follower.node.recovery_request['context']['episode_id']=='episode-B'


def test_early_invalid_recovery_reports_the_requested_episode(follower):
    request(follower,distance=1.)
    assert follower.node.recovery_request is None
    assert packets(follower)[-1]['episode_id']=='episode-A'
    assert packets(follower)[-1]['status']=='RECOVERY_FAILED'


@pytest.mark.parametrize('bad',['',42,True,{},'x'*129])
def test_invalid_episode_identity_never_dispatches_reverse(follower,bad):
    request(follower,bad)
    assert follower.node.recovery_request is None
    follower.node.backup_client.send_goal_async.assert_not_called()
