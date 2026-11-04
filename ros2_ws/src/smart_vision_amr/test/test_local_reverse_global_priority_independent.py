"""Independent callback-order tests for local -> verified reverse -> global.

The reverse selector/body geometry has separate real-geometry tests. These
mock its final verdict to prove action ordering and pending-stop ownership.
No executor, velocity publisher, Gazebo service or motor is used.
"""
from copy import deepcopy
import json
from unittest.mock import Mock

import pytest
from std_msgs.msg import Int32MultiArray, String

from test_local_detour_planner import planner_factory, ready, hits
from test_planner_recovery import make_goal


def install_probes(node, *, stopped=True, reverse=True):
    events=[]
    node._try_local_detour=Mock(side_effect=lambda *args,**kwargs:events.append('local') or False)
    node._reverse_start_stopped=Mock(return_value=stopped)
    node._record_breadcrumb=Mock(return_value=True)
    def verdict(*args,**kwargs):
        events.append('reverse')
        if reverse:
            node.recovery_active=True
            node.failure_state='RECOVERING'
            return True
        node.last_reverse_rejection_code='REAR_CLEARANCE'
        return False
    node._maybe_request_reverse=Mock(side_effect=verdict)
    node.plan_to=Mock(side_effect=lambda *args,**kwargs:events.append('global'))
    return events


def visibility(node,cells):
    node.params['live_route_confirmation_scans']=2
    message=deepcopy(node.map[0]);message.header.stamp=node.get_clock().now().to_msg()
    message.data=[0]*(message.info.width*message.info.height)
    for x,y in cells:message.data[y*message.info.width+x]=100
    node.on_visibility(message)


def blocked_callback(node,mode):
    cells={(17,25)}  # On the active straight route, beyond the current body.
    if mode=='overlay':node.on_observed(hits(node,cells))
    elif mode=='visibility':visibility(node,cells)
    else:node.on_memory(hits(node,cells))


@pytest.mark.parametrize('mode',['overlay','visibility','memory'])
def test_active_obstacle_local_failure_checks_reverse_before_global(planner_factory,mode):
    node=planner_factory();events=install_probes(node)
    blocked_callback(node,mode)
    assert events==['local','reverse']
    assert not node.plan_to.called
    assert node.pending_reverse_probe is None and node.recovery_active
    # The unsafe old path was invalidated before any reverse request/probe.
    assert not node.planned_world_points
    assert node.path_pub.publish.call_args.args[0].poses==[]


@pytest.mark.parametrize('mode',['overlay','visibility','memory'])
def test_unsafe_rear_uses_global_only_after_reverse_verdict(planner_factory,mode):
    node=planner_factory();events=install_probes(node,reverse=False)
    blocked_callback(node,mode)
    assert events==['local','reverse','global']
    assert node.pending_reverse_probe is None
    assert node.plan_metrics['reverse_probe_rejection']=='REAR_CLEARANCE'


def test_not_stopped_remains_pending_through_new_observation_then_timer_retries(planner_factory):
    node=planner_factory();events=install_probes(node,stopped=False)
    blocked_callback(node,'overlay')
    assert events==['local']
    assert node.pending_reverse_probe is not None
    assert node.plan_metrics['recovery_stage']=='await_reverse_stop'
    assert not node._maybe_request_reverse.called and not node.plan_to.called
    # A new callback must not bypass the pending reverse with ordinary A*.
    ready(node,at=100.5)
    node.on_observed(hits(node,{(17,25)}))
    assert events==['local']
    assert node.pending_reverse_probe is not None
    node._reverse_start_stopped.return_value=True
    node.retry_transform()
    assert events==['local','reverse']
    assert not node.plan_to.called and node.recovery_active


@pytest.mark.parametrize('entry',['timer','request','plan'])
def test_all_replan_entrypoints_respect_pending_reverse_stop(planner_factory,entry):
    node=planner_factory();events=install_probes(node,stopped=False)
    blocked_callback(node,'overlay')
    ready(node,at=100.5);node.last_observation_time=100.5
    if entry=='timer':node.retry_transform()
    elif entry=='request':node._request_replan('input changed')
    else:
        # Exercise the production method, not the global-order observer mock.
        from smart_vision_amr.astar_planner import AStarPlannerNode
        AStarPlannerNode.plan_to(node,node.last_goal)
    assert events==['local']
    assert node.pending_reverse_probe is not None
    assert not node._maybe_request_reverse.called and not node.plan_to.called


@pytest.mark.parametrize('change',['manual_stop','goal','map','geometry','reference'])
def test_old_pending_reverse_cannot_be_borrowed_by_changed_context(planner_factory,change):
    node=planner_factory();events=install_probes(node,stopped=False)
    blocked_callback(node,'overlay')
    assert node.pending_reverse_probe is not None
    if change=='manual_stop':
        node.on_navigation_status(String(data=json.dumps({'manual_stop':True,
            'stop_stamp':{'sec':100,'nanosec':0}})))
    elif change=='goal':
        goal=deepcopy(node.last_goal);goal.header.stamp.sec=101
        node.on_goal(goal)
        # Explicit fresh-goal A* is allowed; the previous reverse is not.
        assert events==['local','global']
    elif change=='map':
        message=deepcopy(node.map[0]);message.info.map_load_time.sec+=1
        node.on_map(message)
    elif change=='geometry':node.params['robot_radius']+=.01
    else:node.local_reference_points=[(0.,0.),(1.,1.)]
    node._process_pending_reverse()
    assert node.pending_reverse_probe is None
    assert not node._maybe_request_reverse.called


def test_initial_explicit_goal_starts_with_astar_without_preemptive_reverse(planner_factory):
    node=planner_factory();events=install_probes(node)
    goal=deepcopy(node.last_goal);goal.header.stamp.sec=101
    node.on_goal(goal)
    assert events==['global']
    assert not node._try_local_detour.called and not node._maybe_request_reverse.called
    assert node.pending_reverse_probe is None


def test_successful_local_patch_needs_no_reverse_or_global(planner_factory):
    node=planner_factory();events=install_probes(node)
    node._try_local_detour.side_effect=lambda *args,**kwargs:events.append('local') or True
    before=list(node.planned_world_points)
    blocked_callback(node,'overlay')
    assert events==['local']
    assert not node._maybe_request_reverse.called and not node.plan_to.called
    assert node.planned_world_points==before


@pytest.mark.parametrize('change',[
    {'mode':'manual'}, {'mode':'localization'}, {'mode':'hold'},
    {'estop_latched':True}, {'reset_pending':True},
])
def test_manual_or_stop_authority_discards_old_probe_without_losing_goal(planner_factory,change):
    node=planner_factory();events=install_probes(node,stopped=False)
    blocked_callback(node,'overlay')
    goal=deepcopy(node.last_goal)
    assert node.pending_reverse_probe is not None
    status=dict(node.latest_motion_status,**change)
    node.on_motion_status(String(data=json.dumps(status)))
    assert node.pending_reverse_probe is None
    assert node.last_goal==goal
    assert not node._process_pending_reverse()
    assert events==['local']


def test_auto_sensor_fault_keeps_probe_waiting_without_forward_global(planner_factory):
    node=planner_factory();events=install_probes(node,stopped=False)
    blocked_callback(node,'overlay')
    status=dict(node.latest_motion_status,available=False,autonomous_allowed=False)
    node.on_motion_status(String(data=json.dumps(status)))
    assert node.pending_reverse_probe is not None
    assert node._process_pending_reverse()
    assert node.pending_reverse_probe is not None
    assert node.failure_state=='SENSOR_HOLD'
    assert events==['local']


def test_identical_map_republishes_preserve_pending_measured_stop(planner_factory):
    node=planner_factory();events=install_probes(node,stopped=False)
    blocked_callback(node,'overlay')
    pending=node.pending_reverse_probe
    raw_grid=node.raw_static_grid
    for at in (100.05,100.1,100.15):
        ready(node,at=at)
        # A real callback gets a new message/header stamp, while saved-map
        # content, geometry and load identity remain exactly unchanged.
        message=deepcopy(node.map[0])
        message.header.stamp=node.get_clock().now().to_msg()
        node.on_map(message)
        assert node.pending_reverse_probe is pending
        assert node.raw_static_grid is raw_grid
        assert not node._maybe_request_reverse.called and not node.plan_to.called
    node._reverse_start_stopped.return_value=True
    node.retry_transform()
    assert events==['local','reverse']
    assert node.pending_reverse_probe is None and node.recovery_active
