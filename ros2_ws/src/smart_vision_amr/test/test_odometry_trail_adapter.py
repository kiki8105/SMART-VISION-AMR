"""Source-verified odom projection and evidence-owned recovery callbacks."""
from copy import deepcopy
import json
from pathlib import Path
from unittest.mock import Mock
import pytest
from std_msgs.msg import String
from tf2_ros import TransformException

from smart_vision_amr.recovery_episode import RecoveryPhase
from test_local_detour_planner import planner_factory,hits
from test_recovery_episode_policy_independent import evidence
from test_planner_recovery import status
from test_occlusion_reserve_incident import load_incident


def moving_trace(node):
    for i in range(13):
        evidence(node,100.+i*.1,x=.525+.025*i,y=1.275)
        assert node._record_breadcrumb()
    assert len(node.breadcrumbs)>=10


def test_amcl_correction_reprojects_actual_trace_without_erasing_it(planner_factory):
    node=planner_factory();moving_trace(node)
    original=list(node.odometry_trail.points)
    before=list(node.breadcrumbs)
    tf=deepcopy(node.tf_buffer.lookup_transform.return_value)
    tf.transform.translation.x+=.08;tf.transform.translation.y-=.02
    assert node._record_breadcrumb(transform=tf)
    assert list(node.odometry_trail.points)==original
    assert len(node.breadcrumbs)==len(before)
    for a,b in zip(before,node.breadcrumbs):
        assert b.x-a.x==pytest.approx(.08) and b.y-a.y==pytest.approx(-.02)
    assert node.projected_odom_trail_verified


def test_temporary_tf_loss_revokes_projection_but_preserves_actual_trail(planner_factory):
    node=planner_factory();moving_trace(node)
    original=list(node.odometry_trail.points)
    node.tf_buffer.lookup_transform.side_effect=TransformException('temporary cache gap')
    assert not node._record_breadcrumb()
    assert not node.breadcrumbs and not node.projected_odom_trail_verified
    assert list(node.odometry_trail.points)==original
    node.tf_buffer.lookup_transform.side_effect=None
    assert node._record_breadcrumb()
    assert len(node.breadcrumbs)>=10


def test_slightly_future_latest_tf_is_paired_at_real_odom_source_stamp(planner_factory):
    node=planner_factory();moving_trace(node)
    paired=deepcopy(node.tf_buffer.lookup_transform.return_value)
    future=deepcopy(paired);future.header.stamp.nanosec+=50_000_000
    node.tf_buffer.lookup_transform.side_effect=lambda a,b,t: paired
    assert node._record_breadcrumb(transform=future)
    stamp=node.tf_buffer.lookup_transform.call_args.args[2].nanoseconds
    assert stamp==node.latest_odom[1]
    assert node.projected_odom_trail_verified and len(node.breadcrumbs)>=10


def test_unpairable_tf_never_uses_nearest_pose_to_fabricate_transform(planner_factory):
    node=planner_factory();moving_trace(node)
    future=deepcopy(node.tf_buffer.lookup_transform.return_value)
    future.header.stamp.nanosec+=50_000_000
    node.tf_buffer.lookup_transform.return_value=future
    assert not node._record_breadcrumb(transform=future)
    assert not node.projected_odom_trail_verified and not node.breadcrumbs
    assert len(node.odometry_trail.points)>=10


def test_sensor_restored_physical_blocker_enters_local_before_global(planner_factory):
    node=planner_factory();evidence(node,100.1)
    node.on_navigation_status(status(node,'WAIT_REPLAN',route_failure=False,input_hold=True))
    assert not node.planned_world_points and node.local_reference_points
    evidence(node,100.2)
    events=[]
    node._try_local_detour=Mock(side_effect=lambda *a,**k: events.append('local') or False)
    node.plan_to=Mock(side_effect=lambda *a,**k: events.append('global'))
    node.on_observed(hits(node,{(17,25)}))
    assert events==['local'] and node.pending_reverse_probe is not None
    assert node.recovery_coordinator.current.phase is RecoveryPhase.STOP_PROOF


def test_real_later_confirmed_memory_deletion_wakes_failed_start_not_original_cell(monkeypatch):
    fixture=Path(__file__).resolve().parents[4]/'benchmarks/driving_parameter_tuning_20261104/start_egress_audit/fixture.json'
    record=json.loads(fixture.read_text())
    node,_=load_incident()
    monkeypatch.setattr('smart_vision_amr.astar_planner.time.monotonic',lambda:node.clock['steady'])
    pose=record['pose'];evidence(node,100.,x=pose[0],y=pose[1],yaw=pose[2])
    node.last_goal.pose.position.x,node.last_goal.pose.position.y=(-.25,-1.98)
    node.local_reference_points=[tuple(p) for p in record['reference_path_record']['points']]
    node.local_reference_key=node._local_route_key()
    node.observed={tuple(c) for c in record['observed_cells']}
    node.remembered={tuple(c) for c in record['planner_remembered_cells']}
    coordinator=node.recovery_coordinator
    coordinator.begin('blocked-start',node._local_route_key(),{(39,88)},node.observed,node.remembered)
    coordinator.local_result(False);coordinator.stop_proved();coordinator.reverse_verdict(False)
    assert (39,88) not in node.remembered
    assert node._wait_after_recovery_cycle(relevant_cells={tuple(record['start_cell'])})
    assert {(39,89),(40,88),(40,89),(41,89)}<=coordinator.current.relevant_remembered
    old=coordinator.current
    # Deliver the projector's separately stamped event for this original scan.
    from test_planner_obstacle_hold import visibility
    node.on_visibility(visibility(node,occupied=node.observed))
    node.on_memory(hits(node,set()))
    assert not node.remembered and coordinator.current is not old
    assert coordinator.current.phase is RecoveryPhase.FOLLOW
    assert node.planned_world_points
    stamp=node.current_path_stamp_ns
    node.on_memory(hits(node,set()))
    assert node.current_path_stamp_ns==stamp  # Repeat receipt has no new evidence.
