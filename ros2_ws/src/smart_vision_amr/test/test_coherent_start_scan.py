"""Original scan/visibility event proofs; no ROS nodes or action servers."""
from copy import deepcopy
from unittest.mock import patch

import pytest
from builtin_interfaces.msg import Time
from tf2_ros import TransformException

from test_dynamic_escape_certificate_ownership import incident
from test_local_detour_planner import planner_factory


def stamp_ns(scan):
    return scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec


def receive(node, *, ros=100.1, steady=100.1, frame=None):
    scan = deepcopy(node.latest_scan)
    node.clock.update(ros=ros, steady=steady)
    whole = int(ros)
    scan.header.stamp = Time(sec=whole, nanosec=round((ros-whole)*1e9))
    if frame is not None:
        scan.header.frame_id = frame
    node.on_scan(scan)
    return scan


def project(node, scan=None, *, received=None):
    scan = node.latest_scan if scan is None else scan
    stamp = stamp_ns(scan)
    node.last_visibility_stamp_ns = stamp
    node.last_visibility_at = node.clock['steady'] if received is None else received
    node._record_start_visibility(stamp, node.last_visibility_at)


def visibility(node, cells):
    message = deepcopy(node.map[0])
    message.header.stamp = node.get_clock().now().to_msg()
    message.data = [0]*(message.info.width*message.info.height)
    for x, y in cells:
        message.data[y*message.info.width+x] = 100
    node.on_visibility(message)


def own(node):
    node.plan_to(node.last_goal)
    assert node.active_start_certificate
    node.path_pub.publish.reset_mock()


def test_actual_future_newest_scan_does_not_replace_exact_projected_event(incident):
    node, _ = incident
    own(node)
    original = node.latest_scan
    newer = receive(node)
    transform = node.tf_buffer.lookup_transform.return_value
    requested = []
    def lookup(target, frame, at):
        requested.append(at.nanoseconds)
        if at.nanoseconds > 100_000_000_000:
            raise TransformException('requested100.1 latest100.062: future extrapolation')
        return transform
    with patch.object(node.tf_buffer, 'lookup_transform', side_effect=lookup):
        assert not node._live_route_blockers(node.observed)
    detail = node.plan_metrics['start_scan_selection']
    assert detail['selected_stamp_ns'] == stamp_ns(original)
    assert detail['newest_raw_scan_stamp_ns'] == stamp_ns(newer)
    assert node.active_start_certificate and not node.path_pub.publish.called
    assert 100_100_000_000 not in requested


@pytest.mark.parametrize('owned', [False, True])
def test_new_and_owned_edges_use_same_freshest_transformable_pair(incident, owned):
    node, _ = incident
    if owned:
        own(node)
    newer = receive(node)
    project(node, newer)
    transform = node.tf_buffer.lookup_transform.return_value
    def lookup(target, frame, at):
        if at.nanoseconds > 100_000_000_000:
            raise TransformException('future acquisition')
        return transform
    with patch.object(node.tf_buffer, 'lookup_transform', side_effect=lookup):
        assert node._coherent_start_scan()[0].header.stamp.sec == 100
        if owned:
            assert not node._live_route_blockers(node.observed)
    detail = node.plan_metrics['start_scan_selection']
    assert detail['selected_stamp_ns'] == 100_000_000_000
    assert detail['source_age_s'] == pytest.approx(.1)
    assert detail['raw_first_receipt_age_s'] == pytest.approx(.1)


def test_new_body_hit_is_not_hidden_by_prior_free_scan_or_future_tf(incident):
    node, _ = incident
    own(node)
    receive(node)
    node.last_visibility_stamp_ns = 100_100_000_000
    node.tf_buffer.lookup_transform.return_value.header.stamp.sec = 100
    current = set(node.observed) | {(31,96)}
    assert node._live_route_blockers(current)
    assert not node.active_start_certificate
    assert not node.start_prefix_input_pending
    assert node.plan_metrics['start_certificate_reason'] == 'selected_edge_no_longer_certified'


def test_missing_event_stops_as_sensor_pending_without_physical_episode(incident):
    node, _ = incident
    own(node)
    node._clear_start_scan_history()
    assert not node.active_start_certificate
    node.clock.update(ros=100.1, steady=100.1)
    visibility(node, node.observed | {(36,94)})
    assert node.failure_state == 'SENSOR_HOLD' and node.waiting_for_replan
    assert not node.planned_world_points
    assert node.recovery_coordinator.current is None
    assert node.pending_reverse_probe is None
    assert node.plan_metrics['recovery_stage'] == 'await_start_prefix_inputs'
    count = node.global_plan_count
    node.retry_transform()
    assert node.global_plan_count == count


def test_pending_reacquires_exact_event_and_retains_goal_without_operator(incident):
    node, _ = incident
    own(node)
    goal = node.last_goal
    node._clear_start_scan_history()
    node.clock.update(ros=100.1, steady=100.1)
    visibility(node, node.observed | {(36,94)})
    receive(node, ros=100.1, steady=100.1)
    project(node)
    node.last_observation_time = 100.1
    node.last_plan_time = float('-inf')
    node.retry_transform()
    assert node.last_goal is goal and node.planned_world_points
    assert not node.waiting_for_replan and not node.start_scan_context_invalid
    assert node.recovery_coordinator.current is None


def test_duplicate_source_does_not_renew_either_receipt_or_cache_lease(incident):
    node, _ = incident
    original = node.latest_scan
    first = node.latest_scan_at
    node.clock.update(steady=100.2, ros=100.2)
    node.on_scan(deepcopy(original))
    assert node.latest_scan_at == first
    assert node.start_scan_history[-1][1] == first
    node.clock.update(steady=100.251, ros=100.251)
    assert node._coherent_start_scan() is None


@pytest.mark.parametrize('clock', ['source','receipt','visibility'])
def test_both_original_age_bounds_are_required(incident, clock):
    node, _ = incident
    if clock == 'visibility':
        node.start_visibility_history[-1] = (100_000_000_000, 99.7)
    node.clock.update(ros=100.251 if clock=='source' else 100.1,
                      steady=100.251 if clock=='receipt' else 100.1)
    assert node._coherent_start_scan() is None


def test_backward_source_clears_cache_and_old_owner_without_receipt_refresh(incident):
    node, _ = incident
    own(node)
    first = node.latest_scan_at
    older = deepcopy(node.latest_scan)
    older.header.stamp = Time(sec=99, nanosec=990_000_000)
    node.clock.update(ros=100.1, steady=100.1)
    node.on_scan(older)
    assert not node.start_scan_history and node.latest_scan_at == first
    assert not node.active_start_certificate and node.start_scan_context_invalid


def test_monotonic_clock_rewind_cannot_reuse_old_scan(incident):
    node, _ = incident
    node.clock['steady'] = 99.99
    assert node._coherent_start_scan() is None


def test_frame_change_cannot_borrow_prior_raw_frame_or_owner(incident):
    node, _ = incident
    own(node)
    receive(node, frame='other_laser')
    assert not node.active_start_certificate and node.start_scan_context_invalid
    assert all(scan.header.frame_id=='other_laser' for _,_,scan in node.start_scan_history)
    assert node._coherent_start_scan() is None


def test_real_odom_epoch_reset_clears_scan_cache_and_owner(incident):
    node, _ = incident
    own(node)
    from nav_msgs.msg import Odometry
    odom = Odometry()
    odom.header.frame_id='odom';odom.child_frame_id=node.params['base_frame']
    odom.header.stamp=Time(sec=100,nanosec=100_000_000)
    odom.pose.pose.position.x=10.;odom.pose.pose.orientation.w=1.
    node.clock.update(ros=100.1,steady=100.1)
    node.on_odom(odom)
    assert not node.start_scan_history and not node.active_start_certificate
    assert node.start_scan_context_invalid


def test_map_scope_reset_clears_scan_cache(incident):
    node, _ = incident
    own(node)
    message=deepcopy(node.map[0]);message.info.map_load_time.sec += 1
    node.on_map(message)
    assert not node.start_scan_history and not node.active_start_certificate


def test_both_caches_are_bounded_and_no_unpaired_event_matches(incident):
    node, _ = incident
    for index in range(1,12):
        scan = receive(node, ros=100.+index*.01, steady=100.+index*.01)
        project(node, scan)
    assert len(node.start_scan_history) == len(node.start_visibility_history) == 8
    node.start_visibility_history.clear()
    node.last_visibility_stamp_ns=100_030_000_000
    assert node._coherent_start_scan() is None


def test_invalid_exact_geometry_is_not_replaced_by_prior_free_scan(incident):
    node, _ = incident
    own(node)
    receive(node)
    node.latest_scan.angle_increment=0.
    project(node)
    assert node._coherent_start_scan() is None
    assert node.plan_metrics['start_scan_selection']['reason']=='paired_scan_geometry_invalid'


def test_low_coverage_pair_cannot_borrow_new_healthy_unpaired_scan(incident):
    node, _ = incident
    node.latest_scan.ranges = [float('nan')]*63+[float('inf')]
    healthy = receive(node)
    healthy.ranges = [float('inf')]*64
    assert node._coherent_start_scan() is None
    assert node.plan_metrics['start_scan_selection']['reason'] == 'paired_scan_coverage_invalid'


def test_context_reset_does_not_rearm_queued_original_events(incident):
    node, _ = incident
    own(node)
    old = receive(node, ros=100.1, steady=100.)
    project(node, old)
    node._clear_start_scan_history()
    node.on_scan(deepcopy(old))
    node._record_start_visibility(stamp_ns(old), 100.01)
    assert not node.start_scan_history and not node.start_visibility_history
    assert node._coherent_start_scan() is None
    assert node.start_scan_context_invalid


def test_clock_rewind_drops_owner_and_old_queued_pair(incident):
    node, _ = incident
    own(node)
    original = deepcopy(node.latest_scan)
    node.clock.update(ros=99.9, steady=100.1)
    assert node._coherent_start_scan() is None
    node.on_scan(original)
    node._record_start_visibility(stamp_ns(original),100.1)
    assert not node.start_scan_history and not node.start_visibility_history
    assert not node.active_start_certificate and node.start_scan_context_invalid


def test_new_certificate_requires_paired_original_raw_before_publish(incident):
    node, _ = incident
    node.start_visibility_history.clear()
    node.plan_to(node.last_goal)
    assert not node.active_start_certificate and not node.planned_world_points
    assert node.failure_state == 'SENSOR_HOLD'
    assert node.plan_metrics['start_prefix_validation']['input_pending']
    assert all(not call.args[0].poses for call in node.path_pub.publish.call_args_list)


def test_repeated_projection_cannot_renew_its_first_receipt(incident):
    node, _ = incident
    first = node.start_visibility_history[-1]
    node.clock.update(ros=100.2, steady=100.2)
    node._record_start_visibility(first[0],100.2)
    assert node.start_visibility_history[-1] == first


def test_real_visibility_hit_stays_hard_when_new_pair_tf_is_future(incident):
    node, _ = incident
    own(node)
    receive(node)
    tf = node.tf_buffer.lookup_transform.return_value
    def lookup(target, frame, at):
        if at.nanoseconds > 100_000_000_000:
            raise TransformException('future acquisition')
        return tf
    with patch.object(node.tf_buffer, 'lookup_transform', side_effect=lookup):
        current = node.observed | {(31,96)}
        visibility(node,current)
    assert not node.active_start_certificate
    assert node.plan_metrics['start_certificate_reason'] != 'coherent_raw_scan_input_pending'
    assert not node.planned_world_points


def test_clear_suffix_pending_preserves_same_local_episode_and_resumes_it(planner_factory):
    from test_recovery_episode_policy_independent import stopped, probes, obstacle
    from smart_vision_amr.recovery_episode import RecoveryPhase
    node = planner_factory(); stopped(node)
    events = probes(node,no_global_route=True)
    obstacle(node)
    assert node.recovery_coordinator.current.phase is RecoveryPhase.WAIT_OBSTACLE
    node.observed.clear()
    assert node._resume_recovery_wait(positive_free=True)
    episode = node.recovery_coordinator.current
    assert episode.phase is RecoveryPhase.LOCAL
    assert node.failure_state == 'SENSOR_HOLD'
    assert events == ['local','reverse','global']
    node.retry_transform()
    assert node.recovery_coordinator.current is episode
    assert episode.phase is RecoveryPhase.LOCAL
    assert events == ['local','reverse','global']
    project(node)
    node._request_replan('coherent snapshot recovered')
    assert node.recovery_coordinator.current is episode
    assert episode.phase is RecoveryPhase.FOLLOW
    assert node.planned_world_points
    assert events == ['local','reverse','global']


@pytest.mark.parametrize('owned',[False,True])
def test_actual_tf2_future_extrapolation_selects_ready_paired_original(incident,owned):
    from tf2_ros import Buffer
    node, _ = incident
    if owned:
        own(node)
    template = deepcopy(node.tf_buffer.lookup_transform.return_value)
    buffer = Buffer()
    for nanosec in (0, 62_000_000):
        tf = deepcopy(template)
        tf.header.frame_id = node.params['map_frame']
        tf.child_frame_id = node.params['base_frame']
        tf.header.stamp = Time(sec=100,nanosec=nanosec)
        buffer.set_transform(tf,'coherent-unit')
    node.tf_buffer = buffer
    new = receive(node)
    project(node,new)
    with pytest.raises(TransformException):
        buffer.lookup_transform(node.params['map_frame'],new.header.frame_id,
            __import__('rclpy').time.Time.from_msg(new.header.stamp))
    chosen = node._coherent_start_scan()
    assert chosen and stamp_ns(chosen[0]) == 100_000_000_000
    if owned:
        assert not node._live_route_blockers(node.observed)
        assert node.active_start_certificate
    else:
        node.plan_to(node.last_goal)
        assert node.active_start_certificate and node.planned_world_points
    assert node.plan_metrics['start_scan_selection']['selected_stamp_ns'] == 100_000_000_000


def test_120ms_tf_lag_over_many_10hz_events_does_not_starve_owned_edge(incident):
    from test_local_detour_planner import ready
    node, record = incident
    own(node)
    receive(node); project(node)
    for index in range(2,9):
        at = 100.+index*.1
        ready(node,at=at,x=record['pose_world'][0],y=record['pose_world'][1],
              yaw=record['pose_world'][2],scan_frame=node.params['base_frame'])
        project(node)
        transform = node.tf_buffer.lookup_transform.return_value
        cutoff = round((at-.12)*1e9)
        transform.header.stamp = Time(sec=cutoff//1_000_000_000,
                                      nanosec=cutoff%1_000_000_000)
        def lookup(target,frame,when):
            if when.nanoseconds > cutoff:
                raise TransformException('actual-style120ms acquisition lag')
            return transform
        with patch.object(node.tf_buffer,'lookup_transform',side_effect=lookup):
            assert not node._live_route_blockers(node.observed)
        selection = node.plan_metrics['start_scan_selection']
        assert selection['source_age_s'] == pytest.approx(.2)
        assert selection['raw_first_receipt_age_s'] == pytest.approx(.2)
        assert selection['visibility_first_receipt_age_s'] == pytest.approx(.2)
        assert node.active_start_certificate and not node.path_pub.publish.called
        assert len(node.start_scan_history) <= 8 and len(node.start_visibility_history) <= 8
