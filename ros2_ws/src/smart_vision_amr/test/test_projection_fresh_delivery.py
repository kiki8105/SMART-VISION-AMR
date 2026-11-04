"""Real projection callbacks preserve acquisition time while shedding TF backlog."""

from copy import deepcopy
import json
import math
from unittest.mock import Mock, patch

from tf2_ros import TransformException

from test_live_obstacles import harness


def _stamp(scan):
    return scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec


def _exact_transform(node, *, unavailable=()):
    def lookup(_target, source, when):
        if source != 'odom' and when.nanoseconds in unavailable:
            raise TransformException('acquisition TF not yet delivered')
        value = deepcopy(node.map_to_odom if source == 'odom' else node.transform)
        value.header.stamp = when.to_msg()
        return value
    return lookup


def _queue_scans(node, count=3):
    node.params['scan_tf_wait_sec'] = .35
    node.visibility_pub = Mock()
    node.tf_buffer.lookup_transform.side_effect = TransformException('TF delayed')
    node.observation_pub.publish.reset_mock()
    scans = []
    for index in range(count):
        node.clock.return_value = 10. + index * .03
        scans.append(node.scan())
    assert not node.observation_pub.publish.called
    return scans


def test_tf_backlog_projects_newest_ready_original_frame_before_old_work():
    with harness() as node:
        scans = _queue_scans(node)
        node.clock.return_value = 10.12
        node.ros_ns = _stamp(scans[-1]) + 50_000_000
        node.tf_buffer.lookup_transform.side_effect = _exact_transform(node)
        project = node._project_scan
        entered = []

        def costly_projection(scan, *, received_at):
            entered.append((_stamp(scan), received_at))
            # A slower CPU makes FIFO work obsolete before it is published.
            node.clock.return_value += .08
            node.ros_ns += 80_000_000
            return project(scan, received_at=received_at)

        node._project_scan = costly_projection
        node.retry_pending_scans()

        assert entered == [(_stamp(scans[-1]), 10.06)]
        assert node.projection_valid == 1
        assert not node.pending_scans
        assert node.observed() == {8}  # The actual hit is still projected.
        visibility = node.visibility_pub.publish.call_args.args[0]
        assert visibility.header.stamp == scans[-1].header.stamp
        assert node.ros_ns - _stamp(scans[-1]) == 130_000_000
        diagnostic = json.loads(node.projection_status_pub.publish.call_args.args[0].data)
        assert diagnostic['scan_stamp_ns'] == _stamp(scans[-1])
        assert math.isclose(diagnostic['scan_age_ms'], 130.)
        assert math.isclose(diagnostic['first_receipt_to_publish_ms'], 140.)


def test_newest_unavailable_frame_keeps_first_receipt_while_ready_predecessor_runs():
    with harness() as node:
        scans = _queue_scans(node)
        newest = _stamp(scans[-1])
        node.clock.return_value = 10.12
        node.tf_buffer.lookup_transform.side_effect = _exact_transform(node, unavailable={newest})
        node.retry_pending_scans()

        assert node.projection_valid == 1
        assert node.last_scan_stamp_ns == _stamp(scans[-2])
        assert [(stamp, arrival) for stamp, arrival, _ in node.pending_scans] == [(newest, 10.06)]
        assert node.pending_scans[0][2] is scans[-1]
        node.clock.return_value = 10.17
        node.tf_buffer.lookup_transform.side_effect = _exact_transform(node)
        node.retry_pending_scans()
        assert node.last_scan_stamp_ns == newest
        assert node.projection_valid == 2
        assert not node.pending_scans
        assert node.visibility_pub.publish.call_args.args[0].header.stamp == scans[-1].header.stamp


def test_retry_does_not_extend_original_tf_wait_or_publish_empty_safe_heartbeat():
    with harness() as node:
        scans = _queue_scans(node, count=1)
        node.clock.return_value = 10.2
        node.retry_pending_scans()
        assert node.pending_scans[0][1] == 10.
        node.clock.return_value = 10.351
        node.retry_pending_scans()
        assert not node.pending_scans
        assert not node.observation_pub.publish.called
        assert not node.visibility_pub.publish.called
        diagnostic = json.loads(node.projection_status_pub.publish.call_args.args[0].data)
        assert diagnostic['reason'] == 'measurement_tf_timeout'
        assert diagnostic['scan_stamp_ns'] == _stamp(scans[0])


def test_visibility_proof_is_published_before_same_frame_obstacle_trigger():
    with harness() as node:
        events = []
        node.visibility_pub = Mock()
        node.visibility_pub.publish.side_effect = lambda msg: events.append(('visibility', msg))
        node.observation_pub.publish.side_effect = lambda msg: events.append(('observed', msg))
        scan = node.scan()

        assert [kind for kind, _ in events] == ['visibility', 'observed']
        visibility, observed = (event[1] for event in events)
        assert visibility.header.stamp == scan.header.stamp
        assert visibility.data[8] == 100
        assert set(observed.data) == {8}
        assert scan.header.stamp.sec == 100  # No producer restamp.


def test_projection_cache_preserves_unknown_occupied_and_threshold_replacement():
    with harness() as node:
        replacement = deepcopy(node.map['message'])
        replacement.data[0], replacement.data[1], replacement.data[2] = -1, 100, 60
        node.on_map(replacement)
        cells = node._known_free_base_cells()
        assert (0, 0) not in cells and (1, 0) not in cells and (2, 0) in cells
        assert node._known_free_base_cells() is cells
        node.params['occupied_threshold'] = 50
        assert (2, 0) not in node._known_free_base_cells()


def test_real_projection_still_requires_five_observations_and_two_seconds_to_confirm_clear():
    with harness() as node:
        node.params.update(memory_enabled=True, save_every_scans=0)
        node.scans_since_save = 0
        node.visibility_pub = Mock()
        for index in range(5):
            node.clock.return_value = 10. + index * .5
            with patch('smart_vision_amr.change_memory.time.time', return_value=1000. + index * .5):
                node.scan(advance=.5)
            assert ((2, 1) in node.memory.confirmed_cells) is (index == 4)
        for index in range(5):
            node.clock.return_value = 12.5 + index * .5
            with patch('smart_vision_amr.change_memory.time.time', return_value=1002.5 + index * .5):
                node.scan(math.inf, advance=.5)
            assert ((2, 1) not in node.memory.confirmed_cells) is (index == 4)


def test_pending_diagnostics_do_not_starve_periodic_valid_projection_evidence():
    with harness() as node:
        node.visibility_pub = Mock()
        for index in range(13):
            node.clock.return_value = 10. + index * .1
            scan = node.scan()
            node._projection_diagnostic('PENDING_TF', 'waiting_for_measurement_transform', scan)
        statuses = [json.loads(call.args[0].data)
                    for call in node.projection_status_pub.publish.call_args_list]
        valid = [item for item in statuses if item['status'] == 'VALID']
        assert len(valid) == 2
        assert valid[-1]['valid_scans'] >= 11
        assert valid[-1]['scan_age_ms'] == 0.


def test_localization_jump_does_not_restore_retained_newer_pending_scan():
    with harness() as node:
        node.scan()  # Establish the old map-to-odom solution.
        scans = _queue_scans(node)
        node.clock.return_value = 10.12
        node.map_to_odom.transform.translation.x += .5
        node.tf_buffer.lookup_transform.side_effect = _exact_transform(
            node, unavailable={_stamp(scans[-1])})
        old_memory = node.memory
        node.retry_pending_scans()
        assert node.memory is not old_memory
        assert not node.pending_scans
