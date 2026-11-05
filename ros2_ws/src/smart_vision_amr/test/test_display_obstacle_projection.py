"""UI range uses real projected hits without changing planning or memory."""
from copy import deepcopy
from dataclasses import asdict
import math
from unittest.mock import Mock, patch

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformException

from test_live_obstacles import harness
from test_near_scan_outlier_guard import one_ghost
from test_wall_range_classification import wall_scan_harness


def enable(node):
    node.params['live_obstacle_max_range_m'] = 1.
    node.params['display_obstacle_max_range_m'] = 3.5
    node.display_observation_pub = Mock()
    node.visibility_pub = Mock()
    node.scans_since_save = 0
    node.params['save_every_scans'] = 0


def displayed(node):
    return set(node.display_observation_pub.publish.call_args.args[0].data)


def test_display_disabled_by_default_has_no_new_measurement_publication():
    with harness() as node:
        node.display_observation_pub = Mock()
        node.params['live_obstacle_max_range_m'] = 1.
        node.scan(2.)
        assert not node.observed()
        node.display_observation_pub.publish.assert_not_called()


def test_far_display_point_never_becomes_planner_visibility_hit_or_memory():
    with harness() as node:
        enable(node)
        node.params['memory_enabled'] = True
        original_map = list(node.map['message'].data)
        node.scan(2.)
        assert displayed(node) == {8}
        assert node.observed() == set()
        assert node.visibility_pub.publish.call_args.args[0].data[8] == -1
        assert not node.memory.cells
        assert list(node.map['message'].data) == original_map
        node.scan(.5)
        assert node.observed() == {7}
        assert 7 in displayed(node)
        assert (1, 1) in node.memory.cells  # Existing near memory still works.


@pytest.mark.parametrize('distance,expected', [(1., {7}), (1.03, {7}), (3.5, {10}), (3.6, set())])
def test_display_range_and_existing_one_meter_planning_cap_are_independent(distance, expected):
    with harness() as node:
        enable(node)
        node.scan(distance)
        assert displayed(node) == expected
        assert bool(node.observed()) == (distance <= 1.)


def test_far_return_does_not_bypass_mapped_wall_filter():
    with harness() as node:
        enable(node)
        wall = deepcopy(node.map['message'])
        wall.data[1*6+4] = 100
        node.on_map(wall)
        node.scan(3.5)  # Exactly the first mapped wall at x=4.
        assert not displayed(node)
        assert not node.observed()
        node.scan(2.)  # Real object in front of that wall stays visible.
        assert displayed(node) == {8}
        assert not node.observed()


def test_display_clears_on_measured_free_ray_and_preserves_brief_occlusion_ttl():
    with harness() as node:
        enable(node)
        node.scan(2.)
        assert displayed(node) == {8}
        node.clock.return_value = 10.2
        node.scan(1., math.pi/2)
        assert 8 in displayed(node)  # A different direction is not free proof.
        node.clock.return_value = 12.1
        node.scan(1., math.pi/2)
        assert 8 not in displayed(node)
        node.scan(2.)
        assert 8 in displayed(node)
        node.scan(math.inf)
        assert 8 not in displayed(node)  # A traversed-free ray clears that cell immediately.


def test_invalid_or_replayed_measurements_never_renew_display_evidence():
    with harness() as node:
        enable(node)
        packet = node.scan(2.)
        count = node.display_observation_pub.publish.call_count
        node.on_scan(packet)
        for invalid in (math.nan, -math.inf, 0., 5.):
            node.scan(invalid)
        assert node.display_observation_pub.publish.call_count == count
        assert displayed(node) == {8}


def test_tf_missing_waits_for_exact_scan_stamp_before_displaying_any_cells():
    with harness() as node:
        enable(node)
        node.tf_buffer.lookup_transform.side_effect = TransformException('future')
        packet = node.scan(2.)
        node.display_observation_pub.publish.assert_not_called()
        node.observation_pub.publish.assert_not_called()
        node.clock.return_value = 10.05
        node.tf_buffer.lookup_transform.side_effect = lambda target, source, when: (
            node.map_to_odom if source == 'odom' else node.transform)
        node.retry_pending_scans()
        assert displayed(node) == {8}
        stamp = packet.header.stamp.sec*1_000_000_000+packet.header.stamp.nanosec
        assert all(call.args[2].nanoseconds == stamp for call in node.tf_buffer.lookup_transform.call_args_list)
        assert node.observed() == set()


@pytest.mark.parametrize('reset', ['map', 'initial_pose', 'localization_jump'])
def test_context_reset_discards_display_cache_without_fabricating_measurement(reset):
    with harness() as node:
        enable(node)
        node.scan(2.)
        assert node.display_obstacles
        count = node.display_observation_pub.publish.call_count
        if reset == 'map':
            message = deepcopy(node.map['message'])
            message.info.origin.position.x += 20.
            node.on_map(message)
        elif reset == 'initial_pose':
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            message.pose.pose.orientation.w = 1.
            node.on_initial_pose(message)
        else:
            node.map_to_odom.transform.translation.x += .5
            node.scan(1., math.pi/2)
            assert (2, 1) not in node.display_obstacles
            return
        assert not node.display_obstacles
        assert node.display_observation_pub.publish.call_count == count


def test_disabled_display_does_not_leave_old_points_ready_for_reenable():
    with harness() as node:
        enable(node)
        node.scan(2.)
        node.params['display_obstacle_max_range_m'] = 0.
        count = node.display_observation_pub.publish.call_count
        node.scan(1., math.pi/2)
        assert not node.display_obstacles
        assert node.display_observation_pub.publish.call_count == count


def test_wider_ui_beam_group_cannot_change_near_wall_planning_or_persistent_evidence():
    # Run the identical actual projection twice, including a map wall and
    # long-range mapped returns. Compare every planning publication and memory
    # field; a UI range must not extend a planner wall-aligned beam group.
    outputs = []
    for enabled in (False, True):
        with wall_scan_harness() as node:
            node.params['memory_enabled'] = True
            node.params['live_obstacle_max_range_m'] = 1.
            node.params['display_obstacle_max_range_m'] = 3.5 if enabled else 0.
            node.display_observation_pub = Mock()
            with patch('smart_vision_amr.change_memory.time.time', return_value=1000.):
                # A full real scan contains both near and distant rays.
                packet = node.scan(.2)
                node.ros_ns += 100_000_000
                packet.header.stamp.sec, packet.header.stamp.nanosec = divmod(node.ros_ns, 1_000_000_000)
                node.transform.header.stamp = deepcopy(packet.header.stamp)
                packet.angle_min, packet.angle_increment = -math.pi, math.pi/180
                packet.ranges = [.2 if i % 37 == 0 else 1.8 for i in range(360)]
                node.on_scan(packet)
            outputs.append((
                [list(c.args[0].data) for c in node.observation_pub.publish.call_args_list],
                [list(c.args[0].data) for c in node.visibility_pub.publish.call_args_list],
                {key: asdict(value) for key, value in node.memory.cells.items()},
                list(node.map['message'].data)))
    assert outputs[0] == outputs[1]
