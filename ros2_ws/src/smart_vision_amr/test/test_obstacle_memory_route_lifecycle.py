"""A later goal must use confirmed LiDAR memory until a ray proves removal."""

import math
import json
from unittest.mock import patch

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray

from test_live_obstacles import harness
from test_planner_recovery import PlannerHarness, make_goal


def enable_memory(sensor, tmp_path):
    sensor.params['memory_enabled'] = True
    sensor.params['memory_persist_to_disk'] = False
    sensor.params['save_every_scans'] = 100
    sensor.memory_path = tmp_path / 'environment_memory.json'
    sensor.scans_since_save = 0


def memory_cells_message(sensor):
    return Int32MultiArray(data=[y * sensor.memory.width + x
                                 for x, y in sensor.memory.confirmed_cells])


def test_obstacle_seen_on_first_leg_diverts_later_goal_until_fresh_free_rays(tmp_path):
    with harness() as sensor:
        enable_memory(sensor, tmp_path)
        original_map = list(sensor.map['message'].data)
        # A stable unmapped object is observed for the configured five
        # measurements and two seconds on the first leg of a mission.
        with patch('smart_vision_amr.change_memory.time.time',
                   side_effect=lambda: sensor.ros_ns / 1e9):
            for _ in range(5):
                sensor.scan(2., advance=.5)
            blocked_cell = (2, 1)
            blocked_index = 1 * sensor.memory.width + 2
            assert sensor.memory.confirmed_cells == {blocked_cell}
            assert blocked_index in sensor.overlay_pub.publish.call_args.args[0].data

            # Later the robot looks elsewhere. The transient scan layer may
            # expire, but lack of visibility is not evidence of free space.
            sensor.clock.return_value = 15.
            sensor.scan(1., angle=math.pi / 2, advance=.5)
            assert blocked_index not in sensor.observed()
            assert sensor.memory.confirmed_cells == {blocked_cell}
            assert not sensor.memory_path.exists()

            # A nearer return along the original bearing occludes the old
            # location; it does not prove that the hidden cell is free.
            sensor.scan(1., advance=.5)
            assert sensor.memory.confirmed_cells == {blocked_cell}

            planner = PlannerHarness()
            planner.on_map(sensor.map['message'])
            planner.on_observed(Int32MultiArray(data=[]))
            planner.on_memory(memory_cells_message(sensor))
            goal = make_goal()
            with patch('smart_vision_amr.astar_planner.time.monotonic', return_value=100.):
                planner.on_goal(goal)
                remembered_route = list(planner.planned_cells)
                assert remembered_route
                assert blocked_cell not in remembered_route
                assert remembered_route != [(x, 1) for x in range(5)]

                # One fresh free ray removes the transient hit, but the
                # confirmed memory requires repeated positive evidence.
                sensor.scan(math.inf, advance=.5)
                assert sensor.memory.confirmed_cells == {blocked_cell}
                for _ in range(4):
                    sensor.scan(math.inf, advance=.5)
                assert sensor.memory.confirmed_cells == set()
                assert list(sensor.overlay_pub.publish.call_args.args[0].data) == []

                planner.on_memory(memory_cells_message(sensor))
                planner.plan_to(goal)
                assert planner.planned_cells == [(x, 1) for x in range(5)]
                assert list(sensor.map['message'].data) == original_map
                assert not sensor.memory_path.exists()


def test_brief_false_hit_and_unseen_cell_never_become_confirmed_memory(tmp_path):
    with harness() as sensor:
        enable_memory(sensor, tmp_path)
        with patch('smart_vision_amr.change_memory.time.time',
                   side_effect=lambda: sensor.ros_ns / 1e9):
            # Several rapid returns can be real live obstacles for immediate
            # safety, yet must not become persistent before the duration gate.
            for _ in range(5):
                sensor.scan(2., advance=.1)
            assert sensor.memory.confirmed_cells == set()
            sensor.clock.return_value = 15.
            sensor.scan(1., angle=math.pi / 2, advance=3.)
            assert sensor.memory.confirmed_cells == set()
            assert list(sensor.overlay_pub.publish.call_args.args[0].data) == []


def seeded_memory(sensor):
    for observed_at in (0., .5, 1., 1.5, 2.):
        sensor.memory.observe_occupied((2, 1), observed_at)
    sensor.memory.observe_occupied((3, 1), 2.)  # Unconfirmed evidence also expires.
    assert sensor.memory.confirmed_cells == {(2, 1)}
    sensor.publish_overlay()
    return sensor.memory


def initial_pose(*, frame='map', x=.5, y=1.5, yaw=0.):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.)
    message.pose.pose.orientation.w = math.cos(yaw / 2.)
    return message


def test_valid_initialpose_discards_confirmed_and_candidate_memory_without_fake_scan(tmp_path):
    with harness() as sensor:
        enable_memory(sensor, tmp_path)
        sensor.scan(2.)
        sensor.memory = sensor._new_memory(sensor.map['message'])
        old_memory = seeded_memory(sensor)
        original_map = list(sensor.map['message'].data)
        observed_count = sensor.observation_pub.publish.call_count
        sensor.on_initial_pose(initial_pose())
        assert sensor.memory is not old_memory
        assert not sensor.memory.cells
        assert sensor.live_obstacles == {}
        assert list(sensor.overlay_pub.publish.call_args.args[0].data) == []
        assert sensor.observation_pub.publish.call_count == observed_count
        assert list(sensor.map['message'].data) == original_map
        assert not sensor.memory_path.exists()


def test_invalid_initialpose_preserves_memory_and_overlay(tmp_path):
    with harness() as sensor:
        enable_memory(sensor, tmp_path)
        old_memory = seeded_memory(sensor)
        count = sensor.overlay_pub.publish.call_count
        messages = [initial_pose(frame='odom'), initial_pose(x=math.nan),
                    initial_pose(x=-10.), initial_pose(x=math.inf)]
        invalid_quaternion = initial_pose()
        invalid_quaternion.pose.pose.orientation.w = 0.
        messages.append(invalid_quaternion)
        for message in messages:
            sensor.on_initial_pose(message)
            assert sensor.memory is old_memory
        assert sensor.overlay_pub.publish.call_count == count
        assert sensor.memory.confirmed_cells == {(2, 1)}


def test_initialpose_overwrites_disk_memory_to_prevent_stale_restore(tmp_path):
    with harness() as sensor:
        enable_memory(sensor, tmp_path)
        sensor.params['memory_persist_to_disk'] = True
        seeded_memory(sensor)
        sensor.persist()
        assert len(json.loads(sensor.memory_path.read_text())['cells']) == 2
        sensor.on_initial_pose(initial_pose())
        payload = json.loads(sensor.memory_path.read_text())
        assert payload['cells'] == []
        assert payload['map_signature'] == sensor.map['signature']


def test_only_fresh_significant_map_to_odom_correction_discards_memory(tmp_path):
    with harness() as sensor:
        enable_memory(sensor, tmp_path)
        scan = sensor.scan(1., angle=math.pi / 2)
        old_memory = seeded_memory(sensor)
        measurement_time = Time.from_msg(scan.header.stamp)

        sensor.map_to_odom.transform.translation.x = .24
        sensor.reset_live_on_localization_jump('map', measurement_time)
        assert sensor.memory is old_memory

        sensor.map_to_odom.transform.translation.x = .5
        sensor.map_to_odom.header.stamp.sec -= 10
        sensor.reset_live_on_localization_jump('map', measurement_time)
        assert sensor.memory is old_memory  # Stale TF cannot invalidate memory.

        sensor.map_to_odom.header.stamp = scan.header.stamp
        sensor.reset_live_on_localization_jump('map', measurement_time)
        assert sensor.memory is not old_memory
        assert not sensor.memory.cells
        assert list(sensor.overlay_pub.publish.call_args.args[0].data) == []
