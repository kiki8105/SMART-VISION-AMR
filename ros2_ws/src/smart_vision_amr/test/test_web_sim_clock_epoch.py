"""A long-lived read-only Gazebo viewer must discard an earlier /clock epoch."""

import json
import os
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path as ROSPath
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from smart_vision_amr.web_mission_server import DEFAULT_MARKERS, MissionApplication, create_ros_bridge


def _map(app):
    app.receive_map({'width': 4, 'height': 4, 'resolution': .5,
                     'origin': {'x': -1., 'y': -1., 'yaw': 0.}, 'frame_id': 'map'},
                    np.zeros((4, 4), dtype=np.int16))


def _epoch(node, app, seconds, x, y):
    node.get_clock().set_ros_time_override(Time(seconds=seconds))
    _map(app)
    app.scan_at = app.odom_at = time.monotonic()
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.header.stamp = Time(seconds=seconds).to_msg()
    transform.child_frame_id = 'sim_base'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.w = 1.
    node.tf_buffer.set_transform(transform, 'test_epoch')
    node.tick()
    route = ROSPath()
    route.header.frame_id = 'map'
    route.header.stamp = Time(seconds=seconds, nanoseconds=1).to_msg()
    for px, py in ((x, y), (.2, .3)):
        waypoint = PoseStamped()
        waypoint.header = route.header
        waypoint.pose.position.x, waypoint.pose.position.y = px, py
        route.poses.append(waypoint)
    node.on_path(route)
    packet = {'status': 'READY', 'sent_at_ns': seconds * 1_000_000_000,
              'goal': {'x': .2, 'y': .3, 'frame_id': 'map'},
              'goal_stamp': {'sec': seconds, 'nanosec': 0},
              'path_stamp': {'sec': seconds, 'nanosec': 1}}
    node.on_planner(String(data=json.dumps(packet)))
    app.planner_available = True  # The isolated test injects a planner packet without a ROS publisher.
    return app.state()


def test_sim_viewer_accepts_fresh_pose_goal_and_path_after_clock_rewind():
    with TemporaryDirectory() as directory, patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        context = Context()
        rclpy.init(args=[], context=context, domain_id=219,
                   signal_handler_options=SignalHandlerOptions.NO)
        app = MissionApplication(DEFAULT_MARKERS, 219, True, Path(directory) / 'settings.json')
        node = create_ros_bridge(app, SimpleNamespace(
            camera_topic='/isolated_epoch_camera', base_frame='sim_base'), context)
        try:
            first = _epoch(node, app, 100, -.5, -.4)
            assert first['robot']['pose']['x'] == -.5
            assert first['observed_route']['goal'] == {'x': .2, 'y': .3, 'frame_id': 'map'}
            assert first['observed_route']['active']
            assert len(first['path']) == 2

            # The second Gazebo run starts near zero while this HTTP process stays up.
            node.get_clock().set_ros_time_override(Time(seconds=1))
            cleared = app.state()
            assert cleared['robot']['pose'] is None
            assert cleared['path'] == []
            assert cleared['observed_route']['goal'] is None
            assert not app.sim_external_goal_seen
            assert cleared['map'] is None, 'the old live map must not survive into a new epoch'
            assert node.last_planner_status_stamp_ns == -1
            assert node.last_motion_status_stamp_ns == -1
            assert node.last_health_stamp_ns == -1
            assert node.last_amcl_stamp_ns == 0
            assert node.last_goal_stamp == 0

            node.tick()  # The steady timer refreshes durable subscriptions.
            second = _epoch(node, app, 2, .1, -.2)
            assert second['robot']['pose']['x'] == .1
            assert second['robot']['pose']['y'] == -.2
            assert second['observed_route']['monitoring']
            assert second['observed_route']['active']
            assert second['observed_route']['goal'] == {'x': .2, 'y': .3, 'frame_id': 'map'}
            assert len(second['path']) == 2
            assert app.mission.state == 'IDLE'
            assert app.mission.commands == [], 'clock reset must not dispatch/cancel a mission'
        finally:
            node.destroy_node()
            context.shutdown()


def test_sim_epoch_rebind_replays_map_and_static_tf_published_before_clock_reset():
    """A late clock rewind must not permanently discard latched map/static TF."""
    with TemporaryDirectory() as directory, patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        context = Context()
        rclpy.init(args=[], context=context, domain_id=220,
                   signal_handler_options=SignalHandlerOptions.NO)
        app = MissionApplication(DEFAULT_MARKERS, 220, True, Path(directory) / 'settings.json')
        node = create_ros_bridge(app, SimpleNamespace(
            camera_topic='/isolated_rebind_camera', base_frame='sim_base'), context)
        source = Node('sim_epoch_durable_source', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(source)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        map_pub = source.create_publisher(OccupancyGrid, '/map', durable)
        tf_pub = source.create_publisher(TFMessage, '/tf_static', durable)

        def until(predicate, timeout=3.):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.05)
                if predicate():
                    return True
            return False

        try:
            node.get_clock().set_ros_time_override(Time(seconds=100))
            assert until(lambda: map_pub.get_subscription_count() > 0
                         and tf_pub.get_subscription_count() > 0)
            occupancy = OccupancyGrid()
            occupancy.header.frame_id = 'map'
            occupancy.header.stamp = Time(seconds=100).to_msg()
            occupancy.info.width = occupancy.info.height = 4
            occupancy.info.resolution = .5
            occupancy.info.origin.position.x = -1.
            occupancy.info.origin.position.y = -1.
            occupancy.info.origin.orientation.w = 1.
            occupancy.data = [0] * 16
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.rotation.w = 1.
            moving = TransformStamped()
            moving.header.frame_id = 'odom'
            moving.header.stamp = Time(seconds=100).to_msg()
            moving.child_frame_id = 'sim_base'
            moving.transform.translation.x = .25
            moving.transform.translation.y = -.4
            moving.transform.rotation.w = 1.
            map_pub.publish(occupancy)
            tf_pub.publish(TFMessage(transforms=[transform]))
            assert until(lambda: app.map is not None and not app.map_is_preview
                         and node.tf_buffer.can_transform('map', 'odom', Time()))
            node.tf_buffer.set_transform(moving, 'test_dynamic_tf')
            node.tick()
            assert app.state()['robot']['pose']['x'] == .25
            old_map_sub, old_listener = node.map_sub, node.tf_listener

            # Neither source publishes again. Only a newly matched durable
            # subscription can replay the map and static transform.
            node.get_clock().set_ros_time_override(Time(seconds=1))
            assert app.map is None
            assert node.sim_epoch_rebind_pending
            node.tick()
            assert node.map_sub is not old_map_sub
            assert node.tf_listener is not old_listener
            assert until(lambda: app.map is not None and not app.map_is_preview
                         and node.tf_buffer.can_transform('map', 'odom', Time()))
            # The durable map and static edge are back, but the old dynamic
            # odom->base transform must never count as a new-epoch pose.
            node.tick()
            assert app.state()['robot']['pose'] is None
            moving.header.stamp = Time(seconds=1).to_msg()
            node.tf_buffer.set_transform(moving, 'test_new_epoch_dynamic_tf')
            node.tick()
            state = app.state()
            assert state['robot']['pose']['x'] == .25
            assert state['map']['preview'] is False
            assert app.mission.commands == []
        finally:
            executor.remove_node(source)
            executor.remove_node(node)
            source.destroy_node()
            node.destroy_node()
            executor.shutdown()
            context.shutdown()


def test_zero_stamp_map_base_transform_does_not_refresh_robot_pose():
    """A static or invalid map->base stamp cannot masquerade as fresh odometry."""
    with TemporaryDirectory() as directory, patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        context = Context()
        rclpy.init(args=[], context=context, domain_id=221,
                   signal_handler_options=SignalHandlerOptions.NO)
        app = MissionApplication(DEFAULT_MARKERS, 221, True, Path(directory) / 'settings.json')
        node = create_ros_bridge(app, SimpleNamespace(
            camera_topic='/isolated_zero_stamp_camera', base_frame='sim_base'), context)
        try:
            node.get_clock().set_ros_time_override(Time(seconds=1))
            _map(app)
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'sim_base'
            transform.transform.rotation.w = 1.
            node.tf_buffer.set_transform_static(transform, 'test_zero_stamp_tf')
            node.tick()
            assert app.state()['robot']['pose'] is None
            assert app.pose_at == 0.
            assert app.pose_tf_stamp_ns == 0
        finally:
            node.destroy_node()
            context.shutdown()


def test_repeated_transform_stamp_does_not_refresh_wall_clock_pose_age():
    """A stopped Gazebo clock must not keep the last robot icon fresh forever."""
    with TemporaryDirectory() as directory, patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}):
        context = Context()
        rclpy.init(args=[], context=context, domain_id=222,
                   signal_handler_options=SignalHandlerOptions.NO)
        app = MissionApplication(DEFAULT_MARKERS, 222, True, Path(directory) / 'settings.json')
        node = create_ros_bridge(app, SimpleNamespace(
            camera_topic='/isolated_repeated_stamp_camera', base_frame='sim_base'), context)
        try:
            first = _epoch(node, app, 100, -.5, -.4)
            assert first['robot']['pose'] is not None
            last_stamp = app.pose_tf_stamp_ns
            app.pose_at = time.monotonic() - 3.
            node.tick()  # TF buffer still has exactly the same ROS timestamp.
            assert app.pose_tf_stamp_ns == last_stamp
            assert app.state()['robot']['pose'] is None
            assert app.state()['robot']['pose_age'] >= 3.
        finally:
            node.destroy_node()
            context.shutdown()
