"""Feed real stamped stationary messages into planner-only test harnesses."""
from copy import deepcopy
import json
from math import cos, sin

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def stationary_evidence(node, at, *, x, y, yaw=0., receipt_clock=None):
    """Prove stillness with six independent messages; do not stub safety gates.

    ``receipt_clock`` is the unittest mock for monotonic time. Pytest harnesses
    instead expose a mutable steady/ROS clock dictionary.
    """
    for index in range(6):
        now = float(at)-.25+.05*index
        if receipt_clock is not None:
            receipt_clock.return_value = now
        else:
            node.clock.update(steady=now, ros=now)
        ns = round(now*1_000_000_000)
        stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
        frame = node.get_parameter('base_frame').value
        tf = TransformStamped()
        tf.header.frame_id = node.get_parameter('map_frame').value
        tf.header.stamp = deepcopy(stamp)
        tf.child_frame_id = frame
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.z, tf.transform.rotation.w = sin(yaw/2), cos(yaw/2)
        node.tf_buffer.lookup_transform.return_value = tf
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = deepcopy(stamp)
        odom.child_frame_id = frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = sin(yaw/2), cos(yaw/2)
        node.on_odom(odom)
        scan = LaserScan()
        scan.header.frame_id = frame
        scan.header.stamp = deepcopy(stamp)
        scan.angle_min, scan.angle_increment = -3.141592653589793, .1
        scan.range_min, scan.range_max = .05, 5.
        scan.ranges = [float('inf')]*64
        node.on_scan(scan)
        node.on_localization_health(String(data=json.dumps({
            'level': 'ok', 'autonomous_allowed': True, 'sent_at_ns': ns})))
        node.on_motion_status(String(data=json.dumps({
            'mode': 'auto', 'available': True, 'autonomous_allowed': True,
            'estop_latched': False, 'reset_pending': False, 'sent_at_ns': ns})))
        node.last_observation_time = now
    assert node._fresh_motion_pose() is not None
    assert node._reverse_start_stopped()
