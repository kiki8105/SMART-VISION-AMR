"""Initialize classroom AMCL at a physically verified Home pose.

This command must be run by an operator who has parked the robot at Home.
It never moves the robot and refuses to initialize against competing maps.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import SetInitialPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from smart_vision_amr.localization_geometry import (occupied_distance, scan_map_agreement,
                                                    scan_points)
from smart_vision_amr.web_mission_server import load_markers, load_saved_map, maps_match


def source_fresh(stamp, now_ns, max_age=2.0):
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age = (now_ns - stamp_ns) / 1e9
    return stamp_ns > 0 and -.25 <= age < max_age


def map_from_message(message):
    info = message.info
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    metadata = {'width': info.width, 'height': info.height, 'resolution': info.resolution,
                'origin': {'x': info.origin.position.x, 'y': info.origin.position.y, 'yaw': yaw},
                'frame_id': message.header.frame_id or 'map'}
    data = np.asarray(message.data, dtype=np.int16)
    if info.width <= 0 or info.height <= 0 or data.size != info.width * info.height:
        raise ValueError('수신된 /map 크기와 셀 수가 다릅니다.')
    return metadata, data.reshape(info.height, info.width)


def only_expected_map_server(publishers):
    return len(publishers) == 1 and publishers[0].node_name == 'map_server'


def stationary(odom):
    return abs(odom.twist.twist.linear.x) < .025 and abs(odom.twist.twist.angular.z) < .06


def valid_scan(scan):
    return sum(math.isfinite(value) and scan.range_min <= value <= scan.range_max
               for value in scan.ranges) >= 12


def wait_for(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
        if predicate():
            return True
    return False


def laser_pose(node, buffer, frame):
    try:
        transform = buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
    except TransformException:
        return None
    q = transform.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return transform.transform.translation.x, transform.transform.translation.y, yaw


def home_scan_match(scan, pose, home, metadata, data):
    points = scan_points(scan, pose)
    distance = occupied_distance(data, metadata['resolution'])
    return scan_map_agreement(points, home, metadata, distance)


def fresh_home_transform(node, buffer, home):
    try:
        transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    except TransformException:
        return False
    stamp_ns = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
    # AMCL deliberately stamps map→odom slightly ahead for its transform
    # tolerance. Follow the web bridge's two-second absolute age criterion.
    if not stamp_ns or abs(node.get_clock().now().nanoseconds - stamp_ns) >= 2_000_000_000:
        return False
    # AMCL may adjust the estimate as the laser comes in. Verify that the
    # completed initialization is near the Home entered by the operator.
    dx = transform.transform.translation.x - home['x']
    dy = transform.transform.translation.y - home['y']
    q = transform.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    yaw_error = math.atan2(math.sin(yaw - home['yaw']), math.cos(yaw - home['yaw']))
    return math.hypot(dx, dy) < .5 and abs(yaw_error) < math.radians(20.)


def run(map_file, markers_file, *, force_verified_home=False):
    expected_metadata, expected_data = load_saved_map(map_file)
    markers = load_markers(markers_file)
    home = next((marker for marker in markers if marker['id'] == 0), None)
    if home is None:
        raise ValueError('마커 설정에 Home(0)이 없습니다.')

    rclpy.init()
    node = Node('classroom_home_initial_pose')
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    latest = {'map': None, 'scan': None, 'odom': None}
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda msg: latest.__setitem__('map', msg), durable)
    node.create_subscription(LaserScan, '/scan', lambda msg: latest.__setitem__('scan', msg), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', lambda msg: latest.__setitem__('odom', msg), qos_profile_sensor_data)
    client = node.create_client(SetInitialPose, '/set_initial_pose')
    try:
        if not wait_for(node, lambda: (client.service_is_ready() and latest['map'] is not None
                                       and latest['scan'] is not None and latest['odom'] is not None), 30.):
            raise RuntimeError('AMCL 서비스 또는 /map, /scan, /odom 수신 대기 시간이 초과됐습니다.')
        publishers = node.get_publishers_info_by_topic('/map')
        if not only_expected_map_server(publishers):
            names = ', '.join(endpoint.node_name for endpoint in publishers) or '없음'
            raise RuntimeError(f'/map 발행자가 {names}입니다. SLAM을 종료하고 map_server만 남기세요.')
        metadata, data = map_from_message(latest['map'])
        if not maps_match(expected_metadata, expected_data, metadata, data):
            raise RuntimeError('현재 /map이 설정한 강의실 지도와 다릅니다. AMCL 초기 위치를 넣지 않았습니다.')
        now_ns = node.get_clock().now().nanoseconds
        if not source_fresh(latest['scan'].header.stamp, now_ns):
            raise RuntimeError('새로운 /scan이 없습니다. 로봇 LiDAR/Bringup을 확인하세요.')
        if not valid_scan(latest['scan']):
            raise RuntimeError('/scan에 유효한 LiDAR 반사가 충분하지 않습니다.')
        if not source_fresh(latest['odom'].header.stamp, now_ns):
            raise RuntimeError('새로운 /odom이 없습니다. 로봇 Bringup을 확인하세요.')
        if not stationary(latest['odom']):
            raise RuntimeError('로봇이 움직이는 중입니다. Home에서 정지시킨 뒤 다시 실행하세요.')
        if not wait_for(node, lambda: laser_pose(node, buffer, latest['scan'].header.frame_id) is not None, 5.):
            raise RuntimeError('base_footprint→LiDAR TF를 찾지 못했습니다.')
        score = home_scan_match(latest['scan'],
                                laser_pose(node, buffer, latest['scan'].header.frame_id),
                                home, expected_metadata, expected_data)
        if score < .60:
            if not force_verified_home:
                raise RuntimeError(f'현재 LiDAR와 지도 Home 위치 일치율이 {score:.0%}로 낮습니다. '
                                   '실제 위치를 확인하고 수동 초기 위치를 입력하세요. '
                                   'Home을 물리적으로 확인했다면 --force-verified-home을 명시하세요.')
            print(f'주의: LiDAR와 지도 Home 위치 일치율 {score:.0%}. '
                  '운용자의 물리적 Home 확인으로 초기화를 계속합니다.', file=sys.stderr)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = home['x']
        pose.pose.pose.position.y = home['y']
        pose.pose.pose.orientation.z = math.sin(home['yaw'] / 2)
        pose.pose.pose.orientation.w = math.cos(home['yaw'] / 2)
        pose.pose.covariance[0] = pose.pose.covariance[7] = .05 ** 2
        pose.pose.covariance[35] = .15 ** 2
        request = SetInitialPose.Request()
        request.pose = pose
        future = client.call_async(request)
        if not wait_for(node, future.done, 8.) or future.exception() is not None:
            raise RuntimeError('AMCL이 Home 초기 위치를 확인하지 못했습니다.')
        if not wait_for(node, lambda: fresh_home_transform(node, buffer, home), 12.):
            raise RuntimeError('초기 위치를 보냈지만 map→base TF가 생성되지 않았습니다. AMCL과 LiDAR를 확인하세요.')
        print(f"AMCL Home 위치 확인: x={home['x']:.2f}, y={home['y']:.2f}, "
              f"차체 yaw={home['yaw']:.2f} rad. map→base TF 수신 완료.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--markers', required=True)
    parser.add_argument('--force-verified-home', action='store_true',
                        help='LiDAR/지도 불일치 시 물리적 Home 확인 후에만 사용')
    options = parser.parse_args(args)
    try:
        run(options.map, options.markers, force_verified_home=options.force_verified_home)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'Home 초기 위치 설정 실패: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
