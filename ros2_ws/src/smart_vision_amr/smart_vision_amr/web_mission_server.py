"""Local/LAN ArUco mission web server, ROS bridge, and explicit operator controls.

HTTP handlers never spin ROS or wait for services. ROS commands are serialized by
one executor. Camera confirmation is an observation, never a fabricated arrival.
"""
import argparse
from collections import deque
from copy import deepcopy
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import mimetypes
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import unquote, urlsplit

import cv2
import numpy as np
from PIL import Image as PILImage
import yaml

from smart_vision_amr.mission_logic import ACTIVE_STATES, START_STATES, MarkerMission, MissionError
from smart_vision_amr.delivery_service import DeliveryService
from smart_vision_amr.manual_control import ManualOverride
from smart_vision_amr.assisted_localization import AssistedLocalization
from smart_vision_amr.localization_geometry import rotation_clearance, scan_points, occupied_distance, scan_map_agreement
from smart_vision_amr.home_prior_geometry import home_neighborhood_evidence
from smart_vision_amr.home_prior_observer import HomePriorObserver
from smart_vision_amr.operation_status import (operation_summary, route_preview, localization_explanations,
                                             failure_memory_snapshot, local_detour_snapshot)
from smart_vision_amr.navigation_graph import goal_consumers_ready
from smart_vision_amr.marker_alignment import MarkerAlignment
from smart_vision_amr.wall_alignment import estimate_camera_axis_wall

DEFAULT_MARKERS = [
    {'id': 0, 'name': '시작 지점', 'x': -3.0007995856, 'y': -2.0010944836, 'yaw': .0008724679},
    {'id': 1, 'name': '북쪽 경유 지점', 'x': -1.0051626856, 'y': 3.0006485490, 'yaw': .0008724679},
    {'id': 2, 'name': '남쪽 경유 지점', 'x': 1.3998967139, 'y': -2.7972553209, 'yaw': .0008724679},
    {'id': 3, 'name': '목적 지점', 'x': 3.8996340221, 'y': -2.4950742657, 'yaw': .0008724679},
]


def marker_measurements(corners, ids, width, height):
    """Upright, unmirrored image geometry; this is not an ML confidence score.

    Yaw can center horizontally. Reject cropped/tiny/invalid quadrilaterals
    rather than treating an ID alone as evidence of a centered marker.
    """
    if ids is None or width <= 0 or height <= 0:
        return []
    result = []
    for ident, points in zip(np.asarray(ids).reshape(-1), corners):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if (points.shape != (4, 2) or not np.all(np.isfinite(points))
                or np.any(points[:, 0] < 1) or np.any(points[:, 0] >= width-1)
                or np.any(points[:, 1] < 1) or np.any(points[:, 1] >= height-1)
                or np.min(np.linalg.norm(points-np.roll(points, 1, axis=0), axis=1)) < 8.
                or abs(cv2.contourArea(points.astype(np.float32))) < 64.):
            continue
        result.append({'id': int(ident),
                       'center_x': float(2.*np.mean(points[:, 0])/width-1.),
                       'confidence': 1.})
    return result


def load_markers(path=None):
    source = DEFAULT_MARKERS if path is None else yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if isinstance(source, dict):
        source = source.get('markers', source)
    if isinstance(source, dict):
        source = [dict(value, id=int(key)) for key, value in source.items()]
    if not isinstance(source, list) or not source:
        raise ValueError('markers must be a non-empty list')
    markers, seen = [], set()
    for item in source:
        ident = item.get('id')
        if type(ident) is not int or not 0 <= ident < 50 or ident in seen:
            raise ValueError('unique DICT_4X4_50 marker IDs (0..49) required')
        seen.add(ident)
        marker = {'id': ident, 'name': str(item.get('name', f'마커 {ident}'))}
        marker['camera'] = str(item.get('camera', 'rear')).lower()
        if marker['camera'] not in {'front', 'rear'}:
            raise ValueError('marker camera must be front or rear')
        for key in ('x', 'y', 'yaw'):
            marker[key] = float(item[key])
            if not math.isfinite(marker[key]):
                raise ValueError('finite marker observation poses required')
        markers.append(marker)
    return markers


def default_config_path():
    return Path(os.environ.get('XDG_CONFIG_HOME', str(Path.home()/'.config')))/'smart-vision-amr'/'web-mission.json'


def map_png(data):
    """Render a bottom-left-origin OccupancyGrid without turning unknown into free."""
    raster = np.full(data.shape, 205, np.uint8)
    known = data >= 0
    raster[known] = np.clip(254*(1-data[known]/100.0), 0, 254).astype(np.uint8)
    ok, encoded = cv2.imencode('.png', np.flipud(raster))
    if not ok:
        raise ValueError('지도 PNG 변환에 실패했습니다.')
    return encoded.tobytes()


def cell_boundary_paths(mask, epsilon=.6):
    """Trace exact pixel edges, then simplify only the displayed outline.

    findContours alone follows pixel centers and collapses one-cell walls. Here
    a cell at column x/row y occupies [x,x+1] x [y,y+1]. Interior shared edges
    disappear; hole rings and disconnected components remain separate rings.
    """
    mask = np.asarray(mask, dtype=bool)
    padded = np.pad(mask, 1)
    edges = {}
    directions = (((0, 0), (1, 0), mask & ~padded[:-2, 1:-1]),
                  ((1, 0), (1, 1), mask & ~padded[1:-1, 2:]),
                  ((1, 1), (0, 1), mask & ~padded[2:, 1:-1]),
                  ((0, 1), (0, 0), mask & ~padded[1:-1, :-2]))
    for begin, end, exposed in directions:
        for row, column in np.argwhere(exposed):
            start = (int(column)+begin[0], int(row)+begin[1])
            finish = (int(column)+end[0], int(row)+end[1])
            edges.setdefault(start, set()).add(finish)
    heading = {(1, 0): 0, (0, 1): 1, (-1, 0): 2, (0, -1): 3}
    turn_priority = {1: 0, 0: 1, 3: 2, 2: 3}
    paths = []
    # Sorted starts make the representation stable across identical map loads.
    for start in sorted(edges):
        while start in edges:
            current = start
            previous_heading = None
            ring = []
            while True:
                ring.append(current)
                choices = edges[current]
                if previous_heading is None:
                    following = min(choices)
                else:
                    def turn_rank(point):
                        direction = heading[(point[0]-current[0], point[1]-current[1])]
                        return turn_priority[(direction-previous_heading) % 4]
                    following = min(choices, key=turn_rank)
                choices.remove(following)
                if not choices:
                    del edges[current]
                previous_heading = heading[(following[0]-current[0], following[1]-current[1])]
                current = following
                if current == start:
                    break
            original = np.asarray(ring, np.int32).reshape(-1, 1, 2)
            simplified = cv2.approxPolyDP(original, min(max(float(epsilon), 0.), .65), True)
            if len(simplified) < 3 or cv2.contourArea(simplified) == 0:
                simplified = original
            paths.append(simplified.reshape(-1, 2).tolist())
    return paths


def map_layout(data):
    """Vector floorplan from real map cells, in the PNG's top-left coordinates."""
    top_down = np.flipud(data)
    height, width = top_down.shape
    return {'width': width, 'height': height,
            'free_paths': cell_boundary_paths(top_down == 0),
            'occupied_paths': cell_boundary_paths(top_down >= 65)}


def load_saved_map(path):
    """Read a Nav2 trinary map for preview and comparison with the real /map.

    PNG pixels run top to bottom, whereas OccupancyGrid rows run bottom to top.
    Strict threshold comparisons match map_server's trinary interpretation.
    """
    path = Path(path).expanduser().resolve()
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or not isinstance(document.get('image'), str):
        raise ValueError('지도 YAML에 image 경로가 필요합니다.')
    if document.get('mode', 'trinary') != 'trinary':
        raise ValueError('웹 저장 지도는 mode: trinary 형식이어야 합니다.')
    resolution = float(document['resolution'])
    origin = document['origin']
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError('지도 origin은 [x, y, yaw] 형식이어야 합니다.')
    origin = [float(value) for value in origin]
    free = float(document.get('free_thresh', .25))
    occupied = float(document.get('occupied_thresh', .65))
    negate = document.get('negate', 0)
    if (not math.isfinite(resolution) or resolution <= 0
            or not all(math.isfinite(value) for value in origin)
            or not 0 <= free < occupied <= 1 or negate not in (0, 1)):
        raise ValueError('지도의 해상도, 원점 또는 임계값이 올바르지 않습니다.')
    image_path = Path(document['image']).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent/image_path
    with PILImage.open(image_path) as image:
        width, height = image.size
        if width <= 0 or height <= 0 or width*height > 25000000:
            raise ValueError('지도 크기는 1~25000000 셀이어야 합니다.')
        pixels = np.asarray(image.convert('RGBA'), dtype=np.uint8)
    shade = pixels[:, :, :3].mean(axis=2)
    probability = (shade if negate else 255.0-shade)/255.0
    grid = np.full((height, width), -1, dtype=np.int16)
    opaque = pixels[:, :, 3] == 255
    grid[opaque & (probability < free)] = 0
    grid[opaque & (probability > occupied)] = 100
    grid = np.flipud(grid).copy()
    metadata = {'width': width, 'height': height, 'resolution': resolution,
                'origin': dict(zip(('x', 'y', 'yaw'), origin)), 'frame_id': 'map'}
    return metadata, grid


def map_geometry_matches(expected_metadata, metadata):
    """Compare the coordinate frames independently of their display artwork."""
    if (metadata['width'] != expected_metadata['width']
            or metadata['height'] != expected_metadata['height']
            or metadata['frame_id'] != expected_metadata['frame_id']
            or not math.isclose(metadata['resolution'], expected_metadata['resolution'], rel_tol=0, abs_tol=1e-7)):
        return False
    for key in ('x', 'y'):
        if not math.isclose(metadata['origin'][key], expected_metadata['origin'][key], rel_tol=0, abs_tol=1e-6):
            return False
    angle = metadata['origin']['yaw']-expected_metadata['origin']['yaw']
    if abs(math.atan2(math.sin(angle), math.cos(angle))) > 1e-6:
        return False
    return True


def maps_match(expected_metadata, expected_data, metadata, data):
    """A saved image alone cannot authorize motion on an unrelated live map."""
    return map_geometry_matches(expected_metadata, metadata) and np.array_equal(expected_data, data)


def load_floorplan(path, expected_metadata):
    """Read explicitly authored display polygons tied to one saved map frame.

    Coordinates describe pixel edges from the image's top-left corner. These
    polygons never replace /map cells, the saved raster, or navigation inputs.
    """
    document = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(document, dict) or not isinstance(document.get('map'), dict):
        raise ValueError('평면도에는 기준 map 정보가 필요합니다.')
    geometry = document['map']
    width, height = geometry.get('width'), geometry.get('height')
    origin = geometry.get('origin')
    resolution = geometry.get('resolution')
    def numeric(value):
        if type(value) not in (int, float):
            return False
        try:
            return math.isfinite(value)
        except OverflowError:
            return False
    if (type(width) is not int or type(height) is not int or width <= 0 or height <= 0
            or not numeric(resolution) or resolution <= 0
            or not isinstance(origin, list) or len(origin) != 3
            or not all(numeric(value) for value in origin)):
        raise ValueError('평면도의 지도 크기, 해상도와 원점이 올바르지 않습니다.')
    metadata = {'width': width, 'height': height, 'resolution': resolution,
                'origin': dict(zip(('x', 'y', 'yaw'), origin)), 'frame_id': 'map'}
    if not map_geometry_matches(expected_metadata, metadata):
        raise ValueError('평면도의 크기, 해상도와 원점이 설정 지도와 일치해야 합니다.')
    if document.get('style', 'architectural') != 'architectural':
        raise ValueError('평면도 style은 architectural이어야 합니다.')
    layout = {'width': width, 'height': height, 'style': 'architectural',
              'label': str(document.get('label', '평면도'))}
    for key in ('free_paths', 'occupied_paths', 'obstacle_paths'):
        rings = document.get(key, [])
        if not isinstance(rings, list) or (key != 'obstacle_paths' and not rings):
            raise ValueError(f'평면도 {key}에는 다각형 목록이 필요합니다.')
        validated = []
        for ring in rings:
            if not isinstance(ring, list) or len(ring) < 3:
                raise ValueError(f'평면도 {key}의 다각형에는 점이 3개 이상 필요합니다.')
            points = []
            for point in ring:
                if (not isinstance(point, list) or len(point) != 2
                        or not all(numeric(value) for value in point)
                        or not 0 <= point[0] <= width or not 0 <= point[1] <= height):
                    raise ValueError(f'평면도 {key}의 점은 지도 범위 안의 유한한 [x, y]여야 합니다.')
                points.append([float(point[0]), float(point[1])])
            if len({tuple(point) for point in points}) < 3 or cv2.contourArea(np.asarray(points, np.float32)) <= 0:
                raise ValueError(f'평면도 {key}에 면적이 없는 다각형이 있습니다.')
            validated.append(points)
        layout[key] = validated
    return layout


class MissionApplication:
    def __init__(self, markers, domain_id=76, use_sim_time=False, config_file=None, confirmations=3,
                 access_mode='code'):
        if access_mode not in {'code', 'domain'}:
            raise ValueError('access_mode must be code or domain')
        self.lock = threading.RLock()
        self.access_mode = access_mode
        self.config_file = Path(config_file) if config_file else default_config_path()
        self.saved = {}
        try:
            saved = json.loads(self.config_file.read_text(encoding='utf-8'))
            if isinstance(saved, dict):
                self.saved = saved
        except (OSError, ValueError):
            pass
        self.markers = deepcopy(markers)
        self.domain_id, self.use_sim_time = int(domain_id), bool(use_sim_time)
        self.confirmations = confirmations
        self.mission = self._new_mission()
        self.connected = False
        self.connection_error = ''
        self.pending_connection = None
        self.pose = None
        self.pose_tf_stamp_ns = 0
        self.odom_pose = None
        self.pose_at = self.odom_at = self.scan_at = self.camera_at = 0.0
        self.speed = {'linear': 0.0, 'angular': 0.0}
        self.battery_topic = '/battery_state'
        self.battery_at = None
        self.battery_voltage = self.battery_percentage = None
        self.map = None
        self.map_png = None
        self.map_layout = None
        self.map_version = 0
        self._display_map_metadata = self._display_map_data = None
        self.map_is_preview = False
        self.configured_map_file = None
        self.map_matches_configured = None
        self.received_map_metadata = None
        self._configured_map = None
        self._configured_layout = None
        self.configured_floorplan_file = None
        self.camera_jpeg = None
        self.front_camera_jpeg = None
        self.front_camera_at = 0.0
        self.frame_number = 0
        self.detected_ids = []
        self.front_detected_ids = []
        self.marker_alignment = {'state': 'IDLE', 'reason': '섹터 도착 후 표식을 탐색합니다.'}
        self.path = []
        self.navigation = {'status': 'IDLE', 'planner_status': 'IDLE', 'message': ''}
        self.planner_details, self.planner_details_at = {}, 0.
        self.goal_available = False
        self.planner_available = False
        self.cancel_available = False
        self.cancel_pending = False
        self.navigation_active = False
        self.manual = ManualOverride()
        self.manual_mux_at = 0.
        self.manual_mux_status = {}
        self.localization_health = {}
        self.localization_health_at = 0.
        self.failure_memory_status = {}
        self.failure_memory_at = 0.
        self.require_localization_health = False
        self.require_battery_health = False
        self.minimum_battery_voltage = 0.0
        self.safety_request = None
        self.safety_result = ''
        self.navigation_command_publishers = 0
        self.unmanaged_velocity_publishers = []
        self.manual_cancel_requested = False
        self.manual_auto_ready = False
        # Short Wi-Fi/DDS gaps stop rotation immediately; the bridge then
        # fails after a bounded pause. Keep the logic's odom timeout above
        # the bridge's 0.8 s freshness threshold plus its 1.0 s grace.
        self.localization = AssistedLocalization(sensor_timeout=2.0)
        self.localization_requested = False
        self.localization_stop_requested = False
        self.localization_services_ready = False
        self.localization_amcl_active_at = 0.
        self.localization_clear = False
        self.localization_clearance = None
        self.localization_scan_at = 0.
        self.localization_odom_at = 0.
        self.localization_invalidated = False
        self.localization_use_existing_pose = False
        self.localization_startup_home = False
        self.localization_home_seed = None
        self.localization_reference_pose = None
        self.localization_reference_odom = None
        self.localization_match = None
        self.localization_distance = None
        self.localization_service_pending = False
        self.localization_control = {'state': 'IDLE', 'reason': ''}
        self.home_seed_request = None
        self.home_seed_result = ''
        self.home_seed_match = None
        self.home_seed_match_at = 0.
        self.home_seed_match_key = None
        self.home_prior_evidence = None
        self.manual_pose_service_ready = False
        self.manual_pose_request = None
        self.manual_pose_requested = None
        self.manual_pose_state = 'IDLE'
        self.manual_pose_result = ''
        self.manual_pose_sent_at = 0.
        self.manual_pose_amcl_at = 0.
        self.manual_pose_amcl_received_at = 0.
        self.manual_pose_match = None
        self.manual_pose_map_version = 0
        self.aruco_available = hasattr(cv2, 'aruco')
        self.pairing_code = f'{secrets.randbelow(1000000):06d}'
        self.sessions = {}
        self.session_domains = {}
        self.pair_attempts = {}
        self.inventory_join_code = f'{secrets.randbelow(1000000):06d}'
        while self.inventory_join_code == self.pairing_code:
            self.inventory_join_code = f'{secrets.randbelow(1000000):06d}'
        self.inventory_sessions = {}
        self.inventory_attempts = {}
        self.delivery = DeliveryService(self)

    def load_map_preview(self, path):
        metadata, data = load_saved_map(path)
        encoded = map_png(data)
        with self.lock:
            self.configured_map_file = str(Path(path).expanduser().resolve())
            self._configured_map = (metadata, data, encoded)
            self._configured_layout = map_layout(data)
            self.configured_floorplan_file = None
            self.restore_map_preview()

    def load_floorplan(self, path):
        """Replace display geometry only, invalidating a previously cached view."""
        with self.lock:
            if self._configured_map is None:
                raise ValueError('--floorplan-file에는 --map-file 설정이 필요합니다.')
            layout = load_floorplan(path, self._configured_map[0])
            self._configured_layout = layout
            self.configured_floorplan_file = str(Path(path).expanduser().resolve())
            if self.map is not None:
                self._show_map(self._display_map_metadata, self._display_map_data,
                               preview=self.map['preview'], source=self.map['source'],
                               match=self.map['configured_match'], encoded=self.map_png,
                               layout=layout, force=True)

    def _show_map(self, metadata, data, *, preview, source, match, encoded=None, layout=None, force=False):
        """Cache raster/vector geometry; status changes alone do not invalidate it."""
        if (force or self.map is None or metadata != self._display_map_metadata
                or not np.array_equal(data, self._display_map_data)):
            self.map_version += 1
            self.map_png = map_png(data) if encoded is None else encoded
            self.map_layout = dict(map_layout(data) if layout is None else layout, version=self.map_version)
            self._display_map_metadata = deepcopy(metadata)
            self._display_map_data = data.copy()
        self.map = dict(deepcopy(metadata), version=self.map_version,
                        preview=preview, source=source, configured_match=match)

    def restore_map_preview(self):
        """Reconnects must wait for a new live map, even if a preview is visible."""
        with self.lock:
            self.map_matches_configured = None
            self.received_map_metadata = None
            self.map_is_preview = self._configured_map is not None
            if self._configured_map is None:
                self.map = self.map_png = self.map_layout = None
                self._display_map_metadata = self._display_map_data = None
                return
            metadata, data, encoded = self._configured_map
            self._show_map(metadata, data, preview=True, source='saved_preview', match=None,
                           encoded=encoded, layout=self._configured_layout, force=True)

    def receive_map(self, metadata, data):
        """Install a real ROS map and validate it against the configured map."""
        with self.lock:
            self.localization_distance = occupied_distance(data, metadata['resolution'])
            self.map_matches_configured = (maps_match(*self._configured_map[:2], metadata, data)
                                           if self._configured_map is not None else None)
            self.received_map_metadata = deepcopy(metadata)
            self.map_is_preview = self.map_matches_configured is False
            if self.map_is_preview:
                # An unrelated SLAM/map_server on this domain must not replace
                # the classroom map selected by the operator.
                displayed_metadata, displayed_data, encoded = self._configured_map
                self._show_map(displayed_metadata, displayed_data, preview=True,
                               source='saved_preview', match=False, encoded=encoded,
                               layout=self._configured_layout)
            else:
                self._show_map(metadata, data, preview=False, source='ros', match=self.map_matches_configured,
                               layout=self._configured_layout if self.map_matches_configured is True else None)
            if self.map_matches_configured is False and self.mission.state in ACTIVE_STATES:
                self.mission.pause()
                self.mission.reason = '설정 지도와 다른 /map이 수신되어 일시정지했습니다. 지도 확인 후 명시적으로 재개하세요.'
                self.mission.event(self.mission.reason, 'warning')

    def _new_mission(self):
        domain_saved = self.saved.get('domains', {}).get(str(self.domain_id), {})
        route = domain_saved.get('route', [2, 3])
        try:
            return MarkerMission(self.markers, route, confirmations=self.confirmations)
        except (MissionError, KeyError):
            return MarkerMission(self.markers, [self.markers[0]['id']], confirmations=self.confirmations)

    def save(self):
        """Atomic user configuration, separate from source-control files."""
        candidate = deepcopy(self.saved)
        domains = candidate.setdefault('domains', {})
        domains[str(self.domain_id)] = {'route': self.mission.pending_route or self.mission.route,
                                        'use_sim_time': self.use_sim_time,
                                        'homepoint': deepcopy(self.delivery.homepoint)}
        candidate['last_domain_id'] = self.domain_id
        self.config_file.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(prefix='.web-mission-', dir=self.config_file.parent)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(candidate, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, 0o600)
            os.replace(temporary, self.config_file)
            self.saved = candidate
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def readiness(self, for_home=None, allow_camera_degraded=False, marker_ids=None):
        if for_home is None:
            for_home = self.mission.returning_home
        now = time.monotonic()
        missing = []
        conditions = [(self.connected, 'ROS 연결'),
                            (self.map is not None and not self.map_is_preview, '/map'),
                            (self.pose is not None and now-self.pose_at < 2.0, '최신 map→base TF'),
                            (now-self.scan_at < 2.0, '최신 /scan'),
                            (now-self.odom_at < 2.0, '최신 /odom'),
                            (self.goal_available and self.planner_available, 'A* 계획 노드'),
                            (self.cancel_available, '주행 취소 서비스')]
        if (self.connected and self.map is not None and not self.map_is_preview and
                now-self.scan_at < 2.0 and now-self.odom_at < 2.0 and self.pose is None):
            conditions.append((False, 'AMCL 초기 위치·수렴 대기: 자세 초기화 또는 실제 위치 지정'))
        if self.map_matches_configured is False:
            conditions.append((False, '설정 지도와 다른 /map'))
        if self.require_localization_health:
            conditions.append((now-self.localization_health_at < 1.5 and
                               self.localization_health.get('autonomous_allowed') is True,
                               '위치 신뢰도: '+(' · '.join(localization_explanations(self.localization_health))
                                or '위치 추정 상태 수신 대기')))
        if self.require_battery_health:
            power = self.power_state(now)
            conditions.append((power['status'] == 'online',
                               '최신 유효한 /battery_state 전압 ('+power['status']+')'))
            if self.minimum_battery_voltage > 0.0:
                conditions.append((power['status'] == 'online' and
                                   power['voltage'] >= self.minimum_battery_voltage - 0.001,
                                   f'배터리 전압 {self.minimum_battery_voltage:.2f}V 이상'))
        warnings = []
        if not for_home:
            if marker_ids is None:
                marker_ids = (self.mission.route[self.mission.current_index:]
                              if self.mission.state in ACTIVE_STATES | {'PAUSED'}
                              else self.mission.pending_route or self.mission.route)
            cameras = {self.mission.markers[marker_id].get('camera', 'rear') for marker_id in marker_ids}
            for camera, received_at, label in (('rear', self.camera_at, '카메라'),
                                               ('front', self.front_camera_at, '전방 카메라')):
                if camera not in cameras:
                    continue
                camera_fresh = now-received_at < 2.0
                if allow_camera_degraded:
                    if not camera_fresh:
                        warnings.append(f'{label} 영상 수신 지연: 주행은 계속하되 마커 확인은 영상 복구까지 대기합니다.')
                else:
                    conditions.append((camera_fresh, f'최신 {label} 영상'))
            conditions.append((self.aruco_available, 'OpenCV ArUco'))
        if self.localization.active or self.localization_invalidated:
            conditions.append((False, '라이다 위치 확인 완료 또는 수동 초기 위치 지정'))
        for valid, text in conditions:
            if not valid:
                missing.append(text)
        return {'ready': not missing, 'missing': missing, 'warnings': warnings,
                'map': {'preview': self.map_is_preview, 'configured_file': self.configured_map_file,
                        'configured_match': self.map_matches_configured,
                        'received': deepcopy(self.received_map_metadata)}}

    def state(self, local=False, owner=None):
        with self.lock:
            now = time.monotonic()
            def age(then):
                return round(now-then, 2) if then else None
            connection = {'domain_id': self.domain_id, 'connected': self.connected,
                          'access_mode': self.access_mode,
                          'use_sim_time': self.use_sim_time, 'error': self.connection_error,
                          'reconnecting': self.pending_connection is not None}
            if local and self.access_mode == 'code':
                connection['pairing_code'] = self.pairing_code
            delivery = self.delivery.snapshot()
            if local and self.access_mode == 'code':
                delivery['inventory_join_code'] = self.inventory_join_code
            snapshot = {'connection': connection, 'mission': self.mission.snapshot(),
                    'manual': self.manual_snapshot(owner),
                    'localization': self.localization_snapshot(),
                    'delivery': delivery,
                    'markers': deepcopy(self.markers),
                    'robot': {'pose': (None if self.map_is_preview or now-self.pose_at >= 2.
                                       else deepcopy(self.pose)), 'pose_age': age(self.pose_at),
                              'speed': dict(self.speed), 'scan_age': age(self.scan_at),
                              'camera_age': age(self.camera_at), 'front_camera_age': age(self.front_camera_at),
                              'odom_age': age(self.odom_at),
                              'power': self.power_state(now)},
                    'map': deepcopy(self.map), 'map_version': self.map_version,
                    'path': [] if self.map_is_preview else deepcopy(self.path),
                    'detection': {'ids': list(self.detected_ids) if now-self.camera_at < 2. else [],
                                  'front_ids': list(self.front_detected_ids) if now-self.front_camera_at < 2. else [],
                                  'expected_camera': (self.mission.markers[self.mission.expected_id].get('camera', 'rear')
                                                      if self.mission.expected_id is not None else None),
                                  'expected_id': self.mission.expected_id,
                                  'confirmations': self.mission.confirmations,
                                  'required': self.mission.required_confirmations,
                                  'confirmed': self.mission.state == 'COMPLETED'},
                    'marker_alignment': deepcopy(self.marker_alignment),
                    'navigation': dict(self.navigation),
                    'health': self.readiness(allow_camera_degraded=self.mission.state in ACTIVE_STATES | {'PAUSED'}),
                    'events': deepcopy(self.mission.events[-30:]), 'revision': self.mission.revision}
            mux_fresh = now-self.manual_mux_at < 1.0
            snapshot['safety'] = dict(self.manual_mux_status if mux_fresh else {},
                                      latched=self.mission.estop_latched,
                                      stop_acknowledged=bool(mux_fresh and self.manual_mux_status.get('stop_acknowledged')),
                                      pending=self.safety_request, result=self.safety_result)
            snapshot['localization_health'] = (deepcopy(self.localization_health)
                if now-self.localization_health_at < 1.5 else {'level': 'unknown', 'reasons': ['추정 상태 수신 대기']})
            snapshot['localization_health']['explanations'] = localization_explanations(snapshot['localization_health'])
            snapshot['failure_memory'] = failure_memory_snapshot(
                self.failure_memory_status, self.mission.goal_stamp, self.mission.state,
                now-self.failure_memory_at)
            snapshot['local_detour'] = local_detour_snapshot(
                self.planner_details, self.mission.goal_stamp, self.mission.state,
                now-self.planner_details_at)
            snapshot['operation'] = operation_summary(snapshot)
            snapshot['route_preview'] = route_preview(snapshot)
            return snapshot

    def command(self, endpoint, payload):
        with self.lock:
            if endpoint == '/api/localization/initial-pose':
                return self.manual_pose_command(payload)
            if endpoint == '/api/localization/seed-home':
                return self.seed_home_command(payload)
            if endpoint in {'/api/localization/start', '/api/localization/start-home', '/api/localization/stop'}:
                return self.localization_command(endpoint, payload)
            if self.localization.active:
                if endpoint in {'/api/mission/start', '/api/mission/resume', '/api/mission/restart',
                                '/api/dispatch/start', '/api/home/set', '/api/home/reset', '/api/connection', '/api/mission'}:
                    raise MissionError('위치 확인을 마치거나 중단한 뒤 변경하세요.')
                if endpoint in {'/api/mission/pause', '/api/mission/cancel', '/api/mission/home', '/api/emergency-stop'}:
                    self.cancel_localization('사용자 명령으로 위치 확인을 중단했습니다.')
            if self.manual.in_progress:
                if endpoint in {'/api/mission/start', '/api/mission/resume', '/api/mission/restart',
                                '/api/dispatch/start', '/api/home/set', '/api/home/reset', '/api/connection'}:
                    raise MissionError('조이스틱을 놓고 자동 주행 재개가 끝난 뒤 변경하세요.')
                if endpoint in {'/api/mission/pause', '/api/mission/cancel', '/api/mission/home', '/api/emergency-stop'}:
                    self.manual.finish()
                    self.manual_auto_ready = False
                    self.manual_cancel_requested = False
                    # An explicit pause/home/emergency command overrides auto-resume.
                    if endpoint == '/api/mission/pause' and self.mission.state == 'PAUSED':
                        self.mission.commands = [('cancel', None), ('stop', None)]
                        self.mission.reason = '사용자가 일시정지했습니다. 재개 버튼으로만 다시 출발합니다.'
                        return self.state()
            if endpoint == '/api/dispatch/start':
                self.delivery.start(payload)
            elif endpoint == '/api/home/set':
                self.delivery.set_home()
            elif endpoint == '/api/home/reset':
                self.delivery.reset_home()
            elif endpoint == '/api/mission':
                if self.delivery.active:
                    raise MissionError('배송 운행 중에는 수동 경유 순서를 변경할 수 없습니다.')
                previous = deepcopy(self.mission.__dict__)
                try:
                    self.mission.update_route(payload.get('route'))
                    self.save()
                except OSError:
                    self.mission.__dict__.clear()
                    self.mission.__dict__.update(previous)
                    raise
            elif endpoint in {'/api/mission/start', '/api/mission/resume', '/api/mission/restart'}:
                if self.delivery.active and not endpoint.endswith('/resume'):
                    raise MissionError('배송을 마치고 Homepoint로 복귀한 뒤 수동 실습을 시작하세요.')
                if self.pending_connection is not None:
                    raise MissionError('ROS 연결을 변경하는 중입니다. 연결 완료 후 시작하세요.')
                route = (self.mission.route[self.mission.current_index:]
                         if endpoint.endswith('/resume') else self.mission.pending_route or self.mission.route)
                health = self.readiness(for_home=endpoint.endswith('/resume') and self.mission.returning_home,
                                        allow_camera_degraded=endpoint.endswith('/resume'), marker_ids=route)
                if not health['ready']:
                    raise MissionError('시작 조건을 확인하세요: '+', '.join(health['missing']))
                # Persistence must succeed before any command is queued.
                self.save()
                if endpoint.endswith('/start'):
                    self.mission.start(home=self.delivery.homepoint, auto_return_home=True)
                elif endpoint.endswith('/restart'):
                    self.mission.restart()
                else:
                    self.mission.resume()
            elif endpoint == '/api/mission/pause':
                self.mission.pause()
            elif endpoint in {'/api/mission/cancel', '/api/mission/home'}:
                if self.pending_connection is not None:
                    raise MissionError('ROS 연결을 변경하는 중입니다. 연결 완료 후 홈 상태를 확인하세요.')
                if not self.mission.estop_latched:
                    try:
                        self.delivery.cancel()
                    except (OSError, ValueError, IndexError) as exc:
                        self.delivery.fail(exc)
                        raise
                    self.mission.home = deepcopy(self.delivery.homepoint)
                self.mission.return_home(canceled=endpoint.endswith('/cancel'))
                health = self.readiness(for_home=True)
                if not health['ready']:
                    self.mission.pause()
                    self.mission.reason = '홈 복귀를 일시정지했습니다. 수신 복구 후 재개하세요: '+', '.join(health['missing'])
                    self.mission.event(self.mission.reason, 'warning')
                    raise MissionError(self.mission.reason)
            elif endpoint == '/api/emergency-stop':
                reset = payload.get('reset', False)
                if type(reset) is not bool:
                    raise MissionError('reset은 true 또는 false여야 합니다.')
                if reset:
                    if self.home_seed_request is not None:
                        raise MissionError('등록 홈 초기 위치 입력이 끝난 뒤 해제하세요.')
                    if self.manual_pose_state == 'WAITING':
                        raise MissionError('초기 위치 검증이 끝난 뒤 비상정지를 해제하세요.')
                    # Never clear the UI latch before the shared mux/guard
                    # handshake acknowledges the operator's reset request.
                    if not self.mission.estop_latched:
                        raise MissionError('현재 비상정지 상태가 아닙니다.')
                    if (time.monotonic()-self.manual_mux_at >= 1.0
                            or self.manual_mux_status.get('estop_latched') is not True
                            or self.manual_mux_status.get('stop_acknowledged') is not True):
                        raise MissionError('차량의 비상정지 확인 응답을 받은 뒤 해제하세요.')
                    self.safety_request = 'reset'
                    self.safety_result = '해제 응답 대기 · 자동 출발하지 않습니다.'
                else:
                    self.pending_connection = None
                    self.mission.emergency_stop()
                    self.delivery.emergency()
                    self.safety_request = 'stop'
                    self.safety_result = '정지 요청 전송 중'
            elif endpoint == '/api/connection':
                if (self.delivery.active or self.mission.state not in {'IDLE', 'COMPLETED', 'HOME_REACHED', 'CANCELED', 'ERROR'} or self.mission.returning_home or self.mission.estop_latched
                        or self.cancel_pending or self.navigation_active
                        or any(kind == 'cancel' for kind, _ in self.mission.commands)):
                    raise MissionError('임무 완료 또는 홈 복귀 완료 후에만 도메인/시계를 변경할 수 있습니다.')
                domain = payload.get('domain_id')
                if type(domain) is not int or not 0 <= domain <= 232:
                    raise MissionError('도메인은 0~232의 정수여야 합니다.')
                use_sim = payload.get('use_sim_time', self.use_sim_time)
                if type(use_sim) is not bool:
                    raise MissionError('use_sim_time은 true 또는 false여야 합니다.')
                if domain != self.domain_id or use_sim != self.use_sim_time:
                    self.pending_connection = (domain, use_sim)
            else:
                raise KeyError(endpoint)
            return self.state()

    def complete_safety_result(self, kind, success, detail):
        """Apply only the current shared-service result, never a stale reset.

        HTTP reset requests leave the mission latched. Only the robot-side
        service acknowledgment, after its sensor-guard handshake, may clear it.
        Neither successful reset nor failure creates a new navigation goal.
        """
        with self.lock:
            if kind != self.safety_request:
                return False
            self.safety_request = None
            self.safety_result = str(detail)
            if success and kind == 'reset':
                self.mission.reset_emergency()
            self.mission.event(('정지 제어 응답: ' if success else '정지 제어 실패: ')+str(detail),
                               'info' if success else 'warning')
            return bool(success)

    def sync_delivery(self):
        try:
            self.delivery.sync()
        except (OSError, ValueError, IndexError) as exc:
            self.delivery.fail(exc)

    def manual_readiness(self):
        now = time.monotonic()
        if self.mission.estop_latched or str(self.manual_mux_status.get('mode', '')).upper() == 'ESTOP':
            return False, '비상정지를 먼저 해제하세요.'
        if not self.connected or self.pending_connection is not None:
            return False, '차량 ROS 연결을 확인하세요.'
        if now-self.manual_mux_at >= 1.0:
            return False, '속도 우선순위 노드 연결을 기다립니다.'
        if self.manual_mux_status.get('available') is False:
            return False, '속도 출력 연결을 확인하세요: '+str(self.manual_mux_status.get('reason', ''))
        if self.unmanaged_velocity_publishers:
            return False, '다른 직접 속도 발행기를 종료하세요: '+', '.join(self.unmanaged_velocity_publishers)
        if now-self.scan_at >= 2.0 or now-self.odom_at >= 2.0:
            return False, '최신 LiDAR와 오도메트리 수신이 필요합니다.'
        if self.navigation_command_publishers and not self.cancel_available:
            return False, '자동 주행 취소 서비스가 준비되지 않았습니다.'
        return True, '조작 중 수동 우선, 놓으면 기존 임무를 이어갑니다.'

    def manual_snapshot(self, owner=None):
        available, reason = self.manual_readiness()
        return self.manual.snapshot(owner, available, reason)

    def manual_command(self, owner, payload):
        with self.lock:
            if payload.get('active') is True:
                available, reason = self.manual_readiness()
                if not available:
                    raise MissionError(reason)
            result = self.manual.request(owner, payload)
            if result == 'acquire':
                if self.localization.active:
                    self.cancel_localization('조이스틱 개입으로 위치 확인을 중단했습니다.')
                self.manual.resume_mission = (self.manual.resume_mission or self.mission.state in ACTIVE_STATES)
                if self.mission.state in ACTIVE_STATES:
                    self.mission.pause()
                self.manual.reason = '자동 속도를 차단하고 조종권을 확보하고 있습니다.'
                self.manual_cancel_requested = True
                self.manual_auto_ready = False
                self.mission.event('조이스틱 개입: 목표·방문 순서·완료 기록을 유지하고 수동 조종으로 전환합니다.')
            elif result == 'release':
                self.manual_cancel_requested = True
            return {'manual': self.manual_snapshot(owner)}

    def localization_readiness(self):
        available, reason = self.manual_readiness()
        if not available:
            return False, reason
        if self.manual.in_progress:
            return False, '조이스틱을 놓고 조종권 전환이 끝날 때까지 기다리세요.'
        if self.map is None or self.map_is_preview or self.map_matches_configured is False:
            return False, '선택한 저장 지도의 /map 수신이 필요합니다.'
        if not self.localization_services_ready or time.monotonic()-self.localization_amcl_active_at > 3.:
            return False, 'AMCL 활성 상태와 위치 재추정 서비스를 기다립니다.'
        if (time.monotonic()-self.localization_scan_at > .8
                or time.monotonic()-self.localization_odom_at > .8 or not self.localization_clear):
            return False, '360도 라이다와 차체 중심 반경 30cm 이상의 회전 공간을 확인하세요.'
        return True, '주변 회전 공간을 확보하세요. 라이다와 저장 지도를 대조해 위치를 추정합니다.'

    def startup_home_readiness(self):
        """A Home prior is permitted only while the robot is still at registered Home.

        The prior is not a verified pose; LiDAR/map agreement and AMCL stability
        must still pass the ordinary assisted-localization checks after rotation.
        A fresh marker 0 corroborates Home, but its absence does not block LiDAR.
        """
        available, reason = self.localization_readiness()
        if not available:
            return False, reason
        if self.mission.state not in START_STATES or self.delivery.active or self.navigation_active:
            return False, '임무를 종료하고 홈에서 정지한 뒤 자세를 초기화하세요.'
        if self.cancel_pending or abs(self.speed['linear']) >= .025 or abs(self.speed['angular']) >= .06:
            return False, '차량이 완전히 정지할 때까지 기다리세요.'
        if self.delivery.homepoint != self.delivery.registered_homepoint:
            return False, '출발 Homepoint가 등록 홈과 다릅니다. 등록 홈으로 복원한 뒤 자세를 초기화하세요.'
        # The camera is corroborating evidence, not a dependency for LiDAR
        # localization. A freshly visible *other* marker contradicts Home.
        if (time.monotonic()-self.camera_at < 1.0 and self.detected_ids
                and 0 not in self.detected_ids):
            return False, '후방 카메라에 홈이 아닌 ArUco 마커가 보입니다. 실제 위치를 확인하세요.'
        home = self.delivery.registered_homepoint
        error = self.delivery.home_validation_error(home)
        if error:
            return False, error
        available, reason = self.home_prior_geometry_readiness()
        if not available:
            return False, reason
        if time.monotonic()-self.camera_at < 1.0 and 0 in self.detected_ids:
            return True, '홈 ArUco 0번이 보입니다. 실제 등록 홈에 놓였는지 확인하고 자세를 초기화하세요.'
        return True, '마커 확인 없이 등록 홈을 초기 위치로 가정합니다. 실제로 홈에 놓였을 때만 누르세요.'

    def localization_snapshot(self):
        available, reason = self.localization_readiness()
        result = self.localization.snapshot()
        startup_available, startup_reason = self.startup_home_readiness()
        result.update(available=available, clearance=self.localization_clearance, scan_match=self.localization_match,
                      startup_home_available=startup_available, startup_home_reason=startup_reason)
        result['control'] = deepcopy(self.localization_control)
        seed_available, seed_reason = self.seed_home_readiness()
        result.update(home_seed_available=seed_available, home_seed_reason=seed_reason,
                      home_seed_pending=self.home_seed_request is not None,
                      home_seed_result=self.home_seed_result,
                      home_seed_match=self.home_seed_match,
                      home_prior_evidence=deepcopy(self.home_prior_evidence))
        pose_available, pose_reason = self.manual_pose_readiness()
        result['manual_pose'] = {
            'available': pose_available, 'reason': pose_reason,
            'pending': self.manual_pose_request is not None or self.manual_pose_state == 'WAITING',
            'result': self.manual_pose_result,
            'requested_pose': deepcopy(self.manual_pose_requested),
            'scan_match': self.manual_pose_match,
            'state': self.manual_pose_state,
            'verified': (self.manual_pose_state == 'VERIFIED' and self.connected
                         and self.map is not None and not self.map_is_preview
                         and self.map_matches_configured is True
                         and self.manual_pose_map_version == self.map_version
                         and not self.localization_invalidated
                         and self.pose is not None and self.pose_tf_stamp_ns > 0
                         and time.monotonic()-self.pose_at < 2.
                         and time.monotonic()-self.scan_at < 2.
                         and time.monotonic()-self.odom_at < 2.
                         and time.monotonic()-self.localization_amcl_active_at < 3.),
        }
        use_existing_pose = (self.can_refine_localization() if self.localization.state == 'IDLE'
                             else self.localization_use_existing_pose)
        mode = 'home' if self.localization_startup_home else 'local' if use_existing_pose else 'global'
        result.update(mode=mode,
                      mode_label={'home': '등록 홈 기준 자세 초기화', 'local': '현재 위치 보정',
                                  'global': '지도 전체 위치 탐색'}[mode],
                      use_existing_pose=use_existing_pose)
        estimate = result.get('estimate') or {}
        result['rotated_degrees'] = result.get('rotation_degrees', 0.)
        result['angular_velocity'] = result.get('angular', 0.)
        result['position_std'] = estimate.get('position_std')
        result['yaw_std_degrees'] = math.degrees(estimate['yaw_std']) if 'yaw_std' in estimate else None
        if not self.localization.active and self.localization.state == 'IDLE':
            result['reason'] = reason
        return result

    def home_seed_key(self):
        home = self.delivery.registered_homepoint
        return (self.map_version, None if home is None else
                tuple(home.get(name) for name in ('x', 'y', 'yaw')))

    def home_prior_geometry_readiness(self):
        """Verify the known Home against real sensors without a map->base TF.

        This is a prior, not an AMCL confidence shortcut. Both the stopped
        ESTOP seed and ordinary Home initialization use the same proof.
        """
        now = time.monotonic()
        if (self.map is None or self.map_is_preview or self.map_matches_configured is not True
                or self._display_map_data is None):
            return False, '설정한 저장 지도와 일치하는 /map 수신이 필요합니다.'
        if (now-self.scan_at >= .8 or now-self.odom_at >= .8
                or now-self.localization_scan_at >= .8 or now-self.localization_odom_at >= .8):
            return False, '최신 LiDAR와 오도메트리 수신이 필요합니다.'
        speeds = (self.speed.get('linear'), self.speed.get('angular'))
        if (any(type(value) not in (int, float) or not math.isfinite(value) for value in speeds)
                or abs(speeds[0]) >= .01 or abs(speeds[1]) >= .03):
            return False, '차량이 완전히 정지해야 등록 홈 초기 위치를 입력할 수 있습니다.'
        error = self.delivery.home_validation_error(self.delivery.registered_homepoint)
        if error:
            return False, error
        exact_match = (self.home_seed_match is not None and math.isfinite(self.home_seed_match)
                       and self.home_seed_match >= .65 and 0 <= now-self.home_seed_match_at < .8
                       and self.home_seed_match_key == self.home_seed_key())
        evidence = self.home_prior_evidence or {}
        nearby_match = (evidence.get('supported') is True
                        and evidence.get('distinct_scans', 0) >= 3
                        and evidence.get('consistent_candidates', 0) >= 3
                        and evidence.get('best_min_agreement', 0.) is not None
                        and evidence.get('best_min_agreement', 0.) >= .65
                        and 0 <= now-evidence.get('at', 0.) < .8
                        and evidence.get('key') == self.home_seed_key())
        if not exact_match and not nearby_match:
            return False, '등록 홈 주변의 LiDAR와 지도가 일치하지 않습니다. 실제 위치를 확인하세요.'
        # Nearby evidence validates only the existing broad Home prior. It
        # never chooses a pose, changes Home, or certifies AMCL confidence.
        return True, ''

    def seed_home_readiness(self):
        """No-motion pose prior while ESTOP remains latched, never a reset."""
        now = time.monotonic()
        if (not self.mission.estop_latched or now-self.manual_mux_at >= 1.
                or self.manual_mux_status.get('estop_latched') is not True
                or self.manual_mux_status.get('stop_acknowledged') is not True):
            return False, '차량의 비상정지 확인 응답이 필요합니다.'
        if not self.connected or self.pending_connection is not None:
            return False, 'ROS 연결을 확인하세요.'
        if (self.map is None or self.map_is_preview or self.map_matches_configured is not True
                or self._display_map_data is None):
            return False, '설정한 저장 지도와 일치하는 /map 수신이 필요합니다.'
        if (self.localization.active or self.manual.in_progress or self.navigation_active
                or self.manual_pose_state == 'WAITING' or self.cancel_pending):
            return False, '회전·조종·주행 취소가 모두 끝날 때까지 기다리세요.'
        if (now-self.scan_at >= .8 or now-self.odom_at >= .8
                or now-self.localization_scan_at >= .8 or now-self.localization_odom_at >= .8):
            return False, '최신 LiDAR와 오도메트리 수신이 필요합니다.'
        speeds = (self.speed.get('linear'), self.speed.get('angular'))
        if (any(not isinstance(value, (float, int)) or not math.isfinite(value) for value in speeds)
                or abs(speeds[0]) >= .01 or abs(speeds[1]) >= .03):
            return False, '차량이 완전히 정지해야 초기 위치만 입력할 수 있습니다.'
        if (not self.localization_services_ready or now-self.localization_amcl_active_at >= 3.):
            return False, 'AMCL 활성 상태를 확인하세요.'
        available, reason = self.home_prior_geometry_readiness()
        if not available:
            return False, reason
        return True, '실제 등록 홈 위치·방향이 맞으면 회전 없이 초기 가설만 입력할 수 있습니다.'

    def seed_home_command(self, payload):
        if payload.get('confirm_actual_home') is not True:
            raise MissionError('실제 등록 홈 위치와 방향을 확인한 뒤 확인란을 선택하세요.')
        if self.home_seed_request is not None:
            raise MissionError('등록 홈 초기 위치 입력을 처리 중입니다.')
        available, reason = self.seed_home_readiness()
        if not available:
            raise MissionError(reason)
        self.home_seed_request = {'home': deepcopy(self.delivery.registered_homepoint),
                                  'at': time.monotonic(), 'key': self.home_seed_key()}
        self.home_seed_result = '초기 위치 입력 검증 중 · 비상정지 유지'
        self.mission.event('운용자가 등록 홈 위치·방향을 확인했습니다. 정지 상태에서 초기 가설만 입력합니다.')
        return self.state()

    def manual_pose_readiness(self, *, require_stop_ack=True):
        """Manual map pose is only a prior while the robot is stopped in ESTOP."""
        now = time.monotonic()
        if not self.mission.estop_latched:
            return False, '차량 비상정지와 정지 확인 응답이 필요합니다.'
        if (require_stop_ack and (now-self.manual_mux_at >= 1.
                or self.manual_mux_status.get('estop_latched') is not True
                or self.manual_mux_status.get('stop_acknowledged') is not True)):
            return False, '차량 비상정지와 정지 확인 응답이 필요합니다.'
        if not self.connected or self.pending_connection is not None:
            return False, 'ROS 연결을 확인하세요.'
        if (self.map is None or self.map_is_preview or self.map_matches_configured is not True
                or self._display_map_data is None or self.localization_distance is None):
            return False, '설정한 저장 지도와 일치하는 실제 /map 수신이 필요합니다.'
        if (self.localization.active or self.localization_requested or self.localization_stop_requested
                or self.localization_service_pending or self.manual.in_progress or self.navigation_active
                or self.cancel_pending or self.delivery.active or self.home_seed_request is not None
                or any(kind in {'goal', 'cancel'} for kind, _ in self.mission.commands)):
            return False, '회전·조종·주행·주행 취소가 모두 끝날 때까지 기다리세요.'
        if (now-self.scan_at >= .8 or now-self.odom_at >= .8
                or now-self.localization_scan_at >= .8 or now-self.localization_odom_at >= .8):
            return False, '최신 LiDAR와 오도메트리 수신이 필요합니다.'
        speeds = (self.speed.get('linear'), self.speed.get('angular'))
        if (any(type(value) not in (float, int) or not math.isfinite(value) for value in speeds)
                or abs(speeds[0]) >= .01 or abs(speeds[1]) >= .03):
            return False, '차량이 완전히 정지해야 초기 위치를 입력할 수 있습니다.'
        if not self.manual_pose_service_ready:
            return False, 'AMCL 초기 위치 서비스를 기다립니다.'
        if now-self.localization_amcl_active_at >= 3.:
            return False, 'AMCL 활성 상태를 기다립니다.'
        return True, '실제 차량 위치와 방향을 지도 좌표로 입력하세요. 입력 후에도 비상정지를 유지합니다.'

    def manual_pose_geometry_error(self, pose):
        """Require known free space around the full Waffle Pi footprint.

        Grid coordinates identify cells, not point obstacles. Check distance to
        each unknown/occupied *cell rectangle* plus half a cell diagonal; a
        center-only distance transform can miss an unknown cell beside the base.
        """
        try:
            metadata = self._display_map_metadata
            data = self._display_map_data
            origin = metadata['origin']
            resolution = float(metadata['resolution'])
            width, height = int(metadata['width']), int(metadata['height'])
            if (not math.isfinite(resolution) or resolution <= 0. or width <= 0 or height <= 0
                    or data is None or data.shape != (height, width)):
                raise ValueError('invalid grid')
            dx, dy = pose['x']-float(origin['x']), pose['y']-float(origin['y'])
            c, s = math.cos(float(origin['yaw'])), math.sin(float(origin['yaw']))
            local_x, local_y = c*dx+s*dy, -s*dx+c*dy
            column, row = math.floor(local_x/resolution), math.floor(local_y/resolution)
            if not (0 <= row < height and 0 <= column < width):
                return '초기 위치가 운용 지도 범위 밖입니다.'
            if data[row, column] < 0:
                return '초기 위치가 지도에서 확인되지 않은 영역입니다.'
            if data[row, column] >= 65:
                return '초기 위치가 지도상의 장애물과 겹칩니다.'
            required = .22+resolution/math.sqrt(2.)
            if min(local_x, local_y, width*resolution-local_x, height*resolution-local_y) <= required:
                return '초기 위치의 차체가 지도 밖과 겹칩니다. 지도 안쪽 위치를 입력하세요.'
            min_column = max(0, math.floor((local_x-required)/resolution)-1)
            max_column = min(width-1, math.floor((local_x+required)/resolution)+1)
            min_row = max(0, math.floor((local_y-required)/resolution)-1)
            max_row = min(height-1, math.floor((local_y+required)/resolution)+1)
            for nearby_row in range(min_row, max_row+1):
                for nearby_column in range(min_column, max_column+1):
                    occupancy = data[nearby_row, nearby_column]
                    if 0 <= occupancy < 65:
                        continue
                    nearest_x = min(max(local_x, nearby_column*resolution),
                                    (nearby_column+1)*resolution)
                    nearest_y = min(max(local_y, nearby_row*resolution),
                                    (nearby_row+1)*resolution)
                    if math.hypot(nearest_x-local_x, nearest_y-local_y) <= required:
                        return ('초기 위치의 차체가 미확인 영역과 겹칩니다.' if occupancy < 0 else
                                '초기 위치의 차체가 지도 장애물과 겹칩니다.')
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError, ZeroDivisionError):
            return '초기 위치를 확인할 지도 데이터가 올바르지 않습니다.'
        return ''

    def manual_pose_command(self, payload):
        if payload.get('confirm') is not True:
            raise MissionError('차량의 실제 지도 위치와 방향을 확인한 뒤 확인란을 선택하세요.')
        if self.manual_pose_request is not None or self.manual_pose_state == 'WAITING':
            raise MissionError('이전 초기 위치 입력의 AMCL 응답을 기다리고 있습니다.')
        available, reason = self.manual_pose_readiness()
        if not available:
            raise MissionError(reason)
        values = (payload.get('x'), payload.get('y'), payload.get('yaw_deg'))
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise MissionError('x, y, 방향각에는 유한한 숫자를 입력하세요.')
        x, y, yaw_deg = (float(value) for value in values)
        if not -180. <= yaw_deg <= 180.:
            raise MissionError('방향각은 -180~180도 범위로 입력하세요.')
        pose = {'x': x, 'y': y, 'yaw': math.radians(yaw_deg)}
        error = self.manual_pose_geometry_error(pose)
        if error:
            raise MissionError(error)
        self.manual_pose_request = {'pose': pose, 'at': time.monotonic(), 'map_version': self.map_version}
        self.manual_pose_map_version = self.map_version
        self.manual_pose_requested = {'x': x, 'y': y, 'yaw_deg': yaw_deg}
        self.manual_pose_state = 'WAITING'
        self.manual_pose_result = 'AMCL에 초기 위치를 입력하는 중입니다. 비상정지를 유지합니다.'
        self.manual_pose_amcl_at = self.manual_pose_sent_at = 0.
        self.manual_pose_amcl_received_at = 0.
        self.manual_pose_match = None
        self.mission.event(f'운용자가 지도 초기 위치 x={x:.2f}, y={y:.2f}, 방향={yaw_deg:.1f}°를 지정했습니다. 정지 상태에서 입력합니다.')
        return self.state()

    def cancel_localization(self, reason):
        if self.localization.active:
            self.localization.cancel(reason)
            self.localization_control = {'state': 'CANCELED', 'reason': reason}
            self.localization_requested = False
            self.localization_stop_requested = True
            self.mission.event(reason, 'warning')

    def can_refine_localization(self):
        """A known free, fresh map pose can be checked without erasing AMCL.

        A visual floorplan or an unverified preview cannot establish occupancy.
        The same real raster validation protects Homepoint changes.
        """
        return (not self.localization_invalidated and self.pose is not None
                and time.monotonic()-self.pose_at < 2.0
                and self.map is not None and not self.map_is_preview
                and self.map_matches_configured is not False
                and self._display_map_data is not None
                and not self.delivery.home_validation_error(self.pose))

    def localization_command(self, endpoint, payload=None):
        if endpoint.endswith('/stop'):
            self.cancel_localization('사용자가 위치 확인을 중단했습니다. 차량은 정지 상태를 유지합니다.')
            return self.state()
        if self.manual_pose_state == 'WAITING':
            raise MissionError('초기 위치 입력 검증이 끝난 뒤 라이다 위치 확인을 시작하세요.')
        if self.localization.active:
            raise MissionError('이미 위치 확인 중입니다.')
        if self.localization_service_pending:
            raise MissionError('이전 AMCL 초기화 서비스 응답을 기다리고 있습니다.')
        available, reason = self.localization_readiness()
        if not available:
            raise MissionError(reason)
        startup_home = endpoint.endswith('/start-home')
        if startup_home:
            if (payload or {}).get('confirm_actual_home') is not True:
                raise MissionError('실제 등록 홈 위치와 방향을 확인한 뒤 확인란을 선택하세요.')
            allowed, reason = self.startup_home_readiness()
            if not allowed:
                raise MissionError(reason)
        if self.mission.state in ACTIVE_STATES:
            self.mission.pause()
        self.localization_startup_home = startup_home
        self.localization_home_seed = (deepcopy(self.delivery.registered_homepoint)
                                       if startup_home else None)
        self.localization_use_existing_pose = not startup_home and self.can_refine_localization()
        self.localization_reference_pose = deepcopy(self.pose) if self.localization_use_existing_pose else None
        self.localization_reference_odom = deepcopy(self.odom_pose) if self.localization_use_existing_pose else None
        self.localization.start(home_target=self.localization_home_seed)
        self.localization_control = {'state': 'WAIT_HOLD', 'reason':
                                     '주행을 멈추고 새 정지 응답을 확인합니다.'}
        self.localization_requested = True
        self.localization_stop_requested = False
        self.localization_match = None
        self.mission.event('자세 초기화: 등록 홈을 초기 가설로 입력하고, 라이다·지도 검증 후 홈 방향으로 복귀합니다.'
                           if startup_home else
                           '라이다 위치 보정: 현재 AMCL 위치를 유지하고 정지 후 주변 지도를 대조합니다.'
                           if self.localization_use_existing_pose else
                           '라이다 위치 확인: 유효한 기존 위치가 없어 정지 후 지도 전체에서 위치를 다시 추정합니다.')
        return self.state()

    def reset_power(self):
        """A different ROS connection must never inherit the old vehicle's voltage."""
        with self.lock:
            self.battery_at = None
            self.battery_voltage = self.battery_percentage = None

    def update_battery(self, voltage, percentage=None):
        """Store measured values only; no voltage-to-charge or power-off inference."""
        def finite(value):
            try:
                number = float(value) if not isinstance(value, bool) else math.nan
            except (TypeError, ValueError, OverflowError):
                return None
            return number if math.isfinite(number) else None
        voltage, percentage = finite(voltage), finite(percentage)
        with self.lock:
            self.battery_voltage = voltage if voltage is not None and voltage > 0 else None
            self.battery_percentage = (percentage if percentage is not None and 0 <= percentage <= 1
                                       else None)
            self.battery_at = time.monotonic()

    def power_state(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            age = None if self.battery_at is None else max(0., now-self.battery_at)
            if age is None:
                status = 'waiting'
            elif age >= 5.0:
                status = 'stale'
            else:
                status = 'online' if self.battery_voltage is not None else 'invalid'
            return {'status': status, 'voltage': self.battery_voltage,
                    'percentage': self.battery_percentage, 'age': None if age is None else round(age, 2),
                    'source': self.battery_topic}

    def inventory_identity(self, cookie_header):
        try:
            cookie = cookies.SimpleCookie(cookie_header or '').get('amr_inventory_session')
            token = cookie.value if cookie else ''
        except cookies.CookieError:
            return None
        with self.lock:
            session = self.inventory_sessions.get(token)
            return (session['user_id'] if session and session['expires'] > time.monotonic()
                    and (self.access_mode != 'domain' or session.get('domain_id') == self.domain_id)
                    else None)

    def _check_access(self, code, domain_id, *, inventory=False):
        if self.access_mode == 'domain':
            if type(domain_id) is not int or not 0 <= domain_id <= 232:
                raise MissionError('ROS Domain ID는 0~232의 정수로 입력하세요.')
            if domain_id != self.domain_id:
                raise MissionError('차량의 ROS Domain ID와 일치하지 않습니다.')
        else:
            expected = self.inventory_join_code if inventory else self.pairing_code
            if not secrets.compare_digest(str(code), expected):
                label = '재고 관리자 연결 코드' if inventory else '연결 코드'
                raise MissionError(label+'가 일치하지 않습니다.')

    def pair_inventory(self, client, user_id, code='', *, domain_id=None):
        if type(user_id) is not int or not 1 <= user_id <= 99:
            raise MissionError('재고 관리자 번호는 1~99입니다. 운용자는 별도 화면을 사용하세요.')
        with self.lock:
            now = time.monotonic()
            attempts = deque(t for t in self.inventory_attempts.get(client, []) if now-t < 60)
            if len(attempts) >= 5:
                raise MissionError('인증 시도가 너무 많습니다. 1분 뒤 다시 시도하세요.')
            attempts.append(now)
            self.inventory_attempts[client] = attempts
            self._check_access(code, domain_id, inventory=True)
            token = secrets.token_urlsafe(32)
            self.inventory_sessions = {k: v for k, v in self.inventory_sessions.items() if v['expires'] > now}
            if len(self.inventory_sessions) >= 128:
                self.inventory_sessions.pop(next(iter(self.inventory_sessions)))
            self.inventory_sessions[token] = {'user_id': user_id, 'expires': now+86400,
                                              'domain_id': self.domain_id}
            return token

    def inventory_state(self, user_id):
        with self.lock:
            data = self.delivery.snapshot()
            active_trip = data['active_trip']
            referenced = (set(active_trip['request_ids']) if active_trip is not None else set())
            own_requests = [item for item in data['requests'] if item['user_id'] == user_id]
            finished = [item for item in data['requests']
                        if item['status'] in {'DELIVERED', 'CANCELED'}]
            deferred_ids = set(data['deferred_history_ids'])
            deferred_ids.update(item['id'] for item in finished if item['id'] in referenced)
            for item in own_requests:
                item['deletion_pending'] = item['id'] in data['deferred_history_ids']
            deferred_count = len(deferred_ids)
            return {'user_id': user_id, 'markers': deepcopy(self.markers),
                    'requests': own_requests,
                    'service': {'state': self.mission.state, 'home_ready': data['home_ready'],
                                'active': data['active'], 'power': self.power_state(),
                                'clearable_history_count': len(finished)-deferred_count,
                                'deferred_history_count': deferred_count,
                                'clear_pending': bool(data['deferred_history_ids'])}}

    def _history_clear_home_return(self):
        """Return home after a history reset without treating stale pose as arrival.

        The reset is a durable operation of its own. A failed or unavailable
        home command is reported in the response instead of hiding the fact
        that completed records were removed.
        """
        mission = self.mission

        def result(status, message, cleared_count=0):
            return {'status': status, 'message': message,
                    'cleared_count': cleared_count}

        def unavailable(message):
            # A reset must not leave an existing autonomous goal moving when
            # the sensors needed for a safe home return are unavailable.
            if mission.state in ACTIVE_STATES:
                mission.pause()
                mission.reason = '홈 복귀를 보류하고 주행을 일시정지했습니다: '+message
                mission.event(mission.reason, 'warning')
            elif self.navigation_active or any(kind == 'goal' for kind, _ in mission.commands):
                # Also cover an orphaned ROS goal when the web mission state
                # has already become IDLE or PAUSED.
                mission.commands = [('cancel', None), ('stop', None)]
                mission.reason = '기존 주행 정지를 요청했습니다. 홈 복귀는 보류했습니다: '+message
                mission.event(mission.reason, 'warning')
            return result('unavailable', message)

        if self.pending_connection is not None:
            return result('blocked', 'ROS 연결을 변경하는 중입니다. 연결 완료 후 홈 복귀를 다시 요청하세요.')
        if self.localization.active:
            self.cancel_localization('배송기록 초기화로 라이다 위치 확인을 중단했습니다. 차량은 정지합니다.')
            return result('unavailable', '라이다 위치 확인을 중단했습니다. 정지 확인 후 홈 복귀를 다시 요청하세요.')
        if self.manual.in_progress:
            return result('blocked', '조이스틱 조종 중입니다. 조종기를 놓은 뒤 홈 복귀를 요청하세요.')
        if mission.estop_latched:
            return result('blocked', '비상정지 중에는 자동 홈 복귀를 시작하지 않습니다.')
        error = self.delivery.home_validation_error(self.delivery.homepoint)
        if error:
            return unavailable(error)
        if not self.connected:
            return unavailable('ROS 연결이 없어 홈 위치를 확인하거나 복귀 명령을 보낼 수 없습니다.')
        health = self.readiness(for_home=True)
        if not health['ready']:
            return unavailable('홈 복귀에 필요한 상태를 확인하세요: '+', '.join(health['missing']))

        needs_cancel = (self.delivery.active or mission.state in ACTIVE_STATES | {'PAUSED'}
                        or self.navigation_active or any(kind == 'goal' for kind, _ in mission.commands))
        if self.delivery.at_home():
            if needs_cancel:
                try:
                    # Preserve unfinished requests by re-queuing them; only
                    # close the trip after independently verified home pose.
                    self.delivery.cancel()
                    if self.delivery.active:
                        self.delivery.queue.close_trip()
                    mission.home = deepcopy(self.delivery.homepoint)
                    mission.stop_at_home(self.pose)
                except (MissionError, OSError, ValueError, IndexError) as exc:
                    return unavailable('기존 주행 종료를 확인하지 못했습니다: '+str(exc))
            try:
                cleared_count = self.delivery.queue.clear_deferred_history_at_home()
            except (OSError, ValueError) as exc:
                return unavailable('예약된 배송 기록을 정리하지 못했습니다: '+str(exc))
            return result('already_home', '이미 홈포인트입니다.', cleared_count)

        try:
            # This is the same cancel → stop → fixed-home goal path as the
            # operator's Home button, including active-trip re-queueing.
            self.command('/api/mission/home', {})
        except (MissionError, OSError, ValueError, IndexError) as exc:
            return unavailable('홈 복귀 명령을 시작하지 못했습니다: '+str(exc))
        return result('requested', '차량에 홈포인트 복귀를 요청했습니다. 도착 여부는 주행 상태에서 확인하세요.')

    def inventory_command(self, endpoint, payload, user_id):
        with self.lock:
            if endpoint == '/api/inventory/requests':
                self.delivery.submit(user_id, payload)
            elif endpoint == '/api/inventory/requests/cancel':
                self.delivery.queue.cancel_pending(user_id, payload.get('id'))
            elif endpoint == '/api/inventory/requests/delete':
                deletion = self.delivery.queue.delete_finished_history(user_id, payload.get('id'))
                return dict(self.inventory_state(user_id), deletion=deletion)
            elif endpoint == '/api/inventory/history/clear':
                removed_count = self.delivery.queue.clear_finished_history(defer_active_trip=True)
                home_return = self._history_clear_home_return()
                removed_count += home_return['cleared_count']
                state = self.inventory_state(user_id)
                return dict(state, removed_count=removed_count,
                            deferred_count=len(self.delivery.queue.snapshot()['deferred_history_ids']),
                            home_return=home_return)
            else:
                raise KeyError(endpoint)
            return self.inventory_state(user_id)

    def authenticated(self, cookie_header, local=False):
        if local:
            return True
        try:
            token = cookies.SimpleCookie(cookie_header or '').get('amr_session')
            token = token.value if token else ''
        except cookies.CookieError:
            return False
        with self.lock:
            expiration = self.sessions.get(token, 0)
            return (expiration > time.monotonic()
                    and (self.access_mode != 'domain' or self.session_domains.get(token) == self.domain_id))

    def pair(self, client, code='', *, domain_id=None):
        with self.lock:
            now = time.monotonic()
            attempts = deque(t for t in self.pair_attempts.get(client, []) if now-t < 60)
            if len(attempts) >= 5:
                raise MissionError('인증 시도가 너무 많습니다. 1분 뒤 다시 시도하세요.')
            attempts.append(now)
            self.pair_attempts[client] = attempts
            self._check_access(code, domain_id)
            token = secrets.token_urlsafe(32)
            self.sessions = {k: v for k, v in self.sessions.items() if v > now}
            if len(self.sessions) >= 32:
                self.sessions.pop(next(iter(self.sessions)))
            self.session_domains = {key: domain for key, domain in self.session_domains.items()
                                    if key in self.sessions}
            self.sessions[token] = now+86400
            self.session_domains[token] = self.domain_id
            return token


def image_to_bgr(msg):
    """Decode raw Image or image_transport JPEG without changing camera timing."""
    if not hasattr(msg, 'encoding'):
        # CompressedImage has format/data, while Image has encoding/step/data.
        if not hasattr(msg, 'format') or not msg.data:
            raise ValueError('압축 카메라 영상이 비어 있습니다.')
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('압축 카메라 영상을 디코딩할 수 없습니다.')
        return np.ascontiguousarray(image)
    # Honor Image.step row padding instead of assuming tightly packed RGB.
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}
    encoding = msg.encoding.lower()
    if encoding not in channels:
        raise ValueError(f'지원하지 않는 encoding: {msg.encoding}')
    channel_count = channels[encoding]
    row_bytes = int(msg.width)*channel_count
    if msg.height <= 0 or msg.width <= 0 or msg.step < row_bytes or len(msg.data) < msg.step*msg.height:
        raise ValueError('카메라 Image의 크기/step/data가 올바르지 않습니다.')
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=msg.step*msg.height).reshape(msg.height, msg.step)
    image = rows[:, :row_bytes].reshape(msg.height, msg.width, channel_count)
    if encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif encoding == 'rgba8':
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    elif encoding == 'bgra8':
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    elif encoding == 'mono8':
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(image)


def create_ros_bridge(app, options, context):
    """Import ROS lazily so pure mission/HTTP tests need no ROS installation."""
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as ROSPath
    from nav2_msgs.srv import SetInitialPose
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, CompressedImage, Image, LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, Empty
    from tf2_ros import Buffer, TransformListener, TransformException

    class ROSMissionBridge(Node):
        def __init__(self):
            super().__init__('web_mission_server', context=context,
                             parameter_overrides=[Parameter('use_sim_time', value=app.use_sim_time)])
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.goal_pub = self.create_publisher(PoseStamped, '/smart_vision_amr/goal', 10)
            self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            # Control packets carry a sequence and expire after one second at
            # the mux. A delayed reliable queue cannot make them useful; send
            # only the newest packet and rely on the exact ACK/retry handshake.
            control_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.motion_pub = self.create_publisher(String, '/smart_vision_amr/manual_control', control_qos)
            self.motion_source = secrets.token_urlsafe(18)
            self.motion_seq = 0
            self.localization_hold_seq = self.localization_lease_seq = None
            self.localization_lease_at = None
            self.last_motion_status_stamp_ns = self.last_health_stamp_ns = -1
            self.last_planner_status_stamp_ns = -1
            self.motion_gate = False
            self.sent_estop = False
            # A goal must not reach the planner until this bridge has an
            # accepted, fresh auto-mode receipt from the robot-side mux.
            self.auto_handover_started = None
            self.auto_handover_last_sent = None
            self.auto_handover_sent = {}
            self.auto_handover_first_seq = None
            self.auto_handover_timeout = 8.0
            self.auto_handover_retry = .4
            # A latched stop needs an immediate packet and a slow heartbeat,
            # not a fresh control packet on every 10 Hz bridge tick. Repeated
            # packets otherwise contend with the robot's sensor callbacks.
            self.last_estop_control_at = -math.inf
            self.last_hold_control_at = -math.inf
            self.last_control_mode = None
            self.last_control_operation = None
            self.cancel_client = self.create_client(Trigger, '/smart_vision_amr/cancel_navigation')
            self.estop_client = self.create_client(Trigger, '/smart_vision_amr/emergency_stop')
            self.reset_client = self.create_client(Trigger, '/smart_vision_amr/reset_emergency_stop')
            self.safety_future = None
            self.safety_kind = None
            self.safety_started = 0.
            self.global_localization_client = self.create_client(Empty, '/reinitialize_global_localization')
            self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
            self.amcl_state_client = self.create_client(GetState, '/amcl/get_state')
            self.initial_pose_client = self.create_client(SetInitialPose, '/set_initial_pose')
            self.amcl_state_future = self.localization_future = self.nomotion_future = None
            self.manual_pose_future = None
            self.manual_pose_nomotion_future = None
            self.manual_pose_nomotion_at = 0.
            self.manual_pose_ros_stamp_ns = 0
            self.manual_pose_last_stop_ack_at = None
            self.amcl_state_checked_at = self.nomotion_at = 0.
            self.localization_epoch_ns = self.last_amcl_stamp_ns = 0
            self.manual_initial_pose_ns = None
            self.startup_seed_stamps = deque(maxlen=5)
            self.localization_scan = None
            self.localization_laser_pose = None
            self.home_prior_observer = HomePriorObserver(home_neighborhood_evidence)
            self.localization_angular = 0.
            self.localization_running = False
            self.localization_scan_gap_since = None
            self.localization_scan_gap_reason = None
            self.localization_sensor_gap_since = None
            self.localization_sensor_gap_reason = None
            self.localization_policy_gap_since = None
            self.localization_policy_gap_reason = None
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, durable)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
            self.create_subscription(ROSPath, '/smart_vision_amr/path', self.on_path, 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            rear_type = CompressedImage if options.camera_topic.endswith('/compressed') else Image
            self.create_subscription(rear_type, options.camera_topic, self.on_image, qos_profile_sensor_data)
            front_topic = getattr(options, 'front_camera_topic', '/camera/front/image_raw')
            if front_topic and front_topic != options.camera_topic:
                front_type = CompressedImage if front_topic.endswith('/compressed') else Image
                self.create_subscription(front_type, front_topic, self.on_front_image, qos_profile_sensor_data)
            app.battery_topic = getattr(options, 'battery_topic', '/battery_state')
            self.last_battery_stamp_ns = None
            self.create_subscription(BatteryState, app.battery_topic, self.on_battery, qos_profile_sensor_data)
            self.create_subscription(String, '/smart_vision_amr/navigation_status', self.on_navigation, 10)
            self.create_subscription(String, '/smart_vision_amr/planner_status', self.on_planner, durable)
            self.create_subscription(String, '/smart_vision_amr/motion_status', self.on_motion, durable)
            self.create_subscription(String, '/smart_vision_amr/localization_health', self.on_health, durable)
            self.create_subscription(String, '/smart_vision_amr/failure_memory_status',
                                     self.on_failure_memory, durable)
            # A wall/steady timer keeps the joystick dead-man alive even if /clock pauses.
            from rclpy.clock import Clock, ClockType
            self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.cancel_future = None
            self.cancel_required = False
            self.cancel_sent_at = 0.0
            self.cancel_completed_at = 0.0
            self.goal_waiting = None
            self.stop_until = 0.0
            self.last_goal_stamp = 0
            self.owns_navigation = False
            self.last_camera_processed = 0.0
            self.last_front_camera_processed = 0.0
            self.last_image_error = ''
            self.camera_rotation = getattr(options, 'camera_rotation', 0)
            self.front_camera_rotation = getattr(options, 'front_camera_rotation', 0)
            self.observation_started_ros_ns = None
            self.marker_camera_poses = {
                'front': tuple(getattr(options, 'marker_front_camera_pose', (0., 0., 0.))),
                'rear': tuple(getattr(options, 'marker_rear_camera_pose', (0., 0., math.pi)))}
            if any(len(pose) != 3 or not all(math.isfinite(value) for value in pose)
                   for pose in self.marker_camera_poses.values()):
                raise ValueError('Finite camera mounting x, y and yaw are required')
            self.marker_alignment = MarkerAlignment(
                search_speed=getattr(options, 'marker_search_speed', .12),
                center_gain=getattr(options, 'marker_center_gain', .35),
                center_tolerance=getattr(options, 'marker_center_tolerance', .08),
                timeout=getattr(options, 'marker_search_timeout', 45.),
                required_frames=max(2, app.confirmations),
                require_wall_alignment=getattr(options, 'require_marker_wall_alignment', False),
                front_yaw_sign=getattr(options, 'marker_front_yaw_sign', -1.),
                rear_yaw_sign=getattr(options, 'marker_rear_yaw_sign', -1.))
            self.marker_alignment_key = None
            self.marker_alignment_angular = 0.
            if app.aruco_available:
                self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                self.detector_parameters = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
                self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters) if hasattr(cv2.aruco, 'ArucoDetector') else None
            with app.lock:
                app.connected = True
                app.connection_error = ''
                app.mission.event(f'ROS 도메인 {app.domain_id}에 연결했습니다.')

        def destroy_node(self):
            self.home_prior_observer.close()
            return super().destroy_node()

        def on_map(self, message):
            width, height = int(message.info.width), int(message.info.height)
            if width <= 0 or height <= 0 or width*height != len(message.data) or width*height > 25000000:
                return
            data = np.asarray(message.data, dtype=np.int16).reshape(height, width)
            if np.any(data < -1) or np.any(data > 100):
                return
            q = message.info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            metadata = {'width': width, 'height': height, 'resolution': message.info.resolution,
                        'origin': {'x': message.info.origin.position.x, 'y': message.info.origin.position.y, 'yaw': yaw},
                        'frame_id': message.header.frame_id or 'map'}
            if (not math.isfinite(metadata['resolution']) or metadata['resolution'] <= 0
                    or not all(math.isfinite(value) for value in metadata['origin'].values())):
                return
            app.receive_map(metadata, data)

        def on_path(self, message):
            with app.lock:
                app.path = [{'x': pose.pose.position.x, 'y': pose.pose.position.y} for pose in message.poses]

        def source_fresh(self, stamp, max_age=2.0):
            stamp_ns = stamp.sec*1000000000+stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
            return stamp_ns > 0 and -.25 <= age < max_age

        def on_odom(self, message):
            if not self.source_fresh(message.header.stamp):
                return
            with app.lock:
                app.odom_at = time.monotonic()
                app.speed = {'linear': float(message.twist.twist.linear.x), 'angular': float(message.twist.twist.angular.z)}
                q = message.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                odom_pose = {'x': float(message.pose.pose.position.x),
                             'y': float(message.pose.pose.position.y), 'yaw': yaw}
                if not all(math.isfinite(value) for value in odom_pose.values()):
                    app.odom_pose = None
                    self.home_prior_observer.invalidate()
                    app.home_prior_evidence = None
                    return
                app.odom_pose = odom_pose
                if (not self.home_prior_observer.observe_odom(app.odom_pose, app.speed)
                        or self.home_prior_observer.evidence is None):
                    app.home_prior_evidence = None
                if self.source_fresh(message.header.stamp, .8):
                    if (app.localization.active and app.localization_odom_at
                            and app.odom_at-app.localization_odom_at > .8):
                        app.localization.pause_for_sensor_gap(app.odom_at)
                    app.localization_odom_at = app.odom_at
                    app.localization.on_odom(yaw, app.odom_at)

        def on_scan(self, message):
            if not self.source_fresh(message.header.stamp):
                return
            with app.lock:
                if any(math.isfinite(value) and message.range_min <= value <= message.range_max for value in message.ranges):
                    app.scan_at = time.monotonic()
                if not self.source_fresh(message.header.stamp, .8):
                    return
                scan_now = time.monotonic()
                if (app.localization.active and app.localization_scan_at
                        and scan_now-app.localization_scan_at > .8):
                    app.localization.pause_for_sensor_gap(scan_now)
                self.localization_scan = message
                app.localization_clear = False
                # A previous safe range does not describe this scan. If the
                # laser-to-base TF is briefly unavailable, report unknown
                # clearance instead of showing a stale distance as current.
                app.localization_clearance = None
                app.localization_scan_at = scan_now
                try:
                    transform = self.tf_buffer.lookup_transform(options.base_frame, message.header.frame_id, rclpy.time.Time())
                    q = transform.transform.rotation
                    self.localization_laser_pose = (transform.transform.translation.x, transform.transform.translation.y,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                    app.localization_clear, app.localization_clearance = rotation_clearance(message, self.localization_laser_pose)
                    home = app.delivery.registered_homepoint
                    if (scan_now-app.home_seed_match_at >= .3
                            and (app.mission.estop_latched or app.mission.state in START_STATES)
                            and (not app.localization.active or app.localization.state == 'ACQUIRING')
                            and home is not None and app.map is not None and not app.map_is_preview
                            and app.map_matches_configured is True and app.localization_distance is not None):
                        app.home_seed_match = scan_map_agreement(
                            scan_points(message, self.localization_laser_pose), home,
                            app.map, app.localization_distance)
                        app.home_seed_match_at = scan_now
                        app.home_seed_match_key = app.home_seed_key()
                    eligible = ((app.mission.estop_latched or app.mission.state in START_STATES)
                                and (not app.localization.active or app.localization.state == 'ACQUIRING')
                                and home is not None and app.map is not None and not app.map_is_preview
                                and app.map_matches_configured is True and app.localization_distance is not None)
                    app.home_prior_evidence = self.home_prior_observer.observe(
                        message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
                        scan_points(message, self.localization_laser_pose), app.home_seed_key(), home,
                        app.map, app.localization_distance, app._display_map_data, app.odom_pose,
                        app.speed, app.odom_at, scan_now, eligible)
                except (TransformException, ValueError, TypeError):
                    self.localization_laser_pose = None
                    app.home_seed_match = None
                    self.home_prior_observer.invalidate()
                    app.home_prior_evidence = None

        def on_initial_pose(self, message):
            # An explicit operator estimate supersedes an automatic search.
            if message.header.frame_id.lstrip('/') != 'map':
                return
            with app.lock:
                stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                if stamp in self.startup_seed_stamps:
                    self.startup_seed_stamps.remove(stamp)
                    return
                if app.manual_pose_state == 'WAITING':
                    self.fail_manual_pose('다른 초기 위치 입력이 수신되어 웹 요청을 중단했습니다.')
                app.cancel_localization('수동 초기 위치 입력으로 위치 확인을 중단했습니다.')
                self.manual_initial_pose_ns = self.get_clock().now().nanoseconds

        def on_amcl(self, message):
            stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            if (not self.source_fresh(message.header.stamp, 2.5)
                    or message.header.frame_id.lstrip('/') != 'map' or stamp <= self.last_amcl_stamp_ns):
                return
            with app.lock:
                self.last_amcl_stamp_ns = stamp
                pose = message.pose.pose
                q, covariance = pose.orientation, message.pose.covariance
                values = [pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                          covariance[1], covariance[6], covariance[0], covariance[7], covariance[35]]
                if not all(math.isfinite(value) for value in values) or min(values[-3:]) < 0:
                    return
                cross = (covariance[1]+covariance[6])*.5
                root = math.sqrt((covariance[0]-covariance[7])**2+4*cross**2)
                largest_variance = (covariance[0]+covariance[7]+root)*.5
                if (covariance[0]+covariance[7]-root)*.5 < -1e-6:
                    return
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                position = {'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw}
                match = 0.
                if (self.localization_scan is not None and self.localization_laser_pose is not None
                        and app.map is not None and not app.map_is_preview
                        and app.localization_distance is not None
                        and time.monotonic()-app.localization_scan_at < .8):
                    points = scan_points(self.localization_scan, self.localization_laser_pose)
                    match = scan_map_agreement(points, position, app.map, app.localization_distance)
                if app.localization.active and stamp > self.localization_epoch_ns:
                    app.localization_match = match
                    app.localization.on_estimate(position['x'], position['y'], yaw,
                        math.sqrt(largest_variance), math.sqrt(covariance[35]),
                        quality_ok=bool(match >= .90))
                if (self.manual_initial_pose_ns is not None and stamp > self.manual_initial_pose_ns
                        and not app.localization.active):
                    if app.manual_pose_state == 'WAITING':
                        app.manual_pose_amcl_received_at = time.monotonic()
                        app.manual_pose_match = match
                        requested = app.manual_pose_requested
                        if requested is not None and match >= .60:
                            yaw_delta = math.atan2(math.sin(yaw-math.radians(requested['yaw_deg'])),
                                                   math.cos(yaw-math.radians(requested['yaw_deg'])))
                            if (math.hypot(position['x']-requested['x'], position['y']-requested['y']) < .5
                                    and abs(yaw_delta) < math.radians(30.)):
                                app.manual_pose_amcl_at = time.monotonic()
                    elif match >= .60:
                        app.localization_invalidated = False
                        self.manual_initial_pose_ns = None

        def on_battery(self, message):
            # Cached/duplicate packets must not keep a disconnected vehicle "online".
            if not self.source_fresh(message.header.stamp, max_age=5.0):
                return
            stamp_ns = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            if self.last_battery_stamp_ns is not None and stamp_ns <= self.last_battery_stamp_ns:
                return
            self.last_battery_stamp_ns = stamp_ns
            app.update_battery(message.voltage, message.percentage)

        def process_marker_frame(self, camera, capture_ns, corners, ids, image, now, fresh, capture_age=0.):
            expected = app.mission.expected_id
            if (app.mission.state != 'CONFIRMING' or expected is None
                    or app.mission.markers[expected].get('camera', 'rear') != camera
                    or app.manual.in_progress or app.localization.active
                    or self.observation_started_ros_ns is None
                    or capture_ns < self.observation_started_ros_ns or not fresh
                    or not math.isfinite(capture_age)
                    or not -.25 <= capture_age <= self.marker_alignment.frame_timeout):
                return False
            key = (app.mission.goal_stamp, expected, camera, self.observation_started_ros_ns)
            if (self.marker_alignment_key != key or self.marker_alignment.marker_id != expected
                    or self.marker_alignment.camera != camera):
                return False
            measurements = marker_measurements(corners, ids, image.shape[1], image.shape[0])
            frame_time = now-max(0., capture_age)
            accepted = self.marker_alignment.observe(capture_ns, measurements, camera, frame_time, now)
            self.observe_marker_wall(now)
            app.marker_alignment = self.marker_alignment.snapshot()
            if not accepted:
                return False
            quality = app.localization_health
            safe = (now-app.localization_health_at < 1.
                    and quality.get('autonomous_allowed') is True
                    and quality.get('level') in {'ok', 'healthy', 'warning'}
                    and now-app.localization_scan_at < .5 and app.localization_clear
                    and now-app.odom_at < .5 and abs(app.speed['linear']) < .025
                    and abs(app.speed['angular']) < .06)
            return (safe and self.marker_alignment.state == 'CONFIRMED'
                    and any(item['id'] == expected
                            and abs(item['center_x']) <= self.marker_alignment.center_tolerance
                            for item in measurements))

        def on_image(self, message):
            now = time.monotonic()
            capture_ns = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            capture_age = (self.get_clock().now().nanoseconds-capture_ns)/1e9
            capture_fresh = capture_ns > 0 and -.25 <= capture_age < 1.0
            if now-self.last_camera_processed < .12:
                return
            self.last_camera_processed = now
            try:
                image = image_to_bgr(message)
                # Rotate before both ArUco detection and the web JPEG. The raw
                # ROS topic remains unchanged for other camera consumers.
                if self.camera_rotation == 180:
                    image = cv2.rotate(image, cv2.ROTATE_180)
                ids_list, corners, ids = [], [], None
                if app.aruco_available:
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = self.detector.detectMarkers(gray) if self.detector else cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_parameters)
                    if ids is not None:
                        ids_list = [int(value) for value in ids.flatten()]
                        cv2.aruco.drawDetectedMarkers(image, corners, ids)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with app.lock:
                    if ok:
                        app.camera_jpeg = encoded.tobytes()
                    if capture_fresh:
                        app.camera_at = now
                    app.frame_number += 1
                    app.detected_ids = ids_list
                    stationary = (now-app.odom_at < 1.0 and abs(app.speed['linear']) < .03 and abs(app.speed['angular']) < .08)
                    pose = app.pose if now-app.pose_at < 1.0 else None
                    post_arrival = self.observation_started_ros_ns is not None and capture_ns >= self.observation_started_ros_ns
                    expected = app.mission.expected_id
                    rear_expected = (expected is not None
                                     and app.mission.markers[expected].get('camera', 'rear') == 'rear')
                    if rear_expected and not app.manual.in_progress and not app.localization.active:
                        aligned = self.process_marker_frame('rear', capture_ns, corners, ids, image, now,
                                                            capture_fresh and post_arrival, capture_age)
                        app.mission.observe(capture_ns, ids_list, pose, stationary, now,
                                            camera_fresh=capture_fresh and post_arrival,
                                            alignment_verified=aligned)
                    app.sync_delivery()
            except (ValueError, cv2.error) as exc:
                error = str(exc)
                if error != self.last_image_error:
                    self.get_logger().warning('Camera decode: '+error)
                    self.last_image_error = error

        def on_front_image(self, message):
            """Front video and post-arrival ArUco confirmation for front-facing goals."""
            now = time.monotonic()
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp <= 0 or not -.25 <= age < 1.0 or now-self.last_front_camera_processed < .12:
                return
            self.last_front_camera_processed = now
            try:
                image = image_to_bgr(message)
                if self.front_camera_rotation == 180:
                    image = cv2.rotate(image, cv2.ROTATE_180)
                ids_list, corners, ids = [], [], None
                if app.aruco_available:
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = (self.detector.detectMarkers(gray) if self.detector else
                                       cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_parameters))
                    if ids is not None:
                        ids_list = [int(value) for value in ids.flatten()]
                        cv2.aruco.drawDetectedMarkers(image, corners, ids)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                with app.lock:
                    if ok:
                        app.front_camera_jpeg = encoded.tobytes()
                    app.front_camera_at = now
                    app.front_detected_ids = ids_list
                    stationary = (now-app.odom_at < 1.0 and abs(app.speed['linear']) < .03
                                  and abs(app.speed['angular']) < .08)
                    pose = app.pose if now-app.pose_at < 1.0 else None
                    expected = app.mission.expected_id
                    front_expected = (expected is not None
                                      and app.mission.markers[expected].get('camera', 'rear') == 'front')
                    post_arrival = (self.observation_started_ros_ns is not None
                                    and stamp >= self.observation_started_ros_ns)
                    if front_expected and not app.manual.in_progress and not app.localization.active:
                        aligned = self.process_marker_frame('front', stamp, corners, ids, image, now,
                                                            post_arrival, age)
                        app.mission.observe(stamp, ids_list, pose, stationary, now,
                                            camera_fresh=post_arrival, alignment_verified=aligned)
                    app.sync_delivery()
            except (ValueError, cv2.error) as exc:
                self.get_logger().warning('Front camera decode: '+str(exc))

        def on_navigation(self, message):
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    return
                with app.lock:
                    app.navigation['status'] = payload.get('status', '')
                    app.navigation['message'] = payload.get('message', '')
                    previous_state = app.mission.state
                    pose = app.pose if time.monotonic()-app.pose_at < 2.0 else None
                    stationary = (time.monotonic()-app.odom_at < 2.0
                                  and abs(app.speed['linear']) < .025 and abs(app.speed['angular']) < .06)
                    matched = (app.mission.on_navigation(payload, pose=pose, stationary=stationary)
                               if not app.localization.active else False)
                    if (matched and app.manual.state == 'WAIT_REPLAN'
                            and payload.get('status') in {'MOVING', 'FOLLOWING', 'SUCCEEDED'}):
                        app.manual_auto_ready = True
                        app.manual.finish()
                        self.motion_gate = False
                        self.send_control('auto')
                    if matched and payload.get('status') == 'SUCCEEDED':
                        self.owns_navigation = False
                        app.navigation_active = False
                    if previous_state != 'CONFIRMING' and app.mission.state == 'CONFIRMING':
                        self.observation_started_ros_ns = self.get_clock().now().nanoseconds
                        self.stop()
                    app.sync_delivery()
            except (ValueError, TypeError):
                pass

        def on_planner(self, message):
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    return
                with app.lock:
                    app.navigation['planner_status'] = payload.get('status', '')
                    stamp = payload.get('sent_at_ns')
                    if (type(stamp) is int and stamp > self.last_planner_status_stamp_ns
                            and -.25e9 <= self.get_clock().now().nanoseconds-stamp < 1e9):
                        self.last_planner_status_stamp_ns = stamp
                        app.planner_details = {name: payload[name] for name in (
                            'goal_stamp', 'plan_kind', 'local_detour_state', 'local_attempt_id',
                            'global_plan_count', 'rejoin_point', 'rejoin_progress', 'search_duration_ms',
                            'adapter_duration_ms', 'expansions', 'detour_reason', 'global_duration_ms')
                            if name in payload}
                        app.planner_details_at = time.monotonic()-max(0., (
                            self.get_clock().now().nanoseconds-stamp)/1e9)
                    if not app.localization.active and payload.get('status') in {'WAIT_REPLAN', 'WAIT_POSE', 'WAIT_MAP', 'INVALID_GOAL'}:
                        app.mission.on_navigation(payload)
            except (ValueError, TypeError):
                pass

        def on_motion(self, message):
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    return
                stamp = payload.get('sent_at_ns')
                now_ns = self.get_clock().now().nanoseconds
                if (type(stamp) is not int or stamp < 0
                        or stamp <= self.last_motion_status_stamp_ns):
                    return
                age = (now_ns-stamp)/1e9
                if not -.25 <= age <= .75:
                    return
                with app.lock:
                    self.last_motion_status_stamp_ns = stamp
                    app.manual_mux_at = time.monotonic()-max(0.0, age)
                    app.manual_mux_status = payload
                    if payload.get('estop_latched') is True and not app.mission.estop_latched:
                        # The path follower can latch a safety stop when Nav2
                        # fails to confirm BackUp cancellation. Mirror that
                        # latch in the web state so the operator sees the
                        # emergency reset button and no mission resumes on its
                        # own after the motion mux has stopped the robot.
                        app.mission.emergency_stop()
                        try:
                            app.delivery.emergency()
                        except (OSError, ValueError, IndexError) as exc:
                            app.delivery.fail(exc)
            except (ValueError, TypeError):
                pass

        def on_health(self, message):
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    return
                stamp = payload.get('sent_at_ns', payload.get('stamp_ns'))
                if (type(stamp) is not int or stamp < 0 or stamp <= self.last_health_stamp_ns):
                    return
                age = (self.get_clock().now().nanoseconds-stamp)/1e9
                if not -.25 <= age < 1.5:
                    return
                with app.lock:
                    self.last_health_stamp_ns = stamp
                    app.localization_health = payload
                    app.localization_health_at = time.monotonic()-max(0., age)
            except (ValueError, TypeError, AttributeError):
                return

        def on_failure_memory(self, message):
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    return
                stamp = payload.get('sent_at_ns')
                if type(stamp) is not int or stamp <= 0:
                    return
                age = (self.get_clock().now().nanoseconds-stamp)/1e9
                if not -.25 <= age < 2.:
                    return
                with app.lock:
                    normalized = failure_memory_snapshot(payload, app.mission.goal_stamp,
                                                         app.mission.state, max(0., age))
                    if not normalized['available']:
                        return
                    previous_stamp = app.failure_memory_status.get('sent_at_ns', 0)
                    if (app.failure_memory_status.get('goal_stamp') == normalized['goal_stamp']
                            and stamp <= previous_stamp):
                        return  # Delayed/replayed statuses must not revive an expired display.
                    payload['goal_stamp'] = normalized['goal_stamp']
                    app.failure_memory_status = payload
                    app.failure_memory_at = time.monotonic()-max(0., age)
            except (ValueError, TypeError, AttributeError):
                return

        def tick_safety(self, now):
            if (self.safety_future is not None and self.safety_kind == 'reset'
                    and app.safety_request == 'stop'):
                # A fresh stop must preempt an outstanding reset immediately.
                # Canceling a client future does not cancel the server callback;
                # the stop service itself aborts its pending reset and re-latches.
                self.safety_future.cancel()
                self.safety_future = None
            if self.safety_future is not None:
                if self.safety_future.done():
                    kind = self.safety_kind
                    try:
                        result = self.safety_future.result()
                        success, detail = bool(result.success), result.message
                    except Exception as exc:
                        success, detail = False, str(exc)
                    self.safety_future = None
                    if app.complete_safety_result(kind, success, detail) and kind == 'reset':
                        self.sent_estop = False
                        self.motion_gate = True
                elif now-self.safety_started > 3.0:
                    self.safety_future.cancel()
                    self.safety_future = None
                    app.complete_safety_result(self.safety_kind, False,
                        '정지 서비스 응답 시간 초과. 정지 상태를 직접 확인하세요.')
            if app.safety_request and self.safety_future is None:
                client = self.reset_client if app.safety_request == 'reset' else self.estop_client
                if client.service_is_ready():
                    self.safety_kind = app.safety_request
                    self.safety_started = now
                    self.safety_future = client.call_async(Trigger.Request())
                else:
                    app.safety_result = '공통 정지 서비스 연결 대기. 로봇 motion_mux 실행 상태를 확인하세요.'

        def send_control(self, mode, linear=0., angular=0., operation=None):
            if app.safety_request == 'reset' or (self.safety_future is not None and self.safety_kind == 'reset'):
                # Keep the mux reset source stable during guard verification.
                return
            now = time.monotonic()
            operation = operation or ('localization' if self.localization_running else 'drive')
            if mode == 'estop':
                if now-self.last_estop_control_at < 1.0:
                    return
                self.last_estop_control_at = now
            else:
                # A later emergency stop must go out immediately, even if a
                # previous stop heartbeat was sent less than one second ago.
                self.last_estop_control_at = -math.inf
            if (mode == 'hold' and self.last_control_mode == mode
                    and self.last_control_operation == operation
                    and now-self.last_hold_control_at < 1.0
                    and not (self.localization_running and self.localization_hold_seq is None)):
                return
            self.motion_seq += 1
            message = String()
            message.data = json.dumps({'source_id': self.motion_source, 'seq': self.motion_seq,
                                       'sent_at_ns': self.get_clock().now().nanoseconds,
                                       'mode': mode, 'linear': linear, 'angular': angular,
                                       'operation': operation})
            self.motion_pub.publish(message)
            self.last_control_mode, self.last_control_operation = mode, operation
            if mode == 'hold':
                self.last_hold_control_at = now
            if mode == 'hold' and self.localization_running and self.localization_hold_seq is None:
                self.localization_hold_seq = self.motion_seq
            if mode == 'localization' and self.localization_lease_seq is None:
                self.localization_lease_seq = self.motion_seq
                self.localization_lease_at = time.monotonic()
            return self.motion_seq

        def localization_acknowledged(self, now, mode, minimum_seq, *, stopped=False):
            """A previous operation's durable hold is never a new handoff ACK."""
            status = app.manual_mux_status
            seq = status.get('control_seq')
            return (minimum_seq is not None and now-app.manual_mux_at < .5
                    and status.get('available') is True
                    and status.get('source_id') == self.motion_source
                    and status.get('mode') == mode and type(seq) is int and seq >= minimum_seq
                    and (not stopped or status.get('stop_acknowledged') is True))

        def stop(self):
            self.clear_auto_handover()
            self.motion_gate = True
            self.send_control('estop' if app.mission.estop_latched else 'hold')

        def cancel(self):
            self.stop()
            self.goal_waiting = None
            self.stop_until = time.monotonic()+.8
            self.cancel_required = True
            if self.cancel_future is None and self.cancel_client.service_is_ready():
                self.cancel_future = self.cancel_client.call_async(Trigger.Request())
                self.cancel_sent_at = time.monotonic()

        def publish_goal(self, marker):
            message = PoseStamped()
            message.header.frame_id = 'map'
            stamp = max(self.get_clock().now().nanoseconds, self.last_goal_stamp+1)
            self.last_goal_stamp = stamp
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
            message.pose.position.x, message.pose.position.y = marker['x'], marker['y']
            message.pose.orientation.z = math.sin(marker['yaw']/2)
            message.pose.orientation.w = math.cos(marker['yaw']/2)
            app.mission.goal_dispatched({'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec})
            self.owns_navigation = True
            app.navigation_active = True
            self.goal_pub.publish(message)
            # Auto ownership was acknowledged before the goal was published.
            # A joystick release still waits for the new route's MOVING status.
            self.motion_gate = False
            app.mission.event('홈 복귀 목표를 A*에 전달했습니다.' if app.mission.returning_home
                              else f'마커 {marker["id"]} 관측 목표를 A*에 전달했습니다.')

        def clear_auto_handover(self):
            self.auto_handover_started = None
            self.auto_handover_last_sent = None
            self.auto_handover_sent.clear()
            self.auto_handover_first_seq = None

        def auto_handover_failure_detail(self, now):
            """Report only evidence from this attempt, never an older rejection."""
            status = app.manual_mux_status
            if now-app.manual_mux_at > 1.0:
                return '로봇의 제어 상태 수신이 지연되고 있습니다.'
            if status.get('estop_latched') is True:
                return '비상정지가 유지 중입니다.'
            if status.get('available') is not True:
                return '로봇 속도 출력 제어권을 확인할 수 없습니다.'
            if status.get('autonomous_allowed') is not True:
                return '로봇의 자율주행 허용 조건이 충족되지 않았습니다.'
            if self.motion_pub.get_subscription_count() == 0:
                return '로봇 제어 토픽 구독자가 없습니다.'
            if self.goal_pub.get_subscription_count() == 0:
                return '경로 목표 토픽 구독자가 없습니다.'
            owner = status.get('source_id')
            if (owner not in (None, self.motion_source)
                    and not (isinstance(owner, str) and owner.startswith('mux-service:'))):
                return '다른 조종기가 현재 속도 제어권을 보유 중입니다.'
            first_seq = self.auto_handover_first_seq
            rejection = status.get('last_control_rejection') or {}
            if (first_seq is not None and rejection.get('source_id') == self.motion_source
                    and rejection.get('origin') == 'topic'
                    and type(rejection.get('seq')) is int
                    and rejection['seq'] >= first_seq):
                return '이번 인계 거부 사유: '+str(rejection.get('reason') or '알 수 없음')
            if first_seq is None:
                return '인계 명령을 발행할 준비 조건이 충족되지 않았습니다.'
            return '이번 인계 명령에 대한 로봇 수신 확인이 없습니다.'

        def auto_handover_acknowledged(self, now):
            status = app.manual_mux_status
            receipt = status.get('last_control') or {}
            seq = receipt.get('seq')
            sent_at = self.auto_handover_sent.get(seq) if type(seq) is int else None
            return (sent_at is not None and 0. <= now-sent_at <= .75
                    and 0. <= now-app.manual_mux_at <= .75
                    and status.get('available') is True
                    and status.get('autonomous_allowed') is True
                    and status.get('estop_latched') is False
                    and status.get('mode') == 'auto'
                    and status.get('source_id') == self.motion_source
                    and type(status.get('control_seq')) is int
                    and status['control_seq'] >= seq
                    and receipt.get('source_id') == self.motion_source
                    and receipt.get('mode') == 'auto'
                    and receipt.get('origin') == 'topic'
                    and receipt.get('accepted') is True)

        def tick_auto_handover(self, now):
            """Retry zero-speed control, then dispatch only after the exact ACK.

            A pause or timeout sends a newer hold sequence. Delayed auto packets
            cannot revive the route, because no goal has been published yet.
            """
            if self.auto_handover_started is None:
                self.auto_handover_started = now
                app.mission.reason = '출발 준비: 차량 속도 제어권 인계를 확인하고 있습니다.'
            if self.auto_handover_acknowledged(now):
                marker, self.goal_waiting = self.goal_waiting, None
                self.clear_auto_handover()
                self.publish_goal(marker)
                return
            if now-self.auto_handover_started >= self.auto_handover_timeout:
                reason = ('속도 제어권 인계 확인 실패. motion_mux 연결·시각 동기화·수동 조종 상태를 확인한 뒤 '
                          '임무 재개를 누르세요. '+self.auto_handover_failure_detail(now))
                self.goal_waiting = None
                self.clear_auto_handover()
                app.mission.pause()
                app.mission.reason = reason
                app.mission.event(reason, 'warning')
                self.stop()
                return
            status = app.manual_mux_status
            if (now-app.manual_mux_at > 1.0 or status.get('available') is not True
                    or status.get('estop_latched') is True
                    or status.get('autonomous_allowed') is not True
                    or self.motion_pub.get_subscription_count() == 0
                    or self.goal_pub.get_subscription_count() == 0):
                return
            # Do not steal another bridge's control lease. The reset service
            # is a permitted handoff source; it always leaves the mux held.
            owner = status.get('source_id')
            if (owner not in (None, self.motion_source)
                    and not (isinstance(owner, str) and owner.startswith('mux-service:'))):
                return
            if (self.auto_handover_last_sent is None
                    or now-self.auto_handover_last_sent >= self.auto_handover_retry):
                seq = self.send_control('auto')
                if seq is not None:
                    if self.auto_handover_first_seq is None:
                        self.auto_handover_first_seq = seq
                    self.auto_handover_sent[seq] = now
                    self.auto_handover_last_sent = now
                    # Only recent attempts can acknowledge this goal.
                    self.auto_handover_sent = {key: sent for key, sent in self.auto_handover_sent.items()
                                               if now-sent <= .75}

        def observe_marker_wall(self, now):
            """Use a centered camera axis and the matching fresh LiDAR surface.

            Camera mounts are explicit base-frame geometry. No image FOV or
            learned confidence is inferred. A corner, gap or stale scan cannot
            stand in for evidence that the rear camera faces the marker wall.
            """
            alignment = self.marker_alignment
            if not alignment.require_wall_alignment:
                return
            evidence = {'reliable': False, 'reason': 'center_camera_before_wall_fit',
                        'observed_at': now, 'frame_id': None}
            if (alignment.center_error is not None
                    and abs(alignment.center_error) <= alignment.center_tolerance):
                scan = self.localization_scan
                if (scan is None or self.localization_laser_pose is None
                        or now-app.localization_scan_at >= .5
                        or not self.source_fresh(scan.header.stamp, .5)):
                    evidence['reason'] = 'fresh_scan_and_laser_transform_required'
                else:
                    stamp = scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
                    if self.observation_started_ros_ns is None or stamp < self.observation_started_ros_ns:
                        evidence['reason'] = 'post_arrival_scan_required'
                    else:
                        evidence = estimate_camera_axis_wall(scan, self.localization_laser_pose,
                            camera_pose=self.marker_camera_poses[alignment.camera])
                        evidence.update(frame_id=stamp, observed_at=app.localization_scan_at)
            evidence['reason'] = {
                'center_camera_before_wall_fit': '표식을 카메라 가로 중앙에 맞춘 뒤 벽 방향을 확인합니다.',
                'fresh_scan_and_laser_transform_required': '최신 라이다와 차체 기준 센서 위치 변환을 기다립니다.',
                'post_arrival_scan_required': '목표 도착 이후에 측정한 새 라이다를 기다립니다.',
                'single_marker_bearing_wall': '표식 방향의 단일 벽을 확인했습니다.',
                'invalid_inputs': '벽 방향 측정 입력 또는 카메라 장착 위치를 확인하세요.',
                'insufficient_angular_resolution': '벽 방향을 확인할 라이다 각도 해상도가 부족합니다.',
                'insufficient_returns': '표식 방향에 벽을 확인할 라이다 측정점이 부족합니다.',
                'no_single_surface_spans_marker_bearing': '표식 방향의 단일 벽을 확인할 수 없습니다.',
                'insufficient_angular_span': '보이는 벽의 각도 범위가 너무 좁습니다.',
                'insufficient_surface_span': '보이는 벽의 길이가 너무 짧습니다.',
                'mixed_surfaces_or_corner': '모서리 또는 여러 벽이 겹쳐 방향을 확정할 수 없습니다.',
                'nonplanar_or_short_surface': '벽이 고르지 않거나 보이는 길이가 부족합니다.',
                'fitted_wall_does_not_span_marker_bearing': '확인한 벽이 표식 방향을 지나지 않습니다.',
                'wall_observed_too_obliquely': '벽을 너무 비스듬히 보고 있어 방향 확인을 기다립니다.',
                'wall_distance_or_bearing_intersection_invalid': '표식 방향의 벽 거리 또는 보이는 범위를 확인하세요.',
            }.get(evidence.get('reason'), '벽 방향을 확인할 수 없어 정지를 유지합니다.')
            alignment.observe_wall(evidence, now)

        def tick_marker_alignment(self, now, cancelled, mux_held):
            """Visual yaw servo after terminal navigation, never a second cmd_vel owner."""
            self.marker_alignment_angular = 0.
            mission = app.mission
            expected = mission.expected_id
            if mission.state != 'CONFIRMING' or expected is None:
                self.marker_alignment_key = None
                return
            target = mission.markers[expected]
            camera = target.get('camera', 'rear')
            key = (mission.goal_stamp, expected, camera, self.observation_started_ros_ns)
            if self.marker_alignment_key != key:
                # Hold must be acknowledged and wheel odometry must show a
                # stop before taking the leased in-place rotation channel.
                stopped = (now-app.odom_at < .5 and abs(app.speed['linear']) < .025
                           and abs(app.speed['angular']) < .06)
                yaw = (app.odom_pose or {}).get('yaw')
                if (not cancelled or not mux_held or not stopped
                        or yaw is None or not math.isfinite(yaw)):
                    app.marker_alignment = {'state': 'SEARCHING', 'camera': camera,
                        'marker_id': expected, 'reason': '주행 종료·정지 응답과 오도메트리를 확인합니다.',
                        'frames': 0, 'required': mission.required_confirmations,
                        'max_seconds': self.marker_alignment.timeout}
                    if mission.observation_since is not None and now-mission.observation_since > 5.:
                        mission.pause()
                        mission.reason = '정지 응답을 확인하지 못해 표식 탐색을 일시정지했습니다. 센서 확인 후 재개하세요.'
                        mission.event(mission.reason, 'warning')
                    return
                self.marker_alignment.start(expected, camera, yaw, now)
                self.marker_alignment_key = key
            self.observe_marker_wall(now)
            pose = app.pose
            near_target = (pose is not None and now-app.pose_at < .5
                and math.hypot(pose['x']-target['x'], pose['y']-target['y']) <= mission.pose_tolerance)
            quality = app.localization_health
            quality_ok = (now-app.localization_health_at < 1.
                          and quality.get('autonomous_allowed') is True
                          and quality.get('level') in {'ok', 'healthy', 'warning'})
            sensors = (now-app.localization_odom_at < .5 and now-app.localization_scan_at < .5)
            camera_at = app.front_camera_at if camera == 'front' else app.camera_at
            yaw = (app.odom_pose or {}).get('yaw', math.nan)
            self.marker_alignment_angular = self.marker_alignment.tick(now, yaw,
                pose_ok=near_target and quality_ok, scan_clear=sensors and app.localization_clear,
                stationary=abs(app.speed['angular']) < .06,
                translation_stopped=abs(app.speed['linear']) < .025,
                manual_active=app.manual.in_progress or app.localization.active,
                estop=mission.estop_latched or app.safety_request is not None,
                camera_fresh=now-camera_at < .8)
            app.marker_alignment = self.marker_alignment.snapshot()
            app.marker_alignment['required'] = app.marker_alignment.get('required_frames', 3)
            app.marker_alignment['max_seconds'] = self.marker_alignment.timeout
            if self.marker_alignment.state == 'FAILED':
                mission.pause()
                mission.reason = '표식 정렬을 일시정지했습니다: '+app.marker_alignment['reason']
                mission.event(mission.reason, 'warning')

        def update_localization_services(self, now):
            amcl_ready = self.amcl_state_client.service_is_ready()
            app.localization_services_ready = (self.global_localization_client.service_is_ready()
                and self.nomotion_client.service_is_ready() and amcl_ready)
            app.manual_pose_service_ready = self.initial_pose_client.service_is_ready()
            if not amcl_ready or (self.amcl_state_future is not None
                    and not self.amcl_state_future.done()
                    and now-self.amcl_state_checked_at >= 2.):
                # An unanswered request can otherwise pin this future forever
                # after AMCL restarts, leaving the web unable to verify its state.
                # Never use the last ACTIVE reply after the query is lost.
                pending = self.amcl_state_future
                self.amcl_state_future = None
                if pending is not None and not pending.done():
                    try:
                        self.amcl_state_client.remove_pending_request(pending)
                    except Exception:
                        # The old future is abandoned even if the service
                        # vanished while its pending request was removed.
                        pass
                    pending.cancel()
                app.localization_amcl_active_at = 0.
            if amcl_ready and self.amcl_state_future is not None and self.amcl_state_future.done():
                try:
                    response = self.amcl_state_future.result()
                    app.localization_amcl_active_at = now if response.current_state.id == 3 else 0.
                except Exception:
                    app.localization_amcl_active_at = 0.
                self.amcl_state_future = None
            if (amcl_ready and self.amcl_state_future is None
                    and now-self.amcl_state_checked_at >= 1.):
                self.amcl_state_checked_at = now
                try:
                    self.amcl_state_future = self.amcl_state_client.call_async(GetState.Request())
                except Exception:
                    app.localization_amcl_active_at = 0.
            if self.nomotion_future is not None and self.nomotion_future.done():
                self.nomotion_future = None

        def fail_manual_pose(self, reason):
            if self.manual_pose_future is not None and not self.manual_pose_future.done():
                self.initial_pose_client.remove_pending_request(self.manual_pose_future)
                self.manual_pose_future.cancel()
            self.manual_pose_future = None
            self.clear_manual_pose_nomotion()
            self.manual_pose_last_stop_ack_at = None
            app.manual_pose_request = None
            app.manual_pose_state = 'FAILED'
            app.manual_pose_result = '초기 위치 확인 실패: '+reason+' 비상정지를 유지합니다.'
            app.localization_invalidated = True
            self.manual_initial_pose_ns = None
            app.mission.event(app.manual_pose_result, 'warning')

        def clear_manual_pose_nomotion(self):
            future = self.manual_pose_nomotion_future
            if future is not None and not future.done():
                try:
                    self.nomotion_client.remove_pending_request(future)
                    future.cancel()
                except Exception:
                    pass
            self.manual_pose_nomotion_future = None

        def manual_pose_stop_ack_grace(self, now):
            """Keep a stopped pose prior pending through a brief mux ACK gap.

            A missed status/output tick must not invalidate an AMCL prior that
            was already accepted. This grace cannot certify the pose, and it
            never applies if ESTOP, mux ownership, or zero output is lost.
            """
            status = app.manual_mux_status
            speed = app.speed
            last_ack = self.manual_pose_last_stop_ack_at
            return bool(
                last_ack is not None and 0. <= now-last_ack < 1.5
                and app.mission.estop_latched
                and status.get('available') is True
                and status.get('estop_latched') is True
                and type(status.get('output_linear')) in (int, float)
                and type(status.get('output_angular')) in (int, float)
                and status['output_linear'] == status['output_angular'] == 0.
                and type(speed.get('linear')) in (int, float)
                and type(speed.get('angular')) in (int, float)
                and abs(speed['linear']) < .01 and abs(speed['angular']) < .03)

        def tick_manual_pose(self, now):
            if app.manual_pose_state != 'WAITING':
                return
            if self.localization_future is not None:
                self.fail_manual_pose('다른 AMCL 초기화 서비스가 처리 중입니다.')
                return
            request = app.manual_pose_request
            if request is not None:
                available, reason = app.manual_pose_readiness()
                if (not available or now-request['at'] >= 2.
                        or request['map_version'] != app.map_version):
                    self.fail_manual_pose(reason if not available else '요청이 오래되었거나 운용 지도가 바뀌었습니다.')
                    return
                error = app.manual_pose_geometry_error(request['pose'])
                if error:
                    self.fail_manual_pose(error)
                    return
                pose = request['pose']
                seed = PoseWithCovarianceStamped()
                seed.header.frame_id = 'map'
                stamp = self.get_clock().now().nanoseconds
                seed.header.stamp.sec, seed.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                seed.pose.pose.position.x, seed.pose.pose.position.y = pose['x'], pose['y']
                seed.pose.pose.orientation.z = math.sin(pose['yaw']/2.)
                seed.pose.pose.orientation.w = math.cos(pose['yaw']/2.)
                seed.pose.covariance[0] = seed.pose.covariance[7] = .25**2
                seed.pose.covariance[35] = .35**2
                service = SetInitialPose.Request()
                service.pose = seed
                try:
                    self.manual_pose_future = self.initial_pose_client.call_async(service)
                except Exception as exc:
                    self.fail_manual_pose('AMCL 초기 위치 서비스 요청 오류: '+str(exc))
                    return
                self.manual_pose_ros_stamp_ns = self.manual_initial_pose_ns = self.localization_epoch_ns = stamp
                self.manual_pose_last_stop_ack_at = now
                self.clear_manual_pose_nomotion()
                self.manual_pose_nomotion_at = 0.
                app.manual_pose_request = None
                app.manual_pose_sent_at = now
                app.localization_invalidated = True
                app.manual_pose_result = 'AMCL 초기 위치 서비스 응답과 새 map→base TF를 기다립니다. 비상정지 유지.'
            if self.manual_pose_future is not None:
                if self.manual_pose_future.done():
                    try:
                        self.manual_pose_future.result()
                    except Exception as exc:
                        self.fail_manual_pose('AMCL 서비스 오류: '+str(exc))
                        return
                    self.manual_pose_future = None
                    app.manual_pose_result = 'AMCL이 초기 위치 입력을 수신했습니다. LiDAR·지도와 새 TF 검증 중입니다.'
                elif now-app.manual_pose_sent_at > 5.:
                    self.fail_manual_pose('AMCL 초기 위치 서비스 응답이 없습니다.')
                    return
            available, reason = app.manual_pose_readiness()
            if available:
                self.manual_pose_last_stop_ack_at = now
            else:
                if (reason == '차량 비상정지와 정지 확인 응답이 필요합니다.'
                        and self.manual_pose_stop_ack_grace(now)):
                    context_ready, context_reason = app.manual_pose_readiness(require_stop_ack=False)
                    if not context_ready:
                        reason = context_reason
                else:
                    context_ready = False
                if not context_ready:
                    self.fail_manual_pose('검증 중 출발 조건이 사라졌습니다: '+reason)
                    return
            if app.map_version != app.manual_pose_map_version:
                self.fail_manual_pose('운용 지도가 바뀌었습니다.')
                return
            # AMCL may accept a stopped initial pose without publishing a new
            # posterior until another laser update is requested. The robot-side
            # health node can retain a healthy estimate from a previous AMCL
            # instance, so it cannot always trigger this refresh for us.
            # A service response never verifies the pose; only on_amcl does.
            if self.manual_pose_nomotion_future is not None:
                if self.manual_pose_nomotion_future.done():
                    self.manual_pose_nomotion_future = None
                elif now-self.manual_pose_nomotion_at >= 2.:
                    self.clear_manual_pose_nomotion()
            if (available and self.manual_pose_future is None
                    and app.manual_pose_amcl_received_at <= app.manual_pose_sent_at
                    and now-app.manual_pose_sent_at >= 1.
                    and now-app.manual_pose_sent_at < 12.
                    and now-self.manual_pose_nomotion_at >= 2.
                    and self.manual_pose_nomotion_future is None
                    and self.nomotion_client.service_is_ready()):
                self.manual_pose_nomotion_at = now
                try:
                    self.manual_pose_nomotion_future = self.nomotion_client.call_async(Empty.Request())
                except Exception:
                    # Keep the normal verification deadline and fail closed.
                    pass
            if now-app.manual_pose_sent_at >= 12.:
                if app.manual_pose_amcl_received_at <= app.manual_pose_sent_at:
                    reason = '새 /amcl_pose가 수신되지 않았습니다. /scan과 AMCL 상태를 확인하세요.'
                elif app.manual_pose_match is not None and app.manual_pose_match < .60:
                    reason = (f'라이다·지도 일치율 {app.manual_pose_match:.0%}가 60%보다 낮습니다. '
                              '실제 좌표·방향 또는 지도를 확인하세요.')
                else:
                    reason = '새 AMCL 추정값과 map→base TF가 입력 위치에서 확인되지 않았습니다.'
                self.fail_manual_pose(reason)
                return
            requested = app.manual_pose_requested
            if (available and self.manual_pose_future is None
                    and app.manual_pose_amcl_at > app.manual_pose_sent_at
                    and now-app.manual_pose_amcl_at < 2. and now-app.pose_at < 2.
                    and app.pose_tf_stamp_ns > self.manual_pose_ros_stamp_ns and requested is not None
                    and app.pose is not None):
                yaw_delta = math.atan2(math.sin(app.pose['yaw']-math.radians(requested['yaw_deg'])),
                                       math.cos(app.pose['yaw']-math.radians(requested['yaw_deg'])))
                if (math.hypot(app.pose['x']-requested['x'], app.pose['y']-requested['y']) < .5
                        and abs(yaw_delta) < math.radians(30.)):
                    app.manual_pose_state = 'VERIFIED'
                    app.manual_pose_result = 'AMCL 위치와 새 map→base TF 확인 완료 · 비상정지 유지. 출발 조건을 확인하세요.'
                    app.localization_invalidated = False
                    self.manual_initial_pose_ns = None
                    self.manual_pose_last_stop_ack_at = None
                    self.clear_manual_pose_nomotion()
                    app.mission.event(app.manual_pose_result)

        def tick_localization(self, now, cancelled, mux_held):
            loc = app.localization
            acquiring = loc.state == 'ACQUIRING'
            if self.localization_future is not None and self.localization_future.done():
                app.localization_service_pending = False
                try:
                    response = self.localization_future.result()
                    if loc.state == 'INITIALIZING' and response is not None:
                        self.localization_epoch_ns = self.get_clock().now().nanoseconds
                        loc.initialized(now)
                except Exception as exc:
                    loc.fail('AMCL 초기화 서비스 오류: '+str(exc), now)
                self.localization_future = None
            if loc.active:
                manual_ready, manual_reason = app.manual_readiness()
                if (not manual_ready or app.map_is_preview or app.map is None
                        or app.map_matches_configured is False):
                    detail = manual_reason if not manual_ready else '설정한 저장 지도와 /map 수신을 확인하세요.'
                    loc.fail('위치 확인을 중단합니다: '+detail, now)
                elif now-app.localization_amcl_active_at > 3. or not app.localization_services_ready:
                    loc.fail('AMCL 활성 상태를 확인할 수 없어 회전을 중단합니다.', now)
            hold_ack = self.localization_acknowledged(now, 'hold', self.localization_hold_seq,
                                                     stopped=True)
            health_fresh = now-app.localization_health_at < 1.
            sweep_allowed = health_fresh and app.localization_health.get('allow_localization') is True
            if loc.active and health_fresh and (app.localization_health.get('level') == 'critical'
                    or app.localization_health.get('stop_latched') is True):
                loc.fail('위치 감시의 비상정지 또는 심각한 오류로 위치 확인을 중단합니다.', now)
            if loc.active and now-loc.started_at >= loc.timeout:
                loc.fail('제한 시간 안에 위치를 확인하지 못했습니다. 실제 홈과 센서 상태를 확인하고 다시 초기화하세요.', now)
            if (loc.active and app.localization_startup_home
                    and (app.localization_home_seed != app.delivery.registered_homepoint
                         or app.localization_home_seed != app.delivery.homepoint)):
                loc.fail('등록 홈 또는 출발 Homepoint가 바뀌어 자세 초기화를 중단합니다.', now)
            stationary = (now-app.odom_at < .5 and abs(app.speed['linear']) < .025
                          and abs(app.speed['angular']) < .06)
            if acquiring:
                waiting = []
                if not cancelled:
                    waiting.append('주행 취소 확인')
                if not hold_ack:
                    waiting.append('새 정지 명령 응답')
                if not health_fresh:
                    waiting.append('최신 위치 감시 응답')
                elif not sweep_allowed:
                    waiting.append('위치 확인 회전 허용')
                if not stationary:
                    waiting.append('실제 정지 확인')
                mux_status = app.manual_mux_status
                app.localization_control = {'state': 'WAIT_HOLD', 'reason':
                    '대기: '+', '.join(waiting) if waiting else '정지·제어권 확인 완료',
                    'waiting_for': waiting, 'cancelled': cancelled,
                    'cancel_pending': self.cancel_required or self.cancel_future is not None,
                    'hold_seq': self.localization_hold_seq, 'hold_acknowledged': hold_ack,
                    'health_fresh': health_fresh, 'sweep_allowed': sweep_allowed,
                    'stationary': stationary, 'expected_source_id': self.motion_source,
                    'control_subscribers': self.motion_pub.get_subscription_count(),
                    'mux_source_id': mux_status.get('source_id'),
                    'mux_control_seq': mux_status.get('control_seq'),
                    'mux_mode': mux_status.get('mode'),
                    'mux_control_rejected_reason': mux_status.get('control_rejected_reason'),
                    'mux_last_control_rejection': deepcopy(mux_status.get('last_control_rejection'))}
            if loc.state == 'ACQUIRING' and cancelled and hold_ack and sweep_allowed:
                if stationary:
                    loc.acquired(now)
                    if app.localization_startup_home:
                        available, reason = app.startup_home_readiness()
                        if not available:
                            loc.fail(reason, now)
                        elif not any(info.node_name.lstrip('/') == 'amcl'
                                     for info in self.get_subscriptions_info_by_topic('/initialpose')):
                            loc.fail('AMCL의 초기 위치 입력 구독을 확인할 수 없습니다.', now)
                        else:
                            home = app.localization_home_seed
                            seed = PoseWithCovarianceStamped()
                            seed.header.frame_id = 'map'
                            stamp = self.get_clock().now().nanoseconds
                            seed.header.stamp.sec, seed.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                            seed.pose.pose.position.x = home['x']
                            seed.pose.pose.position.y = home['y']
                            seed.pose.pose.orientation.z = math.sin(home['yaw']/2.)
                            seed.pose.pose.orientation.w = math.cos(home['yaw']/2.)
                            # Home is a prior, not a ground-truth pose.
                            seed.pose.covariance[0] = seed.pose.covariance[7] = .30**2
                            seed.pose.covariance[35] = .35**2
                            self.startup_seed_stamps.append(stamp)
                            self.localization_epoch_ns = stamp
                            app.localization_invalidated = True
                            self.initial_pose_pub.publish(seed)
                            loc.initialized(now)
                            loc.reason = '등록 홈을 초기 가설로 입력했습니다. 회전하며 라이다와 지도의 일치를 검증합니다.'
                    elif app.localization_use_existing_pose:
                        if not app.can_refine_localization():
                            loc.fail('기존 위치가 오래되었거나 지도에서 유효하지 않아 보정을 중단합니다. 수동 초기 위치를 확인하세요.', now)
                        else:
                            # Checking a valid location must not replace a good
                            # AMCL posterior with uniform global hypotheses.
                            self.localization_epoch_ns = self.get_clock().now().nanoseconds
                            loc.initialized(now)
                            loc.reason = '현재 위치를 유지하며 회전해 라이다와 지도를 대조합니다.'
                    else:
                        app.localization_invalidated = True
                        app.localization_service_pending = True
                        self.localization_future = self.global_localization_client.call_async(Empty.Request())
            if (loc.state in {'ROTATING', 'RETURNING', 'SETTLING'} and self.nomotion_future is None
                    and now-self.nomotion_at >= .6 and self.nomotion_client.service_is_ready()):
                self.nomotion_at = now
                self.nomotion_future = self.nomotion_client.call_async(Empty.Request())
            sensors_ready = (now-app.localization_odom_at < .8 and now-app.localization_scan_at < .8)
            if loc.active and not sensors_ready:
                if self.localization_sensor_gap_since is None:
                    self.localization_sensor_gap_since = now
                    self.localization_sensor_gap_reason = loc.reason
                loc.pause_for_sensor_gap(now)
                if now-self.localization_sensor_gap_since < 1.0:
                    self.localization_angular = loc.angular = 0.
                    loc.reason = '라이다·오도메트리 수신 복구를 기다리며 정지합니다.'
                    return
                loc.fail('라이다 또는 오도메트리 수신이 1초 이상 끊겨 위치 확인을 중단합니다.', now)
            elif self.localization_sensor_gap_since is not None:
                if loc.active and self.localization_sensor_gap_reason is not None:
                    loc.reason = self.localization_sensor_gap_reason
                self.localization_sensor_gap_since = self.localization_sensor_gap_reason = None
            # An invalid beam is unknown clearance, not proof of a new nearby
            # obstacle. Stop immediately, but allow one second for a complete
            # replacement scan. Never tick/succeed or turn while blind.
            if (loc.state in {'ROTATING', 'RETURNING', 'SETTLING'} and sensors_ready
                    and not app.localization_clear and app.localization_clearance is None):
                if self.localization_scan_gap_since is None:
                    self.localization_scan_gap_since = now
                    self.localization_scan_gap_reason = loc.reason
                if now-self.localization_scan_gap_since < 1.:
                    self.localization_angular = loc.angular = 0.
                    loc.reason = '라이다 일부 누락 복구를 기다리며 정지합니다.'
                    return
                loc.fail('라이다 일부 누락이 1초 이상 지속되어 위치 확인을 중단합니다.', now)
            elif self.localization_scan_gap_since is not None:
                if loc.active and self.localization_scan_gap_reason is not None:
                    loc.reason = self.localization_scan_gap_reason
                self.localization_scan_gap_since = self.localization_scan_gap_reason = None
            lease_ack = self.localization_acknowledged(now, 'localization', self.localization_lease_seq)
            if loc.state in loc.MOVING_STATES:
                app.localization_control = {'state': 'LEASED' if lease_ack else 'WAIT_LEASE',
                    'reason': '위치 확인 전용 회전 제어권 확인' if lease_ack else
                    '위치 확인 전용 제어권 응답 대기 · 정지 유지',
                    'lease_seq': self.localization_lease_seq, 'lease_acknowledged': lease_ack}
                if not sweep_allowed:
                    if self.localization_policy_gap_since is None:
                        self.localization_policy_gap_since = now
                        self.localization_policy_gap_reason = loc.reason
                    loc.pause_for_sensor_gap(now)
                    self.localization_angular = loc.angular = 0.
                    if now-self.localization_policy_gap_since < 1.:
                        loc.reason = ('위치 감시 입력·오도메트리 좌표 변환 복구를 기다리며 정지합니다.'
                                      if health_fresh else '최신 위치 감시 응답을 기다리며 정지합니다.')
                        app.localization_control = {'state': 'WAIT_HEALTH', 'reason': loc.reason,
                            'lease_acknowledged': lease_ack, 'sweep_allowed': False}
                        return
                    loc.fail('위치 확인 회전 허용 또는 최신 위치 감시 응답이 1초 이상 없어 정지합니다.', now)
                elif not lease_ack:
                    # First send a zero packet to acquire this distinct lease.
                    # Subsequent motion needs its acknowledgment, not a hold
                    # status cached before initialization or another joystick.
                    self.localization_angular = loc.angular = 0.
                    if self.localization_lease_at is not None and now-self.localization_lease_at >= 1.:
                        detail = str(app.manual_mux_status.get('control_rejected_reason') or
                                     app.manual_mux_status.get('reason') or '응답 없음')
                        loc.fail('위치 확인 제어권이 승인되지 않았습니다: '+detail, now)
                    if loc.active:
                        return
                if sweep_allowed and self.localization_policy_gap_since is not None:
                    loc.reason = self.localization_policy_gap_reason or loc.reason
                    self.localization_policy_gap_since = self.localization_policy_gap_reason = None
            settled_health_ok = (health_fresh
                and app.localization_health.get('autonomous_allowed') is True
                and app.localization_health.get('level') in {'ok', 'healthy'})
            if loc.state == 'SETTLING':
                stationary = stationary and hold_ack
                if stationary and loc.mark_settled_stop(now):
                    self.localization_epoch_ns = self.get_clock().now().nanoseconds
                app.localization_control = {'state': 'SETTLING', 'reason':
                    '새 AMCL 결과·실제 정지·정상 위치 신뢰도를 확인합니다.',
                    'stationary': stationary, 'health_ready': settled_health_ok,
                    'hold_acknowledged': hold_ack}
            previous_state = loc.state
            self.localization_angular = loc.tick(now,
                sensors_ready=sensors_ready,
                rotation_clear=app.localization_clear,
                stationary=stationary, settled_health_ok=settled_health_ok)
            if previous_state == 'ACQUIRING' and loc.state == 'FAILED':
                waiting = app.localization_control.get('waiting_for') or []
                if waiting:
                    loc.reason += ' 대기 항목: '+', '.join(waiting)+'.'
            if previous_state != 'RETURNING' and loc.state == 'RETURNING':
                # A new return phase cannot inherit the sweep's accepted lease.
                self.localization_lease_seq = None
                self.localization_lease_at = None
            if previous_state in loc.MOVING_STATES and loc.state == 'SETTLING':
                self.localization_hold_seq = None
                if app.localization_startup_home:
                    # Header time, not receipt time, proves the AMCL result was
                    # produced after the return stopped requesting rotation.
                    self.localization_epoch_ns = self.get_clock().now().nanoseconds
            if self.localization_running and not loc.active:
                self.localization_running = False
                app.localization_control = dict(app.localization_control,
                                                state=loc.state, reason=loc.reason)
                self.localization_hold_seq = self.localization_lease_seq = None
                self.localization_lease_at = None
                self.localization_policy_gap_since = self.localization_policy_gap_reason = None
                self.motion_gate = True
                self.stop_until = now+.8
                if loc.succeeded:
                    app.localization_invalidated = False
                    app.mission.event('위치 추정 완료: 지도에서 차량 위치를 확인하고 임무 재개를 누르세요.')
                else:
                    app.mission.event(loc.reason+(' 기존 AMCL 위치는 전체 초기화하지 않았습니다.'
                                                 if app.localization_use_existing_pose else ''), 'warning')
                self.stop()

        def tick_home_seed(self, now):
            request = app.home_seed_request
            if request is None:
                return
            # Recheck at ROS dispatch time; an HTTP click is not permission to
            # apply an old seed after a delayed callback or connection change.
            available, reason = app.seed_home_readiness()
            app.home_seed_request = None
            if (not available or now-request['at'] >= 2.
                    or request['key'] != app.home_seed_key()):
                app.home_seed_result = '초기 위치 입력 거부: '+(reason if not available else '요청이 오래되었거나 등록 홈이 바뀌었습니다.')
                app.mission.event(app.home_seed_result, 'warning')
                return
            if not any(info.node_name.lstrip('/') == 'amcl'
                       for info in self.get_subscriptions_info_by_topic('/initialpose')):
                app.home_seed_result = '초기 위치 입력 거부: AMCL 구독이 없습니다.'
                app.mission.event(app.home_seed_result, 'warning')
                return
            home = request['home']
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = 'map'
            stamp = self.get_clock().now().nanoseconds
            seed.header.stamp.sec, seed.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            seed.pose.pose.position.x, seed.pose.pose.position.y = home['x'], home['y']
            seed.pose.pose.orientation.z = math.sin(home['yaw']/2.)
            seed.pose.pose.orientation.w = math.cos(home['yaw']/2.)
            seed.pose.covariance[0] = seed.pose.covariance[7] = .30**2
            seed.pose.covariance[35] = .35**2
            self.startup_seed_stamps.append(stamp)
            self.manual_initial_pose_ns = self.localization_epoch_ns = stamp
            app.localization_invalidated = True
            self.initial_pose_pub.publish(seed)
            app.home_seed_result = '등록 홈 초기 가설 입력 완료 · 비상정지 유지. 센서 안정 후 해제하고 자세 초기화를 실행하세요.'
            app.mission.event(app.home_seed_result)

        def tick(self):
            now = time.monotonic()
            with app.lock:
                self.tick_safety(now)
                self.tick_home_seed(now)
                app.goal_available = goal_consumers_ready(
                    endpoint.node_name for endpoint in self.get_subscriptions_info_by_topic('/smart_vision_amr/goal'))
                app.planner_available = self.count_publishers('/smart_vision_amr/planner_status') > 0
                app.cancel_available = self.cancel_client.service_is_ready()
                app.navigation_command_publishers = self.count_publishers('/smart_vision_amr/navigation_cmd_vel')
                app.unmanaged_velocity_publishers = [endpoint.node_name
                    for endpoint in self.get_publishers_info_by_topic('/cmd_vel')
                    if endpoint.node_name != 'motion_mux']
                self.update_localization_services(now)
                self.tick_manual_pose(now)
                if app.localization_stop_requested:
                    self.cancel()
                    app.localization_stop_requested = False
                if app.localization_requested:
                    app.localization_requested = False
                    self.localization_running = True
                    self.localization_hold_seq = self.localization_lease_seq = None
                    self.localization_lease_at = None
                    self.localization_scan_gap_since = self.localization_scan_gap_reason = None
                    self.localization_sensor_gap_since = self.localization_sensor_gap_reason = None
                    self.localization_policy_gap_since = self.localization_policy_gap_reason = None
                    self.motion_gate = True
                    self.goal_waiting = None
                    self.localization_angular = 0.
                    if app.cancel_available or self.owns_navigation or app.navigation_command_publishers:
                        self.cancel()
                    else:
                        self.stop_until = now+.3
                        self.stop()
                if app.manual.expire():
                    app.manual_cancel_requested = True
                if app.manual.active and not app.manual_readiness()[0]:
                    app.manual.release('센서 또는 속도 우선순위 연결을 확인한 뒤 기존 임무를 재개합니다.')
                    app.manual_cancel_requested = True
                if app.manual_cancel_requested:
                    self.motion_gate = True
                    if app.cancel_available or self.owns_navigation or app.navigation_command_publishers:
                        self.cancel()
                    else:
                        self.goal_waiting = None
                        self.stop_until = now+.2
                        self.stop()
                    app.manual_cancel_requested = False
                try:
                    transform = self.tf_buffer.lookup_transform('map', options.base_frame, rclpy.time.Time())
                    q = transform.transform.rotation
                    tf_ns = transform.header.stamp.sec*1000000000+transform.header.stamp.nanosec
                    age = abs(self.get_clock().now().nanoseconds-tf_ns)/1e9 if tf_ns else 0
                    if age < 2.0:
                        app.pose = {'x': transform.transform.translation.x, 'y': transform.transform.translation.y,
                                    'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
                        app.pose_at = now
                        app.pose_tf_stamp_ns = tf_ns
                except TransformException:
                    pass
                app.sync_delivery()
                # A camera outage cannot invalidate an A* route. Arrival remains
                # in CONFIRMING until fresh, post-arrival ArUco frames are seen.
                required_sensors = (app.pose_at, app.scan_at, app.odom_at)
                if app.mission.state in ACTIVE_STATES and any(now-stamp > 3.0 for stamp in required_sensors):
                    app.mission.pause()
                    app.mission.reason = 'TF/LiDAR/오도메트리 수신이 끊겨 일시정지했습니다. 복구 후 명시적으로 재개하세요.'
                    app.mission.event(app.mission.reason, 'warning')
                if not app.manual.in_progress and not app.localization.active and now-app.pose_at < 2.0 and now-app.odom_at < 2.0:
                    app.mission.observe_home(app.pose, abs(app.speed['linear']) < .025 and abs(app.speed['angular']) < .06)
                    app.sync_delivery()
                if self.cancel_future is not None:
                    if self.cancel_future.done():
                        try:
                            response = self.cancel_future.result()
                            if response and response.success:
                                self.cancel_required = False
                                self.owns_navigation = False
                                app.navigation_active = False
                                if app.connection_error == '주행 취소 서비스 응답 지연':
                                    app.connection_error = ''
                            else:
                                app.mission.event('주행 취소가 확인되지 않았습니다. 정지를 유지합니다.', 'warning')
                        except Exception as exc:
                            app.mission.event('주행 취소 서비스 오류: '+str(exc), 'warning')
                        self.cancel_future = None
                        self.cancel_completed_at = now
                    elif now-self.cancel_sent_at > 5.0:
                        app.connection_error = '주행 취소 서비스 응답 지연'
                        self.stop_until = now+.8
                        # A lost reply must not retain one pending request
                        # forever. Cancellation is idempotent; retry it while
                        # preserving the stop and requiring a new success ACK.
                        self.cancel_client.remove_pending_request(self.cancel_future)
                        self.cancel_future.cancel()
                        self.cancel_future = None
                if self.cancel_required:
                    self.stop_until = now+.2
                    if self.cancel_future is None and now-self.cancel_sent_at > 1.0 and self.cancel_client.service_is_ready():
                        self.cancel_future = self.cancel_client.call_async(Trigger.Request())
                        self.cancel_sent_at = now
                for kind, value in app.mission.drain_commands():
                    if kind == 'cancel':
                        self.cancel()
                    elif kind == 'stop':
                        self.stop_until = now+.8
                    elif kind == 'goal':
                        if self.goal_waiting is not None:
                            self.clear_auto_handover()
                        self.goal_waiting = value
                cancelled = (not self.cancel_required and self.cancel_future is None
                             and now-self.cancel_completed_at > .2 and now > self.stop_until)
                mux_held = (now-app.manual_mux_at < 1.
                            and app.manual_mux_status.get('source_id') == self.motion_source
                            and app.manual_mux_status.get('mode') in {'hold', 'manual'})
                self.tick_localization(now, cancelled, mux_held)
                self.tick_marker_alignment(now, cancelled, mux_held)
                if app.manual.state == 'ACQUIRING' and cancelled and mux_held:
                    app.manual.mark_acquired()
                if app.manual.state == 'RELEASING' and cancelled:
                    if app.manual.resume_mission and app.mission.state == 'PAUSED':
                        health = app.readiness(allow_camera_degraded=True)
                        if health['ready']:
                            app.manual.state = 'WAIT_REPLAN'
                            app.manual.reason = '기존 목표를 유지하고 현재 위치에서 경로를 다시 연결합니다.'
                            app.mission.resume()
                        else:
                            app.manual.reason = '임무를 유지하고 재개 조건을 기다립니다: '+', '.join(health['missing'])
                    else:
                        app.manual.finish()
                        self.motion_gate = True
                        self.send_control('hold')
                if app.manual.state == 'WAIT_REPLAN' and app.mission.state == 'PAUSED':
                    app.manual.state = 'RELEASING'
                # resume() appended the same goal after the main drain above.
                if app.manual.state == 'WAIT_REPLAN':
                    for kind, value in app.mission.drain_commands():
                        if kind == 'goal':
                            if self.goal_waiting is not None:
                                self.clear_auto_handover()
                            self.goal_waiting = value
                        elif kind == 'cancel':
                            self.cancel()
                        elif kind == 'stop':
                            self.stop_until = now+.8
                if app.mission.estop_latched:
                    self.sent_estop = True
                    self.stop()
                elif self.sent_estop:
                    self.sent_estop = False
                    # Reset is acknowledged by the shared service; no raw
                    # UI reset packet may bypass the estimator guard.
                    self.send_control('hold')
                    self.motion_gate = True
                elif app.manual.state == 'ACTIVE' and cancelled:
                    self.send_control('manual', app.manual.linear, app.manual.angular)
                elif app.localization.active:
                    if app.localization.state in app.localization.MOVING_STATES and cancelled:
                        self.send_control('localization', 0., self.localization_angular,
                                          operation='localization')
                    else:
                        self.stop()
                elif app.mission.state == 'CONFIRMING':
                    if cancelled and mux_held and self.marker_alignment_angular:
                        self.send_control('manual', 0., self.marker_alignment_angular,
                                          operation='marker_alignment')
                    else:
                        self.stop()
                elif (self.goal_waiting is not None and cancelled
                      and app.mission.state in {'NAVIGATING', 'RETURNING_HOME'}
                      and app.manual.state in {'IDLE', 'WAIT_REPLAN'}
                      and app.safety_request is None and self.safety_future is None):
                    # This is the only place that can publish a waiting goal.
                    # No later hold/ESTOP branch may overwrite its auto ACK.
                    self.tick_auto_handover(now)
                elif app.manual.state == 'WAIT_REPLAN' and self.owns_navigation:
                    # The new goal was sent after auto ACK. Wait for the
                    # matching MOVING status before finishing manual handoff.
                    pass
                elif app.manual.in_progress or now < self.stop_until or self.motion_gate:
                    self.stop()
                app.cancel_pending = self.cancel_required or self.cancel_future is not None

    return ROSMissionBridge()


class MissionHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, app, web_dir):
        self.app, self.web_dir = app, Path(web_dir).resolve()
        super().__init__(address, MissionHTTPRequestHandler)
        addresses = set()
        try:
            addresses.update(item[4][0] for item in socket.getaddrinfo(socket.gethostname(), 0, socket.AF_INET, socket.SOCK_STREAM))
        except OSError:
            pass
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                # connect selects a local interface; no UDP packet is sent.
                probe.connect(('192.0.2.1', 9))
                addresses.add(probe.getsockname()[0])
        except OSError:
            pass
        self.lan_urls = [f'http://{address}:{self.server_port}' for address in sorted(addresses)
                         if not ipaddress.ip_address(address).is_loopback]


class MissionHTTPRequestHandler(BaseHTTPRequestHandler):
    server_version = 'SmartVisionMission/1.0'
    protocol_version = 'HTTP/1.1'

    def setup(self):
        super().setup()
        self.connection.settimeout(10.0)

    def log_message(self, format_string, *args):
        if args and str(args[0]).startswith(('POST', 'PUT', 'DELETE')):
            sys.stderr.write('%s %s\n' % (self.address_string(), format_string % args))

    @property
    def local(self):
        try:
            return ipaddress.ip_address(self.client_address[0]).is_loopback
        except ValueError:
            return False

    def same_origin(self):
        host = self.headers.get('Host', '')
        try:
            parsed = urlsplit('http://'+host)
            if parsed.port != self.server.server_port or parsed.username or parsed.password or parsed.path:
                return False
            name = parsed.hostname
            actual = self.connection.getsockname()[0]
            if name == 'localhost':
                if not self.local:
                    return False
            elif ipaddress.ip_address(name) != ipaddress.ip_address(actual):
                return False
            origin = self.headers.get('Origin')
            return not origin or origin == 'http://'+host
        except (ValueError, TypeError):
            return False

    def send_bytes(self, content, content_type, status=200, headers=None):
        self.send_response(status)
        if status >= 400:
            self.close_connection = True
            self.send_header('Connection', 'close')
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'same-origin')
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def send_json(self, value, status=200, headers=None):
        self.send_bytes(json.dumps(value, ensure_ascii=False, allow_nan=False).encode(), 'application/json; charset=utf-8', status, headers)

    def protected(self):
        if self.server.app.authenticated(self.headers.get('Cookie'), self.local):
            return True
        message = ('차량의 ROS Domain ID를 입력해 연결하세요.' if self.server.app.access_mode == 'domain'
                   else 'PC 터미널에 표시된 연결 코드로 인증하세요.')
        self.send_json({'error': message, 'pairing_required': True}, 401)
        return False

    def operator_identity(self):
        """Opaque control ownership; never accept an identity from a JSON body."""
        header = self.headers.get('Cookie', '')
        if self.server.app.authenticated(header, local=False):
            return cookies.SimpleCookie(header)['amr_session'].value
        return 'local:'+self.client_address[0]

    def do_GET(self):
        if not self.same_origin():
            self.send_json({'error': 'Host/Origin이 일치하지 않습니다.'}, 403)
            return
        path = urlsplit(self.path).path
        app = self.server.app
        if path == '/api/access':
            self.send_json({'mode': app.access_mode})
            return
        if path == '/api/inventory/auth':
            user_id = app.inventory_identity(self.headers.get('Cookie'))
            self.send_json({'authenticated': user_id is not None, 'user_id': user_id,
                            'pairing_required': user_id is None, 'access_mode': app.access_mode})
            return
        if path.startswith('/api/inventory/'):
            user_id = app.inventory_identity(self.headers.get('Cookie'))
            if user_id is None:
                message = ('재고 관리자 번호와 차량의 ROS Domain ID로 접속하세요.' if app.access_mode == 'domain'
                           else '재고 관리자 번호와 전용 연결 코드로 접속하세요.')
                self.send_json({'error': message,
                                'pairing_required': True}, 401)
            elif path == '/api/inventory/state':
                self.send_json(app.inventory_state(user_id))
            else:
                self.send_json({'error': '없는 API입니다.'}, 404)
            return
        if path == '/api/auth':
            authenticated = app.authenticated(self.headers.get('Cookie'), self.local)
            self.send_json({'authenticated': authenticated, 'pairing_required': not authenticated,
                            'user_id': 0 if authenticated else None, 'access_mode': app.access_mode})
            return
        if path.startswith('/api/'):
            if not self.protected():
                return
            if path == '/api/state':
                state = app.state(local=self.local, owner=self.operator_identity())
                if self.local:
                    state['connection']['lan_urls'] = self.server.lan_urls
                    state['delivery']['inventory_urls'] = [url+'/inventory' for url in self.server.lan_urls]
                self.send_json(state)
            elif path == '/api/dispatch/state':
                self.send_json(app.state(local=self.local)['delivery'])
            elif path == '/api/health':
                self.send_json(app.state()['health'])
            elif path == '/api/markers':
                self.send_json({'markers': app.markers})
            elif path == '/api/map-layout':
                with app.lock:
                    layout = deepcopy(app.map_layout)
                if layout is None:
                    self.send_json({'error': '아직 수신된 지도가 없습니다.'}, 503)
                else:
                    self.send_json(layout)
            elif path in {'/api/map.png', '/api/camera.jpg', '/api/camera/front.jpg'}:
                with app.lock:
                    data = (app.map_png if path.endswith('.png') else
                            app.front_camera_jpeg if path.endswith('/front.jpg') else app.camera_jpeg)
                if data is None:
                    self.send_json({'error': '아직 수신된 데이터가 없습니다.'}, 503)
                else:
                    self.send_bytes(data, 'image/png' if path.endswith('.png') else 'image/jpeg')
            else:
                self.send_json({'error': '없는 API입니다.'}, 404)
            return
        # colcon --symlink-install intentionally links these public files through
        # build/ into source/. Resolve only an exact asset allowlist: arbitrary
        # URL paths must never become filesystem paths, including traversal.
        assets = {'/': 'index.html', '/index.html': 'index.html',
                  '/styles.css': 'styles.css', '/app.js': 'app.js',
                  '/favicon.svg': 'favicon.svg', '/inventory': 'inventory.html',
                  '/inventory.html': 'inventory.html', '/inventory.js': 'inventory.js',
                  '/inventory.css': 'inventory.css', '/power.js': 'power.js', '/joystick.js': 'joystick.js'}
        filename = assets.get(unquote(path))
        if filename is None:
            self.send_json({'error': '파일을 찾을 수 없습니다.'}, 404)
            return
        candidate = self.server.web_dir/filename
        try:
            if not candidate.is_file():
                raise FileNotFoundError()
            self.send_bytes(candidate.read_bytes(), mimetypes.guess_type(filename)[0] or 'application/octet-stream')
        except OSError:
            self.send_json({'error': '파일을 찾을 수 없습니다.'}, 404)

    def do_POST(self):
        if not self.same_origin():
            self.send_json({'error': 'Host/Origin이 일치하지 않습니다.'}, 403)
            return
        path = urlsplit(self.path).path
        inventory_user = None
        if path not in {'/api/pair', '/api/inventory/pair'}:
            if path.startswith('/api/inventory/'):
                inventory_user = self.server.app.inventory_identity(self.headers.get('Cookie'))
                if inventory_user is None:
                    self.send_json({'error': '재고 관리자 연결이 필요합니다.', 'pairing_required': True}, 401)
                    return
            elif not self.protected():
                return
        try:
            if self.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/json':
                self.send_json({'error': 'application/json 요청이 필요합니다.'}, 415)
                return
            if self.headers.get('Transfer-Encoding'):
                self.send_json({'error': 'chunked 요청을 지원하지 않습니다.'}, 400)
                return
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 65536:
                self.send_json({'error': 'JSON 요청 크기는 1~65536 바이트여야 합니다.'}, 413)
                return
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise MissionError('JSON 객체를 입력하세요.')
            if path == '/api/pair':
                token = self.server.app.pair(self.client_address[0], payload.get('code', ''),
                                             domain_id=payload.get('domain_id'))
                self.send_json({'authenticated': True, 'pairing_required': False}, headers={
                    'Set-Cookie': 'amr_session='+token+'; HttpOnly; SameSite=Strict; Path=/; Max-Age=86400'})
                return
            if path == '/api/inventory/pair':
                user_id = payload.get('user_id')
                token = self.server.app.pair_inventory(self.client_address[0], user_id, payload.get('code', ''),
                                                       domain_id=payload.get('domain_id'))
                self.send_json({'authenticated': True, 'user_id': user_id}, headers={
                    'Set-Cookie': 'amr_inventory_session='+token+'; HttpOnly; SameSite=Strict; Path=/api/inventory; Max-Age=86400'})
                return
            if path == '/api/inventory/logout':
                try:
                    token = cookies.SimpleCookie(self.headers.get('Cookie', '')).get('amr_inventory_session')
                    if token:
                        with self.server.app.lock:
                            self.server.app.inventory_sessions.pop(token.value, None)
                except cookies.CookieError:
                    pass
                self.send_json({'authenticated': False}, headers={
                    'Set-Cookie': 'amr_inventory_session=; HttpOnly; SameSite=Strict; Path=/api/inventory; Max-Age=0'})
                return
            if path == '/api/manual':
                self.send_json(self.server.app.manual_command(self.operator_identity(), payload))
                return
            if inventory_user is not None:
                self.send_json(self.server.app.inventory_command(path, payload, inventory_user))
                return
            self.send_json(self.server.app.command(path, payload))
        except PermissionError as exc:
            self.send_json({'error': str(exc)}, 403)
        except (MissionError, ValueError, TypeError) as exc:
            self.send_json({'error': str(exc)}, 400)
        except KeyError:
            self.send_json({'error': '없는 API입니다.'}, 404)
        except OSError as exc:
            self.send_json({'error': '설정 저장에 실패했습니다: '+str(exc)}, 500)


def build_parser():
    parser = argparse.ArgumentParser(description='ArUco mission web app with ROS2 navigation and live camera verification')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8790)
    parser.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '76')))
    parser.add_argument('--markers-file')
    parser.add_argument('--map-file', help='Nav2 trinary map YAML: preview only until the matching ROS /map arrives')
    parser.add_argument('--floorplan-file', help='Display-only authored polygons in the configured --map-file coordinate frame')
    parser.add_argument('--web-dir')
    parser.add_argument('--config-file')
    parser.add_argument('--access-mode', choices=('code', 'domain'), default='code',
                        help='code: separate pairing PINs; domain: join using the current ROS domain')
    parser.add_argument('--camera-topic', default='/camera/image_raw')
    parser.add_argument('--front-camera-topic', default='/camera/front/image_raw',
                        help='Optional front Image preview; arrival camera is selected in marker configuration')
    parser.add_argument('--camera-rotation', type=int, choices=(0, 180), default=0,
                        help='Rotate incoming camera frames for ArUco detection and web preview (degrees)')
    parser.add_argument('--front-camera-rotation', type=int, choices=(0, 180), default=0,
                        help='Rotate the front preview and ArUco detection (degrees)')
    parser.add_argument('--battery-topic', default='/battery_state',
                        help='sensor_msgs/BatteryState topic used for the vehicle voltage indicator')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--confirmations', type=int, default=3)
    parser.add_argument('--marker-search-speed', type=float, default=.12,
                        help='Arrival marker search angular speed, rad/s (max .15)')
    parser.add_argument('--marker-center-gain', type=float, default=.35,
                        help='Yaw gain on normalized horizontal pixel error')
    parser.add_argument('--marker-center-tolerance', type=float, default=.08,
                        help='Normalized image center tolerance; .08 is +/-4%% image width')
    parser.add_argument('--marker-search-timeout', type=float, default=45.,
                        help='Maximum seconds for one arrival marker search attempt')
    parser.add_argument('--require-marker-wall-alignment', action='store_true',
                        help='Require fresh LiDAR wall-normal alignment and centered marker before arrival')
    parser.add_argument('--marker-front-camera-pose', type=float, nargs=3, default=(0., 0., 0.),
                        metavar=('X', 'Y', 'YAW'), help='Front camera optical-axis mount in base frame (m,m,rad)')
    parser.add_argument('--marker-rear-camera-pose', type=float, nargs=3, default=(0., 0., math.pi),
                        metavar=('X', 'Y', 'YAW'), help='Rear camera optical-axis mount in base frame (m,m,rad)')
    parser.add_argument('--marker-front-yaw-sign', type=float, choices=(-1., 1.), default=-1.)
    parser.add_argument('--marker-rear-yaw-sign', type=float, choices=(-1., 1.), default=-1.,
                        help='Upright non-mirrored front and rear images both use -1')
    parser.add_argument('--with-motion-mux', action='store_true',
                        help='Start a separate cmd_vel arbiter process (legacy PC deployment only)')
    parser.add_argument('--require-localization-health', action='store_true',
                        help='Gate autonomous dispatch on current localization quality')
    parser.add_argument('--require-battery-health', action='store_true',
                        help='Gate physical autonomous dispatch on fresh valid /battery_state')
    parser.add_argument('--minimum-battery-voltage', type=float, default=0.0,
                        help='Physical departure minimum in volts; 0 disables the voltage floor')
    clock = parser.add_mutually_exclusive_group()
    clock.add_argument('--use-sim-time', action='store_true', dest='use_sim_time')
    clock.add_argument('--no-use-sim-time', action='store_false', dest='use_sim_time')
    parser.set_defaults(use_sim_time=False)
    return parser


def main(args=None):
    options = build_parser().parse_args(args)
    if not 0 <= options.domain_id <= 232 or not 1 <= options.port <= 65535:
        raise SystemExit('domain_id=0..232, port=1..65535 required')
    if not math.isfinite(options.minimum_battery_voltage) or options.minimum_battery_voltage < 0.0:
        raise SystemExit('minimum_battery_voltage must be finite and nonnegative')
    markers = load_markers(options.markers_file)
    if options.web_dir is None:
        from ament_index_python.packages import get_package_share_directory
        options.web_dir = str(Path(get_package_share_directory('smart_vision_amr'))/'web')
    app = MissionApplication(markers, options.domain_id, options.use_sim_time, options.config_file, options.confirmations,
                             access_mode=options.access_mode)
    app.require_localization_health = options.require_localization_health
    app.require_battery_health = options.require_battery_health
    app.minimum_battery_voltage = options.minimum_battery_voltage
    if options.map_file:
        app.load_map_preview(options.map_file)
    if options.floorplan_file:
        app.load_floorplan(options.floorplan_file)
    server = MissionHTTPServer((options.host, options.port), app, options.web_dir)
    server_thread = threading.Thread(target=server.serve_forever, name='mission-http', daemon=True)
    server_thread.start()
    print(f'웹 미션: http://{options.host}:{options.port}  ROS_DOMAIN_ID={options.domain_id}', flush=True)
    if app.access_mode == 'domain':
        print(f'ROS Domain ID {app.domain_id}로 접속 · 운용자: / · 재고 관리자 1~99: /inventory', flush=True)
    else:
        print(f'휴대폰 연결 코드: {app.pairing_code} (이 PC의 IP 주소로 접속)', flush=True)
        print(f'재고 관리자 1~99 연결 코드: {app.inventory_join_code}  주소: /inventory', flush=True)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    stop_requested = threading.Event()
    old_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, lambda *_: stop_requested.set())
    node = mux_process = context = executor = None
    def stop_mux_process():
        nonlocal mux_process
        if mux_process is not None and mux_process.poll() is None:
            os.killpg(mux_process.pid, signal.SIGTERM)
            try:
                mux_process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(mux_process.pid, signal.SIGKILL)
                mux_process.wait()
        mux_process = None
    try:
        while not stop_requested.is_set():
            if node is None:
                context = Context()
                rclpy.init(args=[], context=context, domain_id=app.domain_id, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
                try:
                    node = create_ros_bridge(app, options, context)
                    executor = SingleThreadedExecutor(context=context)
                    executor.add_node(node)
                    if options.with_motion_mux:
                        env = dict(os.environ, ROS_DOMAIN_ID=str(app.domain_id))
                        mux_process = subprocess.Popen(
                            [sys.executable, '-m', 'smart_vision_amr.motion_mux', '--ros-args',
                             '-p', 'use_sim_time:='+str(app.use_sim_time).lower(),
                             '-p', 'allow_legacy_auto_input:=false',
                             '-p', 'require_localization_health:='
                                   +str(options.require_localization_health).lower(),
                             '-p', 'startup_estop:='+str(options.require_localization_health).lower(),
                             '-p', 'allow_topic_reset:='+str(not options.require_localization_health).lower()],
                            env=env, start_new_session=True)
                except Exception:
                    context.shutdown()
                    raise
            executor.spin_once(timeout_sec=.1)
            with app.lock:
                pending = app.pending_connection
            if (pending and not node.owns_navigation and not node.cancel_required
                    and node.cancel_future is None and node.goal_waiting is None
                    and not any(kind == 'cancel' for kind, _ in app.mission.commands)):
                node.stop()
                node.destroy_node()
                stop_mux_process()
                context.shutdown()
                node = context = executor = None
                with app.lock:
                    app.domain_id, app.use_sim_time = pending
                    app.pending_connection = None
                    app.connected = False
                    app.pose = None
                    app.pose_tf_stamp_ns = 0
                    app.pose_at = app.odom_at = app.scan_at = app.camera_at = app.front_camera_at = 0.0
                    app.reset_power()
                    app.camera_jpeg = None
                    app.front_camera_jpeg = None
                    app.restore_map_preview()
                    app.path, app.detected_ids, app.front_detected_ids = [], [], []
                    app.goal_available = app.cancel_available = app.planner_available = False
                    app.cancel_pending = app.navigation_active = False
                    app.manual.finish()
                    app.manual_mux_at = 0.
                    app.manual_mux_status = {}
                    app.localization_health = {}
                    app.localization_health_at = 0.
                    app.safety_request = None
                    app.home_seed_request = None
                    app.home_seed_match = None
                    app.home_seed_match_at = 0.
                    app.home_seed_match_key = None
                    app.home_prior_evidence = None
                    app.home_seed_result = ''
                    app.manual_pose_service_ready = False
                    app.manual_pose_request = app.manual_pose_requested = None
                    app.manual_pose_state = 'IDLE'
                    app.manual_pose_result = ''
                    app.manual_pose_sent_at = app.manual_pose_amcl_at = 0.
                    app.manual_pose_amcl_received_at = 0.
                    app.manual_pose_match = None
                    app.manual_pose_map_version = 0
                    app.manual_cancel_requested = app.manual_auto_ready = False
                    app.localization = AssistedLocalization()
                    app.localization_requested = app.localization_stop_requested = False
                    app.localization_services_ready = app.localization_service_pending = False
                    app.localization_control = {'state': 'IDLE', 'reason': ''}
                    app.localization_amcl_active_at = app.localization_scan_at = app.localization_odom_at = 0.
                    app.localization_clear = app.localization_invalidated = False
                    app.localization_use_existing_pose = False
                    app.localization_startup_home = False
                    app.localization_home_seed = None
                    app.localization_reference_pose = app.localization_reference_odom = app.odom_pose = None
                    app.localization_clearance = app.localization_match = app.localization_distance = None
                    app.mission = app._new_mission()
                    app.delivery = DeliveryService(app)
                    app.save()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # An idle viewer must not cancel a goal sent by another controller.
            if (node.owns_navigation or node.cancel_required or node.goal_waiting is not None
                    or app.mission.state in ACTIVE_STATES or app.mission.state in {'PAUSED', 'ESTOP'}
                    or app.manual.in_progress or app.localization.active
                    or any(kind == 'cancel' for kind, _ in app.mission.commands)):
                app.cancel_localization('웹 서버 종료로 위치 확인을 중단했습니다.')
                node.cancel()
                node.stop()
                deadline = time.monotonic()+1.0
                while node.cancel_future is not None and not node.cancel_future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.05)
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
        stop_mux_process()
        if context is not None and context.ok():
            context.shutdown()
        with app.lock:
            app.connected = False
        server.shutdown()
        server.server_close()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
