#!/usr/bin/env python3
"""ROS 2 Humble / PyQt5 hold-to-drive controller (no colcon build required).

source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=64 python3 cmd_vel_controller.py
Optional ROS remap: --ros-args -r cmd_vel:=/robot/cmd_vel
Requires a desktop display or SSH X11 forwarding. No obstacle avoidance.
"""
import argparse
import os
import signal
import sys
import time
import threading
import math

from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy

from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from navigation_panels import RosBridge, MapPanel, ParameterPanel


SETTINGS_PATH = os.path.expanduser('~/.config/turtlebot3/cmd_vel_controller.ini')


class LazyVelocityPublisher:
    """Keep map/parameter viewing passive and respect the web mux's output."""
    def __init__(self, node, topic='cmd_vel'):
        self.node = node
        self.topic = topic
        self.topic_name = node.resolve_topic_name(topic)
        self.publisher = None
        self.blocked_reason = ''
        self.mux_owns_output = False

    def manual_available(self):
        try:
            endpoints = self.node.get_publishers_info_by_topic(self.topic_name)
            mux_owns_output = any(
                item.node_name == 'motion_mux' or item.node_name.endswith('_motion_mux')
                for item in endpoints)
        except Exception:
            self.mux_owns_output = None
            self.blocked_reason = '속도 토픽 소유권 확인 실패 · 연결 상태를 확인하세요.'
            self.close()
            return False
        if mux_owns_output:
            self.mux_owns_output = True
            self.blocked_reason = '웹 조이스틱이 수동 조종을 담당합니다. PyQt 방향 버튼은 잠겨 있습니다.'
            self.close()
            return False
        self.blocked_reason = ''
        self.mux_owns_output = False
        return True

    def publish(self, message):
        if not self.manual_available():
            return False
        moving = bool(message.linear.x or message.angular.z)
        if self.publisher is None:
            if not moving:
                return True  # Idle/close must not claim /cmd_vel ownership.
            self.publisher = self.node.create_publisher(Twist, self.topic, 1)
        self.publisher.publish(message)
        return True

    def close(self):
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None


def domain_settings(path=SETTINGS_PATH):
    return QtCore.QSettings(path, QtCore.QSettings.IniFormat)


def initial_domain(settings):
    # A saved UI selection takes precedence over the terminal's environment.
    for candidate in (settings.value('ros/domain_id'), os.getenv('ROS_DOMAIN_ID'), 64):
        try:
            value = int(candidate)
            if 0 <= value <= 232:
                return value
        except (TypeError, ValueError):
            pass
    return 64


def initial_sim_time(settings, domain, override=None):
    """Clock choice belongs to the ROS domain, independently of its number."""
    if override is not None:
        return override
    return str(settings.value(f'ros/domain_{domain}/use_sim_time', 'false')).lower() == 'true'


def navigation_profile_key(domain, namespace):
    return f'vehicles/domain_{domain}/{namespace.strip(chr(47)) or "root"}/navigation_profile'


def initial_navigation_profile(settings, domain, namespace, override=None):
    profile = override or str(settings.value(navigation_profile_key(domain, namespace), 'auto'))
    return profile if profile in ('auto', 'nav2', 'team-astar') else 'auto'


def reconnect_arguments(options, domain, namespace, use_sim_time, profile):
    """Recreate the ROS context with the selected connection and saved UI state."""
    arguments = [sys.executable, os.path.abspath(__file__), '--domain-id', str(domain),
                 '--robot-namespace', namespace, '--navigation-profile', profile,
                 '--use-sim-time' if use_sim_time else '--no-use-sim-time']
    if options.settings_file:
        arguments.extend(['--settings-file', options.settings_file])
    # Camera and parameter sources are remembered by the window in their own
    # domain/vehicle scope. Do not reapply the previous vehicle's CLI defaults.
    if options.navigation_cancel_service:
        arguments.extend(['--navigation-cancel-service', options.navigation_cancel_service])
    return arguments


def decode_camera_image(msg):
    """Use Qt's built-in image decoder; avoid importing OpenCV into the GUI."""
    if isinstance(msg, CompressedImage):
        frame = QtGui.QImage.fromData(bytes(msg.data))
        if frame.isNull():
            raise ValueError('JPEG/PNG 압축 영상을 해석할 수 없습니다.')
        return frame.convertToFormat(QtGui.QImage.Format_RGB888)
    formats = {
        'rgb8': (QtGui.QImage.Format_RGB888, 3, False),
        'bgr8': (QtGui.QImage.Format_RGB888, 3, True),
        'mono8': (QtGui.QImage.Format_Grayscale8, 1, False),
        'rgba8': (QtGui.QImage.Format_RGBA8888, 4, False),
        'bgra8': (QtGui.QImage.Format_RGBA8888, 4, True),
    }
    if msg.encoding not in formats:
        raise ValueError(f'지원하지 않는 영상 형식: {msg.encoding}. rgb8 또는 압축 영상을 선택하세요.')
    fmt, channels, swap = formats[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*channels or len(msg.data) < msg.step*msg.height:
        raise ValueError('영상 크기/stride/데이터 길이가 잘못되었습니다.')
    frame = QtGui.QImage(bytes(msg.data), msg.width, msg.height, msg.step, fmt).copy()
    return frame.rgbSwapped() if swap else frame


class CameraFeed:
    """Decode in a dedicated ROS thread; GUI only consumes the latest frame."""
    def __init__(self, max_fps=8.0, start_thread=True, clock=time.monotonic):
        if not math.isfinite(max_fps) or max_fps <= 0:
            raise ValueError('Camera preview frame rate must be finite and positive')
        # This is only a display consumer. Upstream ROS images are untouched.
        self.frame_interval = 1.0/min(float(max_fps), 10.0)
        self.clock = clock
        self.last_decode_at = -math.inf
        self.skipped_frames = 0
        self.request = None
        self.latest = None
        self.status = '카메라 연결 대기'
        self.closed = threading.Event()
        self.thread = None
        if start_thread:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def connect(self, topic, compressed):
        self.latest = None
        self.request = (topic, compressed)
        self.last_decode_at = -math.inf
        self.status = '영상 수신 대기'

    def receive(self, message, token):
        """Drop old-topic/excess frames before JPEG decode or raw-image copy."""
        now = self.clock()
        if token != self.request or now-self.last_decode_at < self.frame_interval:
            self.skipped_frames += 1
            return False
        self.last_decode_at = now
        try:
            frame = decode_camera_image(message)
            if token != self.request:
                return False
            self.latest = (frame, now)
            self.status = f'{frame.width()} × {frame.height()}'
            return True
        except Exception as exc:
            self.status = f'영상 변환 오류: {exc}'
            return False

    def run(self):
        node = Node('turtlebot3_controller_camera')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        sub = None
        active = None
        try:
            while not self.closed.is_set() and rclpy.ok():
                if self.request != active:
                    if sub is not None:
                        node.destroy_subscription(sub)
                        sub = None
                    active = self.request
                    if active:
                        topic, compressed = active
                        def receive(msg, token=active):
                            self.receive(msg, token)
                        try:
                            sub = node.create_subscription(
                                CompressedImage if compressed else Image, topic, receive,
                                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                        except Exception as exc:
                            self.status = f'연결 오류: {exc}'
                executor.spin_once(timeout_sec=.1)
        except Exception as exc:
            self.status = f'카메라 수신 오류: {exc}'
        finally:
            executor.shutdown()
            node.destroy_node()

    def close(self):
        self.closed.set()
        if self.thread is not None:
            self.thread.join(timeout=2)


class Controller(QtWidgets.QWidget):
    vehicleRequested = QtCore.pyqtSignal(str)
    connectionRequested = QtCore.pyqtSignal(int, str, bool, str)
    def __init__(self, publisher, domain_id=None, settings=None, camera=None,
                 node=None, camera_topic=None, navigation_cancel_service=None,
                 parameter_configs=None, use_sim_time=None, navigation_profile=None):
        super().__init__()
        self.publisher = publisher
        self.camera = camera
        self.ros_node = node
        self.vehicle_namespace = node.get_namespace().rstrip('/') if node is not None else ''
        self.settings = settings if settings is not None else domain_settings()
        self.domain_id = initial_domain(self.settings) if domain_id is None else domain_id
        self.use_sim_time = initial_sim_time(self.settings, self.domain_id, use_sim_time)
        self.navigation_profile = initial_navigation_profile(
            self.settings, self.domain_id, self.vehicle_namespace, navigation_profile)
        self.bridge = (RosBridge(node, cancel_service=navigation_cancel_service,
                                 navigation_profile=self.navigation_profile)
                       if node is not None else None)
        self.keys = set()
        self.active = None
        self.manual_override_active = False
        self.direction = (0, 0)
        self.stop_ticks = 0
        self.safety_clients = {}
        self.safety_request = None
        self.last_safety_ack_at = None
        self.setWindowTitle('TurtleBot3 · 카메라 / 파라미터 / 지도 / 조종')
        # Keep the application usable on smaller displays.  The dashboard below
        # owns its natural minimum size, so a compact window gets scroll bars
        # instead of forcing buttons and input fields below their minimum size.
        self.setMinimumSize(640, 480)
        self.setStyleSheet('''
            QWidget { background: white; color: black; font-size: 13px; }
            QPushButton, QToolButton { background: white; border: 1px solid black;
                          border-radius: 0px; padding: 3px; }
            QPushButton:pressed, QToolButton:pressed { background: #e5e5e5; }
            QDoubleSpinBox, QSpinBox, QComboBox { background: white; color: black;
                border: 1px solid black; padding: 7px; border-radius: 0px; }
            QToolTip { background: white; color: black; border: 1px solid black; padding: 5px; }
        ''')
        window_layout = QtWidgets.QVBoxLayout(self)
        window_layout.setContentsMargins(0, 0, 0, 0)
        self.dashboard_scroll = QtWidgets.QScrollArea()
        self.dashboard_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.dashboard_scroll.setWidgetResizable(True)
        self.dashboard = QtWidgets.QWidget()
        outer = QtWidgets.QGridLayout(self.dashboard)
        self.dashboard_scroll.setWidget(self.dashboard)
        window_layout.addWidget(self.dashboard_scroll)
        controls = QtWidgets.QGroupBox('4. 수동 조종기')
        self.controls = controls
        layout = QtWidgets.QVBoxLayout(controls)
        outer.addWidget(controls, 1, 1)
        camera_box = QtWidgets.QGroupBox('1. 카메라')
        panel = QtWidgets.QVBoxLayout(camera_box)
        outer.addWidget(camera_box, 0, 0)
        self.parameters = ParameterPanel(
            self.bridge, self.domain_id, self.stop, parameter_configs=parameter_configs,
            settings=self.settings)
        self.map_panel = MapPanel(
            self.bridge, self.stop, domain=self.domain_id)
        outer.addWidget(self.parameters, 0, 1)
        outer.addWidget(self.map_panel, 1, 0)
        for index in range(2):
            outer.setRowStretch(index, 1)
            outer.setColumnStretch(index, 1)
        panel.addWidget(QtWidgets.QLabel('실시간 카메라'))
        row = QtWidgets.QHBoxLayout()
        self.camera_topic = QtWidgets.QComboBox()
        self.camera_topic.setEditable(True)
        self.camera_settings_prefix = f'vehicles/{self.domain_id}/{self.vehicle_namespace.strip(chr(47)) or "root"}/camera/'
        legacy_camera_prefix = (f'vehicles/{self.domain_id}/{self.vehicle_namespace.strip(chr(47))}/camera/'
                                if self.vehicle_namespace else 'camera/')
        saved_camera_topic = self.settings.value(
            self.camera_settings_prefix + 'topic',
            self.settings.value(legacy_camera_prefix + 'topic', self.vehicle_namespace + '/camera/image_raw'),
        )
        self.camera_topic.addItem(camera_topic or str(saved_camera_topic))
        row.addWidget(self.camera_topic, 1)
        refresh = QtWidgets.QPushButton('토픽 검색')
        refresh.setStyleSheet('min-height: 32px; padding: 4px;')
        refresh.clicked.connect(self.refresh_camera_topics)
        row.addWidget(refresh)
        panel.addLayout(row)
        self.compressed = QtWidgets.QCheckBox('압축 영상 (CompressedImage)')
        # An explicit launcher topic selects its matching transport, overriding
        # a preference saved by a previous raw/compressed camera session.
        self.compressed.setChecked(
            camera_topic.endswith('/compressed') if camera_topic is not None else
            str(self.settings.value(self.camera_settings_prefix+'compressed',
                self.settings.value(legacy_camera_prefix+'compressed', 'false'))).lower() == 'true')
        panel.addWidget(self.compressed)
        connect = QtWidgets.QPushButton('카메라 연결')
        connect.setStyleSheet('min-height: 32px; padding: 4px;')
        connect.clicked.connect(self.connect_camera)
        panel.addWidget(connect)
        self.video = QtWidgets.QLabel('카메라 영상 대기')
        self.video.setAlignment(QtCore.Qt.AlignCenter)
        self.video.setMinimumSize(320, 180)
        self.video.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.video.setStyleSheet('background: white; color: black; border: 1px solid black;')
        panel.addWidget(self.video, 1)
        self.camera_status = QtWidgets.QLabel('카메라 연결 대기')
        self.camera_status.setWordWrap(True)
        panel.addWidget(self.camera_status)
        self.resize(1280, 960)
        self.video_timer = QtCore.QTimer(self)
        self.video_timer.timeout.connect(self.render_camera)
        self.video_timer.start(125)
        self._rendered_camera = None
        if camera is not None:
            QtCore.QTimer.singleShot(0, self.connect_camera)
        title = QtWidgets.QLabel('TurtleBot3 수동 조종')
        title.setStyleSheet('font-size: 18px; font-weight: bold;')
        layout.addWidget(title)
        layout.addWidget(QtWidgets.QLabel('버튼 / 화살표 키를 누르는 동안 이동 · 놓으면 정지'))
        layout.addWidget(QtWidgets.QLabel(
            f'Topic: {publisher.topic_name}  |  연결된 Domain: {self.domain_id}'))
        vehicle_row = QtWidgets.QHBoxLayout()
        vehicle_row.addWidget(QtWidgets.QLabel('차량 namespace'))
        self.vehicle_box = QtWidgets.QComboBox()
        self.vehicle_box.setEditable(True)
        self.vehicle_box.addItem(self.vehicle_namespace or '/')
        vehicle_row.addWidget(self.vehicle_box, 1)
        discover = QtWidgets.QPushButton('차량 검색')
        discover.clicked.connect(self.discover_vehicles)
        vehicle_row.addWidget(discover)
        layout.addLayout(vehicle_row)
        domain_row = QtWidgets.QHBoxLayout()
        domain_row.addWidget(QtWidgets.QLabel('ROS Domain ID'))
        self.domain_input = QtWidgets.QSpinBox()
        self.domain_input.setRange(0, 232)
        self.domain_input.setValue(self.domain_id)
        domain_row.addWidget(self.domain_input)
        self.sim_time = QtWidgets.QCheckBox('시뮬레이션 /clock')
        self.sim_time.setChecked(self.use_sim_time)
        self.sim_time.setToolTip('Gazebo 등 /clock을 발행하는 시뮬레이션에서만 선택하세요. Domain별로 저장됩니다.')
        domain_row.addWidget(self.sim_time)
        self.domain_save = QtWidgets.QPushButton('저장 · 연결')
        self.domain_save.setToolTip('정지 후 새 Domain/차량에 다시 연결합니다. 경유점·미저장 파라미터 목록은 초기화됩니다.')
        self.domain_save.setStyleSheet('min-height: 36px; padding: 4px 12px;')
        self.domain_save.clicked.connect(self.save_domain)
        domain_row.addWidget(self.domain_save)
        layout.addLayout(domain_row)
        profile_row = QtWidgets.QHBoxLayout()
        profile_row.addWidget(QtWidgets.QLabel('내비게이션'))
        self.profile_box = QtWidgets.QComboBox()
        for label, value in [('자동 감지', 'auto'), ('Generic Nav2', 'nav2'), ('Team A*', 'team-astar')]:
            self.profile_box.addItem(label, value)
        self.profile_box.setCurrentIndex(self.profile_box.findData(self.navigation_profile))
        self.profile_box.setToolTip('자동 감지: smart_vision_amr/goal 수신 노드가 있으면 Team A*에 연결합니다.')
        profile_row.addWidget(self.profile_box, 1)
        layout.addLayout(profile_row)
        self.vehicle_box.currentTextChanged.connect(self.load_vehicle_preferences)
        self.domain_input.valueChanged.connect(self.load_connection_preferences)
        domain_hint = QtWidgets.QLabel('같은 네트워크의 로봇 Domain을 입력하고 저장 · 연결을 누르세요.')
        domain_hint.setStyleSheet('font-size: 12px; color: black;')
        layout.addWidget(domain_hint)
        speed_row = QtWidgets.QFormLayout()
        self.linear_speed = QtWidgets.QDoubleSpinBox()
        self.linear_speed.setRange(0.01, 0.22)
        self.linear_speed.setSingleStep(0.01)
        self.linear_speed.setValue(0.10)
        self.linear_speed.setSuffix(' m/s')
        self.angular_speed = QtWidgets.QDoubleSpinBox()
        self.angular_speed.setRange(0.10, 1.00)
        self.angular_speed.setSingleStep(0.05)
        self.angular_speed.setValue(0.50)
        self.angular_speed.setSuffix(' rad/s')
        speed_row.addRow('전진 / 후진 속도', self.linear_speed)
        speed_row.addRow('회전 / 선회 속도', self.angular_speed)
        layout.addLayout(speed_row)
        grid = QtWidgets.QGridLayout()
        self.buttons = {}
        for name, text, row, col, direction in [
                ('forward', '▲ 전진', 0, 1, (1, 0)),
                ('forward_left', '◤ 좌선회', 0, 0, (1, 1)),
                ('forward_right', '우선회 ◥', 0, 2, (1, -1)),
                ('left', '◀ 좌회전', 1, 0, (0, 1)),
                ('right', '우회전 ▶', 1, 2, (0, -1)),
                ('backward_left', '◣ 후진 좌', 2, 0, (-1, 1)),
                ('backward_right', '후진 우 ◢', 2, 2, (-1, -1)),
                ('backward', '▼ 후진', 2, 1, (-1, 0))]:
            button = QtWidgets.QPushButton(text)
            button.setMinimumHeight(46)
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.pressed.connect(lambda b=button, d=direction: self.start(b, d))
            button.released.connect(
                lambda: self.stop(resume_autonomous=True))
            grid.addWidget(button, row, col)
            self.buttons[name] = button
        stop_button = QtWidgets.QPushButton('■ 정지')
        stop_button.setStyleSheet('QPushButton { background: #e53935; color: white; } '
                                 'QPushButton:pressed, QToolButton:pressed { background: #b71c1c; }')
        stop_button.setMinimumHeight(46)
        stop_button.pressed.connect(self.emergency_stop)
        grid.addWidget(stop_button, 1, 1)
        layout.addLayout(grid)
        self.emergency_reset_button = QtWidgets.QPushButton('비상정지 해제 · 출발 대기 유지')
        self.emergency_reset_button.clicked.connect(self.reset_emergency_stop)
        self.emergency_reset_button.setToolTip(
            '차량의 공통 안전 제어기에 해제를 요청합니다. 센서가 정상일 때만 해제되며 자동 출발하지 않습니다.')
        layout.addWidget(self.emergency_reset_button)
        self.status = QtWidgets.QLabel('정지')
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.status)
        hint = QtWidgets.QLabel('좌·우: 제자리 회전 | ↑+←/→ 또는 모서리 버튼: 선회 | Space / Esc: 비상정지 + Nav2 취소\n공통 안전 제어기 연결 시 정지 확인 응답을 표시합니다. 수동 조종에는 장애물 회피가 없습니다.')
        hint.setWordWrap(True)
        hint.setStyleSheet('font-size: 13px; color: black;')
        layout.addWidget(hint)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        QtWidgets.QApplication.instance().installEventFilter(self)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)  # 20 Hz, using wall time rather than simulation time.
        # QGridLayout may otherwise compress its children when the top-level
        # window becomes smaller than the complete 2 x 2 dashboard.
        outer.activate()
        self.dashboard.setMinimumSize(self.dashboard.minimumSizeHint())

    def eventFilter(self, obj, event):
        if not isinstance(obj, QtWidgets.QWidget) or obj.window() is not self:
            return False
        kind = event.type()
        if kind in (QtCore.QEvent.WindowDeactivate, QtCore.QEvent.FocusOut):
            self.stop()
        if kind not in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease,
                        QtCore.QEvent.ShortcutOverride):
            return False
        key = event.key()
        directions = {QtCore.Qt.Key_Up: (1, 0), QtCore.Qt.Key_Down: (-1, 0),
                      QtCore.Qt.Key_Left: (0, 1), QtCore.Qt.Key_Right: (0, -1)}
        if key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
            if kind == QtCore.QEvent.KeyPress:
                self.emergency_stop()
            event.accept()
            return True
        if key not in directions:
            return False
        # Releases must stop even if focus moved into an input field.
        if kind == QtCore.QEvent.KeyRelease:
            if not event.isAutoRepeat() and key in self.keys:
                self.keys.discard(key)
                if self.keys:
                    self.direction = self.combine(directions[k] for k in self.keys)
                else:
                    self.stop(resume_autonomous=True)
            return False
        editing = isinstance(obj, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                                   QtWidgets.QComboBox))
        if editing or event.modifiers() != QtCore.Qt.NoModifier:
            return False
        if kind == QtCore.QEvent.ShortcutOverride:
            event.accept()
            return True
        if not self.manual_drive_ready():
            event.accept()
            return True
        if not event.isAutoRepeat():
            if not self.keys:
                self.stop()
            # Holding ↑/↓ and pressing ←/→ combines into an arc turn.
            self.keys.add(key)
            self.direction = self.combine(directions[k] for k in self.keys)
            self.manual_override_active = True
            self.stop_ticks = 0
            self.tick()
        return True

    @staticmethod
    def combine(directions):
        linear = angular = 0
        for d in directions:
            linear += d[0]
            angular += d[1]
        return (max(-1, min(1, linear)), max(-1, min(1, angular)))

    def discover_vehicles(self):
        self.stop()
        selected = self.vehicle_box.currentText()
        namespaces = {'/', self.vehicle_namespace or '/'}
        if self.ros_node:
            for topic, types in self.ros_node.get_topic_names_and_types():
                if topic.endswith('/cmd_vel') and 'geometry_msgs/msg/Twist' in types:
                    namespaces.add(topic[:-len('/cmd_vel')] or '/')
        self.vehicle_box.clear()
        self.vehicle_box.addItems(sorted(namespaces))
        self.vehicle_box.setCurrentText(selected)
        self.status.setText('차량 namespace를 선택하고 연결하세요. 같은 Domain의 /cmd_vel 기준 검색')

    def select_vehicle(self):
        self.save_domain()

    def connection_change_allowed(self):
        if self.bridge and getattr(self.bridge, 'maintenance', False):
            self.status.setText('리셋 종료 후 연결을 전환하세요.')
            return False
        if self.parameters.write_pending:
            self.status.setText('파라미터 적용 완료 후 차량을 전환하세요.')
            return False
        return True

    def load_connection_preferences(self, domain):
        namespace = str(self.settings.value(f'vehicles/domain_{domain}/namespace', '/'))
        self.vehicle_box.setCurrentText(namespace)
        self.sim_time.setChecked(initial_sim_time(self.settings, domain))
        profile = initial_navigation_profile(self.settings, domain, namespace)
        self.profile_box.setCurrentIndex(self.profile_box.findData(profile))

    def load_vehicle_preferences(self, namespace):
        profile = initial_navigation_profile(self.settings, self.domain_input.value(), namespace)
        self.profile_box.setCurrentIndex(self.profile_box.findData(profile))

    def refresh_camera_topics(self):
        self.stop()
        if self.ros_node is None:
            return
        selected = self.camera_topic.currentText()
        topics = [name for name, types in self.ros_node.get_topic_names_and_types()
                  if name.startswith(self.vehicle_namespace+'/') and any(t in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage') for t in types)]
        self.camera_topic.clear()
        self.camera_topic.addItems(sorted(set(topics + [selected])))
        self.camera_topic.setCurrentText(selected)
        self.camera_status.setText(f'영상 토픽 {len(topics)}개 발견 · 선택 후 연결')

    def connect_camera(self):
        self.stop()
        topic = self.camera_topic.currentText().strip()
        if not topic or self.camera is None:
            return
        compressed = self.compressed.isChecked() or topic.endswith('/compressed')
        self.compressed.setChecked(compressed)
        self.settings.setValue(self.camera_settings_prefix+'topic', topic)
        self.settings.setValue(self.camera_settings_prefix+'compressed', compressed)
        self.settings.sync()
        self.camera.connect(topic, compressed)
        self.video.clear()
        self.video.setText('영상 수신 대기')

    def render_camera(self):
        if self.camera is None:
            return
        latest = self.camera.latest
        if latest is not None and time.monotonic()-latest[1] < 2:
            display_key = (id(latest[0]), latest[1], self.video.width(), self.video.height())
            if display_key != self._rendered_camera:
                self.video.setPixmap(QtGui.QPixmap.fromImage(latest[0]).scaled(
                    self.video.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
                self._rendered_camera = display_key
            self.camera_status.setText('수신 중 · ' + self.camera.status)
        else:
            self._rendered_camera = None
            self.video.clear()
            self.video.setText('영상 미수신' if latest is None else '영상 수신 끊김')
            self.camera_status.setText(self.camera.status + ' · 토픽 / Domain / 카메라 실행 확인')

    def save_domain(self):
        self.stop()
        if not self.connection_change_allowed():
            return
        value = self.domain_input.value()
        namespace = '/'+self.vehicle_box.currentText().strip('/ ')
        try:
            from rclpy.validate_namespace import validate_namespace
            validate_namespace(namespace)
        except Exception as exc:
            self.status.setText('namespace 오류: '+str(exc))
            return
        use_sim_time = self.sim_time.isChecked()
        profile = self.profile_box.currentData()
        self.settings.setValue(self.camera_settings_prefix+'topic', self.camera_topic.currentText().strip())
        self.settings.setValue(self.camera_settings_prefix+'compressed', self.compressed.isChecked())
        self.parameters._remember_parameter_configs()
        self.settings.setValue('ros/domain_id', value)
        self.settings.setValue(f'vehicles/domain_{value}/namespace', namespace)
        self.settings.setValue(f'ros/domain_{value}/use_sim_time', use_sim_time)
        self.settings.setValue(navigation_profile_key(value, namespace), profile)
        self.settings.sync()
        if self.settings.status() != QtCore.QSettings.NoError:
            QtWidgets.QMessageBox.warning(self, '저장 실패',
                                          '설정 파일을 저장하지 못했습니다. 기존 ID를 유지합니다.')
            return
        if (value, namespace.rstrip('/'), use_sim_time, profile) == (
                self.domain_id, self.vehicle_namespace, self.use_sim_time, self.navigation_profile):
            self.status.setText(f'Domain {value} 저장 완료 · 연결됨 · 정지')
            return
        self.status.setText(f'Domain {value}에 다시 연결 중…')
        self.connectionRequested.emit(value, namespace, use_sim_time, profile)

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        if self.publisher.publish(msg) is False:
            self.status.setText(self.publisher.blocked_reason)
            return False
        return True

    def emergency_stop(self):
        if self.bridge:
            cancel = getattr(self.bridge, 'cancel', None)
            if callable(cancel):
                cancel()
            disable = getattr(self.bridge, 'disable_autonomous_drive', None)
            if callable(disable):
                disable()
        self.manual_override_active = False
        self.stop()
        self.publish()
        self.request_mux_safety(reset=False)

    def reset_emergency_stop(self):
        self.request_mux_safety(reset=True)

    def request_mux_safety(self, reset=False):
        """Use the same robot-side latched stop as web, never a competing Twist.

        In standalone sessions the existing cancel/zero behavior remains. A
        discovered mux must acknowledge the service; unavailable transport is
        reported as unconfirmed rather than showing a misleading 'stopped'.
        """
        node = self.ros_node or getattr(self.publisher, 'node', None)
        available = getattr(self.publisher, 'manual_available', None)
        if callable(available):
            available()
        mux_present = getattr(self.publisher, 'mux_owns_output', False)
        if mux_present is False:
            if reset:
                self.status.setText('공통 안전 제어기 미연결 · 해제할 비상정지 없음')
            return False
        if node is None or not hasattr(node, 'create_client'):
            self.status.setText('비상정지 미확인 · 차량 안전 제어기 연결 확인 필요')
            return False
        # An emergency stop supersedes an in-flight reset request locally too.
        if self.safety_request is not None:
            if not reset and not self.safety_request[2]:
                return True
            self.safety_request[0].cancel()
            self.safety_request = None
        name = '/smart_vision_amr/'+('reset_emergency_stop' if reset else 'emergency_stop')
        try:
            client = self.safety_clients.get(name)
            if client is None:
                client = node.create_client(Trigger, name)
                self.safety_clients[name] = client
            if not client.service_is_ready():
                self.status.setText('비상정지 미확인 · 차량 안전 서비스에 연결되지 않음')
                return False
            future = client.call_async(Trigger.Request())
            self.safety_request = (future, time.monotonic(), reset)
            self.status.setText('비상정지 해제 확인 중…' if reset else '비상정지 요청 중 · 차량 응답 대기…')
            return True
        except Exception as exc:
            self.status.setText('비상정지 미확인 · '+str(exc))
            return False

    def poll_mux_safety(self):
        pending = self.safety_request
        if pending is None:
            return
        future, started, reset = pending
        if future.done():
            self.safety_request = None
            try:
                result = future.result()
                if result is None or not result.success:
                    self.status.setText('비상정지 해제 거부 · '+str(getattr(result, 'message', '응답 없음'))
                                        if reset else '비상정지 미확인 · '+str(getattr(result, 'message', '응답 없음')))
                else:
                    self.last_safety_ack_at = None if reset else time.monotonic()
                    self.status.setText('비상정지 해제 확인 · 출발 대기' if reset else
                                        '비상정지 확인 · 차량 제어기 정지 잠금 및 0속도 출력')
            except Exception as exc:
                self.status.setText('비상정지 미확인 · '+str(exc))
        elif time.monotonic()-started > 3.0:
            future.cancel()
            self.safety_request = None
            self.status.setText('비상정지 미확인 · 차량 응답 시간 초과')

    def manual_drive_ready(self):
        """Reserve /cmd_vel for manual control before accepting one input."""
        available = getattr(self.publisher, 'manual_available', None)
        if callable(available) and not available():
            self.status.setText(self.publisher.blocked_reason)
            return False
        if not self.bridge:
            return True
        refresh = getattr(self.bridge, 'refresh_autonomous_drivers', None)
        if callable(refresh):
            refresh()
        if getattr(self.bridge, 'custom_drive_busy', False) is True:
            disable = getattr(self.bridge, 'disable_autonomous_drive', None)
            if callable(disable):
                disable()
            self.keys.clear()
            self.active = None
            self.direction = (0, 0)
            self.stop_ticks = 0
            self.publish()
            self.status.setText(
                '자동주행 정지 요청 중: 완료 후 다시 조작하세요.')
            return False
        if getattr(self.bridge, 'busy', False):
            self.status.setText('Nav2 이동 중: 먼저 정지/취소하세요.')
            return False
        return True

    def start(self, button, direction):
        if not self.manual_drive_ready():
            return
        self.keys.clear()
        self.active, self.direction = button, direction
        self.manual_override_active = True
        self.stop_ticks = 0
        self.tick()

    def stop(self, resume_autonomous=False):
        was_manual = bool(self.keys or self.active is not None)
        self.keys.clear()
        self.active = None
        self.direction = (0, 0)
        if was_manual or not self.bridge or not self.bridge.busy:
            self.publish()
        self.stop_ticks = 5  # Short stop burst; do not continuously compete when idle.
        self.status.setText('정지')
        if resume_autonomous and self.manual_override_active:
            self.manual_override_active = False
            QtCore.QTimer.singleShot(300, self.resume_autonomous)

    def resume_autonomous(self):
        if self.bridge:
            enable = getattr(self.bridge, 'enable_autonomous_drive', None)
            if callable(enable):
                enable()

    def tick(self):
        self.poll_mux_safety()
        available = getattr(self.publisher, 'manual_available', None)
        if callable(available) and not available():
            was_manual = bool(self.keys or self.active is not None)
            self.keys.clear()
            self.active = None
            self.direction = (0, 0)
            self.stop_ticks = 0
            self.manual_override_active = False
            if was_manual:
                self.status.setText(self.publisher.blocked_reason)
            return
        if self.bridge and self.bridge.busy:
            was_manual = bool(self.keys or self.active is not None)
            self.keys.clear()
            self.active = None
            self.stop_ticks = 0
            if was_manual:
                self.publish()
            return
        if self.keys or self.active is not None:
            if not self.keys and not self.active.isDown():
                self.stop(resume_autonomous=True)
                return
            linear = self.direction[0] * self.linear_speed.value()
            angular = self.direction[1] * self.angular_speed.value()
            if self.publish(linear, angular):
                self.status.setText(f'명령 전송: {linear:+.2f} m/s · {angular:+.2f} rad/s')
        elif self.stop_ticks:
            self.publish()
            self.stop_ticks -= 1

    def application_state_changed(self, state):
        if state != QtCore.Qt.ApplicationActive:
            self.stop(resume_autonomous=True)

    def hideEvent(self, event):
        self.stop(resume_autonomous=True)
        super().hideEvent(event)

    def closeEvent(self, event):
        # Discover and stop a custom driver before deciding that the window is
        # idle.  It may not have appeared in the periodic graph poll yet.
        acknowledged = (self.last_safety_ack_at is not None
                        and time.monotonic()-self.last_safety_ack_at < 2.0)
        if not acknowledged:
            self.emergency_stop()
        if self.safety_request is not None:
            self.status.setText('차량 비상정지 확인 중 · 응답 후 창을 닫으세요.')
            event.ignore()
            return
        if self.bridge and self.bridge.busy:
            self.status.setText(
                '자동/Nav2 주행 정지 요청 중입니다. 완료 후 창을 닫으세요.')
            event.ignore()
            return
        QtWidgets.QApplication.instance().removeEventFilter(self)
        self.timer.stop()
        event.accept()


def main():
    # Own SIGINT/SIGTERM so a stop can be sent before the ROS context is destroyed.
    from rclpy.signals import SignalHandlerOptions
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, choices=range(233), metavar='0..232',
                        help='저장된 ID보다 우선하여 이번 실행의 ROS Domain 지정')
    parser.add_argument('--robot-namespace', help='연결할 차량 namespace (예: /robot1)')
    parser.add_argument('--camera-topic', help='시작 시 연결할 ROS Image 토픽')
    parser.add_argument('--parameter-config', action='append', default=None, metavar='YAML',
                        help='기본값 저장으로 갱신할 실제 실행 YAML (여러 번 지정 가능)')
    parser.add_argument('--settings-file', help='이번 조종기의 별도 INI 설정 파일')
    parser.add_argument('--use-sim-time', action=argparse.BooleanOptionalAction,
                        default=None, help='ROS /clock 사용 여부 (기본: Domain별 저장값 또는 실제 시간)')
    parser.add_argument('--navigation-profile', choices=('auto', 'nav2', 'team-astar'),
                        help='목표/경로 연결 방식 (기본: 차량별 저장값 또는 자동 감지)')
    parser.add_argument('--navigation-cancel-service',
                        help='주행 계획까지 지우는 선택적 std_srvs/Trigger 서비스')
    options, ros_args = parser.parse_known_args()
    if not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
        print('GUI 화면이 없습니다. PC 데스크탑에서 실행하거나 ssh -X로 접속하세요.',
              file=sys.stderr)
        return 1
    settings = (domain_settings(os.path.abspath(os.path.expanduser(options.settings_file)))
                if options.settings_file else domain_settings())
    domain_id = initial_domain(settings) if options.domain_id is None else options.domain_id
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    namespace = options.robot_namespace or str(settings.value(f'vehicles/domain_{domain_id}/namespace', '/'))
    namespace = '/'+namespace.strip('/')
    use_sim_time = initial_sim_time(settings, domain_id, options.use_sim_time)
    navigation_profile = initial_navigation_profile(
        settings, domain_id, namespace, options.navigation_profile)
    tf_args = ['--ros-args', '-r', '/tf:='+namespace.rstrip('/')+'/tf',
               '-r', '/tf_static:='+namespace.rstrip('/')+'/tf_static',
               '-r', '/controller_shared_tf:=/tf', '-r', '/controller_shared_tf_static:=/tf_static']
    node = Node('turtlebot3_pyqt_controller', namespace=namespace, cli_args=tf_args,
                parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
    publisher = LazyVelocityPublisher(node)
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName('turtlebot3-controller')
    app.setDesktopFileName('turtlebot3-controller')
    app.setQuitOnLastWindowClosed(False)
    camera = CameraFeed()
    window = Controller(
        publisher,
        domain_id,
        settings,
        camera,
        node,
        camera_topic=options.camera_topic,
        parameter_configs=options.parameter_config,
        navigation_cancel_service=options.navigation_cancel_service,
        use_sim_time=use_sim_time,
        navigation_profile=navigation_profile,
    )
    # RosBridge가 /map 등의 구독과 Qt signal 연결을 모두 만든 뒤 executor를
    # 시작한다. 먼저 spin하면 durable /map이 패널 연결 전에 한 번 도착해
    # 화면에서 유실될 수 있다.
    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(node)
    ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    ros_thread.start()
    app.applicationStateChanged.connect(window.application_state_changed)
    shutting_down = False
    restart_connection = None

    def shutdown(*_):
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        window.emergency_stop()
        started = time.monotonic()
        def finish_when_idle():
            nonlocal shutting_down
            window.poll_mux_safety()
            if window.safety_request is not None:
                QtCore.QTimer.singleShot(50, finish_when_idle)
                return
            if window.bridge and window.bridge.busy:
                if time.monotonic()-started > 8:
                    window.show()
                    window.status.setText('Nav2 취소 확인 실패. 연결/목표 상태를 확인하고 다시 종료하세요.')
                    shutting_down = False
                    return
                QtCore.QTimer.singleShot(100, finish_when_idle)
                return
            window.hide()
            QtCore.QTimer.singleShot(300, app.quit)
        finish_when_idle()

    def reconnect(domain, namespace, use_sim_time, profile):
        nonlocal restart_connection
        restart_connection = (domain, namespace, use_sim_time, profile)
        shutdown()
    window.connectionRequested.connect(reconnect)
    window.vehicleRequested.connect(lambda namespace: reconnect(
        domain_id, namespace, use_sim_time, navigation_profile))
    app.lastWindowClosed.connect(shutdown)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    window.show()
    try:
        result = app.exec_()
    finally:
        window.timer.stop()
        window.video_timer.stop()
        if window.bridge:
            window.bridge.timer.stop()
        camera.close()
        if rclpy.ok():
            window.publish()
        publisher.close()
        ros_executor.shutdown(timeout_sec=2.0)
        ros_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    if restart_connection is not None:
        restart_args = reconnect_arguments(options, *restart_connection)
        os.execv(sys.executable, restart_args + ros_args)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
