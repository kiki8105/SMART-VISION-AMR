#!/usr/bin/env python3
"""Publish a V4L2 camera as a rate-limited ROS 2 JPEG CompressedImage.

Example (run once per camera, with different topics and node names)::

    python3 ros_camera_compressed.py \
      --device /dev/v4l/by-id/usb-example-video-index0 \
      --topic /camera/image_raw/compressed \
      --node-name rear_camera_compressed \
      --frame-id rear_camera_optical_frame

The capture thread continuously drains the V4L2 buffer. The ROS timer publishes
only its newest frame at the requested rate, so a slow Wi-Fi link does not cause
a growing queue of old images. No robot motion commands are published.
"""

import argparse
import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def positive_float(value):
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return number


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return number


def jpeg_quality(value):
    number = int(value)
    if not 1 <= number <= 100:
        raise argparse.ArgumentTypeError('must be between 1 and 100')
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, help='V4L2 path, preferably /dev/v4l/by-id/...')
    parser.add_argument('--topic', required=True, help='CompressedImage topic, e.g. /camera/image_raw/compressed')
    parser.add_argument('--node-name', default='camera_compressed_publisher', help='unique ROS node name')
    parser.add_argument('--frame-id', default='camera_optical_frame', help='image header frame_id')
    parser.add_argument('--width', type=positive_int, default=320, help='capture width (default: 320)')
    parser.add_argument('--height', type=positive_int, default=240, help='capture height (default: 240)')
    parser.add_argument('--fps', type=positive_float, default=5.0, help='published frames per second (default: 5)')
    parser.add_argument('--capture-fps', type=positive_float, default=10.0,
                        help='requested V4L2 capture rate; the thread keeps only the newest frame (default: 10)')
    parser.add_argument('--jpeg-quality', type=jpeg_quality, default=60, help='JPEG quality 1-100 (default: 60)')
    parser.add_argument('--pixel-format', choices=('MJPG', 'YUYV', 'AUTO'), default='MJPG',
                        help='requested V4L2 format (default: MJPG)')
    parser.add_argument('--retry-seconds', type=positive_float, default=2.0,
                        help='delay before reopening a failed camera (default: 2)')
    parser.add_argument('--frame-timeout', type=positive_float, default=2.0,
                        help='do not publish frames older than this many seconds (default: 2)')
    args = parser.parse_args(argv)
    if not args.topic.startswith('/') or not args.topic.endswith('/compressed'):
        parser.error('--topic must be an absolute topic ending in /compressed')
    return args


def encode_jpeg(frame, quality):
    """Encode one OpenCV BGR image; return bytes or raise on encoder failure."""
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('OpenCV JPEG encoding failed')
    return encoded.tobytes()


class LatestFrameCapture:
    """Read camera frames continuously and retain only the newest one."""

    def __init__(self, args, clock):
        self.args = args
        self.clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='v4l2-capture', daemon=True)
        self._sequence = 0
        self._latest = None  # (sequence, BGR frame, ROS stamp, monotonic receipt time)
        self._problem = 'camera opening'

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
        return not self._thread.is_alive()

    def snapshot(self):
        with self._lock:
            return self._latest, self._problem

    def _set_problem(self, problem):
        with self._lock:
            self._problem = problem
            self._latest = None

    def _open(self):
        capture = cv2.VideoCapture(self.args.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            return None
        if self.args.pixel_format != 'AUTO':
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.args.pixel_format))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
        capture.set(cv2.CAP_PROP_FPS, self.args.capture_fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _run(self):
        while not self._stop.is_set():
            capture = None
            try:
                capture = self._open()
                if capture is None:
                    self._set_problem(f'cannot open {self.args.device}')
                else:
                    failures = 0
                    while not self._stop.is_set():
                        ok, frame = capture.read()
                        if not ok or frame is None or frame.size == 0:
                            failures += 1
                            if failures >= 3:
                                self._set_problem(f'camera read failed ({self.args.device})')
                                break
                            self._stop.wait(0.05)
                            continue
                        failures = 0
                        if frame.shape[1] != self.args.width or frame.shape[0] != self.args.height:
                            frame = cv2.resize(frame, (self.args.width, self.args.height),
                                               interpolation=cv2.INTER_AREA)
                        stamp = self.clock.now().to_msg()
                        with self._lock:
                            self._sequence += 1
                            self._latest = (self._sequence, frame, stamp, time.monotonic())
                            self._problem = None
            except Exception as exc:  # A disconnected V4L2 device can raise from OpenCV.
                self._set_problem(f'camera error: {exc}')
            finally:
                if capture is not None:
                    capture.release()
            self._stop.wait(self.args.retry_seconds)


class CompressedCameraNode(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.args = args
        self.publisher = self.create_publisher(CompressedImage, args.topic, qos_profile_sensor_data)
        self.capture = LatestFrameCapture(args, self.get_clock())
        self.last_sequence = 0
        self.last_warning = float('-inf')
        self.capture.start()
        self.timer = self.create_timer(1.0 / args.fps, self.publish_latest)
        self.get_logger().info(
            f'camera={args.device} topic={args.topic} size={args.width}x{args.height} '
            f'fps={args.fps:g} JPEG quality={args.jpeg_quality}'
        )

    def publish_latest(self):
        latest, problem = self.capture.snapshot()
        now = time.monotonic()
        if latest is None or now - latest[3] > self.args.frame_timeout:
            if now - self.last_warning >= 10.0:
                reason = problem or f'last frame older than {self.args.frame_timeout:g}s'
                self.get_logger().warning(f'camera unavailable: {reason}')
                self.last_warning = now
            return
        sequence, frame, stamp, _ = latest
        if sequence == self.last_sequence:
            return
        self.last_sequence = sequence
        try:
            data = encode_jpeg(frame, self.args.jpeg_quality)
        except (cv2.error, RuntimeError) as exc:
            if now - self.last_warning >= 10.0:
                self.get_logger().warning(f'JPEG encode failed: {exc}')
                self.last_warning = now
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.frame_id
        msg.format = 'bgr8; jpeg compressed bgr8'
        msg.data = data
        self.publisher.publish(msg)

    def stop(self):
        if not self.capture.stop():
            self.get_logger().warning('camera read is blocked; the process will release the device on exit')


def main(argv=None):
    args = parse_args(argv)
    # argparse owns the camera CLI flags; do not let rclpy parse them as ROS flags.
    rclpy.init(args=[])
    node = None
    try:
        node = CompressedCameraNode(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
