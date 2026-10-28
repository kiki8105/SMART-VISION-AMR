#!/usr/bin/env python3
"""Hardware-free checks: python3 -m unittest discover -s scripts -p test_ros_camera_compressed.py."""

import unittest
from unittest.mock import patch
import time

import cv2
import numpy as np

from ros_camera_compressed import LatestFrameCapture, encode_jpeg, parse_args


class CompressedCameraTests(unittest.TestCase):
    def test_jpeg_encoding(self):
        frame = np.full((240, 320, 3), (10, 40, 80), dtype=np.uint8)
        result = encode_jpeg(frame, 60)
        decoded = cv2.imdecode(np.frombuffer(result, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, frame.shape)
        self.assertLess(np.abs(decoded.astype(int) - frame.astype(int)).max(), 10)

    def test_args_validate_topic_and_quality(self):
        base = ['--device', '/dev/video0', '--topic', '/camera/image_raw/compressed']
        args = parse_args(base)
        self.assertEqual((args.width, args.height, args.fps, args.jpeg_quality), (320, 240, 5.0, 60))
        with self.assertRaises(SystemExit):
            parse_args(base + ['--jpeg-quality', '101'])
        with self.assertRaises(SystemExit):
            parse_args(['--device', '/dev/video0', '--topic', '/camera/image_raw'])

    def test_open_failure_waits_before_retry(self):
        args = parse_args(['--device', '/dev/missing', '--topic', '/camera/image_raw/compressed',
                           '--retry-seconds', '0.1'])

        class UnavailableCamera:
            def isOpened(self):
                return False

            def release(self):
                pass

        with patch('ros_camera_compressed.cv2.VideoCapture', return_value=UnavailableCamera()) as opener:
            capture = LatestFrameCapture(args, clock=None)
            capture.start()
            time.sleep(0.24)
            self.assertTrue(capture.stop())
            self.assertGreaterEqual(opener.call_count, 1)
            self.assertLessEqual(opener.call_count, 4)
            self.assertIsNone(capture.snapshot()[0])


if __name__ == '__main__':
    unittest.main()
