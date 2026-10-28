"""Camera decoding contracts; these tests never subscribe to a live robot."""

from types import SimpleNamespace
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cv2
import numpy as np

from smart_vision_amr.web_mission_server import (
    DEFAULT_MARKERS, MissionApplication, create_ros_bridge, image_to_bgr,
)

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CompressedImage
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class CameraTransportTests(unittest.TestCase):
    def test_compressed_jpeg_decodes_as_bgr(self):
        original = np.full((32, 48, 3), (15, 75, 210), dtype=np.uint8)
        ok, encoded = cv2.imencode('.jpg', original, [cv2.IMWRITE_JPEG_QUALITY, 95])
        self.assertTrue(ok)
        frame = image_to_bgr(SimpleNamespace(format='bgr8; jpeg compressed bgr8',
                                             data=encoded.tobytes()))
        self.assertEqual(frame.shape, original.shape)
        self.assertTrue(frame.flags.c_contiguous)
        self.assertLess(np.abs(frame.astype(np.int16)-original).max(), 8)

    def test_invalid_compressed_frame_fails_without_image(self):
        with self.assertRaisesRegex(ValueError, '비어'):
            image_to_bgr(SimpleNamespace(format='jpeg', data=b''))
        with self.assertRaisesRegex(ValueError, '디코딩'):
            image_to_bgr(SimpleNamespace(format='jpeg', data=b'not a jpeg'))

    def test_raw_image_still_honors_step_padding_and_rgb_order(self):
        frame = image_to_bgr(SimpleNamespace(encoding='rgb8', width=2, height=1,
                                             step=8, data=bytes((255, 0, 0, 0, 255, 0, 99, 99))))
        self.assertEqual(frame.tolist(), [[[0, 0, 255], [0, 255, 0]]])


@unittest.skipUnless(HAS_ROS, 'source ROS Humble for the isolated bridge test')
class CameraBridgeTests(unittest.TestCase):
    def test_compressed_topics_subscribe_and_update_both_previews(self):
        with TemporaryDirectory() as tmp:
            context = Context()
            rclpy.init(args=[], context=context, domain_id=222,
                       signal_handler_options=SignalHandlerOptions.NO)
            app = MissionApplication(DEFAULT_MARKERS, 222, False,
                                     Path(tmp)/'settings.json')
            node = create_ros_bridge(app, SimpleNamespace(
                camera_topic='/test/rear/compressed',
                front_camera_topic='/test/front/compressed', base_frame='base_footprint'),
                context)
            try:
                for topic in ('/test/rear/compressed', '/test/front/compressed'):
                    endpoints = node.get_subscriptions_info_by_topic(topic)
                    self.assertEqual([item.topic_type for item in endpoints],
                                     ['sensor_msgs/msg/CompressedImage'])
                ok, encoded = cv2.imencode('.jpg', np.zeros((16, 16, 3), np.uint8))
                self.assertTrue(ok)
                message = CompressedImage()
                message.header.stamp = node.get_clock().now().to_msg()
                message.format = 'jpeg'
                message.data = encoded.tobytes()
                node.on_image(message)
                node.on_front_image(message)
                self.assertGreater(app.camera_at, 0.)
                self.assertGreater(app.front_camera_at, 0.)
                self.assertTrue(app.camera_jpeg.startswith(b'\xff\xd8'))
                self.assertTrue(app.front_camera_jpeg.startswith(b'\xff\xd8'))
            finally:
                node.destroy_node()
                context.shutdown()


if __name__ == '__main__':
    unittest.main()
