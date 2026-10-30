"""Session-only domain guard; force every real ROS context to localhost223."""
import os
import pytest


@pytest.fixture(autouse=True)
def isolated_ros_runtime(monkeypatch):
    import rclpy
    original_init = rclpy.init

    def isolated_init(*args, **kwargs):
        os.environ['ROS_DOMAIN_ID'] = '223'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        kwargs['domain_id'] = 223
        return original_init(*args, **kwargs)

    monkeypatch.setattr(rclpy, 'init', isolated_init)
