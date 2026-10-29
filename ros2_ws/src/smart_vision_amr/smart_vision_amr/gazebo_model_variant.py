"""Create a Gazebo model with stock odometry TF disabled for EKF authority."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET


def set_camera_update_rate(source: Path, destination: Path, rate: float) -> None:
    """Limit simulated camera production without changing geometry or IMU rate."""
    if not math.isfinite(rate) or not 0.0 < rate <= 60.0:
        raise ValueError('camera_update_rate must be positive and at most 60 Hz')
    tree = ET.parse(source)
    cameras = tree.findall(".//sensor[@type='camera']")
    if not cameras:
        raise ValueError('TurtleBot3 camera sensor was not found')
    for sensor in cameras:
        update = sensor.find('update_rate')
        if update is None:
            update = ET.SubElement(sensor, 'update_rate')
        update.text = str(float(rate))
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding='unicode', xml_declaration=False)


def make_independent_ekf_model(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    plugin = tree.find(".//plugin[@name='turtlebot3_diff_drive']")
    if plugin is None:
        raise ValueError('TurtleBot3 diff-drive plugin was not found')
    odom_tf = plugin.find('publish_odom_tf')
    odom_topic = plugin.find('odometry_topic')
    if odom_tf is None or odom_topic is None:
        raise ValueError('TurtleBot3 diff-drive odometry settings were not found')
    odom_tf.text = 'false'
    odom_topic.text = 'gazebo/odom_raw'
    # ROS 2 gazebo_ros_diff_drive still advertises `odom` unless explicitly
    # remapped in its <ros> block; the SDF odometry_topic tag alone is ignored.
    ros = plugin.find('ros')
    if ros is None:
        ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'remapping').text = 'odom:=/gazebo/odom_raw'
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Humble's spawn_entity.py passes text to lxml.fromstring; an XML encoding
    # declaration on that Unicode string raises ValueError before spawning.
    tree.write(destination, encoding='unicode', xml_declaration=False)
