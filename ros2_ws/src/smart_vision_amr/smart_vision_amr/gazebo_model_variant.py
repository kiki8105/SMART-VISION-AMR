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


def add_classroom_dual_cameras(source: Path, destination: Path) -> None:
    """Keep the stock front camera and add the real robot's rear view.

    This is only a Gazebo variant. It leaves the installed TurtleBot model and
    hardware camera launch untouched. Both views use the real UI topic names.
    """
    tree = ET.parse(source)
    model = tree.find('model')
    front = tree.find(".//sensor[@name='camera'][@type='camera']")
    if model is None or front is None:
        raise ValueError('The stock Waffle Pi camera or model is missing')
    if tree.find(".//sensor[@name='rear_camera']") is not None:
        raise ValueError('Rear camera already exists in the source model')
    front_plugin = front.find("plugin[@filename='libgazebo_ros_camera.so']")
    if front_plugin is None:
        raise ValueError('The stock front camera ROS plugin is missing')
    front_ros = front_plugin.find('ros')
    if front_ros is None:
        front_ros = ET.SubElement(front_plugin, 'ros')
    # gazebo_ros_camera prefixes topic names with its sensor name before ROS
    # remapping. Bare image_raw:=... does not match /camera/image_raw.
    ET.SubElement(front_ros, 'remapping').text = 'camera/image_raw:=/camera/front/image_raw'
    ET.SubElement(front_ros, 'remapping').text = 'camera/image_raw/compressed:=/camera/front/image_raw/compressed'
    ET.SubElement(front_ros, 'remapping').text = 'camera/camera_info:=/camera/front/camera_info'

    rear_link = ET.SubElement(model, 'link', name='rear_camera_link')
    ET.SubElement(rear_link, 'pose').text = '-0.080 0 0.150 0 0 3.141592653589793'
    sensor = ET.SubElement(rear_link, 'sensor', name='rear_camera', type='camera')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'visualize').text = 'false'
    ET.SubElement(sensor, 'update_rate').text = '8.0'
    camera = ET.SubElement(sensor, 'camera', name='rear_webcam')
    ET.SubElement(camera, 'horizontal_fov').text = '1.085595'
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = '640'
    ET.SubElement(image, 'height').text = '480'
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.03'
    ET.SubElement(clip, 'far').text = '10.0'
    noise = ET.SubElement(camera, 'noise')
    ET.SubElement(noise, 'type').text = 'gaussian'
    ET.SubElement(noise, 'mean').text = '0.0'
    ET.SubElement(noise, 'stddev').text = '0.007'
    plugin = ET.SubElement(sensor, 'plugin', name='rear_camera_driver', filename='libgazebo_ros_camera.so')
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'remapping').text = 'rear_camera/image_raw:=/camera/image_raw'
    ET.SubElement(ros, 'remapping').text = 'rear_camera/image_raw/compressed:=/camera/image_raw/compressed'
    ET.SubElement(ros, 'remapping').text = 'rear_camera/camera_info:=/camera/camera_info'
    ET.SubElement(plugin, 'frame_name').text = 'rear_camera_link'
    joint = ET.SubElement(model, 'joint', name='rear_camera_joint', type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'rear_camera_link'
    ET.SubElement(joint, 'pose').text = '-0.080 0 0.140 0 0 3.141592653589793'
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding='unicode', xml_declaration=False)
