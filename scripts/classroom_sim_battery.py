#!/usr/bin/env python3
"""Publish a stamped battery sample for the isolated classroom Gazebo trial."""

import argparse
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class SimBattery(Node):
    def __init__(self, voltage: float, rate: float):
        super().__init__('classroom_sim_battery')
        self.voltage = voltage
        self.publisher = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_battery)

    def publish_battery(self):
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.voltage = self.voltage
        message.percentage = 0.75
        message.present = True
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--voltage', type=float, default=12.0)
    parser.add_argument('--rate', type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') != '176' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('simulated battery is restricted to ROS domain 176, localhost only')
    if not math.isfinite(args.voltage) or args.voltage <= 0:
        parser.error('--voltage must be finite and positive')
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be finite and positive')
    rclpy.init(args=ros_args)
    node = SimBattery(args.voltage, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
