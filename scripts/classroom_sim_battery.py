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


def require_simulation_environment(domain, environment):
    if (domain not in (176, 178)
            or environment.get('ROS_DOMAIN_ID') != str(domain)
            or environment.get('ROS_LOCALHOST_ONLY') != '1'):
        raise ValueError('simulated battery requires matching simulation domain 176 or 178, localhost only')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--voltage', type=float, default=12.0)
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--domain', type=int, default=176)
    args, ros_args = parser.parse_known_args()
    try:
        require_simulation_environment(args.domain, os.environ)
    except ValueError as exc:
        parser.error(str(exc))
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
