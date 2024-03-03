#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy
from px4_msgs.msg import VehicleCommand


if __name__ == "__main__":
  # Create node
  rclpy.init()
  node = rclpy.create_node('reboot_px4')

  # Create publisher
  topic_cmd = "/fmu/in/vehicle_command"
  qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST,
                   depth=1)
  pub_cmd = node.create_publisher(VehicleCommand, topic_cmd, qos)
  sleep(2)

  # Publish reboot command
  msg = VehicleCommand()
  msg.command = VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN
  msg.param1 = 1.0
  msg.param2 = 0.0
  msg.param3 = 0.0
  msg.param4 = 0.0
  msg.param5 = 0.0
  msg.param6 = 0.0
  msg.param7 = 0.0
  msg.target_system = 1
  msg.target_component = 1
  msg.source_system = 1
  msg.source_component = 1
  msg.from_external = True
  msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
  pub_cmd.publish(msg)
