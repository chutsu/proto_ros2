#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg = "proto_ros2"
  rs_node = Node(package=pkg, executable="rs_node", name="rs_node")
  sbgc_node = Node(package=pkg, executable="sbgc_node", name="sbgc_node")
  return launch.LaunchDescription([rs_node, sbgc_node])
