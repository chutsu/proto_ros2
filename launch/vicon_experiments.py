#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rs_node = Node(
        package="proto_ros2",
        executable="rs_node",
        name="rs_node"
    )
    vicon_node = Node(
        package="ros2_vicon",
        executable="vicon_node",
        name="vicon_node",
        parameters=[
          {"hostname": "10.0.5.127"},
        ]
    )
    return launch.LaunchDescription([rs_node, vicon_node])
