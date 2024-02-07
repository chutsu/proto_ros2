#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  vicon_node = Node(package="ros2_vicon",
                    executable="vicon_node",
                    name="vicon_node",
                    parameters=[
                        {
                            "hostname": "10.0.5.127"
                        },
                    output="screen",
                    ])
  return launch.LaunchDescription([vicon_node])
