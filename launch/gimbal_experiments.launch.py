#!/usr/bin/env python3
import os

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  script_dir = os.path.dirname(__file__)
  config_dir = os.path.abspath(os.path.join(script_dir, "../config"))

  config_path = f"{config_dir}/realsense_d435i.yaml"
  okvis_params = [{"config_path": config_path}]
  okvis_node = Node(package="okvis",
                    executable="okvis_gimbal_node",
                    name="okvis_gimbal_node",
                    parameters=okvis_params,
                    # prefix=["gdb -ex=r -ex bt --args"],
                    output="screen")

  sbgc_node = Node(package="proto_ros2",
                   executable="sbgc_node",
                   name="sbgc_node")

  return launch.LaunchDescription([okvis_node, sbgc_node])
