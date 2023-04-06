#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def ros_gz_bridge(topic, ros_type, gz_type, direction):
    param = f'{topic}@{ros_type}{direction}{gz_type}'
    cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', param]
    return ExecuteProcess(cmd=cmd, output='screen')

def ros_gz_image_bridge(topic):
    ros_type = "sensor_msgs/msg/Image"
    gz_type = "gz.msgs.Image"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_gimbal_joint_cmd_bridge(topic):
    ros_type = "std_msgs/msg/Float64"
    gz_type = "gz.msgs.Double"
    direction = "]"
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_gimbal_joint_state_bridge(topic):
    ros_type = "sensor_msgs/msg/JointState"
    gz_type = "gz.msgs.Model"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_camera_info_bridge(topic):
    ros_type = "sensor_msgs/msg/CameraInfo"
    gz_type = "gz.msgs.CameraInfo"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_pose_bridge(topic):
    ros_type = "geometry_msgs/msg/PoseStamped"
    gz_type = "gz.msgs.Pose"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def generate_launch_description():
    # Settings
    gz_world = "calibration.sdf"

    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('proto_ros2')
    models_path = pkg_share_dir + "/gazebo/models"
    worlds_path = pkg_share_dir + "/gazebo/worlds"
    plugins_path = pkg_share_dir + "/gazebo/plugins"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugins_path

    # Gazebo Simulator
    gz_proc = ExecuteProcess(cmd=['gz', 'sim', gz_world, '-v'], output='screen')

    # Gazebo -> ROS2 bridges
    cam0_bridge = ros_gz_image_bridge("/gimbal/camera0")
    cam1_bridge = ros_gz_image_bridge("/gimbal/camera1")
    yaw_cmd_bridge = ros_gz_gimbal_joint_cmd_bridge("/gimbal/joint0_cmd")
    roll_cmd_bridge = ros_gz_gimbal_joint_cmd_bridge("/gimbal/joint1_cmd")
    pitch_cmd_bridge = ros_gz_gimbal_joint_cmd_bridge("/gimbal/joint2_cmd")
    yaw_state_bridge = ros_gz_gimbal_joint_state_bridge("/gimbal/joint0_state")
    roll_state_bridge = ros_gz_gimbal_joint_state_bridge("/gimbal/joint1_state")
    pitch_state_bridge = ros_gz_gimbal_joint_state_bridge("/gimbal/joint2_state")
    camera_info_bridge = ros_gz_camera_info_bridge("/gimbal/camera_info")
    aprilgrid_pose_bridge = ros_gz_pose_bridge("/model/aprilgrid/pose")
    gimbal_pose_bridge = ros_gz_pose_bridge("/model/gimbal/pose")

    # Launch
    descs = []
    descs.append(gz_proc)
    descs.append(cam0_bridge)
    descs.append(cam1_bridge)
    descs.append(yaw_cmd_bridge)
    descs.append(roll_cmd_bridge)
    descs.append(pitch_cmd_bridge)
    descs.append(yaw_state_bridge)
    descs.append(roll_state_bridge)
    descs.append(pitch_state_bridge)
    descs.append(camera_info_bridge)
    descs.append(aprilgrid_pose_bridge)
    descs.append(gimbal_pose_bridge)

    return LaunchDescription(descs)
