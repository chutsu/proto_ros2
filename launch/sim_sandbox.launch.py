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

def ros_gz_image(topic):
    ros_type = "sensor_msgs/msg/Image"
    gz_type = "gz.msgs.Image"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_float64(topic):
    ros_type = "std_msgs/msg/Float64"
    gz_type = "gz.msgs.Double"
    direction = "]"
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_joint_state(topic):
    ros_type = "sensor_msgs/msg/JointState"
    gz_type = "gz.msgs.Model"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_camera_info(topic):
    ros_type = "sensor_msgs/msg/CameraInfo"
    gz_type = "gz.msgs.CameraInfo"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_pose_stamped(topic):
    ros_type = "geometry_msgs/msg/PoseStamped"
    gz_type = "gz.msgs.Pose"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_vector3(topic):
    ros_type = "geometry_msgs/msg/Vector3"
    gz_type = "gz.msgs.Vector3d"
    direction = "]"
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def generate_launch_description():
    # Settings
    gz_world = "sim_sandbox.sdf"

    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('proto_ros2')
    models_path = pkg_share_dir + "/gazebo/models"
    worlds_path = pkg_share_dir + "/gazebo/worlds"
    plugins_path = pkg_share_dir + "/gazebo/plugins"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugins_path

    # Config path
    proj_dir = "/".join(os.path.dirname(__file__).split("/")[:-1])
    config_path = os.path.join(proj_dir, "gazebo/configs/sim_sandbox.config")

    # Gazebo Simulator
    cmd = ['gz', 'sim', gz_world, '-v', '-r', '--gui-config', config_path]
    gz_proc = ExecuteProcess(cmd=cmd, output='screen')

    # Gazebo -> ROS2 bridges
    # -- Gimbal
    cam0_bridge = ros_gz_image("/gimbal/camera0")
    cam1_bridge = ros_gz_image("/gimbal/camera1")
    joint0_cmd_bridge = ros_gz_float64("/gimbal/joint0/cmd")
    joint1_cmd_bridge = ros_gz_float64("/gimbal/joint1/cmd")
    joint2_cmd_bridge = ros_gz_float64("/gimbal/joint2/cmd")
    joint0_state_bridge = ros_gz_joint_state("/gimbal/joint0/state")
    joint1_state_bridge = ros_gz_joint_state("/gimbal/joint1/state")
    joint2_state_bridge = ros_gz_joint_state("/gimbal/joint2/state")
    target_point_bridge = ros_gz_vector3("/gimbal/target_point/cmd")
    target_attitude_bridge = ros_gz_vector3("/gimbal/target_attitude/cmd")
    camera_info_bridge = ros_gz_camera_info("/gimbal/camera_info")
    gimbal_pose_bridge = ros_gz_pose_stamped("/model/gimbal/pose")
    # -- MAV
    mav_pos_cmd_bridge = ros_gz_vector3("/x500/position/cmd")
    mav_yaw_cmd_bridge = ros_gz_float64("/x500/yaw/cmd")
    # -- AprilGrid
    aprilgrid_pose_bridge = ros_gz_pose_stamped("/model/aprilgrid/pose")

    # Launch
    descs = []
    # -- Gimbal
    descs.append(gz_proc)
    descs.append(cam0_bridge)
    descs.append(cam1_bridge)
    descs.append(joint0_cmd_bridge)
    descs.append(joint1_cmd_bridge)
    descs.append(joint2_cmd_bridge)
    descs.append(joint0_state_bridge)
    descs.append(joint1_state_bridge)
    descs.append(joint2_state_bridge)
    descs.append(target_point_bridge)
    descs.append(target_attitude_bridge)
    descs.append(camera_info_bridge)
    descs.append(gimbal_pose_bridge)
    # -- MAV
    descs.append(mav_pos_cmd_bridge)
    descs.append(mav_yaw_cmd_bridge)
    # -- AprilGrid
    # descs.append(aprilgrid_pose_bridge)
    # -- RQT
    descs.append(ExecuteProcess(cmd=['rqt'], output='screen'))

    return LaunchDescription(descs)
