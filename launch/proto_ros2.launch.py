#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def ros_gz_bridge(topic, ros_type, gz_type, io):
    direction = "]" if io == "i" else "["
    param = f'{topic}@{ros_type}{direction}{gz_type}'
    cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', param]
    return ExecuteProcess(cmd=cmd, output='screen')

def ros_gz_float64(topic, io):
    ros_type = "std_msgs/msg/Float64"
    gz_type = "gz.msgs.Double"
    return ros_gz_bridge(topic, ros_type, gz_type, io)

def ros_gz_int32(topic, io):
    ros_type = "std_msgs/msg/Int32"
    gz_type = "gz.msgs.Int32"
    return ros_gz_bridge(topic, ros_type, gz_type, io)

def ros_gz_pose_stamped(topic):
    ros_type = "geometry_msgs/msg/PoseStamped"
    gz_type = "gz.msgs.Pose"
    return ros_gz_bridge(topic, ros_type, gz_type, "o")

def ros_gz_vector3(topic, io):
    ros_type = "geometry_msgs/msg/Vector3"
    gz_type = "gz.msgs.Vector3d"
    return ros_gz_bridge(topic, ros_type, gz_type, io)

def ros_gz_joint_state(topic):
    ros_type = "sensor_msgs/msg/JointState"
    gz_type = "gz.msgs.Model"
    return ros_gz_bridge(topic, ros_type, gz_type, "o")

def ros_gz_image(topic):
    ros_type = "sensor_msgs/msg/Image"
    gz_type = "gz.msgs.Image"
    return ros_gz_bridge(topic, ros_type, gz_type, "o")

def ros_gz_camera_info(topic):
    ros_type = "sensor_msgs/msg/CameraInfo"
    gz_type = "gz.msgs.CameraInfo"
    direction = "["
    return ros_gz_bridge(topic, ros_type, gz_type, "o")


def ros_gz_gimbal():
    descs = []

    descs.append(ros_gz_image("/gimbal/camera0"))
    descs.append(ros_gz_image("/gimbal/camera1"))
    descs.append(ros_gz_float64("/gimbal/joint0/cmd", "i"))
    descs.append(ros_gz_float64("/gimbal/joint1/cmd", "i"))
    descs.append(ros_gz_float64("/gimbal/joint2/cmd", "i"))
    descs.append(ros_gz_joint_state("/gimbal/joint0/state"))
    descs.append(ros_gz_joint_state("/gimbal/joint1/state"))
    descs.append(ros_gz_joint_state("/gimbal/joint2/state"))
    descs.append(ros_gz_int32("/gimbal/mode/cmd", "i"))
    descs.append(ros_gz_int32("/gimbal/mode/state", "o"))
    descs.append(ros_gz_vector3("/gimbal/target_point/state", "o"))
    descs.append(ros_gz_vector3("/gimbal/target_point/cmd", "i"))
    descs.append(ros_gz_vector3("/gimbal/target_attitude/state", "o"))
    descs.append(ros_gz_vector3("/gimbal/target_attitude/cmd", "i"))
    descs.append(ros_gz_camera_info("/gimbal/camera_info"))
    descs.append(ros_gz_pose_stamped("/model/gimbal/pose"))

    return descs


def ros_gz_mav():
    descs = []
    descs.append(ros_gz_vector3("/x500/position/cmd", "i"))
    descs.append(ros_gz_float64("/x500/yaw/cmd", "i"))
    return descs


def ros_gz_aprilgrid():
    descs = []
    descs.append(ros_gz_pose_stamped("/model/aprilgrid/pose"))
    return descs


def rqt_proc():
    return ExecuteProcess(cmd=['rqt'], output='screen')


def generate_launch_description():
    # Settings
    gz_world = LaunchConfiguration("gz_world")
    enable_rqt = LaunchConfiguration("enable_rqt")
    has_mav = LaunchConfiguration("has_mav")
    has_gimbal = LaunchConfiguration("has_gimbal")
    has_aprilgrid = LaunchConfiguration("has_aprilgrid")

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

    # Processes
    descs = []
    descs.append(gz_proc)

    # -- RQT
    if enable_rqt:
        descs.append(rqt_proc())

    # -- MAV
    if has_mav:
        descs.extend(ros_gz_mav())

    # -- GIMBAL
    if has_gimbal:
        descs.extend(ros_gz_gimbal())

    # -- APRILGRID
    if has_aprilgrid:
        descs.extend(ros_gz_aprilgrid())

    return LaunchDescription(descs)
