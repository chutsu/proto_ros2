#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def launch_argument(context, name, data_type):
    str_data = LaunchConfiguration(name).perform(context)

    if data_type == str:
        return str_data
    elif data_type == bool:
        return str_data == "true"
    elif data_type == int:
        return int(str_data)
    elif data_type == float:
        return float(str_data)
    else:
        raise RunTimeError(f"Invalid data type [{data_type}]")


def ros_gz_bridge(gz_topic, ros_type, gz_type, io):
    direction = "]" if io == "i" else "["
    param = f'{gz_topic}@{ros_type}{direction}{gz_type}'
    cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', param]
    return ExecuteProcess(cmd=cmd, output='screen', shell=True)

def ros_gz_float64(gz_topic, io):
    ros_type = "std_msgs/msg/Float64"
    gz_type = "gz.msgs.Double"
    return ros_gz_bridge(gz_topic, ros_type, gz_type, io)

def ros_gz_int32(gz_topic, io):
    ros_type = "std_msgs/msg/Int32"
    gz_type = "gz.msgs.Int32"
    return ros_gz_bridge(gz_topic, ros_type, gz_type, io)

def ros_gz_pose_stamped(gz_topic):
    ros_type = "geometry_msgs/msg/PoseStamped"
    gz_type = "gz.msgs.Pose"
    return ros_gz_bridge(gz_topic, ros_type, gz_type, "o")

def ros_gz_vector3(gz_topic, io):
    ros_type = "geometry_msgs/msg/Vector3"
    gz_type = "gz.msgs.Vector3d"
    return ros_gz_bridge(gz_topic, ros_type, gz_type, io)

def ros_gz_joint_state(gz_topic):
    ros_type = "sensor_msgs/msg/JointState"
    gz_type = "gz.msgs.Model"
    return ros_gz_bridge(gz_topic, ros_type, gz_type, "o")

def ros_gz_image(gz_topic):
    ros_type = "sensor_msgs/msg/Image"
    gz_type = "gz.msgs.Image"
    return ros_gz_bridge(gz_topic, ros_type, gz_type, "o")

def ros_gz_camera_info(gz_topic):
    ros_type = "sensor_msgs/msg/CameraInfo"
    gz_type = "gz.msgs.CameraInfo"
    direction = "["
    return ros_gz_bridge(gz_topic, ros_type, gz_type, "o")


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
    descs.append(ros_gz_pose_stamped("/model/x500/pose"))
    descs.append(ros_gz_vector3("/x500/position/cmd", "i"))
    descs.append(ros_gz_float64("/x500/yaw/cmd", "i"))
    return descs


def ros_gz_aprilgrid():
    descs = []
    descs.append(ros_gz_pose_stamped("/model/aprilgrid/pose"))
    return descs


def rqt_proc():
    return ExecuteProcess(cmd=['rqt'], output='screen')


def launch_setup(context, *args, **kwargs):
    # Settings
    gz_world = launch_argument(context, "gz_world", str)
    verbose = launch_argument(context, "verbose", bool)
    run_on_start = launch_argument(context, "run_on_start", bool)
    enable_headless = launch_argument(context, "enable_headless", bool)
    enable_rqt = launch_argument(context, "enable_rqt", bool)
    has_mav = launch_argument(context, "has_mav", bool)
    has_gimbal = launch_argument(context, "has_gimbal", bool)
    has_aprilgrid = launch_argument(context, "has_aprilgrid", bool)

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
    cmd = ['gz', 'sim', gz_world, '--gui-config', config_path]
    if verbose:
        cmd.append("-v")

    if run_on_start:
        cmd.append("-r")

    if enable_headless:
        cmd.append("-s")
        cmd.append("--headless-rendering")
    gz_proc = ExecuteProcess(cmd=cmd, output='screen')

    # Processes
    descs = []

    # -- Gazebo Simulator
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

    return descs


def generate_launch_description():
    # ROS2 has made parsing launch arguments incredibly complicated.
    # One cannot get the launch arguments with out this work-around:
    # https://answers.ros.org/question/322636/ros2-access-current-launchconfiguration/?answer=359167#post-id-359167
    return launch.LaunchDescription([
        DeclareLaunchArgument("gz_world", default_value="sim_sandbox.sdf"),
        DeclareLaunchArgument("verbose", default_value='False'),
        DeclareLaunchArgument("run_on_start", default_value='True'),
        DeclareLaunchArgument("enable_headless", default_value='False'),
        DeclareLaunchArgument("enable_rqt", default_value='False'),
        DeclareLaunchArgument("has_mav", default_value='False'),
        DeclareLaunchArgument("has_gimbal", default_value='False'),
        DeclareLaunchArgument("has_aprilgrid", default_value='False'),
        OpaqueFunction(function = launch_setup)
    ])
