import math

import rclpy
from rclpy.node import Node
import message_filters

from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import cv2
import numpy as np
import proto
import matplotlib.pylab as plt
from mpl_toolkits import mplot3d


def convert_msg(msg):
    if isinstance(msg, PoseStamped):
        ts = int(msg.header.stamp.sec * 1e9) + msg.header.stamp.nanosec
        r = msg.pose.position
        q = msg.pose.orientation
        return (ts, proto.pose2tf([r.x, r.y, r.z, q.x, q.y, q.z, q.w]))

    elif isinstance(msg, Image):
        ts = int(msg.header.stamp.sec * 1e9) + msg.header.stamp.nanosec
        cv_bridge = CvBridge()
        return (ts, cv_bridge.imgmsg_to_cv2(msg))

    elif isinstance(msg, JointState):
        ts = int(msg.header.stamp.sec * 1e9) + msg.header.stamp.nanosec
        return (ts, msg.position[0])

    else:
        raise NotImplementedError()


def plot_gimbal_chains():
    q_WF = proto.euler321(-math.pi / 2.0, 0.0, math.pi / 2.0)
    r_WF = np.array([2, 0.49, 0])
    T_WF = proto.tf(q_WF, r_WF)

    gimbal_pose = np.array([0, 0, 1, 0, 0, 0, 1])
    gimbal_ext = np.array([-0.05, 0, -0.05, 1, 0, 0, 4.63268e-05])
    gimbal_link0 = np.array([0, -1.35525e-20, 0.05, 5.48441e-21, 0.707108, 2.39578e-21, 0.707105])
    gimbal_link1 = np.array([2.22045e-16, -1.35525e-20, 0.05, 0.707108, -4.03592e-17, -7.3647e-17, 0.707105])
    cam0_ext = np.array([-1.11022e-16, 1.00615e-17, 0.05, -0.5, 0.5, 0.500002, 0.499998])
    cam1_ext = np.array([0, 1.07552e-17, -0.05, -0.5, 0.5, 0.500002, 0.499998])

    joint0 = 0.0
    joint1 = 0.0
    joint2 = 0.0

    T_WB = proto.pose2tf(gimbal_pose)
    T_BM0 = proto.pose2tf(gimbal_ext)
    T_M0L0 = proto.tf(proto.euler321(joint0, 0, 0), [0, 0, 0]) # Joint0
    T_M1L1 = proto.tf(proto.euler321(joint1, 0, 0), [0, 0, 0]) # Joint1
    T_M2L2 = proto.tf(proto.euler321(joint2, 0, 0), [0, 0, 0]) # Joint2
    T_L0M1 = proto.pose2tf(gimbal_link0) # Link0
    T_L1M2 = proto.pose2tf(gimbal_link1) # Link1
    T_L2C0 = proto.pose2tf(cam0_ext)
    T_L2C1 = proto.pose2tf(cam1_ext)

    T_WL0 = T_WB @ T_BM0 @ T_M0L0
    T_WL1 = T_WB @ T_BM0 @ T_M0L0 @ T_L0M1 @ T_M1L1
    T_WL2 = T_WB @ T_BM0 @ T_M0L0 @ T_L0M1 @ T_M1L1 @ T_L1M2 @ T_M2L2
    T_WC0 = T_WL2 @ T_L2C0
    T_WC1 = T_WL2 @ T_L2C1

    # Visualize
    tf_size = 0.01
    plt.figure()
    ax = plt.axes(projection='3d')
    # proto.plot_tf(ax, T_WF, name="Fiducial", size=tf_size)
    proto.plot_tf(ax, T_WL0, name="Joint0", size=tf_size)
    proto.plot_tf(ax, T_WL1, name="Joint1", size=tf_size)
    proto.plot_tf(ax, T_WL2, name="Joint2", size=tf_size)
    proto.plot_tf(ax, T_WC0, name="cam0", size=tf_size)
    proto.plot_tf(ax, T_WC1, name="cam1", size=tf_size)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    proto.plot_set_axes_equal(ax)
    plt.show()


def plot_camera_overlay(cam_idx, cam_frame, T_WS, joint0, joint1, joint2):
    gimbal_ext = np.array([-0.05, 0, -0.05, 1, 0, 0, 4.63268e-05])
    gimbal_link0 = np.array([0, -1.35525e-20, 0.05, 5.48441e-21, 0.707108, 2.39578e-21, 0.707105])
    gimbal_link1 = np.array([2.22045e-16, -1.35525e-20, 0.05, 0.707108, -4.03592e-17, -7.3647e-17, 0.707105])
    cam0_ext = np.array([-1.11022e-16, 1.00615e-17, 0.05, -0.5, 0.5, 0.500002, 0.499998])
    cam1_ext = np.array([0, 1.07552e-17, -0.05, -0.5, 0.5, 0.500002, 0.499998])

    cam_exts = {0: cam0_ext, 1: cam1_ext}
    links = [gimbal_link0, gimbal_link1]
    joint_angles = [joint0, joint1, joint2]
    gimbal_kinematics = proto.GimbalKinematics(links, joint_angles)

    q_WF = proto.euler321(-math.pi / 2.0, 0.0, math.pi / 2.0)
    r_WF = np.array([2, 0.49, 0])
    T_WF = proto.tf(q_WF, r_WF)

    T_M0L0 = proto.tf(proto.euler321(joint0, 0, 0), [0, 0, 0])
    T_L0M1 = proto.pose2tf(gimbal_link0)
    T_M1L1 = proto.tf(proto.euler321(joint1, 0, 0), [0, 0, 0])
    T_L1M2 = proto.pose2tf(gimbal_link1)
    T_M2L2 = proto.tf(proto.euler321(joint2, 0, 0), [0, 0, 0])
    T_M0L2 = T_M0L0 @ T_L0M1 @ T_M1L1 @ T_L1M2 @ T_M2L2

    T_SM0 = proto.pose2tf(gimbal_ext)
    T_M0L2 = gimbal_kinematics.forward_kinematics(joint_idx=2)
    T_L2Ci = proto.pose2tf(cam_exts[cam_idx])
    T_WCi = T_WS @ T_SM0 @ T_M0L2 @ T_L2Ci
    T_CiF = np.linalg.inv(T_WCi) @ T_WF

    calib_target = proto.AprilGrid(
        tag_rows=10,
        tag_cols=10,
        tag_size=0.08,
        tag_spacing=0.25
    )
    cam_idx = 0
    cam_res = [640, 640]
    cam0_params = np.array([554.38270568847656, 554.38270568847656, 320.0, 320.0, 0, 0, 0, 0])
    cam0_geom = proto.camera_geometry_setup(cam_idx, cam_res, "pinhole", "radtan4")

    tag_ids = []
    corner_idxs = []
    object_points = []
    keypoints = []
    for (tag_id, corner_idx, p_FFi) in calib_target.get_object_points():
      p_CiFi = proto.tf_point(T_CiF, p_FFi)
      status, z = cam0_geom.project(cam0_params, p_CiFi)
      if status:
        tag_ids.append(tag_id)
        corner_idxs.append(corner_idx)
        object_points.append(p_FFi)
        keypoints.append(z)

    cam_data = {
        "num_measurements": len(tag_ids),
        "tag_ids": tag_ids,
        "corner_idxs": corner_idxs,
        "object_points": np.array(object_points),
        "keypoints": np.array(keypoints),
    }

    return proto.draw_keypoints(cam_frame, keypoints)


class Sandbox(Node):
    def __init__(self):
        super().__init__('Sandbox')

        # Gimbal State
        self.cam0_frame = None
        self.joint0 = None
        self.joint1 = None
        self.joint2 = None
        self.mode = None
        self.target_attitude = None
        self.target_point = None

        # MAV Pose
        self.T_WS = None

        # AprilGrid Pose
        self.T_WF = None

        # ROS2
        self.cv_bridge = CvBridge()

        # Publishers
        self.pubs = {}
        self.add_pub('/gimbal/joint0/cmd', Float64)
        self.add_pub('/gimbal/joint1/cmd', Float64)
        self.add_pub('/gimbal/joint2/cmd', Float64)

        # Subscribers
        self.subs = {}
        cam0_sub = message_filters.Subscriber(self, Image, '/gimbal/camera0')
        cam1_sub = message_filters.Subscriber(self, Image, '/gimbal/camera1')
        joint0_sub = message_filters.Subscriber(self, JointState, '/gimbal/joint0/state')
        joint1_sub = message_filters.Subscriber(self, JointState, '/gimbal/joint1/state')
        joint2_sub = message_filters.Subscriber(self, JointState, '/gimbal/joint2/state')
        pose_sub = message_filters.Subscriber(self, PoseStamped, '/x500/pose/state')
        sync_subs = [cam0_sub, cam1_sub, joint0_sub, joint1_sub, joint2_sub, pose_sub]
        msg_time_syncer = message_filters.TimeSynchronizer(sync_subs, 5)
        msg_time_syncer.registerCallback(self.sync_cb)

        self.add_sub('/gimbal/mode/state', Int32, self.mode_cb)
        self.add_sub('/gimbal/target_attitude/state', Vector3, self.target_attitude_cb)
        self.add_sub('/gimbal/target_point/state', Vector3, self.target_point_cb)
        self.add_sub('/model/aprilgrid/pose', PoseStamped, self.aprilgrid_pose_cb)

    def add_pub(self, topic, msg_type, qs=1):
        self.pubs[topic] = self.create_publisher(msg_type, topic, qs)

    def add_sub(self, topic, msg_type, cb, qs=1):
        self.subs[topic] = self.create_subscription(msg_type, topic, cb, qs)

    def sync_cb(self, cam0_msg, cam1_msg, joint0_msg, joint1_msg, joint2_msg, pose_msg):
        cam0_ts, cam0_frame = convert_msg(cam0_msg)
        cam1_ts, cam1_frame = convert_msg(cam1_msg)
        joint0_ts, joint0 = convert_msg(joint0_msg)
        joint1_ts, joint1 = convert_msg(joint1_msg)
        joint2_ts, joint2 = convert_msg(joint2_msg)
        pose_ts, T_WS = convert_msg(pose_msg)

        # print(f"cam0_ts: {cam0_ts}")
        # print(f"cam1_ts: {cam1_ts}")
        # print(f"joint0_ts: {joint0_ts}")
        # print(f"joint1_ts: {joint1_ts}")
        # print(f"joint2_ts: {joint2_ts}")
        # print(f"pose_ts: {pose_ts}")
        # print(f"pose_msg: {pose_msg}")
        # print()

        cam0_viz = plot_camera_overlay(0, cam0_frame, T_WS, joint0, joint1, joint2)
        cv2.imshow("camera", cam0_viz)
        cv2.waitKey(1)

        # cam0_viz = plot_camera_overlay(0, cam0_frame, T_WS, joint0, joint1, joint2)
        # cam1_viz = plot_camera_overlay(1, cam1_frame, T_WS, joint0, joint1, joint2)
        # cv2.imshow("camera", cv2.hconcat([cam0_viz, cam1_viz]))
        # cv2.waitKey(1)

    def mode_cb(self, msg):
        self.mode = msg.data

    def target_attitude_cb(self, msg):
        self.target_attitude = np.array([msg.x, msg.y, msg.z])

    def target_point_cb(self, msg):
        self.target_point = np.array([msg.x, msg.y, msg.z])

    def aprilgrid_pose_cb(self, msg):
        ts, self.T_WF = convert_msg(msg)

    def set_joint(self, joint_idx, joint_angle):
        msg = Float64()
        msg.data = joint_angle
        self.pubs[f"/gimbal/joint{joint_idx}/cmd"].publish(msg)

    def get_joint(self, joint_idx):
        if joint_idx == 0:
            return self.joint0
        elif joint_idx == 1:
            return self.joint1
        elif joint_idx == 2:
            return self.joint2
        raise RunTimeError(f"Invalid joint index [{joint_idx}]")


if __name__ == '__main__':
    # plot_gimbal_chains()

    # Initialize
    rclpy.init(args=None)

    sandbox = Sandbox()
    while True:
        rclpy.spin_once(sandbox)

    sandbox.destroy_node()

    # Clean up
    rclpy.shutdown()
