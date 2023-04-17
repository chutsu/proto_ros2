import math

import rclpy
from rclpy.node import Node

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


def plot_gimbal_chains():
    # T_WF = np.array([[-3.67320510e-06,  1.00000000e+00, -3.67320510e-06,  2.00000000e+00],
    #                  [-5.55111512e-17, -3.67320510e-06, -1.00000000e+00,  4.90000000e-01],
    #                  [-1.00000000e+00, -3.67320510e-06,  1.34924849e-11,  0.00000000e+00],
    #                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    q_WF = proto.euler321(-math.pi / 2.0, 0.0, math.pi / 2.0)
    r_WF = np.array([2, 0.49, 0])
    T_WF = proto.tf(q_WF, r_WF)

    T_WB = proto.pose2tf([0, 0, 0.235, 0, 0, 0, 1])
    T_BM0 = proto.pose2tf([-0.05, 0, -0.02, 1, 0, 0, 4.63268e-05])
    T_M0M1 = proto.pose2tf([0.0175, 0, 0.0425, 5.48441e-21, 0.707108, 2.39578e-21, 0.707105])
    T_M1M2 = proto.pose2tf([0, 0.0175, 0.0425, -0.707108, -7.29657e-17, 1.33061e-16, 0.707105])
    T_M2C0 = proto.pose2tf([7.96326e-05, 8.67362e-18, -0.0499999, 0.499602, -0.500398, 0.5004, 0.4996])
    T_M2C1 = proto.pose2tf([-7.96326e-05, -1.73472e-18, 0.0499999, 0.499602, -0.500398, 0.5004, 0.4996])

    T_WL0 = T_WB @ T_BM0
    T_WL1 = T_WB @ T_BM0 @ T_M0M1
    T_WL2 = T_WB @ T_BM0 @ T_M0M1 @ T_M1M2
    T_WC0 = T_WB @ T_BM0 @ T_M0M1 @ T_M1M2 @ T_M2C0
    T_WC1 = T_WB @ T_BM0 @ T_M0M1 @ T_M1M2 @ T_M2C1

    # Visualize
    plt.figure()
    ax = plt.axes(projection='3d')
    proto.plot_tf(ax, T_WF, name="Fiducial", size=0.05)
    proto.plot_tf(ax, T_WL0, name="Link0", size=0.05)
    proto.plot_tf(ax, T_WL1, name="Link1", size=0.05)
    proto.plot_tf(ax, T_WL2, name="Link2", size=0.05)
    proto.plot_tf(ax, T_WC0, name="cam0", size=0.05)
    proto.plot_tf(ax, T_WC1, name="cam1", size=0.05)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    proto.plot_set_axes_equal(ax)
    plt.show()


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

        # self.r_SB = np.array([-0.05, 0.0, -0.02])
        # self.q_SB = proto.euler321(0, 0, 3.1415)
        # self.T_SB = proto.tf(self.q_SB, self.r_SB)

        # ROS2
        self.cv_bridge = CvBridge()

        # Publishers
        self.pubs = {}
        self.add_pub('/gimbal/joint0/cmd', Float64)
        self.add_pub('/gimbal/joint1/cmd', Float64)
        self.add_pub('/gimbal/joint2/cmd', Float64)

        # Subscribers
        self.subs = {}
        # -- Gimbal
        self.add_sub('/gimbal/camera0', Image, self.camera0_cb)
        # self.add_sub('/gimbal/camera1', Image, self.camera1_cb)
        self.add_sub('/gimbal/joint0/state', JointState, self.joint0_cb)
        self.add_sub('/gimbal/joint1/state', JointState, self.joint1_cb)
        self.add_sub('/gimbal/joint2/state', JointState, self.joint2_cb)
        self.add_sub('/gimbal/mode/state', Int32, self.mode_cb)
        self.add_sub('/gimbal/target_attitude/state', Vector3, self.target_attitude_cb)
        self.add_sub('/gimbal/target_point/state', Vector3, self.target_point_cb)
        # -- AprilGrid
        self.add_sub('/model/aprilgrid/pose', PoseStamped, self.aprilgrid_pose_cb)
        # -- MAV
        self.add_sub('/model/x500/pose', PoseStamped, self.mav_pose_cb)

    def all_ok(self):
        check_list = []
        check_list.append(("cam0_frame", self.cam0_frame))
        check_list.append(("joint0", self.joint0))
        check_list.append(("joint1", self.joint1))
        check_list.append(("joint2", self.joint2))
        check_list.append(("mode", self.mode))
        check_list.append(("target_attitude", self.target_attitude))
        check_list.append(("target_point", self.target_point))
        check_list.append(("mav_pose", self.T_WS))
        check_list.append(("fiducial_pose", self.T_WF))

        for key, x in check_list:
            if x is None:
                return False

        return True

    def add_pub(self, topic, msg_type, qs=1):
        self.pubs[topic] = self.create_publisher(msg_type, topic, qs)

    def add_sub(self, topic, msg_type, cb, qs=1):
        self.subs[topic] = self.create_subscription(msg_type, topic, cb, qs)

    def camera0_cb(self, msg):
        self.cam0_frame = self.cv_bridge.imgmsg_to_cv2(msg)

        if self.all_ok() is False:
            return

        gimbal_ext = np.array([-0.05, 0, -0.02, 1, 0, 0, 4.63268e-05])   # T_SM0
        gimbal_link0 = np.array([0.0175, 0, 0.0425, 5.48441e-21, 0.707108, 2.39578e-21, 0.707105])
        gimbal_link1 = np.array([0, 0.0175, 0.0425, -0.707108, -7.29657e-17, 1.33061e-16, 0.707105])
        cam0_ext = np.array([7.96326e-05, 8.67362e-18, -0.0499999, 0.499602, -0.500398, 0.5004, 0.4996])
        cam1_ext = np.array([-7.96326e-05, -1.73472e-18, 0.0499999, 0.499602, -0.500398, 0.5004, 0.4996])
        links = [gimbal_link0, gimbal_link1]
        joint_angles = [self.joint0, self.joint1, self.joint2]
        gimbal_kinematics = proto.GimbalKinematics(links, joint_angles)

        q_WF = proto.euler321(-math.pi / 2.0, 0.0, math.pi / 2.0)
        r_WF = np.array([2, 0.49, 0])
        T_WF = proto.tf(q_WF, r_WF)

        # T_WF = self.T_WF
        T_WS = self.T_WS
        T_SM0 = proto.pose2tf(gimbal_ext)
        T_M0L2 = gimbal_kinematics.forward_kinematics(joint_idx=2)
        T_L2C0 = proto.pose2tf(cam0_ext)
        T_WC0 = T_WS @ T_SM0 @ T_M0L2 @ T_L2C0
        T_C0F = np.linalg.inv(T_WC0) @ T_WF

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
          p_C0Fi = proto.tf_point(T_C0F, p_FFi)
          status, z = cam0_geom.project(cam0_params, p_C0Fi)
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

        viz = proto.draw_keypoints(self.cam0_frame, keypoints)
        cv2.imshow("camera", viz)
        cv2.waitKey(1)

#     def camera1_cb(self, msg):
#         frame = self.cv_bridge.imgmsg_to_cv2(msg)
#         cv2.imshow("camera", frame)
#         cv2.waitKey(1)

    def joint0_cb(self, msg):
        self.joint0 = msg.position[0]

    def joint1_cb(self, msg):
        self.joint1 = msg.position[0]

    def joint2_cb(self, msg):
        self.joint2 = msg.position[0]

    def mode_cb(self, msg):
        self.mode = msg.data

    def target_attitude_cb(self, msg):
        self.target_attitude = np.array([msg.x, msg.y, msg.z])

    def target_point_cb(self, msg):
        self.target_point = np.array([msg.x, msg.y, msg.z])

    def aprilgrid_pose_cb(self, msg):
        pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.w
        ]
        self.T_WF = proto.pose2tf(pose)

    def mav_pose_cb(self, msg):
        pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.w
        ]
        self.T_WS = proto.pose2tf(pose)

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
