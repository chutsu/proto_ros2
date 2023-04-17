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

    def add_pub(self, topic, msg_type, qs=1):
        self.pubs[topic] = self.create_publisher(msg_type, topic, qs)

    def add_sub(self, topic, msg_type, cb, qs=1):
        self.subs[topic] = self.create_subscription(msg_type, topic, cb, qs)

    def camera0_cb(self, msg):
        self.cam0_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        # cv2.imshow("camera", self.cam0_frame)
        # cv2.waitKey(1)

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
        aprilgrid_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.w
        ]
        self.T_WF = proto.pose2tf(aprilgrid_pose)

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
    # Initialize
    rclpy.init(args=None)

    sandbox = Sandbox()
    # rclpy.spin(sandbox)
    while True:
        rclpy.spin_once(sandbox)
    sandbox.destroy_node()

    # Clean up
    rclpy.shutdown()
