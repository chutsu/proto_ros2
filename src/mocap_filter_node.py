#!/usr/bin/evn python3
import numpy as np
import proto
from proto import KalmanFilter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy
from geometry_msgs.msg import PoseStamped


class MocapFilterNode(Node):
  def __init__(self):
    # Setup Kalman Filter
    # -- Initial state x0
    # x0 = [rx, ry, rz, vx, vy, vz, ax, ay, az]
    x0 = np.zeros(9)
    # -- Transition Matrix
    F = np.eye(9)
    F[0:3, 3:6] = np.eye(3) * dt
    F[0:3, 6:9] = np.eye(3) * dt**2
    F[3:6, 6:9] = np.eye(3) * dt
    # -- Measurement Matrix
    H = np.block([np.eye(3), np.zeros((3, 6))])
    # -- Input Matrix
    B = np.array([0])
    # -- Process Noise Matrix
    Q = 0.1 * np.eye(9)
    # -- Measurement Noise Matrix
    R = 10.0 * np.eye(3)
    # -- Kalman Filter
    kf_kwargs = {"x0": x0, "F": F, "H": H, "B": B, "Q": Q, "R": R}
    self.kf = KalmanFilter(**kf_kwargs)

    # Configure QoS profile for publishing and subscribing
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST,
                     depth=1)

    # Create subscribers and publishers
    topic_pose_in = "/vicon/pose"
    topic_pose_out = "/vicon/pose_filtered"
    sub_init = self.create_subscription
    pub_init = self.create_publisher
    self.sub_pose = sub_init(PoseStamped, topic_pose_in, self.pose_cb, qos)
    self.pub_pose = pub_init(PoseStamped, topic_pose_out, qos)

    # Create timer
    self.dt = 0.001
    self.timer = self.create_timer(self.dt, self.timer_cb)

  def pose_cb(self, msg):
    """Callback function for pose."""
    rx = msg.pose.position.x
    ry = msg.pose.position.y
    rz = msg.pose.position.z
    z = np.array([rx, ry, rz])
    kf.update(z)

  def timer_cb(self):
    """Callback function for the timer."""
    kf.predict()


if __name__ == '__main__':
  rclpy.init()
  node = MocapFilterNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()
