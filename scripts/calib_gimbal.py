#!/usr/bin/env python3
import time
import threading

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber

# Global variables

class CalibGimbalNode(Node):
  def __init__(self):
    super().__init__("calib_gimbal_node")
    self.lock = threading.Lock()  # Create a Lock object

    # -- Subscribers
    # self.sub_joints = self.create_subscription(Vector3, '/sbgc/joints', self.joints_callback, 1)
    self.sub_ir0 = Subscriber(self, Image, '/rs/ir0/image')
    self.sub_ir1 = Subscriber(self, Image, '/rs/ir1/image')
    self.sub_joints = Subscriber(self, Vector3Stamped, '/sbgc/joints')
    self.sync = ApproximateTimeSynchronizer([self.sub_ir0, self.sub_ir1, self.sub_joints], 1, 0.05)
    self.sync.registerCallback(self.image_callback)
    # -- Publishers
    self.pub_joints = self.create_publisher(Vector3Stamped, '/sbgc/set_joints', 1)
    # -- Timer
    timer_period = 5
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Target angles
    self.target_angles = [
      [0.0, 0.0, 45.0],
      [0.0, 0.0, -45.0]
    ]

    # Data
    self.cv_bridge = CvBridge()
    self.shutdown = False
    self.frame0 = None
    self.frame1 = None
    self.joints = None
    self.capture = False

  def image_callback(self, cam0_msg, cam1_msg, joints_msg):
    with self.lock:
      self.frame0 = self.cv_bridge.imgmsg_to_cv2(cam0_msg)
      self.frame1 = self.cv_bridge.imgmsg_to_cv2(cam1_msg)
      self.joints = [joints_msg.vector.x, joints_msg.vector.y, joints_msg.vector.z]

  def timer_callback(self):
    with self.lock:
      if len(self.target_angles) == 0:
        self.shutdown = True
        return

      # Get target angle
      target = self.target_angles[0]
      self.target_angles.pop(0)

      # Request target angle
      msg = Vector3Stamped()
      msg.vector.x = target[0]
      msg.vector.y = target[1]
      msg.vector.z = target[2]
      self.pub_joints.publish(msg)
      time.sleep(4)

      # Capture
      print(self.joints)
      viz = cv2.hconcat([self.frame0, self.frame1])
      cv2.imshow("frame", viz)
      cv2.waitKey(1)


if __name__ == "__main__":
  rclpy.init()
  node = CalibGimbalNode()

  while rclpy.ok() and node.shutdown is False:
    rclpy.spin_once(node, timeout_sec=0.1)

  rclpy.shutdown()
