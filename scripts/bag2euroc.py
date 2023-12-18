#!/usr/bin/env python3
import os
import sys
import argparse

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


def mkdir(dir_path):
  if not os.path.exists(dir_path):
    os.makedirs(dir_path)

  if not os.path.exists(dir_path):
    raise RuntimeError(f"Failed to create [{dir_path}]")


class BagNode(Node):
  def __init__(self, save_dir):
    super().__init__("bag_node")

    # ROS topics
    topic_cam0 = "/rs/ir0/image"
    topic_cam1 = "/rs/ir1/image"
    topic_imu0 = "/rs/imu0"
    topic_body0 = "/vicon/realsense_d435/realsense_d435"
    topic_grid0 = "/vicon/aprilgrid/aprilgrid"
    topic_okvis = "/okvis/pose"

    # Create directories
    self.cam0_dir = os.path.join(save_dir, "cam0")
    self.cam1_dir = os.path.join(save_dir, "cam1")
    self.imu0_dir = os.path.join(save_dir, "imu0")
    self.grid0_dir = os.path.join(save_dir, "grid0")
    self.body0_dir = os.path.join(save_dir, "body0")
    self.okvis_dir = os.path.join(save_dir, "okvis")
    mkdir(save_dir)
    mkdir(self.cam0_dir)
    mkdir(self.cam1_dir)
    mkdir(self.imu0_dir)
    mkdir(self.grid0_dir)
    mkdir(self.body0_dir)
    mkdir(self.okvis_dir)

    # Initialize imu0 csv file
    imu0_csv = os.path.join(self.imu0_dir, "data.csv")
    self.imu0_csv = open(imu0_csv, "w")
    self.imu0_csv.write(f"timestamp,wx,wy,wz,ax,ay,az\n")

    # Initialize grid0 csv file
    grid0_csv = os.path.join(self.grid0_dir, "data.csv")
    self.grid0_csv = open(grid0_csv, "w")
    self.grid0_csv.write(f"timestamp,rx,ry,rz,qx,qy,qz,qw\n")

    # Initialize body0 csv file
    body0_csv = os.path.join(self.body0_dir, "data.csv")
    self.body0_csv = open(body0_csv, "w")
    self.body0_csv.write(f"timestamp,rx,ry,rz,qx,qy,qz,qw\n")

    # Initialize okvis_pose csv file
    okvis_csv = os.path.join(self.okvis_dir, "data.csv")
    self.okvis_csv = open(okvis_csv, "w")
    self.okvis_csv.write(f"timestamp,rx,ry,rz,qx,qy,qz,qw\n")

    # Initialize subscribers
    self.cv_bridge = CvBridge()
    init_sub = self.create_subscription
    self.sub_cam0 = init_sub(Image, topic_cam0, self.cam0_cb, 100)
    self.sub_cam1 = init_sub(Image, topic_cam1, self.cam1_cb, 100)
    self.sub_imu0 = init_sub(Imu, topic_imu0, self.imu0_cb, 100)
    self.sub_grid0 = init_sub(PoseStamped, topic_grid0, self.grid0_cb, 100)
    self.sub_body0 = init_sub(PoseStamped, topic_body0, self.body0_cb, 100)
    self.sub_okvis = init_sub(PoseStamped, topic_okvis, self.okvis_cb, 100)

  def cam0_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.cam0_dir, f"{ts}.png")
    cv2.imwrite(image_path, image)

  def cam1_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.cam1_dir, f"{ts}.png")
    cv2.imwrite(image_path, image)

  def imu0_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    wx = msg.angular_velocity.x
    wy = msg.angular_velocity.y
    wz = msg.angular_velocity.z
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    self.imu0_csv.write(f"{ts},{wx},{wy},{wz},{ax},{ay},{az}\n")

  def _write_pose(self, f, msg):
    print(".", end="")
    sys.stdout.flush()

    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    rx = msg.pose.position.x
    ry = msg.pose.position.y
    rz = msg.pose.position.z
    qw = msg.pose.orientation.w
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    f.write(f"{ts},{rx},{ry},{rz},{qw},{qx},{qy},{qz}\n")

  def grid0_cb(self, msg):
    self._write_pose(self.grid0_csv, msg)

  def body0_cb(self, msg):
    self._write_pose(self.body0_csv, msg)

  def okvis_cb(self, msg):
    self._write_pose(self.okvis_csv, msg)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--save_dir')
  args = parser.parse_args()

  if os.path.exists(args.save_dir):
    raise RuntimeError(f"Save dir [{save_dir}] already exists!")

  rclpy.init()
  bag_node = BagNode(args.save_dir)
  rclpy.spin(bag_node)
  bag_node.destroy_node()
  rclpy.shutdown()
