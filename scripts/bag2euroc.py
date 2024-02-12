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
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge


def mkdir(dir_path):
  if not os.path.exists(dir_path):
    os.makedirs(dir_path)

  if not os.path.exists(dir_path):
    raise RuntimeError(f"Failed to create [{dir_path}]")


class MocapBagProcessor(Node):
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
    self.cam0_dir = os.path.join(save_dir, "cam0", "data")
    self.cam1_dir = os.path.join(save_dir, "cam1", "data")
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

  def cam2_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.cam2_dir, f"{ts}.png")
    cv2.imwrite(image_path, image)

  def cam3_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.cam3_dir, f"{ts}.png")
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
    f.write(f"{ts},{rx},{ry},{rz},{qx},{qy},{qz},{qw}\n")

  def grid0_cb(self, msg):
    self._write_pose(self.grid0_csv, msg)

  def body0_cb(self, msg):
    self._write_pose(self.body0_csv, msg)

  def okvis_cb(self, msg):
    self._write_pose(self.okvis_csv, msg)


class ExperimentBagProcessor(Node):
  def __init__(self, save_dir):
    super().__init__("bag_node")

    # ROS topics
    topic_imu0 = "/rs/imu0"
    topic_imu1 = "/rs/imu1"
    topic_cam0 = "/rs/ir0/image"
    topic_cam1 = "/rs/ir1/image"
    topic_cam2 = "/rs/ir2/image"
    topic_cam3 = "/rs/ir3/image"
    topic_mocap = "/vicon/realsense_d435/realsense_d435"
    topic_pose = "/okvis/pose"
    topic_tracking = "/okvis/tracking"
    topic_sbgc = "/sbgc/joints"

    # Create directories
    self.imu0_dir = os.path.join(save_dir, "imu0")
    self.imu1_dir = os.path.join(save_dir, "imu1")
    self.cam0_dir = os.path.join(save_dir, "cam0", "data")
    self.cam1_dir = os.path.join(save_dir, "cam1", "data")
    self.cam2_dir = os.path.join(save_dir, "cam2", "data")
    self.cam3_dir = os.path.join(save_dir, "cam3", "data")
    self.mocap_dir = os.path.join(save_dir, "mocap")
    self.okvis_dir = os.path.join(save_dir, "okvis")
    self.sbgc_dir = os.path.join(save_dir, "sbgc")
    self.tracking_dir = os.path.join(save_dir, "tracking", "data")
    mkdir(save_dir)
    mkdir(self.imu0_dir)
    mkdir(self.imu1_dir)
    mkdir(self.cam0_dir)
    mkdir(self.cam1_dir)
    mkdir(self.cam2_dir)
    mkdir(self.cam3_dir)
    mkdir(self.mocap_dir)
    mkdir(self.okvis_dir)
    mkdir(self.tracking_dir)
    mkdir(self.sbgc_dir)

    # Initialize imu0 csv file
    imu0_csv = os.path.join(self.imu0_dir, "data.csv")
    self.imu0_csv = open(imu0_csv, "w")
    self.imu0_csv.write(f"timestamp,wx,wy,wz,ax,ay,az\n")

    # Initialize imu0 csv file
    imu1_csv = os.path.join(self.imu1_dir, "data.csv")
    self.imu1_csv = open(imu1_csv, "w")
    self.imu1_csv.write(f"timestamp,wx,wy,wz,ax,ay,az\n")

    # Initialize mocap csv file
    okvis_csv = os.path.join(self.mocap_dir, "data.csv")
    self.mocap_csv = open(mocap_csv, "w")
    self.mocap_csv.write(f"timestamp,rx,ry,rz,qx,qy,qz,qw\n")

    # Initialize okvis_pose csv file
    okvis_csv = os.path.join(self.okvis_dir, "data.csv")
    self.okvis_csv = open(okvis_csv, "w")
    self.okvis_csv.write(f"timestamp,rx,ry,rz,qx,qy,qz,qw\n")

    # Initialize sbgc csv file
    sbgc_csv = os.path.join(self.sbgc_dir, "data.csv")
    self.sbgc_csv = open(sbgc_csv, "w")
    self.sbgc_csv.write(f"timestamp,theta0,theta1,theta2\n")

    # Initialize subscribers
    self.cv_bridge = CvBridge()
    init_sub = self.create_subscription
    self.sub_imu0 = init_sub(Imu, topic_imu0, self.imu0_cb, 100)
    self.sub_imu1 = init_sub(Imu, topic_imu1, self.imu0_cb, 100)
    self.sub_cam0 = init_sub(Image, topic_cam0, self.cam0_cb, 100)
    self.sub_cam1 = init_sub(Image, topic_cam1, self.cam1_cb, 100)
    self.sub_cam2 = init_sub(Image, topic_cam2, self.cam2_cb, 100)
    self.sub_cam3 = init_sub(Image, topic_cam3, self.cam3_cb, 100)
    self.sub_mocap = init_sub(PoseStamped, topic_mocap, self.mocap_cb, 100)
    self.sub_okvis = init_sub(PoseStamped, topic_pose, self.pose_cb, 100)
    self.sub_tracking = init_sub(Image, topic_tracking, self.tracking_cb, 100)
    self.sub_sbgc = init_sub(Vector3Stamped, topic_sbgc, self.sbgc_cb, 100)

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

  def cam2_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.cam2_dir, f"{ts}.png")
    cv2.imwrite(image_path, image)

  def cam3_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.cam3_dir, f"{ts}.png")
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
    f.write(f"{ts},{rx},{ry},{rz},{qx},{qy},{qz},{qw}\n")

  def mocap_cb(self, msg):
    self._write_pose(self.mocap_csv, msg)

  def pose_cb(self, msg):
    self._write_pose(self.okvis_csv, msg)

  def tracking_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    image_path = os.path.join(self.tracking_dir, f"{ts}.png")
    cv2.imwrite(image_path, image)

  def sbgc_cb(self, msg):
    print(".", end="")
    sys.stdout.flush()

    ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    theta0 = msg.vector.x
    theta1 = msg.vector.y
    theta2 = msg.vector.z
    self.sbgc_csv.write(f"{ts},{theta0},{theta1},{theta2}\n")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--mode', required=True)
  parser.add_argument('--save_dir', required=True)
  args = parser.parse_args()

  if os.path.exists(args.save_dir):
    raise RuntimeError(f"Save dir [{args.save_dir}] already exists!")

  rclpy.init()
  if args.mode == "mocap":
    node = MocapBagProcessor(args.save_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  elif args.mode == "exp":
    node = ExperimentBagProcessor(args.save_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  else:
    print(f"Invalid mode [{args.mode}]!")
