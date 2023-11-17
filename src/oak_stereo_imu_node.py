#!/usr/bin/env python3
import os
import time
import pathlib
import argparse


import cv2
import numpy as np
import depthai as dai


def ts2sec(delta):
  return delta.total_seconds()


def clamp(num, v0, v1):
  return max(v0, min(num, v1))


class ImageQueueSynchronizer:
  def __init__(self, device, stream_names):
    self.queues = {}
    self.buffer = {}
    self.tolerance = 1e-3

    for name in stream_names:
      settings = {"name": name, "maxSize": 1, "blocking": False}
      self.queues[name] = device.getOutputQueue(**settings)

  def update(self):
    for stream_name, queue in self.queues.items():
      # Check frame is ready
      frame = queue.tryGet()
      if not frame:
        continue

      # Add frame to buffer
      self.buffer[stream_name] = frame

      # Check timestamps
      frame_ts = frame.getTimestampDevice().total_seconds()
      clear_buffer = False
      for stream_name, buffer_frame in self.buffer.items():
        buffer_frame_ts = buffer_frame.getTimestampDevice().total_seconds()
        time_delta = abs(frame_ts - buffer_frame_ts)
        clear_buf = False if time_delta < self.tolerance else True

      if clear_buf:
        self.clear()

    # Buffer is ready?
    if len(self.buffer) == len(self.queues):
      return True

    return False

  def clear(self):
    self.buffer.clear()


if __name__ == "__main__":
  # parser = argparse.ArgumentParser()
  # parser.add_argument('--output_path')
  # args = parser.parse_args()

  # # Setup output directories
  # output_path = args.output_path
  # cam0_path = os.path.join(args.output_path, "cam0")
  # cam1_path = os.path.join(args.output_path, "cam1")
  # cam0_data_path = os.path.join(args.output_path, "cam0", "data")
  # cam1_data_path = os.path.join(args.output_path, "cam1", "data")
  # imu0_path = os.path.join(args.output_path, "imu0")

  # pathlib.Path(output_path).mkdir(parents=True, exist_ok=True)
  # pathlib.Path(cam0_path).mkdir(exist_ok=True)
  # pathlib.Path(cam1_path).mkdir(exist_ok=True)
  # pathlib.Path(cam0_data_path).mkdir(exist_ok=True)
  # pathlib.Path(cam1_data_path).mkdir(exist_ok=True)
  # pathlib.Path(imu0_path).mkdir(exist_ok=True)

  # Create pipeline
  pipeline = dai.Pipeline()

  # Configure cameras
  mono_left = pipeline.create(dai.node.MonoCamera)
  mono_right = pipeline.create(dai.node.MonoCamera)
  mono_left_xout = pipeline.create(dai.node.XLinkOut)
  mono_right_xout = pipeline.create(dai.node.XLinkOut)

  mono_left.setCamera("left")
  mono_right.setCamera("right")
  mono_left_xout.setStreamName('left')
  mono_right_xout.setStreamName('right')

  mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  mono_left.setFps(10)
  mono_right.setFps(10)
  mono_left.out.link(mono_left_xout.input)
  mono_right.out.link(mono_right_xout.input)

  ctrl_in = pipeline.create(dai.node.XLinkIn)
  ctrl_in.setStreamName("control")
  ctrl_in.out.link(mono_left.inputControl)
  ctrl_in.out.link(mono_right.inputControl)

  # Configure IMU
  imu = pipeline.create(dai.node.IMU)
  imu_xout = pipeline.create(dai.node.XLinkOut)

  imu_xout.setStreamName("imu")
  imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 250)
  imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
  imu.setBatchReportThreshold(1)
  imu.setMaxBatchReports(10)
  imu.out.link(imu_xout.input)

  # Connect to device and start pipeline
  device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

  # Loop
  image_sync = ImageQueueSynchronizer(device, ["left", "right"])
  imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=True)
  ctrl_queue = device.getInputQueue(ctrl_in.getStreamName())

  # # exp_time = 2000
  # # exp_min = 1
  # # exp_max = 33000
  # # exp_step = 500  # us

  # # sens_iso = 800
  # # sens_min = 100
  # # sens_max = 2000
  # # sens_step = 50

  # # Configure Camera properties
  # # print("Setting manual exposure, time:", exp_time, "iso:", sens_iso)
  # # exp_time = clamp(exp_time, exp_min, exp_max)
  # # sens_iso = clamp(sens_iso, sens_min, sens_max)
  # # ctrl = dai.CameraControl()
  # # ctrl.setManualExposure(exp_time, sens_iso)
  # # ctrl_queue.send(ctrl)

  base_ts = time.time_ns()
  imu_buffer = []
  while True:
    imu_data = imu_queue.get()
    for imu_packet in imu_data.packets:
      imu_acc = imu_packet.acceleroMeter
      imu_gyr = imu_packet.gyroscope
      acc_ts = imu_acc.getTimestampDevice()
      gyr_ts = imu_gyr.getTimestampDevice()
      imu_buffer.append([
        int(base_ts + acc_ts.total_seconds() * 1e9),
        imu_acc.x, imu_acc.y, imu_acc.z,
        imu_gyr.x, imu_gyr.y, imu_gyr.z
      ])

    if image_sync.update():
      frame_left = image_sync.buffer["left"]
      frame_right = image_sync.buffer["right"]
      image_sync.clear()

      frame_left_time = frame_left.getTimestampDevice().total_seconds()
      frame_right_time = frame_right.getTimestampDevice().total_seconds()
      time_delta = abs(frame_left_time - frame_right_time)
      frame_ts = int(base_ts + frame_left_time * 1e9)
      # print(f"stereo frame ts delta: {time_delta}")
      assert(time_delta < 1e-4)

      # cv2.imwrite(f'{cam0_data_path}/{frame_ts}.png', frame_left.getCvFrame())
      # cv2.imwrite(f'{cam1_data_path}/{frame_ts}.png', frame_right.getCvFrame())

      viz = cv2.hconcat([frame_left.getCvFrame(), frame_right.getCvFrame()])
      cv2.imshow("Viz", viz)
      key = cv2.waitKey(1)
      if key == ord('q'):
        break

      # elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
      #   if key == ord('i'):
      #     exp_time -= exp_step
      #   if key == ord('o'):
      #     exp_time += exp_step
      #   if key == ord('k'):
      #     sens_iso -= sens_step
      #   if key == ord('l'):
      #     sens_iso += sens_step
      #   exp_time = clamp(exp_time, exp_min, exp_max)
      #   sens_iso = clamp(sens_iso, sens_min, sens_max)
      #   print("Setting manual exposure, time:", exp_time, "iso:", sens_iso)
      #   ctrl = dai.CameraControl()
      #   ctrl.setManualExposure(exp_time, sens_iso)
      #   ctrl_queue.send(ctrl)

  # Save IMU data
  # imu_buffer = np.array(imu_buffer)
  # csv_fmt = '%ld,%f,%f,%f,%f,%f,%f'
  # np.savetxt(f"{imu0_path}/data.csv", imu_buffer, delimiter=',', fmt=csv_fmt)
