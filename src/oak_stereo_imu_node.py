#!/usr/bin/env python3
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
  mono_left.out.link(mono_left_xout.input)
  mono_right.out.link(mono_right_xout.input)

  ctrl_in = pipeline.create(dai.node.XLinkIn)
  ctrl_in.setStreamName("control")
  ctrl_in.out.link(mono_left.inputControl)
  ctrl_in.out.link(mono_right.inputControl)

  # # Configure IMU
  # imu = pipeline.create(dai.node.IMU)
  # imu_xout = pipeline.create(dai.node.XLinkOut)

  # imu_xout.setStreamName("imu")
  # imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
  # imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
  # imu.setBatchReportThreshold(1)
  # imu.setMaxBatchReports(10)
  # imu.out.link(imu_xout.input)

  # Connect to device and start pipeline
  device = dai.Device(pipeline)

  # Loop
  image_sync = ImageQueueSynchronizer(device, ["left", "right"])
  # imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
  ctrl_queue = device.getInputQueue(ctrl_in.getStreamName())

  # exp_time = 2000
  # exp_min = 1
  # exp_max = 33000
  # exp_step = 500  # us

  # sens_iso = 800
  # sens_min = 100
  # sens_max = 2000
  # sens_step = 50

  # Configure Camera properties
  # print("Setting manual exposure, time:", exp_time, "iso:", sens_iso)
  # exp_time = clamp(exp_time, exp_min, exp_max)
  # sens_iso = clamp(sens_iso, sens_min, sens_max)
  # ctrl = dai.CameraControl()
  # ctrl.setManualExposure(exp_time, sens_iso)
  # ctrl_queue.send(ctrl)

  base_ts = None
  while True:
    # imu_data = imu_queue.get()
    # imu_packets = imu_data.packets
    # for imu_packet in imu_packets:
    #   imu_acc = imu_packet.acceleroMeter
    #   imu_gyr = imu_packet.gyroscope

    #   acc_ts = imu_acc.getTimestampDevice()
    #   gyr_ts = imu_gyr.getTimestampDevice()
    #   if base_ts is None:
    #     base_ts = acc_ts if acc_ts < gyr_ts else gyr_ts
    #   acc_ts = ts2sec(acc_ts - base_ts)
    #   gyr_ts = ts2sec(gyr_ts - base_ts)

    #   imuF = "{:.06f}"
    #   tsF  = "{:.03f}"

    #   print(f"imu_acc ts: {tsF.format(acc_ts)} ms")
    #   print(f"imu_acc [m/s^2]: x: {imuF.format(imu_acc.x)} y: {imuF.format(imu_acc.y)} z: {imuF.format(imu_acc.z)}")
    #   print(f"imu_gyr ts: {tsF.format(gyr_ts)} ms")
    #   print(f"imu_gyr [rad/s]: x: {imuF.format(imu_gyr.x)} y: {imuF.format(imu_gyr.y)} z: {imuF.format(imu_gyr.z)} ")

    if image_sync.update():
      frame_left = image_sync.buffer["left"]
      frame_right = image_sync.buffer["right"]
      image_sync.clear()

      frame_left_ts = frame_left.getTimestampDevice().total_seconds()
      frame_right_ts = frame_right.getTimestampDevice().total_seconds()
      time_delta = abs(frame_left_ts - frame_right_ts)
      # print(f"stereo frame ts delta: {time_delta}")

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
