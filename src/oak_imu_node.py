#!/usr/bin/env python3
import time
import cv2
import numpy as np
import matplotlib.pylab as plt
import depthai as dai


def ts2sec(delta):
  """Timestamp to Seconds"""
  return delta.total_seconds()


if __name__ == "__main__":
  # Create pipeline
  pipeline = dai.Pipeline()

  # Configure IMU
  imu_xout = pipeline.create(dai.node.XLinkOut)
  imu_xout.setStreamName("imu")

  imu = pipeline.create(dai.node.IMU)
  imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
  imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
  imu.setBatchReportThreshold(1)
  imu.setMaxBatchReports(1)
  imu.out.link(imu_xout.input)

  # Connect to device and start pipeline
  device = dai.Device(pipeline)

  # Loop
  imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=True)

  num_measurements = 0
  imu_acc_time = []
  imu_acc_data = []
  imu_gyr_time = []
  imu_gyr_data = []

  loop = True
  prev_ts = None
  rates = []
  while loop:
    imu_data = imu_queue.get()
    for imu_packet in imu_data.packets:
      imu_acc = imu_packet.acceleroMeter
      imu_gyr = imu_packet.gyroscope
      imu_acc_ts = ts2sec(imu_acc.getTimestampDevice())
      imu_gyr_ts = ts2sec(imu_gyr.getTimestampDevice())

      # Set prev timestamp
      if prev_ts is None:
        prev_ts = time.time_ns()
      else:
        now_ts = time.time_ns()
        rate = 1.0 / ((now_ts - prev_ts) * 1e-9)
        rates.append(rate)
        print(f"rate: {rate}")
        prev_ts = now_ts

      # Keep track of measurements
      imu_acc_time.append(imu_acc_ts)
      imu_gyr_time.append(imu_gyr_ts)
      imu_acc_data.append([imu_acc.x, imu_acc.y, imu_acc.z])
      imu_gyr_data.append([imu_gyr.x, imu_gyr.y, imu_gyr.z])

      # Break loop?
      num_measurements += 1
      if num_measurements > 100:
          loop = False
          break

  rates = np.array(rates)
  print(f"mean rate: {np.mean(rates)}")

  # Plot data
  imu_acc_time = np.array(imu_acc_time)
  imu_acc_data = np.array(imu_acc_data)
  imu_gyr_time = np.array(imu_gyr_time)
  imu_gyr_data = np.array(imu_gyr_data)
  print(f"imu rate: {1.0 / np.mean(np.diff(imu_acc_time))}")
  print(f"gyr rate: {1.0 / np.mean(np.diff(imu_gyr_time))}")

  # Boxplot
  plt.figure()
  plt.boxplot([1.0 / np.diff(imu_acc_time), 1.0 / np.diff(imu_gyr_time)])
  plt.xticks([1, 2], ["Accelerometer", "Gyroscope"])
  plt.ylabel("Rate [Hz]")

  # Time series plot
  plt.figure()
  plt.plot(1.0 / np.diff(imu_acc_time), 'r.', label="Acceleromenter")
  plt.plot(1.0 / np.diff(imu_gyr_time), 'b.', label="Gyroscope")
  plt.legend(loc=0)
  plt.xlim([0, num_measurements])
  plt.xlabel("i-th Measurement")
  plt.ylabel("Rate [Hz]")

  plt.show()
