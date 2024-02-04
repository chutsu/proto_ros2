#!/usr/bin/env python3
import os
import pandas
import numpy as np
import matplotlib.pylab as plt

if __name__ == "__main__":
  imu_csv = "/home/chutsu/calib_camimu-rs0/imu0/data.csv"
  acc_csv = "/home/chutsu/calib_camimu-rs0/imu0/accel_data.csv"
  gyr_csv = "/home/chutsu/calib_camimu-rs0/imu0/gyro_data.csv"

  imu_data = pandas.read_csv(imu_csv, delimiter=",")
  first_ts = imu_data["#ts"][0]
  imu_time = (imu_data["#ts"] - first_ts) * 1e-9

  acc_data = pandas.read_csv(acc_csv, delimiter=",")
  acc_time = (acc_data["#ts"] - first_ts) * 1e-9

  gyr_data = pandas.read_csv(gyr_csv, delimiter=",")
  gyr_time = (gyr_data["#ts"] - first_ts) * 1e-9

  # Plot accelerometer data
  plt.figure()
  plt.subplot(211)
  plt.plot(imu_time, imu_data["acc_x"], "r.-", label="Acc - x")
  plt.plot(imu_time, imu_data["acc_y"], "g.-", label="Acc - y")
  plt.plot(imu_time, imu_data["acc_z"], "b.-", label="Acc - z")
  plt.subplot(212)
  plt.plot(acc_time, acc_data["acc_x"], "r.-", label="Acc - x")
  plt.plot(acc_time, acc_data["acc_y"], "g.-", label="Acc - y")
  plt.plot(acc_time, acc_data["acc_z"], "b.-", label="Acc - z")

  # Plot gyroscope data
  plt.figure()
  plt.subplot(211)
  plt.plot(imu_time, imu_data["gyr_x"], "r.-", label="Gyr - x")
  plt.plot(imu_time, imu_data["gyr_y"], "g.-", label="Gyr - y")
  plt.plot(imu_time, imu_data["gyr_z"], "b.-", label="Gyr - z")
  plt.subplot(212)
  plt.plot(gyr_time[10:], gyr_data["gyr_x"][10:], "r.-", label="Gyr - x")
  plt.plot(gyr_time[10:], gyr_data["gyr_y"][10:], "g.-", label="Gyr - y")
  plt.plot(gyr_time[10:], gyr_data["gyr_z"][10:], "b.-", label="Gyr - z")

  # Show
  plt.show()
