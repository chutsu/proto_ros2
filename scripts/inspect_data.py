#!/usr/bin/env python3
import pandas
import numpy as np
import matplotlib.pylab as plt

import proto
from proto import KalmanFilter

# cam0_csv = "/data/gimbal_experiments/exp-circle-0/cam0/data.csv"
# cam0_ts = np.array(pandas.read_csv(cam0_csv)["ts"])
# cam0_rates = 1.0 / (np.diff(cam0_ts) * 1e-9)
# N = cam0_rates.shape[0]

# plt.boxplot(cam0_rates)
# plt.show()

mocap_csv = "/data/gimbal_experiments/exp-240207/exp-circle-0/mocap.csv"
mocap_data = pandas.read_csv(mocap_csv)
mocap_ts = np.array(mocap_data["ts"]).astype(int)
mocap_time = (mocap_ts - mocap_ts[0]) * 1e-9
mocap_pos = np.array(mocap_data[["rx", "ry", "rz"]])

# Setup Kalman Filter
dt = 1.0 / 60.0
# -- Initial state x0
# x0 = [rx, ry, rz, vx, vy, vz, ax, ay, az]
x0 = np.zeros(9)
# -- Transition Matrix
F = np.eye(9)
F[0:3, 3:6] = np.eye(3) * dt
F[0:3, 6:9] = np.eye(3) * dt**2
F[3:6, 6:9] = np.eye(3) * dt
# -- Measurement Matrix
# H = np.block([np.eye(6), np.zeros((6, 3))])
H = np.block([np.eye(3), np.zeros((3, 6))])
# -- Input Matrix
B = np.array([0])
# -- Process Noise Matrix
Q = np.eye(9)
Q[0:3, 0:3] = 0.01 * np.eye(3)
Q[3:6, 3:6] = 0.00001**2 * np.eye(3)
Q[6:9, 6:9] = 0.00001**2 * np.eye(3)
# -- Measurement Noise Matrix
R = 0.1**2 * np.eye(3)
# R = np.eye(6)
# R[0:3, 0:3] = 0.1 * np.eye(3)
# R[3:6, 3:6] = 1.0 * np.eye(3)
# -- Kalman Filter
kf_kwargs = {"x0": x0, "F": F, "H": H, "B": B, "Q": Q, "R": R}
kf = KalmanFilter(**kf_kwargs)

# Filter measurements
est_ts = []
est_pos = []
est_vel = []

prev_ts = None
prev_pos = None
for ts, (rx, ry, rz) in zip(mocap_ts, mocap_pos):
  kf.predict()

  if prev_pos is None:
    prev_ts = ts
    prev_pos = [rx, ry, rz]
    continue

  dt = (ts - prev_ts) * 1e-9
  vx = (rx - prev_pos[0]) / dt
  vy = (ry - prev_pos[1]) / dt
  vz = (rz - prev_pos[2]) / dt

  # state = kf.update([rx, ry, rz, vx, vy, vz])
  # prev_ts = ts
  # prev_pos = [rx, ry, rz]

  state = kf.update([rx, ry, rz])

  est_ts.append(ts)
  est_pos.append([state[0], state[1], state[2]])
  est_vel.append([state[3], state[4], state[5]])

est_ts = np.array(est_ts)
est_time = (est_ts - est_ts[0]) * 1e-9
est_pos = np.array(est_pos)
est_vel = np.array(est_vel)


# Plot
plt.figure()

# -- Plot displacement
plt.subplot(321)
plt.plot(mocap_time, mocap_pos[:, 0], "k-", label="Mocap")
plt.plot(est_time, est_pos[:, 0], "r-", label="Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Displacement [m]")

plt.subplot(323)
plt.plot(mocap_time, mocap_pos[:, 1], "k-", label="Mocap")
plt.plot(est_time, est_pos[:, 1], "g-", label="Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Displacement [m]")

plt.subplot(325)
plt.plot(mocap_time, mocap_pos[:, 2], "k-", label="Mocap")
plt.plot(est_time, est_pos[:, 2], "b-", label="Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Displacement [m]")

# -- Plot velocity
mocap_vx = np.diff(mocap_pos[:, 0]) / np.diff(mocap_time)
mocap_vy = np.diff(mocap_pos[:, 1]) / np.diff(mocap_time)
mocap_vz = np.diff(mocap_pos[:, 2]) / np.diff(mocap_time)

window_size = 200
kernel = np.ones(window_size) / window_size
mocap_vx = np.convolve(mocap_vx, kernel, mode='valid')
mocap_vy = np.convolve(mocap_vy, kernel, mode='valid')
mocap_vz = np.convolve(mocap_vz, kernel, mode='valid')

plt.subplot(322)
plt.plot(mocap_time[window_size:], mocap_vx, "k-", label="Mocap")
plt.plot(est_time, est_vel[:, 0], "r-", label="Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [ms^-1]")

plt.subplot(324)
plt.plot(mocap_time[window_size:], mocap_vy, "k-", label="Mocap")
plt.plot(est_time, est_vel[:, 1], "g-", label="Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [ms^-1]")

plt.subplot(326)
plt.plot(mocap_time[window_size:], mocap_vz, "k-", label="Mocap")
plt.plot(est_time, est_vel[:, 2], "b-", label="Kalman Filter")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [ms^-1]")

# -- Show
plt.show()
