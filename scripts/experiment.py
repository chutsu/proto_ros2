#!/usr/bin/env python3
import os
from os.path import join
import sys
import glob
import yaml

import proto
import pandas
import numpy as np
import matplotlib.pylab as plt


def load_yaml(file_path):
  """ Load YAML """
  data = open(file_path, "r").read()
  return yaml.safe_load(data)


def form_transform(pose):
  """ Form 4x4 Transformation Matrix from pose vector """
  x, y, z = pose[0:3]
  qw, qx, qy, qz = pose[3:7]
  r = np.array([x, y, z])
  C = proto.quat2rot(np.array([qw, qx, qy, qz]))
  return proto.tf(C, r)


def gimbal_joint_transform(theta):
  """ Form 4x4 Transformation Matrix from joint angle """
  r = np.array([0.0, 0.0, 0.0])
  C = proto.rotz(theta)
  return proto.tf(C, r)


def load_pose_data(filename, remove_bounds=False):
  """
  Reads a trajectory from a text file.

  File format:
  The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp
  (to be matched) and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D
  orientation) associated to this timestamp.

  Input:
  filename -- File name

  Output:
  dict -- dictionary of (stamp,data) tuples

  """
  file = open(filename)
  data = file.read()
  lines = data.replace(",", " ").replace("\t", " ").split("\n")
  if remove_bounds:
    lines = lines[100:-100]
  data = [[v.strip()
           for v in line.split(" ")
           if v.strip() != ""]
          for line in lines
          if len(line) > 0 and line[0] != "#"]
  # data = [(float(l[0]), l[1:]) for l in data if len(l) > 1]
  # return dict(data)

  file_data = {}
  for data_line in data[1:]:
    ts = int(data_line[0])
    values = data_line[1:]
    file_data[ts] = [float(v) for v in data_line[1:]]

  return file_data


def associate(first_list, second_list, t_offset=0.0, max_difference=0.02 * 1e9):
  """
    Associate two dictionaries of (stamp,data). As the time stamps never match
    exactly, we aim to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    t_offset -- time offset between both dictionaries (e.g., to model the delay
                between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

    """
  # first_keys = first_list.keys()
  # second_keys = second_list.keys()
  first_keys = list(first_list)
  second_keys = list(second_list)
  potential_matches = [(abs(a - (b + t_offset)), a, b)
                       for a in first_keys
                       for b in second_keys
                       if abs(a - (b + t_offset)) < max_difference]
  potential_matches.sort()
  matches = []
  for _, a, b in potential_matches:
    if a in first_keys and b in second_keys:
      first_keys.remove(a)
      second_keys.remove(b)
      matches.append((a, b))

  matches.sort()
  return matches


def align(model, data):
  """Align two trajectories using the method of Horn (closed-form).

    Input:

        model -- first trajectory (3xn)
        data -- second trajectory (3xn)

    Output:

        rot -- rotation matrix (3x3)
        trans -- translation vector (3x1)
        trans_error -- translational error per point (1xn)

    """
  np.set_printoptions(precision=3, suppress=True)
  model_zerocentered = model - model.mean(1)
  data_zerocentered = data - data.mean(1)

  W = np.zeros((3, 3))
  for column in range(model.shape[1]):
    W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
  U, _, Vh = np.linalg.linalg.svd(W.transpose())

  S = np.matrix(np.identity(3))
  if np.linalg.det(U) * np.linalg.det(Vh) < 0:
    S[2, 2] = -1
  rot = U * S * Vh

  rotmodel = rot * model_zerocentered
  dots = 0.0
  norms = 0.0

  for column in range(data_zerocentered.shape[1]):
    dots += np.dot(data_zerocentered[:, column].transpose(), rotmodel[:,
                                                                      column])
    normi = np.linalg.norm(model_zerocentered[:, column])
    norms += normi * normi

  s = float(dots / norms)

  transGT = data.mean(1) - s * rot * model.mean(1)
  trans = data.mean(1) - rot * model.mean(1)

  model_alignedGT = s * rot * model + transGT
  model_aligned = rot * model + trans

  alignment_errorGT = model_alignedGT - data
  alignment_error = model_aligned - data

  trans_errorGT = np.sqrt(
      np.sum(np.multiply(alignment_errorGT, alignment_errorGT), 0)).A[0]
  trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error),
                               0)).A[0]

  return rot, transGT, trans_errorGT, trans, trans_error, s, model_aligned
  # return model_aligned


def compensate_gimbal(calib_path, okvis_vio_file, joints_file):
  # Load gimbal calib file
  calib = load_yaml(f"{calib_path}/calib_gimbal/calib_gimbal-results.yaml")
  T_C0M0 = form_transform(calib["gimbal_ext"])
  T_L0M1 = form_transform(calib["link0_ext"])
  T_L1M2 = form_transform(calib["link1_ext"])
  T_L2C2 = form_transform(calib["end_ext"])
  T_C0C0 = form_transform(calib["cam0_ext"])
  T_C0C1 = form_transform(calib["cam1_ext"])
  T_C2C2 = form_transform(calib["cam2_ext"])
  T_C2C3 = form_transform(calib["cam3_ext"])

  # Load camera-imu calib file
  camimu_calib = load_yaml(f"{calib_path}/calib_camimu-rs0/calib_vi-results.yaml")
  T_S0C0 = np.array(camimu_calib["T_imu0_cam0"]["data"]).reshape((4, 4))
  camimu_calib = load_yaml(f"{calib_path}/calib_camimu-rs1/calib_vi-results.yaml")
  T_S1C2 = np.array(camimu_calib["T_imu0_cam0"]["data"]).reshape((4, 4))

  # Load joints data
  joints = pandas.read_csv(joints_file, header=1).to_numpy()
  joints_ts = joints[:, 0]
  joints_time = (joints_ts - joints_ts[0]) * 1e-9
  # joints0 = joints[:, 1] 0
  # joints1 = joints[:, 2] 1
  # joints2 = joints[:, 3] 2
  joints0 = np.deg2rad(joints[:, 3])
  joints1 = np.deg2rad(joints[:, 1])
  joints2 = np.deg2rad(joints[:, 2])

  # Load OKVIS VIO data
  okvis = pandas.read_csv(okvis_vio_file, header=1).to_numpy()
  okvis_ts = okvis[:, 0]
  okvis_time = ((okvis_ts - okvis_ts[0]) * 1e-9).astype(float)
  okvis_poses = okvis[:, 1:8]

  # Interpolate joints data
  theta0 = np.interp(okvis_time, joints_time, joints0)
  theta1 = np.interp(okvis_time, joints_time, joints1)
  theta2 = np.interp(okvis_time, joints_time, joints2)

  # Visualize interpolation
  # plt.subplot(311)
  # plt.plot(okvis_time, np.rad2deg(theta0), "b-", label="Interpolated")
  # plt.plot(joints_time, np.rad2deg(joints0), "r-", label="Raw")
  # plt.xlabel("Time [s]")
  # plt.ylabel("Joint Angle [deg]")
  # plt.legend(loc=0)
  # plt.subplot(312)
  # plt.plot(okvis_time, np.rad2deg(theta1), "b-", label="Interpolated")
  # plt.plot(joints_time, np.rad2deg(joints1), "r-", label="Raw")
  # plt.xlabel("Time [s]")
  # plt.ylabel("Joint Angle [deg]")
  # plt.legend(loc=0)
  # plt.subplot(313)
  # plt.plot(okvis_time, np.rad2deg(theta2), "b-", label="Interpolated")
  # plt.plot(joints_time, np.rad2deg(joints2), "r-", label="Raw")
  # plt.xlabel("Time [s]")
  # plt.ylabel("Joint Angle [deg]")
  # plt.legend(loc=0)
  # plt.show()

  # Compensate gimbal to base
  csv_file = open(okvis_vio_file.replace(".csv", "-compensated.csv"), "w")
  csv_file.write("timestamp,rx,ry,rz,qx,qy,qz,qw\n")
  for ts, pose, th0, th1, th2 in zip(okvis_ts, okvis_poses, theta0, theta1, theta2):
    rx, ry, rz, qx, qy, qz, qw = pose
    r_WS1 = np.array([rx, ry, rz])
    q_WS1 = np.array([qw, qx, qy, qz])
    T_WS1 = proto.tf(q_WS1, r_WS1)

    T_M0L0 = gimbal_joint_transform(th0)
    T_M1L1 = gimbal_joint_transform(th1)
    T_M2L2 = gimbal_joint_transform(th2)

    T_C0C2 = T_C0M0 @ T_M0L0 @ T_L0M1 @ T_M1L1 @ T_L1M2 @ T_M2L2 @ T_L2C2
    T_C2C0 = np.linalg.inv(T_C0C2)
    T_WS0 = T_WS1 @ T_S1C2 @ T_C2C0 @ np.linalg.inv(T_S0C0)

    r = proto.tf_trans(T_WS0)
    q = proto.tf_quat(T_WS0)
    csv_file.write(f"{ts},")
    csv_file.write(f"{r[0]},")
    csv_file.write(f"{r[1]},")
    csv_file.write(f"{r[2]},")
    csv_file.write(f"{q[1]},")
    csv_file.write(f"{q[2]},")
    csv_file.write(f"{q[3]},")
    csv_file.write(f"{q[0]}\n")
  csv_file.close()


def eval_traj(gnd_file, est_file, **kwargs):
  verbose = kwargs.get("verbose", False)

  # Read files and associate them
  gnd_data = load_pose_data(gnd_file, False)
  est_data = load_pose_data(est_file, False)
  matches = associate(gnd_data, est_data)
  if len(matches) < 2:
    sys.exit("Matches < 2! Did you choose the correct sequence?")

  # Extract matches between ground-truth and estimates and form matrices
  gnd_mat = []
  est_mat = []
  for ts_a, ts_b in matches:
    gnd_data_row = [float(value) for value in gnd_data[ts_a][0:3]]
    est_data_row = [float(value) for value in est_data[ts_b][0:3]]
    gnd_mat.append(gnd_data_row)
    est_mat.append(est_data_row)
  gnd_mat = np.matrix(gnd_mat).transpose()
  est_mat = np.matrix(est_mat).transpose()

  # Align both estimates and ground-truth
  rot, transGT, trans_errorGT, trans, trans_error, scale, model_aligned = align(
      est_mat, gnd_mat)
  gnd = np.array(gnd_mat)
  est = np.array(model_aligned)

  # Calculate errors
  metrics = {
      "ate": {
          "rmse": np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)),
          "mean": np.mean(trans_error),
          "median": np.median(trans_error),
          "std": np.std(trans_error),
          "min": np.min(trans_error),
          "max": np.max(trans_error)
      }
  }

  if verbose:
    print("compared_pose_pairs %d pairs" % (len(trans_error)))
    print("ATE.rmse %f m" % metrics["ate"]["rmse"])
    print("ATE.mean %f m" % metrics["ate"]["mean"])
    print("ATE.median %f m" % metrics["ate"]["median"])
    print("ATE.std %f m" % metrics["ate"]["std"])
    print("ATE.min %f m" % metrics["ate"]["min"])
    print("ATE.max %f m" % metrics["ate"]["max"])

  return metrics, gnd, est


def form_okvis_config(rs_calib, output_path):
  calib = load_yaml(rs_calib)

  cam0_fx, cam0_fy, cam0_cx, cam0_cy = calib["cam0"]["proj_params"]
  cam0_D = calib["cam0"]["dist_params"]
  cam1_fx, cam1_fy, cam1_cx, cam1_cy = calib["cam1"]["proj_params"]
  cam1_D = calib["cam1"]["dist_params"]
  T_C0C1 = np.array(calib["T_cam0_cam1"]["data"]).reshape((4, 4))
  T_SC0 = np.array(calib["T_imu0_cam0"]["data"]).reshape((4, 4))
  T_SC1 = T_SC0 @ T_C0C1
  time_delay = calib["time_delay"]

  config_template = f"""\
%YAML:1.0
# extrinsics and intrinsics per camera
cameras:
     - {{T_SC:[{T_SC0[0, 0]}, {T_SC0[0, 1]}, {T_SC0[0, 2]}, {T_SC0[0, 3]},
              {T_SC0[1, 0]}, {T_SC0[1, 1]}, {T_SC0[1, 2]}, {T_SC0[1, 3]},
              {T_SC0[2, 0]}, {T_SC0[2, 1]}, {T_SC0[2, 2]}, {T_SC0[2, 3]},
              {T_SC0[3, 0]}, {T_SC0[3, 1]}, {T_SC0[3, 2]}, {T_SC0[3, 3]}],
        image_dimension: [640, 480],
        distortion_coefficients: {cam0_D},
        distortion_type: radialtangential,
        focal_length: [{cam0_fx}, {cam0_fy}],
        principal_point: [{cam0_cx}, {cam0_cy}]}}
     - {{T_SC: [{T_SC1[0, 0]}, {T_SC1[0, 1]}, {T_SC1[0, 2]}, {T_SC1[0, 3]},
               {T_SC1[1, 0]}, {T_SC1[1, 1]}, {T_SC1[1, 2]}, {T_SC1[1, 3]},
               {T_SC1[2, 0]}, {T_SC1[2, 1]}, {T_SC1[2, 2]}, {T_SC1[2, 3]},
               {T_SC1[3, 0]}, {T_SC1[3, 1]}, {T_SC1[3, 2]}, {T_SC1[3, 3]}],
        image_dimension: [640, 480],
        distortion_coefficients: {cam1_D},
        distortion_type: radialtangential,
        focal_length: [{cam1_fx}, {cam1_fy}],
        principal_point: [{cam1_cx}, {cam1_cy}]}}

# additional camera parameters
camera_parameters:
    timestamp_tolerance: 0.005 # [s] stereo frame out-of-sync tolerance
    sync_cameras: [0, 1]
    image_delay: 0.0 # [s] timestamp_camera_correct = timestamp_camera - image_delay
    online_calibration: # some parameters to set the online
        do_extrinsics: false # Do we online-calibrate extrinsics?
        do_extrinsics_final_ba: false # Do we online-calibrate extrinsics?
        sigma_r: 0.001 # T_SCi position prior stdev [m]
        sigma_alpha: 0.01 # T_SCi orientation prior stdev [rad]

# the IMU sensor model
imu_parameters:
    used: true # enable IMU
    a_max: 160.0 # acceleration saturation [m/s^2]
    g_max: 10.0 # gyro saturation [rad/s]
    sigma_g_c: 0.00278 # gyro noise density [rad/s/sqrt(Hz)]
    sigma_bg: 0.03 # gyro bias prior [rad/s]
    sigma_a_c: 0.0252 # accelerometer noise density [m/s^2/sqrt(Hz)]
    sigma_ba: 0.1 # accelerometer bias prior [m/s^2]
    sigma_gw_c: 0.0008 # gyro drift noise density [rad/s^s/sqrt(Hz)]
    sigma_aw_c: 0.04 # accelerometer drift noise density [m/s^2/sqrt(Hz)]
    a0: [ 0.0, 0.0, 0.0 ] # Initial accelerometer bias [m/s^2]
    g0: [ 0.0, 0.0, 0.0 ] # Initial gyro bias [rad/s]
    g: 9.81007 # Earth's acceleration due to gravity [m/s^2]
    T_BS:
        [ 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0 ]

# frontend: detection etc.
frontend_parameters:
    detection_threshold: 30.0 # detection threshold. By default the uniformity radius in pixels
    absolute_threshold: 5.0 # absolute Harris corner threshold (noise floor)
    matching_threshold: 60.0
    octaves: 0 # number of octaves for detection. 0 means single-scale at highest resolution
    max_num_keypoints: 400 # restrict to a maximum of this many keypoints per image (strongest ones)
    keyframe_overlap: 0.55 # minimum field-of-view overlap
    use_cnn: false
    parallelise_detection: true
    num_matching_threads: 8

# estimator parameters
estimator_parameters:
    num_keyframes: 5 # number of keyframes in optimisation window
    num_loop_closure_frames: 5 # number of loop closure frames in optimisation window
    num_imu_frames: 3 # number of frames linked by most recent nonlinear IMU error terms
    do_loop_closures: true # whether to do VI-SLAM or VIO
    do_final_ba: true # Whether to run a full final BA
    enforce_realtime: true # whether to limit the time budget for optimisation
    realtime_min_iterations: 3 # minimum number of iterations always performed
    realtime_max_iterations: 10 # never do more than these, even if not converged
    realtime_time_limit: 0.04 # time budget for realtime optimisation [s]
    realtime_num_threads: 2 # number of threads for the realtime optimisation
    full_graph_iterations: 15 # don't do more than these for the full (background) optimisation
    full_graph_num_threads: 2 # number of threads for the full (background) optimisation

# some options for how and what to output
output_parameters:
    display_matches: false # displays debug video and matches. May be slow.
    display_overhead: false # debug overhead image. Is slow.
    publish_imu_propagated_state: false # Should the state that is propagated with IMU messages be published or just the optimised ones?
    imu_propagated_state_publishing_rate: 40.0
"""
  config = open(output_path, "w")
  config.write(config_template)
  config.write("\n")
  config.close()

def run_okvis(config_file, data_dir):
  cmd = []
  cmd.append("cd ~/colcon_ws/build/okvis")
  cmd.append(f"./okvis_app_synchronous {config_file} {data_dir}")
  cmd = " && ".join(cmd)
  os.system(cmd)


def eval_dataset(calib_path, seq_dir):
  run0_path = "/tmp/run0"
  run1_path = "/tmp/run1"

  # Write images data.csv
  for idx in range(4):
    cam_idx = f"cam{idx}"
    csv_file = join(seq_dir, cam_idx, "data.csv")
    # if os.path.exists(csv_file):
    #   continue

    csv = open(csv_file, "w")
    csv.write("ts, filename\n")
    timestamps = []
    for img_path in sorted(glob.glob(join(seq_dir, cam_idx, "data", "*.png")))[10:]:
      fname = os.path.basename(img_path)
      ts = fname.split(".")[0]
      timestamps.append(float(ts) * 1e-9)
      csv.write(f"{ts},{fname}\n")
    csv.close()

    timestamps = np.array(timestamps)
    np.diff(timestamps)

  # Run OKVIS on cam0 cam1 imu0
  okvis_config = "/tmp/okvis_config.yaml"
  rs0_calib = f"{calib_path}/calib_camimu-rs0/calib_vi-results.yaml"
  form_okvis_config(rs0_calib, okvis_config)
  okvis_config = "/data/gimbal_experiments/realsense_d435i.yaml"

  os.system(f"rm -rf {run0_path}")
  os.system(f"mkdir -p {run0_path}")
  os.system(f"cp -r {seq_dir}/cam0 {run0_path}")
  os.system(f"cp -r {seq_dir}/cam1 {run0_path}")
  os.system(f"cp -r {seq_dir}/imu0 {run0_path}")
  run_okvis(okvis_config, run0_path)
  os.system(f"mv {run0_path}/okvis2-vio_trajectory.csv {seq_dir}/rs0-okvis_vio.csv")

  # Run OKVIS on cam2 cam3 imu1
  okvis_config = "/tmp/okvis_config.yaml"
  rs1_calib = f"{calib_path}/calib_camimu-rs1/calib_vi-results.yaml"
  form_okvis_config(rs1_calib, okvis_config)
  okvis_config = "/data/gimbal_experiments/realsense_d435i.yaml"

  os.system(f"rm -rf {run1_path}")
  os.system(f"mkdir -p {run1_path}")
  os.system(f"cp -r {seq_dir}/cam2 {run1_path}")
  os.system(f"cp -r {seq_dir}/cam3 {run1_path}")
  os.system(f"cp -r {seq_dir}/imu1 {run1_path}")
  os.system(f"mv {run1_path}/cam2 {run1_path}/cam0")
  os.system(f"mv {run1_path}/cam3 {run1_path}/cam1")
  os.system(f"mv {run1_path}/imu1 {run1_path}/imu0")
  run_okvis(okvis_config, run1_path)
  os.system(f"mv {run1_path}/okvis2-vio_trajectory.csv {seq_dir}/rs1-okvis_vio.csv")

def plot_3d(gnd, est0, est1, **kwargs):
  plt.figure()
  ax = plt.axes(projection='3d')

  ax.plot(gnd[0, :], gnd[1, :], gnd[2, :], "k--", label="Ground Truth")
  # ax.plot(est0[0, :], est0[1, :], est0[2, :], "b-", label="Static Camera")
  ax.plot(est1[0, :], est1[1, :], est1[2, :], "r-", label="Gimbal Camera")

  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_zlabel("z [m]")
  proto.plot_set_axes_equal(ax)
  plt.show()


def plot_results(gnd, est0, est1, save_path, **kwargs):
  figsize = (1100, 600)
  dpi = 90
  plt.figure(figsize=(figsize[0] / dpi, figsize[1] / dpi))
  # Plot Ground-Truth and Static Camera
  plt.subplot(121)
  plt.plot(gnd[0, :], gnd[1, :], "k--", label="Ground Truth")
  plt.plot(est0[0, :], est0[1, :], "r-", label="Static Camera")

  min_x = np.min(gnd[0, :])
  min_y = np.min(gnd[1, :])
  max_y = np.max(gnd[1, :])
  step_y = (max_y - min_y) * 0.05
  plt.text(min_x, min_y - step_y * 0, f"RMSE:    {metrics0['ate']['rmse']:.4f}m")
  plt.text(min_x, min_y - step_y * 1, f"Mean:    {metrics0['ate']['mean']:.4f}m")
  plt.text(min_x, min_y - step_y * 2, f"Median: {metrics0['ate']['median']:.4f}m")
  plt.text(min_x, min_y - step_y * 3, f"Stddev:  {metrics0['ate']['std']:.4f}m")
  plt.text(min_x, min_y - step_y * 4, f"Min:       {metrics0['ate']['min']:.4f}m")
  plt.text(min_x, min_y - step_y * 5, f"Max:      {metrics0['ate']['max']:.4f}m")

  plt.legend(loc=0)
  plt.axis("equal")
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")

  # Plot Ground-Truth and Gimbal Camera
  plt.subplot(122)
  plt.plot(gnd[0, :], gnd[1, :], "k--", label="Ground Truth")
  plt.plot(est1[0, :], est1[1, :], "b-", label="Dynamic Camera")

  min_x = np.min(gnd[0, :])
  min_y = np.min(gnd[1, :])
  max_y = np.max(gnd[1, :])
  step_y = (max_y - min_y) * 0.05
  plt.text(min_x, min_y - step_y * 0, f"RMSE:    {metrics1['ate']['rmse']:.4f}m")
  plt.text(min_x, min_y - step_y * 1, f"Mean:    {metrics1['ate']['mean']:.4f}m")
  plt.text(min_x, min_y - step_y * 2, f"Median: {metrics1['ate']['median']:.4f}m")
  plt.text(min_x, min_y - step_y * 3, f"Stddev:  {metrics1['ate']['std']:.4f}m")
  plt.text(min_x, min_y - step_y * 4, f"Min:       {metrics1['ate']['min']:.4f}m")
  plt.text(min_x, min_y - step_y * 5, f"Max:      {metrics1['ate']['max']:.4f}m")

  plt.legend(loc=0)
  plt.axis("equal")
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")

  # plt.tight_layout()
  plt.subplots_adjust(wspace=0.3, hspace=0)
  if kwargs.get("savefig", False):
    plt.savefig(save_path)
  if kwargs.get("show", False):
    plt.show()


if __name__ == "__main__":
  calib_path = "/data/gimbal_experiments/exp-240207/calib"
  # seq_path = "/data/gimbal_experiments/exp-240207/exp-circle-0"
  # seq_path = "/data/gimbal_experiments/exp-240207/exp-circle-1"
  # seq_path = "/data/gimbal_experiments/exp-240207/exp-figure8-0"
  # seq_path = "/data/gimbal_experiments/exp-240207/exp-figure8-1"
  # seq_path = "/data/gimbal_experiments/exp-240207/exp-noisy-0"
  # seq_path = "/data/gimbal_experiments/exp-240207/exp-noisy-1"

  # calib_path = "/data/gimbal_experiments/exp-240220/calib-240225"
  # seq_path = "/data/gimbal_experiments/exp-240220/exp-circle-0"
  # seq_path = "/data/gimbal_experiments/exp-240220/exp-circle-1"
  # seq_path = "/data/gimbal_experiments/exp-240220/exp-figure8-0"
  # seq_path = "/data/gimbal_experiments/exp-240220/exp-figure8-1"
  # seq_path = "/data/gimbal_experiments/exp-240220/exp-noisy-0"
  # seq_path = "/data/gimbal_experiments/exp-240220/exp-noisy-1"
  seq_path = "/data/gimbal_experiments/exp-240220/exp-noisy-2"

  # Evaluate dataset
  # eval_dataset(calib_path, seq_path)

  # Evaluate mocap vs vio estimate
  mocap_file = join(seq_path, "mocap.csv")
  rs0_file = join(seq_path, "rs0-okvis_vio.csv")
  rs1_file = join(seq_path, "rs1-okvis_vio.csv")

  joints_file = join(seq_path, "joints.csv")
  compensate_gimbal(calib_path, rs1_file, joints_file)
  rs1_file = join(seq_path, "rs1-okvis_vio-compensated.csv")

  metrics0, gnd, est0 = eval_traj(mocap_file, rs0_file, verbose=True)
  metrics1, gnd, est1 = eval_traj(mocap_file, rs1_file, verbose=True)

  # Plot results
  seq_name = os.path.basename(seq_path)
  save_path = f"{seq_path}/../plot_odom-{seq_name}.png"
  kwargs = {"show": True, "savefig": True}
  plot_results(gnd, est0, est1, save_path, **kwargs)
  # plot_3d(gnd, est0, est1)
