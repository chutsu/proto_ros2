#!/usr/bin/env python3
import os
from os.path import join
import sys

import proto
import pandas
import numpy as np
import matplotlib.pylab as plt


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


def eval_traj(gnd_file, est_file, **kwargs):
  verbose = kwargs.get("verbose", False)
  plot = kwargs.get("plot", False)

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

  if plot:
    gnd = np.array(gnd_mat)
    est = np.array(model_aligned)
    plt.plot(gnd[0, :], gnd[1, :], "k--", label="Ground Truth")
    plt.plot(est[0, :], est[1, :], "r-", label="Estimate")
    plt.legend(loc=0)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

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

  return metrics


if __name__ == "__main__":
  data_dir = "/data/gimbal_experiments/exp-circle-0"

  # # Plot mocap vs vio estimate
  # mocap_file = join(data_dir, "mocap.csv")
  # vio_file = join(data_dir, "okvis_poses.csv")
  # eval_traj(mocap_file, vio_file, verbose=True, plot=True)
