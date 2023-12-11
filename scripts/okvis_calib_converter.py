#!/usr/bin/env python3
import yaml
import numpy as np



def load_yaml(file_path):
  data = open(file_path, "r").read()
  return yaml.safe_load(data)


if __name__ == "__main__":
  calib_path = "/home/chutsu/calib_camimu/calib-results.yaml"
  calib = load_yaml(calib_path)

  cam0_fx, cam0_fy, cam0_cx, cam0_cy = calib["cam0"]["proj_params"]
  cam0_D = calib["cam0"]["dist_params"]
  cam1_fx, cam1_fy, cam1_cx, cam1_cy = calib["cam1"]["proj_params"]
  cam1_D = calib["cam1"]["dist_params"]
  T_C0C1 = np.array(calib["T_cam0_cam1"]["data"]).reshape((4, 4))
  T_SC0 = np.array(calib["T_imu0_cam0"]["data"]).reshape((4, 4))
  T_SC1 = T_SC0 @ T_C0C1

  config_template = f"""\
%YAML:1.0
# extrinsics and intrinsics per camera
cameras:
     - T_SC:
        [{T_SC0[0, 0]}, {T_SC0[0, 1]}, {T_SC0[0, 2]}, {T_SC0[0, 3]},
         {T_SC0[1, 0]}, {T_SC0[1, 1]}, {T_SC0[1, 2]}, {T_SC0[1, 3]},
         {T_SC0[2, 0]}, {T_SC0[2, 1]}, {T_SC0[2, 2]}, {T_SC0[2, 3]},
         {T_SC0[3, 0]}, {T_SC0[3, 1]}, {T_SC0[3, 2]}, {T_SC0[3, 3]}],
        image_dimension: [640, 480],
        distortion_coefficients: {cam0_D},
        distortion_type: radialtangential,
        focal_length: [{cam0_fx}, {cam0_fy}],
        principal_point: [{cam0_cx}, {cam0_cy}]

     - T_SC:
        [{T_SC1[0, 0]}, {T_SC1[0, 1]}, {T_SC1[0, 2]}, {T_SC1[0, 3]},
         {T_SC1[1, 0]}, {T_SC1[1, 1]}, {T_SC1[1, 2]}, {T_SC1[1, 3]},
         {T_SC1[2, 0]}, {T_SC1[2, 1]}, {T_SC1[2, 2]}, {T_SC1[2, 3]},
         {T_SC1[3, 0]}, {T_SC1[3, 1]}, {T_SC1[3, 2]}, {T_SC1[3, 3]}],
        image_dimension: [640, 480],
        distortion_coefficients: {cam1_D},
        distortion_type: radialtangential,
        focal_length: [{cam1_fx}, {cam1_fy}],
        principal_point: [{cam1_cx}, {cam1_cy}]

# additional camera parameters
camera_parameters:
    timestamp_tolerance: 0.005 # [s] stereo frame out-of-sync tolerance
    sync_cameras: [0, 1]
    image_delay: -0.00780146 # [s] timestamp_camera_correct = timestamp_camera - image_delay
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
    display_matches: true  # displays debug video and matches. May be slow.
    display_overhead: false # debug overhead image. Is slow.
    publish_imu_propagated_state: false # Should the state that is propagated with IMU messages be published or just the optimised ones?
    imu_propagated_state_publishing_rate: 40.0
"""
  print(config_template)
