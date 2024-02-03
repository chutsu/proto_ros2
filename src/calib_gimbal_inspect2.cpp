#include <iostream>
#include <sys/stat.h>
#include <mutex>
#include <thread>

#define SBGC_IMPLEMENTATION
#define SBGC_DEV "/dev/ttyUSB0"
#include <sbgc.h>

#include "util.hpp"
#include "realsense.hpp"

/**
 * CalibConfig
 */
struct CalibConfig {
  int num_cams = 0;
  int num_links = 0;

  int cam_res[2] = {0};
  char proj_model[30] = {0};
  char dist_model[30] = {0};
  std::map<int, Eigen::Vector<double, 8>> cam_params;

  Eigen::Vector<double, 7> gimbal_ext;
  Eigen::Vector<double, 7> link0_ext;
  Eigen::Vector<double, 7> link1_ext;
  Eigen::Vector<double, 7> end_ext;
  Eigen::Vector<double, 7> cam0_ext;
  Eigen::Vector<double, 7> cam1_ext;
  Eigen::Vector<double, 7> cam2_ext;
  Eigen::Vector<double, 7> cam3_ext;

  Eigen::Matrix4d T_C0M0;
  Eigen::Matrix4d T_L0M1;
  Eigen::Matrix4d T_L1M2;
  Eigen::Matrix4d T_L2C2;
  Eigen::Matrix4d T_C0C0;
  Eigen::Matrix4d T_C0C1;
  Eigen::Matrix4d T_C2C2;
  Eigen::Matrix4d T_C2C3;
  Eigen::Matrix4d T_WF;

  CalibConfig(const std::string &conf_path, const bool format_v2 = true) {
    // Open config file
    FILE *conf = fopen(conf_path.c_str(), "r");
    if (conf == NULL) {
      FATAL("Failed to open [%s]!\n", conf_path.c_str());
    }

    // Parse general
    parse_key_value(conf, "num_cams", "int", &num_cams);
    parse_key_value(conf, "num_links", "int", &num_links);
    parse_skip_line(conf);

    // Parse camera parameters
    for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
      real_t p[4] = {0};
      real_t d[4] = {0};

      parse_skip_line(conf);
      parse_key_value(conf, "resolution", "vec2i", cam_res);
      parse_key_value(conf, "proj_model", "string", proj_model);
      parse_key_value(conf, "dist_model", "string", dist_model);
      parse_key_value(conf, "proj_params", "vec4d", p);
      parse_key_value(conf, "dist_params", "vec4d", d);
      parse_skip_line(conf);

      const Eigen::Vector4d K{p[0], p[1], p[2], p[3]};
      const Eigen::Vector4d D{d[0], d[1], d[2], d[3]};

      cam_params[cam_idx] << K, D;
    }

    // Parse extrinsics
    parse_key_value(conf, "gimbal_ext", "pose", gimbal_ext.data());
    parse_key_value(conf, "link0_ext", "pose", link0_ext.data());
    parse_key_value(conf, "link1_ext", "pose", link1_ext.data());
    parse_key_value(conf, "end_ext", "pose", end_ext.data());
    parse_key_value(conf, "cam0_ext", "pose", cam0_ext.data());
    parse_key_value(conf, "cam1_ext", "pose", cam1_ext.data());
    parse_key_value(conf, "cam2_ext", "pose", cam2_ext.data());
    parse_key_value(conf, "cam3_ext", "pose", cam3_ext.data());

    T_C0M0 = transform(gimbal_ext.data());
    T_L0M1 = transform(link0_ext.data());
    T_L1M2 = transform(link1_ext.data());
    T_L2C2 = transform(end_ext.data());
    T_C0C0 = transform(cam0_ext.data());
    T_C0C1 = transform(cam1_ext.data());
    T_C2C2 = transform(cam2_ext.data());
    T_C2C3 = transform(cam3_ext.data());

    print_matrix("T_C0M0", T_C0M0, "  ");
    print_matrix("T_L0M1", T_L0M1, "  ");
    print_matrix("T_L1M2", T_L1M2, "  ");
    print_matrix("T_L2C2", T_L2C2, "  ");
    print_matrix("T_C0C0", T_C0C0, "  ");
    print_matrix("T_C0C1", T_C0C1, "  ");
    print_matrix("T_C2C2", T_C2C2, "  ");
    print_matrix("T_C2C3", T_C2C3, "  ");

    // Clean up
    fclose(conf);
  }
};

int main(int argc, char *argv[]) {
  // Parse device index from command args
  if (argc < 2) {
    printf("calib_gimbal_inspect2 <calib_gimbal.yaml>\n");
    printf("Example: calib_gimbal_inspect2 /calib_gimbal/calib_gimbal.yaml\n");
    return -1;
  }

  // Setup
  const std::string save_dir{argv[1]};
  CalibConfig calib_conf{save_dir};
  rs_multi_d435i_t rs_devices;
  const std::string serial0 = "843112071984";
  const std::string serial1 = "943222072527";
  const auto rs_cfg0 = rs_devices.configs[serial0];
  const auto rs_cfg1 = rs_devices.configs[serial1];
  rs2::frame ir0_frame;
  rs2::frame ir1_frame;
  rs2::frame ir2_frame;
  rs2::frame ir3_frame;

  // Start pipelines
  rs_devices.pipelines[serial0].start(rs_cfg0, [&](const rs2::frame &frame) {
    // Stereo Module Callback
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      ir0_frame = fs.get_infrared_frame(1);
      ir1_frame = fs.get_infrared_frame(2);
    }
  });
  rs_devices.pipelines[serial1].start(rs_cfg1, [&](const rs2::frame &frame) {
    // Stereo Module Callback
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      ir2_frame = fs.get_infrared_frame(1);
      ir3_frame = fs.get_infrared_frame(2);
    }
  });

  // Thread functions
  double joint0 = 0.0;
  double joint1 = 0.0;
  double joint2 = 0.0;
  int fwidth = 640;
  int fheight = 480;

  int num_rows = 6;
  int num_cols = 6;
  double tsize = 0.038;
  double tspacing = 0.3;
  AprilTags::AprilGridDetector detector;
  aprilgrid_t *grid0 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

  // -- Realsense thread
  auto realsense_thread = [&]() {
    signal(SIGINT, realsense_signal_handler);
    sleep(5);

    const Eigen::Matrix4d T_C0M0 = calib_conf.T_C0M0;
    const Eigen::Matrix4d T_L0M1 = calib_conf.T_L0M1;
    const Eigen::Matrix4d T_L1M2 = calib_conf.T_L1M2;
    const Eigen::Matrix4d T_L2C2 = calib_conf.T_L2C2;

    int64_t last_ts = 0;
    while (realsense_keep_running) {
      const auto ts0 = vframe2ts(ir0_frame, true);
      const auto ts1 = vframe2ts(ir1_frame, true);
      const auto ts2 = vframe2ts(ir2_frame, true);
      const auto ts3 = vframe2ts(ir3_frame, true);
      const std::vector<uint64_t> tss = {ts0, ts1, ts2, ts3};
      const auto ts_max = *std::max_element(tss.begin(), tss.end());
      const auto ts_min = *std::min_element(tss.begin(), tss.end());

      if ((ts_max - ts_min) * 1e-9 < 0.01 && (ts0 - last_ts) * 1e-9 > 0.01) {
        auto frame0 = frame2cvmat(ir0_frame, fwidth, fheight, CV_8UC1);
        auto frame2 = frame2cvmat(ir2_frame, fwidth, fheight, CV_8UC1);

        cv::Mat frame2_viz;
        cv::cvtColor(frame2, frame2_viz, cv::COLOR_GRAY2BGR);

        // Detect AprilGrid
        detect_aprilgrid(detector, ts0, frame0.clone(), grid0);
        auto frame0_viz = aprilgrid_draw(grid0, frame0);

        // Get keypoint measurements and object points
        int tag_ids[6 * 6 * 4] = {0};
        int corner_indices[6 * 6 * 4] = {0};
        real_t kps[6 * 6 * 4 * 2] = {0};
        real_t pts[6 * 6 * 4 * 3] = {0};
        aprilgrid_measurements(grid0, tag_ids, corner_indices, kps, pts);

        std::vector<Eigen::Vector2d> keypoints;
        std::vector<Eigen::Vector3d> points;
        for (int i = 0; i < grid0->corners_detected; i++) {
          keypoints.emplace_back(kps[2 * i + 0], kps[2 * i + 1]);
          points.emplace_back(pts[3 * i + 0], pts[3 * i + 1], pts[3 * i + 2]);
        }
        if (keypoints.size() < 10) {
          continue;
        }

        // SolvePnp
        Eigen::Matrix4d T_C0F;
        auto cam0_params = calib_conf.cam_params[0];
        auto cam2_params = calib_conf.cam_params[0];
        solvepnp(calib_conf.cam_res, cam0_params, keypoints, points, T_C0F);

        // clang-format off
        const auto T_M0L0 = gimbal_joint_transform(joint0);
        const auto T_M1L1 = gimbal_joint_transform(joint1);
        const auto T_M2L2 = gimbal_joint_transform(joint2);
        const auto T_C0C2 = T_C0M0 * T_M0L0 * T_L0M1 * T_M1L1 * T_L1M2 * T_M2L2 * T_L2C2;
        const auto T_C2C0 = T_C0C2.inverse();
        // clang-format on

        // Draw
        for (int i = 0; i < grid0->corners_detected; i++) {
          const Eigen::Vector2d kp{kps[i * 2 + 0], kps[i * 2 + 1]};
          const Eigen::Vector3d p_FFi{pts[i * 3 + 0],
                                      pts[i * 3 + 1],
                                      pts[i * 3 + 2]};
          const Eigen::Vector4d hp_FFi = p_FFi.homogeneous();
          const Eigen::Vector3d p_C2 = (T_C2C0 * T_C0F * hp_FFi).head(3);
          Eigen::Vector2d z;
          pinhole_radtan4_project(calib_conf.cam_res, cam2_params, p_C2, z);

          const int marker_size = 2;
          const cv::Scalar color{0, 255, 0};
          const cv::Point2f p(z.x(), z.y());
          cv::circle(frame2_viz, p, marker_size, color, -1);
        }

        // Visualize
        cv::Mat viz;
        cv::vconcat(frame2_viz, frame0_viz, viz);
        cv::imshow("Viz", viz);
        if (cv::waitKey(1) == 'q') {
          printf("Stopping realsense thread...\n");
          realsense_keep_running = false;
        }

        // Update
        last_ts = ts0;
        aprilgrid_clear(grid0);
      }
    }
  };

  // -- SBGC Thread
  auto sbgc_thread = [&]() {
    sbgc_t sbgc;

    // Connect to gimbal
    if (sbgc_connect(&sbgc, SBGC_DEV) != 0) {
      FATAL("Failed to connect to SBGC!");
    }

    // Switch the gimbal on
    if (sbgc_on(&sbgc) != 0) {
      FATAL("Failed to turn on SBGC!");
    }

    // Loop
    while (realsense_keep_running) {
      // sbgc_set_angle(&sbgc, target_angle0, target_angle1, target_angle2);
      sbgc_update(&sbgc);
      joint0 = deg2rad(sbgc.encoder_angles[2]);
      joint1 = deg2rad(sbgc.encoder_angles[0]);
      joint2 = deg2rad(sbgc.encoder_angles[1]);
      usleep(10 * 1000);
    }
    printf("Stopping SBGC thread...\n");

    sbgc_off(&sbgc);
  };

  // Loop
  std::thread thread0(sbgc_thread);
  std::thread thread1(realsense_thread);
  thread0.join();
  thread1.join();

  // Clean up
  aprilgrid_free(grid0);

  return 0;
}
