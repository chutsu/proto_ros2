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

  Eigen::Vector<double, 7> end_ext;
  Eigen::Vector<double, 7> cam0_ext;
  Eigen::Vector<double, 7> cam1_ext;
  Eigen::Vector<double, 7> link0_ext;
  Eigen::Vector<double, 7> link1_ext;
  Eigen::Vector<double, 7> gimbal_ext;
  Eigen::Vector<double, 7> fiducial_pose;

  Eigen::Matrix4d T_WB = Eigen::MatrixXd::Identity(4, 4);
  Eigen::Matrix4d T_BM0;
  Eigen::Matrix4d T_L0M1;
  Eigen::Matrix4d T_L1M2;
  Eigen::Matrix4d T_L2E;
  Eigen::Matrix4d T_EC0;
  Eigen::Matrix4d T_EC1;
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
    parse_key_value(conf, "fiducial_pose", "pose", fiducial_pose.data());

    T_BM0 = transform(gimbal_ext.data());
    T_L0M1 = transform(link0_ext.data());
    T_L1M2 = transform(link1_ext.data());
    T_L2E = transform(end_ext.data());
    T_EC0 = transform(cam0_ext.data());
    T_EC1 = transform(cam1_ext.data());
    T_WF = transform(fiducial_pose.data());

    print_matrix("T_BM0", T_BM0, "  ");
    print_matrix("T_L0M1", T_L0M1, "  ");
    print_matrix("T_L1M2", T_L1M2, "  ");
    print_matrix("T_L2E", T_L2E, "  ");
    print_matrix("T_EC0", T_EC0, "  ");
    print_matrix("T_EC1", T_EC1, "  ");
    print_matrix("T_WF", T_WF, "  ");

    // Clean up
    fclose(conf);
  }
};

int main(int argc, char *argv[]) {
  // Setup
  CalibConfig calib_conf{"/home/chutsu/calib_gimbal2/results.yaml"};
  rs_d435i_t device;
  const double scale_factor = 0.5;

  std::mutex mtx;
  bool run = true;
  int64_t ts = 0;
  cv::Mat frame0;
  double target_angle0 = 0.0;
  double target_angle1 = 0.0;
  double target_angle2 = 0.0;
  double joint0 = 0.0;
  double joint1 = 0.0;
  double joint2 = 0.0;

  // Thread functions
  // -- RealSense thread
  auto realsense_thread = [&]() {
    device.image_callback = [&](const rs2::video_frame &ir0,
                                const rs2::video_frame &ir1) {
      const int width = ir0.get_width();
      const int height = ir0.get_height();
      const std::string encoding = "mono8";
      ts = vframe2ts(ir0, true);
      std::lock_guard<std::mutex> lock(mtx);
      frame0 = frame2cvmat(ir0, width, height, CV_8UC1);

      const int new_width = width * scale_factor;
      const int new_height = height * scale_factor;
      cv::resize(frame0, frame0, cv::Size(new_width, new_height));
    };

    device.start();
    while (run) {}
  };

  // -- SBGC Thread
  auto sbgc_thread = [&]() {
    sbgc_t sbgc;

    // Connect to gimbal
    if (sbgc_connect(&sbgc, SBGC_DEV) != 0) {
      printf("Failed to connect to SBGC!");
      run = false;
    }

    // Switch the gimbal on
    if (sbgc_on(&sbgc) != 0) {
      printf("Failed to turn on SBGC!");
      run = false;
    }

    // Loop
    while (run) {
      sbgc_set_angle(&sbgc, target_angle0, target_angle1, target_angle2);
      sbgc_update(&sbgc);

      joint0 = deg2rad(sbgc.encoder_angles[2]);
      joint1 = deg2rad(sbgc.encoder_angles[0]);
      joint2 = deg2rad(sbgc.encoder_angles[1]);

      usleep(10 * 1000);
    }

    sbgc_off(&sbgc);
  };

  // Inspect thread
  auto inspect_thread = [&]() {
    sleep(3);

    bool fiducial_estimated = false;
    Eigen::Matrix4d T_WF;

    int num_rows = 6;
    int num_cols = 6;
    double tsize = 0.038;
    double tspacing = 0.3;
    AprilTags::AprilGridDetector detector;
    aprilgrid_t *grid0 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

    while (run) {
      if (frame0.empty()) {
        continue;
      }

      // std::lock_guard<std::mutex> lock(mtx);
      detect_aprilgrid(detector, ts, frame0.clone(), grid0);
      auto viz0 = aprilgrid_draw(grid0, frame0);

      const Eigen::Matrix4d T_BM0 = calib_conf.T_BM0;
      const Eigen::Matrix4d T_M0L0 = gimbal_joint_transform(joint0);
      const Eigen::Matrix4d T_L0M1 = calib_conf.T_L0M1;
      const Eigen::Matrix4d T_M1L1 = gimbal_joint_transform(joint1);
      const Eigen::Matrix4d T_L1M2 = calib_conf.T_L1M2;
      const Eigen::Matrix4d T_M2L2 = gimbal_joint_transform(joint2);
      const Eigen::Matrix4d T_L2E = calib_conf.T_L2E;
      const Eigen::Matrix4d T_EC0 = calib_conf.T_EC0;
      const Eigen::Matrix4d T_WF = calib_conf.T_WF;

      // clang-format off
      const Eigen::Matrix4d T_WC0 = T_BM0 * T_M0L0 * T_L0M1 * T_M1L1 * T_L1M2 * T_M2L2 * T_L2E * T_EC0;
      const Eigen::Matrix4d T_C0F = T_WC0.inverse() * T_WF;
      // clang-format on

      int tag_ids[6 * 6 * 4] = {0};
      int corner_indices[6 * 6 * 4] = {0};
      real_t kps[6 * 6 * 4 * 2] = {0};
      real_t pts[6 * 6 * 4 * 3] = {0};
      aprilgrid_measurements(grid0, tag_ids, corner_indices, kps, pts);

      auto cam_params = calib_conf.cam_params[0];
      cam_params[0] = cam_params[0] * scale_factor;
      cam_params[1] = cam_params[1] * scale_factor;
      cam_params[2] = cam_params[2] * scale_factor;
      cam_params[3] = cam_params[3] * scale_factor;

      // if (fiducial_estimated == false) {
      //   std::vector<Eigen::Vector2d> keypoints;
      //   std::vector<Eigen::Vector3d> points;
      //   Eigen::Matrix4d T_CF;
      //   for (int i = 0; i < grid0->corners_detected; i++) {
      //     keypoints.emplace_back(kps[2 * i + 0] * scale_factor,
      //                            kps[2 * i + 1] * scale_factor);
      //     points.emplace_back(pts[3 * i + 0], pts[3 * i + 1], pts[3 * i + 2]);
      //   }

      //   printf("num keypoints: %ld\n", keypoints.size());
      //   printf("num points: %ld\n", points.size());

      //   if (keypoints.size() < 10) {
      //     return;
      //   }

      //   solvepnp(calib_conf.cam_res, cam_params, keypoints, points, T_CF);
      //   T_WF = T_WC0 * T_CF;

      //   std::cout << "T_WF:" << std::endl;
      //   std::cout << T_WF << std::endl;

      //   fiducial_estimated = true;

      // } else {

      for (int i = 0; i < grid0->corners_detected; i++) {
        const Eigen::Vector2d kp{kps[i * 2 + 0], kps[i * 2 + 1]};
        const Eigen::Vector3d p_FFi{pts[i * 3 + 0],
                                    pts[i * 3 + 1],
                                    pts[i * 3 + 2]};
        const Eigen::Vector4d hp_FFi = p_FFi.homogeneous();
        const Eigen::Vector3d p_C0 = (T_C0F * hp_FFi).head(3);
        Eigen::Vector2d z;
        pinhole_radtan4_project(calib_conf.cam_res, cam_params, p_C0, z);

        const int marker_size = 2;
        const cv::Scalar color{0, 255, 0};
        const cv::Point2f p(z.x(), z.y());
        cv::circle(viz0, p, marker_size, color, -1);
      }
      // }

      cv::imshow("Viz", viz0);
      if (cv::waitKey(1) == 'q') {
        run = false;
      }

      // Clear aprilgrid detection
      aprilgrid_clear(grid0);
    }

    // Clean up
    aprilgrid_free(grid0);
  };

  // Loop
  std::thread thread0(sbgc_thread);
  std::thread thread1(realsense_thread);
  std::thread thread2(inspect_thread);
  thread0.join();
  thread1.join();
  thread2.join();

  return 0;
}
