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
 * Convert Degrees to Radians.
 */
double deg2rad(const double d) {
  return d * (M_PI / 180.0);
}

/**
 * Form rotation matrix around z axis
 */
template <typename T>
Eigen::Matrix<T, 3, 3> ROTZ(const T theta) {
  Eigen::Matrix<T, 3, 3> C;

  C(0, 0) = cos(theta);
  C(0, 1) = -sin(theta);
  C(0, 2) = T(0.0);

  C(1, 0) = sin(theta);
  C(1, 1) = cos(theta);
  C(1, 2) = T(0.0);

  C(2, 0) = T(0.0);
  C(2, 1) = T(0.0);
  C(2, 2) = T(1.0);

  return C;
}

/**
 * Quaternion to rotation matrix.
 */
template <typename T>
Eigen::Matrix<T, 3, 3> quat2rot(const Eigen::Quaternion<T> &q) {
  const T qw = q.w();
  const T qx = q.x();
  const T qy = q.y();
  const T qz = q.z();

  const T qx2 = qx * qx;
  const T qy2 = qy * qy;
  const T qz2 = qz * qz;
  const T qw2 = qw * qw;

  // Homogeneous form
  Eigen::Matrix<T, 3, 3> C;
  // -- 1st row
  C(0, 0) = qw2 + qx2 - qy2 - qz2;
  C(0, 1) = T(2.0) * (qx * qy - qw * qz);
  C(0, 2) = T(2.0) * (qx * qz + qw * qy);
  // -- 2nd row
  C(1, 0) = T(2.0) * (qx * qy + qw * qz);
  C(1, 1) = qw2 - qx2 + qy2 - qz2;
  C(1, 2) = T(2.0) * (qy * qz - qw * qx);
  // -- 3rd row
  C(2, 0) = T(2.0) * (qx * qz - qw * qy);
  C(2, 1) = T(2.0) * (qy * qz + qw * qx);
  C(2, 2) = qw2 - qx2 - qy2 + qz2;

  return C;
}

/**
 * Form transformation matrix.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> transform(const T *params) {
  const Eigen::Quaternion<T> q{params[3], params[4], params[5], params[6]};
  const Eigen::Vector<T, 3> r{params[0], params[1], params[2]};

  Eigen::Matrix<T, 4, 4> transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = quat2rot(q);
  transform.block(0, 3, 3, 1) = r;

  return transform;
}

/**
 * Form transformation matrix.
 */
Eigen::VectorXd transform_vector(const Eigen::Matrix4d &T) {
  const Eigen::Vector3d r = T.block<3, 1>(0, 3);
  const Eigen::Quaterniond q{T.block<3, 3>(0, 0)};
  Eigen::VectorXd vec;
  vec.resize(7);
  vec << r.x(), r.y(), r.z(), q.w(), q.x(), q.y(), q.z();
  return vec;
}

/**
 * Gimbal joint angle to transformation matrix.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> gimbal_joint_transform(const T angle) {
  Eigen::Matrix<T, 4, 4> transform;
  transform.setIdentity();
  transform.block(0, 0, 3, 3) = ROTZ(angle);
  return transform;
}

/**
 * Project point
 */
Eigen::Vector2d project_point(const Eigen::Vector3d &p_C) {
  return Eigen::Vector2d{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
}

/**
 * Radtan4 Distort
 */
Eigen::Vector2d radtan4_distort(const Eigen::Vector4d &dist_params,
                                const Eigen::Vector2d &p) {
  const real_t x = p.x();
  const real_t y = p.y();

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

  return Eigen::Vector2d{x_ddash, y_ddash};
}

/**
 * Radtan4 Point Jacobian
 */
Eigen::Matrix2d radtan4_point_jacobian(const Eigen::Vector4d &dist_params,
                                       const Eigen::Vector2d &p) {
  const real_t x = p(0);
  const real_t y = p(1);

  const real_t k1 = dist_params(0);
  const real_t k2 = dist_params(1);
  const real_t p1 = dist_params(2);
  const real_t p2 = dist_params(3);

  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Let p = [x; y] normalized point
  // Let p' be the distorted p
  // The jacobian of p' w.r.t. p (or dp'/dp) is:
  Eigen::Matrix2d J_point;
  J_point(0, 0) = 1.0 + k1 * r2 + k2 * r4;
  J_point(0, 0) += 2.0 * p1 * y + 6.0 * p2 * x;
  J_point(0, 0) += x * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(1, 0) = 2.0 * p1 * x + 2.0 * p2 * y;
  J_point(1, 0) += y * (2.0 * k1 * x + 4.0 * k2 * x * r2);
  J_point(0, 1) = J_point(1, 0);
  J_point(1, 1) = 1.0 + k1 * r2 + k2 * r4;
  J_point(1, 1) += 6.0 * p1 * y + 2.0 * p2 * x;
  J_point(1, 1) += y * (2.0 * k1 * y + 4.0 * k2 * y * r2);
  // Above is generated using sympy

  // const auto radtan = k1 * r2 + k2 * r2 * r2;
  // J_point(0, 0) = 1 + radtan + k1 * 2.0 * x2 + k2 * r2 * 4 * x2 +
  //                 2.0 * p1 * p.y() + 6 * p2 * p.x();
  // J_point(1, 0) = k1 * 2.0 * p.x() * p.y() + k2 * 4 * r2 * p.x() * p.y() +
  //                 p1 * 2.0 * p.x() + 2.0 * p2 * p.y();
  // J_point(0, 1) = J_point(1, 0);
  // J_point(1, 1) = 1 + radtan + k1 * 2.0 * y2 + k2 * r2 * 4 * y2 +
  //                 6 * p1 * p.y() + 2.0 * p2 * p.x();

  return J_point;
}

/**
 * Radtan4 Undistort
 */
Eigen::Vector2d radtan4_undistort(const Eigen::Vector4d &dist_params,
                                  const Eigen::Vector2d &p0) {
  int max_iter = 5;
  Eigen::Vector2d p = p0;

  for (int i = 0; i < max_iter; i++) {
    // Error
    const Eigen::Vector2d p_distorted = radtan4_distort(dist_params, p);
    const Eigen::Vector2d err = (p0 - p_distorted);

    // Jacobian
    const Eigen::Matrix2d J = radtan4_point_jacobian(dist_params, p);
    const Eigen::Vector2d dp =
        (J.transpose() * J).inverse() * J.transpose() * err;
    p = p + dp;

    if ((err.transpose() * err) < 1.0e-15) {
      break;
    }
  }

  return p;
}

/**
 * Pinhole-Radtan4 Undistort
 */
Eigen::Vector2d pinhole_radtan4_undistort(const Eigen::VectorXd &params,
                                          const Eigen::Vector2d &z) {
  // Back-project and undistort
  const real_t fx = params(0);
  const real_t fy = params(1);
  const real_t cx = params(2);
  const real_t cy = params(3);
  const real_t px = (z.x() - cx) / fx;
  const real_t py = (z.y() - cy) / fy;
  const Eigen::Vector2d p{px, py};
  const Eigen::Vector2d p_undist = radtan4_undistort(params.tail(4), p);

  // Project undistorted point to image plane
  const real_t x = p_undist.x() * fx + cx;
  const real_t y = p_undist.y() * fy + cy;
  const Eigen::Vector2d z_undist = {x, y};

  return z_undist;
}

/**
 * Pinhole-Radtan4 Project
 */
int pinhole_radtan4_project(const int res[2],
                            const Eigen::VectorXd &params,
                            const Eigen::Vector3d &p_C,
                            Eigen::Vector2d &z_hat) {
  // Setup
  const Eigen::Vector4d proj_params = params.head(4);
  const Eigen::Vector4d dist_params = params.tail(4);

  // Project, distort and then scale and center
  const real_t fx = proj_params(0);
  const real_t fy = proj_params(1);
  const real_t cx = proj_params(2);
  const real_t cy = proj_params(3);
  const Eigen::Vector2d p = project_point(p_C);
  const Eigen::Vector2d p_d = radtan4_distort(dist_params, p);
  z_hat.x() = fx * p_d.x() + cx;
  z_hat.y() = fy * p_d.y() + cy;

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res[0]);
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res[1]);
  const bool z_ok = (p_C.z() > 0.0);
  const bool valid = (x_ok && y_ok && z_ok) ? true : false;

  return (valid) ? 0 : -1;
}

/**
 * Convert cv::Mat to Eigen::Matrix
 */
void convert(const cv::Mat &x, Eigen::MatrixXd &y) {
  y.resize(x.rows, x.cols);

  for (int i = 0; i < x.rows; i++) {
    for (int j = 0; j < x.cols; j++) {
      y(i, j) = x.at<real_t>(i, j);
    }
  }
}

/**
 * Convert cv::Mat to Eigen::Matrix
 */
Eigen::MatrixXd convert(const cv::Mat &x) {
  Eigen::MatrixXd y;
  convert(x, y);
  return y;
}

/**
 * SolvePnP
 */
int solvepnp(const int cam_res[2],
             const Eigen::VectorXd &cam_params,
             const std::vector<Eigen::Vector2d> &keypoints,
             const std::vector<Eigen::Vector3d> &object_points,
             Eigen::Matrix4d &T_CF) {
  UNUSED(cam_res);
  assert(keypoints.size() == object_points.size());

  // Create object points (counter-clockwise, from bottom left)
  // Note: SolvPnP assumes radtan which may not be true, therefore we
  // have to manually undistort the keypoints ourselves
  size_t nb_points = keypoints.size();
  std::vector<cv::Point2f> img_pts;
  std::vector<cv::Point3f> obj_pts;
  for (size_t i = 0; i < nb_points; i++) {
    // Check keypoint is valid
    const Eigen::Vector2d z = keypoints[i];
    const bool x_ok = (z.x() >= 0 && z.x() <= cam_res[0]);
    const bool y_ok = (z.y() >= 0 && z.y() <= cam_res[1]);
    const bool valid = (x_ok && y_ok) ? true : false;
    if (valid == false) {
      printf("INVALID!\n");
      continue;
    }

    // Keypoint
    const Eigen::Vector2d &kp = pinhole_radtan4_undistort(cam_params, z);
    img_pts.emplace_back(kp.x(), kp.y());

    // Object point
    const Eigen::Vector3d &pt = object_points[i];
    obj_pts.emplace_back(pt.x(), pt.y(), pt.z());
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = cam_params(0);
  K.at<double>(1, 1) = cam_params(1);
  K.at<double>(0, 2) = cam_params(2);
  K.at<double>(1, 2) = cam_params(3);

  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);
  cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec);

  // Form relative tag pose as a 4x4 tfation matrix
  // -- Convert Rodrigues rotation vector to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  // -- Form full transformation matrix
  T_CF.setIdentity();
  T_CF.block<3, 3>(0, 0) = convert(R);
  T_CF.block<3, 1>(0, 3) = convert(tvec);

  return 0;
}

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
