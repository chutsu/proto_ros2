#include <iostream>
#include <sys/stat.h>
#include <mutex>

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
 * Record
 */
void record(const int64_t ts,
            const sbgc_t *sbgc,
            const cv::Mat &frame0,
            const cv::Mat &frame1,
            std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> &images,
            std::vector<std::pair<int64_t, Eigen::Vector3d>> &joint_angles) {
  Eigen::Vector3d joints;
  joints[0] = deg2rad(sbgc->encoder_angles[2]); // Motor 1: Yaw
  joints[1] = deg2rad(sbgc->encoder_angles[0]); // Monor 2: Roll
  joints[2] = deg2rad(sbgc->encoder_angles[1]); // Motor 3: Pitch

  printf("joints: [%.2f, %.2f, %.2f]\n",
         sbgc->encoder_angles[0],
         sbgc->encoder_angles[1],
         sbgc->encoder_angles[2]);

  images.push_back({ts, frame0.clone(), frame1.clone()});
  joint_angles.push_back({ts, joints});
}

/**
 * Save data
 */
void save_data(
    const std::string save_dir,
    const std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> &images,
    const std::vector<std::pair<int64_t, Eigen::Vector3d>> &joint_angles) {
  // Save image pairs
  // -- Setup save directory
  const std::string cam0_path = save_dir + "/cam0/data";
  const std::string cam1_path = save_dir + "/cam1/data";
  const std::string grid0_cam0_path = save_dir + "/grid0/cam0";
  const std::string grid0_cam1_path = save_dir + "/grid0/cam1";
  dir_create(save_dir);
  dir_create(cam0_path);
  dir_create(cam1_path);
  dir_create(grid0_cam0_path);
  dir_create(grid0_cam1_path);

  // -- Image pairs
  int num_rows = 6;
  int num_cols = 6;
  double tsize = 0.038;
  double tspacing = 0.3;
  AprilTags::AprilGridDetector detector;
  aprilgrid_t *grid0 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);
  aprilgrid_t *grid1 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

  for (const auto &tuple : images) {
    const auto ts = std::get<0>(tuple);
    const auto frame0 = std::get<1>(tuple);
    const auto frame1 = std::get<2>(tuple);

    const std::string fname = std::to_string(ts);
    const std::string frame0_path = cam0_path + "/" + fname + ".png";
    const std::string frame1_path = cam1_path + "/" + fname + ".png";
    const std::string det0_path = grid0_cam0_path + "/" + fname + ".csv";
    const std::string det1_path = grid0_cam1_path + "/" + fname + ".csv";

    cv::imwrite(frame0_path, frame0);
    cv::imwrite(frame1_path, frame1);

    detect_aprilgrid(detector, ts, frame0, grid0);
    aprilgrid_save(grid0, det0_path.c_str());

    detect_aprilgrid(detector, ts, frame1, grid1);
    aprilgrid_save(grid1, det1_path.c_str());

    const cv::Mat viz0 = aprilgrid_draw(grid0, frame0);
    const cv::Mat viz1 = aprilgrid_draw(grid1, frame1);
    cv::Mat viz;
    cv::hconcat(viz0, viz1, viz);
    cv::imshow("Viz", viz);
    cv::waitKey(1);

    aprilgrid_clear(grid0);
    aprilgrid_clear(grid1);
  }

  aprilgrid_free(grid0);
  aprilgrid_free(grid1);

  // -- Joint angles
  const int num_views = images.size();
  const int num_joints = 3;
  const std::string joints_path = save_dir + "/joint_angles.dat";
  FILE *joints_dat = fopen(joints_path.c_str(), "w");

  fprintf(joints_dat, "num_views: %d\n", num_views);
  fprintf(joints_dat, "num_joints: %d\n", num_joints);
  fprintf(joints_dat, "\n");
  fprintf(joints_dat, "#ts,joint0,joint1,joint2\n");
  for (const auto &pair : joint_angles) {
    const auto ts = pair.first;
    const auto joints = pair.second;
    fprintf(joints_dat, "%ld,", ts);
    fprintf(joints_dat, "%f,%f,%f", joints.x(), joints.y(), joints.z());
    fprintf(joints_dat, "\n");
  }
  fclose(joints_dat);

  // -- Poses
  const int64_t first_ts = std::get<0>(images.front());
  const std::string poses_path = save_dir + "/poses.dat";
  FILE *poses_dat = fopen(poses_path.c_str(), "w");
  fprintf(poses_dat, "num_poses: 1\n");
  fprintf(poses_dat, "\n");
  fprintf(poses_dat, "#ts,x,y,z,qw,qx,qy,qz\n");
  fprintf(poses_dat, "%ld,0.0,0.0,0.0,1.0,0.0,0.0,0.0\n", first_ts);
  fclose(poses_dat);
}

int main(int argc, char *argv[]) {
  // Setup
  rs_d435i_t device;
  sbgc_t sbgc;

  // int num_rows = 6;
  // int num_cols = 6;
  // double tsize = 0.038;
  // double tspacing = 0.3;
  // AprilTags::AprilGridDetector detector;
  // aprilgrid_t *grid0 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);
  // aprilgrid_t *grid1 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

  std::mutex mtx;
  cv::Mat frame0;
  cv::Mat frame1;
  std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> images;
  std::vector<std::pair<int64_t, Eigen::Vector3d>> joint_angles;

  // -- Register image callback
  device.image_callback = [&](const rs2::video_frame &ir0,
                              const rs2::video_frame &ir1) {
    // std::lock_guard<std::mutex> lock(mtx);
    const int width = ir0.get_width();
    const int height = ir0.get_height();
    const std::string encoding = "mono8";
    frame0 = frame2cvmat(ir0, width, height, CV_8UC1);
    frame1 = frame2cvmat(ir1, width, height, CV_8UC1);
  };

  // Start realsense and SBGC
  // device.loop();
  device.start();
  sbgc_connect(&sbgc, SBGC_DEV);
  if (sbgc_on(&sbgc) != 0) {
    printf("Failed to connect to SBGC!");
    exit(-1);
  }
  sleep(3);

  // Zero gimbal
  sbgc_set_angle(&sbgc, 0, 0, 0);
  sleep(3);

  const auto range_roll = linspace(-30, 30, 8);
  const auto range_pitch = linspace(-30, 30, 8);
  const auto range_yaw = linspace(-30, 30, 8);
  for (auto roll : range_roll) {
    for (auto pitch : range_pitch) {
      for (auto yaw : range_yaw) {
        sbgc_set_angle(&sbgc, roll, pitch, yaw);

        sleep(3);
        printf("Capture! ");
        fflush(stdout);

        sbgc_update(&sbgc);
        std::lock_guard<std::mutex> lock(mtx);
        const auto ts = timestamp_now();
        record(ts, &sbgc, frame0, frame1, images, joint_angles);
      }
    }
  }

  // const int num_captures = 30;
  // for (int i = 0; i < num_captures; i++) {
  //   sleep(3);
  //   printf("Capture! ");

  //   sbgc_update(&sbgc);
  //   std::lock_guard<std::mutex> lock(mtx);
  //   const auto ts = timestamp_now();
  //   record(ts, &sbgc, frame0, frame1, images, joint_angles);
  // }

  // Save data
  const std::string save_dir = "/home/chutsu/calib_gimbal";
  save_data(save_dir, images, joint_angles);

  // Clean up
  sbgc_off(&sbgc);
  // aprilgrid_free(grid0);
  // aprilgrid_free(grid1);

  return 0;
}
