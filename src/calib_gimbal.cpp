#include <iostream>
#include <sys/stat.h>
#include <mutex>

#include <Eigen/Core>

#define SBGC_IMPLEMENTATION
#define SBGC_DEV "/dev/ttyUSB0"
#include <sbgc.h>

#include "realsense.hpp"

std::vector<double> linspace(double start, double end, int numPoints) {
  std::vector<double> result;

  if (numPoints <= 1) {
    result.push_back(start);
    return result;
  }

  double step = (end - start) / (numPoints - 1);
  for (int i = 0; i < numPoints; ++i) {
    result.push_back(start + i * step);
  }

  return result;
}

int path_exists(const std::string &path) {
  struct stat buffer;
  return stat(path.c_str(), &buffer) == 0;
}

int dir_create(const std::string &path) {
  if (path_exists(path)) {
    return 0;
  }

  if (mkdir(path.c_str(), 0777) != 0) {
    return -1;
  }

  return 0;
}

void record(const int64_t ts,
            const sbgc_t *sbgc,
            const cv::Mat &frame0,
            const cv::Mat &frame1,
            const Eigen::Vector3d &joints,
            std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> &images,
            std::vector<std::pair<int64_t, Eigen::Vector3d>> &joint_angles) {
  Eigen::Vector3d joints;
  joints.x() = sbgc.camera_angles[0];
  joints.y() = sbgc.camera_angles[1];
  joints.z() = sbgc.camera_angles[2];

  printf("joints: [%.2f, %.2f, %.2f]\n", joints.x(), joints.y(), joints.z());
  cv::Mat viz;
  cv::hconcat(frame0, frame1, viz);
  cv::imshow("Viz", viz);
  cv::waitKey(10);

  images.push_back({ts, frame0.clone(), frame1.clone()});
  joint_angles.push_back({ts, joints});
}

void save_data(
    const std::string save_dir,
    const std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> &images,
    const std::vector<std::pair<int64_t, Eigen::Vector3d>> &joint_angles) {
  // Save image pairs
  // -- Setup save directory
  const std::string cam0_path = save_dir + "/cam0";
  const std::string cam1_path = save_dir + "/cam1";
  const std::string joints_path = save_dir + "/joints";
  dir_create(save_dir);
  dir_create(cam0_path);
  dir_create(cam1_path);
  dir_create(joints_path);

  // -- Image pairs
  for (const auto &tuple : images) {
    const auto ts = std::get<0>(tuple);
    const auto frame0 = std::get<1>(tuple);
    const auto frame1 = std::get<2>(tuple);

    const std::string fname = std::to_string(ts) + ".png";
    const std::string frame0_path = cam0_path + "/" + fname;
    const std::string frame1_path = cam1_path + "/" + fname;

    cv::imwrite(frame0_path, frame0);
    cv::imwrite(frame1_path, frame0);

    // cv::Mat viz;
    // cv::hconcat(pair.first, pair.second, viz);
    // cv::imshow("Viz", viz);
    // cv::waitKey(0);
  }

  // -- Joint angles
  const std::string joints_csv_path = joints_path + "/data.csv";
  FILE *joints_csv = fopen(joints_csv_path.c_str(), "w");
  fprintf(joints_csv, "#timestamp, joint0, joint1, joint2\n");
  for (const auto &pair : joint_angles) {
    const auto ts = pair.first;
    const auto joints = pair.second;
    fprintf(joints_csv, "%ld,", ts);
    fprintf(joints_csv, "%f,%f,%f", joints.x(), joints.y(), joints.z());
    fprintf(joints_csv, "\n");
  }
  fclose(joints_csv);
}

int main(int argc, char *argv[]) {
  // Setup
  rs_d435i_t device;
  sbgc_t sbgc;

  std::mutex mtx;
  cv::Mat frame0;
  cv::Mat frame1;
  std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> images;
  std::vector<std::pair<int64_t, Eigen::Vector3d>> joint_angles;

  // -- Register image callback
  device.image_callback = [&](const rs2::video_frame &ir0,
                              const rs2::video_frame &ir1) {
    std::lock_guard<std::mutex> lock(mtx);
    const int width = ir0.get_width();
    const int height = ir0.get_height();
    const std::string encoding = "mono8";
    frame0 = frame2cvmat(ir0, width, height, CV_8UC1);
    frame1 = frame2cvmat(ir1, width, height, CV_8UC1);
  };

  // Start realsense and SBGC
  device.start();
  sbgc_connect(&sbgc, SBGC_DEV);
  if (sbgc_on(&sbgc) != 0) {
    printf("Failed to connect to SBGC!");
    exit(-1);
  }

  // Zero gimbal
  // sbgc_set_angle(&sbgc, 0, 0, 0);
  // sleep(3);

  // const auto range_roll = linspace(-30, 30, 8);
  // const auto range_pitch = linspace(-30, 30, 10);
  // const auto range_yaw = linspace(-30, 30, 9);
  // for (auto roll : range_roll) {
  //   for (auto pitch : range_pitch) {
  //     for (auto yaw : range_yaw) {
  //       sbgc_set_angle(&sbgc, roll, pitch, yaw);

  //       sleep(3);
  //       printf("Capture!");

  //       sbgc_update(&sbgc);
  //       std::lock_guard<std::mutex> lock(mtx);
  //       const auto ts = timestamp_now();
  //       record(ts, &sbgc, frame0, frame1, joints, images, joint_angles);
  //     }
  //   }
  // }

  const int num_captures = 30;
  for (int i = 0; i < num_captures; i++) {
    sleep(3);
    printf("Capture!");

    sbgc_update(&sbgc);
    std::lock_guard<std::mutex> lock(mtx);
    const auto ts = timestamp_now();
    record(ts, &sbgc, frame0, frame1, joints, images, joint_angles);
  }

  // Save data
  const std::string save_dir = "/home/chutsu/calib_gimbal_data";
  save_data(save_dir, images, joints);

  // Clean up
  sbgc_off(&sbgc);

  return 0;
}
