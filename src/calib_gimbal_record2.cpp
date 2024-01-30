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
 * Save data
 */
void save_data(
    const std::string save_dir,
    const std::vector<std::tuple<int64_t, cv::Mat, cv::Mat, cv::Mat, cv::Mat>>
        &images,
    const std::vector<std::pair<int64_t, Eigen::Vector3d>> &joint_angles,
    const bool debug = false) {
  if (images.size() != joint_angles.size()) {
    printf("Error: images.size() != joint_angles.size()\n");
    return;
  } else if (images.size() == 0) {
    printf("Error: images.size() == 0\n");
    return;
  }

  // Save image pairs
  // -- Setup save directory
  const std::string cam0_path = save_dir + "/cam0";
  const std::string cam1_path = save_dir + "/cam1";
  const std::string cam2_path = save_dir + "/cam2";
  const std::string cam3_path = save_dir + "/cam3";
  const std::string grid0_cam0_path = save_dir + "/grid0/cam0";
  const std::string grid0_cam1_path = save_dir + "/grid0/cam1";
  const std::string grid0_cam2_path = save_dir + "/grid0/cam2";
  const std::string grid0_cam3_path = save_dir + "/grid0/cam3";
  dir_create(save_dir);
  dir_create(cam0_path);
  dir_create(cam1_path);
  dir_create(cam2_path);
  dir_create(cam3_path);
  dir_create(grid0_cam0_path);
  dir_create(grid0_cam1_path);
  dir_create(grid0_cam2_path);
  dir_create(grid0_cam3_path);

  // -- Image pairs
  int num_rows = 6;
  int num_cols = 6;
  double tsize = 0.038;
  double tspacing = 0.3;
  AprilTags::AprilGridDetector detector;
  aprilgrid_t *grid0 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);
  aprilgrid_t *grid1 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);
  aprilgrid_t *grid2 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);
  aprilgrid_t *grid3 = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

  for (const auto &tuple : images) {
    const auto ts = std::get<0>(tuple);
    const auto frame0 = std::get<1>(tuple);
    const auto frame1 = std::get<2>(tuple);
    const auto frame2 = std::get<3>(tuple);
    const auto frame3 = std::get<4>(tuple);

    const std::string fname = std::to_string(ts);
    const std::string frame0_path = cam0_path + "/" + fname + ".png";
    const std::string frame1_path = cam1_path + "/" + fname + ".png";
    const std::string frame2_path = cam2_path + "/" + fname + ".png";
    const std::string frame3_path = cam3_path + "/" + fname + ".png";

    const std::string det0_path = grid0_cam0_path + "/" + fname + ".csv";
    const std::string det1_path = grid0_cam1_path + "/" + fname + ".csv";
    const std::string det2_path = grid0_cam2_path + "/" + fname + ".csv";
    const std::string det3_path = grid0_cam3_path + "/" + fname + ".csv";

    cv::imwrite(frame0_path, frame0);
    cv::imwrite(frame1_path, frame1);
    cv::imwrite(frame2_path, frame2);
    cv::imwrite(frame3_path, frame3);

    detect_aprilgrid(detector, ts, frame0, grid0);
    detect_aprilgrid(detector, ts, frame1, grid1);
    detect_aprilgrid(detector, ts, frame2, grid2);
    detect_aprilgrid(detector, ts, frame3, grid3);

    aprilgrid_save(grid0, det0_path.c_str());
    aprilgrid_save(grid1, det1_path.c_str());
    aprilgrid_save(grid2, det2_path.c_str());
    aprilgrid_save(grid3, det3_path.c_str());

    if (debug) {
      const cv::Mat viz0 = aprilgrid_draw(grid0, frame0);
      const cv::Mat viz1 = aprilgrid_draw(grid1, frame1);
      const cv::Mat viz2 = aprilgrid_draw(grid2, frame2);
      const cv::Mat viz3 = aprilgrid_draw(grid3, frame3);

      cv::Mat viz_row0;
      cv::Mat viz_row1;
      cv::hconcat(viz0, viz1, viz_row0);
      cv::hconcat(viz2, viz3, viz_row1);

      cv::Mat viz;
      cv::vconcat(viz_row0, viz_row1, viz);

      cv::imshow("Detection", viz);
      cv::waitKey(1);
    }

    aprilgrid_clear(grid0);
    aprilgrid_clear(grid1);
    aprilgrid_clear(grid2);
    aprilgrid_clear(grid3);
  }

  aprilgrid_free(grid0);
  aprilgrid_free(grid1);
  aprilgrid_free(grid2);
  aprilgrid_free(grid3);

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
  sbgc_t sbgc;
  rs_multi_d435i_t rs_devices;
  const std::string serial0 = "843112071984";
  const std::string serial1 = "943222072527";
  const auto cfg0 = rs_devices.configs[serial0];
  const auto cfg1 = rs_devices.configs[serial1];

  std::mutex mtx;
  bool run = true;
  cv::Mat frame0;
  cv::Mat frame1;
  cv::Mat frame2;
  cv::Mat frame3;
  double target_angle0 = 0.0;
  double target_angle1 = 0.0;
  double target_angle2 = 0.0;
  std::vector<std::tuple<int64_t, cv::Mat, cv::Mat, cv::Mat, cv::Mat>> images;
  std::vector<std::pair<int64_t, Eigen::Vector3d>> joint_angles;

  // -- Realsense thread
  auto realsense_thread = [&]() {
    // Start pipeline
    rs2::frame ir0_frame;
    rs2::frame ir1_frame;
    rs2::frame ir2_frame;
    rs2::frame ir3_frame;

    rs_devices.pipelines[serial0].start(cfg0, [&](const rs2::frame &frame) {
      if (rs2::frameset fs = frame.as<rs2::frameset>()) {
        ir0_frame = fs.get_infrared_frame(1);
        ir1_frame = fs.get_infrared_frame(2);
      }
    });

    rs_devices.pipelines[serial1].start(cfg1, [&](const rs2::frame &frame) {
      if (rs2::frameset fs = frame.as<rs2::frameset>()) {
        ir2_frame = fs.get_infrared_frame(1);
        ir3_frame = fs.get_infrared_frame(2);
      }
    });

    int64_t last_ts = 0;
    while (run) {
      std::lock_guard<std::mutex> lock(mtx);
      const auto ts0 = vframe2ts(ir0_frame, true);
      const auto ts1 = vframe2ts(ir1_frame, true);
      const auto ts2 = vframe2ts(ir2_frame, true);
      const auto ts3 = vframe2ts(ir3_frame, true);
      const std::vector<uint64_t> tss = {ts0, ts1, ts2, ts3};
      const auto ts_max = *std::max_element(tss.begin(), tss.end());
      const auto ts_min = *std::min_element(tss.begin(), tss.end());

      if ((ts_max - ts_min) * 1e-9 < 0.01 && (ts0 - last_ts) * 1e-9 > 0.01) {
        const int64_t ts = vframe2ts(ir0_frame, true);
        const int width = rs_devices.ir_width;
        const int height = rs_devices.ir_height;
        frame0 = frame2cvmat(ir0_frame, width, height, CV_8UC1);
        frame1 = frame2cvmat(ir1_frame, width, height, CV_8UC1);
        frame2 = frame2cvmat(ir2_frame, width, height, CV_8UC1);
        frame3 = frame2cvmat(ir3_frame, width, height, CV_8UC1);
        last_ts = ts;

        cv::Mat viz;
        cv::hconcat(frame0, frame2, viz);
        cv::imshow("Viz", viz);
        if (cv::waitKey(1) == 'q') {
          run = false;
        }
      }
    }

    rs_devices.pipelines[serial0].stop();
    rs_devices.pipelines[serial1].stop();
  };

  // -- SBGC thread
  auto sbgc_thread = [&]() {
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
      // std::lock_guard<std::mutex> lock(mtx);
      sbgc_set_angle(&sbgc, target_angle0, target_angle1, target_angle2);
      sbgc_update(&sbgc);
      usleep(10 * 1000);
    }

    sbgc_off(&sbgc);
  };

  // -- Calibration data capture thread
  auto capture_thread = [&]() {
    sleep(5);

    // Zero gimbal
    sbgc_set_angle(&sbgc, 0, 0, 0);
    sleep(3);

    const int num_captures = 300;
    for (int i = 0; i < num_captures; i++) {
      // sleep(3);
      usleep(100 * 1000);
      printf("Capture! ");

      std::lock_guard<std::mutex> lock(mtx);
      const auto ts = timestamp_now();
      const Eigen::Vector3d joints{deg2rad(sbgc.encoder_angles[2]),
                                   deg2rad(sbgc.encoder_angles[0]),
                                   deg2rad(sbgc.encoder_angles[1])};
      images.push_back(
          {ts, frame0.clone(), frame1.clone(), frame2.clone(), frame3.clone()});
      joint_angles.push_back({ts, joints});
    }

    // // Sample different joint angles
    // const auto range_roll = linspace(-30, 30, 8);
    // const auto range_pitch = linspace(-30, 30, 8);
    // const auto range_yaw = linspace(-30, 30, 8);
    // for (auto roll : range_roll) {
    //   for (auto pitch : range_pitch) {
    //     for (auto yaw : range_yaw) {
    //       sbgc_set_angle(&sbgc, roll, pitch, yaw);

    //       sleep(3);
    //       printf("Capture! ");
    //       fflush(stdout);

    //       sbgc_update(&sbgc);
    //       std::lock_guard<std::mutex> lock(mtx);
    //       const auto ts = timestamp_now();
    //       record(ts, &sbgc, frame0, frame1, images, joint_angles);
    //     }
    //   }
    // }

    // Stop other threads
    run = false;

    // Save data
    const std::string save_dir = "/home/chutsu/calib_gimbal";
    save_data(save_dir, images, joint_angles);
  };

  // Run threads
  std::thread thread0(realsense_thread);
  std::thread thread1(sbgc_thread);
  std::thread thread2(capture_thread);
  thread0.join();
  thread1.join();
  thread2.join();

  return 0;
}
