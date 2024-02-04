#include <iostream>
#include <sys/stat.h>

#include "util.hpp"
#include "realsense.hpp"

// clang-format off
typedef std::vector<std::tuple<int64_t, cv::Mat, cv::Mat, cv::Mat, cv::Mat>> ImageData;
typedef std::vector<std::tuple<int64_t, Eigen::Vector3d, Eigen::Vector3d>> ImuData;
typedef std::vector<std::pair<int64_t, Eigen::Vector3d>> JointData;
// clang-format on

/**
 * Save data
 */
void save_data(const std::string save_dir,
               const ImageData &images,
               const ImuData &imu0_data,
               const ImuData &imu1_data,
               const bool debug = true) {
  // Save image pairs
  // -- Setup save directory
  const std::string imu0_path = save_dir + "/imu0";
  const std::string imu1_path = save_dir + "/imu1";
  const std::string cam0_path = save_dir + "/cam0";
  const std::string cam1_path = save_dir + "/cam1";
  const std::string cam2_path = save_dir + "/cam2";
  const std::string cam3_path = save_dir + "/cam3";
  const std::string grid0_cam0_path = save_dir + "/grid0/cam0";
  const std::string grid0_cam1_path = save_dir + "/grid0/cam1";
  const std::string grid0_cam2_path = save_dir + "/grid0/cam2";
  const std::string grid0_cam3_path = save_dir + "/grid0/cam3";
  dir_create(save_dir);
  dir_create(imu0_path);
  dir_create(imu1_path);
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

  printf("Saving images: ");
  for (const auto &tuple : images) {
    printf(".");
    fflush(stdout);

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
      cv::hconcat(viz2, viz3, viz_row0);
      cv::hconcat(viz0, viz1, viz_row1);

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
  printf("\n");
  aprilgrid_free(grid0);
  aprilgrid_free(grid1);
  aprilgrid_free(grid2);
  aprilgrid_free(grid3);

  // -- Imu Data
  {
    const std::string imu0_csv_path = imu0_path + "/data.csv";
    FILE *imu0_csv = fopen(imu0_csv_path.c_str(), "w");

    printf("Saving imu0 data: %s\n", imu0_csv_path.c_str());
    for (const auto &tuple : imu0_data) {
      const auto ts = std::get<0>(tuple);
      const auto acc = std::get<1>(tuple);
      const auto gyr = std::get<2>(tuple);
      fprintf(imu0_csv, "%ld,", ts);
      fprintf(imu0_csv, "%f,", acc.x());
      fprintf(imu0_csv, "%f,", acc.y());
      fprintf(imu0_csv, "%f,", acc.z());
      fprintf(imu0_csv, "%f,", gyr.x());
      fprintf(imu0_csv, "%f,", gyr.y());
      fprintf(imu0_csv, "%f\n", gyr.z());
    }
    fclose(imu0_csv);
  }
  {
    const std::string imu1_csv_path = imu1_path + "/data.csv";
    FILE *imu1_csv = fopen(imu1_csv_path.c_str(), "w");

    printf("Saving imu1 data: %s\n", imu1_csv_path.c_str());
    for (const auto &tuple : imu1_data) {
      const auto ts = std::get<0>(tuple);
      const auto acc = std::get<1>(tuple);
      const auto gyr = std::get<2>(tuple);
      fprintf(imu1_csv, "%ld,", ts);
      fprintf(imu1_csv, "%f,", acc.x());
      fprintf(imu1_csv, "%f,", acc.y());
      fprintf(imu1_csv, "%f,", acc.z());
      fprintf(imu1_csv, "%f,", gyr.x());
      fprintf(imu1_csv, "%f,", gyr.y());
      fprintf(imu1_csv, "%f\n", gyr.z());
    }
    fclose(imu1_csv);
  }

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
  // Parse command line arguments
  if (argc != 2) {
    printf("calib_mult_rs_record <save_path>\n");
    printf("Example: calib_mult_rs_record /tmp/calib_multi_rs\n");
    return -1;
  }

  // Save path
  const std::string save_dir = argv[1];
  if (path_exists(save_dir)) {
    printf("save path [%s] already exists!\n", save_dir.c_str());
    return -1;
  }

  // Setup
  rs_multi_d435i_t rs_devices;
  const std::string serial0 = "843112071984";
  const std::string serial1 = "943222072527";
  const auto cfg0 = rs_devices.configs[serial0];
  const auto cfg1 = rs_devices.configs[serial1];

  // Start pipeline
  ImageData images;
  ImuData imu0_data;
  ImuData imu1_data;

  lerp_buf_t buf0;
  lerp_buf_t buf1;
  rs2::frame ir0_frame;
  rs2::frame ir1_frame;
  rs2::frame ir2_frame;
  rs2::frame ir3_frame;

  rs_devices.pipelines[serial0].start(cfg0, [&](const rs2::frame &frame) {
    // Handle motion frame
    if (auto mf = frame.as<rs2::motion_frame>()) {
      if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        // Accelerometer measurement
        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        buf0.addAccel(ts_s, data.x, data.y, data.z);

      } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
        // Gyroscope measurement
        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        buf0.addGyro(ts_s, data.x, data.y, data.z);
      }

      if (buf0.ready()) {
        // IMU measurement
        buf0.interpolate();
        while (buf0.lerped_gyro_ts_.size()) {
          // Timestamp
          const auto ts = buf0.lerped_gyro_ts_.front();
          buf0.lerped_gyro_ts_.pop_front();
          buf0.lerped_accel_ts_.pop_front();

          // Accel
          const auto accel = buf0.lerped_accel_data_.front();
          buf0.lerped_accel_data_.pop_front();

          // Gyro
          const auto gyro = buf0.lerped_gyro_data_.front();
          buf0.lerped_gyro_data_.pop_front();

          // Save imu data
          imu0_data.push_back({ts * 1e9, accel, gyro});
        }

        buf0.clear();
      }

      return;
    }

    // Handle stereo images
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      ir0_frame = fs.get_infrared_frame(1);
      ir1_frame = fs.get_infrared_frame(2);
    }
  });

  rs_devices.pipelines[serial1].start(cfg1, [&](const rs2::frame &frame) {
    // Handle motion frame
    if (auto mf = frame.as<rs2::motion_frame>()) {
      if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        // Accelerometer measurement
        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        buf1.addAccel(ts_s, data.x, data.y, data.z);

      } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
        // Gyroscope measurement
        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        buf1.addGyro(ts_s, data.x, data.y, data.z);
      }

      if (buf1.ready()) {
        // IMU measurement
        buf1.interpolate();
        while (buf1.lerped_gyro_ts_.size()) {
          // Timestamp
          const auto ts = buf1.lerped_gyro_ts_.front();
          buf1.lerped_gyro_ts_.pop_front();
          buf1.lerped_accel_ts_.pop_front();

          // Accel
          const auto accel = buf1.lerped_accel_data_.front();
          buf1.lerped_accel_data_.pop_front();

          // Gyro
          const auto gyro = buf1.lerped_gyro_data_.front();
          buf1.lerped_gyro_data_.pop_front();

          // Save imu data
          imu1_data.push_back({ts * 1e9, accel, gyro});
        }

        buf1.clear();
      }

      return;
    }

    // Handle stereo images
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      ir2_frame = fs.get_infrared_frame(1);
      ir3_frame = fs.get_infrared_frame(2);
    }
  });

  // Loop
  int64_t last_ts = 0;
  bool run = true;
  while (run) {
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
      auto frame0 = frame2cvmat(ir0_frame, width, height, CV_8UC1);
      auto frame1 = frame2cvmat(ir1_frame, width, height, CV_8UC1);
      auto frame2 = frame2cvmat(ir2_frame, width, height, CV_8UC1);
      auto frame3 = frame2cvmat(ir3_frame, width, height, CV_8UC1);
      images.push_back(
          {ts, frame0.clone(), frame1.clone(), frame2.clone(), frame3.clone()});
      last_ts = ts;

      cv::Mat viz_row0;
      cv::Mat viz_row1;
      cv::Mat viz;
      cv::hconcat(frame2, frame3, viz_row0);
      cv::hconcat(frame0, frame1, viz_row1);
      cv::vconcat(viz_row0, viz_row1, viz);

      cv::imshow("Viz", viz);
      if (cv::waitKey(1) == 'q') {
        run = false;
      }
    }
  }
  cv::destroyAllWindows();

  // Stop realsenses
  rs_devices.pipelines[serial0].stop();
  rs_devices.pipelines[serial1].stop();

  // Save data
  save_data(save_dir, images, imu0_data, imu1_data);

  return 0;
}
