#include <iostream>
#include <mutex>

#include "util.hpp"
#include "realsense.hpp"

/**
 * Save data
 */
void save_data(
    const std::string save_dir,
    const std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> &images) {
  // Save image pairs
  // -- Setup save directory
  const std::string cam0_dir = save_dir + "/cam0";
  const std::string cam1_dir = save_dir + "/cam1";
  const std::string grid0_dir = save_dir + "/grid0";
  dir_create(save_dir);
  dir_create(cam0_dir);
  dir_create(cam1_dir);
  dir_create(grid0_dir);
  dir_create(grid0_dir + "/cam0");
  dir_create(grid0_dir + "/cam1");

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
    const std::string frame0_path = cam0_dir + "/" + fname + ".png";
    const std::string frame1_path = cam1_dir + "/" + fname + ".png";
    const std::string det0_path = grid0_dir + "/cam0/" + fname + ".csv";
    const std::string det1_path = grid0_dir + "/cam1/" + fname + ".csv";

    cv::imwrite(frame0_path, frame0);
    cv::imwrite(frame1_path, frame0);
    grid0->timestamp = ts;
    grid1->timestamp = ts;
    detect_aprilgrid(detector, ts, frame0, grid0);
    detect_aprilgrid(detector, ts, frame1, grid1);
    aprilgrid_save(grid0, det0_path.c_str());
    aprilgrid_save(grid1, det1_path.c_str());

    const cv::Mat viz0 = aprilgrid_draw(grid0, frame0);
    const cv::Mat viz1 = aprilgrid_draw(grid1, frame1);
    cv::Mat viz;
    cv::hconcat(viz0, viz1, viz);
    cv::imshow("Viz", viz);
    cv::waitKey(100);

    aprilgrid_clear(grid0);
    aprilgrid_clear(grid1);
  }

  aprilgrid_free(grid0);
  aprilgrid_free(grid1);
}

int main(int argc, char *argv[]) {
  // Setup
  rs_d435i_t device;
  cv::Mat frame0;
  cv::Mat frame1;
  std::vector<std::tuple<int64_t, cv::Mat, cv::Mat>> images;

  // -- Register image callback
  device.image_callback = [&](const rs2::video_frame &ir0,
                              const rs2::video_frame &ir1) {
    const int width = ir0.get_width();
    const int height = ir0.get_height();
    const std::string encoding = "mono8";
    frame0 = frame2cvmat(ir0, width, height, CV_8UC1);
    frame1 = frame2cvmat(ir1, width, height, CV_8UC1);
    images.push_back({time_now(), frame0.clone(), frame1.clone()});

    cv::Mat viz;
    cv::hconcat(frame0, frame1, viz);
    cv::imshow("Viz", viz);
    cv::waitKey(1);
  };

  // Start realsense
  device.start();
  sleep(10);
  device.stop();

  // Save data
  const std::string save_dir = "/home/chutsu/calib_camera";
  save_data(save_dir, images);

  return 0;
}
