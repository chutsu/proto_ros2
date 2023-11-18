#include <iostream>
#include <string.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define FATAL(M, ...)                                                          \
  fprintf(stdout,                                                              \
          "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__);                                                      \
  exit(-1)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr,                                                              \
          "\033[31m[ERROR] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

#define LOG_WARN(M, ...)                                                       \
  fprintf(stdout, "\033[33m[WARN] " M "\033[0m\n", ##__VA_ARGS__)

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#define ROS_PARAM(NH, X, Y)                                                    \
  if (NH.getParam(X, Y) == false) {                                            \
    std::cerr << "Failed to get ROS param [" << X << "]!" << std::endl;        \
    exit(-1);                                                                  \
  }

#define ROS_OPTIONAL_PARAM(NH, X, Y, DEFAULT_VALUE)                            \
  if (NH.getParam(X, Y) == false) {                                            \
    Y = DEFAULT_VALUE;                                                         \
  }

// String to timestamp
uint64_t str2ts(const std::string &s) {
  uint64_t ts = 0;
  int idx = 0;

  for (int i = s.length() - 1; i >= 0; i--) {
    const char c = s[i];

    if (c != '.') {
      const uint64_t base = static_cast<uint64_t>(pow(10, idx));
      ts += (c - '0') * base;
      //    ^ Convert ascii to integer
      // Note: character '0' has the ASCII code of 48
      idx++;
    }
  }

  return ts;
}

// Print RealSense Frame Timestamps
void print_rsframe_timestamps(const rs2::frame &frame) {
  const auto frame_ts_meta_key = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
  const auto frame_ts_us = frame.get_frame_metadata(frame_ts_meta_key);
  const auto sensor_ts_meta_key = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
  const auto sensor_ts_us = frame.get_frame_metadata(sensor_ts_meta_key);

  // clang-format off
  printf("RS2_FRAME_METADATA_FRAME_TIMESTAMP [us]:  %lld\n", frame_ts_us);
  printf("RS2_FRAME_METADATA_SENSOR_TIMESTAMP [us]: %lld\n", sensor_ts_us);
  printf("RS2_FRAME_METADATA_FRAME_TIMESTAMP [s]:   %.6f\n", frame_ts_us * 1e-6);
  printf("RS2_FRAME_METADATA_SENSOR_TIMESTAMP [s]:  %.6f\n", sensor_ts_us * 1e-6);
  // clang-format on

  switch (frame.get_frame_timestamp_domain()) {
    case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
      printf("Timestamp domain: hardware clock!\n");
      printf("Measured in relation to the camera clock\n");
      break;
    case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
      printf("Timestamp domain: system time!\n");
      printf("Measured in relation to the OS system clock\n");
      break;
    case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME:
      printf("Timestamp domain: global time!\n");
      printf("Measured in relation to the camera clock and converted to\n");
      printf("OS system clock by constantly measure the difference\n");
      break;
    case RS2_TIMESTAMP_DOMAIN_COUNT:
      printf("Timestamp domain: count! Not a valid domain\n");
      printf("Not a valid input: intended to be used in for-loops!\n");
      break;
    default:
      printf("Not a valid time domain!\n");
      break;
  }
}

// Print RealSense Exception
void print_rs_exception(const rs2::error &err) {
  switch (err.get_type()) {
    case RS2_EXCEPTION_TYPE_UNKNOWN:
      LOG_ERROR("RS2_EXCEPTION_TYPE_UNKNOWN Error!");
      break;
    case RS2_EXCEPTION_TYPE_CAMERA_DISCONNECTED:
      LOG_ERROR("RS2_EXCEPTION_TYPE_CAMERA_DISCONNECTED Error!");
      break;
    case RS2_EXCEPTION_TYPE_BACKEND:
      LOG_ERROR("RS2_EXCEPTION_TYPE_BACKEND Error!");
      break;
    case RS2_EXCEPTION_TYPE_INVALID_VALUE:
      LOG_ERROR("RS2_EXCEPTION_TYPE_INVALID_VALUE Error!");
      break;
    case RS2_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE:
      LOG_ERROR("RS2_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE Error!");
      break;
    case RS2_EXCEPTION_TYPE_NOT_IMPLEMENTED:
      LOG_ERROR("RS2_EXCEPTION_TYPE_NOT_IMPLEMENTED Error!");
      break;
    case RS2_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE:
      LOG_ERROR("RS2_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE Error!");
      break;
    case RS2_EXCEPTION_TYPE_IO:
      LOG_ERROR("RS2_EXCEPTION_TYPE_IO Error!");
      break;
    case RS2_EXCEPTION_TYPE_COUNT:
      LOG_ERROR("RS2_EXCEPTION_TYPE_COUNT Error!");
      break;
  };
  LOG_ERROR("FAILED AT FUNC  : %s", err.get_failed_function().c_str());
  LOG_ERROR("FAILED WITH ARGS: %s", err.get_failed_args().c_str());
  FATAL("[RealSense Exception]: %s", err.what());
}

// Realsense video frame to OpenCV matrix
cv::Mat frame2cvmat(const rs2::frame &frame,
                    const int width,
                    const int height,
                    const int format) {
  const cv::Size size(width, height);
  const auto stride = cv::Mat::AUTO_STEP;
  const cv::Mat cv_frame(size, format, (void *) frame.get_data(), stride);
  return cv_frame;
}

// Video frame to timestamp
uint64_t vframe2ts(const rs2::video_frame &vf, const bool correct_ts) {
  // Correct timestamp?
  if (correct_ts == false) {
    const auto ts_ms = vf.get_timestamp();
    const auto ts_ns = str2ts(std::to_string(ts_ms));
    return ts_ns;
  }

  // Frame metadata timestamp
  const auto frame_meta_key = RS2_FRAME_METADATA_FRAME_TIMESTAMP;
  if (vf.supports_frame_metadata(frame_meta_key) == false) {
    FATAL("[RS2_FRAME_METADATA_FRAME_TIMESTAMP] Not supported!");
  }
  const auto frame_ts_us = vf.get_frame_metadata(frame_meta_key);
  const auto frame_ts_ns = static_cast<uint64_t>(frame_ts_us) * 1000;

  // Sensor metadata timestamp
  const auto sensor_meta_key = RS2_FRAME_METADATA_SENSOR_TIMESTAMP;
  if (vf.supports_frame_metadata(sensor_meta_key) == false) {
    FATAL("[RS2_FRAME_METADATA_SENSOR_TIMESTAMP] Not supported! "
          "Did you patch the kernel?");
  }
  const auto sensor_ts_us = vf.get_frame_metadata(sensor_meta_key);
  const auto sensor_ts_ns = static_cast<uint64_t>(sensor_ts_us) * 1000;

  // Calculate corrected timestamp
  const auto ts_ms = vf.get_timestamp();
  const auto ts_ns = str2ts(std::to_string(ts_ms));
  const auto half_exposure_time_ns = frame_ts_ns - sensor_ts_ns;
  const auto ts_corrected_ns = ts_ns - half_exposure_time_ns;

  return static_cast<uint64_t>(ts_corrected_ns);
}

void debug_imshow(const cv::Mat &frame_left, const cv::Mat &frame_right) {
  // Display in a GUI
  cv::Mat frame;
  cv::hconcat(frame_left, frame_right, frame);
  cv::namedWindow("Stereo Module", cv::WINDOW_AUTOSIZE);
  cv::imshow("Stereo Module", frame);

  if (cv::waitKey(1) == 'q') {
    exit(-1);
  }
}

// Linear interpolation
template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

// IMU measurements interpolator
struct lerp_buf_t {
  std::deque<std::string> buf_type_;
  std::deque<double> buf_ts_;
  std::deque<Eigen::Vector3d> buf_data_;

  std::deque<double> lerped_gyro_ts_;
  std::deque<Eigen::Vector3d> lerped_gyro_data_;
  std::deque<double> lerped_accel_ts_;
  std::deque<Eigen::Vector3d> lerped_accel_data_;

  lerp_buf_t() {
  }

  bool ready() {
    if (buf_ts_.size() >= 3 && buf_type_.back() == "A") {
      return true;
    }
    return false;
  }

  void addAccel(const double ts_s,
                const double ax,
                const double ay,
                const double az) {
    buf_type_.push_back("A");
    buf_ts_.push_back(ts_s);
    buf_data_.emplace_back(ax, ay, az);
  }

  void addGyro(const double ts_s,
               const double wx,
               const double wy,
               const double wz) {
    if (buf_type_.size() && buf_type_.front() == "A") {
      buf_type_.push_back("G");
      buf_ts_.push_back(ts_s);
      buf_data_.emplace_back(wx, wy, wz);
    }
  }

  void print() {
    for (size_t i = 0; i < buf_ts_.size(); i++) {
      const double ts = buf_ts_.at(i);
      const std::string dtype = buf_type_.at(i);
      const Eigen::Vector3d data = buf_data_.at(i);
      const double x = data(0);
      const double y = data(1);
      const double z = data(1);
      printf("[%.6f] - [%s] - (%.2f, %.2f, %.2f)\n",
             ts,
             dtype.c_str(),
             x,
             y,
             z);
    }
  }

  void interpolate() {
    // Lerp data
    double t0 = 0;
    Eigen::Vector3d d0;
    double t1 = 0;
    Eigen::Vector3d d1;
    bool t0_set = false;

    std::deque<double> lerp_ts;
    std::deque<Eigen::Vector3d> lerp_data;

    double ts = 0.0;
    std::string dtype;
    Eigen::Vector3d data;

    while (buf_ts_.size()) {
      // Timestamp
      ts = buf_ts_.front();
      buf_ts_.pop_front();

      // Datatype
      dtype = buf_type_.front();
      buf_type_.pop_front();

      // Data
      data = buf_data_.front();
      buf_data_.pop_front();

      // Lerp
      if (t0_set == false && dtype == "A") {
        t0 = ts;
        d0 = data;
        t0_set = true;

      } else if (t0_set && dtype == "A") {
        t1 = ts;
        d1 = data;

        while (lerp_ts.size()) {
          const double lts = lerp_ts.front();
          const Eigen::Vector3d ldata = lerp_data.front();
          const double dt = t1 - t0;
          const double alpha = (lts - t0) / dt;

          lerped_accel_ts_.push_back(lts);
          lerped_accel_data_.push_back(lerp(d0, d1, alpha));

          lerped_gyro_ts_.push_back(lts);
          lerped_gyro_data_.push_back(ldata);

          lerp_ts.pop_front();
          lerp_data.pop_front();
        }

        t0 = t1;
        d0 = d1;

      } else if (t0_set && ts >= t0 && dtype == "G") {
        lerp_ts.push_back(ts);
        lerp_data.push_back(data);
      }
    }

    buf_ts_.push_back(ts);
    buf_type_.push_back(dtype);
    buf_data_.push_back(data);
  }

  void clear() {
    lerped_gyro_ts_.clear();
    lerped_gyro_data_.clear();
    lerped_accel_ts_.clear();
    lerped_accel_data_.clear();
  }
};

// Connect
rs2::device rs2_connect() {
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  }

  return devices[0];
}

// List realsense sensors
void rs2_list_sensors(const int device_idx = 0) {
  // Connect to the device
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device device;
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  } else {
    device = devices[device_idx]; // Default to first device
  }

  printf("Sensors:\n");
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    printf("  - %s\n", sensor_name);
  }
}

// Get realsense sensor by name
int rs2_get_sensor(const rs2::device &device,
                   const std::string &target,
                   rs2::sensor &sensor) {
  for (const auto &query_sensor : device.query_sensors()) {
    const auto sensor_name = query_sensor.get_info(RS2_CAMERA_INFO_NAME);
    if (strcmp(sensor_name, target.c_str()) == 0) {
      sensor = query_sensor;
      return 0;
    }
  }

  return -1;
}

// Realsense Format Converter
rs2_format rs2_format_convert(const std::string &format) {
  if (format == "ANY")
    return RS2_FORMAT_ANY;
  if (format == "Z16")
    return RS2_FORMAT_Z16;
  if (format == "DISPARITY16")
    return RS2_FORMAT_DISPARITY16;
  if (format == "XYZ32F")
    return RS2_FORMAT_XYZ32F;
  if (format == "YUYV")
    return RS2_FORMAT_YUYV;
  if (format == "RGB8")
    return RS2_FORMAT_RGB8;
  if (format == "BGR8")
    return RS2_FORMAT_BGR8;
  if (format == "RGBA8")
    return RS2_FORMAT_RGBA8;
  if (format == "BGRA8")
    return RS2_FORMAT_BGRA8;
  if (format == "Y8")
    return RS2_FORMAT_Y8;
  if (format == "Y16")
    return RS2_FORMAT_Y16;
  if (format == "RAW10")
    return RS2_FORMAT_RAW10;
  if (format == "RAW16")
    return RS2_FORMAT_RAW16;
  if (format == "RAW8")
    return RS2_FORMAT_RAW8;
  if (format == "UYVY")
    return RS2_FORMAT_UYVY;
  if (format == "MOTION_RAW")
    return RS2_FORMAT_MOTION_RAW;
  if (format == "MOTION_XYZ32F")
    return RS2_FORMAT_MOTION_XYZ32F;
  if (format == "GPIO_RAW")
    return RS2_FORMAT_GPIO_RAW;
  if (format == "6DOF")
    return RS2_FORMAT_6DOF;
  if (format == "DISPARITY32")
    return RS2_FORMAT_DISPARITY32;
  if (format == "Y10BPACK")
    return RS2_FORMAT_Y10BPACK;
  if (format == "DISTANCE")
    return RS2_FORMAT_DISTANCE;
  if (format == "MJPEG")
    return RS2_FORMAT_MJPEG;
  if (format == "COUNT")
    return RS2_FORMAT_COUNT;

  FATAL("Opps! Unsupported format [%s]!", format.c_str());
}

// Signal handler
bool keep_running = true;
void signal_handler(int sig) {
  UNUSED(sig);
  keep_running = false;
}

// Run Intel D435i Node
int main(int argc, char *argv[]) {
  // Initialize ROS node
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  const auto node = rclcpp::Node::make_shared("intel_d435i_node", options);
  const std::string topic_ir0 = "/rs/ir0/image";
  const std::string topic_ir1 = "/rs/ir1/image";
  const std::string topic_acc0 = "/rs/acc0/raw";
  const std::string topic_gyr0 = "/rs/gyr0/raw";
  const std::string topic_imu0 = "/rs/imu0/data";

  // clang-format off
  image_transport::ImageTransport it(node);
  const auto pub_ir0 = it.advertise(topic_ir0, 1);
  const auto pub_ir1 = it.advertise(topic_ir1, 1);
  const auto pub_acc = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_acc0, 1);
  const auto pub_gyr = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_gyr0, 1);
  const auto pub_imu = node->create_publisher<sensor_msgs::msg::Imu>(topic_imu0, 1);
  // clang-format on

  // Connect to device
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  if (devices.size() == 0) {
    FATAL("No device connected, please connect a RealSense device");
  }
  rs2::device device = devices[0];

  // Print device basic info
  // clang-format off
  printf("SDK version:      %s\n", RS2_API_FULL_VERSION_STR);
  printf("Device Name:      %s\n", device.get_info(RS2_CAMERA_INFO_NAME));
  printf("Serial Number:    %s\n", device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  printf("Firmware Version: %s\n", device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));
  printf("Physical Port:    %s\n", device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));
  fflush(stdout);
  // clang-format on

  // Configure stream
  rs2::config cfg;
  // -- Configure IR stereo cameras
  const int ir_width = 640;
  const int ir_height = 480;
  const auto ir_format = rs2_format_convert("Y8");
  const int ir_fps = 15;
  const double ir_exposure = 50000.0;

  // clang-format off
  rs2::sensor stereo;
  if (rs2_get_sensor(device, "Stereo Module", stereo) != 0) {
    FATAL("This RealSense device does not have a [Stereo Module]");
  }
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, ir_width, ir_height, ir_format, ir_fps);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, ir_width, ir_height, ir_format, ir_fps);
  stereo.set_option(RS2_OPTION_EXPOSURE, ir_exposure);
  stereo.set_option(RS2_OPTION_EMITTER_ENABLED, false);
  stereo.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);
  // clang-format on

  // -- Configure Motion Module
  const int accel_hz = 250;
  const int gyro_hz = 400;

  rs2::sensor motion;
  if (rs2_get_sensor(device, "Motion Module", motion) != 0) {
    FATAL("This RealSense device does not have a [Motion Module]");
  }
  motion.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, accel_hz);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, gyro_hz);

  // Create geometry_msgs::msg::Vector3 Message
  auto create_vec3_msg = [&](const rs2::motion_frame &f,
                             const std::string &frame_id) {
    // Form msg stamp
    const int64_t ts_ns = f.get_timestamp() * 1e6;
    rclcpp::Time msg_stamp{ts_ns};

    // Form msg header
    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp = msg_stamp;

    // Form msg
    const rs2_vector data = f.get_motion_data();
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header = header;
    msg.vector.x = data.x;
    msg.vector.y = data.y;
    msg.vector.z = data.z;

    return msg;
  };

  // Create Image message
  auto create_image_msg = [&](const rs2::video_frame &vf,
                              const std::string &frame_id) {
    // Form msg stamp
    const int64_t ts_ns = vframe2ts(vf, true);
    rclcpp::Time msg_stamp{ts_ns};

    // Form msg header
    std_msgs::msg::Header header = std_msgs::msg::Header();
    header.frame_id = frame_id;
    header.stamp = msg_stamp;

    // Image message
    const int width = vf.get_width();
    const int height = vf.get_height();
    const std::string encoding = "mono8";
    const cv::Mat cv_frame = frame2cvmat(vf, width, height, CV_8UC1);
    return cv_bridge::CvImage(std_msgs::msg::Header(), encoding, cv_frame)
        .toImageMsg();
  };

  // Create IMU message
  auto create_imu_msg = [&](const double ts_s,
                            const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &accel,
                            const std::string &frame_id) {
    sensor_msgs::msg::Imu msg;

    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Time{(int64_t) (ts_s * 1e9)};
    msg.angular_velocity.x = gyro.x();
    msg.angular_velocity.y = gyro.y();
    msg.angular_velocity.z = gyro.z();
    msg.linear_acceleration.x = accel.x();
    msg.linear_acceleration.y = accel.y();
    msg.linear_acceleration.z = accel.z();

    return msg;
  };

  // Publish IR frames
  auto publish_ir_msgs = [&](const rs2::video_frame &ir0,
                             const rs2::video_frame &ir1) {
    const auto cam0_msg = create_image_msg(ir0, "rs/ir0");
    const auto cam1_msg = create_image_msg(ir1, "rs/ir1");
    pub_ir0.publish(cam0_msg);
    pub_ir1.publish(cam1_msg);
  };

  // Publish Accelerometer message
  auto publish_accel_msg = [&](const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/accel0");
    pub_acc->publish(msg);
  };

  // Publish Gyroscope message
  auto publish_gyro_msg = [&](const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/gyro0");
    pub_gyr->publish(msg);
  };

  // Publish IMU message
  auto publish_imu_msg = [&](lerp_buf_t &buf) {
    while (buf.lerped_gyro_ts_.size()) {
      // Timestamp
      const auto ts = buf.lerped_gyro_ts_.front();
      buf.lerped_gyro_ts_.pop_front();
      buf.lerped_accel_ts_.pop_front();

      // Accel
      const auto accel = buf.lerped_accel_data_.front();
      buf.lerped_accel_data_.pop_front();

      // Gyro
      const auto gyro = buf.lerped_gyro_data_.front();
      buf.lerped_gyro_data_.pop_front();

      // Publish imu messages
      const auto msg = create_imu_msg(ts, gyro, accel, "rs/imu0");
      pub_imu->publish(msg);
    }

    buf.clear();
  };

  // Start pipeline
  rs2::pipeline pipe;
  lerp_buf_t lerp_buf;
  pipe.start(cfg, [&](const rs2::frame &frame) {
    // Handle motion frame
    if (auto mf = frame.as<rs2::motion_frame>()) {
      if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        publish_accel_msg(mf);

        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        lerp_buf.addAccel(ts_s, data.x, data.y, data.z);

      } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
        publish_gyro_msg(mf);

        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        lerp_buf.addGyro(ts_s, data.x, data.y, data.z);
      }

      if (lerp_buf.ready()) {
        lerp_buf.interpolate();
        publish_imu_msg(lerp_buf);
      }

      return;
    }

    // Stereo Module Callback
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      // Show rgbd ir image
      const auto ir_left = fs.get_infrared_frame(1);
      const auto ir_right = fs.get_infrared_frame(2);
      publish_ir_msgs(ir_left, ir_right);

      // Show stereo image
      // const int width = ir_left.get_width();
      // const int height = ir_left.get_height();
      // cv::Mat frame_left = frame2cvmat(ir_left, width, height, CV_8UC1);
      // cv::Mat frame_right = frame2cvmat(ir_right, width, height, CV_8UC1);
      // cv::Mat frame;
      // cv::hconcat(frame_left, frame_right, frame);
      // cv::namedWindow("Stereo Module", cv::WINDOW_AUTOSIZE);
      // cv::imshow("Stereo Module", frame);

      // User input
      // auto key = cv::waitKey(1);
      // if (key == 27 || key == 'q') {
      //   keep_running = false;
      // }
    }
  });

  // Block until frame counter threadhold is reached
  signal(SIGINT, signal_handler);
  while (keep_running) {
    sleep(0.1);
  }

  return 0;
}
