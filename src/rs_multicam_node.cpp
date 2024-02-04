#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "realsense.hpp"

// Create Vector3 Message
geometry_msgs::msg::Vector3Stamped
create_vec3_msg(const rs2::motion_frame &f, const std::string &frame_id) {
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
}

// Create Image Message
sensor_msgs::msg::Image::SharedPtr
create_image_msg(const int64_t ts_ns,
                 const rs2::video_frame &vf,
                 const std::string &frame_id) {
  // Form msg stamp
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
  return cv_bridge::CvImage(header, encoding, cv_frame).toImageMsg();
}

// Create IMU message
sensor_msgs::msg::Imu create_imu_msg(const double ts_s,
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
}

// Run Multi Intel D435i Node
int main(int argc, char *argv[]) {
  // Initialize ROS node
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  const auto node = rclcpp::Node::make_shared("rs_multicam_node", options);
  const std::string topic_ir0 = "/rs/ir0/image";
  const std::string topic_ir1 = "/rs/ir1/image";
  const std::string topic_ir2 = "/rs1/ir2/image";
  const std::string topic_ir3 = "/rs1/ir3/image";
  const std::string topic_acc0 = "/rs/acc0/raw";
  const std::string topic_gyr0 = "/rs/gyr0/raw";
  const std::string topic_imu0 = "/rs/imu0/data";
  const std::string topic_acc1 = "/rs/acc1/raw";
  const std::string topic_gyr1 = "/rs/gyr1/raw";
  const std::string topic_imu1 = "/rs/imu1/data";

  // clang-format off
  image_transport::ImageTransport it(node);
  const auto pub_ir0 = it.advertise(topic_ir0, 1);
  const auto pub_ir1 = it.advertise(topic_ir1, 1);
  const auto pub_ir2 = it.advertise(topic_ir2, 1);
  const auto pub_ir3 = it.advertise(topic_ir3, 1);
  const auto pub_acc0 = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_acc0, 1);
  const auto pub_gyr0 = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_gyr0, 1);
  const auto pub_imu0 = node->create_publisher<sensor_msgs::msg::Imu>(topic_imu0, 1);
  const auto pub_acc1 = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_acc1, 1);
  const auto pub_gyr1 = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_gyr1, 1);
  const auto pub_imu1 = node->create_publisher<sensor_msgs::msg::Imu>(topic_imu1, 1);
  // clang-format on

  // Connect to device
  rs2_list_devices();
  RCLCPP_INFO(node->get_logger(), "Connecting to RealSense Device");
  rs_multi_d435i_t devices;

  const std::string serial0 = "843112071984";
  const std::string serial1 = "943222072527";
  const auto cfg0 = devices.configs[serial0];
  const auto cfg1 = devices.configs[serial1];

  // Start pipeline
  lerp_buf_t buf0;
  lerp_buf_t buf1;
  rs2::frame ir0_frame;
  rs2::frame ir1_frame;
  rs2::frame ir2_frame;
  rs2::frame ir3_frame;

  devices.pipelines[serial0].start(cfg0, [&](const rs2::frame &frame) {
    // Handle motion frame
    if (auto mf = frame.as<rs2::motion_frame>()) {
      if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        // Accelerometer measurement
        const auto msg = create_vec3_msg(mf, "rs/accel0");
        pub_acc0->publish(msg);

        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        buf0.addAccel(ts_s, data.x, data.y, data.z);

      } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
        // Gyroscope measurement
        const auto msg = create_vec3_msg(mf, "rs/gyro0");
        pub_gyr0->publish(msg);

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

          // Publish imu messages
          const auto msg = create_imu_msg(ts, gyro, accel, "rs/imu0");
          pub_imu0->publish(msg);
        }

        buf0.clear();
      }

      return;
    }

    // Stereo Module Callback
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      ir0_frame = fs.get_infrared_frame(1);
      ir1_frame = fs.get_infrared_frame(2);
    }
  });

  // Start pipeline
  devices.pipelines[serial1].start(cfg1, [&](const rs2::frame &frame) {
    // Handle motion frame
    if (auto mf = frame.as<rs2::motion_frame>()) {
      if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        // Accelerometer measurement
        const auto msg = create_vec3_msg(mf, "rs/accel1");
        pub_acc1->publish(msg);

        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        buf1.addAccel(ts_s, data.x, data.y, data.z);

      } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
        // Gyroscope measurement
        const auto msg = create_vec3_msg(mf, "rs/gyro1");
        pub_gyr1->publish(msg);

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

          // Publish imu messages
          const auto msg = create_imu_msg(ts, gyro, accel, "rs/imu1");
          pub_imu1->publish(msg);
        }

        buf1.clear();
      }

      return;
    }

    // Stereo Module Callback
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      ir2_frame = fs.get_infrared_frame(1);
      ir3_frame = fs.get_infrared_frame(2);
    }
  });

  // Block until frame counter threadhold is reached
  RCLCPP_INFO(node->get_logger(), "Streaming ...");
  signal(SIGINT, realsense_signal_handler);
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
      const int64_t ts = vframe2ts(ir0_frame, true);
      const auto cam0_msg = create_image_msg(ts, ir0_frame, "rs/ir0");
      const auto cam1_msg = create_image_msg(ts, ir1_frame, "rs/ir1");
      const auto cam2_msg = create_image_msg(ts, ir2_frame, "rs/ir2");
      const auto cam3_msg = create_image_msg(ts, ir3_frame, "rs/ir3");
      pub_ir0.publish(cam0_msg);
      pub_ir1.publish(cam1_msg);
      pub_ir2.publish(cam2_msg);
      pub_ir3.publish(cam3_msg);
      last_ts = ts;
    }
  }

  return 0;
}
