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
create_image_msg(const rs2::video_frame &vf, const std::string &frame_id) {
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
  const auto node = rclcpp::Node::make_shared("rs_multi_node", options);
  const std::string topic_dev0_ir0 = "/rs0/ir0/image";
  const std::string topic_dev0_ir1 = "/rs0/ir1/image";
  const std::string topic_dev0_acc0 = "/rs0/acc0/raw";
  const std::string topic_dev0_gyr0 = "/rs0/gyr0/raw";
  const std::string topic_dev0_imu0 = "/rs0/imu0/data";
  const std::string topic_dev1_ir0 = "/rs1/ir0/image";
  const std::string topic_dev1_ir1 = "/rs1/ir1/image";

  // clang-format off
  image_transport::ImageTransport it(node);
  const auto pub_dev0_ir0 = it.advertise(topic_dev0_ir0, 1);
  const auto pub_dev0_ir1 = it.advertise(topic_dev0_ir1, 1);
  const auto pub_dev0_acc = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_dev0_acc0, 1);
  const auto pub_dev0_gyr = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_dev0_gyr0, 1);
  const auto pub_dev0_imu = node->create_publisher<sensor_msgs::msg::Imu>(topic_dev0_imu0, 1);
  const auto pub_dev1_ir0 = it.advertise(topic_dev1_ir0, 1);
  const auto pub_dev1_ir1 = it.advertise(topic_dev1_ir1, 1);
  // clang-format on

  // Connect to device
  rs2_list_devices();
  RCLCPP_INFO(node->get_logger(), "Connecting to RealSense Device");
  rs_multi_d435i_t devices{false, true};

  const std::string serial0 = "843112071984";
  const std::string serial1 = "943222072527";
  const auto cfg0 = devices.configs[serial0];
  const auto cfg1 = devices.configs[serial1];

  // Start pipeline
  lerp_buf_t lerp_buf;
  devices.pipelines[serial0].start(cfg0, [&](const rs2::frame &frame) {
    // Handle motion frame
    if (auto mf = frame.as<rs2::motion_frame>()) {
      if (mf && mf.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        // Accelerometer measurement
        const auto msg = create_vec3_msg(mf, "rs0/accel0");
        pub_dev0_acc->publish(msg);

        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        lerp_buf.addAccel(ts_s, data.x, data.y, data.z);

      } else if (mf && mf.get_profile().stream_type() == RS2_STREAM_GYRO) {
        // Gyroscope measurement
        const auto msg = create_vec3_msg(mf, "rs0/gyro0");
        pub_dev0_gyr->publish(msg);

        double ts_s = mf.get_timestamp() * 1e-3;
        const rs2_vector data = mf.get_motion_data();
        lerp_buf.addGyro(ts_s, data.x, data.y, data.z);
      }

      if (lerp_buf.ready()) {
        // IMU measurement
        lerp_buf.interpolate();
        while (lerp_buf.lerped_gyro_ts_.size()) {
          // Timestamp
          const auto ts = lerp_buf.lerped_gyro_ts_.front();
          lerp_buf.lerped_gyro_ts_.pop_front();
          lerp_buf.lerped_accel_ts_.pop_front();

          // Accel
          const auto accel = lerp_buf.lerped_accel_data_.front();
          lerp_buf.lerped_accel_data_.pop_front();

          // Gyro
          const auto gyro = lerp_buf.lerped_gyro_data_.front();
          lerp_buf.lerped_gyro_data_.pop_front();

          // Publish imu messages
          const auto msg = create_imu_msg(ts, gyro, accel, "rs0/imu0");
          pub_dev0_imu->publish(msg);
        }

        lerp_buf.clear();
      }

      return;
    }

    // Stereo Module Callback
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      const auto ir0 = fs.get_infrared_frame(1);
      const auto ir1 = fs.get_infrared_frame(2);
      const auto cam0_msg = create_image_msg(ir0, "rs0/ir0");
      const auto cam1_msg = create_image_msg(ir1, "rs0/ir1");
      pub_dev0_ir0.publish(cam0_msg);
      pub_dev0_ir1.publish(cam1_msg);
    }
  });

  // Start pipeline
  devices.pipelines[serial1].start(cfg1, [&](const rs2::frame &frame) {
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      const auto ir0 = fs.get_infrared_frame(1);
      const auto ir1 = fs.get_infrared_frame(2);
      const auto cam0_msg = create_image_msg(ir0, "rs1/ir0");
      const auto cam1_msg = create_image_msg(ir1, "rs1/ir1");
      pub_dev1_ir0.publish(cam0_msg);
      pub_dev1_ir1.publish(cam1_msg);
    }
  });

  // Block until frame counter threadhold is reached
  // RCLCPP_INFO(node->get_logger(), "Streaming ...");
  signal(SIGINT, realsense_signal_handler);
  while (realsense_keep_running) {
    sleep(0.1);
  }

  return 0;
}
