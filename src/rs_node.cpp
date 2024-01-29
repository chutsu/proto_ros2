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

// Run Intel D435i Node
int main(int argc, char *argv[]) {
  // Initialize ROS node
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  const auto node = rclcpp::Node::make_shared("rs_node", options);
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
  rs2_list_devices();
  RCLCPP_INFO(node->get_logger(), "Connecting to RealSense Device");
  rs_d435i_t device{0, false, true};

  // -- Register image callback
  device.image_callback = [&](const rs2::video_frame &ir0,
                              const rs2::video_frame &ir1) {
    const auto cam0_msg = create_image_msg(ir0, "rs/ir0");
    const auto cam1_msg = create_image_msg(ir1, "rs/ir1");
    pub_ir0.publish(cam0_msg);
    pub_ir1.publish(cam1_msg);
  };

  // -- Register accelerometer callback
  device.accel_callback = [&](const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/accel0");
    pub_acc->publish(msg);
  };

  // -- Register gyroscope callback
  device.gyro_callback = [&](const rs2::motion_frame &mf) {
    const auto msg = create_vec3_msg(mf, "rs/gyro0");
    pub_gyr->publish(msg);
  };

  // -- Register IMU callback
  device.imu_callback = [&](lerp_buf_t &buf) {
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
  RCLCPP_INFO(node->get_logger(), "Streaming ...");
  device.loop();

  return 0;
}
