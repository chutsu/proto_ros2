#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <okvis/ThreadedSlam.hpp>

// Aliases
using namespace std::placeholders;
using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

// Global Variables
bool debug = false;
std::unique_ptr<okvis::ThreadedSlam> slam;

// IMU Callback
void imu_callback(const ImuMsg::SharedPtr msg) {
  // Timestamp
  const uint32_t sec = msg->header.stamp.sec;
  const uint32_t nsec = msg->header.stamp.nanosec;
  const okvis::Time ts{sec, nsec};

  // Accelerometer measurement
  const double acc_x = msg->linear_acceleration.x;
  const double acc_y = msg->linear_acceleration.y;
  const double acc_z = msg->linear_acceleration.z;
  const Eigen::Vector3d acc{acc_x, acc_y, acc_z};

  // Gyroscope measurement
  const double gyr_x = msg->angular_velocity.x;
  const double gyr_y = msg->angular_velocity.y;
  const double gyr_z = msg->angular_velocity.z;
  const Eigen::Vector3d gyr{gyr_x, gyr_y, gyr_z};

  // Add IMU Measurement
  // slam->addImuMeasurement(ts, acc, gyr);

  // Debug
  if (debug) {
    printf("acc: (%.2f, %.2f, %.2f)\n",
           msg->linear_acceleration.x,
           msg->linear_acceleration.y,
           msg->linear_acceleration.z);
    printf("gyr: (%.2f, %.2f, %.2f)\n",
           msg->angular_velocity.x,
           msg->angular_velocity.y,
           msg->angular_velocity.z);
  }
}

// Image Callback
void image_callback(const ImageMsg::ConstSharedPtr &cam0_msg,
                    const ImageMsg::ConstSharedPtr &cam1_msg) {
  const auto image_encoding = sensor_msgs::image_encodings::MONO8;
  const auto cam0_frame = cv_bridge::toCvCopy(cam0_msg, image_encoding);
  const auto cam1_frame = cv_bridge::toCvCopy(cam1_msg, image_encoding);
  const cv::Mat image0 = cam0_frame->image;
  const cv::Mat image1 = cam1_frame->image;

  // Debug
  if (debug) {
    cv::Mat viz;
    cv::hconcat(image0, image1, viz);
    cv::imshow("viz", viz);
    cv::waitKey(1);
  }
}

// Run ROS Node
int main(int argc, char *argv[]) {
  // Read Parameters
  // std::string config_path = "/home/chutsu/config_realsense_D435i_Chris.yaml";
  // okvis::ViParametersReader paramReader(config_path);
  // okvis::ViParameters params;
  // paramReader.getParameters(params);
  // slam = std::make_unique<okvis::ThreadedSlam>() slam.setBlocking(false);

  // Setup ROS node
  // clang-format off
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("okvis_node");
  auto sub_imu0 = nh->create_subscription<ImuMsg>("/rs/imu0/data", 10, imu_callback);
  message_filters::Subscriber<ImageMsg> sub_cam0(nh.get(), "/rs/ir0/image");
  message_filters::Subscriber<ImageMsg> sub_cam1(nh.get(), "/rs/ir1/image");
  message_filters::TimeSynchronizer<ImageMsg, ImageMsg> sync(sub_cam0, sub_cam1, 1);
  sync.registerCallback(std::bind(&image_callback, _1, _2));
  // clang-format on

  while (rclcpp::ok()) {
    // slam.processFrame();
    // slam.display();
    rclcpp::spin_some(nh);
  }

  return 0;
}
