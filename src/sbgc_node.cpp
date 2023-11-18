#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define SBGC_IMPLEMENTATION
#include <sbgc.h>
#define SBGC_DEV "/dev/ttyUSB0"
#define Vec3Msg geometry_msgs::msg::Vector3

using namespace std::chrono_literals;
using std::placeholders::_1;

struct sbgc_node_t : public rclcpp::Node {
  std::mutex mtx;
  sbgc_t sbgc;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<Vec3Msg>::SharedPtr pub_joints;
  rclcpp::Subscription<Vec3Msg>::SharedPtr sub_joints;

  sbgc_node_t() : Node("sbgc_node") {
    // Connect to SBGC
    sbgc_connect(&sbgc, SBGC_DEV);
    if (sbgc_on(&sbgc) != 0) {
      RCLCPP_FATAL(get_logger(), "Failed to connect to SBGC!");
      exit(-1);
    }

    // Publishers
    pub_joints = create_publisher<Vec3Msg>("/sbgc/joints", 1);

    // Subscribers
    auto joints_cb = std::bind(&sbgc_node_t::joints_callback, this, _1);
    sub_joints =
        create_subscription<Vec3Msg>("/sbgc/set_joints", 10, joints_cb);

    // Callback
    auto timer_func = std::bind(&sbgc_node_t::timer_update, this);
    timer = create_wall_timer(1ms, timer_func);
  }

  ~sbgc_node_t() {
    sbgc_off(&sbgc);
  }

  void timer_update() {
    std::lock_guard<std::mutex> guard(mtx);
    if (sbgc_update(&sbgc) != 0) {
      return;
    }

    auto msg = Vec3Msg();
    msg.x = sbgc.data.camera_angles[0];
    msg.y = sbgc.data.camera_angles[1];
    msg.z = sbgc.data.camera_angles[2];

    pub_joints->publish(msg);
  }

  void joints_callback(const Vec3Msg::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(mtx);
    const float roll = msg->x;
    const float pitch = msg->y;
    const float yaw = msg->z;

    printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);
    sbgc_set_angle(&sbgc, roll, pitch, yaw);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sbgc_node_t>());
  rclcpp::shutdown();
  return 0;
}
