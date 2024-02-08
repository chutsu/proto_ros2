#include <chrono>
#include <iostream>
#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;

  // clang-format off
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  std::atomic<uint64_t> timestamp_;
  uint64_t offboard_setpoint_counter_;
  // clang-format on

  void publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }

  void publish_trajectory_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command,
                               float param1 = 0.0,
                               float param2 = 0.0) {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }

public:
  OffboardControl() : Node("offboard_control") {
    // clang-format off
    offboard_control_mode_publisher_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    offboard_setpoint_counter_ = 0;
    // clang-format on

    auto timer_callback = [this]() -> void {
      if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        arm();
      }

      // offboard_control_mode needs to be paired with trajectory_setpoint
      publish_offboard_control_mode();
      publish_trajectory_setpoint();

      // stop the counter after reaching 11
      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
    };
    timer_ = create_wall_timer(100ms, timer_callback);
  }

  void arm() {
    // clang-format off
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(get_logger(), "Arm command send");
    // clang-format on
  }

  void disarm() {
    // clang-format off
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(get_logger(), "Disarm command send");
    // clang-format on
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
