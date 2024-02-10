#include <chrono>
#include <iostream>
#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace px4_msgs::msg;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using Command = px4_msgs::msg::VehicleCommand;
using LocalPosition = px4_msgs::msg::VehicleLocalPosition;
using AttitudeSetpoint = px4_msgs::msg::VehicleAttitudeSetpoint;
using ThrustSetpoint = px4_msgs::msg::VehicleThrustSetpoint;

class OffboardControl : public rclcpp::Node {
private:
  const std::string topic_offboard_ = "/fmu/in/offboard_control_mode";
  const std::string topic_traj_ = "/fmu/in/trajectory_setpoint";
  const std::string topic_att_ = "/fmu/out/vehicle_attitude_setpoint";
  const std::string topic_thrust_ = "/fmu/out/vehicle_thrust_setpoint";
  const std::string topic_cmd_ = "/fmu/in/vehicle_command";
  const std::string topic_pos_ = "/fmu/out/vehicle_local_position";

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_traj_;
  rclcpp::Publisher<AttitudeSetpoint>::SharedPtr pub_att_;
  rclcpp::Publisher<ThrustSetpoint>::SharedPtr pub_thrust_;
  rclcpp::Publisher<Command>::SharedPtr pub_cmd_;
  rclcpp::Subscription<LocalPosition>::SharedPtr sub_pos_;

  uint64_t offboard_setpoint_counter_;
  uint64_t position_ts_ = 0;
  double position_[3] = {0.0, 0.0, 0.0};
  double heading_ = 0.0;

  void publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    pub_offboard_->publish(msg);
  }

  void publish_trajectory_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = 0.0 + M_PI / 2.0; // [-PI:PI]
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    pub_traj_->publish(msg);
  }

  void publish_attitude_setpoint() {
    const auto ts = get_clock()->now().nanoseconds() / 1000;

    AttitudeSetpoint att_msg{};
    att_msg.timestamp = ts + 100;
    // att_msg.roll_body = 0.0;
    // att_msg.pitch_body = 0.0;
    // att_msg.yaw_body = 0.0;
    // att_msg.yaw_sp_move_rate = 1.0; // rad/s (commanded by user)
    att_msg.thrust_body[0] = 0.0;
    att_msg.thrust_body[1] = 0.0;
    att_msg.thrust_body[2] = -1.0;
    att_msg.q_d[0] = 1.0;
    att_msg.q_d[1] = 0.0;
    att_msg.q_d[2] = 0.0;
    att_msg.q_d[3] = 0.0;
    att_msg.reset_integral = false;
    att_msg.fw_control_yaw_wheel = false;
    pub_att_->publish(att_msg);

    // ThrustSetpoint thrust_msg{};
    // thrust_msg.timestamp = ts + 100;
    // thrust_msg.timestamp_sample = ts;
    // thrust_msg.xyz[0] = 0.0;
    // thrust_msg.xyz[1] = 0.0;
    // thrust_msg.xyz[2] = 1.0;
    // pub_thrust_->publish(thrust_msg);
  }

  void publish_vehicle_command(uint16_t command,
                               float param1 = 0.0,
                               float param2 = 0.0) {
    Command msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    pub_cmd_->publish(msg);
  }

  void pos_callback(const LocalPosition::SharedPtr msg) {
    // Convert NED to ENU
    //  y -> x
    //  x -> y
    // -z -> z
    const double x = msg->y;
    const double y = msg->x;
    const double z = -1.0 * msg->z;
    const double heading = msg->heading - M_PI / 2.0;

    // RCLCPP_INFO(get_logger(),
    //             "x: %.2f, y: %.2f, z: %.2f, heading: %.2f",
    //             x,
    //             y,
    //             z,
    //             heading);

    position_ts_ = 0;
    position_[0] = x;
    position_[1] = y;
    position_[2] = z;
    heading_ = heading;
  }

public:
  OffboardControl() : Node("offboard_control") {
    // ROS topics and callbacks
    auto pos_cb = std::bind(&OffboardControl::pos_callback, this, _1);

    // Setup Publishers and Subscribers
    const rmw_qos_profile_t qos_prof = rmw_qos_profile_sensor_data;
    const auto qos_init = rclcpp::QoSInitialization(qos_prof.history, 5);
    const auto qos = rclcpp::QoS(qos_init, qos_prof);
    pub_offboard_ = create_publisher<OffboardControlMode>(topic_offboard_, 10);
    pub_traj_ = create_publisher<TrajectorySetpoint>(topic_traj_, 10);
    pub_att_ = create_publisher<AttitudeSetpoint>(topic_att_, 10);
    pub_thrust_ = create_publisher<ThrustSetpoint>(topic_thrust_, 10);
    pub_cmd_ = create_publisher<Command>(topic_cmd_, 10);
    sub_pos_ = create_subscription<LocalPosition>(topic_pos_, qos, pos_cb);
    offboard_setpoint_counter_ = 0;

    sleep(5);
    publish_vehicle_command(Command::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    arm();

    // Timer
    auto timer_callback = [this]() -> void {
      if (offboard_setpoint_counter_ == 1000) {
        publish_vehicle_command(Command::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
        publish_offboard_control_mode();
      }

      if (offboard_setpoint_counter_ > 1000) {
        // publish_trajectory_setpoint();
        publish_attitude_setpoint();
      }

      // Stop the counter after reaching 11
      if (offboard_setpoint_counter_ < 1001) {
        offboard_setpoint_counter_++;
      }
    };
    timer_ = create_wall_timer(1ms, timer_callback);
  }

  void arm() {
    const auto cmd = Command::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    const auto val = 1.0;
    publish_vehicle_command(cmd, val);
    RCLCPP_INFO(get_logger(), "Arming MAV ...");
  }

  void disarm() {
    const auto cmd = Command::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    const auto val = 0.0;
    publish_vehicle_command(cmd, val);
    RCLCPP_INFO(get_logger(), "Disarming MAV ...");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
