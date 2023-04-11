#include <memory>
#include <complex>

#include <gz/math/Pose3.hh>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/config.hh>

#include <gz/msgs/Utility.hh>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include <proto.h>

std::string parseString(const std::shared_ptr<const sdf::Element> &sdf,
                        const std::string &element_name) {
  if (sdf->HasElement(element_name)) {
    return sdf->Get<std::string>(element_name);
  } else {
    gzerr << "Failed to parse [" << element_name << "]!";
    exit(-1);
  }
}

class MavPositionControllerPlugin : public gz::sim::System,
                                    public gz::sim::ISystemConfigure,
                                    public gz::sim::ISystemPreUpdate,
                                    public gz::sim::ISystemPostUpdate {
private:
  // Gazebo topics
  std::string position_cmd_topic_;
  std::string yaw_cmd_topic_;
  std::string twist_cmd_topic_;

  // Fields
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher twist_pub_;

  gz::math::Pose3d pose_;
  gz::math::Vector3d position_setpoint_{0.0, 0.0, 3.0};
  double yaw_setpoint_ = 0.0;

  gz::math::Vector3d linear_velocity_cmd_{0.0, 0.0, 0.0};
  gz::math::Vector3d angular_velocity_cmd_{0.0, 0.0, 0.0};

  double dt_ = 0.0;

  double pos_err_x_sum_ = 0.0;
  double pos_err_x_prev_ = 0.0;
  double vx_kp_ = 1.0;
  double vx_ki_ = 0.0;
  double vx_kd_ = 0.8;

  double pos_err_y_sum_ = 0.0;
  double pos_err_y_prev_ = 0.0;
  double vy_kp_ = 1.0;
  double vy_ki_ = 0.0;
  double vy_kd_ = 0.8;

  double pos_err_z_sum_ = 0.0;
  double pos_err_z_prev_ = 0.0;
  double vz_kp_ = 100.0;
  double vz_ki_ = 1.0;
  double vz_kd_ = 60.0;

  double err_yaw_sum_ = 0.0;
  double err_yaw_prev_ = 0.0;
  double wz_kp_ = 1.0;
  double wz_ki_ = 0.0;
  double wz_kd_ = 1.0;

  double vx_limits_[2] = {-5.0, 5.0};
  double vy_limits_[2] = {-5.0, 5.0};
  double vz_limits_[2] = {-10.0, 10.0};
  double wz_limits_[2] = {-10.0, 10.0};

  void CalculateLinearVelocityCommands() {
    // Calculate position error
    auto pos_sp = position_setpoint_;              // Setpoint
    auto pos_pv = pose_.Pos();                     // Process Variable (actual)
    auto q_BW = pose_.Rot().Inverse();             // Rotation to body frame
    auto pos_err_W = pos_sp - pos_pv;              // Error in world frame
    auto pos_err_B = q_BW.RotateVector(pos_err_W); // Error in body frame

    // X-axis command
    double vx_cmd = 0.0;
    vx_cmd = vx_kp_ * pos_err_B.X();
    vx_cmd += vx_ki_ * pos_err_x_sum_;
    vx_cmd += vx_kd_ * (pos_err_B.X() - pos_err_x_prev_) / dt_;
    pos_err_x_sum_ += pos_err_B.X() * dt_;
    pos_err_x_prev_ = pos_err_B.X();

    if (vx_cmd < vx_limits_[0]) {
      vx_cmd = vx_limits_[0];
    } else if (vx_cmd > vx_limits_[1]) {
      vx_cmd = vx_limits_[1];
    }

    // Y-axis command
    double vy_cmd = 0.0;
    vy_cmd = vy_kp_ * pos_err_B.Y();
    vy_cmd += vy_ki_ * pos_err_y_sum_;
    vy_cmd += vy_kd_ * (pos_err_B.Y() - pos_err_y_prev_) / dt_;
    pos_err_y_sum_ += pos_err_B.Y() * dt_;
    pos_err_y_prev_ = pos_err_B.Y();

    if (vy_cmd < vy_limits_[0]) {
      vy_cmd = vy_limits_[0];
    } else if (vy_cmd > vy_limits_[1]) {
      vy_cmd = vy_limits_[1];
    }

    // Z-axis command
    double vz_cmd = 0.0;
    vz_cmd = vz_kp_ * pos_err_B.Z();
    vz_cmd += vz_ki_ * pos_err_z_sum_;
    vz_cmd += vz_kd_ * (pos_err_B.Z() - pos_err_z_prev_) / dt_;
    pos_err_z_sum_ += pos_err_B.Z() * dt_;
    pos_err_z_prev_ = pos_err_B.Z();

    if (vz_cmd < vz_limits_[0]) {
      vz_cmd = vz_limits_[0];
    } else if (vz_cmd > vz_limits_[1]) {
      vz_cmd = vz_limits_[1];
    }

    // Set linear velocity command
    linear_velocity_cmd_.X() = vx_cmd;
    linear_velocity_cmd_.Y() = vy_cmd;
    linear_velocity_cmd_.Z() = vz_cmd;
  }

  void CalculateAngularVelocityCommands() {
    // Calculate yaw error
    auto yaw_sp = yaw_setpoint_;     // Setpoint
    auto yaw_pv = pose_.Rot().Yaw(); // Process Variable (actual)
    auto yaw_err = yaw_sp - yaw_pv;  // Error in world frame
                                     //
    // Rotation Z-axis command
    double wz_cmd = 0.0;
    wz_cmd = wz_kp_ * yaw_err;
    wz_cmd += wz_ki_ * err_yaw_sum_;
    wz_cmd += wz_kd_ * (yaw_err - err_yaw_prev_) / dt_;
    err_yaw_sum_ += yaw_err * dt_;
    err_yaw_prev_ = yaw_err;

    if (wz_cmd < wz_limits_[0]) {
      wz_cmd = wz_limits_[0];
    } else if (wz_cmd > wz_limits_[1]) {
      wz_cmd = wz_limits_[1];
    }

    // Set angular velocity command
    angular_velocity_cmd_.X() = 0.0;
    angular_velocity_cmd_.Y() = 0.0;
    angular_velocity_cmd_.Z() = wz_cmd;
  }

  /** Position setpoint message callback **/
  void PositionSetpointCallback(const gz::msgs::Vector3d &msg) {
    position_setpoint_ = gz::msgs::Convert(msg);
  }

  /** Yaw setpoint message callback **/
  void YawSetpointCallback(const gz::msgs::Double &msg) {
    yaw_setpoint_ = gz::msgs::Convert(msg);
  }

public:
  /** Constructors and Destructors **/
  MavPositionControllerPlugin() = default;
  virtual ~MavPositionControllerPlugin() = default;

  /** Configure plugin **/
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override {
    // Entity and model
    entity_ = entity;
    model_ = gz::sim::Model(entity);

    // Parse settings from SDF file
    // clang-format off
    position_cmd_topic_ = parseString(sdf, "positionCommandTopic");
    yaw_cmd_topic_ = parseString(sdf, "yawCommandTopic");
    twist_cmd_topic_ = parseString(sdf, "twistCommandTopic");
    // clang-format on

    // Publishers and subscribers
    twist_pub_ = node_.Advertise<gz::msgs::Twist>(twist_cmd_topic_);
    node_.Subscribe(position_cmd_topic_,
                    &MavPositionControllerPlugin::PositionSetpointCallback,
                    this);
    node_.Subscribe(yaw_cmd_topic_,
                    &MavPositionControllerPlugin::YawSetpointCallback,
                    this);
  }

  /** Plugin Pre-Update **/
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    // Simulation paused?
    if (info.paused) {
      return;
    }

    // Limit position controller rate
    dt_ += std::chrono::duration<double>(info.dt).count();
    if (dt_ < 0.01) {
      return;
    }

    // Get Model pose
    auto comp = ecm.Component<gz::sim::components::Pose>(entity_);
    if (comp == nullptr) {
      ecm.CreateComponent(entity_, gz::sim::components::Pose());
      return;
    }
    pose_ = comp->Data();

    // Calculate linear and angular velocity commands
    CalculateLinearVelocityCommands();
    CalculateAngularVelocityCommands();

    // Update
    dt_ = 0.0;
  }

  /** Plugin Pose-Update **/
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    // Message timestamp
    const gz::msgs::Time stamp = gz::msgs::Convert(info.simTime);

    // Publish twist message
    gz::msgs::Twist twist_msg;
    twist_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    gz::msgs::Set(twist_msg.mutable_linear(), linear_velocity_cmd_);
    gz::msgs::Set(twist_msg.mutable_angular(), angular_velocity_cmd_);
    twist_pub_.Publish(twist_msg);
  }
};

// Register plugin
GZ_ADD_PLUGIN(MavPositionControllerPlugin,
              gz::sim::System,
              MavPositionControllerPlugin::ISystemConfigure,
              MavPositionControllerPlugin::ISystemPreUpdate,
              MavPositionControllerPlugin::ISystemPostUpdate)