#include <memory>
#include <complex>

#include <gz/math/Pose3.hh>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/config.hh>

#include <gz/msgs/Utility.hh>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include <proto.h>

class MavPlugin : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate,
                  public gz::sim::ISystemPostUpdate {
private:
  // Gazebo topics
  std::string twist_state_topic_ = "/mav/twist/state";
  std::string pose_state_topic_ = "/mav/pose/state";
  std::string setpoint_position_topic_ = "/mav/setpoint_position/cmd";

  // Fields
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher twist_pub_;
  gz::transport::Node::Publisher pose_pub_;

  // Model state
  mav_model_t mav_;
  mav_att_ctrl_t mav_att_ctrl_;
  mav_pos_ctrl_t mav_pos_ctrl_;
  gz::math::Vector3d position_{0.0, 0.0, 0.0};
  gz::math::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  gz::math::Vector3d linear_velocity_{0.0, 0.0, 0.0};
  gz::math::Vector3d angular_velocity_{0.0, 0.0, 0.0};
  gz::math::Vector3d setpoint_position_{0.0, 0.0, 3.0};

  void SetupMAV() {
    // clang-format off
    const real_t x[12] = {
      // Attitude [rad]
      0.0, 0.0, 0.0,
      // Angular Velocity [rad / s]
      0.0, 0.0, 0.0,
      // Position [m]
      0.0, 0.0, 0.0,
      // Linear velocity [m / s]
      0.0, 0.0, 0.0
    };
    // clang-format on
    const real_t inertia[3] = {0.0963, 0.0963, 0.1927}; // Moment of inertia
    const real_t kr = 0.1; // Rotation drag constant
    const real_t kt = 0.2; // Translation drag constant
    const real_t l = 0.9;  // Arm Length
    const real_t d = 1.0;  // Drag constant
    const real_t m = 1.0;  // Mass
    const real_t g = 9.81; // Gravitational constant
    mav_model_setup(&mav_, x, inertia, kr, kt, l, d, m, g);
    mav_att_ctrl_setup(&mav_att_ctrl_);
    mav_pos_ctrl_setup(&mav_pos_ctrl_);
  }

  /** Update MAV **/
  void UpdateMAV(gz::sim::EntityComponentManager &ecm) {
    // Update MAV motion model
    const real_t dt = 0.001;
    const real_t pos_sp[3] = {setpoint_position_.X(),
                              setpoint_position_.Y(),
                              setpoint_position_.Z()};
    const real_t pos_pv[4] = {mav_.x[6], mav_.x[7], mav_.x[8], mav_.x[2]};
    const real_t att_pv[3] = {mav_.x[0], mav_.x[1], mav_.x[2]};

    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_pos_ctrl_update(&mav_pos_ctrl_, pos_sp, pos_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl_, att_sp, att_pv, dt, u);
    mav_model_update(&mav_, u, dt);

    real_t ypr[3] = {mav_.x[2], mav_.x[1], mav_.x[0]};
    real_t quat[4] = {0};
    euler2quat(ypr, quat);

    // Set MAV state to Gazebo containers
    orientation_.W() = quat[0];
    orientation_.X() = quat[1];
    orientation_.Y() = quat[2];
    orientation_.Z() = quat[3];

    angular_velocity_.X() = mav_.x[3];
    angular_velocity_.Y() = mav_.x[4];
    angular_velocity_.Z() = mav_.x[5];

    position_.X() = mav_.x[6];
    position_.Y() = mav_.x[7];
    position_.Z() = mav_.x[8];

    linear_velocity_.X() = mav_.x[9];
    linear_velocity_.Y() = mav_.x[10];
    linear_velocity_.Z() = mav_.x[11];

    UpdatePose(ecm);
    UpdateTwist(ecm);
  }

  /** Update model pose **/
  void UpdatePose(gz::sim::EntityComponentManager &ecm) {
    gz::math::Pose3d pose{position_, orientation_};
    model_.SetWorldPoseCmd(ecm, pose);
  }

  /** Update model twist **/
  void UpdateTwist(gz::sim::EntityComponentManager &ecm) {
    // Update linear velocity
    auto lvel_cmd = gz::sim::components::LinearVelocityCmd(linear_velocity_);
    auto lvel = ecm.Component<gz::sim::components::LinearVelocityCmd>(entity_);
    if (lvel == nullptr) {
      ecm.CreateComponent(entity_, lvel_cmd);
    } else {
      *lvel = lvel_cmd;
    }

    // Update angular velocity
    auto avel_cmd = gz::sim::components::AngularVelocityCmd(angular_velocity_);
    auto avel = ecm.Component<gz::sim::components::AngularVelocityCmd>(entity_);
    if (avel == nullptr) {
      ecm.CreateComponent(entity_, avel_cmd);
    } else {
      *avel = avel_cmd;
    }
  }

  /** Setpoint position message callback **/
  void SetpointPositionCallback(const gz::msgs::Vector3d &msg) {
    setpoint_position_ = gz::msgs::Convert(msg);
  }

public:
  /** Constructors and Destructors **/
  MavPlugin() = default;
  virtual ~MavPlugin() = default;

  /** Configure plugin **/
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override {
    // Entity and model
    entity_ = entity;
    model_ = gz::sim::Model(entity);
    SetupMAV();

    // Publishers and subscribers
    twist_pub_ = node_.Advertise<gz::msgs::Twist>(twist_state_topic_);
    pose_pub_ = node_.Advertise<gz::msgs::Pose>(pose_state_topic_);
    node_.Subscribe(setpoint_position_topic_,
                    &MavPlugin::SetpointPositionCallback,
                    this);
  }

  /** Plugin Pre-Update **/
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    UpdateMAV(ecm);
  }

  /** Plugin Pose-Update **/
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    // Message timestamp
    const gz::msgs::Time stamp = gz::msgs::Convert(info.simTime);

    // Publish twist message
    gz::msgs::Twist twist_msg;
    twist_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    gz::msgs::Set(twist_msg.mutable_linear(), linear_velocity_);
    gz::msgs::Set(twist_msg.mutable_angular(), angular_velocity_);
    twist_pub_.Publish(twist_msg);

    // Publish pose message
    gz::msgs::Pose pose_msg;
    pose_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    gz::msgs::Set(pose_msg.mutable_position(), position_);
    gz::msgs::Set(pose_msg.mutable_orientation(), orientation_);
    pose_pub_.Publish(pose_msg);
  }
};

// Register plugin
GZ_ADD_PLUGIN(MavPlugin,
              gz::sim::System,
              MavPlugin::ISystemConfigure,
              MavPlugin::ISystemPreUpdate,
              MavPlugin::ISystemPostUpdate)
