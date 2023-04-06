#include <memory>

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
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

class ModelPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate,
                    public gz::sim::ISystemPostUpdate {
private:
  // Gazebo topics
  std::string twist_cmd_topic_ = "/model/twist/cmd";
  std::string pose_cmd_topic_ = "/model/pose/cmd";
  std::string twist_state_topic_ = "/model/twist/state";
  std::string pose_state_topic_ = "/model/pose/state";

  // Fields
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::transport::Node node_;

  // Model state
  gz::math::Vector3d position_{0.0, 0.0, 0.0};
  gz::math::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  gz::math::Vector3d linear_velocity_{0.0, 0.0, 0.0};
  gz::math::Vector3d angular_velocity_{0.0, 0.0, 0.0};

  /** Pose message callback **/
  void PoseCallback(const gz::msgs::Pose &msg) {
    position_ = gz::msgs::Convert(msg.position());
    orientation_ = gz::msgs::Convert(msg.orientation());
  }

  /** Twist message callback **/
  void TwistCallback(const gz::msgs::Twist &msg) {
    linear_velocity_ = gz::msgs::Convert(msg.linear());
    angular_velocity_ = gz::msgs::Convert(msg.angular());
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

public:
  gz::transport::Node::Publisher twist_pub_;
  gz::transport::Node::Publisher pose_pub_;

  ModelPlugin() = default;
  virtual ~ModelPlugin() = default;

  /** Configure plugin **/
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override {
    entity_ = entity;
    model_ = gz::sim::Model(entity);

    // Publishers and subscribers
    twist_pub_ = node_.Advertise<gz::msgs::Twist>(twist_state_topic_);
    pose_pub_ = node_.Advertise<gz::msgs::Pose>(pose_state_topic_);
    node_.Subscribe(twist_cmd_topic_, &ModelPlugin::TwistCallback, this);
    node_.Subscribe(pose_cmd_topic_, &ModelPlugin::PoseCallback, this);
  }

  /** Plugin Pre-Update **/
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    UpdatePose(ecm);
    UpdateTwist(ecm);
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
GZ_ADD_PLUGIN(ModelPlugin,
              gz::sim::System,
              ModelPlugin::ISystemConfigure,
              ModelPlugin::ISystemPreUpdate,
              ModelPlugin::ISystemPostUpdate)
