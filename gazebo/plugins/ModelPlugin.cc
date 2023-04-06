#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/config.hh>
#include <gz/transport/Node.hh>
#include <memory>

class ModelPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
 private:
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::transport::Node node_;

  gz::math::Vector3d position_{0.0, 0.0, 0.0};
  gz::math::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  gz::math::Vector3d linear_velocity_{0.0, 0.0, 0.0};
  gz::math::Vector3d angular_velocity_{0.0, 0.0, 0.0};

  void PoseCallback(const gz::msgs::Pose &msg) {
    position_ = gz::msgs::Convert(msg.position());
    orientation_ = gz::msgs::Convert(msg.orientation());
  }

  void TwistCallback(const gz::msgs::Twist &msg) {
    linear_velocity_ = gz::msgs::Convert(msg.linear());
    angular_velocity_ = gz::msgs::Convert(msg.angular());
  }

  void UpdatePose(gz::sim::EntityComponentManager &ecm) {
    gz::math::Pose3d pose{position_, orientation_};
    model_.SetWorldPoseCmd(ecm, pose);
  }

  void UpdateTwist(gz::sim::EntityComponentManager &ecm) {
    auto lvel_cmd = gz::sim::components::LinearVelocityCmd(linear_velocity_);
    auto lvel = ecm.Component<gz::sim::components::LinearVelocityCmd>(entity_);
    if (lvel == nullptr) {
      ecm.CreateComponent(entity_, lvel_cmd);
    } else {
      *lvel = lvel_cmd;
    }

    auto avel_cmd = gz::sim::components::AngularVelocityCmd(angular_velocity_);
    auto avel = ecm.Component<gz::sim::components::AngularVelocityCmd>(entity_);
    if (avel == nullptr) {
      ecm.CreateComponent(entity_, avel_cmd);
    } else {
      *avel = avel_cmd;
    }
  }

 public:
  ModelPlugin() = default;
  virtual ~ModelPlugin() = default;

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override {
    entity_ = entity;
    model_ = gz::sim::Model(entity);

    const auto twist_topic = "/model/twist";
    node_.Subscribe(twist_topic, &ModelPlugin::TwistCallback, this);

    const auto pose_topic = "/model/pose";
    node_.Subscribe(pose_topic, &ModelPlugin::PoseCallback, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    UpdatePose(ecm);
    UpdateTwist(ecm);
  }
};

// Register plugin
GZ_ADD_PLUGIN(ModelPlugin, gz::sim::System, ModelPlugin::ISystemConfigure,
              ModelPlugin::ISystemPreUpdate)
