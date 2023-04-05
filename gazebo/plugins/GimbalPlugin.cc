#include <gz/sim/Util.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/plugin/Register.hh>

namespace proto_gazebo {

class GimbalPlugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate {
private:
  gz::sim::Entity cam0_;
  gz::sim::Entity joint_yaw_;
  gz::sim::Entity joint_roll_;
  gz::sim::Entity joint_pitch_;

  size_t counter_ = 0;
  int toggle = 0;

public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &event_manager) override {
    auto model = gz::sim::Model(entity);
    cam0_ = model.LinkByName(ecm, "camera0");
    joint_yaw_ = model.JointByName(ecm, "joint_yaw");
    // joint_roll_ = model.JointByName(ecm, "joint_roll");
    // joint_pitch_ = model.JointByName(ecm, "joint_pitch");

    auto yaw_comp =
        ecm.Component<gz::sim::components::JointPosition>(joint_yaw_);
    if (!yaw_comp) {
      ecm.CreateComponent(joint_yaw_, gz::sim::components::JointPosition());
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    // std::cout << worldPose(cam0_, ecm) << std::endl;

    if (counter_ % 1000 == 0) {
      // joint_yaw_
      double val = toggle * 1;

      // Update velocity command.
      auto vel =
          ecm.Component<gz::sim::components::JointVelocityCmd>(joint_yaw_);
      if (vel == nullptr) {
        ecm.CreateComponent(joint_yaw_,
                            gz::sim::components::JointVelocityCmd({val}));
      } else {
        if (toggle == 0) {
          *vel = gz::sim::components::JointVelocityCmd({val});
          toggle *= -1;
        } else {
          *vel = gz::sim::components::JointVelocityCmd({val});
          toggle *= -1;
        }
      }
    }

    counter_++;
  }
};

} // namespace proto_gazebo

// Register plugin
GZ_ADD_PLUGIN(proto_gazebo::GimbalPlugin,
              gz::sim::System,
              proto_gazebo::GimbalPlugin::ISystemConfigure,
              proto_gazebo::GimbalPlugin::ISystemPreUpdate)
