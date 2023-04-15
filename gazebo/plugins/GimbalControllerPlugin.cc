#include <memory>

#include <gz/math/Pose3.hh>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Util.hh>

#include <gz/msgs/Utility.hh>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "GazeboPluginUtils.hh"

#define GIMBAL_IDEL_MODE 0
#define GIMBAL_STABILIZATION_MODE 1
#define GIMBAL_TRACKING_MODE 2

class GimbalControllerPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPreUpdate,
                               public gz::sim::ISystemPostUpdate {
private:
  // Fields
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher joint0_pub_;
  gz::transport::Node::Publisher joint1_pub_;
  gz::transport::Node::Publisher joint2_pub_;

  int mode_ = GIMBAL_STABILIZATION_MODE;
  gz::math::Vector3d target_point_{0.0, 0.0, 0.0};
  gz::math::Vector3d target_attitude_{0.0, 0.0, 0.0};

  /** Gimbal mode message callback **/
  void GimbalModeCallback(const gz::msgs::Int32 &msg) {
    mode_ = gz::msgs::Convert(msg);
  }

  /** Target point message callback **/
  void TargetPointCallback(const gz::msgs::Vector3d &msg) {
    target_point_ = gz::msgs::Convert(msg);
  }

  /** Target attitude message callback **/
  void TargetAttitudeCallback(const gz::msgs::Vector3d &msg) {
    target_attitude_ = gz::msgs::Convert(msg);
  }

public:
  /** Constructors and Destructors **/
  GimbalControllerPlugin() = default;
  virtual ~GimbalControllerPlugin() = default;

  /** Configure plugin **/
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override {
    // Entity and model
    entity_ = entity;
    model_ = gz::sim::Model(entity);

    // Parse joint topics from SDF file
    const auto joint0_cmd_topic = parseString(sdf, "joint0_cmd_topic");
    const auto joint1_cmd_topic = parseString(sdf, "joint1_cmd_topic");
    const auto joint2_cmd_topic = parseString(sdf, "joint2_cmd_topic");
    const auto gimbal_mode_topic = parseString(sdf, "gimbal_mode_topic");
    const auto target_point_topic = parseString(sdf, "target_point_topic");
    const auto target_attitude_topic =
        parseString(sdf, "target_attitude_topic");

    // Publishers and subscribers
    joint0_pub_ = node_.Advertise<gz::msgs::Double>(joint0_cmd_topic);
    joint1_pub_ = node_.Advertise<gz::msgs::Double>(joint1_cmd_topic);
    joint2_pub_ = node_.Advertise<gz::msgs::Double>(joint2_cmd_topic);
    node_.Subscribe(gimbal_mode_topic,
                    &GimbalControllerPlugin::GimbalModeCallback,
                    this);
    node_.Subscribe(target_point_topic,
                    &GimbalControllerPlugin::TargetPointCallback,
                    this);
    node_.Subscribe(target_attitude_topic,
                    &GimbalControllerPlugin::TargetAttitudeCallback,
                    this);
  }

  /** Plugin Pre-Update **/
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
  }

  /** Plugin Pose-Update **/
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
      auto pose = gz::sim::worldPose(model_.Entity(), ecm);

      const auto roll_setpoint = target_attitude_.X();
      const auto pitch_setpoint = target_attitude_.Y();
      const auto yaw_setpoint = target_attitude_.Z();

      const auto roll_actual = pose.Rot().Roll();
      const auto pitch_actual = pose.Rot().Pitch();
      const auto yaw_actual = pose.Rot().Yaw();

      const auto roll_error = roll_setpoint - roll_actual;
      const auto pitch_error = pitch_setpoint - pitch_actual;
      const auto yaw_error = yaw_setpoint - yaw_actual;

      const auto joint0_cmd = -yaw_error;
      const auto joint1_cmd = roll_error;
      const auto joint2_cmd = pitch_error;

      gz::msgs::Double joint0_msg;
      gz::msgs::Double joint1_msg;
      gz::msgs::Double joint2_msg;

      const gz::msgs::Time stamp = gz::msgs::Convert(info.simTime);
      joint0_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
      joint1_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
      joint2_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);

      joint0_msg.set_data(joint0_cmd);
      joint1_msg.set_data(joint1_cmd);
      joint2_msg.set_data(joint2_cmd);

      joint0_pub_.Publish(joint0_msg);
      joint1_pub_.Publish(joint1_msg);
      joint2_pub_.Publish(joint2_msg);
  }
};

// Register plugin
GZ_ADD_PLUGIN(GimbalControllerPlugin,
              gz::sim::System,
              GimbalControllerPlugin::ISystemConfigure,
              GimbalControllerPlugin::ISystemPreUpdate,
              GimbalControllerPlugin::ISystemPostUpdate)