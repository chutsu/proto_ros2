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

class GimbalPlugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate {
private:
  // Gazebo topics
  std::string pose_state_topic_ = "/gimbal/pose/state";
  std::string setpoint_topic_ = "/gimbal/setpoint/cmd";

  // Fields
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pose_pub_;

  // Model state
  gimbal_model_t gimbal_;
  gimbal_ctrl_t ctrl_;
  gz::math::Vector3d setpoint_{0.0, 0.0, 0.0};
  gz::math::Vector3d position_{0.0, 0.0, 0.0};
  gz::math::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};

  void SetupGimbal() {
    // clang-format off
    const real_t x[6] = {
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0
    };
    // clang-format on
    gimbal_model_setup(&gimbal_);
  }

  /** Update Gimbal **/
  void UpdateGimbal(gz::sim::EntityComponentManager &ecm) {
    // Update model
    const real_t dt = 0.001;
    const real_t sp[3] = {setpoint_.X(), setpoint_.Y(), setpoint_.Z()};
    const real_t pv[3] = {gimbal_.x[0], gimbal_.x[2], gimbal_.x[4]};

    real_t u[4] = {0};
    gimbal_ctrl_update(&ctrl_, sp, pv, dt, u);
    gimbal_model_update(&gimbal_, u, dt);

    real_t ypr[3] = {gimbal_.x[4], gimbal_.x[2], gimbal_.x[0]};
    real_t quat[4] = {0};
    euler2quat(ypr, quat);

    // Set state
    orientation_.W() = quat[0];
    orientation_.X() = quat[1];
    orientation_.Y() = quat[2];
    orientation_.Z() = quat[3];

    UpdatePose(ecm);
  }

  /** Update model pose **/
  void UpdatePose(gz::sim::EntityComponentManager &ecm) {
    gz::math::Pose3d pose{position_, orientation_};
    model_.SetWorldPoseCmd(ecm, pose);
  }

  /** Setpoint position message callback **/
  void SetpointPositionCallback(const gz::msgs::Vector3d &msg) {
    setpoint_ = gz::msgs::Convert(msg);
  }

public:
  /** Constructors and Destructors **/
  GimbalPlugin() = default;
  virtual ~GimbalPlugin() = default;

  /** Configure plugin **/
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override {
    // Entity and model
    entity_ = entity;
    model_ = gz::sim::Model(entity);
    SetupGimbal();

    // Publishers and subscribers
    pose_pub_ = node_.Advertise<gz::msgs::Pose>(pose_state_topic_);
    node_.Subscribe(setpoint_topic_,
                    &GimbalPlugin::SetpointPositionCallback,
                    this);
  }

  /** Plugin Pre-Update **/
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) {
      return;
    }

    UpdateGimbal(ecm);
  }

  /** Plugin Pose-Update **/
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    // Message timestamp
    const gz::msgs::Time stamp = gz::msgs::Convert(info.simTime);

    // Publish pose message
    gz::msgs::Pose pose_msg;
    pose_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    gz::msgs::Set(pose_msg.mutable_position(), position_);
    gz::msgs::Set(pose_msg.mutable_orientation(), orientation_);
    pose_pub_.Publish(pose_msg);
  }
};

// Register plugin
GZ_ADD_PLUGIN(GimbalPlugin,
              gz::sim::System,
              GimbalPlugin::ISystemConfigure,
              GimbalPlugin::ISystemPreUpdate,
              GimbalPlugin::ISystemPostUpdate)
