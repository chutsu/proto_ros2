#include <memory>

#include <gz/math/Pose3.hh>
#include <gz/math/Matrix4.hh>

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

class GimbalControllerPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPreUpdate,
                               public gz::sim::ISystemPostUpdate {
private:
  // Fields
  gz::sim::Entity entity_;
  gz::sim::Model model_;
  gz::sim::Entity joint0_;
  gz::sim::Entity joint1_;
  gz::sim::Entity joint2_;
  gz::sim::Entity cam0_;
  gz::sim::Entity cam1_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher mode_state_pub_;
  gz::transport::Node::Publisher target_point_state_pub_;
  gz::transport::Node::Publisher target_attitude_state_pub_;
  gz::transport::Node::Publisher joint0_pub_;
  gz::transport::Node::Publisher joint1_pub_;
  gz::transport::Node::Publisher joint2_pub_;

  int mode_ = GIMBAL_STABILIZATION_MODE;
  // int mode_ = GIMBAL_IDEL_MODE;
  gz::math::Vector3d target_point_{0.0, 0.0, 0.0};
  gz::math::Vector3d target_attitude_{0.0, 0.0, 0.0};

  /** Print pose **/
  void PrintPose(const std::string &prefix,
                 const gz::math::Matrix4d &tf) const {
    const auto rx = tf.Translation().X();
    const auto ry = tf.Translation().Y();
    const auto rz = tf.Translation().Z();

    const auto qw = tf.Rotation().W();
    const auto qx = tf.Rotation().X();
    const auto qy = tf.Rotation().Y();
    const auto qz = tf.Rotation().Z();

    // std::cout << prefix + ": ";
    // std::cout << rx << ", ";
    // std::cout << ry << ", ";
    // std::cout << rz << ", ";
    // std::cout << qx << ", ";
    // std::cout << qy << ", ";
    // std::cout << qz << ", ";
    // std::cout << qw << "  ";
    // std::cout << "# x, y, z, qx, qy, qz, qw" << std::endl;

    std::cout << prefix + " = np.array([";
    std::cout << rx << ", ";
    std::cout << ry << ", ";
    std::cout << rz << ", ";
    std::cout << qx << ", ";
    std::cout << qy << ", ";
    std::cout << qz << ", ";
    std::cout << qw << "";
    std::cout << "])" << std::endl;
  }

  /** Print gimbal kinematics **/
  void PrintGimbalKinematics(const gz::sim::EntityComponentManager &ecm) const {
    const auto pose_WB = gz::sim::worldPose(model_.Entity(), ecm);
    const auto pose_WM0 = gz::sim::worldPose(joint0_, ecm);
    const auto pose_WM1 = gz::sim::worldPose(joint1_, ecm);
    const auto pose_WM2 = gz::sim::worldPose(joint2_, ecm);
    const auto pose_WC0 = gz::sim::worldPose(cam0_, ecm);
    const auto pose_WC1 = gz::sim::worldPose(cam1_, ecm);

    const gz::math::Matrix4 T_WB{pose_WB};
    const gz::math::Matrix4 T_WM0{pose_WM0};
    const gz::math::Matrix4 T_WM1{pose_WM1};
    const gz::math::Matrix4 T_WM2{pose_WM2};
    const gz::math::Matrix4 T_WC0{pose_WC0};
    const gz::math::Matrix4 T_WC1{pose_WC1};

    const gz::math::Quaterniond cam_rot{-M_PI / 2, 0.0, -M_PI / 2.0};
    const gz::math::Vector3d cam_trans{0.0, 0.0, 0.0};
    const gz::math::Pose3d cam_correction{cam_trans, cam_rot};
    const gz::math::Matrix4d T_correction{cam_correction};

    const auto T_BM0 = T_WB.Inverse() * T_WM0;
    const auto T_M0M1 = T_WM0.Inverse() * T_WM1;
    const auto T_M1M2 = T_WM1.Inverse() * T_WM2;
    const auto T_M2C0 = T_WM2.Inverse() * T_WC0 * T_correction;
    const auto T_M2C1 = T_WM2.Inverse() * T_WC1 * T_correction;
    const auto T_WC0_corrected = T_WC0 * T_correction;
    const auto T_WC1_corrected = T_WC1 * T_correction;

    std::cout << "Gimbal Kinematics:" << std::endl;
    // PrintPose("T_WB", T_WB);
    // PrintPose("T_BM0", T_BM0);
    // PrintPose("T_M0M1", T_M0M1);
    // PrintPose("T_M1M2", T_M1M2);
    // PrintPose("T_M2C0", T_M2C0);
    // PrintPose("T_M2C1", T_M2C1);

    PrintPose("gimbal_pose", T_WB);
    PrintPose("gimbal_ext", T_BM0);
    PrintPose("gimbal_link0", T_M0M1);
    PrintPose("gimbal_link1", T_M1M2);
    PrintPose("cam0_ext", T_M2C0);
    PrintPose("cam1_ext", T_M2C1);
    PrintPose("cam0_pose", T_WC0_corrected);
    PrintPose("cam1_pose", T_WC1_corrected);
  }

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
    joint0_ = model_.LinkByName(ecm, "gimbal_yaw_motor");
    joint1_ = model_.LinkByName(ecm, "gimbal_roll_motor");
    joint2_ = model_.LinkByName(ecm, "gimbal_pitch_motor");
    cam0_ = *gz::sim::entitiesFromScopedName("camera0", ecm).begin();
    cam1_ = *gz::sim::entitiesFromScopedName("camera1", ecm).begin();
    PrintGimbalKinematics(ecm);

    // Parse joint topics from SDF file
    // clang-format off
    const auto joint0_cmd_topic = parseString(sdf, "joint0_cmd_topic");
    const auto joint1_cmd_topic = parseString(sdf, "joint1_cmd_topic");
    const auto joint2_cmd_topic = parseString(sdf, "joint2_cmd_topic");
    const auto gimbal_mode_state_topic = parseString(sdf, "gimbal_mode_state_topic");
    const auto target_point_state_topic = parseString(sdf, "target_point_state_topic");
    const auto target_attitude_state_topic = parseString(sdf, "target_attitude_state_topic");
    const auto gimbal_mode_cmd_topic = parseString(sdf, "gimbal_mode_cmd_topic");
    const auto target_point_cmd_topic = parseString(sdf, "target_point_cmd_topic");
    const auto target_attitude_cmd_topic = parseString(sdf, "target_attitude_cmd_topic");
    // clang-format on

    // Publishers and subscribers
    // clang-format off
    joint0_pub_ = node_.Advertise<gz::msgs::Double>(joint0_cmd_topic);
    joint1_pub_ = node_.Advertise<gz::msgs::Double>(joint1_cmd_topic);
    joint2_pub_ = node_.Advertise<gz::msgs::Double>(joint2_cmd_topic);
    mode_state_pub_ = node_.Advertise<gz::msgs::Int32>(gimbal_mode_state_topic);
    target_point_state_pub_ = node_.Advertise<gz::msgs::Vector3d>(target_point_state_topic);
    target_attitude_state_pub_ = node_.Advertise<gz::msgs::Vector3d>(target_attitude_state_topic);
    node_.Subscribe(gimbal_mode_cmd_topic, &GimbalControllerPlugin::GimbalModeCallback, this);
    node_.Subscribe(target_point_cmd_topic, &GimbalControllerPlugin::TargetPointCallback, this);
    node_.Subscribe(target_attitude_cmd_topic, &GimbalControllerPlugin::TargetAttitudeCallback, this);
    // clang-format on
  }

  /** Plugin Pre-Update **/
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
  }

  /** Plugin Pose-Update **/
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    double joint0_cmd = 0.0;
    double joint1_cmd = 0.0;
    double joint2_cmd = 0.0;

    if (mode_ == GIMBAL_STABILIZATION_MODE) {
      const auto roll_setpoint = target_attitude_.X();
      const auto pitch_setpoint = target_attitude_.Y();
      const auto yaw_setpoint = target_attitude_.Z();

      const auto pose = gz::sim::worldPose(model_.Entity(), ecm);
      const auto roll_actual = pose.Rot().Roll();
      const auto pitch_actual = pose.Rot().Pitch();
      const auto yaw_actual = pose.Rot().Yaw();

      const auto roll_error = roll_setpoint - roll_actual;
      const auto pitch_error = pitch_setpoint - pitch_actual;
      const auto yaw_error = yaw_setpoint - yaw_actual;

      joint0_cmd = -yaw_error;
      joint1_cmd = roll_error;
      joint2_cmd = pitch_error;
    }

    // -- Publish joint commnds
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

    if (mode_ != GIMBAL_IDEL_MODE) {
      joint0_pub_.Publish(joint0_msg);
      joint1_pub_.Publish(joint1_msg);
      joint2_pub_.Publish(joint2_msg);
    }

    // -- Publish gimbal mode
    gz::msgs::Int32 mode_msg;
    mode_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    mode_msg.set_data(mode_);
    mode_state_pub_.Publish(mode_msg);

    // -- Publish target point
    gz::msgs::Vector3d target_point_msg;
    target_point_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    target_point_msg.set_x(target_point_.X());
    target_point_msg.set_y(target_point_.Y());
    target_point_msg.set_z(target_point_.Z());
    target_point_state_pub_.Publish(target_point_msg);

    // -- Publish target attitude
    gz::msgs::Vector3d target_attitude_msg;
    target_attitude_msg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    target_attitude_msg.set_x(target_attitude_.X());
    target_attitude_msg.set_y(target_attitude_.Y());
    target_attitude_msg.set_z(target_attitude_.Z());
    target_attitude_state_pub_.Publish(target_attitude_msg);
  }
};

// Register plugin
GZ_ADD_PLUGIN(GimbalControllerPlugin,
              gz::sim::System,
              GimbalControllerPlugin::ISystemConfigure,
              GimbalControllerPlugin::ISystemPreUpdate,
              GimbalControllerPlugin::ISystemPostUpdate)
