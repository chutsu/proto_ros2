#include <iostream>
#include <memory>
#include <map>
#include <thread>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#define APRILGRID_IMPLEMENTATION
#include "aprilgrid.h"
#include "gz_utils.hpp"

using timestamp_t = int64_t;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
typedef message_filters::sync_policies::
    ApproximateTime<Image, Image, JointState, JointState, JointState>
        approx_policy;
bool keep_running = true;

/**
 * Signal handler
 */
void signal_handler(int signum) {
  keep_running = false;
}

/**
 * Camera Property.
 */
struct CameraProperty {
  int cam_idx;
  int image_width;
  int image_height;
  std::string distortion_model;
  std::vector<double> distortions;
  std::vector<double> intrinsics;

  CameraProperty(const CameraInfo &msg) {
    const auto frame_id = msg.header.frame_id;
    cam_idx = frame_id[frame_id.length() - 1] - '0';

    image_width = msg.width;
    image_height = msg.height;

    distortion_model = msg.distortion_model;
    for (size_t i = 0; i < msg.d.size(); i++) {
      distortions.push_back(msg.d[i]);
    }

    const auto fx = msg.k[0];
    const auto fy = msg.k[4];
    const auto cx = msg.k[2];
    const auto cy = msg.k[5];
    intrinsics.push_back(fx);
    intrinsics.push_back(fy);
    intrinsics.push_back(cx);
    intrinsics.push_back(cy);
  }
};

/**
 * Calibration view.
 */
struct CalibView {
  timestamp_t ts_;
  int num_rows_;
  int num_cols_;
  double tag_size_;
  double tag_spacing_;
  int corners_detected_;

  int *tag_ids_ = nullptr;
  int *corner_indices_ = nullptr;
  double *kps_ = nullptr;
  double *pts_ = nullptr;

  CalibView(const aprilgrid_t *grid) {
    ts_ = grid->timestamp;
    num_rows_ = grid->num_rows;
    num_cols_ = grid->num_cols;
    tag_size_ = grid->tag_size;
    tag_spacing_ = grid->tag_spacing;
    corners_detected_ = grid->corners_detected;

    tag_ids_ = MALLOC(int, corners_detected_);
    corner_indices_ = MALLOC(int, corners_detected_);
    kps_ = MALLOC(double, corners_detected_ * 2);
    pts_ = MALLOC(double, corners_detected_ * 3);
    aprilgrid_measurements(grid, tag_ids_, corner_indices_, kps_, pts_);
  }

  ~CalibView() {
    if (tag_ids_) {
      free(tag_ids_);
    }

    if (corner_indices_) {
      free(corner_indices_);
    }

    if (kps_) {
      free(kps_);
    }

    if (pts_) {
      free(pts_);
    }
  }

  void save(const std::string &save_dir) const {
    auto save_path = save_dir + "/" + std::to_string(ts_) + ".sim";
    FILE *data_file = fopen(save_path.c_str(), "w");

    fprintf(data_file, "num_corners: %d\n", corners_detected_);
    fprintf(data_file, "\n");
    fprintf(data_file, "#tag_id,corner_idx,px,py,pz,kp_x,kp_y\n");
    for (int i = 0; i < corners_detected_; i++) {
      const double kx = kps_[i * 2 + 0];
      const double ky = kps_[i * 2 + 1];
      const double px = pts_[i * 3 + 0];
      const double py = pts_[i * 3 + 1];
      const double pz = pts_[i * 3 + 2];

      fprintf(data_file, "%d,", tag_ids_[i]);
      fprintf(data_file, "%d,", corner_indices_[i]);
      fprintf(data_file, "%f,%f,%f,", px, py, pz);
      fprintf(data_file, "%f,%f\n", kx, ky);
    }
    fclose(data_file);
  }
};

/**
 * Calibration Data
 */
struct CalibData {
  bool gimbal_pose_set = false;
  bool fiducial_pose_set = false;

  std::vector<timestamp_t> timestamps_;
  std::vector<double> joint0_data_;
  std::vector<double> joint1_data_;
  std::vector<double> joint2_data_;
  std::map<int, CameraProperty> camera_props_;
  std::vector<std::shared_ptr<CalibView>> cam0_data_;
  std::vector<std::shared_ptr<CalibView>> cam1_data_;
  std::vector<double> gimbal_pose;

  std::string output_dir_;
  std::string cam0_dir_;
  std::string cam1_dir_;
  std::string joints_fpath_;
  std::string config_fpath_;
  std::string poses_fpath_;

  double cam0_ext[7] = {0};
  double cam1_ext[7] = {0};
  double link0_ext[7] = {0};
  double link1_ext[7] = {0};
  double gimbal_ext[7] = {0};
  double fiducial_ext[7] = {0};

  CalibData(const std::string &output_dir) {
    output_dir_ = output_dir;
    cam0_dir_ = output_dir_ + "/cam0";
    cam1_dir_ = output_dir_ + "/cam1";
    joints_fpath_ = output_dir_ + "/joint_angles.sim";
    config_fpath_ = output_dir_ + "/calib.config";
    poses_fpath_ = output_dir_ + "/poses.sim";

    // Get these from the gimbal model.sdf
    double gzpose_link0[6] = {-0.08, 0.0, 0.1, 0.0, 1.5708, 0.0};
    double gzpose_link1[6] = {0, 0.02, 0.1, 1.5708, 0, 0};
    double gzpose_cam0_ext[6] = {0, 0, -0.1, -1.5708, 0, 1.5708};
    double gzpose_cam1_ext[6] = {0, 0, 0.1, -1.5708, 0, 1.5708};
    double gzpose_gimbal_ext[6] = {0, 0, 0, 0, 0, 0};

    // Convert pose vector to transformation matrices
    double T_link0[4 * 4] = {0};
    double T_link1[4 * 4] = {0};
    double T_gz_cam0_ext[4 * 4] = {0};
    double T_gz_cam1_ext[4 * 4] = {0};
    double T_gimbal_ext[4 * 4] = {0};
    gzpose2tf(gzpose_link0, T_link0);
    gzpose2tf(gzpose_link1, T_link1);
    gzpose2tf(gzpose_cam0_ext, T_gz_cam0_ext);
    gzpose2tf(gzpose_cam1_ext, T_gz_cam1_ext);
    gzpose2tf(gzpose_gimbal_ext, T_gimbal_ext);

    // clang-format off
    double T_rot[4 * 4] = {0};
    T_rot[0]  =  0.0; T_rot[1]  =  0.0;  T_rot[2]  =  1.0;  T_rot[3]  =  0.0;
    T_rot[4]  = -1.0; T_rot[5]  =  0.0;  T_rot[6]  =  0.0;  T_rot[7]  =  0.0;
    T_rot[8]  =  0.0; T_rot[9]  = -1.0;  T_rot[10] =  0.0;  T_rot[11] =  0.0;
    T_rot[12] =  0.0; T_rot[13] =  0.0;  T_rot[14] =  0.0;  T_rot[15] =  1.0;

    double T_cam0_ext[4 * 4] = {0};
    double T_cam1_ext[4 * 4] = {0};
    dot(T_gz_cam0_ext, 4, 4, T_rot, 4, 4, T_cam0_ext);
    dot(T_gz_cam1_ext, 4, 4, T_rot, 4, 4, T_cam1_ext);
    // clang-format on

    // For the transformation matrices we actually want
    tf_vector(T_link0, link0_ext);
    tf_vector(T_link1, link1_ext);
    tf_vector(T_cam0_ext, cam0_ext);
    tf_vector(T_cam1_ext, cam1_ext);
    tf_vector(T_gimbal_ext, gimbal_ext);
  }

  void setGimbalPose(const PoseStamped &msg) {
    if (gimbal_pose_set == false) {
      gimbal_pose.push_back(msg.pose.position.x);
      gimbal_pose.push_back(msg.pose.position.y);
      gimbal_pose.push_back(msg.pose.position.z);
      gimbal_pose.push_back(msg.pose.orientation.w);
      gimbal_pose.push_back(msg.pose.orientation.x);
      gimbal_pose.push_back(msg.pose.orientation.y);
      gimbal_pose.push_back(msg.pose.orientation.z);
    }
    gimbal_pose_set = true;
  }

  void setFiducialPose(const PoseStamped &msg) {
    if (fiducial_pose_set == false) {
      fiducial_ext[0] = msg.pose.position.x;
      fiducial_ext[1] = msg.pose.position.y;
      fiducial_ext[2] = msg.pose.position.z;
      fiducial_ext[3] = msg.pose.orientation.w;
      fiducial_ext[4] = msg.pose.orientation.x;
      fiducial_ext[5] = msg.pose.orientation.y;
      fiducial_ext[6] = msg.pose.orientation.z;
    }
    fiducial_pose_set = true;
  }

  void addCamera(const CameraInfo &msg) {
    const auto frame_id = msg.header.frame_id;
    const int cam_idx = frame_id[frame_id.length() - 1] - '0';
    if (camera_props_.count(cam_idx) == 0) {
      camera_props_.emplace(cam_idx, msg);
    }
  }

  void add(const timestamp_t ts,
           const double joint0,
           const double joint1,
           const double joint2,
           const aprilgrid_t *grid0,
           const aprilgrid_t *grid1) {
    timestamps_.push_back(ts);
    joint0_data_.push_back(joint0);
    joint1_data_.push_back(joint1);
    joint2_data_.push_back(joint2);
    cam0_data_.push_back(std::make_shared<CalibView>(grid0));
    cam1_data_.push_back(std::make_shared<CalibView>(grid1));
  }

  void save() const {
    if (timestamps_.size() == 0) {
      return;
    }

    createOutputDirs();
    saveJointAngles();
    saveCameraViews();
    saveCalibConfig();
    savePoses();
  }

private:
  void createOutputDirs() const {
    // Create output directories
    mkdir(output_dir_.c_str(), 0777);
    mkdir(cam0_dir_.c_str(), 0777);
    mkdir(cam1_dir_.c_str(), 0777);
  }

  void saveJointAngles() const {
    FILE *joints_file = fopen(joints_fpath_.c_str(), "w");
    const int num_views = joint0_data_.size();
    const int num_joints = 3;

    fprintf(joints_file, "num_views: %d\n", num_views);
    fprintf(joints_file, "num_joints: %d\n", num_joints);
    fprintf(joints_file, "\n");
    fprintf(joints_file, "#ts,joint0,joint1,joint2\n");
    for (int i = 0; i < num_views; i++) {
      fprintf(joints_file, "%ld,", timestamps_[i]);
      fprintf(joints_file, "%f,", joint0_data_[i]);
      fprintf(joints_file, "%f,", joint1_data_[i]);
      fprintf(joints_file, "%f\n", joint2_data_[i]);
    }
    fclose(joints_file);
  }

  void saveCameraViews() const {
    for (const auto &view : cam0_data_) {
      view->save(cam0_dir_);
    }
    for (const auto &view : cam1_data_) {
      view->save(cam1_dir_);
    }
  }

  void saveCalibConfig() const {
    FILE *config_file = fopen(config_fpath_.c_str(), "w");
    const int num_cams = 2;
    const int num_links = 2;

    fprintf(config_file, "num_cams: %d\n", num_cams);
    fprintf(config_file, "num_links: %d\n", num_links);
    fprintf(config_file, "\n");

    for (const auto &[cam_idx, prop] : camera_props_) {
      const int res_w = prop.image_width;
      const int res_h = prop.image_height;

      std::string proj = "[";
      proj += std::to_string(prop.intrinsics[0]) + ", ";
      proj += std::to_string(prop.intrinsics[1]) + ", ";
      proj += std::to_string(prop.intrinsics[2]) + ", ";
      proj += std::to_string(prop.intrinsics[3]) + "]";

      std::string dist = "[";
      dist += std::to_string(prop.distortions[0]) + ", ";
      dist += std::to_string(prop.distortions[1]) + ", ";
      dist += std::to_string(prop.distortions[2]) + ", ";
      dist += std::to_string(prop.distortions[3]) + "]";

      fprintf(config_file, "cam%d:\n", prop.cam_idx);
      fprintf(config_file, "  resolution: [%d, %d]\n", res_w, res_h);
      fprintf(config_file, "  proj_model: \"%s\"\n", "pinhole");
      fprintf(config_file, "  dist_model: \"%s\"\n", "radtan4");
      fprintf(config_file, "  proj_params: %s\n", proj.c_str());
      fprintf(config_file, "  dist_params: %s\n", dist.c_str());
      fprintf(config_file, "\n");
    }

    fprintf(config_file, "cam0_ext: [%s]\n", pose2str(cam0_ext).c_str());
    fprintf(config_file, "cam1_ext: [%s]\n", pose2str(cam1_ext).c_str());
    fprintf(config_file, "link0_ext: [%s]\n", pose2str(link0_ext).c_str());
    fprintf(config_file, "link1_ext: [%s]\n", pose2str(link1_ext).c_str());
    fprintf(config_file, "gimbal_ext: [%s]\n", pose2str(gimbal_ext).c_str());
    fprintf(config_file,
            "fiducial_ext: [%s]\n",
            pose2str(fiducial_ext).c_str());

    fclose(config_file);
  }

  void savePoses() const {
    FILE *poses_file = fopen(poses_fpath_.c_str(), "w");
    fprintf(poses_file, "num_poses: %d\n", 1);
    fprintf(poses_file, "\n");
    fprintf(poses_file, "#ts,rx,ry,rz,qw,qx,qy,qz\n");

    const auto rx = gimbal_pose[0];
    const auto ry = gimbal_pose[1];
    const auto rz = gimbal_pose[2];
    const auto qw = gimbal_pose[3];
    const auto qx = gimbal_pose[4];
    const auto qy = gimbal_pose[5];
    const auto qz = gimbal_pose[6];
    fprintf(poses_file, "%ld,", timestamps_[0]);
    fprintf(poses_file, "%f,%f,%f,%f,%f,%f,%f\n", rx, ry, rz, qw, qx, qy, qz);

    fclose(poses_file);
  }
};

/**
 * Gimbal calibration dance.
 */
class GZGimbal : public rclcpp::Node {
private:
  // Mutex
  std::mutex mutex_;

  // Publishers / Subscribers
  // clang-format off
  rclcpp::Publisher<Float64>::SharedPtr joint0_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr joint1_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr joint2_cmd_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_;
  rclcpp::Subscription<PoseStamped>::SharedPtr aprilgrid_pose_;
  rclcpp::Subscription<PoseStamped>::SharedPtr gimbal_pose_;
  message_filters::Subscriber<Image> cam0_;
  message_filters::Subscriber<Image> cam1_;
  message_filters::Subscriber<JointState> joint0_state_;
  message_filters::Subscriber<JointState> joint1_state_;
  message_filters::Subscriber<JointState> joint2_state_;
  std::shared_ptr<message_filters::Synchronizer<approx_policy>> sync_;
  // clang-format on

  // Joint limits
  double jlims0_[2] = {deg2rad(-60.0), deg2rad(60.0)};
  double jlims1_[2] = {deg2rad(-45.0), deg2rad(45.0)};
  double jlims2_[2] = {deg2rad(-45.0), deg2rad(45.0)};

  // Dance limits
  double dlims0_[2] = {deg2rad(-10.0), deg2rad(10.0)};
  double dlims1_[2] = {deg2rad(-35.0), deg2rad(35.0)};
  double dlims2_[2] = {deg2rad(-10.0), deg2rad(10.0)};

  // Dance intervals
  int num_yaw_ = 3;
  int num_roll_ = 4;
  int num_pitch_ = 4;

  // State
  timestamp_t ts_ = 0;
  cv::Mat cam0_img_;
  cv::Mat cam1_img_;
  double joint0_ = 0;
  double joint1_ = 0;
  double joint2_ = 0;

  // Thread
  std::thread calib_dance_thread_;

  // Outputs
  std::string output_dir_ = "/tmp/calib_gimbal";
  CalibData calib_data_{output_dir_};
  aprilgrid_detector_t *detector_ = nullptr;

  /* Measurement Callback. **/
  void measurementCallback(const Image::ConstSharedPtr &cam0_msg,
                           const Image::ConstSharedPtr &cam1_msg,
                           const JointState::ConstSharedPtr &joint0_msg,
                           const JointState::ConstSharedPtr &joint1_msg,
                           const JointState::ConstSharedPtr &joint2_msg) {
    std::lock_guard<std::mutex> guard(mutex_);

    // Images
    auto stamp = cam0_msg->header.stamp;
    ts_ = stamp.sec * 1e9 + stamp.nanosec;
    const auto ptr0 = cv_bridge::toCvCopy(cam0_msg, cam0_msg->encoding);
    const auto ptr1 = cv_bridge::toCvCopy(cam1_msg, cam1_msg->encoding);
    auto img0 = ptr0->image;
    auto img1 = ptr1->image;
    cv::cvtColor(img0, cam0_img_, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img1, cam1_img_, cv::COLOR_BGR2GRAY);

    // Joint angles
    joint0_ = joint0_msg->position[0];
    joint1_ = joint1_msg->position[0];
    joint2_ = joint2_msg->position[0];
  }

  /** Camera information callback. **/
  void cameraInfoCallback(const CameraInfo &msg) {
    calib_data_.addCamera(msg);
  }

  /** AprilGrid pose callback. **/
  void aprilgridCallback(const PoseStamped &msg) {
    calib_data_.setFiducialPose(msg);
  }

  /** Gimbal pose callback. **/
  void gimbalCallback(const PoseStamped &msg) {
    calib_data_.setGimbalPose(msg);
  }

public:
  GZGimbal() : Node("GZGimbal") {
    // Setup
    int num_rows = 10;
    int num_cols = 10;
    double tag_size = 0.08;
    double tag_spacing = 0.25;
    detector_ =
        aprilgrid_detector_malloc(num_rows, num_cols, tag_size, tag_spacing);

    // Setup publishers / subscribers
    // clang-format off
    auto info_cb = std::bind(&GZGimbal::cameraInfoCallback, this, _1);
    auto grid_cb = std::bind(&GZGimbal::aprilgridCallback, this, _1);
    auto gimbal_cb = std::bind(&GZGimbal::gimbalCallback, this, _1);
    auto meas_cb = std::bind(&GZGimbal::measurementCallback, this, _1, _2, _3, _4, _5);
    joint0_cmd_ = create_publisher<Float64>("/gimbal/joint0_cmd", 1);
    joint1_cmd_ = create_publisher<Float64>("/gimbal/joint1_cmd", 1);
    joint2_cmd_ = create_publisher<Float64>("/gimbal/joint2_cmd", 1);
    camera_info_ = create_subscription<CameraInfo>("/gimbal/camera_info", 1, info_cb);
    aprilgrid_pose_ = create_subscription<PoseStamped>("/model/aprilgrid/pose", 1, grid_cb);
    gimbal_pose_ = create_subscription<PoseStamped>("/model/gimbal/pose", 1, gimbal_cb);
    cam0_.subscribe(this, "/gimbal/camera0");
    cam1_.subscribe(this, "/gimbal/camera1");
    joint0_state_.subscribe(this, "/gimbal/joint0_state");
    joint1_state_.subscribe(this, "/gimbal/joint1_state");
    joint2_state_.subscribe(this, "/gimbal/joint2_state");
    sync_ = std::make_shared<message_filters::Synchronizer<approx_policy>>(
      approx_policy(10),
      cam0_,
      cam1_,
      joint0_state_,
      joint1_state_,
      joint2_state_
    );
    sync_->registerCallback(meas_cb);
    // clang-format on

    calib_dance_thread_ = std::thread(&GZGimbal::calibrationDance, this);
  }

  virtual ~GZGimbal() {
    calib_dance_thread_.join();
    aprilgrid_detector_free(detector_);
  }

  /** Perform gimbal calibration dance. **/
  void calibrationDance() {
    // Reset
    reset();

    // clang-format off
    double dyaw = (dlims0_[1] - dlims0_[0]) / (num_yaw_ - 1);
    double dpitch = (dlims1_[1] - dlims1_[0]) / (num_roll_ - 1);
    double droll = (dlims2_[1] - dlims2_[0]) / (num_pitch_ - 1);
    double yaw = dlims0_[0];
    double pitch = dlims1_[0];
    double roll = dlims2_[0];
    // clang-format on

    int view_idx = 0;
    for (int i = 0; i < num_yaw_; i++) {
      roll = dlims1_[0];
      for (int j = 0; j < num_roll_; j++) {
        pitch = dlims1_[0];
        for (int k = 0; k < num_pitch_; k++) {
          publishJointCommand(0, yaw);
          publishJointCommand(1, roll);
          publishJointCommand(2, pitch);

          printf("view_idx: %d, ", view_idx);
          printf("yaw: %f, ", yaw);
          printf("roll: %f, ", roll);
          printf("pitch: %f ", pitch);
          printf("\n");
          fflush(stdout);

          view_idx += 1;
          pitch += dpitch;
          sleep(2);

          // Add to calibration data
          {
            std::lock_guard<std::mutex> guard(mutex_);
            aprilgrid_t *grid0 = aprilgrid_detector_detect(detector_,
                                                           ts_,
                                                           cam0_img_.cols,
                                                           cam0_img_.rows,
                                                           cam0_img_.cols,
                                                           cam0_img_.data);
            aprilgrid_t *grid1 = aprilgrid_detector_detect(detector_,
                                                           ts_,
                                                           cam1_img_.cols,
                                                           cam1_img_.rows,
                                                           cam1_img_.cols,
                                                           cam1_img_.data);

            // const auto viz = aprilgrid_draw(grid0, cam0_img_);
            // cv::imshow("Viz", viz);
            // cv::waitKey(1);

            calib_data_.add(ts_, joint0_, joint1_, joint2_, grid0, grid1);
            aprilgrid_free(grid0);
            aprilgrid_free(grid1);
          }
          sleep(1);

          if (keep_running == false) {
            i = num_yaw_;
            j = num_roll_;
            k = num_pitch_;
          }
        }
        roll += droll;
      }
      yaw += dyaw;
    }

    // Save calibration data
    calib_data_.save();

    // Reset
    reset();
  }

  /** Reset gimbal angles to 0, 0, 0 **/
  void reset() {
    sleep(2);
    publishJointCommand(0, 0);
    publishJointCommand(1, 0);
    publishJointCommand(2, 0);
    sleep(2);
  }

  /** Publish joint angle command **/
  void publishJointCommand(const int joint_idx, const double cmd) {
    double val = cmd;
    Float64 msg{};

    switch (joint_idx) {
      case 0:
        clip(&val, 1, jlims0_[0], jlims0_[1]);
        msg.data = val;
        joint0_cmd_->publish(msg);
        break;
      case 1:
        clip(&val, 1, jlims1_[0], jlims1_[1]);
        msg.data = val;
        joint1_cmd_->publish(msg);
        break;
      case 2:
        clip(&val, 1, jlims2_[0], jlims2_[1]);
        msg.data = val;
        joint2_cmd_->publish(msg);
        break;
    }
  }
};

int main(int argc, char *argv[]) {
  // Setup signal handler
  signal(SIGINT, signal_handler);

  // Setup ROS2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GZGimbal>());
  rclcpp::shutdown();

  return 0;
}
