#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

#include "System.h"

class NgdSlamRgbdNode : public rclcpp::Node
{
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

  NgdSlamRgbdNode()
  : rclcpp::Node("ngd_slam_rgbd"),
    rgb_sub_(),
    depth_sub_()
  {
    voc_file_ = this->declare_parameter<std::string>("voc_file", "");
    settings_file_ = this->declare_parameter<std::string>("settings_file", "");
    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ =
      this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/ngd_slam/camera_pose");
    tracking_state_topic_ =
      this->declare_parameter<std::string>("tracking_state_topic", "/ngd_slam/tracking_state");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera");
    queue_size_ = this->declare_parameter<int>("queue_size", 10);
    use_viewer_ = this->declare_parameter<bool>("use_viewer", true);
    save_keyframe_trajectory_ =
      this->declare_parameter<std::string>("save_keyframe_trajectory", "");
    save_camera_trajectory_ =
      this->declare_parameter<std::string>("save_camera_trajectory", "");

    if (voc_file_.empty() || settings_file_.empty()) {
      throw std::runtime_error(
              "Both 'voc_file' and 'settings_file' parameters are required.");
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    tracking_state_pub_ = this->create_publisher<std_msgs::msg::Int32>(tracking_state_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Loading vocabulary: %s", voc_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Loading settings: %s", settings_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing RGB topic: %s", rgb_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing depth topic: %s", depth_topic_.c_str());

    slam_ = std::make_unique<ORB_SLAM3::System>(
      voc_file_, settings_file_, ORB_SLAM3::System::RGBD, use_viewer_);

    const rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
    rgb_sub_.subscribe(this, rgb_topic_, sensor_qos);
    depth_sub_.subscribe(this, depth_topic_, sensor_qos);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(static_cast<uint32_t>(queue_size_)), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&NgdSlamRgbdNode::grab_rgbd, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~NgdSlamRgbdNode() override
  {
    shutdown_slam();
  }

private:
  void grab_rgbd(
    const ImageMsg::ConstSharedPtr & rgb_msg,
    const ImageMsg::ConstSharedPtr & depth_msg)
  {
    cv_bridge::CvImageConstPtr rgb_ptr;
    cv_bridge::CvImageConstPtr depth_ptr;

    try {
      rgb_ptr = cv_bridge::toCvShare(rgb_msg);
      depth_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    Sophus::SE3f tcw = slam_->TrackRGBD(
      rgb_ptr->image, depth_ptr->image,
      rclcpp::Time(rgb_msg->header.stamp).seconds());

    const int tracking_state = slam_->GetTrackingState();
    std_msgs::msg::Int32 state_msg;
    state_msg.data = tracking_state;
    tracking_state_pub_->publish(state_msg);

    if ((tracking_state != ORB_SLAM3::Tracking::OK) &&
      (tracking_state != ORB_SLAM3::Tracking::OK_KLT))
    {
      return;
    }

    if (!tcw.matrix().allFinite()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received non-finite pose from NGD-SLAM, skipping publish.");
      return;
    }

    const Sophus::SE3f twc = tcw.inverse();
    publish_pose(twc, rgb_msg->header.stamp);
  }

  void publish_pose(const Sophus::SE3f & twc, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;

    const Eigen::Vector3f translation = twc.translation();
    const Eigen::Quaternionf quaternion(twc.rotationMatrix());

    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();

    pose_pub_->publish(pose_msg);
  }

  void shutdown_slam()
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    if (!slam_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Shutting down NGD-SLAM.");
    slam_->Shutdown();

    if (!save_camera_trajectory_.empty()) {
      slam_->SaveTrajectoryTUM(save_camera_trajectory_);
      RCLCPP_INFO(
        this->get_logger(), "Saved camera trajectory to %s",
        save_camera_trajectory_.c_str());
    }

    if (!save_keyframe_trajectory_.empty()) {
      slam_->SaveKeyFrameTrajectoryTUM(save_keyframe_trajectory_);
      RCLCPP_INFO(
        this->get_logger(), "Saved keyframe trajectory to %s",
        save_keyframe_trajectory_.c_str());
    }

    slam_.reset();
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;

  message_filters::Subscriber<ImageMsg> rgb_sub_;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracking_state_pub_;

  std::mutex shutdown_mutex_;

  std::string voc_file_;
  std::string settings_file_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string pose_topic_;
  std::string tracking_state_topic_;
  std::string map_frame_;
  std::string camera_frame_;
  std::string save_keyframe_trajectory_;
  std::string save_camera_trajectory_;
  int queue_size_;
  bool use_viewer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<NgdSlamRgbdNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ngd_slam_rgbd"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
