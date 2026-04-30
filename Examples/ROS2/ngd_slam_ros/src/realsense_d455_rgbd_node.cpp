#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class RealSenseD455RgbdNode : public rclcpp::Node
{
public:
  RealSenseD455RgbdNode()
  : rclcpp::Node("realsense_d455_rgbd"),
    running_(true)
  {
    serial_no_ = this->declare_parameter<std::string>("serial_no", "");
    color_topic_ = this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    color_info_topic_ = this->declare_parameter<std::string>(
      "color_info_topic", "/camera/color/camera_info");
    depth_info_topic_ = this->declare_parameter<std::string>(
      "depth_info_topic", "/camera/aligned_depth_to_color/camera_info");
    color_frame_id_ = this->declare_parameter<std::string>(
      "color_frame_id", "camera_color_optical_frame");
    depth_frame_id_ = this->declare_parameter<std::string>(
      "depth_frame_id", "camera_color_optical_frame");
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);
    fps_ = this->declare_parameter<int>("fps", 30);
    enable_auto_exposure_ = this->declare_parameter<bool>("enable_auto_exposure", true);
    emitter_enabled_ = this->declare_parameter<bool>("emitter_enabled", true);
    align_depth_ = this->declare_parameter<bool>("align_depth", true);
    publish_camera_info_ = this->declare_parameter<bool>("publish_camera_info", true);

    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(color_topic_, 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_, 10);
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic_, 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic_, 10);

    setup_pipeline();
    capture_thread_ = std::thread(&RealSenseD455RgbdNode::capture_loop, this);
  }

  ~RealSenseD455RgbdNode() override
  {
    running_.store(false);
    try {
      pipeline_.stop();
    } catch (const rs2::error &) {
    }

    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

private:
  void setup_pipeline()
  {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
      throw std::runtime_error("No RealSense device connected.");
    }

    std::string selected_serial;
    std::string selected_name;

    for (auto && device : devices) {
      const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
      const std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (!serial_no_.empty() && serial == serial_no_) {
        selected_serial = serial;
        selected_name = name;
        break;
      }
      if (serial_no_.empty() && name.find("D455") != std::string::npos) {
        selected_serial = serial;
        selected_name = name;
        break;
      }
    }

    if (selected_serial.empty()) {
      auto fallback = devices.front();
      selected_serial = fallback.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      selected_name = fallback.get_info(RS2_CAMERA_INFO_NAME);
    }

    RCLCPP_INFO(
      this->get_logger(), "Opening RealSense device '%s' (serial: %s)",
      selected_name.c_str(), selected_serial.c_str());

    config_.enable_device(selected_serial);
    config_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);
    config_.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);

    profile_ = pipeline_.start(config_);
    configure_sensor_options(profile_.get_device());

    color_profile_ = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    depth_profile_ = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

    color_info_template_ = build_camera_info(color_profile_.get_intrinsics(), color_frame_id_);
    depth_info_template_ = build_camera_info(
      align_depth_ ? color_profile_.get_intrinsics() : depth_profile_.get_intrinsics(),
      align_depth_ ? depth_frame_id_ : "camera_depth_optical_frame");
  }

  void configure_sensor_options(const rs2::device & device)
  {
    for (auto && sensor : device.query_sensors()) {
      if (!sensor.supports(RS2_CAMERA_INFO_NAME)) {
        continue;
      }

      const std::string sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
      if (sensor_name.find("Stereo Module") != std::string::npos) {
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_exposure_ ? 1.0f : 0.0f);
        }
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
          sensor.set_option(RS2_OPTION_EMITTER_ENABLED, emitter_enabled_ ? 1.0f : 0.0f);
        }
      }

      if (sensor_name.find("RGB Camera") != std::string::npos &&
        sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
      {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_exposure_ ? 1.0f : 0.0f);
      }
    }
  }

  sensor_msgs::msg::CameraInfo build_camera_info(
    const rs2_intrinsics & intrinsics,
    const std::string & frame_id) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.width = intrinsics.width;
    info.height = intrinsics.height;
    info.distortion_model = "plumb_bob";
    info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 5);

    info.k = {
      intrinsics.fx, 0.0, intrinsics.ppx,
      0.0, intrinsics.fy, intrinsics.ppy,
      0.0, 0.0, 1.0
    };

    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    info.p = {
      intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
      0.0, intrinsics.fy, intrinsics.ppy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    return info;
  }

  sensor_msgs::msg::Image make_image_msg(
    const rs2::video_frame & frame,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id,
    const std::string & encoding,
    std::size_t bytes_per_pixel) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.width = static_cast<uint32_t>(frame.get_width());
    msg.height = static_cast<uint32_t>(frame.get_height());
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(msg.width * bytes_per_pixel);

    const auto data_size = msg.step * msg.height;
    msg.data.resize(data_size);
    std::memcpy(msg.data.data(), frame.get_data(), data_size);
    return msg;
  }

  void publish_camera_infos(const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_camera_info_) {
      return;
    }

    auto color_info = color_info_template_;
    color_info.header.stamp = stamp;
    color_info_pub_->publish(color_info);

    auto depth_info = depth_info_template_;
    depth_info.header.stamp = stamp;
    depth_info_pub_->publish(depth_info);
  }

  void capture_loop()
  {
    rs2::align align_to_color(RS2_STREAM_COLOR);

    while (rclcpp::ok() && running_.load()) {
      rs2::frameset frames;
      try {
        frames = pipeline_.wait_for_frames();
      } catch (const rs2::error & e) {
        if (!running_.load()) {
          break;
        }
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "RealSense wait_for_frames failed: %s", e.what());
        continue;
      }

      if (align_depth_) {
        frames = align_to_color.process(frames);
      }

      auto color_frame = frames.get_color_frame();
      auto depth_frame = frames.get_depth_frame();
      if (!color_frame || !depth_frame) {
        continue;
      }

      const builtin_interfaces::msg::Time stamp = this->get_clock()->now();
      auto color_msg = make_image_msg(
        color_frame, stamp, color_frame_id_, sensor_msgs::image_encodings::RGB8, 3);
      auto depth_msg = make_image_msg(
        depth_frame, stamp, depth_frame_id_, sensor_msgs::image_encodings::TYPE_16UC1, 2);

      color_pub_->publish(color_msg);
      depth_pub_->publish(depth_msg);
      publish_camera_infos(stamp);
    }
  }

  std::atomic<bool> running_;
  std::thread capture_thread_;

  rs2::pipeline pipeline_;
  rs2::config config_;
  rs2::pipeline_profile profile_;
  rs2::video_stream_profile color_profile_;
  rs2::video_stream_profile depth_profile_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

  sensor_msgs::msg::CameraInfo color_info_template_;
  sensor_msgs::msg::CameraInfo depth_info_template_;

  std::string serial_no_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string color_info_topic_;
  std::string depth_info_topic_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  int width_;
  int height_;
  int fps_;
  bool enable_auto_exposure_;
  bool emitter_enabled_;
  bool align_depth_;
  bool publish_camera_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RealSenseD455RgbdNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("realsense_d455_rgbd"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
