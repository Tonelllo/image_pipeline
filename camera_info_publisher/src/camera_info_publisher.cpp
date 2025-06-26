#include "camera_info_publisher/camera_info_publisher.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

namespace image_pipeline {
CameraInfoPublisher::CameraInfoPublisher(const rclcpp::NodeOptions & options)
  : Node("camera_info_publisher", options)
{
  declare_parameter("timer_period_milliseconds", 0);
  declare_parameter("camera_info_out", "UNSET");
  declare_parameter("heartbeat_rate", 0);
  declare_parameter("heartbeat_topic", "UNSET");
  mTimerPeriod_ = get_parameter("timer_period_milliseconds").as_int();
  mCameraInfoTopic_ = get_parameter("camera_info_out").as_string();

  mHeartBeatTopic_ = get_parameter("heartbeat_topic").as_string();
  mHeartBeatRate_ = get_parameter("heartbeat_rate").as_int();
  mHeartBeatPubisher_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::msg::Empty>(
      create_publisher<std_msgs::msg::Empty>(
        mHeartBeatTopic_,
        1
        )
      )
    );
  mHeartBeatTimer_ = create_wall_timer(
    std::chrono::milliseconds(mHeartBeatRate_),
    [this](){
    if (mHeartBeatPubisher_->trylock()) {
      mHeartBeatPubisher_->unlockAndPublish();
    }
  });

  std::filesystem::path filePath = ament_index_cpp::get_package_prefix("image_pipeline_camera_info_publisher");
  filePath = filePath / "share" / "image_pipeline_camera_info_publisher" / "params" /
             "camera_params.yaml";
  mCameraInfoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "rov_cam",
    "file://" + filePath.string());

  mCameraInfoTimer_ = create_wall_timer(
    std::chrono::milliseconds(mTimerPeriod_),
    std::bind(&CameraInfoPublisher::cameraInfoPublisher, this));

  mCameraInfoPublisher_.reset(
    new realtime_tools::RealtimePublisher<sensor_msgs::msg::CameraInfo>(
      create_publisher<sensor_msgs::msg::CameraInfo>(
        mCameraInfoTopic_,
        1
        )
      )
    );
}

void CameraInfoPublisher::cameraInfoPublisher(){
  if (mCameraInfoManager_->isCalibrated()) {
    if (mCameraInfoPublisher_->trylock()) {
      mCameraInfoPublisher_->msg_ = mCameraInfoManager_->getCameraInfo();
      mCameraInfoPublisher_->unlockAndPublish();
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Camera calibration params is not beeing correctly read");
  }
}

}  // namespace image_pipeline
RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::CameraInfoPublisher);
