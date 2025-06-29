// Copyright 2025 UNIGE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "image_getter/image_getter.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/publisher.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/types.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <image_transport/image_transport.hpp>

namespace image_pipeline
{
ImageGetter::ImageGetter(const rclcpp::NodeOptions & options)
  : Node("image_getter", options)
{
  declare_parameter("heartbeat_rate", 0);
  declare_parameter("heartbeat_topic", "UNSET");
  declare_parameter("image_topic", "UNSET");
  declare_parameter("timer_period", 0);
  mHeartBeatTopic_ = get_parameter("heartbeat_topic").as_string();
  mImageTopic_ = get_parameter("image_topic").as_string();
  mHeartBeatRate_ = get_parameter("heartbeat_rate").as_int();
  mTimerPeriod_ = get_parameter("timer_period").as_int();

  rclcpp::QoS customQos(1);
  customQos.best_effort();
  customQos.durability_volatile();


  // mImagePublisher_.reset(
  //   new realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>(
  //     create_publisher<sensor_msgs::msg::Image>(
  //       mImageTopic_,
  //       customQos
  //       )
  //     )
  //   );

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


  pubReady_ = false;

  std::string gst_str =
    "udpsrc port=5600 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";

  // mCam_ = cv::VideoCapture(gst_str, cv::CAP_GSTREAMER);
  mCam_ = cv::VideoCapture("/home/tonello/Downloads/3-orbit.mkv");

  if (!mCam_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "UNABLE TO OPEN CAMERA STREAM");
  }
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    std::bind(&ImageGetter::postInit, this));

  mTimerCallback_ = create_wall_timer(
    std::chrono::milliseconds(mTimerPeriod_),
    std::bind(&ImageGetter::processFrame, this));
}

void ImageGetter::postInit()
{
  // Cancel timer after first run
  this->timer_->cancel();

  // Now it's safe to use shared_from_this()
  image_transport::ImageTransport it(shared_from_this());


  rmw_qos_profile_t customProfile;
  customProfile.depth = 1;
  customProfile.durability = rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_VOLATILE;
  customProfile.reliability = rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  customProfile.liveliness = rmw_qos_liveliness_policy_e::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  customProfile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  customProfile.liveliness_lease_duration = {0, 0};  // Zero means default
  customProfile.deadline = {0, 0};
  customProfile.lifespan = {0, 0};
  testPublisher = it.advertise("image_out", customProfile);
  pubReady_ = true;
}

void ImageGetter::processFrame(){
  cv::Mat frame;
  if (!mCam_.read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to grab frame");
    return;
  }

  std_msgs::msg::Header hdr;
  hdr.frame_id = "camera";
  hdr.stamp = now();

  // if (mImagePublisher_->trylock()) {
  //   mImagePublisher_->msg_ = *cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
  //   mImagePublisher_->unlockAndPublish();
  // }
  if(!pubReady_){
    return;
  }
  testPublisher.publish(*cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::ImageGetter);

