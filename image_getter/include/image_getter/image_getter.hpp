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

#pragma once
#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/publisher.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "realtime_tools/realtime_publisher.hpp"
#include <image_transport/image_transport.hpp>

namespace image_pipeline
{
class ImageGetter : public rclcpp::Node
{
public:
  explicit ImageGetter(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr mHeartBeatTimer_;
  std::string mHeartBeatTopic_;
  std::string mImageTopic_;
  int mTimerPeriod_;
  int mHeartBeatRate_;

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Empty>>
  mHeartBeatPubisher_;
  image_transport::Publisher mImagePublisher_;
  cv::VideoCapture mCam_;
  rclcpp::TimerBase::SharedPtr mTimerCallback_;
  rclcpp::TimerBase::SharedPtr mStartTimer_;
  void processFrame();
  void postInit();
};
}  // namespace image_pipeline
