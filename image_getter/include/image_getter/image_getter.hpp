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
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "realtime_tools/realtime_publisher.hpp"

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
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>
  mImagePublisher_;
  cv::VideoCapture mCam_;
  rclcpp::TimerBase::SharedPtr mTimerCallback_;
  rclcpp::TimerBase::SharedPtr mPubCallback_;
  void processFrame();
  void publishFrame();
  std::mutex frameMutex;
  cv::Mat frame;
};
}  // namespace image_pipeline
