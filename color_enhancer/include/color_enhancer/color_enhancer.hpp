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

#include <vector>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/empty.hpp>
#include "simpleEnhancer/simpleEnhancer.hpp"
#include "udcp/udcp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

namespace underwaterEnhancer
{
class ColorEnhancer : public rclcpp::Node
{
public:
  ColorEnhancer();

private:
  std::string mInTopic_;
  std::string mOutTopic_;
  std::string mAlgoritm_;
  cv_bridge::CvImagePtr mCvPtr_;
  cv::Mat mCurrentFrame_;
  rclcpp::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>> mResPub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  void processImage(sensor_msgs::msg::Image::SharedPtr);
  bool mShowResult_;
  std::unique_ptr<UDCP> mUdcp_;
  std::unique_ptr<SimpleEnhancer> mSeAvg_;
  std::unique_ptr<SimpleEnhancer> mSePca_;
  cv::Mat mEnhanced_;
  cv::Mat mSegmented_;
};
}  // namespace underwaterEnhancer
