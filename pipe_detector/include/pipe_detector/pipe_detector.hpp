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
#include <string>

#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <image_pipeline_msgs/msg/pipe_direction.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

namespace underwaterEnhancer
{
class PipeDetector : public rclcpp::Node
{
public:
  PipeDetector();

private:
  bool mFlipDirection_;
  cv::Point2f mPrevDir_;
  std::string mInTopic_;
  std::string mOutTopic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  rclcpp::Publisher<image_pipeline_msgs::msg::PipeDirection>::SharedPtr mOutPub_;
  cv_bridge::CvImagePtr mCvPtr_;
  cv::Mat mCurrentFrame_;
  void getFrame(sensor_msgs::msg::Image::SharedPtr);
  int mHueMin_;
  int mSatMin_;
  int mValMin_;
  int mHueMax_;
  int mSatMax_;
  int mValMax_;
  bool mShowResult_;
};
}  // namespace underwaterEnhancer
