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
#include <vector>

#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <image_pipeline_msgs/msg/buoy_position_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace underwaterEnhancer
{
class BuoyDetector : public rclcpp::Node
{
public:
  BuoyDetector();

private:
  std::string mInTopic_;
  std::string mOutTopic_;
  std::vector<int64_t> mRedBuoy_;
  std::vector<int64_t> mWhiteBuoy_;
  std::vector<int64_t> mBlackBuoy_;
  std::vector<int64_t> mOrangeBuoy_;
  std::vector<int64_t> mYellowBuoy_;
  int64_t mMinBuoySize_;
  int64_t mMaxBuoySize_;
  int64_t mBlobDilationSize_;
  int64_t mBlobMedianBlurSize_;
  bool mShowResult_;
  std::vector<std::vector<int64_t>> mBuoysParams_;
  std::array<std::string, 5> mBuoysNames_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  rclcpp::Publisher<image_pipeline_msgs::msg::BuoyPositionArray>::SharedPtr mBuoysPub_;
  cv_bridge::CvImagePtr mCvPtr_;
  cv::Mat mCurrentFrame_;
  cv::Ptr<cv::SimpleBlobDetector> mSbd_;
  void getFrame(sensor_msgs::msg::Image::SharedPtr);
};
}  // namespace underwaterEnhancer
