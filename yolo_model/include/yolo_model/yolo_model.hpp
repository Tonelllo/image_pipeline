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

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cuda_engine/inference.hpp>
#include "realtime_tools/realtime_publisher.hpp"
#include <image_pipeline_msgs/msg/bounding_box2_d_array.hpp>
#include <std_msgs/msg/empty.hpp>

namespace image_pipeline {
class YoloModel : public rclcpp::Node {
public:
  explicit YoloModel(const rclcpp::NodeOptions & options);
private:
  std::string mEngine_;
  std::string mInTopic_;
  std::string mOutTopic_;
  std::string mOutDetectionTopic_;
  std::string mModelPath_;
  std::vector<std::string> mClasses_;
  void processFrame(sensor_msgs::msg::Image::SharedPtr);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>> mOutPub_;
  std::unique_ptr<realtime_tools::RealtimePublisher
  <image_pipeline_msgs::msg::BoundingBox2DArray>> mOutDetectionPub_;
  cv_bridge::CvImagePtr mCvPtr_;
  std::unique_ptr<Inference> inf;
  std::string mTrtModelPath_;
  bool mSaveDetections_;

  rclcpp::TimerBase::SharedPtr mHeartBeatTimer_;
  std::string mHeartBeatTopic_;
  int mHeartBeatRate_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Empty>>
  mHeartBeatPubisher_;
};
}  // namespace image_pipeline
