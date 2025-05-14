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
#include "tensorrt_engine/yolov8.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cuda_engine/inference.hpp>

namespace ai {
class YoloModel : public rclcpp::Node {
public:
  YoloModel();
private:
  std::string mEngine_;
  std::string mInTopic_;
  std::string mOutTopic_;
  std::string mModelPath_;
  std::vector<std::string> mClasses_;
  void processFrame(sensor_msgs::msg::Image::SharedPtr);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mOutPub_;
  cv_bridge::CvImagePtr mCvPtr_;
  std::unique_ptr<Inference> inf;
  YoloV8Config mConfig_;
  std::string mTrtModelPath_;
  std::unique_ptr<YoloV8> mYolo_;
};
}  // namespace ai
