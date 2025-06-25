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
#include <rclcpp/rclcpp.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <filesystem>
#include <realtime_tools/realtime_publisher.hpp>
namespace image_pipeline {
class CameraInfoPublisher : public rclcpp::Node {
public:
  CameraInfoPublisher(const rclcpp::NodeOptions & options);
private:
  std::shared_ptr<camera_info_manager::CameraInfoManager> mCameraInfoManager_;
  rclcpp::TimerBase::SharedPtr mCameraInfoTimer_;
  int mTimerPeriod_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::CameraInfo>>
  mCameraInfoPublisher_;
  std::string mCameraInfoTopic_;
  void cameraInfoPublisher();
};
} //namespace image_pipeline
