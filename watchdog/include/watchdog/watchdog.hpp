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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include "realtime_tools/realtime_publisher.hpp"

namespace image_pipeline
{
class WatchDog : public rclcpp::Node
{
public:
  explicit WatchDog(const rclcpp::NodeOptions & options);
private:
  int mTimeoutMilliseconds_;
  std::vector<std::string> mTopicsToMonitor_;
  std::string mOutTopic_;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> mLatestTimestamps_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> mSubscribers_;
  rclcpp::TimerBase::SharedPtr mWatchDogTimer_;
  int mWatchDogCheckingRate_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>
  mHeartBeatPublisher_;
  void heartbeatPub();
};
}  // namespace image_pipeline
