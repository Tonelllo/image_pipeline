#pragma once
/*
 * Copyright(2025)
 */
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <rclcpp/rclcpp.hpp>

namespace underwaterEnhancer {
class BuoyDetector : public rclcpp::Node {
public:
  BuoyDetector();

private:
  std::string mInTopic_;
  std::string mOutTopic_;
  std::vector<int64_t> mRedBuoy_;
  std::vector<int64_t> mWhiteBuoy_;
  std::vector<int64_t> mBlackBuoy_;
  std::vector<int64_t> mOrangeBuoy_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  cv_bridge::CvImagePtr mCvPtr_;
  cv::Mat mCurrentFrame_;
  void getFrame(sensor_msgs::msg::Image::SharedPtr);
};
}  // namespace underwaterEnhancer
