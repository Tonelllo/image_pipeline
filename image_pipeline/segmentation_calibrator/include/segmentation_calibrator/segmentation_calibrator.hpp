#pragma once
/*
 * Copyright(2025)
 */

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace underwaterEnhancer {
class SegmentationCalibrator : public rclcpp::Node {
public:
  SegmentationCalibrator();
private:
  cv_bridge::CvImagePtr mCvPtr_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mResPub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  int mHueMin_;
  int mSatMin_;
  int mValMin_;
  int mHueMax_;
  int mSatMax_;
  int mValMax_;
  void regulate(sensor_msgs::msg::Image::SharedPtr);
};
}  // namespace underwaterEnhancer
