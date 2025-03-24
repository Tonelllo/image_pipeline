#pragma once
/*
 * Copyright(2025)
 */
#include <cv_bridge/cv_bridge.h>
#include <string>

#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <image_pipeline_msgs/msg/pipe_line.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

namespace underwaterEnhancer {
class PipeDetector : public rclcpp::Node {
public:
  PipeDetector();

private:
  std::string mInTopic_;
  std::string mOutTopic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  rclcpp::Publisher<image_pipeline_msgs::msg::PipeLine>::SharedPtr mOutPub_;
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
