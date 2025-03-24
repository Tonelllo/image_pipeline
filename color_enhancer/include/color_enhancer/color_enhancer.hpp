#pragma once
/*
 * Copyright(2025) UNIGE
 */
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

#include <rclcpp/rclcpp.hpp>

namespace underwaterEnhancer {
class ColorEnhancer : public rclcpp::Node {
public:
  ColorEnhancer();

private:
  std::string mInTopic_;
  std::string mOutTopic_;
  std::string mAlgoritm_;
  cv_bridge::CvImagePtr mCvPtr_;
  cv::Mat mCurrentFrame_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mResPub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  void processImage(sensor_msgs::msg::Image::SharedPtr);
  std::unique_ptr<UDCP> mUdcp_;
  std::unique_ptr<SimpleEnhancer> mSeAvg_;
  std::unique_ptr<SimpleEnhancer> mSePca_;
  cv::Mat mEnhanced_;
  cv::Mat mSegmented_;
};
}  // namespace underwaterEnhancer
