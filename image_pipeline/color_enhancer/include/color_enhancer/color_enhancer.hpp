#pragma once
/*
 * Copyright(2025)
 */
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/empty.hpp>
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
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mSetupSrv_;
  void processImage(sensor_msgs::msg::Image::SharedPtr);
  void toggleSetup(const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr);
  void setupParameters();
  UDCP mUdcp_;
  cv::Mat mEnhanced_;
  cv::Mat mSegmented_;
  bool mSetup_;
  int mHueMin_;
  int mSatMin_;
  int mValMin_;
  int mHueMax_;
  int mSatMax_;
  int mValMax_;

  static void trackbarCallback(int, void* obj){
    ColorEnhancer* mclass = static_cast<ColorEnhancer*>(obj);
    cv::Mat mask;
    cv::Mat process;
    cv::Mat segment;
    std::vector<std::vector<cv::Point>> contours;
    mclass->mEnhanced_.convertTo(process, CV_BGR2HSV);
    cv::inRange(process,
                cv::Scalar(mclass->mHueMin_, mclass->mSatMin_, mclass->mValMin_),
                cv::Scalar(mclass->mHueMax_, mclass->mSatMax_, mclass->mValMax_),
                mask);
    cv::bitwise_and(mclass->mEnhanced_, mclass->mEnhanced_, segment, mask);
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<cv::RotatedRect> minEllipse( contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
      if (contours[i].size() > 5) {
        minEllipse[i] = cv::fitEllipse(contours[i]);
      }
    }
    cv::Scalar color = cv::Scalar( 200, 200, 200);
    for (size_t i = 0; i< contours.size(); i++) {
      cv::ellipse(segment, minEllipse[i], color, 2);
      cv::Point2f center = minEllipse[i].center;
      cv::Size2f axes = minEllipse[i].size;
      float angle = minEllipse[i].angle;

      // Calculate the two extreme points on the ellipse's major axis
      cv::Point2f point1(center.x - axes.height / 2 * sin(angle * CV_PI / 180.0),
                         center.y - axes.height / 2 * -cos(angle * CV_PI / 180.0));
      cv::Point2f point2(center.x + axes.height / 2 * sin(angle * CV_PI / 180.0),
                         center.y + axes.height / 2 * -cos(angle * CV_PI / 180.0));

      // Draw the longest line (major axis) of the ellipse
      cv::line(segment, point1, point2, cv::Scalar(0, 0, 255), 2);
    }

    cv::imshow("Segmentation Result", segment);
    cv::waitKey(100);
    /*mclass->mHue_;*/
  }
};
}  // namespace underwaterEnhancer
