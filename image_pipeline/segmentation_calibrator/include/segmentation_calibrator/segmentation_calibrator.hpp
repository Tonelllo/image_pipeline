#pragma once
/*
 * Copyright(2025)
 */

#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace underwaterEnhancer {
class SegmentationCalibrator : public rclcpp::Node {
public:
  SegmentationCalibrator();
  ~SegmentationCalibrator();

private:
  cv_bridge::CvImagePtr mCvPtr_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mInSub_;
  cv::Mat mCurrentFrame_;
  std::string mInTopic_;
  int mHueMin_;
  int mSatMin_;
  int mValMin_;
  int mHueMax_;
  int mSatMax_;
  int mValMax_;

  void getFrame(sensor_msgs::msg::Image::SharedPtr);
  static void setup(int, void* obj){
    SegmentationCalibrator* mclass = static_cast<SegmentationCalibrator*>(obj);
    cv::Mat mask;
    cv::Mat process;
    cv::Mat segment;
    std::vector<std::vector<cv::Point>> contours;
    mclass->mCurrentFrame_.convertTo(process, CV_BGR2HSV);
    if(process.empty()){
      std::cout << "error" << std::endl;
      return;
    }else{
      std::cout << "cane" << std::endl;
    }
    cv::inRange(process,
                cv::Scalar(mclass->mHueMin_, mclass->mSatMin_, mclass->mValMin_),
                cv::Scalar(mclass->mHueMax_, mclass->mSatMax_, mclass->mValMax_),
                mask);
    cv::bitwise_and(mclass->mCurrentFrame_, mclass->mCurrentFrame_, segment, mask);
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
