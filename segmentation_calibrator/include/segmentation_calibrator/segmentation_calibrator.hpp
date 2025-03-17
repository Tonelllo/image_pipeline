#pragma once
/*
 * Copyright(2025)
 */

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/ucontext.h>

#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <filesystem>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace underwaterEnhancer {
class SegmentationCalibrator : public rclcpp::Node {
public:
  SegmentationCalibrator();
  ~SegmentationCalibrator();

private:
  YAML::Node mConfig_;
  std::string mPkgShare_;
  std::unique_ptr<std::ofstream> mWriter_;
  int mOption_;
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
  bool mSavePrompt_;

  void getFrame(sensor_msgs::msg::Image::SharedPtr);
  void saveParams();
  static void setup(int, void* obj){
    SegmentationCalibrator* mclass = static_cast<SegmentationCalibrator*>(obj);
    cv::Mat mask;
    cv::Mat process;
    cv::Mat segment;
    std::vector<std::vector<cv::Point>> contours;
    mclass->mCurrentFrame_.convertTo(process, CV_BGR2HSV);
    if (process.empty()) {
      return;
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
    std::string selection;
    switch (mclass->mOption_) {
    case 0:
      selection = "red buoy";
      break;
    case 1:
      selection = "black buoy";
      break;
    case 2:
      selection = "yellow buoy";
      break;
    case 3:
      selection = "orange buoy";
      break;
    case 4:
      selection = "white buoy";
      break;
    case 5:
      selection = "pipes";
      break;
    case 6:
      selection = "number";
      break;
    }
    cv::putText(segment, selection, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 5);
    char key = cv::waitKey(100);
    if (key == 's') {
      mclass->mSavePrompt_ = true;
    }
    if (mclass->mSavePrompt_) {
      cv::putText(segment, "Save params? (y/n)", cv::Point(10, 60),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 5);
      if(key == 'y'){
        mclass->saveParams();
        mclass->mSavePrompt_ = false;
      }else if(key == 'n'){
        mclass->mSavePrompt_ = false;
      }
    }
    cv::imshow("Segmentation Result", segment);
  }
};
}  // namespace underwaterEnhancer
