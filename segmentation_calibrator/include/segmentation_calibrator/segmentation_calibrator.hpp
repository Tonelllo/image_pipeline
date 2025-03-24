#pragma once
/*
 * Copyright(2025) UNIGE
 */

#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/ucontext.h>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <filesystem>

#include <rclcpp/logging.hpp>
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
  int mPrevVal_;
  bool mSavePrompt_;

  void getFrame(sensor_msgs::msg::Image::SharedPtr);
  void saveParams();
  static void setup(int, void* obj){
    SegmentationCalibrator* mclass = static_cast<SegmentationCalibrator*>(obj);
    cv::Mat mask;
    cv::Mat process;
    cv::Mat segment;
    std::vector<std::vector<cv::Point>> contours;
    if (mclass->mCurrentFrame_.empty()) {
      return;
    }
    cv::cvtColor(mclass->mCurrentFrame_, process, CV_BGR2HSV);
    cv::inRange(process,
                cv::Scalar(mclass->mHueMin_, mclass->mSatMin_, mclass->mValMin_),
                cv::Scalar(mclass->mHueMax_, mclass->mSatMax_, mclass->mValMax_),
                mask);
    cv::bitwise_and(mclass->mCurrentFrame_, mclass->mCurrentFrame_, segment, mask);

    std::string selection;
    bool restore = false;
    if (mclass->mPrevVal_ != mclass->mOption_) {
      restore = true;
    }
    switch (mclass->mOption_) {
    case 0:
      selection = "red_buoy";
      break;
    case 1:
      selection = "black_buoy";
      break;
    case 2:
      selection = "yellow_buoy";
      break;
    case 3:
      selection = "orange_buoy";
      break;
    case 4:
      selection = "white_buoy";
      break;
    case 5:
      selection = "pipes";
      break;
    case 6:
      selection = "number";
      break;
    }
    if (mclass->mOption_ <= 4 && restore) {
      auto vec = mclass->mConfig_["image_pipeline/buoy_detector"]["ros__parameters"][selection];
      std::vector<int> vals;
      for (const auto & v : vec) {
        vals.push_back(v.as<int>());
      }
      mclass->mHueMin_ = vals[0];
      mclass->mSatMin_ = vals[1];
      mclass->mValMin_ = vals[2];
      mclass->mHueMax_ = vals[3];
      mclass->mSatMax_ = vals[4];
      mclass->mValMax_ = vals[5];
      cv::setTrackbarPos("Hue min", "Segmentation Result", mclass->mHueMin_);
      cv::setTrackbarPos("Sat min", "Segmentation Result", mclass->mSatMin_);
      cv::setTrackbarPos("Val min", "Segmentation Result", mclass->mValMin_);
      cv::setTrackbarPos("Hue max", "Segmentation Result", mclass->mHueMax_);
      cv::setTrackbarPos("Sat max", "Segmentation Result", mclass->mSatMax_);
      cv::setTrackbarPos("Val max", "Segmentation Result", mclass->mValMax_);
      restore = false;
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
      if (key == 'y') {
        mclass->saveParams();
        mclass->mSavePrompt_ = false;
      } else if (key == 'n') {
        mclass->mSavePrompt_ = false;
      }
    }
    mclass->mPrevVal_ = mclass->mOption_;
    cv::imshow("Segmentation Result", segment);
  }
};
}  // namespace underwaterEnhancer
