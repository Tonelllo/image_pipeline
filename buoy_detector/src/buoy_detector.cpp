/*
 * Copyright(2025)
 */
#include "buoy_detector/buoy_detector.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace underwaterEnhancer {
BuoyDetector::BuoyDetector() :
  Node("buoy_detector", "/image_pipeline"){
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("red_buoy", std::vector<int64_t>());
  declare_parameter("white_buoy", std::vector<int64_t>());
  declare_parameter("yellow_buoy", std::vector<int64_t>());
  declare_parameter("black_buoy", std::vector<int64_t>());
  declare_parameter("orange_buoy", std::vector<int64_t>());

  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mRedBuoy_ = get_parameter("red_buoy").as_integer_array();
  mWhiteBuoy_ = get_parameter("white_buoy").as_integer_array();
  mBlackBuoy_ = get_parameter("black_buoy").as_integer_array();
  mOrangeBuoy_ = get_parameter("orange_buoy").as_integer_array();
  mYellowBuoy_ = get_parameter("yellow_buoy").as_integer_array();

  std::cout << mInTopic_ << std::endl;
  mInSub_ = create_subscription<sensor_msgs::msg::Image>
              (mInTopic_, 10,
              std::bind(&BuoyDetector::getFrame, this, std::placeholders::_1));
}

void BuoyDetector::getFrame(sensor_msgs::msg::Image::SharedPtr img){
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  cv::Mat mask;
  cv::Mat segment;
  mCurrentFrame_ = mCvPtr_->image;
  cv::inRange(mCurrentFrame_,
              cv::Scalar(mRedBuoy_[0], mRedBuoy_[1], mRedBuoy_[2]),
              cv::Scalar(mRedBuoy_[3], mRedBuoy_[4], mRedBuoy_[5]),
              mask);
  cv::bitwise_and(mCurrentFrame_, mCurrentFrame_, segment, mask);
  cv::medianBlur(mask, mask, 5);
  std::vector<cv::Vec3f> circles;
  HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1,
               mask.rows/16,
               100, 30, 1, 1000
               // (min_radius & max_radius) to detect larger circles
               );
  for (size_t i = 0; i < circles.size(); i++)
  {
    cv::Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    cv::circle( mCurrentFrame_, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    cv::circle( mCurrentFrame_, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
  }
  cv::imshow("buoy", mCurrentFrame_);
  cv::waitKey(100);
}
}  // namespace underwaterEnhancer

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::BuoyDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
