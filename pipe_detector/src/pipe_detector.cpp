/*
 * Copyright(2025)
 */
#include "pipe_detector/pipe_detector.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace underwaterEnhancer {
PipeDetector::PipeDetector() :
  Node("pipe_detector", "/image_pipeline"){
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("hue_min", 0);
  declare_parameter("val_min", 0);
  declare_parameter("sat_min", 0);
  declare_parameter("hue_max", 255);
  declare_parameter("sat_max", 255);
  declare_parameter("val_max", 255);

  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mHueMin_ = get_parameter("hue_min").as_int();
  mSatMin_ = get_parameter("sat_min").as_int();
  mValMin_ = get_parameter("val_min").as_int();
  mHueMax_ = get_parameter("hue_max").as_int();
  mSatMax_ = get_parameter("sat_max").as_int();
  mValMax_ = get_parameter("val_max").as_int();

  mInSub_ = create_subscription<sensor_msgs::msg::Image>
              (mInTopic_, 10,
              std::bind(&PipeDetector::getFrame, this, std::placeholders::_1));
  mOutPub_ = create_publisher<std_msgs::msg::Float32>(mOutTopic_, 10);
}

void PipeDetector::getFrame(sensor_msgs::msg::Image::SharedPtr img){
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  mCurrentFrame_ = mCvPtr_->image;
  cv::Mat mask;
  cv::Mat process;
  cv::Mat segment;
  std::vector<std::vector<cv::Point>> contours;
  mCurrentFrame_.convertTo(process, CV_BGR2HSV);
  if (process.empty()) {
    return;
  }

  cv::inRange(process,
              cv::Scalar(mHueMin_, mSatMin_, mValMin_),
              cv::Scalar(mHueMax_, mSatMax_, mValMax_),
              mask);
  cv::bitwise_and(mCurrentFrame_, mCurrentFrame_, segment, mask);
  cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  std::vector<cv::RotatedRect> minEllipse( contours.size());
  for (size_t i = 0; i < contours.size(); i++) {
    if (contours[i].size() > 5) {
      minEllipse[i] = cv::fitEllipse(contours[i]);
    }
  }
  cv::Scalar color = cv::Scalar( 200, 200, 200);
  cv::Point2f maxP1;
  cv::Point2f maxP2;
  uint64 currentMaxSize = 0;
  for (size_t i = 0; i< contours.size(); i++) {
    cv::ellipse(segment, minEllipse[i], color, 2);
    cv::Point2f center = minEllipse[i].center;
    cv::Size2f axes = minEllipse[i].size;
    auto size = minEllipse[i].size.height * minEllipse[i].size.width;
    float angle = minEllipse[i].angle;

    // Calculate the two extreme points on the ellipse's major axis
    cv::Point2f point1(center.x - axes.height / 2 * sin(angle * CV_PI / 180.0),
                       center.y - axes.height / 2 * -cos(angle * CV_PI / 180.0));
    cv::Point2f point2(center.x + axes.height / 2 * sin(angle * CV_PI / 180.0),
                       center.y + axes.height / 2 * -cos(angle * CV_PI / 180.0));

    if (size > currentMaxSize) {
      maxP1 = point1;
      maxP2 = point2;
      currentMaxSize = size;
    }
    // Draw the longest line (major axis) of the ellipse
    cv::line(segment, point1, point2, cv::Scalar(0, 0, 255), 2);
  }
  cv::circle(segment, maxP1, 3, cv::Scalar(255, 0, 0), 7);
  std::cout << maxP1 << std::endl;
  /*std::cout << maxP1 << "\t" << maxP2 << std::endl;*/
  cv::imshow("res", segment);
  cv::waitKey(10);
  /*std_msgs::msg::Float32 angle;*/
  /*angle.data = 10.0;*/
  /*mOutPub_->publish(angle);*/
}
}  // namespace underwaterEnhancer

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::PipeDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
