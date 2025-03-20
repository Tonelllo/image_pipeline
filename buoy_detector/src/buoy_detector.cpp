/*
 * Copyright(2025)
 */
#include "buoy_detector/buoy_detector.hpp"

namespace underwaterEnhancer {
BuoyDetector::BuoyDetector():
  Node("pipe_detector", "/image_pipeline"){
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
  mCurrentFrame_ = mCvPtr_->image;
}
}  // namespace underwaterEnhancer

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::BuoyDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
