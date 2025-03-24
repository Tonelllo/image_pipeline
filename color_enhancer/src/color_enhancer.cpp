/*
 * Copyright(2025) UNIGE
 */

#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui.hpp>
#include <color_enhancer/color_enhancer.hpp>

namespace underwaterEnhancer {
ColorEnhancer::ColorEnhancer() : Node("color_enhancer", "/image_pipeline"){
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("algorithm", "UNSET");

  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mAlgoritm_ = get_parameter("algorithm").as_string();

  mUdcp_ = std::make_unique<UDCP>(false, 25);
  mSeAvg_ = std::make_unique<SimpleEnhancer>(false, SimpleEnhancer::fusionMode_::AVG);
  mSePca_ = std::make_unique<SimpleEnhancer>(false, SimpleEnhancer::fusionMode_::PCA);

  mResPub_ = create_publisher<sensor_msgs::msg::Image>(mOutTopic_, 10);
  mInSub_ = create_subscription<sensor_msgs::msg::Image>
              (mInTopic_, 10, std::bind(&ColorEnhancer::processImage, this, std::placeholders::_1));
}

void ColorEnhancer::processImage(sensor_msgs::msg::Image::SharedPtr img){
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }

  // Converting the image in black and white to have better detection of the arucos
  mCurrentFrame_ = mCvPtr_->image;
  if (mAlgoritm_ == "udcp") {
    mEnhanced_ = mUdcp_->enhance(mCurrentFrame_);
  } else if (mAlgoritm_ == "se_avg") {
    mEnhanced_ = mSeAvg_->enhance(mCurrentFrame_);
  } else if (mAlgoritm_ == "se_pca") {
    mEnhanced_ = mSeAvg_->enhance(mCurrentFrame_);
  }
  cv::Mat tmp;
  mCvPtr_->image = mEnhanced_;
  cv::waitKey(10);
  mResPub_->publish(*mCvPtr_->toImageMsg());
}
}  // namespace underwaterEnhancer

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::ColorEnhancer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
