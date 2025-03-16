/*
 * Copyright(2025)
 */
#include <segmentation_calibrator/segmentation_calibrator.hpp>

namespace underwaterEnhancer {
SegmentationCalibrator::SegmentationCalibrator() :
  Node("segmentation_calibrator", "/image_pipeline"){
  declare_parameter("in_topic", "UNSET");

  mInTopic_ = get_parameter("in_topic").as_string();

  mInSub_ = create_subscription<sensor_msgs::msg::Image>
              (mInTopic_, 10,
              std::bind(&SegmentationCalibrator::getFrame, this, std::placeholders::_1));
  mHueMin_ = 0;
  mSatMin_ = 0;
  mValMin_ = 0;
  mHueMax_ = 255;
  mSatMax_ = 255;
  mValMax_ = 255;


  cv::namedWindow("Segmentation Result");
  cv::createTrackbar("Hue min", "Segmentation Result",
                     &mHueMin_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar("Sat min", "Segmentation Result",
                     &mValMin_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar("Val min", "Segmentation Result",
                     &mSatMin_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar("Hue max", "Segmentation Result",
                     &mHueMax_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar("Sat max", "Segmentation Result",
                     &mValMax_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar("Val max", "Segmentation Result",
                     &mSatMax_, 255, &SegmentationCalibrator::setup, this);
}

SegmentationCalibrator::~SegmentationCalibrator(){
  cv::destroyAllWindows();
}

void SegmentationCalibrator::getFrame(sensor_msgs::msg::Image::SharedPtr img){
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  mCurrentFrame_ = mCvPtr_->image;
  setup(0, this);
}
}  // namespace underwaterEnhancer

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::SegmentationCalibrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
