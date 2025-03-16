/*
 * Copyright(2025)
 */
#include <segmentation_calibrator/segmentation_calibrator.hpp>

namespace underwaterEnhancer {
SegmentationCalibrator::SegmentationCalibrator() :
  Node("segmentation_calibrator", "/image_pipeline"){
  mResPub_ = create_publisher<sensor_msgs::msg::Image>("/segmentation_calibrator/out", 10);
  mInSub_ = create_subscription<sensor_msgs::msg::Image>
              ("/image_enhancer/in", 10,
               std::bind(&SegmentationCalibrator::regulate, this, std::placeholders::_1));

  mHueMin_ = 0;
  mSatMin_ = 0;
  mValMin_ = 0;
  mHueMax_ = 179;
  mSatMax_ = 255;
  mValMax_ = 255;
}
void SegmentationCalibrator::regulate(sensor_msgs::msg::Image::SharedPtr){
}
}  // namespace underwaterEnhancer

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::SegmentationCalibrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
