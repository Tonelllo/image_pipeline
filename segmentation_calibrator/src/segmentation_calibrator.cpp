// Copyright 2025 UNIGE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <opencv2/highgui.hpp>
#include <segmentation_calibrator/segmentation_calibrator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace image_pipeline
{
SegmentationCalibrator::SegmentationCalibrator(const rclcpp::NodeOptions & options)
: Node("segmentation_calibrator", options)
{
  declare_parameter("in_topic", "UNSET");

  mInTopic_ = get_parameter("in_topic").as_string();

  mInSub_ = create_subscription<sensor_msgs::msg::Image>(
    mInTopic_, 10,
    std::bind(&SegmentationCalibrator::getFrame, this, std::placeholders::_1));
  mHueMin_ = 0;
  mSatMin_ = 0;
  mValMin_ = 0;
  mHueMax_ = 255;
  mSatMax_ = 255;
  mValMax_ = 255;
  mOption_ = 0;

  mSavePrompt_ = false;
  mPkgShare_ = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("image_pipeline_launcher")) /
    "../../../../src/image_pipeline/launcher" /
    "params" / "config.yaml";
  mConfig_ = YAML::LoadFile(mPkgShare_);

  cv::namedWindow("Segmentation Result");
  cv::createTrackbar(
    "Hue min", "Segmentation Result",
    &mHueMin_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar(
    "Sat min", "Segmentation Result",
    &mSatMin_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar(
    "Val min", "Segmentation Result",
    &mValMin_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar(
    "Hue max", "Segmentation Result",
    &mHueMax_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar(
    "Sat max", "Segmentation Result",
    &mSatMax_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar(
    "Val max", "Segmentation Result",
    &mValMax_, 255, &SegmentationCalibrator::setup, this);
  cv::createTrackbar(
    "Calibrating for", "Segmentation Result",
    &mOption_, 6, &SegmentationCalibrator::setup, this);
}

SegmentationCalibrator::~SegmentationCalibrator()
{
  cv::destroyAllWindows();
}

void SegmentationCalibrator::getFrame(sensor_msgs::msg::Image::SharedPtr img)
{
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  mCurrentFrame_ = mCvPtr_->image;
  setup(0, this);
}

void SegmentationCalibrator::saveParams()
{
  std::ofstream writer(mPkgShare_);
  std::array<int, 6> vals = {mHueMin_, mSatMin_, mValMin_, mHueMax_, mSatMax_, mValMax_};
  if (mOption_ <= 4) {
    std::string buoyType;
    switch (mOption_) {
      case 0:
        // "red buoy"
        buoyType = "red_buoy";
        break;
      case 1:
        // "black buoy"
        buoyType = "black_buoy";
        break;
      case 2:
        // "yellow buoy"
        buoyType = "yellow_buoy";
        break;
      case 3:
        // "orange buoy"
        buoyType = "orange_buoy";
        break;
      case 4:
        // "white buoy"
        buoyType = "white_buoy";
        break;
    }
    mConfig_["image_pipeline/buoy_detector"]["ros__parameters"][buoyType] = vals;
  } else {
    switch (mOption_) {
      case 5:
        mConfig_["image_pipeline/pipe_detector"]["ros__parameters"]["hue_min"] = mHueMin_;
        mConfig_["image_pipeline/pipe_detector"]["ros__parameters"]["sat_min"] = mSatMin_;
        mConfig_["image_pipeline/pipe_detector"]["ros__parameters"]["val_min"] = mValMin_;
        mConfig_["image_pipeline/pipe_detector"]["ros__parameters"]["hue_max"] = mHueMax_;
        mConfig_["image_pipeline/pipe_detector"]["ros__parameters"]["sat_max"] = mSatMax_;
        mConfig_["image_pipeline/pipe_detector"]["ros__parameters"]["val_max"] = mValMax_;
        // "pipes"
        break;
      case 6:
        // "number"
        break;
    }
  }
  writer << mConfig_;
  writer.close();
}
}  // namespace image_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::SegmentationCalibrator)
