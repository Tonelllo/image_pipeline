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

#include "buoy_detector/buoy_detector.hpp"
#include <algorithm>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace underwaterEnhancer
{
BuoyDetector::BuoyDetector()
: Node("buoy_detector", "/image_pipeline")
{
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("red_buoy", std::vector<int64_t>());
  declare_parameter("white_buoy", std::vector<int64_t>());
  declare_parameter("yellow_buoy", std::vector<int64_t>());
  declare_parameter("black_buoy", std::vector<int64_t>());
  declare_parameter("orange_buoy", std::vector<int64_t>());
  declare_parameter("min_buoy_radius_px", 1);
  declare_parameter("max_buoy_radius_px", 300);
  declare_parameter("blob_dilation_size", 15);
  declare_parameter("blob_median_blur_size", 15);
  declare_parameter("show_result", false);

  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mRedBuoy_ = get_parameter("red_buoy").as_integer_array();
  mWhiteBuoy_ = get_parameter("white_buoy").as_integer_array();
  mBlackBuoy_ = get_parameter("black_buoy").as_integer_array();
  mOrangeBuoy_ = get_parameter("orange_buoy").as_integer_array();
  mYellowBuoy_ = get_parameter("yellow_buoy").as_integer_array();
  mMinBuoySize_ = get_parameter("min_buoy_radius_px").as_int();
  mMaxBuoySize_ = get_parameter("max_buoy_radius_px").as_int();
  mBlobDilationSize_ = get_parameter("blob_dilation_size").as_int();
  mBlobMedianBlurSize_ = get_parameter("blob_median_blur_size").as_int();
  mShowResult_ = get_parameter("show_result").as_bool();

  mInSub_ = create_subscription<sensor_msgs::msg::Image>(
    mInTopic_, 10,
    std::bind(&BuoyDetector::getFrame, this, std::placeholders::_1));

  mBuoysPub_.reset(
    new realtime_tools::RealtimePublisher<image_pipeline_msgs::msg::BuoyPositionArray>(
      create_publisher<image_pipeline_msgs::msg::BuoyPositionArray>(
        mOutTopic_, 1
      )
    )
  )

  mBuoysParams_.push_back(mRedBuoy_);
  mBuoysParams_.push_back(mWhiteBuoy_);
  mBuoysParams_.push_back(mBlackBuoy_);
  mBuoysParams_.push_back(mOrangeBuoy_);
  mBuoysParams_.push_back(mYellowBuoy_);

  mBuoysNames_ = {"RED", "WHITE", "BLACK", "ORANGE", "YELLOW"};

  cv::SimpleBlobDetector::Params params;
  params.filterByColor = true;
  params.blobColor = 255;
  params.filterByArea = true;
  params.minArea = CV_PI * mMinBuoySize_ * mMinBuoySize_;
  params.maxArea = CV_PI * mMaxBuoySize_ * mMaxBuoySize_;
  params.filterByCircularity = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;
  mSbd_ = cv::SimpleBlobDetector::create(params);
}

void BuoyDetector::getFrame(sensor_msgs::msg::Image::SharedPtr img)
{
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  cv::Mat mask;
  cv::Mat kernel;
  cv::Mat segment;
  cv::Mat process;
  cv::Mat out;
  image_pipeline_msgs::msg::BuoyPositionArray ret;
  kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(
      2 * mBlobDilationSize_ + 1,
      2 * mBlobDilationSize_ + 1),
    cv::Point(mBlobDilationSize_, mBlobDilationSize_));
  mCurrentFrame_ = mCvPtr_->image;
  out = mCurrentFrame_;
  for (size_t index = 0; index < mBuoysNames_.size(); index++) {
    image_pipeline_msgs::msg::BuoyPosition bp;
    cv::cvtColor(mCurrentFrame_, process, CV_BGR2HSV);
    cv::inRange(
      process,
      cv::Scalar(
        mBuoysParams_[index][0],
        mBuoysParams_[index][1],
        mBuoysParams_[index][2]),
      cv::Scalar(
        mBuoysParams_[index][3],
        mBuoysParams_[index][4],
        mBuoysParams_[index][5]),
      mask);
    cv::bitwise_and(process, process, segment, mask);
    cv::medianBlur(mask, mask, mBlobMedianBlurSize_);
    cv::dilate(mask, mask, kernel);
    /*cv::imshow("segmented", mask);*/
    std::vector<cv::KeyPoint> keypoints;
    mSbd_->detect(mask, keypoints);
    auto biggestBlob = std::ranges::max_element(
      keypoints,
      [](const cv::KeyPoint k1, const cv::KeyPoint k2) {
        return k1.size < k2.size;
      });
    if (biggestBlob == keypoints.end()) {
      continue;
    }
    if (mShowResult_) {
      cv::drawKeypoints(
        out,
        keypoints,
        out,
        cv::Scalar(0, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      cv::circle(out, biggestBlob->pt, 3, cv::Scalar(200, 200, 200), 10);
      cv::putText(
        out, mBuoysNames_[index], biggestBlob->pt,
        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 5);
    }
    bp.position.x = biggestBlob->pt.x;
    bp.position.y = biggestBlob->pt.y;
    switch (index) {
      case 0:
        bp.color = image_pipeline_msgs::msg::BuoyPosition::RED;
        break;
      case 1:
        bp.color = image_pipeline_msgs::msg::BuoyPosition::WHITE;
        break;
      case 2:
        bp.color = image_pipeline_msgs::msg::BuoyPosition::BLACK;
        break;
      case 3:
        bp.color = image_pipeline_msgs::msg::BuoyPosition::ORANGE;
        break;
      case 4:
        bp.color = image_pipeline_msgs::msg::BuoyPosition::YELLOW;
        break;
    }
    ret.buoys.push_back(bp);
  }
  ret.header.stamp = now();
  mBuoysPub_->publish(ret);
  if (mShowResult_) {
    cv::imshow("buoy detection result", out);
    cv::waitKey(100);
  }
}
}  // namespace underwaterEnhancer

RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::BuoyDetector)
