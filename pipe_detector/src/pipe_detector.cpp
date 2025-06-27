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

#include "pipe_detector/pipe_detector.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace image_pipeline
{
PipeDetector::PipeDetector(const rclcpp::NodeOptions & options)
  : Node("pipe_detector", options)
{
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("hue_min", 0);
  declare_parameter("val_min", 0);
  declare_parameter("sat_min", 0);
  declare_parameter("hue_max", 255);
  declare_parameter("sat_max", 255);
  declare_parameter("val_max", 255);
  declare_parameter("show_result", false);
  declare_parameter("heartbeat_rate", 0);
  declare_parameter("heartbeat_topic", "UNSET");

  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mHueMin_ = get_parameter("hue_min").as_int();
  mSatMin_ = get_parameter("sat_min").as_int();
  mValMin_ = get_parameter("val_min").as_int();
  mHueMax_ = get_parameter("hue_max").as_int();
  mSatMax_ = get_parameter("sat_max").as_int();
  mValMax_ = get_parameter("val_max").as_int();
  mShowResult_ = get_parameter("show_result").as_bool();

  mHeartBeatTopic_ = get_parameter("heartbeat_topic").as_string();
  mHeartBeatRate_ = get_parameter("heartbeat_rate").as_int();
  mHeartBeatPubisher_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::msg::Empty>(
      create_publisher<std_msgs::msg::Empty>(
        mHeartBeatTopic_,
        1
        )
      )
    );
  mHeartBeatTimer_ = create_wall_timer(
    std::chrono::milliseconds(mHeartBeatRate_),
    [this](){
    if (mHeartBeatPubisher_->trylock()) {
      mHeartBeatPubisher_->unlockAndPublish();
    }
  });

  mFlipDirection_ = false;

  mInSub_ = create_subscription<sensor_msgs::msg::Image>(
    mInTopic_, rclcpp::SensorDataQoS(),
    std::bind(&PipeDetector::getFrame, this, std::placeholders::_1));
  mOutPub_.reset(
    new realtime_tools::RealtimePublisher<image_pipeline_msgs::msg::PipeDirection>(
      create_publisher<image_pipeline_msgs::msg::PipeDirection>(
        mOutTopic_, 1
      )
    )
  );
}

void PipeDetector::getFrame(sensor_msgs::msg::Image::SharedPtr img)
{
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
  cv::cvtColor(mCurrentFrame_, process, CV_BGR2HSV);
  if (process.empty()) {
    return;
  }

  cv::inRange(
    process,
    cv::Scalar(mHueMin_, mSatMin_, mValMin_),
    cv::Scalar(mHueMax_, mSatMax_, mValMax_),
    mask);
  cv::bitwise_and(mCurrentFrame_, mCurrentFrame_, segment, mask);
  cv::Mat points, covar, mean, maxPoints;
  cv::Point2f center;
  cv::dilate(mask, mask,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19, 19), cv::Point(9, 9)));
  cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::RotatedRect> minEllipse(contours.size());
  for (size_t i = 0; i < contours.size(); i++) {
    if (contours[i].size() > 5) {
      minEllipse[i] = cv::fitEllipse(contours[i]);
    }
  }

  uint64 currentMaxSize = 0;
  cv::Size2f eliAxes;
  cv::RotatedRect maxEl;

  for (size_t i = 0; i < contours.size(); i++) {
    uint64 size = 0;
    cv::Mat blobMask = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::drawContours(blobMask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    cv::findNonZero(mask, points);
    size = points.rows;

    if (size > currentMaxSize) {
      eliAxes = minEllipse[i].size;
      currentMaxSize = size;
      maxPoints = points;
      cv::Moments m = cv::moments(contours[i]);
      center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
    }
  }

  if (!maxPoints.empty()) {
    maxPoints.convertTo(maxPoints, CV_32FC1);
    maxPoints = maxPoints.reshape(1);
    cv::calcCovarMatrix(maxPoints, covar, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covar, eigenvalues, eigenvectors);
    cv::Point2f axisDir = cv::Point2f(eigenvectors.row(0));
    if (mPrevDir_.x * axisDir.x < 0 && mPrevDir_.y * axisDir.y < 0) {
      mFlipDirection_ = !mFlipDirection_;
    }
    mPrevDir_ = axisDir;
    if (mFlipDirection_) {
      axisDir = -axisDir;
    }
    if (mShowResult_) {
      cv::ellipse(segment, maxEl, cv::Scalar(0, 255, 0), 2);
      cv::arrowedLine(segment,
                      cv::Point2f(100, 100),
                      cv::Point2f(100, 100) + axisDir * 30,
                      cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.2);
      cv::circle(segment, center, 5, cv::Scalar(0, 0, 255), -1);

      cv::imshow("pipe detection result", segment);
      cv::waitKey(10);
    }

    image_pipeline_msgs::msg::PipeDirection p;
    p.direction.x = axisDir.x;
    p.direction.y = axisDir.y;
    p.position.x = center.x;
    p.position.y = center.y;
    p.size.x = eliAxes.width;
    p.size.y = eliAxes.height;
    p.header.stamp = now();
    if (mOutPub_->trylock()){
      mOutPub_->msg_ = p;
      mOutPub_->unlockAndPublish();
    }
  }
}
}  // namespace image_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::PipeDetector)
