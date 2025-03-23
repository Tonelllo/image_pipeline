/*
 * Copyright(2025)
 */
#include "buoy_detector/buoy_detector.hpp"
#include <algorithm>
#include <opencv2/core/cvdef.h>
#include <opencv2/features2d.hpp>
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

  mInSub_ = create_subscription<sensor_msgs::msg::Image>
              (mInTopic_, 10,
              std::bind(&BuoyDetector::getFrame, this, std::placeholders::_1));

  mBuoysPub_ = create_publisher<image_pipeline_msgs::msg::BuoyPositionArray>("outbuoys", 10);

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
  params.minArea = CV_PI * 10 * 10;
  params.maxArea = CV_PI * 500 * 500;
  params.filterByCircularity = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;
  mSbd_ = cv::SimpleBlobDetector::create(params);
}

void BuoyDetector::getFrame(sensor_msgs::msg::Image::SharedPtr img){
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
  int dilationSize = 15;
  kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                     cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                     cv:: Point(dilationSize, dilationSize));
  mCurrentFrame_ = mCvPtr_->image;
  out = mCurrentFrame_;
  for (size_t index = 0; index < mBuoysNames_.size(); index++) {
    image_pipeline_msgs::msg::BuoyPosition bp;
    cv::cvtColor(mCurrentFrame_, process, CV_BGR2HSV);
    cv::inRange(process,
                cv::Scalar(mBuoysParams_[index][0],
                           mBuoysParams_[index][1],
                           mBuoysParams_[index][2]),
                cv::Scalar(mBuoysParams_[index][3],
                           mBuoysParams_[index][4],
                           mBuoysParams_[index][5]),
                mask);
    cv::bitwise_and(process, process, segment, mask);
    cv::medianBlur(mask, mask, 15);
    cv::dilate(mask, mask, kernel);
    /*cv::imshow("segmented", mask);*/
    std::vector<cv::KeyPoint> keypoints;
    mSbd_->detect(mask, keypoints);
    auto biggestBlob = std::ranges::max_element(keypoints,
                                                [](const cv::KeyPoint k1, const cv::KeyPoint k2){
      return k1.size < k2.size;
    });
    if (biggestBlob == keypoints.end()) {
      continue;
    }
    /*cv::drawKeypoints(out,*/
    /*                  keypoints,*/
    /*                  out,*/
    /*                  cv::Scalar(0, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);*/
    /*cv::circle(out, biggestBlob->pt, 3, cv::Scalar(200, 200, 200), 10);*/
    /*cv::putText(out, mBuoysNames_[index], biggestBlob->pt,*/
    /*            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 5);*/
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
  mBuoysPub_->publish(ret);
  /*cv::imshow("buoy", out);*/
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
