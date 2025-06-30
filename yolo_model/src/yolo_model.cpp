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
#include <cv_bridge/cv_bridge.h>
#include <image_pipeline_msgs/msg/detail/bounding_box2_d_array__struct.hpp>
#include <opencv2/imgproc/types_c.h>
#include <rclcpp/qos.hpp>
#include <sys/types.h>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <yolo_model/yolo_model.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "image_pipeline_msgs/msg/bounding_box2_d.hpp"
#include <image_pipeline_msgs/msg/bounding_box2_d_array.hpp>

namespace image_pipeline {
YoloModel::YoloModel(const rclcpp::NodeOptions & options)
  : Node("yolo_model", options){
  declare_parameter("engine", "UNSET");  // cuda, tensorrt
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("out_detection", "UNSET");
  declare_parameter("cuda_model_path", "UNSET");
  declare_parameter("tensorrt_model_path", "UNSET");
  declare_parameter("classes", std::vector<std::string>());
  declare_parameter("heartbeat_rate", 0);
  declare_parameter("heartbeat_topic", "UNSET");
  mEngine_ = get_parameter("engine").as_string();
  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mOutDetectionTopic_ = get_parameter("out_detection").as_string();
  mModelPath_ = get_parameter("cuda_model_path").as_string();
  mClasses_ = get_parameter("classes").get_value<std::vector<std::string>>();
  mTrtModelPath_ = get_parameter("tensorrt_model_path").as_string();
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

  std::string gst_str =
    "udpsrc port=5600 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";


  mCam_ = cv::VideoCapture(gst_str, cv::CAP_GSTREAMER);

  if (!mCam_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "UNABLE TO OPEN CAMERA STREAM");
  }

  mOutDetectionPub_.reset(
    new realtime_tools::RealtimePublisher<image_pipeline_msgs::msg::BoundingBox2DArray>(
      create_publisher<image_pipeline_msgs::msg::BoundingBox2DArray>(
        mOutDetectionTopic_, 1
        )
      )
    );

  mTimer_ = create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&YoloModel::processFrame, this));

  inf = std::make_unique<Inference>(mModelPath_, cv::Size(640, 640), mClasses_, true);
}

void YoloModel::processFrame(){
  cv::Mat frame;
  if (!mCam_.read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to grab frame");
    return;
  }

  if (mEngine_ == "cuda") {
    std::vector<Detection> output = inf->runInference(frame);

    int detections = output.size();

    image_pipeline_msgs::msg::BoundingBox2DArray bb2dArr;
    for (int i = 0; i < detections; ++i)
    {
      Detection detection = output[i];

      cv::Rect box = detection.box;
      cv::Scalar color = detection.color;

      // Detection box
      cv::rectangle(frame, box, color, 2);

      // Detection box text
      std::string classString = detection.className +
                                ' ' + std::to_string(detection.confidence).substr(0, 4);
      cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
      cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

      cv::rectangle(frame, textBox, color, cv::FILLED);
      cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10),
                  cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
      image_pipeline_msgs::msg::BoundingBox2D bb2d;
      bb2d.center_x = detection.box.x + static_cast<float>(detection.box.width) / 2;
      bb2d.center_y = detection.box.y + static_cast<float>(detection.box.height) / 2;
      bb2d.size_x = detection.box.width;
      bb2d.size_y = detection.box.height;
      bb2d.id = i;
      bb2d.conf = detection.confidence;
      bb2d.desc = detection.className;
      bb2dArr.boxes.emplace_back(bb2d);
    }
    bb2dArr.header.stamp = now();
    if (mOutDetectionPub_->trylock()) {
      mOutDetectionPub_->msg_ = bb2dArr;
      mOutDetectionPub_->unlockAndPublish();
    }
  }
  mCvPtr_->image = frame;
  if (mOutPub_->trylock()) {
    mOutPub_->msg_ = *mCvPtr_->toImageMsg();
    mOutPub_->unlockAndPublish();
  }
}
}  // namespace image_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::YoloModel)
