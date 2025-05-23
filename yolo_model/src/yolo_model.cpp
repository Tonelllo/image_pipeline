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
#include <opencv2/imgproc/types_c.h>
#include <tensorrt_engine/yolov8.h>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <yolo_model/yolo_model.hpp>

namespace ai {
YoloModel::YoloModel()
  : Node("yolo_model", "/image_pipeline"){
  declare_parameter("engine", "UNSET");  // cuda, tensorrt
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("cuda_model_path", "UNSET");
  declare_parameter("tensorrt_model_path", "UNSET");
  declare_parameter("classes", std::vector<std::string>());
  mEngine_ = get_parameter("engine").as_string();
  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mModelPath_ = get_parameter("cuda_model_path").as_string();
  mClasses_ = get_parameter("classes").get_value<std::vector<std::string>>();
  mTrtModelPath_ = get_parameter("tensorrt_model_path").as_string();
  mInSub_ = create_subscription<sensor_msgs::msg::Image>(
    mInTopic_, 10,
    std::bind(&YoloModel::processFrame, this, std::placeholders::_1));
  mOutPub_ = create_publisher<sensor_msgs::msg::Image>(mOutTopic_, 10);
  inf = std::make_unique<Inference>(mModelPath_, cv::Size(640, 640), mClasses_, true);
  mConfig_.classNames = mClasses_;
  mYolo_ = std::make_unique<YoloV8>(mModelPath_, mTrtModelPath_, mConfig_);
}

void YoloModel::processFrame(sensor_msgs::msg::Image::SharedPtr img){
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  cv::Mat frame = mCvPtr_->image;

  if (mEngine_ == "cuda") {
    std::vector<Detection> output = inf->runInference(frame);

    int detections = output.size();
    std::cout << detections << std::endl;

    for (int i = 0; i < detections; ++i)
    {
      Detection detection = output[i];

      cv::Rect box = detection.box;
      cv::Scalar color = detection.color;

      // Detection box
      cv::rectangle(frame, box, color, 2);

      // Detection box text
      std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
      cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
      cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

      cv::rectangle(frame, textBox, color, cv::FILLED);
      cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
    }
    cv::cvtColor(frame, frame, CV_BGR2RGB);
  } else if (mEngine_ == "tensorrt") {
    cv::cvtColor(frame, frame, CV_BGR2RGB);
    auto objects = mYolo_->detectObjects(frame);
    mYolo_->drawObjectLabels(frame, objects);
  }
  mCvPtr_->image = frame;
  mOutPub_->publish(*mCvPtr_->toImageMsg());
}
}  // namespace ai
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ai::YoloModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
