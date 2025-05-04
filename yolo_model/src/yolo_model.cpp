#include <opencv2/imgproc.hpp>
#include <yolo_model/yolo_model.hpp>

namespace ai {
YoloModel::YoloModel()
  : Node("yolo_model", "/image_pipeline"){
  declare_parameter("engine", "UNSET");  // cuda, tensorrt
  declare_parameter("in_topic", "UNSET");
  declare_parameter("out_topic", "UNSET");
  declare_parameter("cuda_model_path", "UNSET");
  declare_parameter("classes", std::vector<std::string>());
  mEngine_ = get_parameter("engine").as_string();
  mInTopic_ = get_parameter("in_topic").as_string();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mModelPath_ = get_parameter("cuda_model_path").as_string();
  mClasses_ = get_parameter("classes").get_value<std::vector<std::string>>();
  mInSub_ = create_subscription<sensor_msgs::msg::Image>(
    mInTopic_, 10,
    std::bind(&YoloModel::processFrame, this, std::placeholders::_1));
  mOutPub_ = create_publisher<sensor_msgs::msg::Image>(mOutTopic_, 10);
  inf = std::make_unique<Inference>(mModelPath_, cv::Size(640, 640), mClasses_, true);
}

void YoloModel::processFrame(sensor_msgs::msg::Image::SharedPtr img){
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while decoding image: %s", e.what());
    return;
  }
  cv::Mat frame = mCvPtr_->image;
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
  mCvPtr_->image = frame;
  cv::cvtColor(frame, frame, CV_BGR2RGB);
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
