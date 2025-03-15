/*
 * Copyright 2025
 */
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <simpleEnhancer/simpleEnhancer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <underwater_enhancer/underwater_enhancer.hpp>


int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<underwaterEnhancer::UnderwaterEnhancer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
