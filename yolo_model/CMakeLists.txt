cmake_minimum_required(VERSION 3.20)

project(image_pipeline_yolo_model)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(libs)

add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.10.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(yolo_model_component SHARED
src/yolo_model.cpp)

target_link_libraries( yolo_model_component PUBLIC
  rclcpp_components::component
  rclcpp::rclcpp
  cuda_engine
  YoloV8_TRT
  realtime_tools::realtime_tools
  ${OpenCV_LIBRARIES}
  ${cv_bridge_TARGETS}
  ${sensor_msgs_TARGETS}
)

target_include_directories(yolo_model_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})

ament_export_targets(export_yolo_model_component)
install(
  TARGETS yolo_model_component
  EXPORT export_yolo_model_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_target_dependencies(
  yolo_model_component

  OpenCV
  cv_bridge
  sensor_msgs
  rclcpp_components
  realtime_tools
)

rclcpp_components_register_node(
  yolo_model_component
  PLUGIN "image_pipeline::YoloModel"
  EXECUTABLE yolo_model
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
