cmake_minimum_required(VERSION 3.20)

project(image_pipeline_camera_info_publisher)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(camera_info_manager REQUIRED)

add_library(camera_info_publisher_component SHARED
src/camera_info_publisher.cpp)

target_link_libraries(camera_info_publisher_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  realtime_tools::realtime_tools
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${camera_info_manager_TARGETS}
)

target_include_directories(camera_info_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(export_camera_info_publisher_component)
install(
  TARGETS camera_info_publisher_component
  EXPORT export_camera_info_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params/
)

ament_export_dependencies(
  sensor_msgs
  std_srvs
  rclcpp_components
  realtime_tools
  camera_info_manager
)

rclcpp_components_register_node(
  camera_info_publisher_component
  PLUGIN "image_pipeline::CameraInfoPublisher"
  EXECUTABLE camera_info_publisher
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
