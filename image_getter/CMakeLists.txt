cmake_minimum_required(VERSION 3.20)

project(image_pipeline_image_getter)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()


add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_pipeline_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(image_transport REQUIRED)

add_library(image_getter_component SHARED
  SHARED
src/image_getter.cpp)

target_link_libraries(image_getter_component PUBLIC
  rclcpp_components::component
  rclcpp::rclcpp
  realtime_tools::realtime_tools
  ${OpenCV_LIBRARIES}
  ${cv_bridge_TARGETS}
  ${sensor_msgs_TARGETS}
  ${image_pipeline_msgs_TARGETS}
  ${image_transport_TARGETS}
)

target_include_directories(image_getter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(export_image_getter_component)
install(
  TARGETS image_getter_component
  EXPORT export_image_getter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  OpenCV
  cv_bridge
  sensor_msgs
  image_pipeline_msgs
  std_srvs
  rclcpp_components
  realtime_tools
  image_transport
)

rclcpp_components_register_node(
  image_getter_component
  PLUGIN "image_pipeline::ImageGetter"
  EXECUTABLE image_getter
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
