cmake_minimum_required(VERSION 3.20)

project(image_pipeline_color_enhancer)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_C_FLAGS -pg)
# set(CMAKE_CXX_FLAGS -pg)
# set(CMAKE_EXE_LINKER_FLAGS -pg)
# set(CMAKE_SHARED_LINKER_FLAGS -pg)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(libs)

add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(color_enhancer_component
src/color_enhancer.cpp)

target_link_libraries(color_enhancer_component PUBLIC
  simpleEnhancer
  udcp
  rclcpp::rclcpp
  rclcpp_components::component
  realtime_tools::realtime_tools
  ${OpenCV_LIBRARIES}
  ${cv_bridge_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

target_include_directories(color_enhancer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(export_color_enhancer_component)
install(
  TARGETS color_enhancer_component
  EXPORT export_color_enhancer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  OpenCV
  cv_bridge
  sensor_msgs
  std_srvs
  rclcpp_components
  realtime_tools
)

rclcpp_components_register_node(
  color_enhancer_component
  PLUGIN "image_pipeline::ColorEnhancer"
  EXECUTABLE color_enhancer
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
