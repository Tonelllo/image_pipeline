cmake_minimum_required(VERSION 3.8)
project(image_pipeline_buoy_color)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(nlohmann_json REQUIRED)
find_package(image_pipeline_msgs REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)


include_directories(
  include()
  ${JSONCPP_INCLUDE_DIRS}
  ${image_pipeline_msgs_INCLUDE_DIRS}
)

add_library(buoy_color_component SHARED 
  src/component.cpp 
  src/multi_histogram_voting.cpp)

target_link_libraries(buoy_color_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${message_filters_TARGETS}
  ${OpenCV_LIBRARIES}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  ${image_pipeline_msgs_TARGETS}
  nlohmann_json::nlohmann_json
)

target_include_directories(buoy_color_component PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(export_buoy_color_component)
install(
  TARGETS buoy_color_component nlohmann_json
  EXPORT export_buoy_color_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  rclcpp
  message_filters
  std_msgs
  OpenCV
  nav_msgs
  sensor_msgs
  cv_bridge
  nlohmann_json
  image_pipeline_msgs
)

rclcpp_components_register_node(
  buoy_color_component
  PLUGIN "image_pipeline::BuoyColor"
  EXECUTABLE buoy_color
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
