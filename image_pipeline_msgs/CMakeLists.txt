cmake_minimum_required(VERSION 3.20)

project(image_pipeline_msgs)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/BuoyPosition.msg"
"msg/BuoyPositionArray.msg"
"msg/PipeLine.msg"
"msg/PipeDirection.msg"
"msg/Buoy.msg"
"msg/Marker.msg"
"msg/Number.msg"
"msg/Pipe.msg"
"msg/Obstacles.msg"
"msg/BoundingBox2D.msg"
"msg/BoundingBox2DArray.msg"
"msg/Obstacle.msg"
"msg/ObstacleArray.msg"
"msg/ObstDetectionSettings.msg"
"msg/ObstDetectionStats.msg"
DEPENDENCIES
std_msgs
geometry_msgs
geographic_msgs)

ament_export_dependencies(rosidl_default_runtime)

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
