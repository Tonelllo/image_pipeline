cmake_minimum_required(VERSION 3.20)

project(image_pipeline_buoy_detector)

include(FetchContent)
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


add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_pipeline_msgs REQUIRED)

add_executable(buoy_detector
src/buoy_detector.cpp)

target_link_libraries(
  buoy_detector
  rclcpp::rclcpp
  ${OpenCV_LIBRARIES}
  ${cv_bridge_TARGETS}
  ${sensor_msgs_TARGETS}
  ${image_pipeline_msgs_TARGETS}
)

target_include_directories(buoy_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS buoy_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    include
    DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(
  buoy_detector

  OpenCV
  cv_bridge
  sensor_msgs
  image_pipeline_msgs
  std_srvs
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
