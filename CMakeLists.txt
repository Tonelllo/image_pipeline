cmake_minimum_required(VERSION 3.20)

project(underwater_images_enhancement)

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
add_subdirectory(libs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(enhancer 
src/enhancer.cpp
src/underwater_enhancer.cpp)

target_link_libraries(
  enhancer
  simpleEnhancer
  udcp
  rclcpp::rclcpp
  ${OpenCV_LIBRARIES}
  ${cv_bridge_TARGETS}
)

target_include_directories(enhancer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_include_directories(
#   enhancer
#   PRIVATE
#   include
# )

install(TARGETS enhancer
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY
    include
    DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(
  enhancer

  OpenCV
  cv_bridge
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
