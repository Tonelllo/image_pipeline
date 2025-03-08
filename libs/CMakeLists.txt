project(underwater_images_enhancement)

set(simpleEnhancer_FILES 
simpleEnhancer/simpleEnhancer.cpp
simpleEnhancer/simpleEnhancer.hpp)

add_library(simpleEnhancer ${simpleEnhancer_FILES})

target_include_directories(
simpleEnhancer
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libs>)

find_package(OpenCV REQUIRED)
target_link_libraries(
  simpleEnhancer
  ${OpenCV_LIBRARIES}
)
