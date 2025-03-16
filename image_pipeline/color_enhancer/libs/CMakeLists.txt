project(underwater_images_enhancement)

set(simpleEnhancer_FILES 
simpleEnhancer/simpleEnhancer.cpp
simpleEnhancer/simpleEnhancer.hpp)

set(udcp_FILES 
udcp/udcp.cpp
udcp/udcp.hpp)

add_library(simpleEnhancer ${simpleEnhancer_FILES})
add_library(udcp ${udcp_FILES})

target_include_directories(
simpleEnhancer
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libs>)

target_include_directories(
udcp
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libs>)

find_package(OpenCV REQUIRED)
target_link_libraries(
  simpleEnhancer
  ${OpenCV_LIBRARIES}
)
target_link_libraries(
  udcp
  ${OpenCV_LIBRARIES}
)
