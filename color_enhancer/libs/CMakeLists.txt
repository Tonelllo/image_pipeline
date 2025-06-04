project(underwater_images_enhancement)

# set(CMAKE_C_FLAGS -pg)
# set(CMAKE_CXX_FLAGS -pg)
# set(CMAKE_EXE_LINKER_FLAGS -pg)
# set(CMAKE_SHARED_LINKER_FLAGS -pg)

set(simpleEnhancer_FILES 
simpleEnhancer/simpleEnhancer.cpp
simpleEnhancer/simpleEnhancer.hpp)

set(udcp_FILES 
udcp/udcp.cpp
udcp/udcp.hpp)

add_library(simpleEnhancer SHARED ${simpleEnhancer_FILES})
add_library(udcp SHARED ${udcp_FILES})

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

install(
  TARGETS simpleEnhancer udcp
  EXPORT export_yolo_model_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
