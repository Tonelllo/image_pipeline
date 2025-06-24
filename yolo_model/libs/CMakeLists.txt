project(image_pipeline_yolo_model)

# set(CMAKE_C_FLAGS -pg)
# set(CMAKE_CXX_FLAGS -pg)
# set(CMAKE_EXE_LINKER_FLAGS -pg)
# set(CMAKE_SHARED_LINKER_FLAGS -pg)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 11)
# set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/libs/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")

option(ENABLE_BENCHMARKS "Benchmark the preprocessing, inference, and post processing" OFF)
if (ENABLE_BENCHMARKS)
    add_compile_definitions(ENABLE_BENCHMARKS)
endif()

find_package(CUDA 12.8 REQUIRED)
find_package(OpenCV 4.10.0 REQUIRED)

set(cuda_FILES 
cuda_engine/inference.cpp
cuda_engine/inference.hpp)

# set(tensorrt_FILES 
# tensorrt_engine/src/yolov8.cpp
# tensorrt_engine/src/yolov8.hpp)

add_library(cuda_engine STATIC ${cuda_FILES})
set_target_properties(cuda_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
# add_library(tensorrt_engine ${tensorrt_FILES})

target_include_directories(
cuda_engine
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libs>)

target_link_libraries(
  cuda_engine
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS cuda_engine
  EXPORT export_yolo_model_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install( TARGETS YoloV8_TRT
#   DESTINATION lib/${PROJECT_NAME}
# )

# target_link_libraries(
#   tensorrt_engine
#   ${OpenCV_LIBRARIES}
#   YoloV8_TRT
# )
