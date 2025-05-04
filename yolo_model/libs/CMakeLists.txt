project(underwater_images_enhancement)

# set(CMAKE_C_FLAGS -pg)
# set(CMAKE_CXX_FLAGS -pg)
# set(CMAKE_EXE_LINKER_FLAGS -pg)
# set(CMAKE_SHARED_LINKER_FLAGS -pg)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 11)

set(cuda_FILES 
cuda_engine/inference.cpp
cuda_engine/inference.hpp)

# set(tensorrt_FILES 
# tensorrt_engine/tensorrt.cpp
# tensorrt_engine/tensorrt.hpp)

add_library(cuda_engine ${cuda_FILES})
# add_library(tensorrt_engine ${tensorrt_FILES})

target_include_directories(
cuda_engine
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libs>)

# target_include_directories(
# tensorrt_engine
# PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libs>)

find_package(CUDA 11 REQUIRED)
find_package(OpenCV 4.10.0 REQUIRED)
target_link_libraries(
  cuda_engine
  ${OpenCV_LIBRARIES}
)

# target_link_libraries(
#   tensorrt_engine
#   ${OpenCV_LIBRARIES}
# )
