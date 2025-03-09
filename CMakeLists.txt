cmake_minimum_required(VERSION 3.20)

project(underwater_images_enhancement)

set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS -pg)
set(CMAKE_CXX_FLAGS -pg)
set(CMAKE_EXE_LINKER_FLAGS -pg)
set(CMAKE_SHARED_LINKER_FLAGS -pg)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_subdirectory(libs)

add_compile_options(-Wall -Wextra -Wpedantic -O3)


find_package(OpenCV REQUIRED)

add_executable(main src/main.cpp)

target_link_libraries(
  main
  simpleEnhancer
  ${OpenCV_LIBRARIES}
)
