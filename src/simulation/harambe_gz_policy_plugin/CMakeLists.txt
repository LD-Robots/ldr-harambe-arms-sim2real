cmake_minimum_required(VERSION 3.16)
project(harambe_gz_policy_plugin)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# ONNX Runtime
set(ONNXRUNTIME_DIR "/home/alex/Documents/GitHub/ldr-harambe-arms-sim2real/third_party/onnxruntime")
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")

# KDL for gravity compensation
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/HarambePolicyPlugin.cc
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${ONNXRUNTIME_DIR}/include
)
target_link_libraries(${PROJECT_NAME}
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
  orocos-kdl
  kdl_parser::kdl_parser
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_package()
