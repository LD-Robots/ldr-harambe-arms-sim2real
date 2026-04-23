cmake_minimum_required(VERSION 3.8)
project(harambe_policy_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# ONNX Runtime — point to the third_party dir (same as Gazebo plugin).
# Override on Jetson by setting -DONNXRUNTIME_DIR=/path/to/onnxruntime at configure time.
if(NOT DEFINED ONNXRUNTIME_DIR)
  # Default for dev machine; Jetson must override
  set(ONNXRUNTIME_DIR "/home/alex/Documents/GitHub/ldr-harambe-arms-sim2real/third_party/onnxruntime"
      CACHE PATH "Path to ONNX Runtime install")
endif()
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")

if(NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR
    "ONNX Runtime not found at ${ONNXRUNTIME_DIR}. "
    "Set -DONNXRUNTIME_DIR=/path/to/onnxruntime when running colcon build. "
    "On Jetson Ubuntu 22.04, install via `apt install libonnxruntime-dev` or download "
    "the aarch64 release from https://github.com/microsoft/onnxruntime/releases")
endif()

add_library(${PROJECT_NAME} SHARED
  src/harambe_policy_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  orocos_kdl
  kdl_parser
)

target_link_libraries(${PROJECT_NAME}
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES harambe_policy_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

pluginlib_export_plugin_description_file(controller_interface harambe_policy_controller.xml)

ament_package()
