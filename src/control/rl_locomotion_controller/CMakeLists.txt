cmake_minimum_required(VERSION 3.8)
project(rl_locomotion_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ONNX Runtime — set ONNXRUNTIME_ROOT env var or CMake cache var before build.
# Common install locations:
#   Linux x86_64: /usr/local, /opt/onnxruntime, ~/onnxruntime
#   Jetson:       /opt/onnxruntime (after extracting the aarch64 tarball)
# Example: export ONNXRUNTIME_ROOT=/opt/onnxruntime
if(NOT DEFINED ONNXRUNTIME_ROOT)
  if(DEFINED ENV{ONNXRUNTIME_ROOT})
    set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
  else()
    # Probe a few common locations
    foreach(candidate "/opt/onnxruntime" "/usr/local" "$ENV{HOME}/onnxruntime")
      if(EXISTS "${candidate}/include/onnxruntime_cxx_api.h")
        set(ONNXRUNTIME_ROOT "${candidate}")
        break()
      endif()
    endforeach()
  endif()
endif()

if(NOT ONNXRUNTIME_ROOT OR NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR
    "onnxruntime_cxx_api.h not found. Install ONNX Runtime and set "
    "ONNXRUNTIME_ROOT env var, e.g. 'export ONNXRUNTIME_ROOT=/opt/onnxruntime', "
    "then re-run colcon build.")
endif()
message(STATUS "ONNXRUNTIME_ROOT: ${ONNXRUNTIME_ROOT}")

add_library(${PROJECT_NAME} SHARED
  src/rl_locomotion_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_ROOT}/include
)

target_link_libraries(${PROJECT_NAME}
  ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
  realtime_tools
  ament_index_cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY policies/
  DESTINATION share/${PROJECT_NAME}/policies
)

install(FILES rl_locomotion_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface rl_locomotion_controller.xml)

ament_package()
