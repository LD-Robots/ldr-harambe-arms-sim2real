cmake_minimum_required(VERSION 3.8)
project(harambe_policy_legs_controller)

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
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ONNX Runtime — point to the workspace third_party dir (same as the Gazebo
# plugin / harambe_policy_controller). Override on Jetson with
# -DONNXRUNTIME_DIR=/path/to/onnxruntime at configure time.
if(NOT DEFINED ONNXRUNTIME_DIR)
  set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/onnxruntime"
      CACHE PATH "Path to ONNX Runtime install")
endif()
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")

if(NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR
    "ONNX Runtime not found at ${ONNXRUNTIME_DIR}. "
    "Set -DONNXRUNTIME_DIR=/path/to/onnxruntime when running colcon build.")
endif()

add_library(${PROJECT_NAME} SHARED
  src/harambe_policy_legs_controller.cpp
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
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
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

install(FILES harambe_policy_legs_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# obs_csv_logger.py — runnable via `ros2 run harambe_policy_legs_controller
# obs_csv_logger.py`. Writes the same obs CSV format as the sims from ~/debug.
install(PROGRAMS
  scripts/obs_csv_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface harambe_policy_legs_controller.xml)

ament_package()
