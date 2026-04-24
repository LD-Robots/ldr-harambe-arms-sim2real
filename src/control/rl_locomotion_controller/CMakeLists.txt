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

# ONNX Runtime — expected under $ONNXRUNTIME_ROOT or default path
if(NOT DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "/home/andrei/Git/pendulum_test/third_party/onnxruntime")
else()
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR
    "onnxruntime_cxx_api.h not found under ${ONNXRUNTIME_ROOT}/include. "
    "Set ONNXRUNTIME_ROOT env var to point at your ONNX Runtime install.")
endif()

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
