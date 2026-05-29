cmake_minimum_required(VERSION 3.16)
project(robot_pvt_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(robot_safety REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
  control_msgs
  lifecycle_msgs
  robot_safety
)

add_library(${PROJECT_NAME} SHARED
  src/robot_pvt_controller.cpp
  src/hermite.cpp
  src/body_group.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio Eigen3::Eigen)

pluginlib_export_plugin_description_file(controller_interface
  robot_pvt_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES robot_pvt_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Pure-math: cubic + quintic Hermite samplers + trajectory dispatch.
  ament_add_gmock(test_hermite_sampler test/test_hermite_sampler.cpp)
  target_link_libraries(test_hermite_sampler ${PROJECT_NAME})

  # body_group classifier — runtime scope decision used by the controller.
  ament_add_gmock(test_body_group test/test_body_group.cpp)
  target_link_libraries(test_body_group ${PROJECT_NAME})

  # TODO: test_pvt_controller_lifecycle (FREE/HOLD/e-stop/body_group masking)
  #       — needs ros2_control_test_assets + loaned command/state interfaces.
  #       Defer until the fixture pattern is brought in; the math + classifier
  #       tests above cover the deterministic pieces, and the controller's
  #       behaviour on real hardware is exercised by the Gate 2 smoke test.
endif()

ament_package()
