cmake_minimum_required(VERSION 3.10)
project(arm_ethercat_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Safety monitor node (uncomment when implementing)
# add_executable(safety_monitor_node
#   src/safety_monitor_node.cpp
#   src/watchdog.cpp
#   src/estop_handler.cpp
#   src/joint_limit_monitor.cpp
#   src/fault_handler.cpp
# )
# target_include_directories(safety_monitor_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(safety_monitor_node
#   rclcpp
#   sensor_msgs
#   std_srvs
#   std_msgs
#   diagnostic_msgs
# )
# install(TARGETS safety_monitor_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config and launch
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
