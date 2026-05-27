cmake_minimum_required(VERSION 3.16)
project(robot_safety LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
)

# --- Shared limiter library (linked by control packages) ---
add_library(${PROJECT_NAME} SHARED
  src/safety_limits.cpp
  src/rate_limiter.cpp
  src/sustained_effort_monitor.cpp
  src/thermal.cpp
  src/breach.cpp
  src/param_loader.cpp
  src/supervisor_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})

# --- Supervisor node executable ---
add_executable(${PROJECT_NAME}_supervisor src/supervisor_main.cpp)
target_link_libraries(${PROJECT_NAME}_supervisor ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_supervisor ${DEPS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_supervisor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter ${PROJECT_NAME})
endif()

ament_package()
