cmake_minimum_required(VERSION 3.8)
project(harambe_ankle_ethercat_driver_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ethercat_driver REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Pure-math kinematics (Eigen only; no ROS/EtherCAT) ───────────────────────
# Separated so the unit/grid tests build and run without the EtherLab master.
add_library(${PROJECT_NAME}_kinematics SHARED
  src/ankle_kinematics.cpp
  src/cubic_ankle_kinematics.cpp
  src/analytic_ankle_kinematics.cpp
)
target_include_directories(${PROJECT_NAME}_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_kinematics Eigen3::Eigen)

# ── EtherCAT driver plugins ──────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/ankle_linkage_driver.cpp
  src/harambe_ethercat_driver.cpp
  src/harambe_pvt_driver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ethercat_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_kinematics Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES ${PROJECT_NAME}.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Grid validation: cubic (and analytic, for contrast) vs the real 9x9 table.
  ament_add_gtest(test_ankle_grid_calibration test/test_ankle_grid_calibration.cpp)
  target_link_libraries(test_ankle_grid_calibration ${PROJECT_NAME}_kinematics)

  # Kinematics unit tests (neutral, round-trip, Jacobian, reachability).
  ament_add_gtest(test_ankle_kinematics test/test_ankle_kinematics.cpp)
  target_link_libraries(test_ankle_kinematics ${PROJECT_NAME}_kinematics)

  # Table-vs-recording: loads a CSV via ANKLE_RECORDING_CSV; SKIPs if unset.
  ament_add_gtest(test_ankle_recording test/test_ankle_recording.cpp)
  target_link_libraries(test_ankle_recording ${PROJECT_NAME}_kinematics)
endif()

ament_package()
