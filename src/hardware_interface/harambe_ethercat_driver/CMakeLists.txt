cmake_minimum_required(VERSION 3.8)
project(harambe_ethercat_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ethercat_driver REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ankle_linkage.cpp
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
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES harambe_ethercat_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface harambe_ethercat_driver.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure-math: eccentric-pushrod ankle linkage transform (motor <-> joint).
  ament_add_gtest(test_ankle_linkage test/test_ankle_linkage.cpp)
  target_link_libraries(test_ankle_linkage ${PROJECT_NAME})
endif()

ament_package()
