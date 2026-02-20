cmake_minimum_required(VERSION 3.10)
project(myactuator_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find_package(myactuator_ethercat REQUIRED)

# Hardware interface plugin library (uncomment when implementing)
# add_library(${PROJECT_NAME} SHARED
#   src/myactuator_system.cpp
# )
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(${PROJECT_NAME}
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   myactuator_ethercat
# )

# Register plugin with pluginlib (uncomment when implementing)
# pluginlib_export_plugin_description_file(hardware_interface myactuator_hardware_plugin.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin XML
install(FILES myactuator_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install library (uncomment when implementing)
# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}Targets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})

ament_package()
