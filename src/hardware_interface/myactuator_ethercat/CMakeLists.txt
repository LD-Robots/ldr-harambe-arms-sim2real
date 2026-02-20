cmake_minimum_required(VERSION 3.10)
project(myactuator_ethercat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# EtherLab IgH EtherCAT master (system library)
find_library(ETHERCAT_LIB ethercat
  PATHS /usr/lib /usr/local/lib /opt/etherlab/lib
)
find_path(ETHERCAT_INCLUDE ecrt.h
  PATHS /usr/include /usr/local/include /opt/etherlab/include
)

if(NOT ETHERCAT_LIB)
  message(FATAL_ERROR "EtherLab libethercat not found. Install EtherLab IgH master.")
endif()
if(NOT ETHERCAT_INCLUDE)
  message(FATAL_ERROR "EtherLab ecrt.h not found. Install EtherLab IgH development headers.")
endif()

message(STATUS "EtherLab library: ${ETHERCAT_LIB}")
message(STATUS "EtherLab include: ${ETHERCAT_INCLUDE}")

# Library
add_library(${PROJECT_NAME} SHARED
  src/ethercat_master.cpp
  src/myactuator_slave.cpp
  src/cia402_state_machine.cpp
  src/unit_conversion.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ETHERCAT_INCLUDE}
)
target_link_libraries(${PROJECT_NAME}
  ${ETHERCAT_LIB}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config and ESI files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY esi/
  DESTINATION share/${PROJECT_NAME}/esi
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export for downstream packages
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
