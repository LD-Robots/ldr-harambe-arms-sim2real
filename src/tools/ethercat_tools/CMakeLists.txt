cmake_minimum_required(VERSION 3.14)
project(ethercat_tools)

find_package(ament_cmake REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install UI files as shared resources
install(
  FILES
    ethercat_tools/demo_ethercat_status/ethercat_status.ui
    ethercat_tools/demo_joint_offset/joint_offset.ui
    ethercat_tools/demo_joint_state/joint_state.ui
  DESTINATION share/${PROJECT_NAME}/ui
)

# Register console script entry points
install(
  PROGRAMS
    scripts/demo_ethercat_status
    scripts/demo_joint_offset
    scripts/demo_joint_state
    scripts/joint_calibration
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
