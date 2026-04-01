cmake_minimum_required(VERSION 3.14)
project(robot_tools)

find_package(ament_cmake REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Register console script entry points
install(
  PROGRAMS
    scripts/compute_pid_gains
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
