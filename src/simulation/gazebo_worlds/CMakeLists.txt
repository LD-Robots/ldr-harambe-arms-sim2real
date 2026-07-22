cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

find_package(ament_cmake REQUIRED)

# worlds/ holds the .sdf files; models/ holds everything they pull in via
# model:// URIs. Consumers put share/gazebo_worlds/models on GZ_SIM_RESOURCE_PATH
# so Gazebo can resolve those URIs.
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
