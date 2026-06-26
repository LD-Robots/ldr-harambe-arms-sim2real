cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Python utility nodes (run with `ros2 run arm_gazebo <name>`)
install(PROGRAMS
  scripts/reset_lever.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reset_lever
)
install(PROGRAMS
  scripts/reset_robot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reset_robot
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
