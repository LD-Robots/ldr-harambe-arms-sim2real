cmake_minimum_required(VERSION 3.22)
project(dual_arm_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS
      scripts/cycle_poses.py
      scripts/motion_recorder.py
      scripts/motion_player.py
    DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/motions")
  install(DIRECTORY motions/
    DESTINATION share/${PROJECT_NAME}/motions)
endif()
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
