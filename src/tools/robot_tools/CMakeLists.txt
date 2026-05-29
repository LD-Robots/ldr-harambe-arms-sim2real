cmake_minimum_required(VERSION 3.8)
project(robot_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module — source layout is src/robot_tools/, so PACKAGE_DIR is needed
# (the macro defaults to <project>/<package>/).
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

# Python executables. ament_cmake_python doesn't auto-install entry-point
# shims, so each script needs an explicit install(PROGRAMS) block (same
# pattern as arm_gui_tools).
install(PROGRAMS
  src/${PROJECT_NAME}/pvt_tuner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pvt_tuner
)

install(PROGRAMS
  src/${PROJECT_NAME}/pvt_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pvt_dashboard
)

install(PROGRAMS
  src/${PROJECT_NAME}/bus_voltage_viewer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bus_voltage_viewer
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
