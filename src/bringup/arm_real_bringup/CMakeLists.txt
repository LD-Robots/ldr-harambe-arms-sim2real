cmake_minimum_required(VERSION 3.8)
project(arm_real_bringup)

find_package(ament_cmake REQUIRED)

# ── Install canonical EtherCAT configs (single source of truth) ──
install(DIRECTORY config/ethercat/
  DESTINATION share/${PROJECT_NAME}/config/ethercat)

# ── Install controller configs ──
install(FILES
  config/controllers.yaml
  config/controllers_gravcomp.yaml
  config/controllers_viewer.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# ── Generate readonly variant (auto_state_transitions=false, drives stay disabled) ──
set(READONLY_DIR ${CMAKE_CURRENT_BINARY_DIR}/ethercat_readonly)
add_custom_target(generate_ethercat_readonly ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${READONLY_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/config/ethercat ${READONLY_DIR}
  COMMAND sed -i
    -e "s/^auto_fault_reset: true/auto_fault_reset: false/"
    -e "s/^auto_state_transitions: true/auto_state_transitions: false/"
    ${READONLY_DIR}/*.yaml
  COMMAND sed -i
    -e "s/command_interface: mode_of_operation,/command_interface: ~,/"
    ${READONLY_DIR}/*.yaml
  COMMENT "Generating readonly EtherCAT configs from canonical ethercat/"
)
install(DIRECTORY ${READONLY_DIR}/
  DESTINATION share/${PROJECT_NAME}/config/ethercat_readonly)

# ── Generate gravcomp variant (mode_of_operation=10 CST for torque control) ──
set(GRAVCOMP_DIR ${CMAKE_CURRENT_BINARY_DIR}/ethercat_gravcomp)
add_custom_target(generate_ethercat_gravcomp ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${GRAVCOMP_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/config/ethercat ${GRAVCOMP_DIR}
  COMMAND sed -i
    -e "s/^mode_of_operation: 8/mode_of_operation: 10/"
    ${GRAVCOMP_DIR}/*.yaml
  COMMAND sed -i
    -e "/0x6060/s/default: 8/default: 10/"
    ${GRAVCOMP_DIR}/*.yaml
  COMMENT "Generating gravcomp EtherCAT configs from canonical ethercat/"
)
install(DIRECTORY ${GRAVCOMP_DIR}/
  DESTINATION share/${PROJECT_NAME}/config/ethercat_gravcomp)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/homing_sequence.py
  scripts/gravity_comp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
