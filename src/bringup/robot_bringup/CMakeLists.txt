cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── Install launch + config (PVT ethercat YAMLs ship as source of truth) ──
#
# config/ethercat_pvt/ was bootstrapped once from arm_real_bringup/config/ethercat/
# via scripts/generate_ethercat_pvt.py. After bootstrapping the YAMLs are the
# source of truth — they carry per-joint PVT tuning (PVT_KP / PVT_KD factors,
# max_torque, offsets, etc.). The generator is kept as a one-shot rebootstrap
# tool but is NOT invoked at build time, so hand-edits are never overwritten.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# ── Generate readonly variant (auto_state_transitions=false, drives stay disabled) ──
# Pure derivative of config/ethercat_pvt/ — two boolean flips, no independent
# tuning. Always regenerated on build so it can't drift from the canonical YAMLs.
# Mirrors arm_real_bringup/CMakeLists.txt readonly target; the PVT layout already
# has command_interface: ~ on the mode_of_operation channel, so no mapping rewrite
# is needed (unlike the legacy CSP variant).
set(READONLY_PVT_DIR ${CMAKE_CURRENT_BINARY_DIR}/ethercat_pvt_readonly)
add_custom_target(generate_ethercat_pvt_readonly ALL
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${READONLY_PVT_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/config/ethercat_pvt ${READONLY_PVT_DIR}
  COMMAND sed -i
    -e "s/^auto_fault_reset: true/auto_fault_reset: false/"
    -e "s/^auto_state_transitions: true/auto_state_transitions: false/"
    ${READONLY_PVT_DIR}/*.yaml
  COMMENT "Generating readonly PVT EtherCAT configs from canonical ethercat_pvt/"
)
install(DIRECTORY ${READONLY_PVT_DIR}/
  DESTINATION share/${PROJECT_NAME}/config/ethercat_pvt_readonly)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  scripts/generate_ethercat_pvt.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
