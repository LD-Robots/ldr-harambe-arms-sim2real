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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  scripts/generate_ethercat_pvt.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
